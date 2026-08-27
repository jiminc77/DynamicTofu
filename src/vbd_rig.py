"""VBD tofu grasp rig (V-track rebuild, spec: reports/consult-vbd-rebuild.md).

Adapts examples/multiphysics/example_proxy_joint_gripper.py:
- tofu: 4x4x4 cm add_soft_grid (frozen size), density 1000, E=25 kPa nu=0.45
  (k_mu 8.6e3, k_lambda 77.6e3, k_damp 1.0) -- PRIMARY firm anchor.
- floating 3-DOF gripper: world -> Z-prismatic (lift) -> palm -> 2 finger
  prismatics (NO arm yet).
- gravity (0,0,-9.81) + ground plane; tofu rests just above z=0.
- force-limited closure: effort_limit is THE variable (per-finger grip force).
- state machine: approach -> close -> hold -> lift.
- SolverCoupledProxy(SolverMuJoCo gripper + SolverVBD tofu).

DIAGNOSTIC/PROTOTYPE. MPM E1 data stays frozen; this is a separate rig.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np
import warp as wp

import newton
import newton.examples
from newton.solvers import SolverMuJoCo, SolverVBD
from newton.solvers.experimental.coupled import SolverCoupledProxy

# --- frozen geometry ------------------------------------------------------
BLOCK_EDGE_M = 0.04
BLOCK_DIM = 4                     # 4 cells -> 4 cm at cell 0.01
BLOCK_CELL = BLOCK_EDGE_M / BLOCK_DIM
BLOCK_BOTTOM_Z = 0.002           # rest just above the ground
GRASP_Z = BLOCK_BOTTOM_Z + 0.5 * BLOCK_EDGE_M   # finger height = block mid
PALM_X = -0.075
FINGER_OPEN_Y = 0.075            # each finger starts this far off-center in y
FPS = 60


@dataclass
class VbdConfig:
    grip_force_n: float = 0.8        # per-finger effort_limit [N] -- THE variable
    soft_contact_mu: float = 0.5
    E_pa: float = 25e3
    nu: float = 0.45
    k_damp: float = 1.0
    density: float = 1000.0
    vbd_iterations: int = 30
    substeps: int = 12               # 8-16 per spec
    soft_contact_ke: float = 5.0e4
    soft_contact_kd: float = 1.0e-3
    soft_contact_kf: float = 1.0e3   # tangential stiffness (consult placeholder); kf-ladder variable
    particle_radius: float = 0.007   # soft-body surface resolution (tunneling suspect)
    finger_shape_mu: float = 0.7     # ACTUAL Coulomb friction for the VBD ALM rigid-particle
    # contact (avg_mu = sqrt(shape_mu0*shape_mu1)); soft_contact_mu is NOT used by this path.
    target_ke: float = 2.0e4
    target_kd: float = 200.0
    # state-machine phase end times [s]
    t_approach: float = 1.0
    t_close: float = 2.0
    t_hold: float = 2.5
    lift_height_m: float = 0.06
    lift_duration_s: float = 1.0
    hold_after_lift_s: float = 5.0   # HARD MILESTONE: hold >= 5 s


def lame_from_E_nu(E, nu):
    mu = E / (2.0 * (1.0 + nu))
    lam = E * nu / ((1.0 + nu) * (1.0 - 2.0 * nu))
    return mu, lam


@wp.kernel
def _set_targets(joint_target_q: wp.array[float], li: int, ri: int, zi: int,
                 finger_t: float, lift_t: float):
    joint_target_q[li] = finger_t
    joint_target_q[ri] = finger_t
    joint_target_q[zi] = lift_t


class VbdTofuRig:
    def __init__(self, cfg: VbdConfig):
        self.cfg = cfg
        self.fps = FPS
        self.frame_dt = 1.0 / FPS
        self.sim_substeps = cfg.substeps
        self.sim_dt = self.frame_dt / cfg.substeps
        self.sim_time = 0.0

        k_mu, k_lambda = lame_from_E_nu(cfg.E_pa, cfg.nu)
        self.k_mu, self.k_lambda = k_mu, k_lambda

        builder = newton.ModelBuilder(gravity=(0.0, 0.0, -9.81))
        SolverMuJoCo.register_custom_attributes(builder)
        SolverVBD.register_custom_attributes(builder)
        builder.default_particle_radius = cfg.particle_radius

        self.soft_start = builder.particle_count
        builder.add_soft_grid(
            pos=wp.vec3(-0.5 * BLOCK_EDGE_M, -0.5 * BLOCK_EDGE_M, BLOCK_BOTTOM_Z),
            rot=wp.quat_identity(), vel=wp.vec3(0.0, 0.0, 0.0),
            dim_x=BLOCK_DIM, dim_y=BLOCK_DIM, dim_z=BLOCK_DIM,
            cell_x=BLOCK_CELL, cell_y=BLOCK_CELL, cell_z=BLOCK_CELL,
            density=cfg.density, k_mu=k_mu, k_lambda=k_lambda, k_damp=cfg.k_damp,
            particle_radius=cfg.particle_radius, label="tofu",
        )
        self.soft_end = builder.particle_count

        self.gripper_bodies, self.gripper_joints = self._emit_gripper(builder)
        self.fixed_z_joint, self.left_joint, self.right_joint = self.gripper_joints

        builder.add_ground_plane()
        builder.color()
        self.model = builder.finalize()
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.model)

        self.model.soft_contact_ke = cfg.soft_contact_ke
        self.model.soft_contact_kd = cfg.soft_contact_kd
        self.model.soft_contact_kf = cfg.soft_contact_kf
        self.model.soft_contact_mu = cfg.soft_contact_mu

        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.collision_pipeline = newton.CollisionPipeline(self.model)
        self.contacts = self.collision_pipeline.contacts()
        self.control = self.model.control()
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state_0)
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state_1)

        tqs = self.model.joint_target_q_start.numpy()
        self.zi = int(tqs[self.fixed_z_joint])
        self.li = int(tqs[self.left_joint])
        self.ri = int(tqs[self.right_joint])
        qs = self.model.joint_q_start.numpy()
        self.z_qi = int(qs[self.fixed_z_joint])
        self.left_qi = int(qs[self.left_joint])
        self.right_qi = int(qs[self.right_joint])
        self.initial_com = self._tofu_com()

        self.solver = SolverCoupledProxy(
            model=self.model,
            entries=[
                SolverCoupledProxy.Entry(
                    name="mjc",
                    solver=lambda v: SolverMuJoCo(model=v, iterations=20,
                                                 disable_contacts=True, use_mujoco_contacts=False),
                    bodies=self.gripper_bodies, joints=self.gripper_joints,
                ),
                SolverCoupledProxy.Entry(
                    name="vbd",
                    solver=lambda v: SolverVBD(model=v, iterations=cfg.vbd_iterations,
                                               rigid_compliant_alm=True,
                                               particle_enable_self_contact=False,
                                               particle_enable_tile_solve=False,
                                               rigid_body_particle_contact_buffer_size=1024),
                    particles=list(range(self.soft_start, self.soft_end)),
                ),
            ],
            coupling=SolverCoupledProxy.Config(
                proxies=[SolverCoupledProxy.Proxy(
                    source="mjc", destination="vbd",
                    bodies=self.gripper_bodies, joints=self.gripper_joints,
                    mass_scale=1.0, mode="lagged",
                    collision_pipeline=lambda model: newton.examples.create_collision_pipeline(model, broad_phase="explicit"),
                    collide_interval=1,
                )],
                iterations=1,
            ),
        )

    def _emit_gripper(self, builder):
        cfg = newton.ModelBuilder.ShapeConfig(density=800.0, ke=8.0e4, kd=1.0e-4, kf=1.0e3, mu=self.cfg.finger_shape_mu)
        palm_pos = wp.vec3(PALM_X, 0.0, GRASP_Z)
        palm = builder.add_link(xform=wp.transform(p=palm_pos, q=wp.quat_identity()), label="palm")
        left = builder.add_link(xform=wp.transform(p=wp.vec3(0.0, FINGER_OPEN_Y, GRASP_Z), q=wp.quat_identity()), label="left_finger")
        right = builder.add_link(xform=wp.transform(p=wp.vec3(0.0, -FINGER_OPEN_Y, GRASP_Z), q=wp.quat_identity()), label="right_finger")
        # palm bar (behind the tofu in x)
        builder.add_shape_box(palm, hx=0.012, hy=FINGER_OPEN_Y + 0.02, hz=0.02, cfg=cfg, label="palm_bar")
        # finger pads sized to the 4 cm tofu (thin in y = contact normal)
        builder.add_shape_box(left, hx=0.02, hy=0.006, hz=0.02, cfg=cfg, label="left_pad")
        builder.add_shape_box(right, hx=0.02, hy=0.006, hz=0.02, cfg=cfg, label="right_pad")
        # world -> Z-prismatic (lift) -> palm
        z_joint = builder.add_joint_prismatic(
            parent=-1, child=palm, axis=wp.vec3(0.0, 0.0, 1.0),
            parent_xform=wp.transform(p=palm_pos, q=wp.quat_identity()),
            child_xform=wp.transform(p=wp.vec3(0.0, 0.0, 0.0), q=wp.quat_identity()),
            target_pos=0.0, target_vel=0.0, target_ke=3.0e5, target_kd=3.0e3,
            limit_lower=-0.02, limit_upper=0.20, limit_ke=1.0e5, limit_kd=100.0,
            effort_limit=1.0e4, label="lift_z",
        )
        # palm -> left finger (prismatic -y, closes inward)
        left_joint = builder.add_joint_prismatic(
            parent=palm, child=left, axis=wp.vec3(0.0, -1.0, 0.0),
            parent_xform=wp.transform(p=wp.vec3(-PALM_X, FINGER_OPEN_Y, 0.0), q=wp.quat_identity()),
            child_xform=wp.transform(p=wp.vec3(0.0, 0.0, 0.0), q=wp.quat_identity()),
            target_pos=0.0, target_vel=0.0, target_ke=self.cfg.target_ke, target_kd=self.cfg.target_kd,
            limit_lower=0.0, limit_upper=FINGER_OPEN_Y, limit_ke=5.0e4, limit_kd=100.0,
            effort_limit=self.cfg.grip_force_n, label="left_slide",
        )
        right_joint = builder.add_joint_prismatic(
            parent=palm, child=right, axis=wp.vec3(0.0, 1.0, 0.0),
            parent_xform=wp.transform(p=wp.vec3(-PALM_X, -FINGER_OPEN_Y, 0.0), q=wp.quat_identity()),
            child_xform=wp.transform(p=wp.vec3(0.0, 0.0, 0.0), q=wp.quat_identity()),
            target_pos=0.0, target_vel=0.0, target_ke=self.cfg.target_ke, target_kd=self.cfg.target_kd,
            limit_lower=0.0, limit_upper=FINGER_OPEN_Y, limit_ke=5.0e4, limit_kd=100.0,
            effort_limit=self.cfg.grip_force_n, label="right_slide",
        )
        builder.add_articulation([z_joint, left_joint, right_joint], label="floating_gripper")
        return [palm, left, right], [z_joint, left_joint, right_joint]

    def _tofu_com(self):
        pq = self.state_0.particle_q.numpy()[self.soft_start:self.soft_end]
        return pq.mean(axis=0)

    def _tofu_bounds(self):
        pq = self.state_0.particle_q.numpy()[self.soft_start:self.soft_end]
        return pq.min(axis=0), pq.max(axis=0)

    def _phase_targets(self):
        c = self.cfg
        t = self.sim_time
        # finger close target: reach the tofu face (FINGER_OPEN_Y - block_half) + squeeze
        close_target = FINGER_OPEN_Y - 0.5 * BLOCK_EDGE_M + 0.004
        if t < c.t_approach:
            return 0.0, 0.0
        elif t < c.t_close:
            a = (t - c.t_approach) / (c.t_close - c.t_approach)
            return close_target * a, 0.0
        elif t < c.t_hold:
            return close_target, 0.0
        else:
            a = min(1.0, (t - c.t_hold) / c.lift_duration_s)
            return close_target, c.lift_height_m * a

    def _update_targets(self):
        ft, lt = self._phase_targets()
        wp.launch(_set_targets, dim=1,
                  inputs=[self.control.joint_target_q, self.li, self.ri, self.zi, ft, lt],
                  device=self.model.device)

    def simulate(self):
        self.collision_pipeline.collide(self.state_0, self.contacts)
        for _ in range(self.sim_substeps):
            self.state_0.clear_forces()
            self.solver.step(self.state_0, self.state_1, self.control, self.contacts, self.sim_dt)
            self.state_0, self.state_1 = self.state_1, self.state_0

    def step(self):
        self._update_targets()
        self.simulate()
        self.sim_time += self.frame_dt

    def metrics(self):
        com = self._tofu_com()
        lo, hi = self._tofu_bounds()
        pq = self.state_0.particle_q.numpy()[self.soft_start:self.soft_end]
        jq = self.state_0.joint_q.numpy()
        return {
            "t": self.sim_time,
            "com_z": float(com[2]), "com_rise": float(com[2] - self.initial_com[2]),
            "bbox": [float(hi[i] - lo[i]) for i in range(3)],
            "finite": bool(np.all(np.isfinite(pq))),
            "lift_q": float(jq[self.z_qi]), "left_q": float(jq[self.left_qi]), "right_q": float(jq[self.right_qi]),
        }
