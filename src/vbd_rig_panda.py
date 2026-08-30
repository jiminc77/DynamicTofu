"""Pure-VBD grasp rig (V-track, second consult reports/consult-vbd2.md).

Architecture precedent: examples/vbd/example_vbd_gripper_soft_grid.py.
- PURE SolverVBD (NO SolverCoupledProxy / SolverMuJoCo) -> no uncapped proxy PD.
- enable_rigid_soft_full_surface_contact=True (water-tight soft-surface pass;
  per-particle contact alone slips).
- friction_epsilon exposed (velocity-regularized Coulomb; default 1e-2 is too
  large -> creep; use 2e-4).
- gantry gripper: world -> X-prismatic carriage -> Z-prismatic lift -> palm ->
  2 finger prismatics with FORCE control via Control.joint_f (target_ke=0),
  grip force = effort directly (no deep position target).
- mu/ke MIX with pad shape (avg_mu geometric, ke arithmetic) -> set BOTH soft
  and pad-shape values.

State machine: force ramp -> preload -> lift (50 mm / 2 s) -> hold (5 s).
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
import numpy as np
import warp as wp

import newton
from src.vbd_rig2 import BLOCK_EDGE_M, FINGER_HALF, FPS, GRAB_Z, Vbd2Config, lame_from_E_nu

BLOCK_BOTTOM_Z = 0.002


class PandaRig:
    def __init__(self, cfg: Vbd2Config):
        self.cfg = cfg
        self.fps = FPS
        self.frame_dt = 1.0 / FPS
        self.sim_substeps = cfg.substeps
        self.sim_dt = self.frame_dt / cfg.substeps
        self.sim_time = 0.0
        k_mu, k_lambda = lame_from_E_nu(cfg.E_pa, cfg.nu)

        builder = newton.ModelBuilder(gravity=(0.0, 0.0, -9.81))
        builder.default_particle_radius = cfg.particle_radius

        # --- soft block (volume) ---
        dim = max(1, int(round(BLOCK_EDGE_M / cfg.cell_m)))
        cell = BLOCK_EDGE_M / dim
        self.soft_start = builder.particle_count
        builder.add_soft_grid(
            pos=wp.vec3(-0.5 * BLOCK_EDGE_M, -0.5 * BLOCK_EDGE_M, BLOCK_BOTTOM_Z),
            rot=wp.quat_identity(), vel=wp.vec3(0.0, 0.0, 0.0),
            dim_x=dim, dim_y=dim, dim_z=dim, cell_x=cell, cell_y=cell, cell_z=cell,
            density=cfg.density, k_mu=k_mu, k_lambda=k_lambda, k_damp=cfg.k_damp,
            particle_radius=cfg.particle_radius, label="block",
        )
        self.soft_end = builder.particle_count

        self._build_gripper(builder)
        builder.add_ground_plane()
        builder.color()
        self.model = builder.finalize()

        # FIX add_soft_grid mass over-lumping (+42%): total soft mass MUST equal
        # density * volume. Rescale soft particle_mass/inv_mass to the intended
        # total so the block is the correct 64 g (4 cm / density 1000), not 91 g.
        if getattr(cfg, "correct_mass", True):
            tp = self.model.tet_poses.numpy()
            block_vol = float((1.0 / (6.0 * np.abs(np.linalg.det(tp)) + 1e-30)).sum())
            pm = self.model.particle_mass.numpy()
            cur = float(pm[self.soft_start:self.soft_end].sum())
            intended = cfg.density * block_vol
            if cur > 0:
                scale = intended / cur
                pm[self.soft_start:self.soft_end] *= scale
                self.model.particle_mass.assign(pm)
                inv = np.where(pm > 0, 1.0 / np.where(pm > 0, pm, 1.0), 0.0).astype(np.float32)
                self.model.particle_inv_mass.assign(inv)

        self.model.soft_contact_ke = cfg.contact_ke
        self.model.soft_contact_kd = cfg.contact_kd
        self.model.soft_contact_mu = cfg.mu_pair

        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.model)

        self.solver = newton.solvers.SolverVBD(
            self.model, iterations=cfg.vbd_iterations, rigid_compliant_alm=True,
            integrate_with_external_rigid_solver=False, friction_epsilon=cfg.friction_epsilon,
            rigid_body_contact_buffer_size=1024,
            rigid_body_particle_contact_buffer_size=8192,
        )
        self.collision_pipeline = newton.CollisionPipeline(
            self.model, broad_phase="nxn", soft_contact_margin=cfg.soft_contact_margin,
            enable_rigid_soft_full_surface_contact=cfg.full_surface_contact,
        )
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()
        self.contacts = self.collision_pipeline.contacts()
        self._substep_hooks = []
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state_0)
        wp.copy(self.state_1.body_q, self.state_0.body_q)

        # seeded initial jitter (reproducible per-seed variation for 3-seed confirmation)
        if getattr(cfg, "seed", 0):
            rng = np.random.default_rng(cfg.seed)
            pq = self.state_0.particle_q.numpy()
            pq[self.soft_start:self.soft_end] += rng.normal(0.0, 2.0e-4, (self.soft_end - self.soft_start, 3)).astype(np.float32)
            self.state_0.particle_q.assign(pq)

        qs = self.model.joint_q_start.numpy()
        tqs = self.model.joint_target_q_start.numpy()
        qds = self.model.joint_qd_start.numpy()
        self.x_qi = int(qs[self.j_x]); self.x_ti = int(tqs[self.j_x])
        self.x_dof = int(qds[self.j_x])
        self.z_qi = int(qs[self.j_z]); self.z_ti = int(tqs[self.j_z])
        self.l_dof = int(qds[self.j_left])
        self.r_dof = int(qds[self.j_right])
        self.l_qi = int(qs[self.j_left]); self.r_qi = int(qs[self.j_right])
        labels = list(self.model.body_label)
        body_by_label = {label: i for i, label in enumerate(labels)}
        self.b_carriage = body_by_label["carriage"]
        self.b_palm = body_by_label["palm"]  # fr3_hand collapses into palm (fixed joint)
        self.b_left = body_by_label["fr3/fr3_leftfinger"]
        self.b_right = body_by_label["fr3/fr3_rightfinger"]
        shape_body = self.model.shape_body.numpy()
        left_shapes = np.flatnonzero(shape_body == self.b_left)
        right_shapes = np.flatnonzero(shape_body == self.b_right)
        if len(left_shapes) != 1 or len(right_shapes) != 1:
            raise RuntimeError("expected exactly one replacement pad shape per Panda finger")
        self.s_left = int(left_shapes[0])
        self.s_right = int(right_shapes[0])
        mass = self.model.body_mass.numpy()
        inv_mass = self.model.body_inv_mass.numpy()
        inertia = self.model.body_inertia.numpy()[self.b_carriage]
        inv_inertia = self.model.body_inv_inertia.numpy()[self.b_carriage]
        assert np.isclose(mass[self.b_carriage], 0.050)
        assert inv_mass[self.b_carriage] > 0.0
        assert np.all(np.linalg.eigvalsh(inertia) > 0.0)
        assert np.all(np.isfinite(inv_inertia))
        self.initial_com = self._com()
        self.grab_z = GRAB_Z
        # per-tet rest data for Green-strain instrumentation
        self.tet_idx = self.model.tet_indices.numpy()
        self.tet_poses = self.model.tet_poses.numpy()   # Dm^-1 per tet
        self.tet_rest_vol = 1.0 / (6.0 * np.abs(np.linalg.det(self.tet_poses)) + 1e-30)
        self._weight_n = 9.81 * float(self.model.particle_mass.numpy()[self.soft_start:self.soft_end].sum())
        # D3-C acceleration feed-forward masses (external ruling 2026-08-28). Convention: transported
        # rigid gripper (carriage+palm+fingers). Fallback convention adds the grasped block. a_cmd=0
        # in all quasi-static phases => zero feed-forward there, so the G0'-prime equivalence receipt
        # remains valid without re-running.
        self.m_gripper_rigid = float(mass[self.b_carriage] + mass[self.b_palm]
                                     + mass[self.b_left] + mass[self.b_right])
        self.m_block = float(self.model.particle_mass.numpy()[self.soft_start:self.soft_end].sum())
        # D3-C DISABLED (external ruling 2026-08-28): feed-forward effort is clamped by the j_x
        # position PD and cannot improve accel tracking at any mass -- see reports/vbd/g_trk_gate.md.
        # Realized-acceleration axis (option A) adopted instead. FF code path retained but inert (0 kg).
        self.transport_ff_mass = 0.0

    def strain_stats(self, threshold):
        """Per-tet max principal Green-Lagrange strain, volume-weighted P99, and
        damaged-volume fraction above `threshold`."""
        pq = self.state_0.particle_q.numpy()
        ti = self.tet_idx
        x0 = pq[ti[:, 0]]
        Ds = np.stack([pq[ti[:, 1]] - x0, pq[ti[:, 2]] - x0, pq[ti[:, 3]] - x0], axis=-1)
        F = Ds @ self.tet_poses
        E = 0.5 * (np.transpose(F, (0, 2, 1)) @ F - np.eye(3))
        maxp = np.linalg.eigvalsh(E)[:, -1]   # largest principal strain per tet
        v = self.tet_rest_vol
        order = np.argsort(maxp)
        cw = np.cumsum(v[order]) / v.sum()
        p99 = float(maxp[order][min(len(maxp) - 1, int(np.searchsorted(cw, 0.99)))])
        dmg = float(v[maxp > threshold].sum() / v.sum())
        return {"max_principal_strain": float(maxp.max()), "p99_vol_weighted_strain": p99,
                "damaged_vol_frac": dmg, "threshold": threshold}

    def strain_field(self):
        """Per-tet max principal Green-Lagrange strain (raw field) + rest volumes,
        for storage / post-hoc damage labeling."""
        pq = self.state_0.particle_q.numpy()
        ti = self.tet_idx
        x0 = pq[ti[:, 0]]
        Ds = np.stack([pq[ti[:, 1]] - x0, pq[ti[:, 2]] - x0, pq[ti[:, 3]] - x0], axis=-1)
        F = Ds @ self.tet_poses
        E = 0.5 * (np.transpose(F, (0, 2, 1)) @ F - np.eye(3))
        return np.linalg.eigvalsh(E)[:, -1].astype(np.float32), self.tet_rest_vol.astype(np.float32)

    def contact_count(self):
        try:
            rc = self.contacts.soft_contact_count.numpy()
            return int(rc.reshape(-1)[0])
        except Exception:
            return -1

    def rigid_contact_count_legacy(self):
        try:
            rc = self.contacts.rigid_contact_count.numpy()
            return int(rc.reshape(-1)[0])
        except Exception:
            return -1

    def _build_gripper(self, builder):
        cfg = self.cfg
        pad = newton.ModelBuilder.ShapeConfig(density=1000.0, ke=cfg.contact_ke, kd=cfg.contact_kd,
                                              kf=1.0e3, mu=cfg.mu_pair)
        if cfg.full_surface_contact:
            pad.configure_sdf(force_sdf=True)   # box fingers need an SDF for the full-surface pass
        open_gap = 0.06
        gz = GRAB_Z
        self.open_gap = open_gap
        carriage = builder.add_link(
            mass=0.050,
            inertia=wp.mat33(1.0e-4, 0.0, 0.0, 0.0, 1.0e-4, 0.0, 0.0, 0.0, 1.0e-4),
            label="carriage",
        )
        palm = builder.add_link(xform=wp.transform(wp.vec3(0.0, 0.0, gz), wp.quat_identity()), mass=0.05, label="palm")
        # world -> X carriage -> Z prismatic (position PD, lift)
        self.j_x = builder.add_joint_prismatic(
            parent=-1, child=carriage, axis=wp.vec3(1.0, 0.0, 0.0),
            target_ke=1.0e4, target_kd=2.0e2, target_pos=0.0,
            limit_lower=-2.0, limit_upper=2.0, label="gantry_x",
        )
        self.j_z = builder.add_joint_prismatic(
            parent=carriage, child=palm, axis=wp.vec3(0.0, 0.0, 1.0),
            target_ke=1.0e4, target_kd=2.0e2, target_pos=gz, limit_lower=-0.05, limit_upper=0.30,
            label="gantry_z")
        builder.add_articulation([self.j_x, self.j_z], label="gantry")
        builder.joint_q[builder.joint_q_start[self.j_z]] = gz

        # Import the three genuine Panda hand links and their authored finger joints.
        # All authored collisions are removed: the boxes below are the only finger
        # collision surfaces.  SolverVBD cannot enforce the authored URDF mimic
        # constraint, so symmetry is projected after every substep (see step()).
        asset = newton.utils.download_asset("franka_emika_panda")
        source = asset / "urdf" / "fr3_franka_hand.urdf"
        root = ET.parse(source).getroot()
        keep_links = {"fr3_hand", "fr3_leftfinger", "fr3_rightfinger"}
        keep_joints = {"fr3_finger_joint1", "fr3_finger_joint2"}
        for node in list(root):
            if ((node.tag == "link" and node.get("name") not in keep_links)
                    or (node.tag == "joint" and node.get("name") not in keep_joints)):
                root.remove(node)
        # Physics-only rig: strip ALL authored collisions AND visuals so each finger
        # body carries exactly one shape (our replacement pad box below); the frozen
        # _pad_shapes requires exactly one shape per pad. The v5 renderer draws the
        # real Panda hand meshes separately at the recorded body poses.
        for link in root.findall("link"):
            for collision in list(link.findall("collision")):
                link.remove(collision)
            for visual in list(link.findall("visual")):
                link.remove(visual)
        builder.add_urdf(
            ET.tostring(root, encoding="unicode"), parent_body=palm, floating=False,
            collapse_fixed_joints=True, enable_self_collisions=False,
        )
        joints = {label: i for i, label in enumerate(builder.joint_label)}
        bodies = {label: i for i, label in enumerate(builder.body_label)}
        self.j_left = joints["fr3/fr3_finger_joint1"]
        self.j_right = joints["fr3/fr3_finger_joint2"]
        left, right = bodies["fr3/fr3_leftfinger"], bodies["fr3/fr3_rightfinger"]
        self._panda_hand_label = "palm"  # fr3_hand collapsed into palm
        # Finger origins are 58.4 mm above the hand frame. Offset the replacement
        # pads back to the hand origin, matching the frozen pad centre at GRAB_Z.
        pad_xform = wp.transform(wp.vec3(0.0, 0.0, -0.0584), wp.quat_identity())
        builder.add_shape_box(left, xform=pad_xform, hx=FINGER_HALF, hy=0.006,
                              hz=FINGER_HALF, cfg=pad, color=wp.vec3(0.85, 0.4, 0.3))
        builder.add_shape_box(right, xform=pad_xform, hx=FINGER_HALF, hy=0.006,
                              hz=FINGER_HALF, cfg=pad, color=wp.vec3(0.3, 0.4, 0.85))
        # Start at the frozen 60 mm centre gap (q=30 mm on each Panda finger).
        builder.joint_q[builder.joint_q_start[self.j_left]] = 0.03
        builder.joint_q[builder.joint_q_start[self.j_right]] = 0.03

    def _com(self):
        return self.state_0.particle_q.numpy()[self.soft_start:self.soft_end].mean(axis=0)

    def _bounds(self):
        pq = self.state_0.particle_q.numpy()[self.soft_start:self.soft_end]
        return pq.min(axis=0), pq.max(axis=0)

    def _palm_z(self):
        return float(self.state_0.body_q.numpy()[self.b_palm][2])

    def _shape_center(self, body_q, shape_index):
        """Return a body-attached shape's world center (xyzw quaternion)."""
        body = int(self.model.shape_body.numpy()[shape_index])
        local = self.model.shape_transform.numpy()[shape_index, :3]
        quat = body_q[body, 3:7]
        axis = quat[:3]
        rotated = (local + 2.0 * quat[3] * np.cross(axis, local)
                   + 2.0 * np.cross(axis, np.cross(axis, local)))
        return body_q[body, :3] + rotated

    def set_control(self, close_force, lift_target, x_target=None, x_vel=0.0, x_accel=0.0):
        jf = self.control.joint_f.numpy(); jf[:] = 0.0
        # closing: left axis (0,-1,0) closes toward -y -> +force along axis; right (0,+1,0) toward +y.
        # both fingers move inward with POSITIVE joint_f along their inward axes.
        # sign check (first-frame, verified): +joint_f along the inward axes CLOSES
        # the fingers onto the block; -joint_f opens them.
        jf[self.l_dof] = -close_force
        jf[self.r_dof] = -close_force
        # D3-C: acceleration feed-forward effort on the transport axis (zero when x_accel==0).
        jf[self.x_dof] = self.transport_ff_mass * x_accel
        self.control.joint_f.assign(jf)
        tq = self.control.joint_target_q.numpy()
        tq[self.x_ti] = 0.0 if x_target is None else x_target
        tq[self.z_ti] = lift_target
        self.control.joint_target_q.assign(tq)
        tqd = self.control.joint_target_qd.numpy()
        tqd[self.x_dof] = x_vel
        self.control.joint_target_qd.assign(tqd)

    def add_substep_hook(self, fn):
        self._substep_hooks.append(fn)

    def step(self, close_force, lift_target, x_target=None, x_vel=0.0, x_accel=0.0):
        self.set_control(close_force, lift_target, x_target=x_target, x_vel=x_vel, x_accel=x_accel)
        for k in range(self.sim_substeps):
            self.state_0.clear_forces()
            self.collision_pipeline.collide(self.state_0, self.contacts)
            self.solver.step(self.state_0, self.state_1, self.control, self.contacts, self.sim_dt)
            self.state_0, self.state_1 = self.state_1, self.state_0
            # Software symmetric coupling in SolverVBD's integrated maximal
            # coordinates. Preserve the VBD-computed finger gap and move only
            # the pair's common-mode y so its midpoint coincides with the hand.
            # Never run FK here: joint_q is stale in pure maximal-coordinate VBD
            # and would reset the entire transported articulation.
            bq = self.state_0.body_q.numpy()
            left_y = bq[self.b_left, 1]
            right_y = bq[self.b_right, 1]
            half_gap = 0.5 * (left_y - right_y)
            palm_y = bq[self.b_palm, 1]
            bq[self.b_left, 1] = palm_y + half_gap
            bq[self.b_right, 1] = palm_y - half_gap
            self.state_0.body_q.assign(bq)
            for fn in self._substep_hooks:
                fn(self, k)
        self.sim_time += self.frame_dt

    def metrics(self):
        com = self._com(); lo, hi = self._bounds()
        pq = self.state_0.particle_q.numpy()[self.soft_start:self.soft_end]
        bq = self.state_0.body_q.numpy()
        bqd = self.state_0.body_qd.numpy()
        FH = 0.006  # pad half-thickness (hy)
        left_y = float(self._shape_center(bq, self.s_left)[1])
        right_y = float(self._shape_center(bq, self.s_right)[1])
        # pad-block penetration: block +y/-y faces vs each pad inner face. >0 = real contact.
        block_ymax = float(pq[:, 1].max()); block_ymin = float(pq[:, 1].min())
        pen_left = block_ymax - (left_y - FH)     # left pad at +y; inner face = left_y - FH
        pen_right = (right_y + FH) - block_ymin   # right pad at -y; inner face = right_y + FH
        # per-pad normal force estimate: penalty contact Fn = contact_ke * penetration (clamped >=0)
        fn_left = self.cfg.contact_ke * max(0.0, pen_left)
        fn_right = self.cfg.contact_ke * max(0.0, pen_right)
        # finger linear-y speed (equilibrium check: ~0 in hold => Fn ~ applied joint_f)
        fvy_linear = float(max(abs(bqd[self.b_left][1]), abs(bqd[self.b_right][1])))
        # Frozen receipt compatibility only: slot 4 is angular-y, not linear-y.
        fwy_legacy = float(max(abs(bqd[self.b_left][4]), abs(bqd[self.b_right][4])))
        return {"t": self.sim_time, "com": com.tolist(), "com_z": float(com[2]),
                "com_rise": float(com[2] - self.initial_com[2]),
                "bbox": [float(hi[i] - lo[i]) for i in range(3)],
                "finite": bool(np.all(np.isfinite(pq))),
                "palm_z": self._palm_z(),
                "palm_x": float(bq[self.b_palm][0]),
                "palm_vx": float(bqd[self.b_palm][0]),
                "palm_pos": bq[self.b_palm][:3].tolist(),
                "left_y": left_y, "right_y": right_y, "gap_m": float(abs(left_y - right_y)),
                "pen_left_mm": pen_left * 1000.0, "pen_right_mm": pen_right * 1000.0,
                "fn_left_n": fn_left, "fn_right_n": fn_right,
                "finger_vy": fvy_linear,
                "finger_vy_linear": fvy_linear, "finger_wy_legacy": fwy_legacy,
                "contacts": self.contact_count()}


def run_vbd2(cfg: Vbd2Config, snap_dir: str | None = None):
    rig = PandaRig(cfg)
    frames = []
    series = []
    t_ramp_end = cfg.ramp_s
    t_preload_end = cfg.ramp_s + cfg.preload_s
    t_lift_end = t_preload_end + cfg.lift_s
    t_end = t_lift_end + cfg.hold_s
    n = int(t_end * FPS)
    # gripper-frame reference (block COM minus palm z) captured at preload end
    ref_rel = None
    si = 0
    for f in range(n):
        t = rig.sim_time
        cf = cfg.grip_force_n * min(1.0, t / cfg.ramp_s) if t < t_ramp_end else cfg.grip_force_n
        if t < t_lift_end:
            lift_target = GRAB_Z + cfg.lift_height_m * max(0.0, (t - t_preload_end)) / cfg.lift_s if t > t_preload_end else GRAB_Z
        else:
            lift_target = GRAB_Z + cfg.lift_height_m
        rig.step(cf, min(lift_target, GRAB_Z + cfg.lift_height_m))
        if f % 6 == 0:
            m = rig.metrics()
            rel = m["com_z"] - m["palm_z"]
            if ref_rel is None and rig.sim_time >= t_preload_end:
                ref_rel = rel
            m["rel_slip_mm"] = (abs(rel - ref_rel) * 1000.0) if ref_rel is not None else 0.0
            m["phase"] = ("ramp" if rig.sim_time < t_ramp_end else "preload" if rig.sim_time < t_preload_end
                          else "lift" if rig.sim_time < t_lift_end else "hold")
            series.append(m)
        if snap_dir and f % 8 == 0:
            os.makedirs(snap_dir, exist_ok=True)
            s0 = rig.state_0
            np.savez_compressed(os.path.join(snap_dir, f"f_{si:04d}.npz"),
                                particle_q=s0.particle_q.numpy()[rig.soft_start:rig.soft_end].astype(np.float32),
                                body_q=s0.body_q.numpy().astype(np.float32),
                                body_labels=np.asarray(rig.model.body_label),
                                t=np.float64(rig.sim_time)); si += 1
    return rig, series, dict(t_preload_end=t_preload_end, t_lift_end=t_lift_end, t_end=t_end)
