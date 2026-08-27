"""Diagnostics-only grasp rig (external consult 2026-08-27, sections c/GateA/GateB).

DIAGNOSTIC ONLY: E1 frozen data is never touched. This harness overrides
material / friction / voxel / controller / proxy-iterations at runtime (module
constant patching, no frozen-code changes) and logs the rich >=100 Hz signals
the consult requires:
  per-finger F_n, F_t, F_t/(mu*F_n); jaw gap; block centroid rel displacement;
  active contact-node count + per-collider-id histogram; von Mises q, pressure
  p, deviatoric yield-active fraction (the deviatoric observable BEYOND Jp,
  consult H7); MPM iteration count/residual if exposed; explicit drop-cause tag.

Controller modes (consult item 5 terminology: closure is EFFORT-controlled):
  'effort' - constant closing joint effort after preload (the E1 mode);
  'lock'   - position-clamp after preload (effort-space stiff PD holding the
             preload jaw gap).
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import numpy as np
import newton

import src.scene as S
from src.coupling import build_coupled_solver, mpm_entry_state, node_reduction_per_body
from src.control import ArmIK, FingerForceCommand, assert_control_contract
from src.health import HealthAccumulator

GRASP_QUAT_WXYZ = (1.0, 0.0, 0.0, 0.0)
PREGRASP_Z = 0.32
GRASP_Z = 0.22
FRAME_DT = 0.005  # 200 Hz top-level tick (>=100 Hz logging)
# Position-lock is an effort-space PD holding the preload jaw gap. The Franka
# finger is ~15 g, so at dt=5 ms stability needs KP < 4*m/dt^2 ~ 2400 N/m; a
# stiff KP=3000 blew the fingers up. Use a conservative critically-damped PD
# with a force clamp (the preload forces are <1 N; a few-N clamp is ample).
LOCK_KP = 1500.0  # just below the dt=5ms/15g finger stability limit (~2400)
LOCK_KD = 10.0    # ~critically damped for the finger inertia
LOCK_FORCE_CLAMP_N = 12.0


@dataclass
class DiagConfig:
    name: str
    sigma_y: float = 6000.0
    E_pa: float = S.BLOCK_E_PA
    nu: float = S.BLOCK_NU
    viscosity: float = S.VISCOSITY_PA_S
    tensile_ratio: float = S.TENSILE_YIELD_RATIO
    yield_pressure_factor: float = S.YIELD_PRESSURE_FACTOR
    mu: float = 1.0
    voxel: float = 0.005
    proxy_iterations: int = 4
    control: str = "effort"          # 'effort' | 'lock'
    pad: str = "stock"               # 'stock' | 'sensor' (sensor_format_pad)
    target_Nf: float = 0.60          # per-finger normal target [N]
    lift_s: float = 1.0
    hold_s: float = 5.0
    transport: bool = False
    seed: int = 0


def _patch(cfg: DiagConfig):
    S.BLOCK_E_PA = cfg.E_pa
    S.BLOCK_NU = cfg.nu
    S.VISCOSITY_PA_S = cfg.viscosity
    S.TENSILE_YIELD_RATIO = cfg.tensile_ratio
    S.YIELD_PRESSURE_FACTOR = cfg.yield_pressure_factor
    S.PAD_FRICTION_MU = cfg.mu
    S.VOXEL_SIZE_M = cfg.voxel


class DiagRig:
    def __init__(self, cfg: DiagConfig):
        self.cfg = cfg
        _patch(cfg)
        self.model, self.meta, _ = S.build_scene(cfg.sigma_y, seed=cfg.seed, include_block=True,
                                                  material_completion=True,
                                                  sensor_pad=(cfg.pad == "sensor"))
        assert_control_contract(self.model, self.meta)
        # also set the block's own mpm:friction to mu (Coulomb oracle needs it)
        self.solver = build_coupled_solver(self.model, self.meta, voxel_size=cfg.voxel,
                                           proxy_iterations=cfg.proxy_iterations)
        self.mpm = self.solver.solver("mpm")
        self.state = self.model.state()
        self.control = self.model.control()
        spec = newton.CollisionPipeline.SpeculativeContactConfig(max_speculative_extension=min(0.005, cfg.voxel))
        self.pipeline = newton.CollisionPipeline(self.model, soft_contact_max=0, rigid_contact_max=512,
                                                 speculative_config=spec)
        self.contacts = self.pipeline.contacts()
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state)
        self.ik = ArmIK(self.model, self.meta, (S.BLOCK_CENTER[0], S.BLOCK_CENTER[1], PREGRASP_Z), GRASP_QUAT_WXYZ)
        self.fingers = FingerForceCommand(self.meta)
        self.health = HealthAccumulator()
        self.t = 0.0
        tq = self.control.joint_target_q.numpy(); tq[:] = self.model.joint_q.numpy()
        self.control.joint_target_q.assign(tq)
        self.fingers.apply_open(self.control)
        self.q_lock = None

    # --- stepping ----------------------------------------------------------
    def step(self, n=1):
        for _ in range(n):
            self.state.clear_forces()
            self.pipeline.collide(self.state, self.contacts, dt=FRAME_DT)
            self.solver.step(self.state, self.state, self.control, self.contacts, FRAME_DT)
            self.t += FRAME_DT

    def jp(self):
        return mpm_entry_state(self.solver).mpm.particle_Jp.numpy()

    def realized_tool(self):
        from src.control import EE_TOOL_OFFSET
        bq = self.state.body_q.numpy()[self.meta.ee_body_index]
        x, y, z, w = bq[3:7]
        u = np.array([x, y, z]); v = np.array(EE_TOOL_OFFSET)
        return bq[:3] + 2 * np.dot(u, v) * u + (w * w - np.dot(u, u)) * v + 2 * w * np.cross(u, v)

    def move_ee(self, pos, duration_s):
        start = self.control.joint_target_q.numpy().copy()
        sol = self.ik.solve_to_targets(pos, GRASP_QUAT_WXYZ)
        arm = np.asarray(self.meta.arm_coord_indices, dtype=int)
        n = max(1, int(duration_s / FRAME_DT))
        for k in range(n):
            a = (k + 1) / n
            tq = self.control.joint_target_q.numpy()
            tq[arm] = (1 - a) * start[arm] + a * sol[arm]
            self.control.joint_target_q.assign(tq)
            self.apply_fingers()
            self.step(1)

    def move_ee_converge(self, pos, tol=0.003, iters=12):
        target = np.asarray(pos, float); bias = np.zeros(3)
        arm = np.asarray(self.meta.arm_coord_indices, dtype=int)
        for _ in range(iters):
            sol = self.ik.solve_to_targets((target + bias).tolist(), GRASP_QUAT_WXYZ)
            tq = self.control.joint_target_q.numpy(); tq[arm] = sol[arm]
            self.control.joint_target_q.assign(tq)
            for _ in range(int(0.25 / FRAME_DT)):
                self.apply_fingers(); self.step(1)
            err = target - self.realized_tool()
            if np.linalg.norm(err) <= tol:
                break
            bias += 0.8 * err

    # --- finger control ----------------------------------------------------
    def apply_fingers(self):
        if self.q_lock is None:
            return  # open/preload handled explicitly by caller
        if self.cfg.control == "lock":
            jf = self.control.joint_f.numpy(); jf[:] = 0.0
            q = self.state.joint_q.numpy(); qd = self.state.joint_qd.numpy()
            for d, c in zip(self.meta.finger_dof_indices, self.meta.finger_coord_indices):
                f = -LOCK_KP * (q[c] - self.q_lock[c]) - LOCK_KD * qd[d]
                jf[d] = float(np.clip(f, -LOCK_FORCE_CLAMP_N, LOCK_FORCE_CLAMP_N))
            self.control.joint_f.assign(jf)
        else:
            self.fingers.apply(self.control, self.cfg.target_Nf)

    def pad_normals_world(self):
        bq = self.state.body_q.numpy()
        left, right = self.meta.finger_body_indices
        axis = bq[left][:3] - bq[right][:3]
        axis /= (np.linalg.norm(axis) + 1e-12)
        return {left: axis, right: -axis}

    # --- rich per-tick logging --------------------------------------------
    def log_tick(self):
        left, right = self.meta.finger_body_indices
        bq = self.state.body_q.numpy()
        reduced = node_reduction_per_body(self.mpm, self.state, bq, self.model.body_com.numpy(), FRAME_DT)
        normals = self.pad_normals_world()
        rec = {"t": self.t}
        # per-finger Fn/Ft + contact-node histogram
        imp, pos, cid = self.mpm.collect_collider_impulses(self.state)
        cidn = cid.numpy().astype(int); mags = np.linalg.norm(imp.numpy(), axis=1)
        body_of = self.mpm.collider_body_index.numpy().astype(int)
        hist = {}
        for k in np.nonzero(mags > 1e-9)[0]:
            c = cidn[k]
            if 0 <= c < len(body_of):
                hist[int(c)] = hist.get(int(c), 0) + 1
        rec["collider_hist"] = hist
        for side, b in (("L", left), ("R", right)):
            F, _T, n = reduced.get(b, (np.zeros(3), np.zeros(3), 0))
            nb = normals[b]
            Fn = float(np.dot(F, nb))
            Ft = float(np.linalg.norm(F - Fn * nb))
            rec[f"Fn_{side}"] = Fn
            rec[f"Ft_{side}"] = Ft
            rec[f"slip_ratio_{side}"] = Ft / (self.cfg.mu * abs(Fn) + 1e-9)
            rec[f"nodes_{side}"] = int(n)
        # jaw gap + finger q
        q = self.state.joint_q.numpy()
        rec["jaw_gap_m"] = float(sum(q[c] for c in self.meta.finger_coord_indices))
        # block centroid
        pq = self.state.particle_q.numpy()
        rec["block_centroid"] = pq.mean(axis=0).tolist()
        # deviatoric observable beyond Jp: von Mises q, pressure p, yield-active frac
        try:
            stress = mpm_entry_state(self.solver).mpm.particle_stress.numpy()  # [N,3,3]
            tr = np.trace(stress, axis1=1, axis2=2)
            p = -tr / 3.0
            dev = stress - (tr / 3.0)[:, None, None] * np.eye(3)[None]
            vm = np.sqrt(1.5 * np.sum(dev * dev, axis=(1, 2)))
            rec["p_mean_pa"] = float(p.mean()); rec["p_max_pa"] = float(p.max())
            rec["q_vm_mean_pa"] = float(vm.mean()); rec["q_vm_max_pa"] = float(vm.max())
            rec["yield_active_frac"] = float(np.mean(vm >= self.cfg.sigma_y * 0.999))
        except Exception as exc:  # noqa: BLE001
            rec["stress_error"] = repr(exc)[:120]
        jp = self.jp()
        rec["jp_damage_frac"] = float(np.mean(np.abs(jp - 1.0) > 0.05))
        return rec


def _snap(rig, snap_dir, idx):
    import numpy as _np
    os.makedirs(snap_dir, exist_ok=True)
    _np.savez_compressed(os.path.join(snap_dir, f"trial_{idx:04d}.npz"),
                         particle_q=rig.state.particle_q.numpy().astype(_np.float32),
                         jp=rig.jp().astype(_np.float32),
                         body_q=rig.state.body_q.numpy().astype(_np.float32),
                         t=_np.float64(rig.t))


def run_diag(cfg: DiagConfig, frames_dir: str | None = None, snap_dir: str | None = None, snap_every: int = 30):
    rig = DiagRig(cfg)
    log = []
    # approach + servo to grasp pose
    rig.step(int(0.5 / FRAME_DT))
    rig.move_ee((S.BLOCK_CENTER[0], S.BLOCK_CENTER[1], PREGRASP_Z), 1.5)
    rig.move_ee((S.BLOCK_CENTER[0], S.BLOCK_CENTER[1], GRASP_Z), 1.5)
    rig.move_ee_converge((S.BLOCK_CENTER[0], S.BLOCK_CENTER[1], GRASP_Z))

    # preload to target_Nf (ramp closing effort), logging
    n_ramp = int(0.5 / FRAME_DT)
    for k in range(n_ramp):
        rig.fingers.apply(rig.control, cfg.target_Nf * (k + 1) / n_ramp)
        rig.step(1)
    # settle the preload briefly
    for _ in range(int(0.3 / FRAME_DT)):
        rig.fingers.apply(rig.control, cfg.target_Nf); rig.step(1)
    rig.q_lock = rig.state.joint_q.numpy().copy()  # capture preload gap for 'lock'
    preload_rec = rig.log_tick()

    # lift (no transport per Gate A/B), then hold
    n_lift = int(cfg.lift_s / FRAME_DT)
    ref_centroid_grip = None
    grasp_established = True
    lost_since = None
    drop_cause = None
    z0 = GRASP_Z

    def grip_frame_disp():
        bq = rig.state.body_q.numpy()[rig.meta.ee_body_index]
        pq = rig.state.particle_q.numpy().mean(axis=0)
        # gripper-frame relative displacement magnitude
        return pq - bq[:3]

    ref = None
    snap_i = [0]
    if snap_dir:
        _snap(rig, snap_dir, snap_i[0]); snap_i[0] += 1
    for k in range(n_lift):
        s = (k + 1) / n_lift; s = s * s * (3 - 2 * s)
        rig.move_ee((S.BLOCK_CENTER[0], S.BLOCK_CENTER[1], z0 + 0.05 * s), FRAME_DT)
        if ref is None:
            ref = grip_frame_disp()
        rec = rig.log_tick(); rec["phase"] = "lift"; log.append(rec)
        if snap_dir and k % 6 == 0:
            _snap(rig, snap_dir, snap_i[0]); snap_i[0] += 1

    n_hold = int(cfg.hold_s / FRAME_DT)
    for k in range(n_hold):
        rig.apply_fingers(); rig.step(1)
        if snap_dir and k % snap_every == 0:
            _snap(rig, snap_dir, snap_i[0]); snap_i[0] += 1
        if k % 2 == 0:
            rec = rig.log_tick(); rec["phase"] = "hold"; log.append(rec)
            # drop detection
            bilateral = rec["nodes_L"] > 0 and rec["nodes_R"] > 0
            disp = float(np.linalg.norm(grip_frame_disp() - ref))
            if not bilateral:
                lost_since = lost_since if lost_since is not None else rec["t"]
                if rec["t"] - lost_since > 0.2 and drop_cause is None:
                    drop_cause = "bilateral_contact_lost>0.2s"
            else:
                lost_since = None
            if disp > 0.02 and drop_cause is None:
                drop_cause = "gripper_frame_disp>2cm"
        rig.health.check_tick(rig.state.particle_q.numpy(), rig.state.particle_qd.numpy(), rig.jp(), mpm_solver=rig.mpm)

    dropped = drop_cause is not None
    final = log[-1]
    result = {
        "cfg": cfg.__dict__,
        "outcome": "drop" if dropped else "hold",
        "drop_cause": drop_cause,
        "preload": {k: preload_rec[k] for k in preload_rec if k not in ("collider_hist", "block_centroid")},
        "final": {k: final[k] for k in final if k not in ("collider_hist", "block_centroid")},
        "health_clean": bool(rig.health.clean),
        "n_log": len(log),
    }
    if frames_dir:
        os.makedirs(frames_dir, exist_ok=True)
        np.savez_compressed(os.path.join(frames_dir, "log.npz"),
                            **{k: np.array([r.get(k, np.nan) for r in log], dtype=object) for k in
                               ("t", "Fn_L", "Fn_R", "Ft_L", "Ft_R", "jaw_gap_m", "yield_active_frac",
                                "q_vm_max_pa", "p_max_pa", "jp_damage_frac", "nodes_L", "nodes_R")})
    return result, log, rig
