"""Single E1 trial engine: phase machine -> judgment inputs -> e1.v1 document.

Phases (pre-registered): settle 0.5 s -> close to commanded force + hold 0.5 s
-> lift 5 cm in 0.3 s -> hold 0.2 s -> transport profile -> settle 0.5 s.
Labels evaluated from lift-complete to settle-end (judgment v1, frozen).

The engine is material-agnostic: material parameters come from src.scene
(protocol constants) and the trial config block records everything used.
"""

from __future__ import annotations

import hashlib
import os
import time

import numpy as np

import src.scene as S
from src import io_schemas, judgment, profiles
from src.control import (
    PHASE_CLOSE_HOLD_S,
    PHASE_FINAL_SETTLE_S,
    PHASE_LIFT_S,
    PHASE_POSTLIFT_HOLD_S,
    PHASE_SETTLE_S,
)
from src.coupling import coupling_params_dict
from scripts.probes.gn2_ar_probe import FRAME_DT, GRASP_QUAT_WXYZ, GRASP_Z, PREGRASP_Z, Rig

ROOT = os.path.join(os.path.dirname(__file__), "..")

# P4 protocol observable (externally approved): measured realized bilateral-sum
# normal-force plateaus from the dynamic ladder (reports/logs/gn2-dynamic-ladder.json).
F_BEARING_CAPACITY_N = {2000: 5.61, 3333: 6.11, 6000: 7.18}


def _realized_bilateral_mean_normal(rig) -> float:
    from src.coupling import node_reduction_per_body

    bq = rig.state.body_q.numpy()
    reduced = node_reduction_per_body(rig.mpm, rig.state, bq, rig.model.body_com.numpy(), FRAME_DT)
    normals = rig.pad_normals_world()
    vals = []
    for b in rig.meta.finger_body_indices:
        F, _T, _n = reduced.get(b, (np.zeros(3), np.zeros(3), 0))
        vals.append(abs(float(np.dot(F, normals[b]))))
    return float(np.mean(vals))
LIFT_M = 0.05
TRANSPORT_Z = GRASP_Z + LIFT_M
IMPULSE_EPS = 1e-8
SAMPLE_EVERY_TICKS = 2  # 100 Hz judgment sampling at the 200 Hz tick


def _sha256_file(path: str) -> str:
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def _git_sha() -> str:
    import subprocess
    try:
        return subprocess.check_output(["git", "-C", ROOT, "rev-parse", "HEAD"], text=True).strip()
    except Exception:
        return "unknown"


def _quat_conj_rotate(q_xyzw, v):
    x, y, z, w = q_xyzw
    u = np.array([-x, -y, -z])
    return 2 * np.dot(u, v) * u + (w * w - np.dot(u, u)) * v + 2 * w * np.cross(u, v)


class TrialRecorder:
    def __init__(self, rig: Rig):
        self.rig = rig
        self.samples = []
        self.grasp_established = False
        self.ref_grip = None       # block centroid in gripper frame at grasp establishment
        self.ref_window = None     # ... at lift-complete (slip reference)
        self.slip_peak = 0.0
        self._bilateral_run = 0

    def _finger_contacts(self):
        imp, _pos, cid = self.rig.mpm.collect_collider_impulses(self.rig.state)
        mags = np.linalg.norm(imp.numpy(), axis=1)
        cidn = cid.numpy().astype(int)
        body_of = self.rig.mpm.collider_body_index.numpy().astype(int)
        counts = {b: 0 for b in self.rig.meta.finger_body_indices}
        for k in np.nonzero(mags > IMPULSE_EPS)[0]:
            c = cidn[k]
            if 0 <= c < len(body_of) and body_of[c] in counts:
                counts[body_of[c]] += 1
        return counts

    def _grip_frame_centroid(self):
        pq = self.rig.state.particle_q.numpy()
        centroid = pq.mean(axis=0)
        bq = self.rig.state.body_q.numpy()[self.rig.meta.ee_body_index]
        return _quat_conj_rotate(bq[3:7], centroid - bq[:3])

    def sample(self):
        counts = self._finger_contacts()
        bilateral = all(v > 0 for v in counts.values())
        if bilateral:
            self._bilateral_run += 1
        else:
            self._bilateral_run = 0
        if not self.grasp_established and self._bilateral_run >= int(0.1 / FRAME_DT / SAMPLE_EVERY_TICKS):
            self.grasp_established = True
            self.ref_grip = self._grip_frame_centroid()
        gf = self._grip_frame_centroid()
        rel = float(np.linalg.norm(gf - self.ref_grip)) if self.ref_grip is not None else 0.0
        slip = float(np.linalg.norm(gf - self.ref_window)) if self.ref_window is not None else 0.0
        self.slip_peak = max(self.slip_peak, slip)
        self.samples.append({
            "t": round(self.rig.t, 6),
            "jp": self.rig.jp(),
            "grasp_established": self.grasp_established,
            "bilateral_contact": bilateral,
            "relative_displacement_m": rel,
            "slip_net_m": slip,
            "slip_peak_m": self.slip_peak,
        })

    def mark_lift_complete(self):
        self.ref_window = self._grip_frame_centroid()
        self.slip_peak = 0.0


def run_trial(
    sigma_y: float,
    a_peak: float,
    f_g: float,
    seed: int,
    *,
    profile_id: str = "trapz_reversal_default",
    calibration: dict,
    out_json: str | None = None,
    extra_config: dict | None = None,
    frames_dir: str | None = None,
    frame_every_ticks: int = 40,
    lift_duration_s: float = PHASE_LIFT_S,  # protocol constant; override ONLY for
    # externally-ordered diagnostic probes (never counted as E1 cells)
):
    t_wall0 = time.time()
    # seeds are the replication unit: +/-1 mm pose jitter (deterministic per seed)
    rig = Rig(include_block=True, sigma_y=sigma_y, seed=seed, material_completion=True, pose_jitter_m=0.001)
    rec = TrialRecorder(rig)
    prof = profiles.generate(profile_id, a_peak, dt=FRAME_DT)

    # --- approach (not part of the judged phases) ---------------------------
    rig.step(int(0.5 / FRAME_DT))
    rig.move_ee((S.BLOCK_CENTER[0], S.BLOCK_CENTER[1], PREGRASP_Z), 1.5)
    rig.move_ee((S.BLOCK_CENTER[0], S.BLOCK_CENTER[1], GRASP_Z), 1.5)
    rig.move_ee_converge((S.BLOCK_CENTER[0], S.BLOCK_CENTER[1], GRASP_Z))

    phase_ts = {}
    t0 = rig.t
    tick = [0]
    snap_i = [0]

    def maybe_frame(force=False):
        if frames_dir and (force or tick[0] % frame_every_ticks == 0):
            from scripts.probes.gn2_lift_jp import snapshot

            snapshot(rig, frames_dir, "trial", snap_i[0])
            snap_i[0] += 1

    def advance(n, on_tick=None):
        for _ in range(n):
            if on_tick:
                on_tick()
            rig.step(1)
            tick[0] += 1
            if tick[0] % SAMPLE_EVERY_TICKS == 0:
                rec.sample()
            maybe_frame()

    # --- settle -------------------------------------------------------------
    phase_ts["settle"] = rig.t - t0
    advance(int(PHASE_SETTLE_S / FRAME_DT))

    # --- close to commanded force + hold ------------------------------------
    phase_ts["close_hold"] = rig.t - t0
    n_close = int(PHASE_CLOSE_HOLD_S / FRAME_DT)
    n_ramp = int(0.3 / FRAME_DT)
    hold_normals = []
    for k in range(n_close):
        rig.fingers.apply(rig.control, f_g * min(1.0, (k + 1) / n_ramp))
        rig.step(1)
        tick[0] += 1
        if tick[0] % SAMPLE_EVERY_TICKS == 0:
            rec.sample()
        if k >= n_ramp and (k % 5 == 0):  # realized force during the steady hold
            hold_normals.append(_realized_bilateral_mean_normal(rig))
        maybe_frame()
    maybe_frame(force=True)  # grasp key frame

    # --- lift 5 cm in 0.3 s (smoothstep) ------------------------------------
    phase_ts["lift"] = rig.t - t0
    n_lift = int(lift_duration_s / FRAME_DT)
    for k in range(n_lift):
        s = (k + 1) / n_lift
        s = s * s * (3 - 2 * s)
        rig.move_ee((S.BLOCK_CENTER[0], S.BLOCK_CENTER[1], GRASP_Z + LIFT_M * s), FRAME_DT)
        tick[0] += 1
        if tick[0] % SAMPLE_EVERY_TICKS == 0:
            rec.sample()
        maybe_frame()
    phase_ts["lift_complete"] = rig.t - t0
    rec.mark_lift_complete()
    maybe_frame(force=True)  # lift-complete key frame

    # --- post-lift hold ------------------------------------------------------
    advance(int(PHASE_POSTLIFT_HOLD_S / FRAME_DT))

    # --- transport (profile-driven EE tracking; realized accel measured) ----
    phase_ts["transport"] = rig.t - t0
    pos = np.atleast_2d(prof["pos"])
    if pos.shape[0] == 1 and pos.shape[1] > 2:
        pos = pos.T
    realized_tool = []
    n_prof = len(pos)
    for k in range(n_prof):
        if pos.shape[1] == 1:  # lateral +/-y axis per the brief's routing
            target = (S.BLOCK_CENTER[0], S.BLOCK_CENTER[1] + float(pos[k, 0]), TRANSPORT_Z)
        else:  # 2D arc in the horizontal plane
            target = (S.BLOCK_CENTER[0] + float(pos[k, 0]), S.BLOCK_CENTER[1] + float(pos[k, 1]), TRANSPORT_Z)
        rig.move_ee(target, FRAME_DT)
        tick[0] += 1
        realized_tool.append(rig.realized_tool())
        if tick[0] % SAMPLE_EVERY_TICKS == 0:
            rec.sample()
        maybe_frame()
    if "reversal_time" in prof.get("phase_timestamps", {}):
        phase_ts["reversal"] = phase_ts["transport"] + prof["phase_timestamps"]["reversal_time"]
    maybe_frame(force=True)  # transport-end key frame

    # --- final settle --------------------------------------------------------
    phase_ts["final_settle"] = rig.t - t0
    advance(int(PHASE_FINAL_SETTLE_S / FRAME_DT))
    phase_ts["settle_end"] = rig.t - t0
    maybe_frame(force=True)  # settle-end key frame

    # --- realized transport acceleration (from realized tool positions) -----
    rt = np.asarray(realized_tool)
    if len(rt) > 8:
        vel = np.gradient(rt, FRAME_DT, axis=0)
        acc = np.gradient(vel, FRAME_DT, axis=0)
        # light smoothing window to suppress finite-difference tick noise
        w = max(3, int(0.02 / FRAME_DT))
        kern = np.ones(w) / w
        acc_s = np.stack([np.convolve(acc[:, i], kern, mode="same") for i in range(3)], axis=1)
        a_realized = float(np.max(np.linalg.norm(acc_s, axis=1)))
    else:
        a_realized = float("nan")

    # --- judgment ------------------------------------------------------------
    # sample timestamps are absolute rig.t; phase_ts are relative to t0
    verdict = judgment.evaluate(
        rec.samples,
        lift_complete=phase_ts["lift_complete"] + t0,
        settle_end=phase_ts["settle_end"] + t0,
        particle_count=rig.model.particle_count,
    )
    labels = verdict["labels"] if verdict["labels"] else ["intact"]

    # Time-frame reconciliation (documented): the judgment reducer works in the
    # ABSOLUTE sim clock (rig.t); phase_timestamps are stored RELATIVE to t0 (the
    # sim time when the judged phases begin, after approach+servo). drop_t and
    # damage_latch_t come back ABSOLUTE. Store both frames + explicit in-window
    # booleans so each JSON is self-verifiable without external state.
    lc_abs = phase_ts["lift_complete"] + t0
    se_abs = phase_ts["settle_end"] + t0
    dt_abs = verdict["drop_t"]
    dm_abs = verdict["damage_latch_t"]
    drop_in_window = dt_abs is not None and lc_abs - 1e-9 <= dt_abs <= se_abs + 1e-9
    damage_in_window = dm_abs is not None and lc_abs - 1e-9 <= dm_abs <= se_abs + 1e-9
    # invariant self-check: the reducer only sets these from in-window samples,
    # so a label present with an out-of-window time is a hard contract violation
    if ("drop" in labels) != drop_in_window and "drop" in labels:
        raise AssertionError(f"drop label with out-of-window drop_t={dt_abs} window=[{lc_abs},{se_abs}]")

    payload = {
        "sigma_y_pa": sigma_y,
        "a_peak_cmd_ms2": a_peak,
        "a_peak_realized_ms2": a_realized,
        "f_g_n": f_g,
        "f_g_realized_n": float(np.mean(hold_normals)) if hold_normals else None,
        "seed": seed,
        "labels": labels,
        "cell_color": verdict["cell_color"],
        "t0_abs_s": t0,
        "judgment_window_abs_s": [lc_abs, se_abs],
        "damage_latch_t": dm_abs,
        "damage_latch_t_rel": (dm_abs - t0) if dm_abs is not None else None,
        "drop_t": dt_abs,
        "drop_t_rel": (dt_abs - t0) if dt_abs is not None else None,
        "drop_evidence_in_window": bool(drop_in_window),
        "damage_evidence_in_window": bool(damage_in_window),
        "damage_after_drop": verdict["damage_after_drop"],
        "peak_damage_fraction": verdict["peak_damage_fraction"],
        "health": rig.health.report(),
        "phase_timestamps": {k: round(v, 4) for k, v in phase_ts.items()},
        "phase_timestamps_frame": "relative_to_t0_abs_s (drop_t/damage_latch_t are absolute; *_rel are t0-subtracted)",
        "wall_time_s": time.time() - t_wall0,
    }
    config = {
        "brief_sha256": _sha256_file(os.path.join(ROOT, "BRIEF_WS.md")),
        "newton_commit": "b74df534bee62a17e0e57cc9cdfd1a67d91ca817",
        "asset_urdf_sha256": "2a270e19a9b9c7ca5eb62ec9d503d779281605b6bba881f5ac6e8090aa382497",
        "dt": FRAME_DT,
        "substeps": 4,
        "particle_count": rig.model.particle_count,
        "voxel_size": S.VOXEL_SIZE_M,
        "contact_params": {
            "default_shape_mu": 0.5,
            "pad_friction_mu": S.PAD_FRICTION_MU,
            "impulse_eps": IMPULSE_EPS,
            "max_speculative_extension_m": 0.005,
        },
        "provenance": {
            # consult item 4/5 + item 10: lift height is 5 cm (code + brief);
            # the "10 cm" in an external description was wrong.
            "lift_height_m": LIFT_M,
            "pad_collision_face_mm": [17.5, 18.5],
            "pad_collision_boxes_urdf_mm": ["22x15x20", "17.5x7x23.5"],
            "git_sha": _git_sha(),
            "controller_mode": "effort_controlled_open_loop",
            "closure_terminology": "effort-controlled (open-loop joint effort, EFFORT mode, zero finger stiffness) - NOT closed-loop force control",
        },
        "windows": {
            "judgment": "lift_complete..settle_end inclusive",
            "thresholds": judgment.JudgmentThresholds().__dict__,
            "phases_s": {
                "settle": PHASE_SETTLE_S, "close_hold": PHASE_CLOSE_HOLD_S,
                "lift": PHASE_LIFT_S, "postlift_hold": PHASE_POSTLIFT_HOLD_S,
                "final_settle": PHASE_FINAL_SETTLE_S,
            },
        },
        "f_g_convention": "per_finger_normal_mean",
        "seed_rng_derivation": "np.random.SeedSequence([1234, seed]) -> xy pose jitter",
        "profile_id": profile_id,
        "coupling_params": coupling_params_dict(FRAME_DT, S.VOXEL_SIZE_M),
        "calibration": calibration,
        "material": {
            "E_pa": S.BLOCK_E_PA, "nu": S.BLOCK_NU, "rho": S.BLOCK_RHO,
            "sigma_y_pa": sigma_y, "mpm_damping_s": S.BLOCK_MPM_DAMPING,
            "yield_pressure_pa": S.YIELD_PRESSURE_FACTOR * sigma_y,
            "yield_pressure_factor": S.YIELD_PRESSURE_FACTOR,
            "tensile_yield_ratio": S.TENSILE_YIELD_RATIO,
            "viscosity_pa_s": S.VISCOSITY_PA_S,
            # P4 first-class observable: measured realized bilateral-sum plateau
            # (gn2-dynamic-ladder.json); per-finger is half the sum.
            "f_bearing_capacity_n": F_BEARING_CAPACITY_N.get(int(sigma_y)),
        },
    }
    if extra_config:
        config.update(extra_config)
    doc = io_schemas.make("e1.v1", payload, config)
    if out_json:
        io_schemas.write_json(out_json, doc)
    return doc
