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
LIFT_M = 0.05
TRANSPORT_Z = GRASP_Z + LIFT_M
IMPULSE_EPS = 1e-8
SAMPLE_EVERY_TICKS = 2  # 100 Hz judgment sampling at the 200 Hz tick


def _sha256_file(path: str) -> str:
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


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

    def advance(n, on_tick=None):
        for _ in range(n):
            if on_tick:
                on_tick()
            rig.step(1)
            tick[0] += 1
            if tick[0] % SAMPLE_EVERY_TICKS == 0:
                rec.sample()

    # --- settle -------------------------------------------------------------
    phase_ts["settle"] = rig.t - t0
    advance(int(PHASE_SETTLE_S / FRAME_DT))

    # --- close to commanded force + hold ------------------------------------
    phase_ts["close_hold"] = rig.t - t0
    n_close = int(PHASE_CLOSE_HOLD_S / FRAME_DT)
    n_ramp = int(0.3 / FRAME_DT)
    for k in range(n_close):
        rig.fingers.apply(rig.control, f_g * min(1.0, (k + 1) / n_ramp))
        rig.step(1)
        tick[0] += 1
        if tick[0] % SAMPLE_EVERY_TICKS == 0:
            rec.sample()

    # --- lift 5 cm in 0.3 s (smoothstep) ------------------------------------
    phase_ts["lift"] = rig.t - t0
    n_lift = int(PHASE_LIFT_S / FRAME_DT)
    for k in range(n_lift):
        s = (k + 1) / n_lift
        s = s * s * (3 - 2 * s)
        rig.move_ee((S.BLOCK_CENTER[0], S.BLOCK_CENTER[1], GRASP_Z + LIFT_M * s), FRAME_DT)
        tick[0] += 1
        if tick[0] % SAMPLE_EVERY_TICKS == 0:
            rec.sample()
    phase_ts["lift_complete"] = rig.t - t0
    rec.mark_lift_complete()

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
    if "reversal_time" in prof.get("phase_timestamps", {}):
        phase_ts["reversal"] = phase_ts["transport"] + prof["phase_timestamps"]["reversal_time"]

    # --- final settle --------------------------------------------------------
    phase_ts["final_settle"] = rig.t - t0
    advance(int(PHASE_FINAL_SETTLE_S / FRAME_DT))
    phase_ts["settle_end"] = rig.t - t0

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

    payload = {
        "sigma_y_pa": sigma_y,
        "a_peak_cmd_ms2": a_peak,
        "a_peak_realized_ms2": a_realized,
        "f_g_n": f_g,
        "seed": seed,
        "labels": labels,
        "cell_color": verdict["cell_color"],
        "damage_latch_t": verdict["damage_latch_t"],
        "drop_t": verdict["drop_t"],
        "damage_after_drop": verdict["damage_after_drop"],
        "peak_damage_fraction": verdict["peak_damage_fraction"],
        "health": rig.health.report(),
        "phase_timestamps": {k: round(v, 4) for k, v in phase_ts.items()},
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
        },
    }
    if extra_config:
        config.update(extra_config)
    doc = io_schemas.make("e1.v1", payload, config)
    if out_json:
        io_schemas.write_json(out_json, doc)
    return doc
