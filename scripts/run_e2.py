"""E2 runner: nine guaranteed tactile trials at sigma_Y=3333 mid-band force.

Contract (pending-approval.md E2 + stage-03 intent):
- Matrix: a_peak {1, 5, 15} x seeds {0, 1, 2} at F_mid from the certified
  Stage-A band at a=1 (pass-set level nearest the geometric mean of
  F_min/F_max, ties to the lower level; single-point band yields that point).
- The top-level coupled tick (5 ms = 200 Hz) is the sample; harvest exactly
  once per tick; force = impulse / exact MPM dt.
- Raw per-node field per pad + pad poses recorded (concat+offsets); aggregates
  derived from the same stored field, bitwise recomputable.
- Verification per trial: median dt <= 5 ms, unique timestamps, bitwise
  recompute, size guard (npz > 100 MB or > 2000 nodes/sample -> stop).
- Extension to 27 only if Stage B yields usable bands (checked externally);
  a band-less material is censored_no_band, never an invented force.

Usage: cd newton && uv run --no-sync python ../scripts/run_e2.py [--scratch] [--f-mid <N>]
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np

import src.scene as S
from src import io_schemas, tactile
from src.coupling import coupling_params_dict
from scripts.probes.gn2_ar_probe import FRAME_DT, GRASP_Z, PREGRASP_Z, Rig
from src.trial import _sha256_file  # same brief-hash discipline

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
ACCELS = [1.0, 5.0, 15.0]
SEEDS = [0, 1, 2]
LIFT_M = 0.05
NPZ_GUARD_BYTES = 100 * 1024 * 1024
NODES_GUARD = 2000


def f_mid_from_band(band_json_path: str):
    doc = io_schemas.read_json(band_json_path)
    row = next(r for r in doc["payload"]["rows"] if r["a_peak"] == 1.0)
    if row["band_status"] == "empty" or row["F_min"] is None:
        return None, "censored_no_band"
    cells = None  # pass-set levels are the certified passing forces
    # reconstruct pass set from coverage: rows only carry extrema; use geometric mean of extrema
    lo, hi = row["F_min"], row["F_max"]
    if row["band_status"] == "single_point":
        return lo, "single_point"
    gm = math.sqrt(lo * hi)
    grid = [0.3, 0.5, 0.8, 1.2, 1.8, 2.5, 3.5, 5.0]
    inband = [f for f in grid if lo <= f <= hi]
    best = min(inband, key=lambda f: (abs(f - gm), f))
    return best, "geometric_mean_nearest_level"


def run_e2_trial(a_peak: float, seed: int, f_mid: float, out_dir: str, calibration: dict, f_mid_selection: str):
    from src import profiles

    t0 = time.time()
    rig = Rig(include_block=True, sigma_y=3333.0, seed=seed, material_completion=True, pose_jitter_m=0.001)
    rec = tactile.TactileRecorder(rig)
    prof = profiles.generate("trapz_reversal_default", a_peak, dt=FRAME_DT)

    def advance(n):
        for _ in range(n):
            rig.step(1)
            rec.capture(FRAME_DT)

    rig.step(int(0.5 / FRAME_DT))  # un-recorded scene settle
    rig.move_ee((S.BLOCK_CENTER[0], S.BLOCK_CENTER[1], PREGRASP_Z), 1.5)
    rig.move_ee((S.BLOCK_CENTER[0], S.BLOCK_CENTER[1], GRASP_Z), 1.5)
    rig.move_ee_converge((S.BLOCK_CENTER[0], S.BLOCK_CENTER[1], GRASP_Z))

    rec.mark("record_start")
    advance(int(0.5 / FRAME_DT))                      # settle
    rec.mark("close")
    n_ramp = int(0.3 / FRAME_DT)
    n_close = int(0.5 / FRAME_DT)
    for k in range(n_close):
        rig.fingers.apply(rig.control, f_mid * min(1.0, (k + 1) / n_ramp))
        rig.step(1)
        rec.capture(FRAME_DT)
    rec.mark("lift")
    n_lift = int(0.3 / FRAME_DT)
    for k in range(n_lift):
        s = (k + 1) / n_lift
        s = s * s * (3 - 2 * s)
        rig.move_ee((S.BLOCK_CENTER[0], S.BLOCK_CENTER[1], GRASP_Z + LIFT_M * s), FRAME_DT)
        rec.capture(FRAME_DT)
    rec.mark("lift_complete")
    advance(int(0.2 / FRAME_DT))                      # post-lift hold
    rec.mark("transport")
    pos = np.atleast_2d(prof["pos"])
    if pos.shape[0] == 1 and pos.shape[1] > 2:
        pos = pos.T
    reversal_rel = prof["phase_timestamps"].get("reversal_time")
    t_transport0 = rig.t
    for k in range(len(pos)):
        target = (S.BLOCK_CENTER[0], S.BLOCK_CENTER[1] + float(pos[k, 0]), GRASP_Z + LIFT_M)
        rig.move_ee(target, FRAME_DT)
        rec.capture(FRAME_DT)
        if reversal_rel is not None and abs((rig.t - t_transport0) - reversal_rel) < FRAME_DT / 2:
            rec.mark("reversal")
    rec.mark("final_settle")
    advance(int(0.5 / FRAME_DT))
    rec.mark("settle_end")

    raw = rec.arrays(S.VOXEL_SIZE_M)
    raw["phase_marks"] = json.dumps(rec.phase_marks)

    # --- verification -------------------------------------------------------
    ts = raw["sample_t_s"]
    dts = np.diff(ts)
    checks = {
        "median_dt_s": float(np.median(dts)),
        "rate_ok": bool(np.median(dts) <= 0.005 + 1e-9),
        "unique_ts_ok": bool(len(np.unique(ts)) == len(ts)),
        "max_nodes_per_sample": int(np.max(np.diff(raw["sample_offsets"]))) if len(raw["sample_offsets"]) > 1 else 0,
        "health_clean": bool(rig.health.clean),
    }
    npz_path = os.path.join(out_dir, f"e2_a{a_peak:g}_seed{seed}.npz")
    np.savez_compressed(npz_path, **raw)
    checks["npz_bytes"] = os.path.getsize(npz_path)
    checks["size_guard_ok"] = checks["npz_bytes"] <= NPZ_GUARD_BYTES and checks["max_nodes_per_sample"] <= NODES_GUARD
    with np.load(npz_path, allow_pickle=False) as npz:
        re_agg = tactile.recompute_aggregates(npz)
        checks["bitwise_recompute_ok"] = all(
            np.array_equal(npz[k], np.asarray(v), equal_nan=True) for k, v in re_agg.items()
        )

    config = {
        "brief_sha256": _sha256_file(os.path.join(ROOT, "BRIEF_WS.md")),
        "newton_commit": "b74df534bee62a17e0e57cc9cdfd1a67d91ca817",
        "asset_urdf_sha256": "2a270e19a9b9c7ca5eb62ec9d503d779281605b6bba881f5ac6e8090aa382497",
        "dt": FRAME_DT, "substeps": 4, "particle_count": rig.model.particle_count,
        "voxel_size": S.VOXEL_SIZE_M,
        "contact_params": {"default_shape_mu": 0.5, "pad_friction_mu": S.PAD_FRICTION_MU,
                           "impulse_eps": tactile.IMPULSE_EPS, "max_speculative_extension_m": 0.005},
        "windows": {"phases_s": {"settle": 0.5, "close_hold": 0.5, "lift": 0.3,
                                 "postlift_hold": 0.2, "final_settle": 0.5}},
        "f_g_convention": "per_finger_normal_mean",
        "seed_rng_derivation": "np.random.SeedSequence([1234, seed]) -> xy pose jitter",
        "profile_id": "trapz_reversal_default",
        "coupling_params": coupling_params_dict(FRAME_DT, S.VOXEL_SIZE_M),
        "calibration": calibration,
        "raw_field_recorded": True, "raw_field_layout": "concat+offsets",
        "impulse_eps": tactile.IMPULSE_EPS,
        "pad_normal_local": [0.0, 1.0, 0.0],
        "pad_frame_convention": "finger body frame; pad-outward normal block_to_pad_outward",
        "coupled_tick_s": FRAME_DT,
        "aggregates_derived_from_raw": True,
        "taxel_binning": "post_hoc_out_of_scope",
        "signal_source": "solver_collider_impulse",
        "f_mid_n": f_mid, "f_mid_selection": f_mid_selection,
        "material": {"sigma_y_pa": 3333.0,
                     "yield_pressure_pa": S.YIELD_PRESSURE_FACTOR * 3333.0,
                     "yield_pressure_factor": S.YIELD_PRESSURE_FACTOR},
    }
    payload = {
        "a_peak_cmd_ms2": a_peak, "seed": seed, "npz": os.path.relpath(npz_path, ROOT),
        "checks": checks, "phase_marks": rec.phase_marks,
        "peak_shear_n": [float(np.max(raw["agg_shear_n"][:, i])) for i in (0, 1)],
        "wall_time_s": time.time() - t0,
    }
    doc = io_schemas.make("e2.v1", payload, config)
    io_schemas.write_json(os.path.join(out_dir, f"e2_a{a_peak:g}_seed{seed}.json"), doc)
    return doc, raw


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scratch", action="store_true")
    ap.add_argument("--f-mid", type=float, default=None, help="override F_mid (scratch/shakedown only)")
    ap.add_argument("--only", default=None, help="single case a:seed for shakedown, e.g. 5:0")
    args = ap.parse_args()

    out_dir = os.path.join(ROOT, "reports", "logs", "e2-scratch") if args.scratch else os.path.join(ROOT, "ralph", "results")
    os.makedirs(out_dir, exist_ok=True)

    if args.f_mid is not None:
        f_mid, f_mid_sel = args.f_mid, "override_shakedown"
    else:
        band_path = os.path.join(ROOT, "ralph", "results", "e1_band_3333.json")
        if not os.path.exists(band_path):
            print("FATAL: Stage-A band artifact missing; E2 requires the certified a=1 band.")
            return 2
        f_mid, f_mid_sel = f_mid_from_band(band_path)
        if f_mid is None:
            print("censored_no_band: no usable a=1 band for sigma=3333; E2 cannot run at an invented force.")
            return 3

    cal = {"slope": 1.0, "intercept": 0.0, "residual": 0.0, "hysteresis": 0.0, "status": "scratch"}
    cal_path = os.path.join(ROOT, "reports", "logs", "gn2-calibration.json")
    if os.path.exists(cal_path) and not args.scratch:
        cj = json.load(open(cal_path))
        cal = {"slope": cj["slope"], "intercept": cj["intercept_n"],
               "residual": cj["max_residual_n"], "hysteresis": cj["hysteresis_n"]}

    cases = [(a, s) for a in ACCELS for s in SEEDS]
    if args.only:
        a, s = args.only.split(":")
        cases = [(float(a), int(s))]

    summary = {"f_mid_n": f_mid, "f_mid_selection": f_mid_sel, "per_seed": {}}
    shear_peaks = {}
    for a, s in cases:
        doc, raw = run_e2_trial(a, s, f_mid, out_dir, cal, f_mid_sel)
        c = doc["payload"]["checks"]
        ok = all(c[k] for k in ("rate_ok", "unique_ts_ok", "size_guard_ok", "bitwise_recompute_ok", "health_clean"))
        print(f"a={a:g} seed={s}: checks_ok={ok} median_dt={c['median_dt_s']*1000:.2f}ms "
              f"nodes_max={c['max_nodes_per_sample']} npz={c['npz_bytes']/1e6:.1f}MB "
              f"wall={doc['payload']['wall_time_s']:.0f}s")
        shear_peaks[(a, s)] = max(doc["payload"]["peak_shear_n"])
        cent = raw["agg_centroid_pad_m"]
        finite = cent[np.isfinite(cent).all(axis=2).any(axis=1)]
        exc = 0.0
        if len(finite):
            for side in (0, 1):
                cs = cent[:, side, :]
                cs = cs[np.isfinite(cs).all(axis=1)]
                if len(cs) > 1:
                    exc = max(exc, float(np.max(np.linalg.norm(cs - cs[0], axis=1))))
        area = raw["agg_area_m2"]
        summary["per_seed"].setdefault(str(s), {})[f"a{a:g}"] = {
            "peak_shear_n": max(doc["payload"]["peak_shear_n"]),
            "max_centroid_excursion_mm": exc * 1000,
            "area_change_m2": float(np.ptp(area.sum(axis=1))),
            "checks_ok": bool(ok),
        }
        if not ok and (not c["size_guard_ok"]):
            print("SIZE GUARD tripped (PM-9): stopping for inspection.")
            break

    if not args.only:
        for s in SEEDS:
            lo, hi = shear_peaks.get((1.0, s)), shear_peaks.get((15.0, s))
            if lo and hi:
                summary["per_seed"][str(s)]["peak_shear_ratio_a15_over_a1"] = hi / lo
    with open(os.path.join(out_dir, "e2_summary.json"), "w") as fh:
        json.dump(summary, fh, indent=2)
    print("summary ->", os.path.join(out_dir, "e2_summary.json"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
