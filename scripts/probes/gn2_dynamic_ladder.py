"""OFFICIAL sigma_Y monotonicity gate (P3, externally approved).

Dynamic-ladder onset: per material, dynamic crush trials with the E1 close
profile (0.3 s ramp to F + 2.0 s hold) over ladder F in {1.8, 2.5, 3.5, 5.0} N,
3 seeds each. Per trial: peak damage fraction (judgment predicate, entry-state
Jp) + realized bilateral normal (steady hold mean). F_onset = interpolation on
REALIZED bilateral normal between the bracketing ladder points where the
per-seed-mean peak fraction crosses 10%.

Gate: strictly increasing F_onset with sigma_Y; separation per consecutive
pair > max(0.05 N, 2 * max sd_seed); censoring (no crossing within the ladder)
valid ONLY at the largest sigma_Y. Rate adequacy: sigma=3333 repeated with a
0.6 s close ramp; |F_onset(0.3) - F_onset(0.6)| <= max(0.05 N, 5%).

Also measures f_bearing_capacity_n per material (P4 observable): the realized
bilateral normal plateau at the ladder top.

Run: cd newton && uv run --no-sync python ../scripts/probes/gn2_dynamic_ladder.py
"""

from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import numpy as np

from src.scene import BLOCK_CENTER
from scripts.probes.gn2_ramp_gate import realized_bilateral_normal
from scripts.probes.gn2_ar_probe import FRAME_DT, GRASP_Z, PREGRASP_Z, Rig

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
SIGMAS = [2000.0, 3333.0, 6000.0]
LADDER = [1.8, 2.5, 3.5, 5.0]
SEEDS = [0, 1, 2]
DAMAGE_FRAC = 0.10
HOLD_S = 2.0


def crush_point(sigma: float, f_cmd: float, seed: int, ramp_s: float = 0.3):
    rig = Rig(include_block=True, sigma_y=sigma, seed=seed, material_completion=True, pose_jitter_m=0.001)
    rig.step(int(0.5 / FRAME_DT))
    rig.move_ee((BLOCK_CENTER[0], BLOCK_CENTER[1], PREGRASP_Z), 1.5)
    rig.move_ee((BLOCK_CENTER[0], BLOCK_CENTER[1], GRASP_Z), 1.5)
    rig.move_ee_converge((BLOCK_CENTER[0], BLOCK_CENTER[1], GRASP_Z))
    n_ramp = int(ramp_s / FRAME_DT)
    for k in range(n_ramp):
        rig.fingers.apply(rig.control, f_cmd * (k + 1) / n_ramp)
        rig.step(1)
    peak_frac = 0.0
    normals = []
    n_hold = int(HOLD_S / FRAME_DT)
    for k in range(n_hold):
        rig.step(1)
        if (k + 1) % 10 == 0:
            peak_frac = max(peak_frac, float(np.mean(np.abs(rig.jp() - 1.0) > 0.05)))
            if k > n_hold // 2:
                normals.append(realized_bilateral_normal(rig))
    return {"sigma": sigma, "f_cmd": f_cmd, "seed": seed, "ramp_s": ramp_s,
            "peak_frac": peak_frac,
            "f_real_bilateral_n": float(np.mean(normals)) if normals else None,
            "health_clean": bool(rig.health.clean)}


def ladder_onset(points):
    """points: list per ladder force of dicts with seed results. Returns
    (onset_realized_n, status) using per-force means."""
    means = []
    for f in LADDER:
        rows = [p for p in points if p["f_cmd"] == f]
        means.append({
            "f_cmd": f,
            "frac_mean": float(np.mean([r["peak_frac"] for r in rows])),
            "f_real_mean": float(np.mean([r["f_real_bilateral_n"] for r in rows])),
        })
    prev = None
    for m in means:
        if prev is not None and prev["frac_mean"] <= DAMAGE_FRAC < m["frac_mean"]:
            w = (DAMAGE_FRAC - prev["frac_mean"]) / (m["frac_mean"] - prev["frac_mean"])
            onset = prev["f_real_mean"] + w * (m["f_real_mean"] - prev["f_real_mean"])
            return onset, "observed", means
        prev = m
    if means and means[0]["frac_mean"] > DAMAGE_FRAC:
        return None, "censored_low", means
    return None, "censored_high", means


def main() -> int:
    t0 = time.time()
    all_points = {}
    onsets, statuses, ladders = {}, {}, {}
    for sigma in SIGMAS:
        pts = [crush_point(sigma, f, s) for f in LADDER for s in SEEDS]
        all_points[sigma] = pts
        onset, status, means = ladder_onset(pts)
        onsets[sigma], statuses[sigma], ladders[sigma] = onset, status, means
        print(f"sigma={int(sigma)}: onset={None if onset is None else round(onset, 3)} ({status}) "
              f"fracs={[round(m['frac_mean'],3) for m in means]} "
              f"f_real={[round(m['f_real_mean'],2) for m in means]}")

    # per-seed onsets for sd (per material, using per-seed curves)
    def seed_onsets(sigma):
        vals = []
        for s in SEEDS:
            rows = [p for p in all_points[sigma] if p["seed"] == s]
            prev = None
            for f in LADDER:
                m = next(r for r in rows if r["f_cmd"] == f)
                if prev is not None and prev["peak_frac"] <= DAMAGE_FRAC < m["peak_frac"]:
                    w = (DAMAGE_FRAC - prev["peak_frac"]) / (m["peak_frac"] - prev["peak_frac"])
                    vals.append(prev["f_real_bilateral_n"] + w * (m["f_real_bilateral_n"] - prev["f_real_bilateral_n"]))
                    break
                prev = m
        return vals

    sds = {}
    for sigma in SIGMAS:
        so = seed_onsets(sigma)
        sds[sigma] = float(np.std(so)) if len(so) >= 2 else 0.0

    # censoring rule: only the largest sigma may be censored (censored_high)
    non_top_bad = [s for s in SIGMAS[:-1] if onsets[s] is None]
    conclusive = not non_top_bad
    observed = [s for s in SIGMAS if onsets[s] is not None]
    mono = all(onsets[a] < onsets[b] for a, b in zip(observed, observed[1:]))
    max_sd = max((sds[s] for s in observed), default=0.0)
    sep_ok = all((onsets[b] - onsets[a]) > max(0.05, 2 * max_sd) for a, b in zip(observed, observed[1:]))

    # rate adequacy at 3333 with 0.6 s close
    slow_pts = [crush_point(3333.0, f, s, ramp_s=0.6) for f in LADDER for s in SEEDS]
    slow_onset, slow_status, slow_means = ladder_onset(slow_pts)
    if onsets[3333.0] is not None and slow_onset is not None:
        rate_delta = abs(onsets[3333.0] - slow_onset)
        rate_ok = rate_delta <= max(0.05, 0.05 * onsets[3333.0])
    else:
        rate_delta, rate_ok = None, False

    bearing = {int(s): ladders[s][-1]["f_real_mean"] for s in SIGMAS}
    healthy = all(p["health_clean"] for pts in all_points.values() for p in pts) and \
        all(p["health_clean"] for p in slow_pts)
    gate_pass = conclusive and mono and sep_ok and rate_ok and healthy

    out = {
        "official": "P3 dynamic-ladder onset gate (externally approved)",
        "ladder_n": LADDER, "hold_s": HOLD_S,
        "onset_realized_n": {int(k): v for k, v in onsets.items()},
        "onset_status": {int(k): v for k, v in statuses.items()},
        "per_force_means": {int(k): v for k, v in ladders.items()},
        "per_seed_sd_n": {int(k): v for k, v in sds.items()},
        "rate_adequacy": {"onset_0p3": onsets[3333.0], "onset_0p6": slow_onset,
                          "delta_n": rate_delta, "ok": bool(rate_ok),
                          "slow_means": slow_means},
        "f_bearing_capacity_n": bearing,
        "checks": {"conclusive": bool(conclusive), "non_top_censored": non_top_bad,
                   "monotone_ok": bool(mono), "separation_ok": bool(sep_ok),
                   "health_all_clean": bool(healthy)},
        "raw_points": {int(k): v for k, v in all_points.items()},
        "raw_points_slow_3333": slow_pts,
        "gate_pass": bool(gate_pass),
        "wall_s": time.time() - t0,
    }
    with open(os.path.join(ROOT, "reports", "logs", "gn2-dynamic-ladder.json"), "w") as fh:
        json.dump(out, fh, indent=2)
    print(json.dumps({k: out[k] for k in ("onset_realized_n", "onset_status", "per_seed_sd_n",
                                          "rate_adequacy", "f_bearing_capacity_n", "checks")},
                     indent=2, default=str))
    print("DYNAMIC LADDER GATE:", "PASS" if gate_pass else "FAIL")
    return 0 if gate_pass else 1


if __name__ == "__main__":
    sys.exit(main())
