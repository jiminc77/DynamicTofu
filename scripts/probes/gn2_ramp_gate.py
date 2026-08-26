"""G-N2 sigma_Y monotonicity gate (operational ramp, frozen definition).

- Quasi-static crush ramp: arm held still at the grasp pose; gripper commanded
  a linear force ramp 0.2 -> 6.0 N over 12 s (0.483 N/s).
- Damage-onset event: first crossing of damage fraction > 10% (judgment v1
  predicate, same definition as the sweep).
- F_onset: REALIZED bilateral normal force (sum of the two per-finger normal
  resultants) at onset, linearly interpolated between bracketing samples.
- 3 seeds per material (9 ramps); rate adequacy: sigma=3333 repeated at half
  rate (0.2415 N/s), |F_onset(full) - F_onset(half)| <= max(0.05 N, 5%).
- Direction: strictly increasing F_onset(2000) < F_onset(3333) < F_onset(6000);
  separation per consecutive pair > max(0.05 N, 2 * max sd_seed).
- Censoring: onset not reached by 6.0 N -> censored_high. Monotonicity on the
  censored ordering is valid ONLY if the censored material is the largest
  sigma_Y; any other censoring makes the gate inconclusive (= miss).

Run: cd newton && uv run --no-sync python ../scripts/probes/gn2_ramp_gate.py
"""

from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import numpy as np

from src.scene import BLOCK_CENTER
from src.coupling import node_reduction_per_body
from scripts.probes.gn2_ar_probe import FRAME_DT, GRASP_Z, PREGRASP_Z, Rig

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
SIGMAS = [2000.0, 3333.0, 6000.0]
SEEDS = [0, 1, 2]
RAMP_LO, RAMP_HI = 0.2, 6.0
RAMP_FULL_S = 12.0
DAMAGE_FRAC = 0.10


def realized_bilateral_normal(rig) -> float:
    bq = rig.state.body_q.numpy()
    reduced = node_reduction_per_body(rig.mpm, rig.state, bq, rig.model.body_com.numpy(), FRAME_DT)
    normals = rig.pad_normals_world()
    total = 0.0
    for b in rig.meta.finger_body_indices:
        F, _T, _n = reduced.get(b, (np.zeros(3), np.zeros(3), 0))
        total += abs(float(np.dot(F, normals[b])))
    return total


def run_ramp(sigma: float, seed: int, ramp_s: float):
    rig = Rig(include_block=True, sigma_y=sigma, seed=seed, material_completion=True, pose_jitter_m=0.001)
    rig.step(int(0.5 / FRAME_DT))
    rig.move_ee((BLOCK_CENTER[0], BLOCK_CENTER[1], PREGRASP_Z), 1.5)
    rig.move_ee((BLOCK_CENTER[0], BLOCK_CENTER[1], GRASP_Z), 1.5)
    rig.move_ee_converge((BLOCK_CENTER[0], BLOCK_CENTER[1], GRASP_Z))

    n = int(ramp_s / FRAME_DT)
    trace = []
    onset_f = None
    prev = None
    for k in range(n):
        f_cmd = RAMP_LO + (RAMP_HI - RAMP_LO) * (k + 1) / n
        rig.fingers.apply(rig.control, f_cmd)
        rig.step(1)
        if (k + 1) % 4 == 0:  # 50 Hz sampling
            frac = float(np.mean(np.abs(rig.jp() - 1.0) > 0.05))
            f_real = realized_bilateral_normal(rig)
            trace.append({"t": round(rig.t, 4), "f_cmd": f_cmd, "f_real_bilateral": f_real, "frac": frac})
            if onset_f is None and frac > DAMAGE_FRAC and prev is not None:
                f0, c0 = prev["f_real_bilateral"], prev["frac"]
                f1, c1 = f_real, frac
                w = (DAMAGE_FRAC - c0) / max(c1 - c0, 1e-12)
                onset_f = f0 + w * (f1 - f0)
                break
            prev = trace[-1]
    healthy = rig.health.clean
    return {
        "sigma": sigma, "seed": seed, "ramp_s": ramp_s,
        "f_onset_n": onset_f,
        "onset_status": "observed" if onset_f is not None else "censored_high",
        "health_clean": bool(healthy),
        "trace_tail": trace[-8:],
    }


def main() -> int:
    t0 = time.time()
    results = {}
    for sigma in SIGMAS:
        results[sigma] = [run_ramp(sigma, s, RAMP_FULL_S) for s in SEEDS]
        vals = [r["f_onset_n"] for r in results[sigma]]
        print(f"sigma={int(sigma)}: onsets={['%.3f' % v if v else 'censored' for v in vals]}")
    half = [run_ramp(3333.0, s, RAMP_FULL_S * 2.0) for s in SEEDS]
    print(f"sigma=3333 half-rate: onsets={['%.3f' % r['f_onset_n'] if r['f_onset_n'] else 'censored' for r in half]}")

    def stats(rs):
        vals = [r["f_onset_n"] for r in rs if r["f_onset_n"] is not None]
        return (float(np.mean(vals)), float(np.std(vals)), len(vals)) if vals else (None, None, 0)

    means, sds = {}, {}
    censored = {}
    for sigma in SIGMAS:
        m, sd, k = stats(results[sigma])
        means[sigma], sds[sigma] = m, sd
        censored[sigma] = any(r["f_onset_n"] is None for r in results[sigma])

    # censoring rule
    non_top_censored = [s for s in SIGMAS[:-1] if censored[s]]
    conclusive = not non_top_censored
    checks = {"conclusive": conclusive, "non_top_censored": non_top_censored}

    ordered = [s for s in SIGMAS if means[s] is not None]
    mono = all(means[a] < means[b] for a, b in zip(ordered, ordered[1:]))
    max_sd = max((sds[s] or 0.0) for s in ordered) if ordered else 0.0
    sep_ok = all(
        (means[b] - means[a]) > max(0.05, 2 * max_sd) for a, b in zip(ordered, ordered[1:])
    )
    checks["monotone_ok"] = bool(mono)
    checks["separation_ok"] = bool(sep_ok)
    if censored[SIGMAS[-1]] and not non_top_censored:
        checks["top_censored_note"] = "largest sigma censored_high: monotonicity evaluated on censored ordering (allowed)"

    hm, _, hk = stats(half)
    fm = means[3333.0]
    if fm is not None and hm is not None:
        checks["half_rate_delta_n"] = abs(fm - hm)
        checks["half_rate_ok"] = bool(abs(fm - hm) <= max(0.05, 0.05 * fm))
    else:
        checks["half_rate_ok"] = False

    all_healthy = all(r["health_clean"] for rs in results.values() for r in rs) and all(r["health_clean"] for r in half)
    gate_pass = conclusive and mono and sep_ok and checks["half_rate_ok"] and all_healthy

    out = {"results": {str(int(k)): v for k, v in results.items()}, "half_rate_3333": half,
           "means": {str(int(k)): v for k, v in means.items()},
           "sd": {str(int(k)): v for k, v in sds.items()},
           "checks": checks, "gate_pass": bool(gate_pass), "wall_s": time.time() - t0}
    with open(os.path.join(ROOT, "reports", "logs", "gn2-ramp-gate.json"), "w") as fh:
        json.dump(out, fh, indent=2)
    print(json.dumps({"means": out["means"], "checks": checks}, indent=2))
    print("RAMP GATE:", "PASS" if gate_pass else "FAIL")
    return 0 if gate_pass else 1


if __name__ == "__main__":
    sys.exit(main())
