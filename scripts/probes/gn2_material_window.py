"""Condition-3 evidence: (yield_pressure_factor, tensile_yield_ratio) window map.

For each candidate pair, at sigma_Y=3333: CRUSH (5 N, 1.5 s hold) and
GENTLE (1.5 N ramped close 1 s, 0.5 s hold, 5 cm smoothstep lift, 1.2 s hold).
Records judgment-predicate damage fractions (hold + window peak), |Jp-1|
distribution percentiles, geometry, health. The winning pair is re-validated
on sigma_Y = 2000 and 6000.

Window criterion (sign-off condition 1): crush fires well above 10 percent;
gentle stays below 10 percent (hold AND lift window).

Run: cd newton && uv run --no-sync python ../scripts/probes/gn2_material_window.py
"""

from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import numpy as np

import src.scene as S
from src.scene import BLOCK_CENTER
from scripts.probes.gn2_ar_probe import FRAME_DT, GRASP_Z, PREGRASP_Z, Rig

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
SIGMA_MAIN = 3333.0
GRID = [(0.6, 0.75), (0.6, 1.0), (0.75, 0.75), (0.75, 1.0), (0.85, 0.75), (0.85, 1.0)]


def jp_stats(jp):
    dev = np.abs(jp - 1.0)
    return {
        "frac_gt_0p05": float(np.mean(dev > 0.05)),
        "dev_percentiles": {str(p): float(np.percentile(dev, p)) for p in (50, 75, 90, 95, 99)},
        "jp_min": float(jp.min()),
        "jp_max": float(jp.max()),
    }


def run_case(sigma, ypf, tyr, mode):
    S.YIELD_PRESSURE_FACTOR = ypf
    rig = Rig(include_block=True, sigma_y=sigma, material_completion=True)
    rig.model.mpm.tensile_yield_ratio.fill_(tyr)
    rig.mpm.model.mpm.tensile_yield_ratio.fill_(tyr)
    rig.step(int(0.5 / FRAME_DT))
    rig.move_ee((BLOCK_CENTER[0], BLOCK_CENTER[1], PREGRASP_Z), 1.5)
    rig.move_ee((BLOCK_CENTER[0], BLOCK_CENTER[1], GRASP_Z), 1.5)
    rig.move_ee_converge((BLOCK_CENTER[0], BLOCK_CENTER[1], GRASP_Z))
    f_target = 5.0 if mode == "crush" else 1.5
    n_ramp = int((0.3 if mode == "crush" else 1.0) / FRAME_DT)
    for k in range(n_ramp):
        rig.fingers.apply(rig.control, f_target * (k + 1) / n_ramp)
        rig.step(1)
    rig.step(int(0.5 / FRAME_DT))
    frac_hold = float(np.mean(np.abs(rig.jp() - 1) > 0.05))
    frac_peak = frac_hold
    if mode == "gentle":
        n = int(0.3 / FRAME_DT)
        for k in range(n):
            s = (k + 1) / n
            s = s * s * (3 - 2 * s)
            rig.move_ee((BLOCK_CENTER[0], BLOCK_CENTER[1], GRASP_Z + 0.05 * s), FRAME_DT)
        for _ in range(12):
            rig.step(int(0.1 / FRAME_DT))
            frac_peak = max(frac_peak, float(np.mean(np.abs(rig.jp() - 1) > 0.05)))
    else:
        for _ in range(15):
            rig.step(int(0.1 / FRAME_DT))
            frac_peak = max(frac_peak, float(np.mean(np.abs(rig.jp() - 1) > 0.05)))
    pq = rig.state.particle_q.numpy()
    return {
        "sigma_y_pa": sigma,
        "yield_pressure_factor": ypf,
        "tensile_yield_ratio": tyr,
        "mode": mode,
        "frac_hold": frac_hold,
        "frac_window_peak": frac_peak,
        "jp_final": jp_stats(rig.jp()),
        "ext_final": [round(float(pq[:, i].max() - pq[:, i].min()), 4) for i in range(3)],
        "rise_mm": round((float(pq[:, 2].mean()) - 0.2187) * 1000, 1),
        "health_clean": bool(rig.health.clean),
    }


def main() -> int:
    t0 = time.time()
    results = []
    for ypf, tyr in GRID:
        crush = run_case(SIGMA_MAIN, ypf, tyr, "crush")
        gentle = run_case(SIGMA_MAIN, ypf, tyr, "gentle")
        ok = crush["frac_window_peak"] > 0.10 and gentle["frac_window_peak"] < 0.10 and \
            crush["health_clean"] and gentle["health_clean"]
        margin = crush["frac_window_peak"] - gentle["frac_window_peak"]
        results.append({"ypf": ypf, "tyr": tyr, "crush": crush, "gentle": gentle,
                        "window_ok": bool(ok), "margin": margin})
        print(f"ypf={ypf} tyr={tyr}: crush={crush['frac_window_peak']:.3f} "
              f"gentle={gentle['frac_window_peak']:.3f} ok={ok}")

    winners = [r for r in results if r["window_ok"]]
    cross = []
    best = None
    if winners:
        best = max(winners, key=lambda r: r["margin"])
        for sigma in (2000.0, 6000.0):
            c = run_case(sigma, best["ypf"], best["tyr"], "crush")
            g = run_case(sigma, best["ypf"], best["tyr"], "gentle")
            ok = c["frac_window_peak"] > 0.10 and g["frac_window_peak"] < 0.10
            cross.append({"sigma": sigma, "crush": c, "gentle": g, "ok": bool(ok)})
            print(f"cross sigma={int(sigma)} (ypf={best['ypf']} tyr={best['tyr']}): "
                  f"crush={c['frac_window_peak']:.3f} gentle={g['frac_window_peak']:.3f} ok={ok}")

    out = {"grid": results, "best": None if best is None else {"ypf": best["ypf"], "tyr": best["tyr"]},
           "cross_material": cross, "wall_s": time.time() - t0}
    with open(os.path.join(ROOT, "reports", "logs", "gn2-material-window.json"), "w") as fh:
        json.dump(out, fh, indent=2)
    all_ok = best is not None and all(c["ok"] for c in cross)
    print("WINDOW:", "FOUND" if all_ok else ("PARTIAL" if best else "NONE"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
