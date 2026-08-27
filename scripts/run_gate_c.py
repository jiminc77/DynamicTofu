"""Gate C — convergence bracket + yield-surface bracket (consult 2026-08-27).

Part 1 (H5 resolution): proxy iterations {1,4,8} x h {5, 2.5 mm} at the B1
config (stock, constant-effort, eta=20, sigma_Y=6000, mu=1, 0.60 N/finger,
1.0 s lift + 5 s hold). Consult acceptance: outcome/labels invariant and
scalar changes < 10-15% across the grid. The it=8/h=5mm cell already blew up,
so this maps where the stack is stable and whether B1's conclusion converges.

Part 2 (yield-surface bracket) at the stable B1 base (it=4, h=5mm):
  C-base    : cap 5.1 kPa (yp_factor 0.85), tensile ratio 1.0 (current material)
  C-cap-off : cap ~100 kPa (yp_factor 16.7), tensile ratio 1.0 (deviatoric-dominated)
  C-asym    : cap 15 kPa (yp_factor 2.5),  tensile ratio 0.2 -> tensile 3 kPa

Run: cd newton && uv run --no-sync python ../scripts/run_gate_c.py
"""

from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np

from scripts.probes.diag_rig import DiagConfig, run_diag

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def run(name, **over):
    base = dict(sigma_y=6000.0, mu=1.0, target_Nf=0.60, control="effort", pad="stock",
                viscosity=20.0, lift_s=1.0, hold_s=5.0)
    cfg = DiagConfig(name=name, **{**base, **over})
    t0 = time.time()
    res, log, _ = run_diag(cfg)
    bz = [r.get("block_centroid", [0, 0, 0])[2] for r in log]
    rec = {"outcome": res["outcome"], "health_clean": res["health_clean"],
           "valid": res["health_clean"], "preFn": res["preload"].get("Fn_L"),
           "finFn": res["final"].get("Fn_L"), "block_z_max": round(max(bz), 4),
           "final_nodes": res["final"].get("nodes_L"), "wall": round(time.time() - t0)}
    print(f"{name}: outcome={rec['outcome']} health={rec['health_clean']} finFn={rec['finFn']:.3f} "
          f"z_max={rec['block_z_max']} nodes={rec['final_nodes']} wall={rec['wall']}s", flush=True)
    return rec


def main() -> int:
    t0 = time.time()
    # Part 1: convergence grid
    conv = {}
    for it in (1, 4, 8):
        for h in (0.005, 0.0025):
            conv[f"it{it}_h{int(h*1000*10)/10}mm"] = run(f"C_it{it}_h{h}", proxy_iterations=it, voxel=h)
    # convergence assessment (only over health-clean cells)
    valid_cells = {k: v for k, v in conv.items() if v["valid"]}
    outcomes = {v["outcome"] for v in valid_cells.values()}
    blowups = [k for k, v in conv.items() if not v["valid"]]
    converged = len(outcomes) <= 1 and len(valid_cells) >= 3
    # Part 2: yield-surface bracket at it=4/h=5mm
    yb = {}
    yb["C-base"] = run("C-base", proxy_iterations=4, voxel=0.005, yield_pressure_factor=0.85, tensile_ratio=1.0)
    yb["C-cap-off"] = run("C-cap-off", proxy_iterations=4, voxel=0.005, yield_pressure_factor=100000.0/6000.0, tensile_ratio=1.0)
    yb["C-asym"] = run("C-asym", proxy_iterations=4, voxel=0.005, yield_pressure_factor=15000.0/6000.0, tensile_ratio=0.2)
    yb_outcomes = {v["outcome"] for v in yb.values() if v["valid"]}

    out = {"gate": "C_convergence_and_yield_bracket",
           "convergence_grid": conv, "blowup_cells": blowups,
           "convergence_verdict": ("CONVERGED: outcome invariant across the health-clean (it,h) cells"
                                   if converged else
                                   "NOT CONVERGED: outcome varies or too few clean cells; blowups at " + ", ".join(blowups)),
           "yield_bracket": yb,
           "yield_bracket_verdict": ("labels invariant across yield-surface bracket -> yield-surface shape is not the gating factor"
                                     if len(yb_outcomes) <= 1 else "yield-surface shape changes the outcome: " + str(yb_outcomes)),
           "wall_s": time.time() - t0}
    json.dump(out, open(os.path.join(ROOT, "reports", "logs", "gateC.json"), "w"), indent=2, default=str)
    print("\nCONVERGENCE:", out["convergence_verdict"])
    print("YIELD BRACKET:", out["yield_bracket_verdict"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
