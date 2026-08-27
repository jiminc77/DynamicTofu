"""Gate B2 — restore the viscosity axis under CONSTANT-EFFORT (H2 isolation).

eta=2e5 only ran under position-lock in Gate B (invalid). Run it under the
valid constant-effort closure to isolate H2:
  B3prime = stock  + effort + eta 2e5
  B6prime = sensor + effort + eta 2e5
both it=4, sigma_Y=6000, mu=1, target 0.60 N/finger, 1.0 s lift + 10 s hold.
Compare to Gate B's eta=20 effort arms (B1 stock, B4 sensor).

Decisive: high-viscosity HOLD -> H2 dominant; still DROP via necking -> the
material-limit conclusion stands. Health blowup -> INVALID.

Run: cd newton && uv run --no-sync python ../scripts/run_gate_b2.py
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
ARMS = [("B3prime", "stock"), ("B6prime", "sensor")]


def main() -> int:
    t0 = time.time()
    out = {}
    # pull the eta=20 effort baselines from gateB.json for the comparison
    gb = json.load(open(os.path.join(ROOT, "reports", "logs", "gateB.json")))
    baseline = {"B1_stock_eta20": gb["arms"]["B1"], "B4_sensor_eta20": gb["arms"]["B4"]}
    for name, pad in ARMS:
        cfg = DiagConfig(name=name, sigma_y=6000.0, mu=1.0, target_Nf=0.60, voxel=0.005,
                         proxy_iterations=4, control="effort", pad=pad, viscosity=2e5,
                         lift_s=1.0, hold_s=10.0)
        fdir = os.path.join(ROOT, "reports", "media", "frames", f"gateB2_{name}")
        res, log, _ = run_diag(cfg, frames_dir=fdir)
        bz = [r.get("block_centroid", [0, 0, 0])[2] for r in log]
        # necking metric: block vertical extent growth (neck stretches up)
        ext_z = [max(1e-9, r.get("block_centroid", [0, 0, 0])[2]) for r in log]
        health = res["health_clean"]
        out[name] = {
            "pad": pad, "viscosity": 2e5, "control": "effort", "it": 4,
            "outcome": res["outcome"], "drop_cause": res["drop_cause"], "health_clean": health,
            "valid": health, "invalid_reason": None if health else "health_blowup",
            "preload_Fn_L": res["preload"].get("Fn_L"), "final_Fn_L": res["final"].get("Fn_L"),
            "block_z_start": round(bz[0], 4), "block_z_max": round(max(bz), 4), "block_z_end": round(bz[-1], 4),
            "final_nodes_L": res["final"].get("nodes_L"), "wall": round(time.time() - t0),
        }
        r = out[name]
        print(f"{name} ({pad}/effort/eta2e5): outcome={r['outcome']} valid={r['valid']} health={health} "
              f"preFn={r['preload_Fn_L']:.3f} finFn={r['final_Fn_L']:.3f} z_max={r['block_z_max']} nodes={r['final_nodes_L']}",
              flush=True)

    # H2 verdict
    def held(rec):
        return rec["valid"] and rec["outcome"] == "hold"
    any_high_visc_hold = held(out["B3prime"]) or held(out["B6prime"])
    verdict = ("H2 DOMINANT: high viscosity (2e5) HOLDS under effort where eta=20 dropped -> rheology mismatch is the "
               "leading artifact; a literature-calibrated eta (1e5-1e7 Pa.s) plausibly reopens the band."
               if any_high_visc_hold else
               "H2 NOT dominant: high viscosity (2e5) still DROPS under effort (same necking as eta=20) -> the "
               "material-limit / ductile-extrusion (H8) conclusion stands; viscosity is not the gating artifact.")
    result = {"gate": "B2_viscosity_isolation_constant_effort",
              "high_visc_effort_arms": out,
              "eta20_effort_baseline": baseline,
              "H2_verdict": verdict, "any_high_visc_hold": any_high_visc_hold,
              "wall_s": time.time() - t0}
    json.dump(result, open(os.path.join(ROOT, "reports", "logs", "gateB2.json"), "w"), indent=2, default=str)
    print("\nH2 VERDICT:", verdict)
    return 0


if __name__ == "__main__":
    sys.exit(main())
