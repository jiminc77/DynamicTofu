"""Gate B — H1/H2/H6 factorial on sigma_Y=6000 tofu (consult 2026-08-27, Option a).

Constant-effort is the PRIMARY closure; position-lock is a LABELED DIAGNOSTIC
(valid on soft tofu per the user ruling: 0.6 N indents many mm, so sub-mm lock
drift does not collapse Fn as it did on the stiff E=70 kPa oracle).

Factorial (all: sigma_Y=6000, E=7 kPa, mu=1, target 0.60 N/finger, it=4,
1.0 s lift + 10 s hold, no transport, h=5 mm):
  pad {stock, sensor_format_pad} x control {effort, lock} x viscosity {20, 2e5}
| B1 stock  effort 20 | B2 stock  lock 20 | B3 stock  lock 2e5 |
| B4 sensor effort 20 | B5 sensor lock 20 | B6 sensor lock 2e5 |

Safeguards: (1) per-arm Fn(t) logged; a LOCK arm whose Fn collapses like the
oracle (final Fn < 25% of preload while still 'held') is marked INVALID, not
interpreted. (2) any health blowup -> INVALID (rerun/flag, never interpret).

Run: cd newton && uv run --no-sync python ../scripts/run_gate_b.py
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

BASE = dict(sigma_y=6000.0, mu=1.0, target_Nf=0.60, voxel=0.005, proxy_iterations=4,
            lift_s=1.0, hold_s=10.0, transport=False)
ARMS = [
    ("B1", dict(pad="stock", control="effort", viscosity=20.0)),
    ("B2", dict(pad="stock", control="lock", viscosity=20.0)),
    ("B3", dict(pad="stock", control="lock", viscosity=2e5)),
    ("B4", dict(pad="sensor", control="effort", viscosity=20.0)),
    ("B5", dict(pad="sensor", control="lock", viscosity=20.0)),
    ("B6", dict(pad="sensor", control="lock", viscosity=2e5)),
]


def main() -> int:
    t0 = time.time()
    results = {}
    frames_root = os.path.join(ROOT, "reports", "media", "frames")
    for name, over in ARMS:
        cfg = DiagConfig(name=name, **{**BASE, **over})
        fdir = os.path.join(frames_root, f"gateB_{name}")
        res, log, _rig = run_diag(cfg, frames_dir=fdir)
        preFn = res["preload"].get("Fn_L") or 0.0
        finFn = res["final"].get("Fn_L") or 0.0
        health = res["health_clean"]
        # per-arm validity
        invalid_reason = None
        if not health:
            invalid_reason = "health_blowup"
        elif over["control"] == "lock" and res["outcome"] == "hold" and abs(finFn) < 0.25 * abs(preFn):
            invalid_reason = "lock_Fn_collapse_like_oracle"
        bz = [r.get("block_centroid", [0, 0, 0])[2] for r in log]
        results[name] = {
            "cfg": {k: over[k] for k in over}, "outcome": res["outcome"], "drop_cause": res["drop_cause"],
            "health_clean": health, "valid": invalid_reason is None, "invalid_reason": invalid_reason,
            "preload_Fn_L": preFn, "final_Fn_L": finFn,
            "block_z_start": round(bz[0], 4) if bz else None, "block_z_max": round(max(bz), 4) if bz else None,
            "block_z_end": round(bz[-1], 4) if bz else None,
            "final_nodes_L": res["final"].get("nodes_L"), "n_log": res["n_log"],
            "frames": os.path.relpath(fdir, ROOT),
        }
        r = results[name]
        print(f"{name} ({over['pad']}/{over['control']}/eta{over['viscosity']:g}): outcome={r['outcome']} "
              f"valid={r['valid']}({r['invalid_reason']}) health={health} preFn={preFn:.3f} finFn={finFn:.3f} "
              f"z_max={r['block_z_max']} nodes={r['final_nodes_L']}")

    # signatures (only over VALID arms)
    def held(n):
        return results[n]["valid"] and results[n]["outcome"] == "hold"

    sig = []
    if (held("B4") or held("B5")) and not (held("B1") or held("B2")):
        sig.append("sensor>>stock->H1_pad_pressure")
    if held("B2") and not held("B1"):
        sig.append("lock>effort_stock->H6_jaw_advance_hurts_effort")
    if held("B5") and not held("B4"):
        sig.append("lock>effort_sensor->H6_jaw_advance_hurts_effort")
    if held("B3") and not held("B2"):
        sig.append("eta2e5>eta20_stock->H2_viscosity")
    if held("B6") and not held("B5"):
        sig.append("eta2e5>eta20_sensor->H2_viscosity")
    if held("B6"):
        sig.append("B6_holds->quasi_static_grasp_POSSIBLE_in_stack")

    out = {"gate": "B_H1_H2_H6_factorial_sigma6000", "base": BASE, "arms": results,
           "signatures_fired": sig,
           "any_hold": any(held(n) for n, _ in ARMS),
           "wall_s": time.time() - t0}
    with open(os.path.join(ROOT, "reports", "logs", "gateB.json"), "w") as fh:
        json.dump(out, fh, indent=2, default=str)
    print("\nSIGNATURES:", sig)
    print("ANY VALID HOLD:", out["any_hold"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
