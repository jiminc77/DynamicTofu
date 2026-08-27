"""Gate A — elastic Coulomb oracle (external consult 2026-08-27, section c).

Separates numerics from rheology: a near-elastic block (E=70 kPa, nu=0.30,
sigma_Y=1 MPa so it never yields, eta=1e6) grasped with position-clamp after
preload, 1.0 s lift + 5 s hold, no transport, stock pad.

HARD GATE: if A4 or A5 DROP, the contact stack itself is invalid -> STOP
everything and escalate (do not interpret the tofu sweeps).

Run: cd newton && uv run --no-sync python ../scripts/run_gate_a.py
"""

from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scripts.probes.diag_rig import DiagConfig, run_diag

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

ELASTIC = dict(E_pa=70e3, nu=0.30, sigma_y=1.0e6, viscosity=1.0e6,
               tensile_ratio=1.0, yield_pressure_factor=0.85,
               control="lock", lift_s=1.0, hold_s=5.0, transport=False)

RUNS = [
    # A0 harness sanity: a deliberately strong grip on the elastic block MUST
    # hold; if it drops, the harness/contact is broken (not a valid signal).
    ("A0", dict(mu=1.0, target_Nf=2.0, voxel=0.005, proxy_iterations=8), "hold"),
    ("A1", dict(mu=0.0, target_Nf=0.80, voxel=0.005, proxy_iterations=8), "drop"),   # frictionless -> must drop
    ("A2", dict(mu=1.0, target_Nf=0.25, voxel=0.005, proxy_iterations=8), "drop"),   # under-force -> must drop
    ("A3", dict(mu=1.0, target_Nf=0.45, voxel=0.005, proxy_iterations=1), "measure"),
    ("A4", dict(mu=1.0, target_Nf=0.45, voxel=0.005, proxy_iterations=4), "hold"),   # HARD GATE
    ("A5", dict(mu=1.0, target_Nf=0.45, voxel=0.005, proxy_iterations=8), "hold"),   # HARD GATE
    ("A6", dict(mu=1.0, target_Nf=0.45, voxel=0.0025, proxy_iterations=8), "hold"),
]


def main() -> int:
    t0 = time.time()
    results = {}
    frames_root = os.path.join(ROOT, "reports", "media", "frames")
    for name, over, required in RUNS:
        cfg = DiagConfig(name=name, **{**ELASTIC, **over})
        res, log, _rig = run_diag(cfg, frames_dir=os.path.join(frames_root, f"gateA_{name}"))
        res["required"] = required
        res["meets_required"] = (required == "measure") or (res["outcome"] == required)
        results[name] = res
        print(f"{name}: outcome={res['outcome']} (required {required}) cause={res['drop_cause']} "
              f"Fn_L={res['final'].get('Fn_L'):.3f} Ft_L={res['final'].get('Ft_L'):.3f} "
              f"jaw={res['final'].get('jaw_gap_m'):.4f} health={res['health_clean']}")

    # signatures
    sig = []
    if results["A1"]["outcome"] == "hold":
        sig.append("A1_holds->numerical_adhesion")
    if results["A2"]["outcome"] == "hold":
        sig.append("A2_holds->excess_numerical_friction")
    hard_gate_fail = results["A4"]["outcome"] == "drop" or results["A5"]["outcome"] == "drop"
    if hard_gate_fail:
        sig.append("A4/A5_drop->CONTACT_STACK_INVALID")
    if results["A3"]["outcome"] == "drop" and results["A4"]["outcome"] == "hold":
        sig.append("A3_only_drops->H5_coupling_iterations")
    # A5 vs A6 gap (H4 discretization): compare final jaw / slip
    try:
        a5_ft = abs(results["A5"]["final"]["Ft_L"]); a6_ft = abs(results["A6"]["final"]["Ft_L"])
        if abs(a5_ft - a6_ft) > 0.10 * max(a5_ft, a6_ft, 1e-6):
            sig.append("A5!=A6->H4_discretization_sensitivity")
    except Exception:
        pass

    out = {
        "gate": "A_elastic_coulomb_oracle",
        "material": {"E_pa": 70e3, "nu": 0.30, "sigma_Y_pa": 1e6, "viscosity_pa_s": 1e6},
        "controller_mode": "effort_controlled_open_loop_preload_then_position_lock",
        "runs": results,
        "signatures_fired": sig,
        "hard_gate": "FAIL_CONTACT_STACK_INVALID" if hard_gate_fail else "PASS",
        "wall_s": time.time() - t0,
    }
    with open(os.path.join(ROOT, "reports", "logs", "gateA.json"), "w") as fh:
        json.dump(out, fh, indent=2, default=str)
    print("\nSIGNATURES:", sig)
    print("HARD GATE:", out["hard_gate"])
    return 0 if not hard_gate_fail else 1


if __name__ == "__main__":
    sys.exit(main())
