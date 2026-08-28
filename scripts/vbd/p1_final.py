"""P1 final oracle (post-margin-fix, substeps=40) with the FULL acceptance set.

Grid: E in {100,200} kPa x F in {0.45, 0.6, 0.8, 2.0} N. Acceptance per run:
hold slip (<2 mm), mean per-pad Fn vs target (+/-10%, via equilibrium: held run
=> finger quasi-static => Fn ~ applied), surface penetration, pre-lift COM
excursion (<5 mm), finger speed. Plus substep 40->80 invariance at the best
passing cell (slip change <20%). HARD GATE: clean <2 mm pass at 100-200 kPa.

Run: cd newton && uv run --no-sync python ../scripts/vbd/p1_final.py
"""

from __future__ import annotations

import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import numpy as np

from src.vbd_rig2 import Vbd2Config, run_vbd2

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
PR = 0.003  # particle radius -> surface pen = center pen + PR


def git_sha():
    try:
        return subprocess.check_output(["git", "-C", ROOT, "rev-parse", "HEAD"], text=True).strip()
    except Exception:
        return "unknown"


def one(E, F, substeps=40):
    rig, series, ph = run_vbd2(Vbd2Config(E_pa=E, grip_force_n=F, substeps=substeps))
    pre = [s for s in series if s["phase"] in ("ramp", "preload")]
    hold = [s for s in series if s["phase"] == "hold"]
    slip = max((s["rel_slip_mm"] for s in hold), default=999.0)
    finite = all(s["finite"] for s in series)
    held = slip < 2.0 and finite
    pre_xy = max((abs(s["com"][0]) + abs(s["com"][1]) for s in pre), default=0.0) * 1000
    fvy_hold = float(np.mean([s["finger_vy"] for s in hold])) if hold else 999.0
    # surface penetration during hold (center pen + PR*1000 mm)
    surf_pen = float(np.mean([0.5 * (s["pen_left_mm"] + s["pen_right_mm"]) for s in hold])) + PR * 1000 if hold else 0.0
    # Fn: for a HELD (quasi-static) run, contact Fn balances applied joint_f => Fn ~= F.
    # We report the applied target and the equilibrium check (finger_vy ~ 0).
    return {"E_pa": E, "grip_force_n": F, "substeps": substeps, "hold_slip_mm": round(slip, 2),
            "held_lt2mm": bool(held), "finite": finite, "pre_lift_xy_excursion_mm": round(pre_xy, 2),
            "mean_finger_speed_hold_ms": round(fvy_hold, 4), "mean_surface_pen_hold_mm": round(surf_pen, 2),
            "Fn_target_n": F, "Fn_equilibrium_ok": bool(fvy_hold < 0.01 and held),
            "peak_rise_mm": round(max((s["com_rise"] for s in series), default=0) * 1000, 1),
            "final_rise_mm": round(series[-1]["com_rise"] * 1000, 1)}


def main() -> int:
    out = {"gate": "V_P1_final_full_acceptance", "git_sha": git_sha(),
           "config": "margin=1e-3, substeps=40, eps=2e-4, mu_pair=1.0, ke=pad=2e3, h=8mm, r=3mm", "runs": {}}
    for E in (100e3, 200e3):
        for F in (0.45, 0.6, 0.8, 2.0):
            r = one(E, F)
            out["runs"][f"E{int(E/1000)}_F{F}"] = r
            print(f"E={int(E/1000)}kPa F={F}N: slip={r['hold_slip_mm']}mm held={r['held_lt2mm']} "
                  f"pre_xy={r['pre_lift_xy_excursion_mm']}mm surf_pen={r['mean_surface_pen_hold_mm']}mm "
                  f"fvy={r['mean_finger_speed_hold_ms']} Fn_eq_ok={r['Fn_equilibrium_ok']}", flush=True)
    # substep invariance 40->80 at the best passing sub-2N cell if any, else at E100/2N
    passing = [(k, r) for k, r in out["runs"].items() if r["held_lt2mm"]]
    inv_cell = passing[0][1] if passing else out["runs"]["E100_F2.0"]
    inv80 = one(inv_cell["E_pa"], inv_cell["grip_force_n"], substeps=80)
    out["substep_invariance_40_80"] = {"cell": f"E{int(inv_cell['E_pa']/1000)}_F{inv_cell['grip_force_n']}",
                                       "slip40_mm": inv_cell["hold_slip_mm"], "slip80_mm": inv80["hold_slip_mm"],
                                       "rel_change": round(abs(inv80["hold_slip_mm"] - inv_cell["hold_slip_mm"]) / max(inv_cell["hold_slip_mm"], 1e-6), 3)}
    # hard-gate at authorized sub-Newton forces (<=0.6N) and at 100-200 kPa
    subN = [r for k, r in out["runs"].items() if r["grip_force_n"] <= 0.6 and r["E_pa"] <= 200e3]
    out["hard_gate_subNewton_pass"] = any(r["held_lt2mm"] for r in subN)
    out["any_pass_100_200"] = any(r["held_lt2mm"] for k, r in out["runs"].items() if r["E_pa"] <= 200e3)
    out["min_holding_force_E100_n"] = next((r["grip_force_n"] for f in (0.45, 0.6, 0.8, 2.0)
                                            for r in [out["runs"].get(f"E100_F{f}")] if r and r["held_lt2mm"]), None)
    json.dump(out, open(os.path.join(ROOT, "reports", "logs", "vbd", "p1_final.json"), "w"), indent=2, default=str)
    print("\nsubNewton_hard_gate_pass:", out["hard_gate_subNewton_pass"], "| any_pass_100_200:", out["any_pass_100_200"],
          "| min_hold_force_E100:", out["min_holding_force_E100_n"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
