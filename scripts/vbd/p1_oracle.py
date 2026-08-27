"""P1 pure-VBD Coulomb oracle (second consult) — reproducible runner.

Runs the E-ladder at the recipe force (and a 2 N reference) and persists full
per-run traces + acceptance measurements. HARD GATE: if 100-200 kPa cannot hold
<2 mm relative slip over the 5 s hold, STOP (do not touch tofu).

Run: cd newton && uv run --no-sync python ../scripts/vbd/p1_oracle.py
"""

from __future__ import annotations

import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import numpy as np

from src.vbd_rig2 import Vbd2Rig, Vbd2Config, run_vbd2

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
LADDER = [(100e3, 0.45), (100e3, 2.0), (200e3, 0.45), (200e3, 2.0), (500e3, 0.45), (1000e3, 0.45)]


def git_sha():
    try:
        return subprocess.check_output(["git", "-C", ROOT, "rev-parse", "HEAD"], text=True).strip()
    except Exception:
        return "unknown"


def one(E, F, substeps=20):
    rig, series, ph = run_vbd2(Vbd2Config(E_pa=E, grip_force_n=F, substeps=substeps))
    pre = [s for s in series if s["phase"] in ("ramp", "preload")]
    hold = [s for s in series if s["phase"] == "hold"]
    slip = max((s["rel_slip_mm"] for s in hold), default=999.0)
    peak = max((s["com_rise"] for s in series), default=0.0)
    pre_xy = max((abs(s["com"][0]) + abs(s["com"][1]) for s in pre), default=0.0) * 1000
    f = series[-1]
    return {"E_pa": E, "grip_force_n": F, "substeps": substeps,
            "hold_slip_mm": round(slip, 2), "peak_rise_mm": round(peak * 1000, 1),
            "final_rise_mm": round(f["com_rise"] * 1000, 1), "final_gap_mm": round(f["gap_m"] * 1000, 1),
            "pre_lift_xy_excursion_mm": round(pre_xy, 2), "finite_all": all(s["finite"] for s in series),
            "held_lt2mm": bool(slip < 2.0 and all(s["finite"] for s in series)),
            "series": series}


def main() -> int:
    out = {"gate": "V_P1_pure_vbd_coulomb_oracle", "git_sha": git_sha(), "runs": {}}
    for E, F in LADDER:
        r = one(E, F)
        out["runs"][f"E{int(E/1000)}kPa_F{F}"] = r
        print(f"E={int(E/1000)}kPa F={F}N: slip={r['hold_slip_mm']}mm peak={r['peak_rise_mm']}mm "
              f"final={r['final_rise_mm']}mm held<2mm={r['held_lt2mm']}", flush=True)
    # substep-doubling invariance on the best partial-hold cell
    inv = one(100e3, 2.0, substeps=40)
    out["substep_doubling_E100_F2"] = {"substeps40_slip_mm": inv["hold_slip_mm"], "substeps20_slip_mm": out["runs"]["E100kPa_F2.0"]["hold_slip_mm"]}
    out["hard_gate_100_200_pass"] = any(out["runs"][k]["held_lt2mm"] for k in out["runs"] if out["runs"][k]["E_pa"] <= 200e3)
    json.dump(out, open(os.path.join(ROOT, "reports", "logs", "vbd", "p1_oracle_full.json"), "w"), indent=2, default=str)
    print("hard_gate_pass:", out["hard_gate_100_200_pass"], "-> reports/logs/vbd/p1_oracle_full.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
