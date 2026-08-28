"""Day-2 tofu band grid (external ruling + addendum).

E in {7,15,25} kPa x F in {0.4,0.6,0.8,1.0,1.2} N (+ {1.5,2.0} N damage-branch),
substeps=80, mass-corrected 64 g tofu. SINGLE-SEED screening; boundary cells get
3-seed confirmation in a second pass (tofu_grid_confirm.py). Per-cell logging:
hold success, rel slip, Fn (applied+equilibrium), contact count, Coulomb load
fraction, bbox deformation, max/hold principal strain, vol-weighted P99, damaged
fraction at a PLACEHOLDER threshold. Stores the final per-tet strain field for
post-hoc labeling after the damage-threshold sign-off.

Run: cd newton && uv run --no-sync python ../scripts/vbd/tofu_grid.py
"""

from __future__ import annotations

import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
from tofu_probe import run_cell, PLACEHOLDER_EPS_DAMAGE  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
ES = [7e3, 15e3, 25e3]
FS = [0.4, 0.6, 0.8, 1.0, 1.2, 1.5, 2.0]
FIELD_DIR = os.path.join(ROOT, "reports", "logs", "vbd", "strain_fields")


def label(res):
    if not res["finite"]:
        return "blowup"
    if res["held_lt2mm"]:
        return "intact"          # damage overlay applied post-hoc after threshold sign-off
    return "slip"


def main() -> int:
    os.makedirs(FIELD_DIR, exist_ok=True)
    out = {"gate": "V_day2_tofu_grid_singleseed", "substeps": 80, "mass_corrected_64g": True,
           "placeholder_damage_threshold": PLACEHOLDER_EPS_DAMAGE,
           "git_sha": subprocess.check_output(["git", "-C", ROOT, "rev-parse", "HEAD"], text=True).strip(),
           "cells": {}}
    for E in ES:
        for F in FS:
            key = f"E{int(E/1000)}_F{F}"
            res, series = run_cell(E, F, eps=2e-4, substeps=80, save_field=os.path.join(FIELD_DIR, key + ".npz"))
            res["label"] = label(res)
            out["cells"][key] = res
            print(f"{key}: {res['label']} slip={res['hold_slip_mm']}mm load_frac={res['coulomb_load_fraction']} "
                  f"peak_strain={res['peak_principal_strain']} p99={res['hold_mean_p99_strain']} "
                  f"contacts={res['contacts_hold_mean']} bbox={res['bbox_final']}", flush=True)
        json.dump(out, open(os.path.join(ROOT, "reports", "logs", "vbd", "tofu_grid.json"), "w"), indent=2, default=str)
    # identify hold/slip boundary cells (per E, the F where label flips) for 3-seed confirmation
    boundaries = []
    for E in ES:
        labs = [(F, out["cells"][f"E{int(E/1000)}_F{F}"]["label"]) for F in FS]
        for i in range(1, len(labs)):
            if labs[i][1] != labs[i - 1][1]:
                boundaries.append(f"E{int(E/1000)}_F{labs[i-1][0]}")
                boundaries.append(f"E{int(E/1000)}_F{labs[i][0]}")
    out["boundary_cells_for_3seed"] = sorted(set(boundaries))
    json.dump(out, open(os.path.join(ROOT, "reports", "logs", "vbd", "tofu_grid.json"), "w"), indent=2, default=str)
    print("\nBOUNDARY CELLS (need 3-seed):", out["boundary_cells_for_3seed"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
