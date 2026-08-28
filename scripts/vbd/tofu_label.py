"""Post-hoc judgment-v2 labeling from stored TEMPORAL-MAX strain fields.

Judgment v2 (VBD, approved): slip = >2 mm gripper-relative displacement;
damage = volume-weighted damaged-volume fraction (DVF, tets with eps1>0.15
Green principal strain) >= 0.5%, LATCHED over the trial (temporal max);
intact otherwise. Precedence damage>slip only if damage latches before drop
(non-binding here: slipping low-force cells have DVF~0).

Reports peak, P99, DVF per cell and emits the final phase-diagram table.

Run: cd newton && uv run --no-sync python ../scripts/vbd/tofu_label.py
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
EPS_DAMAGE = 0.15
DVF_MIN = 0.005
FIELD_DIR = os.path.join(ROOT, "reports", "logs", "vbd", "strain_fields")
ES = [7, 15, 25]
FS = [0.4, 0.6, 0.8, 1.0, 1.2, 1.5, 2.0]


def dvf_stats(npz_path):
    d = np.load(npz_path)
    s = d["temporal_max_principal_strain"] if "temporal_max_principal_strain" in d else d["max_principal_strain"]
    v = d["tet_rest_vol"]
    peak = float(s.max())
    order = np.argsort(s); cw = np.cumsum(v[order]) / v.sum()
    p99 = float(s[order][min(len(s) - 1, int(np.searchsorted(cw, 0.99)))])
    dvf = float(v[s > EPS_DAMAGE].sum() / v.sum())
    return peak, p99, dvf


def main() -> int:
    grid = json.load(open(os.path.join(ROOT, "reports", "logs", "vbd", "tofu_grid.json")))["cells"]
    labeled = {}
    for E in ES:
        for F in FS:
            key = f"E{E}_F{F}"
            slip_mm = grid[key]["hold_slip_mm"]
            peak, p99, dvf = dvf_stats(os.path.join(FIELD_DIR, key + ".npz"))
            slipped = slip_mm >= 2.0 or not grid[key]["finite"]
            if dvf >= DVF_MIN:
                label = "damage"
            elif slipped:
                label = "slip"
            else:
                label = "intact"
            labeled[key] = {"label": label, "slip_mm": slip_mm, "peak_strain": round(peak, 4),
                            "p99_strain": round(p99, 4), "dvf": round(dvf, 5), "damage_if_dvf_ge_0.5pct": dvf >= DVF_MIN}
            print(f"{key}: {label:7s} slip={slip_mm:.2f}mm peak={peak:.3f} p99={p99:.3f} DVF={dvf*100:.3f}%", flush=True)
    out = {"gate": "V_day2_judgment_v2_labels", "eps_damage": EPS_DAMAGE, "dvf_min_frac": DVF_MIN, "cells": labeled}
    # phase-diagram table
    tab = ["| E\\\\F | " + " | ".join(f"{F}" for F in FS) + " |", "|" + "---|" * (len(FS) + 1)]
    sym = {"intact": "I", "slip": "s", "damage": "D", "blowup": "X"}
    for E in ES:
        tab.append(f"| **{E}kPa** | " + " | ".join(sym[labeled[f'E{E}_F{F}']['label']] for F in FS) + " |")
    out["phase_diagram"] = tab
    out["damage_cells"] = [k for k, v in labeled.items() if v["label"] == "damage"]
    json.dump(out, open(os.path.join(ROOT, "reports", "logs", "vbd", "tofu_labels_v2.json"), "w"), indent=2, default=str)
    print("\n".join(tab))
    print("DAMAGE CELLS (need h=4mm confirm):", out["damage_cells"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
