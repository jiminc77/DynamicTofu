"""3-seed confirmation of the label-boundary cells (v1 band rule).

Single chaotic runs are not acceptable for band boundaries. For each boundary
cell flagged by tofu_grid.py, run seeds 1 and 2 (seed 0 from the grid) and
report the 3-seed label consensus (>=2/3 sets the confirmed label).

Run: cd newton && uv run --no-sync python ../scripts/vbd/tofu_grid_confirm.py
"""

from __future__ import annotations

import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.dirname(__file__))

from tofu_probe import run_cell  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def label(res):
    return "blowup" if not res["finite"] else ("intact" if res["held_lt2mm"] else "slip")


def parse(key):
    e, f = key.split("_F")
    return float(e[1:]) * 1000, float(f)


def main() -> int:
    grid = json.load(open(os.path.join(ROOT, "reports", "logs", "vbd", "tofu_grid.json")))
    cells = grid["boundary_cells_for_3seed"]
    out = {"gate": "V_day2_boundary_3seed", "substeps": 80,
           "git_sha": subprocess.check_output(["git", "-C", ROOT, "rev-parse", "HEAD"], text=True).strip(), "cells": {}}
    for key in cells:
        E, F = parse(key)
        seed0 = {"seed": 0, "label": grid["cells"][key]["label"], "slip_mm": grid["cells"][key]["hold_slip_mm"]}
        seeds = [seed0]
        for s in (1, 2):
            res, _ = run_cell(E, F, eps=2e-4, substeps=80, seed=s)
            seeds.append({"seed": s, "label": label(res), "slip_mm": res["hold_slip_mm"]})
        labs = [x["label"] for x in seeds]
        consensus = max(set(labs), key=labs.count)
        unanimous = len(set(labs)) == 1
        out["cells"][key] = {"seeds": seeds, "labels": labs, "consensus_label": consensus, "unanimous": unanimous}
        print(f"{key}: seeds={labs} -> consensus={consensus} unanimous={unanimous} "
              f"slips={[round(x['slip_mm'],2) for x in seeds]}", flush=True)
        json.dump(out, open(os.path.join(ROOT, "reports", "logs", "vbd", "tofu_grid_confirm.json"), "w"), indent=2, default=str)
    print("\nboundary confirmation done -> tofu_grid_confirm.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
