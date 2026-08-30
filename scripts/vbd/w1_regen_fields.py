#!/usr/bin/env python3
"""R1 field regeneration (GPU, seed 0, frozen config).

The 126-cell W1 screen persisted only the scalar dvf at the frozen eps=0.15;
the per-element post-lift temporal-max strain FIELDS were never saved
(run_screen calls run_transport_cell without save_field). The eps in
{0.10, 0.20} columns of the R1 sensitivity therefore require re-simulating
seed 0 with field persistence.

This re-runs each seed-0 cell with the FROZEN configuration and the existing
output-only save_field hook (no physics, no label logic touched), writing
reports/logs/vbd/w1_strain_fields/<cell>.npz. It is:
  * resumable  -- a cell whose .npz already exists is skipped;
  * reproduction-gated -- the re-derived frozen-eps label MUST equal the stored
    screen label; a mismatch (expected occasionally from documented GPU
    run-to-run nondeterminism) is recorded, not silently accepted;
  * fault-tolerant -- a per-cell exception is logged and the sweep continues.

Cells are ordered low-a first (a=1,2.5,5 ...) so the closure-critical region is
regenerated first and a partial run still supports a partial eps sweep.

Run (GPU):
  cd newton && PYTHONPATH=/home/simx2204/Workspace/DynamicTofu \
    uv run --no-sync python /home/simx2204/Workspace/DynamicTofu/scripts/vbd/w1_regen_fields.py
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

ROOT = Path("/home/simx2204/Workspace/DynamicTofu")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts/vbd"))

from w1_transport import run_transport_cell  # noqa: E402

LOG = ROOT / "reports/logs/vbd"
SCREEN = LOG / "w1_screen"
FIELD_DIR = LOG / "w1_strain_fields"
SUMMARY = LOG / "w1_regen_summary.json"
A_ORDER = [1, 2.5, 5, 10, 20, 30]


def cell_name(e: int, a: float, f: float) -> str:
    return f"E{e}_a{a:g}_F{f:g}"


def load_grid() -> list[dict]:
    cells = []
    for path in sorted(SCREEN.glob("*_s0.json")):
        r = json.loads(path.read_text())
        cells.append({
            "E_pa": float(r["E_pa"]), "E": round(float(r["E_pa"]) / 1000),
            "a": float(r["commanded_a_peak_m_s2"]), "F": float(r["grip_force_n"]),
            "stored_label": r["label"], "stored_dvf": r["dvf"],
        })
    cells.sort(key=lambda c: (A_ORDER.index(c["a"]) if c["a"] in A_ORDER else 99, c["E"], c["F"]))
    return cells


def main() -> int:
    FIELD_DIR.mkdir(parents=True, exist_ok=True)
    grid = load_grid()
    results = []
    if SUMMARY.exists():
        try:
            results = json.loads(SUMMARY.read_text()).get("cells", [])
        except (OSError, json.JSONDecodeError):
            results = []
    done_names = {r["cell"] for r in results}
    started = time.monotonic()
    processed = 0
    for c in grid:
        name = cell_name(c["E"], c["a"], c["F"])
        fp = FIELD_DIR / f"{name}.npz"
        if fp.exists() and name in done_names:
            continue
        t0 = time.monotonic()
        try:
            rec = run_transport_cell(c["E_pa"], c["F"], c["a"], 0, save_field=str(fp))
            relabel = rec.get("label")
            entry = {
                "cell": name, "status": "ok",
                "stored_label": c["stored_label"], "relabel": relabel,
                "label_reproduced": relabel == c["stored_label"],
                "stored_dvf": c["stored_dvf"], "regen_dvf": rec.get("dvf"),
                "wall_s": round(time.monotonic() - t0, 1),
            }
        except Exception as exc:  # noqa: BLE001 -- fault-tolerant sweep
            entry = {"cell": name, "status": "error", "error": repr(exc)[:300],
                     "wall_s": round(time.monotonic() - t0, 1)}
        results = [r for r in results if r["cell"] != name] + [entry]
        SUMMARY.write_text(json.dumps({
            "schema": "w1_regen_summary.v1",
            "field_dir": str(FIELD_DIR.relative_to(ROOT)),
            "n_done": sum(1 for r in results if r["status"] == "ok"),
            "n_error": sum(1 for r in results if r["status"] == "error"),
            "n_label_mismatch": sum(1 for r in results
                                    if r["status"] == "ok" and not r["label_reproduced"]),
            "cells": results,
        }, indent=2) + "\n")
        processed += 1
        flag = "" if entry.get("label_reproduced", True) else "  !! LABEL MISMATCH"
        print(f"[{len(results)}/126] {name} {entry['status']} "
              f"repro={entry.get('label_reproduced')} {entry['wall_s']}s "
              f"(elapsed {(time.monotonic()-started)/60:.1f}m){flag}", flush=True)
    print(f"DONE processed={processed} total_ok="
          f"{sum(1 for r in results if r['status']=='ok')}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
