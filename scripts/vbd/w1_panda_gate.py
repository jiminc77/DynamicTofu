#!/usr/bin/env python3
"""P-rig acceptance gates P-G0 and P-G1 (external directive).

Runs the Panda rig (via scripts/vbd/w1_panda.run_panda_cell, which reuses the
frozen run_transport_cell frame loop + judgment verbatim) on the gate cells and
compares each reduced label to the FROZEN band label. Per the directive, this
does NOT tune anything: it reports the raw diff. If any cell's label differs, the
gate FAILS and the result is escalated (no self-tuning).

P-G0: quasi-static a=1 transport row on E7, seed 0, all 7 forces.
  frozen: F0.4/0.6/0.8 slip, 1.0/1.2/1.5 intact, 2.0 damage.
P-G1: the three demo cells reproduce their labels.
  E15/a1/F1.2 intact, E15/a30/F1.2 slip, E7/a5/F2.0 damage.

Run (GPU, concurrent OK):
  cd newton && PYTHONPATH=/home/simx2204/Workspace/DynamicTofu \
    uv run --no-sync python ../scripts/vbd/w1_panda_gate.py
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

ROOT = Path("/home/simx2204/Workspace/DynamicTofu")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts/vbd"))

import w1_panda  # noqa: E402

LOG = ROOT / "reports/logs/vbd/panda"
OUT = LOG / "p_gate.json"

# (E_kPa, F_N, a, seed, frozen_label)
P_G0 = [
    (7, 0.4, 1, 0, "slip"), (7, 0.6, 1, 0, "slip"), (7, 0.8, 1, 0, "slip"),
    (7, 1.0, 1, 0, "intact"), (7, 1.2, 1, 0, "intact"), (7, 1.5, 1, 0, "intact"),
    (7, 2.0, 1, 0, "damage"),
]
P_G1 = [
    (15, 1.2, 1, 0, "intact"), (15, 1.2, 30, 0, "slip"), (7, 2.0, 5, 0, "damage"),
]


def run(gate_name, cells, results):
    for e, f, a, seed, frozen in cells:
        key = f"{gate_name}:E{e}_F{f:g}_a{a:g}_s{seed}"
        if any(r["key"] == key for r in results):
            continue
        t0 = time.monotonic()
        try:
            rec = w1_panda.run_panda_cell(e * 1000.0, f, a, seed)
            got = rec.get("label")
            entry = {"key": key, "gate": gate_name, "E_kPa": e, "F": f, "a": a, "seed": seed,
                     "frozen_label": frozen, "panda_label": got,
                     "reproduced": got == frozen, "dvf": rec.get("dvf"),
                     "ejected": rec.get("ejected"), "hold_slip_z_mm": rec.get("hold_slip_z_mm"),
                     "transport_slip_xz_mm": rec.get("transport_slip_xz_mm"),
                     "realized_F_g_n": rec.get("realized_F_g_n"),
                     "wall_s": round(time.monotonic() - t0, 1), "status": "ok"}
        except Exception as exc:  # noqa: BLE001
            entry = {"key": key, "gate": gate_name, "status": "error",
                     "error": repr(exc)[:300], "wall_s": round(time.monotonic() - t0, 1)}
        results.append(entry)
        ok = [r for r in results if r.get("status") == "ok"]
        payload = {
            "schema": "p_gate.v1",
            "note": ("P-rig label reproduction vs frozen band. Directive: do NOT tune to "
                     "force a pass; a mismatch is escalated with diffs."),
            "n_run": len(ok),
            "n_reproduced": sum(r.get("reproduced", False) for r in ok),
            "P_G0_pass": all(r["reproduced"] for r in ok if r["gate"] == "P-G0")
            and sum(r["gate"] == "P-G0" for r in ok) == len(P_G0),
            "P_G1_pass": all(r["reproduced"] for r in ok if r["gate"] == "P-G1")
            and sum(r["gate"] == "P-G1" for r in ok) == len(P_G1),
            "cells": results,
        }
        OUT.write_text(json.dumps(payload, indent=2) + "\n")
        st = "REPRO" if entry.get("reproduced") else ("ERR" if entry["status"] == "error" else "MISMATCH")
        print(f"{key}: {st} frozen={entry.get('frozen_label')} panda={entry.get('panda_label')} "
              f"dvf={entry.get('dvf')} {entry['wall_s']}s", flush=True)


def main() -> int:
    LOG.mkdir(parents=True, exist_ok=True)
    results = []
    if OUT.exists():
        try:
            results = json.loads(OUT.read_text()).get("cells", [])
        except (OSError, json.JSONDecodeError):
            results = []
    run("P-G1", P_G1, results)   # demo cells first (most decision-relevant)
    run("P-G0", P_G0, results)
    print("DONE", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
