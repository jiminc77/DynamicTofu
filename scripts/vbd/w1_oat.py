#!/usr/bin/env python3
"""R4 (stretch): one-at-a-time contact-parameter sensitivity (external consult #3).

friction_epsilon OAT {1e-4, 5e-4, 1e-3} on 3 slip-boundary cells and mu {0.8, 1.2}
on 2 cells (frozen baselines eps=2e-4, mu=1.0). For each perturbed run, the label
is re-derived by the frozen judgment and compared to the frozen screen label. This
CHARACTERISES sensitivity; it is diagnostic-only and changes no production value
(each run is tagged frozen_check=false + sensitivity_overrides). Physics/labels of
the frozen band are untouched.

Run (GPU, idle only -- lower priority than the R1 regen):
  cd newton && PYTHONPATH=/home/simx2204/Workspace/DynamicTofu \
    uv run --no-sync python ../scripts/vbd/w1_oat.py
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

ROOT = Path("/home/simx2204/Workspace/DynamicTofu")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts/vbd"))

from w1_transport import _json_safe, run_transport_cell  # noqa: E402

LOG = ROOT / "reports/logs/vbd"
SCREEN = LOG / "w1_screen"
OUT = LOG / "g_oat.json"
RAW = LOG / "g_oat_raw"

# slip-boundary cells (slip side, adjacent to an intact cell): (E_kPa, F_N, a)
EPS_CELLS = [(7, 0.8, 1), (15, 0.6, 1), (25, 0.6, 1)]
MU_CELLS = [(7, 0.8, 1), (15, 0.6, 1)]
EPS_VALUES = [1e-4, 5e-4, 1e-3]   # frozen baseline 2e-4
MU_VALUES = [0.8, 1.2]            # frozen baseline 1.0


def frozen_label(e, f, a):
    p = SCREEN / f"E{e}_F{f:g}_a{a:g}_s0.json"
    return json.loads(p.read_text())["label"]


def run_one(results, e, f, a, *, friction_epsilon=None, mu_pair=None):
    tag = (f"E{e}_F{f:g}_a{a:g}_"
           + (f"eps{friction_epsilon:g}" if friction_epsilon is not None else f"mu{mu_pair:g}"))
    if any(r["tag"] == tag for r in results):
        return
    t0 = time.monotonic()
    try:
        rec = run_transport_cell(e * 1000.0, f, a, 0,
                                 friction_epsilon=friction_epsilon, mu_pair=mu_pair)
        RAW.mkdir(parents=True, exist_ok=True)
        (RAW / f"{tag}.json").write_text(json.dumps(_json_safe(rec), indent=2, allow_nan=False) + "\n")
        fl = frozen_label(e, f, a)
        entry = {"tag": tag, "E_kPa": e, "F": f, "a": a,
                 "param": ("friction_epsilon" if friction_epsilon is not None else "mu"),
                 "value": friction_epsilon if friction_epsilon is not None else mu_pair,
                 "frozen_baseline": {"friction_epsilon": 2e-4, "mu": 1.0},
                 "frozen_label": fl, "oat_label": rec["label"], "changed": rec["label"] != fl,
                 "dvf": rec.get("dvf"), "hold_slip_z_mm": rec.get("hold_slip_z_mm"),
                 "transport_slip_xz_mm": rec.get("transport_slip_xz_mm"),
                 "ejected": rec.get("ejected"), "status": "ok",
                 "wall_s": round(time.monotonic() - t0, 1)}
    except Exception as exc:  # noqa: BLE001
        entry = {"tag": tag, "status": "error", "error": repr(exc)[:300],
                 "wall_s": round(time.monotonic() - t0, 1)}
    results.append(entry)
    ok = [r for r in results if r["status"] == "ok"]
    OUT.write_text(json.dumps({
        "schema": "g_oat.v1",
        "note": ("R4 OAT contact-parameter sensitivity vs frozen baseline (eps=2e-4, mu=1.0). "
                 "Diagnostic only; no production value changed; each run tagged non-frozen."),
        "eps_values": EPS_VALUES, "mu_values": MU_VALUES,
        "n_run": len(ok), "n_label_changed": sum(r.get("changed", False) for r in ok),
        "cells": results,
    }, indent=2) + "\n")
    st = "CHANGED" if entry.get("changed") else ("ERR" if entry["status"] == "error" else "same")
    print(f"{tag}: {st} frozen={entry.get('frozen_label')} oat={entry.get('oat_label')} "
          f"dvf={entry.get('dvf')} {entry['wall_s']}s", flush=True)


def main() -> int:
    results = []
    if OUT.exists():
        try:
            results = json.loads(OUT.read_text()).get("cells", [])
        except (OSError, json.JSONDecodeError):
            results = []
    for e, f, a in EPS_CELLS:
        for eps in EPS_VALUES:
            run_one(results, e, f, a, friction_epsilon=eps)
    for e, f, a in MU_CELLS:
        for mu in MU_VALUES:
            run_one(results, e, f, a, mu_pair=mu)
    print("DONE", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
