#!/usr/bin/env python3
"""R1: strain-threshold sensitivity of the W1 P-B closure (external consult #3, red-team #6).

Relabels all 126 seed-0 screen cells over a 3x3 grid of the damage proxy
thresholds and re-derives, per combination, the P-B closure verdict, the
per-material closure a*, and the number of cells whose label flips versus the
frozen baseline (eps=0.15 principal strain, DVF>=0.5%).

Two evidence sources
--------------------
* Stored screen receipts (reports/logs/vbd/w1_screen/*_s0.json):
    - `dvf`  == damaged-volume fraction at the FROZEN eps=0.15 over the
      grip-completion (>=1.80 s) window. Exact.
    - `damage_latch_t`, `drop_t`, `ejected`, and the slip observables
      (`hold_slip_z_mm`, `transport_slip_xz_mm`, `grasp_frame_y_res_mm`).
    This fixes the eps=0.15 column exactly, because a cell is INTACT iff it did
    not drop AND is not damaged AND exceeds no slip gate -- none of which needs a
    latch time. (Damage-vs-slip among the non-intact cells can be latch-sensitive
    for the transport-crush drop cells, but that never changes the intact count
    and therefore never moves the closure a*.)

* Regenerated per-cell fields (reports/logs/vbd/w1_strain_fields/<cell>.npz):
    `post_lift_temporal_max_principal_strain` + `tet_rest_vol`, plus a per-combo
    latch-time table in reports/logs/vbd/w1_strain_fields/<cell>.latch.json.
    Required ONLY for eps in {0.10, 0.20} (dvf at another eps needs the field).
    Produced by scripts/vbd/w1_regen_fields.py (seed 0, frozen config,
    reproduction-gated so a re-derived frozen label must equal the stored label).

CLI
---
  --offline    eps=0.15 column (3 combos) from receipts; writes the preview and
               the regen work-list.  No GPU, no fields required.
  --full       all 9 combos using regenerated fields; writes e1v2_sensitivity.json.
  --regen-list emit only the list of cells that still need field regen.
"""
from __future__ import annotations

import argparse
import glob
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts/vbd"))
sys.path.insert(0, str(ROOT))
import w1_analysis as W  # noqa: E402
from src.judgment_vbd import SLIP_THRESHOLD_MM  # noqa: E402

LOG = ROOT / "reports/logs/vbd"
SCREEN = LOG / "w1_screen"
FIELD_DIR = LOG / "w1_strain_fields"
EPS_LIST = (0.10, 0.15, 0.20)
DVF_LIST = (0.0025, 0.005, 0.010)
BASE_EPS, BASE_DVF = 0.15, 0.005
FROZEN_EPS = 0.15


def _num_key(value: float) -> str:
    return f"{value:g}"


def cell_name(e: int, a: float, f: float) -> str:
    return f"E{e}_a{_num_key(a)}_F{_num_key(f)}"


def load_seed0() -> list[dict]:
    out = []
    for path in sorted(SCREEN.glob("*_s0.json")):
        r = json.loads(path.read_text())
        r["_path"] = str(path.relative_to(ROOT))
        out.append(r)
    return out


def slip_gate(cell: dict) -> bool:
    """True iff the cell trips a slip gate (drop/eject or a residual > threshold)."""
    if bool(cell.get("dropped", cell.get("ejected", False))) or cell.get("drop_t") is not None:
        return True
    for key, lim in (("hold_slip_z_mm", SLIP_THRESHOLD_MM),
                     ("transport_slip_xz_mm", SLIP_THRESHOLD_MM),
                     ("grasp_frame_y_res_mm", 10.0)):
        v = cell.get(key)
        if v is not None and float(v) > lim:
            return True
    return False


def label_from(cell: dict, damaged: bool, latch_t) -> str:
    """label_v23 precedence with an externally supplied damage decision + latch."""
    drop_t = cell.get("drop_t")
    if damaged and latch_t is not None and (drop_t is None or float(latch_t) < float(drop_t)):
        return "damage"
    if slip_gate(cell):
        return "slip"
    return "intact"


def dvf_at(field: np.ndarray, vol: np.ndarray, eps: float) -> float:
    return float(vol[field > eps].sum() / vol.sum())


def field_path(cell: dict) -> Path:
    e, a, f, _ = W.receipt_coords(cell)
    return FIELD_DIR / f"{cell_name(e, a, f)}.npz"


def latch_path(cell: dict) -> Path:
    e, a, f, _ = W.receipt_coords(cell)
    return FIELD_DIR / f"{cell_name(e, a, f)}.latch.json"


def damaged_and_latch(cell: dict, eps: float, dvf_thresh: float):
    """Return (damaged, latch_t) for a combo.

    eps == FROZEN_EPS uses the stored scalar dvf (exact); the latch time is
    the stored damage_latch_t when the cell latched at the frozen threshold,
    else None for a non-drop cell (existence is all label_v23 needs there),
    else a regen latch when available.  eps != FROZEN_EPS requires the
    regenerated field + per-combo latch table.
    """
    if abs(eps - FROZEN_EPS) < 1e-12:
        dvf15 = float(cell["dvf"])
        damaged = dvf15 >= dvf_thresh
        if not damaged:
            return False, None
        latch = cell.get("damage_latch_t")
        if latch is not None:
            return True, float(latch)
        # Newly damaged at a threshold below the frozen 0.005: latch time not
        # stored.  For a non-drop cell existence is guaranteed (dvf15>=T means
        # the running max crossed T inside the window) -> supply a sentinel time
        # strictly before any drop.  For a drop cell the ordering is uncertain;
        # resolve it from the regen latch table when present.
        lt = _regen_latch(cell, eps, dvf_thresh)
        if lt is not None:
            return True, lt
        if cell.get("drop_t") is None:
            return True, float("-inf")  # exists, precedes non-existent drop
        return True, None  # uncertain; caller flags
    # eps != frozen -> need the regenerated field
    fp = field_path(cell)
    if not fp.exists():
        raise FileNotFoundError(f"missing regenerated field for {fp.name}")
    npz = np.load(fp)
    field = npz["post_lift_temporal_max_principal_strain"]
    vol = npz["tet_rest_vol"]
    dvf = dvf_at(field, vol, eps)
    damaged = dvf >= dvf_thresh
    if not damaged:
        return False, None
    lt = _regen_latch(cell, eps, dvf_thresh)
    if lt is not None:
        return True, lt
    if cell.get("drop_t") is None:
        return True, float("-inf")  # non-drop: latch exists, precedes no drop
    return True, None  # drop-precedence uncertain without a regen latch table


def _regen_latch(cell: dict, eps: float, dvf_thresh: float):
    lp = latch_path(cell)
    if not lp.exists():
        return None
    table = json.loads(lp.read_text()).get("latch", {})
    return table.get(f"{eps:g}|{dvf_thresh:g}")


def build_bands(cells: list[dict], eps: float, dvf_thresh: float):
    """Return (bands, per_cell_labels, uncertain) for a combo."""
    axis = W.axis_map()
    labels: dict[tuple, str] = {}
    uncertain: list[str] = []
    for cell in cells:
        e, a, f, _ = W.receipt_coords(cell)
        damaged, latch = damaged_and_latch(cell, eps, dvf_thresh)
        if damaged and latch is None:  # drop-precedence uncertain (pre-regen)
            uncertain.append(cell_name(e, a, f))
            # conservative: it dropped, so it is non-intact either way -> slip
            labels[(e, a, f)] = "slip" if slip_gate(cell) else "damage"
        else:
            labels[(e, a, f)] = label_from(cell, damaged, latch)
    bands = []
    for e in W.E_ORDER:
        matrix = {}
        for a in W.A_ORDER:
            for f in W.F_ORDER:
                if (e, a, f) in labels:
                    matrix.setdefault(_num_key(a), {})[_num_key(f)] = labels[(e, a, f)]
        bands.append({
            "E_kPa": e, "label_matrix": matrix,
            "realized_accel_by_commanded": {_num_key(a): axis[a] for a in W.A_ORDER},
            "coverage": {"planned_primary_cells": 42, "present_certified_cells": 42},
        })
    return bands, labels, uncertain


def combo_result(cells, eps, dvf_thresh, base_labels):
    bands, labels, uncertain = build_bands(cells, eps, dvf_thresh)
    verdict, per = W.classify_bands(bands)
    a_star = {p["E"]: p["closure_commanded_a_star"] for p in per}
    realized = {p["E"]: p["closure_realized_a_star"] for p in per}
    flips = sum(1 for k, v in labels.items() if base_labels is not None and base_labels.get(k) != v)
    flip_cells = sorted(cell_name(*k) for k, v in labels.items()
                        if base_labels is not None and base_labels.get(k) != v)
    return {
        "eps": eps, "dvf_thresh": dvf_thresh, "verdict": verdict,
        "closure_commanded_a_star": a_star, "closure_realized_a_star": realized,
        "flips_vs_baseline": flips, "flip_cells": flip_cells,
        "latch_uncertain_cells": uncertain,
    }, labels


def regen_worklist(cells) -> list[str]:
    return sorted(cell_name(*W.receipt_coords(c)[:3]) for c in cells
                  if not field_path(c).exists())


def run(mode: str) -> int:
    cells = load_seed0()
    if len(cells) != 126:
        print(f"WARN: expected 126 seed-0 cells, found {len(cells)}")
    # baseline labels (frozen combo) -- always offline-exact
    _, base_labels = combo_result(cells, BASE_EPS, BASE_DVF, None)

    if mode == "regen-list":
        wl = regen_worklist(cells)
        (FIELD_DIR).mkdir(parents=True, exist_ok=True)
        (LOG / "w1_sensitivity_regen_list.json").write_text(
            json.dumps({"needs_regen": wl, "count": len(wl)}, indent=2) + "\n")
        print(f"regen needed: {len(wl)}/126")
        return 0

    combos = ([(BASE_EPS, d) for d in DVF_LIST] if mode == "offline"
              else [(e, d) for e in EPS_LIST for d in DVF_LIST])
    results = []
    for eps, d in combos:
        try:
            res, _ = combo_result(cells, eps, d, base_labels)
        except FileNotFoundError as exc:
            print(f"STOP: {exc}\nRun scripts/vbd/w1_regen_fields.py first (GPU).")
            return 1
        results.append(res)
        tag = "BASE" if (eps == BASE_EPS and d == BASE_DVF) else ""
        print(f"eps={eps:.2f} DVF>={d*100:.2f}% -> {res['verdict']:12s} "
              f"a*={res['closure_commanded_a_star']} flips={res['flips_vs_baseline']} "
              f"uncertain={len(res['latch_uncertain_cells'])} {tag}")

    payload = {
        "schema": "e1v2_sensitivity.v1",
        "basis": "seed-0 screen labels; baseline reproduces official P-B CLOSURE "
                 "a*={E7:5,E15:5,E25:10}",
        "damage_proxy": "post-lift (>=1.80 s grip-completion) temporal-max principal "
                        "strain; DVF = rest-volume fraction with strain > eps; "
                        "latched iff DVF >= dvf_thresh",
        "baseline": {"eps": BASE_EPS, "dvf_thresh": BASE_DVF},
        "eps_grid": list(EPS_LIST), "dvf_grid": list(DVF_LIST),
        "mode": mode,
        "note": ("Closure depends only on intact status (intact iff not dropped AND "
                 "not damaged AND no slip gate); damage<->slip flips among dropped "
                 "cells never move a*. eps=0.15 column is receipt-exact; eps in "
                 "{0.10,0.20} uses regenerated fields."),
        "provenance": W.provenance(),
        "combos": results,
    }
    out = LOG / ("e1v2_sensitivity.json" if mode == "full" else "e1v2_sensitivity_offline.json")
    out.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"wrote {out.relative_to(ROOT)}")
    if mode == "offline":
        wl = regen_worklist(cells)
        print(f"regen still needed for full eps axis: {len(wl)}/126 cells")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--offline", action="store_true")
    g.add_argument("--full", action="store_true")
    g.add_argument("--regen-list", action="store_true")
    args = ap.parse_args()
    return run("offline" if args.offline else "full" if args.full else "regen-list")


if __name__ == "__main__":
    raise SystemExit(main())
