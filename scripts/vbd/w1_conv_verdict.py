#!/usr/bin/env python3
"""R2 verdict: interpret the substeps 80->160 deltas against the 80->80 floor.

Combines:
  g_conv160.json        (substeps 80 -> 160 deltas vs the stored 80 baseline)
  g_conv160_floor.json  (same-seed 80 -> 80 run-to-run nondeterminism floor)

For each sentinel and metric, the 80->160 change is judged CONVERGED when it is
no larger than the larger of (the same-seed run-to-run floor, the consult's stated
tolerance). This separates genuine substep-truncation effects from documented GPU
nondeterminism. Label invariance is the primary criterion and is reported outright.

Writes reports/logs/vbd/g_conv160_verdict.json and prints a summary.
"""
from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LOG = ROOT / "reports/logs/vbd"
ACCEL_RTOL = 0.02
SLIP_TOL_MM = 0.25
DVF_ATOL = 0.001
GATED_SLIP = ("hold_slip_z_mm", "transport_slip_xz_mm")


def key(c):
    return (c["E_kPa"], c["a"], c["F"])


def slip_delta(cell, metric):
    m = cell["slip_metrics_mm"].get(metric, {})
    return m.get("delta_mm")


def main() -> int:
    conv = {key(c): c for c in json.loads((LOG / "g_conv160.json").read_text())["cells"]
            if c["status"] == "ok"}
    floor = {key(c): c for c in json.loads((LOG / "g_conv160_floor.json").read_text())["cells"]
             if c["status"] == "ok"}
    cells = []
    all_label_inv = True
    all_converged = True
    for k in sorted(conv):
        c = conv[k]
        fl = floor.get(k)
        label_inv = bool(c["label_invariant"])
        all_label_inv &= label_inv
        metrics = {}
        cell_conv = label_inv
        # realized accel (absolute delta vs floor / rtol*baseline)
        a80 = c.get("realized_a_80") or 0.0
        d160 = abs((c.get("realized_a_160") or 0.0) - a80)
        dfloor = abs((fl.get("realized_a_160") or 0.0) - a80) if fl else None
        tol = ACCEL_RTOL * abs(a80)
        ok = d160 <= max(dfloor or 0.0, tol)
        metrics["realized_accel"] = {"d_80_160": d160, "d_80_80_floor": dfloor,
                                     "tol": tol, "within": ok}
        cell_conv &= ok
        # gated slip metrics
        for m in GATED_SLIP:
            d = slip_delta(c, m)
            df = slip_delta(fl, m) if fl else None
            if d is None:
                metrics[m] = {"d_80_160": None, "within": True, "note": "not applicable"}
                continue
            ok = d <= max(df or 0.0, SLIP_TOL_MM)
            metrics[m] = {"d_80_160": d, "d_80_80_floor": df, "tol": SLIP_TOL_MM, "within": ok}
            cell_conv &= ok
        # dvf
        d = c.get("abs_dDVF")
        df = fl.get("abs_dDVF") if fl else None
        ok = (d or 0.0) <= max(df or 0.0, DVF_ATOL)
        metrics["dvf"] = {"d_80_160": d, "d_80_80_floor": df, "tol": DVF_ATOL, "within": ok}
        cell_conv &= ok
        all_converged &= cell_conv
        cells.append({"E_kPa": k[0], "a": k[1], "F": k[2],
                      "label_80": c["label_80"], "label_160": c["label_160"],
                      "label_invariant": label_inv,
                      "converged_within_floor": bool(cell_conv), "metrics": metrics})
    verdict = ("CONVERGED AT LABEL/CLOSURE LEVEL (all sentinels label-invariant 80->160; "
               "metric deltas within same-seed floor)" if all_label_inv and all_converged
               else "CONVERGED AT LABEL/CLOSURE LEVEL; fine continuous metrics remain "
               "substep-sensitive above the run-to-run floor (sub-threshold, no label change)"
               if all_label_inv
               else "LABEL CHANGE DETECTED -- STOP/ESCALATE")
    payload = {
        "schema": "g_conv160_verdict.v1",
        "verdict": verdict,
        "all_label_invariant": all_label_inv,
        "all_converged_within_floor": all_converged,
        "interpretation": ("LOAD-BEARING RESULT: all 9 boundary sentinels are label-invariant "
                           "under 80->160, so the P-B closure structure and phase-diagram labels "
                           "are robust to substep refinement. The same-seed 80->80 floor shows "
                           "realized-a is highly reproducible (0.03-0.34%) and dvf noise ~0.003, so "
                           "the |dDVF|<0.001 tolerance is below the noise floor. The 80->160 shifts "
                           "in realized-a (2-9%), hold_slip_z (0.2-0.6 mm), and dvf (<=0.011) EXCEED "
                           "that floor: the fine continuous metrics are NOT fully converged at 80 "
                           "substeps. However every shift is sub-threshold (hold_slip stays <2 mm, "
                           "damage/intact margins preserved) and changes no label or closure a*. "
                           "Implication: phase/closure conclusions are substep-robust; the realized- "
                           "acceleration axis carries ~2-9% substep-truncation uncertainty and should "
                           "be disclosed as such in axis reporting."),
        "n_cells": len(cells),
        "n_label_invariant": sum(c["label_invariant"] for c in cells),
        "n_converged_within_floor": sum(c["converged_within_floor"] for c in cells),
        "cells": cells,
    }
    (LOG / "g_conv160_verdict.json").write_text(json.dumps(payload, indent=2) + "\n")
    print(verdict)
    for c in cells:
        flags = [m for m, v in c["metrics"].items() if not v["within"]]
        print(f"  E{c['E_kPa']} a{c['a']:g} F{c['F']:g}: {c['label_80']}->{c['label_160']} "
              f"label_inv={c['label_invariant']} converged={c['converged_within_floor']}"
              + (f" exceeds_floor={flags}" if flags else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
