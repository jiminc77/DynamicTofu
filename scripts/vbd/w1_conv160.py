#!/usr/bin/env python3
"""R2: substeps=160 temporal-convergence sentinels (external consult #3, red-team #2).

Re-runs a set of boundary-adjacent seed-0 cells spanning the E7/E15/E25 slip AND
damage boundaries at substeps=160 (double the frozen 80) with the frozen config
otherwise, and checks convergence against the stored 80-substep screen receipts.

Acceptance per cell (consult #3 R2):
  * label invariant           label(160) == label(80)
  * realized-a within 2%      |a160 - a80| / a80 < 0.02
  * slip residual delta       |slip(160) - slip(80)| < 0.25 mm on shared metrics
  * |dDVF| < 0.001            |dvf160 - dvf80| < 0.001

Sentinels (9) -- each sits on a label boundary; includes the closure-critical
intact cells and the DVF-sensitive E15 a=5/F1.2 marginal damage cell:
  E7 : (1,1.0)I  (2.5,1.0)I  (5,1.2)D
  E15: (1,0.8)I  (2.5,1.2)I  (5,1.2)D
  E25: (1,0.8)I  (5,2.0)I   (10,2.0)D

Run (GPU):
  cd newton && PYTHONPATH=/home/simx2204/Workspace/DynamicTofu \
    uv run --no-sync python /home/simx2204/Workspace/DynamicTofu/scripts/vbd/w1_conv160.py
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
OUT = LOG / "g_conv160.json"

SENTINELS = [
    (7, 1.0, 1.0), (7, 2.5, 1.0), (7, 5.0, 1.2),
    (15, 1.0, 0.8), (15, 2.5, 1.2), (15, 5.0, 1.2),
    (25, 1.0, 0.8), (25, 5.0, 2.0), (25, 10.0, 2.0),
]
SLIP_METRICS = ("hold_slip_z_mm", "transport_slip_xz_mm", "slip3d_max_mm")
SLIP_TOL_MM = 0.25
ACCEL_RTOL = 0.02
DVF_ATOL = 0.001


def screen_name(e, a, f):
    return f"E{e}_F{f:g}_a{a:g}_s0.json"


def load_baseline(e, a, f):
    return json.loads((SCREEN / screen_name(e, a, f)).read_text())


def _realized(rec):
    """Realized accel: top-level (screen wrapper) or nested tracking (raw runner)."""
    if rec.get("realized_accel_m_s2") is not None:
        return rec["realized_accel_m_s2"]
    tracking = rec.get("tracking") or {}
    return tracking.get("realized_accel_m_s2")


def compare(base, conv):
    label_ok = base["label"] == conv["label"]
    a0, a1 = _realized(base), _realized(conv)
    accel_rel = (abs(a1 - a0) / abs(a0)) if (a0 and a1 and a0 != 0) else None
    accel_ok = accel_rel is not None and accel_rel < ACCEL_RTOL
    slip = {}
    slip_ok = True
    for m in SLIP_METRICS:
        b, c = base.get(m), conv.get(m)
        if b is None or c is None:
            slip[m] = {"base": b, "conv": c, "delta_mm": None}
            continue
        d = abs(float(c) - float(b))
        slip[m] = {"base": float(b), "conv": float(c), "delta_mm": d}
        if m in ("hold_slip_z_mm", "transport_slip_xz_mm"):
            slip_ok &= d < SLIP_TOL_MM
    dvf0, dvf1 = float(base["dvf"]), float(conv["dvf"])
    ddvf = abs(dvf1 - dvf0)
    dvf_ok = ddvf < DVF_ATOL
    cell_pass = bool(label_ok and accel_ok and slip_ok and dvf_ok)
    return {
        "label_80": base["label"], "label_160": conv["label"], "label_invariant": label_ok,
        "realized_a_80": a0, "realized_a_160": a1, "accel_rel": accel_rel,
        "accel_within_2pct": accel_ok,
        "slip_metrics_mm": slip, "slip_delta_ok": slip_ok,
        "dvf_80": dvf0, "dvf_160": dvf1, "abs_dDVF": ddvf, "dvf_delta_ok": dvf_ok,
        "ejected_80": base.get("ejected"), "ejected_160": conv.get("ejected"),
        "pass": cell_pass,
    }


def main() -> int:
    results = []
    if OUT.exists():
        try:
            results = json.loads(OUT.read_text()).get("cells", [])
        except (OSError, json.JSONDecodeError):
            results = []
    done = {(r["E_kPa"], r["a"], r["F"]) for r in results}
    started = time.monotonic()
    for e, a, f in SENTINELS:
        if (e, a, f) in done:
            continue
        t0 = time.monotonic()
        try:
            conv = run_transport_cell(e * 1000.0, f, a, 0, substeps=160, convergence=True)
            raw_dir = LOG / "g_conv160_raw"
            raw_dir.mkdir(parents=True, exist_ok=True)
            (raw_dir / f"E{e}_a{a:g}_F{f:g}_s160.json").write_text(
                json.dumps(_json_safe(conv), indent=2, allow_nan=False) + "\n")
            base = load_baseline(e, a, f)
            entry = {"E_kPa": e, "a": a, "F": f, "status": "ok", **compare(base, conv),
                     "wall_s": round(time.monotonic() - t0, 1)}
        except Exception as exc:  # noqa: BLE001
            entry = {"E_kPa": e, "a": a, "F": f, "status": "error",
                     "error": repr(exc)[:300], "wall_s": round(time.monotonic() - t0, 1)}
        results = [r for r in results if (r["E_kPa"], r["a"], r["F"]) != (e, a, f)] + [entry]
        ok_cells = [r for r in results if r["status"] == "ok"]
        payload = {
            "schema": "g_conv160.v1",
            "purpose": "temporal convergence: substeps 80 -> 160 on slip+damage "
                       "boundary sentinels; frozen config otherwise (seed 0).",
            "acceptance": {"label_invariant": True, "realized_accel_rtol": ACCEL_RTOL,
                           "slip_residual_tol_mm": SLIP_TOL_MM, "dvf_atol": DVF_ATOL},
            "n_cells": len(SENTINELS), "n_done": len(ok_cells),
            "n_pass": sum(r.get("pass", False) for r in ok_cells),
            "all_pass": len(ok_cells) == len(SENTINELS) and all(r.get("pass") for r in ok_cells),
            "cells": results,
        }
        OUT.write_text(json.dumps(payload, indent=2) + "\n")
        st = "PASS" if entry.get("pass") else ("ERR" if entry["status"] == "error" else "FAIL")
        print(f"E{e} a{a:g} F{f:g}: {st} label {entry.get('label_80')}->{entry.get('label_160')} "
              f"a_rel={entry.get('accel_rel')} dDVF={entry.get('abs_dDVF')} "
              f"{entry['wall_s']}s (elapsed {(time.monotonic()-started)/60:.1f}m)", flush=True)
    print("DONE", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
