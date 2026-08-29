"""W2 geometry-only tactile proxy runner.

GPU simulation is entered only by ``--smoke`` and ``--grid``.  Pure reducers and
``--report`` are CPU-only.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from contextlib import contextmanager
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from scripts.vbd.w1_transport import run_transport_cell

WINDOW_START = 9.30
WINDOW_END = 11.60
UNAVAILABLE_REASON = "UNAVAILABLE: ATTR=GEOMETRY_ONLY, per-pad contact forces not attributable"
MATERIALS = (7, 15, 25)
ACCELS = (1, 2.5, 5, 10, 20, 30)
OUT = ROOT / "reports/logs/vbd/e2v2_tactile.json"
RAW_DIR = ROOT / "reports/logs/vbd/e2v2_tactile_raw"
FALSIFIER_OUT = ROOT / "reports/logs/vbd/e2v2_falsifier.json"


def unavailable_tangential_ratio():
    return {"value": None, "reason": UNAVAILABLE_REASON}


def centroid_excursion_mm(series):
    """Maximum pad-frame displacement from the first available centroid."""
    result = {}
    for pad in ("left", "right"):
        points = [frame["pads"][pad]["centroid_pad"]
                  for frame in series if frame["pads"][pad]["centroid_pad"] is not None]
        if not points:
            result[pad] = None
            continue
        reference = np.asarray(points[0], dtype=float)
        result[pad] = float(max(np.linalg.norm(np.asarray(point) - reference)
                                for point in points) * 1000.0)
    return result


def peak_lr_asymmetry(series):
    """Peak rigid-translation-invariant L-R geometry asymmetry."""
    counts = [frame["asymmetry"]["abs_count_difference"] for frame in series]
    offsets = [frame["asymmetry"]["centroid_y_offset_pad_m"] for frame in series
               if frame["asymmetry"]["centroid_y_offset_pad_m"] is not None]
    return {"abs_count_difference": max(counts) if counts else None,
            "centroid_y_offset_mm": max(offsets) * 1000.0 if offsets else None}


def material_summary(per_cell):
    """Build stable per-material centroid-excursion versus realized-accel rows."""
    output = {}
    for E in MATERIALS:
        rows = []
        for cell in per_cell:
            if cell.get("status", "ok") != "ok" or int(cell["E_kPa"]) != E:
                continue
            rows.append({"commanded_accel_m_s2": cell["a"],
                         "realized_accel_m_s2": cell.get("realized_accel"),
                         "centroid_excursion_mm": cell["centroid_excursion_mm"],
                         "peak_lr_asymmetry": cell["peak_lr_asymmetry"]})
        rows.sort(key=lambda row: row["commanded_accel_m_s2"])
        output[str(E)] = rows
    return output


def reduce_falsifier(cells, low_a=1.0, high_a=10.0):
    """Reduce endpoint replicates using each cell's peak excursion over both pads."""
    endpoints = {}
    for acceleration in (low_a, high_a):
        values = []
        for cell in cells:
            if float(cell["a"]) != acceleration:
                continue
            excursions = [value for value in cell["centroid_excursion_mm"].values()
                          if value is not None]
            if excursions:
                values.append(float(max(excursions)))
        values.sort()
        endpoints[f"{acceleration:g}"] = {
            "n": len(values), "centroid_excursion_mm": values,
            "range_mm": [values[0], values[-1]] if values else None,
            "median_mm": float(np.median(values)) if values else None,
        }
    low = endpoints[f"{low_a:g}"]
    high = endpoints[f"{high_a:g}"]
    complete = low["n"] == 3 and high["n"] == 3
    non_overlap = bool(
        complete and (high["range_mm"][0] > low["range_mm"][1]
                      or low["range_mm"][0] > high["range_mm"][1]))
    difference = (high["median_mm"] - low["median_mm"]) if complete else None
    return {"endpoints": endpoints, "strict_non_overlap": non_overlap,
            "signed_median_difference_mm_high_minus_low": difference,
            "peak_tangential_ratio": unavailable_tangential_ratio()}


def _contact_frame(rig, t):
    contacts = rig.contacts
    count = int(contacts.soft_contact_count.numpy().reshape(-1)[0])
    shapes = contacts.soft_contact_shape.numpy()
    indices = contacts.soft_contact_indices.numpy()
    bary = contacts.soft_contact_barycentric.numpy()
    q = rig.state_0.particle_q.numpy()
    body_q = rig.state_0.body_q.numpy()
    usable = min(max(count, 0), len(shapes))
    shape_body = rig.model.shape_body.numpy()
    pad_shapes = {
        "left": int(np.flatnonzero(shape_body == rig.b_left)[0]),
        "right": int(np.flatnonzero(shape_body == rig.b_right)[0]),
    }
    pads = {}
    pad_bodies = {"left": rig.b_left, "right": rig.b_right}
    for name, shape_id in pad_shapes.items():
        points = []
        for record in range(usable):
            if int(shapes[record]) != shape_id:
                continue
            ids = np.asarray(indices[record], dtype=int)
            valid = ids >= 0
            if not np.any(valid) or np.any(ids[valid] < rig.soft_start) or np.any(ids[valid] >= rig.soft_end):
                continue
            weights = np.asarray(bary[record], dtype=float)[valid]
            points.append(np.sum(q[ids[valid]] * weights[:, None], axis=0))
        if points:
            patch = np.asarray(points)
            centroid_world = np.mean(patch, axis=0)
            pad_position = np.asarray(body_q[pad_bodies[name]][:3], dtype=float)
            patch_pad = patch - pad_position
            centroid_pad = np.mean(patch_pad, axis=0)
            extent = float(np.linalg.norm(np.max(patch, axis=0) - np.min(patch, axis=0)))
            pads[name] = {"soft_contact_count": len(points),
                          "centroid_world": centroid_world.tolist(),
                          "centroid_pad": centroid_pad.tolist(), "extent_pad_m": extent,
                          "pad_world_position": pad_position.tolist()}
        else:
            pads[name] = {"soft_contact_count": 0, "centroid_world": None,
                          "centroid_pad": None, "extent_pad_m": None,
                          "pad_world_position":
                              np.asarray(body_q[pad_bodies[name]][:3], dtype=float).tolist()}
    lc, rc = pads["left"]["soft_contact_count"], pads["right"]["soft_contact_count"]
    ly, ry = pads["left"]["centroid_pad"], pads["right"]["centroid_pad"]
    centroid_y_offset = None if ly is None or ry is None else abs(float(ly[1]) - float(ry[1]))
    m = rig.metrics()
    return {"t": float(t), "pads": pads,
            "asymmetry": {"abs_count_difference": abs(lc - rc),
                           "centroid_y_offset_pad_m": centroid_y_offset},
            "palm": {"position": list(map(float, m["palm_pos"])),
                     "velocity_x_m_s": float(m["palm_vx"])}}


@contextmanager
def _capture_metrics(series):
    """Observe W1's per-frame metrics call without changing its frozen runner."""
    from src.vbd_rig2 import Vbd2Rig
    original = Vbd2Rig.metrics

    def wrapped(rig):
        m = original(rig)
        if WINDOW_START <= float(m["t"]) <= WINDOW_END:
            # Avoid recursively calling this wrapper in _contact_frame.
            Vbd2Rig.metrics = original
            try:
                series.append(_contact_frame(rig, m["t"]))
            finally:
                Vbd2Rig.metrics = wrapped
        return m

    Vbd2Rig.metrics = wrapped
    try:
        yield
    finally:
        Vbd2Rig.metrics = original


def run_tactile_cell(E, F, a, seed):
    series = []
    with _capture_metrics(series):
        receipt = run_transport_cell(float(E) * 1000.0, F, a, seed)
    tracking = receipt.get("tracking") or {}
    realized = tracking.get("realized_accel_m_s2")
    for frame in series:
        frame["realized_accel_m_s2"] = realized
    raw_path = _write_raw_npz(E, F, a, seed, series)
    return {"status": receipt.get("status", "ok"), "E_kPa": int(E), "F": float(F),
            "a": float(a), "seed": int(seed), "realized_accel": realized,
            "per_frame_series": series, "centroid_excursion_mm": centroid_excursion_mm(series),
            "peak_lr_asymmetry": peak_lr_asymmetry(series),
            "raw_npz": str(raw_path.relative_to(ROOT)),
            "peak_tangential_ratio": unavailable_tangential_ratio(),
            "git_sha": receipt.get("git_sha"), "prereg_sha256": receipt.get("prereg_sha256")}


def _error_cell(E, F, a, seed, exc):
    return {"status": "error", "E_kPa": E, "F": F, "a": a, "seed": seed,
            "realized_accel": None, "per_frame_series": [],
            "centroid_excursion_mm": {"left": None, "right": None},
            "peak_lr_asymmetry": {"abs_count_difference": None,
                                  "centroid_y_offset_mm": None},
            "peak_tangential_ratio": unavailable_tangential_ratio(),
            "error": f"{type(exc).__name__}: {exc}"}


def _write_raw_npz(E, F, a, seed, series):
    """Persist the exact JSON-compatible per-frame observations durably."""
    RAW_DIR.mkdir(parents=True, exist_ok=True)
    path = RAW_DIR / f"E{int(E)}_F{float(F):g}_a{float(a):g}_s{int(seed)}.npz"
    np.savez_compressed(path, per_frame_json=np.asarray(
        [json.dumps(frame, allow_nan=False) for frame in series]))
    return path


def _raw_path(E, F, a, seed):
    return RAW_DIR / f"E{int(E)}_F{float(F):g}_a{float(a):g}_s{int(seed)}.npz"


def _cell_from_raw(E, F, a, seed, path):
    with np.load(path, allow_pickle=False) as data:
        series = [json.loads(value) for value in data["per_frame_json"].tolist()]
    return {"status": "ok", "E_kPa": int(E), "F": float(F), "a": float(a),
            "seed": int(seed), "per_frame_series": series,
            "centroid_excursion_mm": centroid_excursion_mm(series),
            "raw_npz": str(path.relative_to(ROOT))}


def run_falsifier(resume=False):
    cells = []
    for a in (1, 10):
        for seed in (0, 1, 2):
            path = _raw_path(15, 1.2, a, seed)
            if resume and path.exists():
                cells.append(_cell_from_raw(15, 1.2, a, seed, path))
                continue
            cells.append(run_tactile_cell(15, 1.2, a, seed))
    result = {"schema": "e2v2_falsifier.v1", **reduce_falsifier(cells),
              "cells": [{"E_kPa": cell["E_kPa"], "F": cell["F"], "a": cell["a"],
                         "seed": cell["seed"], "centroid_excursion_mm":
                             cell["centroid_excursion_mm"],
                         "raw_npz": cell["raw_npz"]} for cell in cells],
              "raw_npz_files": [cell["raw_npz"] for cell in cells]}
    FALSIFIER_OUT.parent.mkdir(parents=True, exist_ok=True)
    FALSIFIER_OUT.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    return result


def run_grid():
    cells = []
    for E in MATERIALS:
        for a in ACCELS:
            try:
                cells.append(run_tactile_cell(E, 1.2, a, 0))
            except Exception as exc:
                cells.append(_error_cell(E, 1.2, a, 0, exc))
    good = next((cell for cell in cells if cell["status"] == "ok"), {})
    doc = {"schema": "e2v2_tactile.v1", "per_cell": cells,
           "summary": material_summary(cells),
           "provenance": {"git_sha": good.get("git_sha") or _git_sha(),
                          "prereg_sha256": good.get("prereg_sha256"),
                          "ATTR": "geometry_only"}}
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(doc, indent=2, allow_nan=False) + "\n")
    return doc


def _git_sha():
    return subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, check=True,
                          capture_output=True, text=True).stdout.strip()


def write_report():
    doc = json.loads(OUT.read_text())
    summary = doc.get("summary") or material_summary(doc["per_cell"])
    report = ["# W2 geometry-only tactile proxy", "", UNAVAILABLE_REASON, ""]
    overlay_dir = ROOT / "reports/logs/vbd"
    for E in MATERIALS:
        rows = summary.get(str(E), [])
        report += [f"## E={E} kPa", "", "| commanded a | realized a | left excursion (mm) | right excursion (mm) | peak count asym. | peak centroid-y asym. (mm) |",
                   "|---:|---:|---:|---:|---:|---:|"]
        for row in rows:
            excursion = row["centroid_excursion_mm"]
            asym = row["peak_lr_asymmetry"]
            report.append(f"| {row['commanded_accel_m_s2']:g} | {row['realized_accel_m_s2']} | {excursion['left']} | {excursion['right']} | {asym['abs_count_difference']} | {asym['centroid_y_offset_mm']} |")
        report.append("")
        (overlay_dir / f"e2v2_overlay_{E}.json").write_text(
            json.dumps({"E_kPa": E, "series": rows}, indent=2, allow_nan=False) + "\n")
    path = ROOT / "reports/vbd/w2_tactile.md"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(report) + "\n")
    return path


def main(argv=None):
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--smoke", action="store_true")
    mode.add_argument("--grid", action="store_true")
    mode.add_argument("--report", action="store_true")
    mode.add_argument("--falsifier", action="store_true")
    parser.add_argument("--resume", action="store_true",
                        help="with --falsifier, reuse existing per-cell raw NPZs")
    args = parser.parse_args(argv)
    if args.smoke:
        cell = run_tactile_cell(15, 1.2, 5, 0)
        print("per_frame_series length:", len(cell["per_frame_series"]))
        print("centroid_excursion_mm:", cell["centroid_excursion_mm"])
        print(cell["peak_tangential_ratio"]["reason"])
        finite = bool(cell["per_frame_series"]) and all(
            np.isfinite(frame["t"]) and np.all(np.isfinite(frame["palm"]["position"]))
            for frame in cell["per_frame_series"])
        return 0 if finite else 1
    if args.grid:
        run_grid()
    elif args.falsifier:
        run_falsifier(resume=args.resume)
    else:
        write_report()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
