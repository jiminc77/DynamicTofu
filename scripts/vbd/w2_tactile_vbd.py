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


def unavailable_tangential_ratio():
    return {"value": None, "reason": UNAVAILABLE_REASON}


def centroid_excursion_mm(series):
    """Maximum per-pad displacement from its first available window centroid."""
    result = {}
    for pad in ("left", "right"):
        points = [(frame["t"], frame["pads"][pad]["centroid"])
                  for frame in series if frame["pads"][pad]["centroid"] is not None]
        if not points:
            result[pad] = None
            continue
        reference = np.asarray(points[0][1], dtype=float)
        result[pad] = float(max(np.linalg.norm(np.asarray(point) - reference)
                                for _, point in points) * 1000.0)
    return result


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
                         "centroid_excursion_mm": cell["centroid_excursion_mm"]})
        rows.sort(key=lambda row: row["commanded_accel_m_s2"])
        output[str(E)] = rows
    return output


def _contact_frame(rig, t):
    contacts = rig.contacts
    count = int(contacts.soft_contact_count.numpy().reshape(-1)[0])
    shapes = contacts.soft_contact_shape.numpy()
    indices = contacts.soft_contact_indices.numpy()
    bary = contacts.soft_contact_barycentric.numpy()
    q = rig.state_0.particle_q.numpy()
    usable = min(max(count, 0), len(shapes))
    shape_body = rig.model.shape_body.numpy()
    pad_shapes = {
        "left": int(np.flatnonzero(shape_body == rig.b_left)[0]),
        "right": int(np.flatnonzero(shape_body == rig.b_right)[0]),
    }
    pads = {}
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
            centroid = np.mean(patch, axis=0)
            extent = float(np.linalg.norm(np.max(patch, axis=0) - np.min(patch, axis=0)))
            pads[name] = {"soft_contact_count": len(points), "centroid": centroid.tolist(),
                          "extent_m": extent}
        else:
            pads[name] = {"soft_contact_count": 0, "centroid": None, "extent_m": None}
    lc, rc = pads["left"]["soft_contact_count"], pads["right"]["soft_contact_count"]
    ly, ry = pads["left"]["centroid"], pads["right"]["centroid"]
    centroid_y_offset = None if ly is None or ry is None else abs(float(ly[1]) - float(ry[1]))
    m = rig.metrics()
    return {"t": float(t), "pads": pads,
            "asymmetry": {"abs_count_difference": abs(lc - rc),
                           "centroid_y_offset_m": centroid_y_offset},
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
    return {"status": receipt.get("status", "ok"), "E_kPa": int(E), "F": float(F),
            "a": float(a), "seed": int(seed), "realized_accel": realized,
            "per_frame_series": series, "centroid_excursion_mm": centroid_excursion_mm(series),
            "peak_tangential_ratio": unavailable_tangential_ratio(),
            "git_sha": receipt.get("git_sha"), "prereg_sha256": receipt.get("prereg_sha256")}


def _error_cell(E, F, a, seed, exc):
    return {"status": "error", "E_kPa": E, "F": F, "a": a, "seed": seed,
            "realized_accel": None, "per_frame_series": [],
            "centroid_excursion_mm": {"left": None, "right": None},
            "peak_tangential_ratio": unavailable_tangential_ratio(),
            "error": f"{type(exc).__name__}: {exc}"}


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
        report += [f"## E={E} kPa", "", "| commanded a | realized a | left excursion (mm) | right excursion (mm) |",
                   "|---:|---:|---:|---:|"]
        for row in rows:
            excursion = row["centroid_excursion_mm"]
            report.append(f"| {row['commanded_accel_m_s2']:g} | {row['realized_accel_m_s2']} | {excursion['left']} | {excursion['right']} |")
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
    else:
        write_report()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
