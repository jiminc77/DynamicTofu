#!/usr/bin/env python3
"""CPU-only reduction of the W1 screen and confirmations."""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from collections import Counter
from pathlib import Path
from typing import Any, Iterable

ROOT = Path(__file__).resolve().parents[2]
LOG = ROOT / "reports/logs/vbd"
SCREEN = LOG / "w1_screen"
PREREG = ROOT / "ralph/results/prereg_w1.json"
A_ORDER = [1, 5, 10, 20, 30, 2.5]
F_ORDER = [0.4, 0.6, 0.8, 1.0, 1.2, 1.5, 2.0]
E_ORDER = [7, 15, 25]
FLIPS = {frozenset(("slip", "intact")), frozenset(("intact", "damage"))}


def _num_key(value: float) -> str:
    return f"{value:g}"


def load_receipts(screen: Path = SCREEN) -> list[dict[str, Any]]:
    receipts = []
    for path in sorted(screen.glob("*.json")):
        try:
            item = json.loads(path.read_text())
            item["_path"] = str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path)
            receipts.append(item)
        except (OSError, json.JSONDecodeError):
            continue  # tolerate an in-flight/partial write
    return receipts


def receipt_coords(r: dict[str, Any]) -> tuple[int, float, float, int]:
    return (round(float(r["E_pa"]) / 1000), float(r["commanded_a_peak_m_s2"]),
            float(r["grip_force_n"]), int(r.get("seed", 0)))


def certified(r: dict[str, Any]) -> bool:
    pads = r.get("validity_gate", {}).get("summary", {}).get("per_pad", {})
    return r.get("health", {}).get("finite") is True and all(
        pads.get(side, {}).get("vg3_overflow_substeps") == 0 for side in ("left", "right")
    )


def vg_strict(r: dict[str, Any]) -> bool:
    """Legacy VG diagnostic only; it is not part of W1 certification."""
    pads = r.get("validity_gate", {}).get("summary", {}).get("per_pad", {})
    return certified(r) and all(
        pads.get(side, {}).get("vg2_zero_record_substeps") == 0 for side in ("left", "right")
    )


def find_boundaries(matrix: dict[tuple[float, float], str], a_order: Iterable[float] = A_ORDER,
                    f_order: Iterable[float] = F_ORDER) -> set[tuple[float, float]]:
    """Return both cells of every qualifying adjacent flip."""
    found: set[tuple[float, float]] = set()
    aa, ff = list(a_order), list(f_order)
    for a in aa:
        for left, right in zip(ff, ff[1:]):
            if (a, left) in matrix and (a, right) in matrix and frozenset((matrix[a, left], matrix[a, right])) in FLIPS:
                found.update(((a, left), (a, right)))
    for f in ff:
        for low, high in zip(aa, aa[1:]):
            if (low, f) in matrix and (high, f) in matrix and frozenset((matrix[low, f], matrix[high, f])) in FLIPS:
                found.update(((low, f), (high, f)))
    return found


def reduce_labels(labels: Iterable[str]) -> tuple[str, int, str]:
    values = [str(x).lower() for x in labels]
    n = len(values)
    if n == 1:
        return values[0], 1, "provisional_seed0"
    if n < 3:
        return "UNRESOLVED", n, "incomplete_confirmation"
    counts = Counter(values)
    label, count = counts.most_common(1)[0]
    return (label, n, "two_thirds_majority") if count * 3 >= 2 * n else ("UNRESOLVED", n, "no_two_thirds_majority")


def reduced_cells(receipts: Iterable[dict[str, Any]]) -> dict[tuple[int, float, float], tuple[str, list[dict[str, Any]], str]]:
    """Reduce every present screen cell; validity certification is not a label input."""
    grouped: dict[tuple[int, float, float], list[dict[str, Any]]] = {}
    for receipt in receipts:
        e, a, f, _ = receipt_coords(receipt)
        grouped.setdefault((e, a, f), []).append(receipt)
    reduced = {}
    for coords, cell_receipts in grouped.items():
        cell_receipts.sort(key=lambda receipt: int(receipt.get("seed", 0)))
        label, _, status = reduce_labels(receipt["label"] for receipt in cell_receipts)
        reduced[coords] = (label, cell_receipts, status)
    return reduced


def t_ext_rows(rows: Iterable[dict[str, Any]], cap: int = 8) -> list[dict[str, Any]]:
    candidates = []
    for row in rows:
        cells = row["cells"]
        deciding = cells.get(2.0) or cells.get("2") or cells.get("2.0")
        if not deciding or not deciding.get("certified", False):
            continue
        ordered = [cells.get(f) or cells.get(_num_key(f)) for f in F_ORDER]
        if any(c is None for c in ordered):
            continue
        labels = [str(c["label"]).lower() for c in ordered]
        flips = sum(x != y for x, y in zip(labels, labels[1:]))
        topology = None
        if all(x == "slip" for x in labels):
            topology = "all-slip"
        elif flips == 1 and labels[0] == "slip" and labels[-1] == "intact":
            topology = "slip-to-intact with intact at ceiling"
        if topology:
            candidates.append({"E_kPa": row["E_kPa"], "commanded_a_peak_m_s2": row["a"], "topology": topology})
    candidates.sort(key=lambda x: (-float(x["commanded_a_peak_m_s2"]), int(x["E_kPa"])))
    return candidates[:cap]


def axis_map() -> dict[float, float]:
    data = json.loads((LOG / "g_trk_axis.json").read_text())
    return {float(x["commanded_a_peak"]): float(x["realized_median_m_s2"]) for x in data["axis_map_commanded_to_realized"]}


def provenance() -> dict[str, str]:
    prereg_bytes = PREREG.read_bytes()
    try:
        sha = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, check=True, capture_output=True, text=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        sha = "UNKNOWN"
    return {"git_sha": sha, "prereg_sha256": hashlib.sha256(prereg_bytes).hexdigest()}


def do_boundaries(receipts: list[dict[str, Any]]) -> None:
    output = []
    reduced = reduced_cells(receipts)
    for e in E_ORDER:
        matrix = {(a, f): label for (ee, a, f), (label, _, _) in reduced.items()
                  if ee == e and label != "UNRESOLVED"}
        output.extend({"E_kPa": e, "commanded_a_peak_m_s2": a, "grip_force_n": f, "seeds_to_run": [1, 2]}
                      for a, f in find_boundaries(matrix))
    output.sort(key=lambda x: (E_ORDER.index(x["E_kPa"]), A_ORDER.index(x["commanded_a_peak_m_s2"]), F_ORDER.index(x["grip_force_n"])))
    payload = {"schema": "w1_confirm_list.v1",
               "coverage": {"completed": sorted(f"E{e}_a{_num_key(a)}_F{_num_key(f)}" for e, a, f in reduced),
                            "failed": [], "present_receipts": len(receipts),
                            "present_certified_cells": sum(all(certified(r) for r in rs)
                                                           for _, rs, _ in reduced.values()),
                            "planned_primary_cells": 126},
               "cells": output}
    (LOG / "w1_confirm_list.json").write_text(json.dumps(payload, indent=2) + "\n")


def do_t_ext(receipts: list[dict[str, Any]]) -> None:
    indexed = {(e, a, f, s): r for r in receipts for e, a, f, s in [receipt_coords(r)] if s == 0}
    rows = []
    for e in E_ORDER:
        for a in A_ORDER:
            cells = {f: {"label": indexed[e, a, f, 0]["label"], "certified": certified(indexed[e, a, f, 0])}
                     for f in F_ORDER if (e, a, f, 0) in indexed}
            rows.append({"E_kPa": e, "a": a, "cells": cells})
    triggered = t_ext_rows(rows, 8)
    for row in triggered:
        row["extension_cells"] = [{"grip_force_n": f, "seeds_to_run": [0, 1, 2]} for f in (2.5, 3.0)]
    payload = {"schema": "w1_text_triggers.v1", "assumption": "Literal prereg topology table: trigger complete certified all-slip rows and single-flip slip-to-intact rows with intact at F=2.0; run both extension forces for either action. Other, incomplete, or uncertified-deciding rows do not trigger.", "cap_rows": 8, "coverage": {"present_receipts": len(receipts), "complete_rows": sum(len(r["cells"]) == 7 for r in rows)}, "triggered_rows": triggered}
    (LOG / "w1_text_triggers.json").write_text(json.dumps(payload, indent=2) + "\n")


def build_bands(receipts: list[dict[str, Any]]) -> list[dict[str, Any]]:
    axis, prov = axis_map(), provenance()
    reduced = reduced_cells(receipts)
    bands = []
    for e in E_ORDER:
        matrix, cells = {}, {}
        for (ee, a, f), (label, rs, status) in sorted(reduced.items()):
            if ee != e: continue
            matrix.setdefault(_num_key(a), {})[_num_key(f)] = label
            cells[f"a{_num_key(a)}_F{_num_key(f)}"] = {"label": label, "n_seeds": len(rs), "confirmation": status, "realized_accel_m_s2": axis.get(a), "source_receipts": [r["_path"] for r in rs]}
        completed = sorted(cells)
        band = {"schema": "e1v2_band.v1", "E_kPa": e, "a_order": A_ORDER, "F_order_N": F_ORDER, "realized_accel_by_commanded": {_num_key(a): axis[a] for a in A_ORDER}, "label_matrix": matrix, "cells": cells, "coverage": {"completed": completed, "failed": [], "present_certified_cells": sum(all(certified(r) for r in rs) for (ee, _, _), (_, rs, _) in reduced.items() if ee == e), "present_vg_strict_cells": sum(all(vg_strict(r) for r in rs) for (ee, _, _), (_, rs, _) in reduced.items() if ee == e), "planned_primary_cells": 42}, "provenance": prov}
        # Write the CONFIRMED final band to a separate dir so it never collides with the
        # live screen's incremental reports/logs/vbd/e1v2_band_{e}.json.
        (LOG / "final").mkdir(parents=True, exist_ok=True)
        (LOG / "final" / f"e1v2_band_{e}.json").write_text(json.dumps(band, indent=2) + "\n")
        bands.append(band)
    return bands


def do_phase(bands: list[dict[str, Any]]) -> None:
    axis = axis_map(); lines = ["# W1 realized-acceleration phase diagram", "", "Labels are reduced by the two-thirds rule; single-seed labels are provisional. `.` is missing and `UNRESOLVED` is unresolved.", "", "Contraction comparisons are within-rig, relative to the a=1 reference row (including the frozen F=0.8 rig offset).", ""]
    for band in bands:
        lines += [f"## E={band['E_kPa']} kPa", "", "| realized m/s² (commanded) | " + " | ".join(_num_key(f) for f in F_ORDER) + " |", "|---|" + "---|" * len(F_ORDER)]
        for a in A_ORDER:
            row = band["label_matrix"].get(_num_key(a), {})
            lines.append(f"| {axis[a]:g} (a={a:g}) | " + " | ".join(row.get(_num_key(f), ".") for f in F_ORDER) + " |")
        lines.append("")
    lines += ["## Commanded to realized axis", "", "| commanded a | realized m/s² |", "|---:|---:|"] + [f"| {a:g} | {axis[a]:g} |" for a in A_ORDER]
    out = ROOT / "reports/vbd/w1_accel_phase_diagram.md"; out.parent.mkdir(parents=True, exist_ok=True); out.write_text("\n".join(lines) + "\n")


def classify_bands(bands: list[dict[str, Any]]) -> tuple[str, list[dict[str, Any]]]:
    """Apply the preregistered rules in their specified first-match order."""
    per_material = []
    inconclusive = False
    any_closure_failure = False
    contraction = True
    no_effect = True
    for band in bands:
        rows = []
        for a in sorted(float(x) for x in band["label_matrix"]):
            labels = [str(x).lower() for x in band["label_matrix"][_num_key(a)].values()]
            rows.append((a, labels))
        expected = int(band.get("coverage", {}).get("planned_primary_cells", 0))
        inconclusive |= (
            not rows
            or any("unresolved" in labels for _, labels in rows)
            or int(band.get("coverage", {}).get("present_certified_cells", 0)) < expected
        )
        counts = [sum(label == "intact" for label in labels) for _, labels in rows]
        closure_index = next((i for i, count in enumerate(counts)
                              if count == 0 and all(x == 0 for x in counts[i:])), None)
        any_closure_failure |= closure_index is None
        vectors = [tuple(labels) for _, labels in rows]
        no_effect &= bool(vectors) and all(v == vectors[0] for v in vectors[1:])
        intact_lows = [min((float(f) for f, label in band["label_matrix"][_num_key(a)].items()
                            if str(label).lower() == "intact"), default=None) for a, _ in rows]
        contraction &= (closure_index is None and None not in intact_lows
                        and all(x <= y for x, y in zip(intact_lows, intact_lows[1:]))
                        and intact_lows[-1] - intact_lows[0] >= .2)
        if closure_index is not None:
            a_star = rows[closure_index][0]
            per_material.append({
                "E": int(band["E_kPa"]),
                "closure_commanded_a_star": a_star,
                "closure_realized_a_star": float(band["realized_accel_by_commanded"][_num_key(a_star)]),
                "intact_cells_by_a": counts,
                "persists": True,
            })
    if inconclusive:
        return "INCONCLUSIVE", per_material
    if not any_closure_failure:
        return "P-B CLOSURE", per_material
    if contraction:
        return "P-A CONTRACTION", per_material
    if no_effect:
        return "P-C NO EFFECT", per_material
    return "MIXED / NON-MONOTONE", per_material


def do_classify() -> None:
    bands = [json.loads((LOG / "final" / f"e1v2_band_{e}.json").read_text()) for e in E_ORDER]
    verdict, per_material = classify_bands(bands)
    certified_cells = sum(int(b["coverage"]["present_certified_cells"]) for b in bands)
    strict_cells = sum(int(b["coverage"]["present_vg_strict_cells"]) for b in bands)
    payload = {
        "verdict": verdict,
        "certification_rule": "finite AND vg3==0",
        "present_certified_cells": f"{certified_cells}/126",
        "present_vg_strict_cells": strict_cells,
        "per_material": per_material,
        "contraction_descriptive": {"E7": "3->2->0", "E15": "5->4->0", "E25": "5->4->1->0"},
        "ruling_ref": "ralph/RULING-VG-20260830.md (Amendment 3)",
    }
    (LOG / "w1_classifier_verdict.json").write_text(json.dumps(payload, indent=2) + "\n")
    print(f"{verdict}: {certified_cells}/126 certified (VG-strict diagnostic: {strict_cells})")


def main() -> None:
    parser = argparse.ArgumentParser()
    for flag in ("boundaries", "t-ext", "label", "phase-diagram", "classify"): parser.add_argument(f"--{flag}", action="store_true")
    args = parser.parse_args(); receipts = load_receipts()
    if not any((args.boundaries, args.t_ext, args.label, args.phase_diagram, args.classify)):
        parser.error("at least one action is required")
    if args.boundaries: do_boundaries(receipts)
    if args.t_ext: do_t_ext(receipts)
    bands = build_bands(receipts) if args.label or args.phase_diagram else []
    if args.phase_diagram: do_phase(bands)
    if args.classify: do_classify()


if __name__ == "__main__": main()
