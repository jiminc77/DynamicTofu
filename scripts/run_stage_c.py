"""Stage C: boundary densification to 5 seeds (pre-registered extra_replications).

Densifies the σ=2000 non-unanimous drop/damage boundary cells (F=0.8,
a∈{2.5,5,10,15}) from 3 to 5 seeds by running seeds 3 and 4. These are
EXTRA REPLICATIONS: they never enter the 360-coordinate universe; they live
in a separate `extra_replications` block. The ≥4/5 fraction rule
(required_pass(5)=4) determines the sharpened cell attribution.

Writes seed-3/4 trials to ralph/results/extra_replications/ and folds an
extra_replications block into ralph/results/e1_band_2000.json.

Usage: cd newton && uv run --no-sync python ../scripts/run_stage_c.py
"""

from __future__ import annotations

import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src import io_schemas

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SIGMA = 2000.0
EXTRA_SEEDS = [3, 4]
CAL = {"slope": 0.99984, "intercept": 0.00027, "residual": 0.00034, "hysteresis": 0.0254,
       "status": "P4 pre-saturation acceptance (approved)"}


def boundary_cells():
    return [tuple(c) for c in json.load(open(os.path.join(ROOT, "reports", "logs", "stageC-boundary-cells.json")))]


def main() -> int:
    from src.trial import run_trial

    cells = boundary_cells()
    print(f"Stage C: {len(cells)} σ=2000 boundary cells × {len(EXTRA_SEEDS)} extra seeds = {len(cells)*len(EXTRA_SEEDS)} trials")
    extra_dir = os.path.join(ROOT, "ralph", "results", "extra_replications")
    os.makedirs(extra_dir, exist_ok=True)

    for (a, f) in cells:
        for seed in EXTRA_SEEDS:
            key = f"s{int(SIGMA)}_a{a:g}_f{f:g}_seed{seed}"
            out = os.path.join(extra_dir, f"{key}.json")
            if os.path.exists(out):
                continue
            doc = run_trial(SIGMA, a, f, seed, calibration=CAL, out_json=out,
                            extra_config={"replication_class": "extra_seed_boundary_densification_NOT_in_360_universe"})
            p = doc["payload"]
            print(f"[{key}] {p['labels']} color={p['cell_color']} frac={p['peak_damage_fraction']:.3f} "
                  f"drop_in_window={p['drop_evidence_in_window']}")

    # --- fold extra_replications into the sigma=2000 band -------------------
    extra = {}
    for (a, f) in cells:
        colors = {}
        for seed in range(5):
            if seed < 3:
                pth = os.path.join(ROOT, "ralph", "results", "trials", f"s{int(SIGMA)}_a{a:g}_f{f:g}_seed{seed}.json")
            else:
                pth = os.path.join(extra_dir, f"s{int(SIGMA)}_a{a:g}_f{f:g}_seed{seed}.json")
            if os.path.exists(pth):
                colors[seed] = io_schemas.read_json(pth)["payload"]["cell_color"]
        n = len(colors)
        req = math.ceil(2 * n / 3)  # >=2/3 at 3, >=4/5 at 5 (same formula, both anchors)
        counts = {}
        for c in colors.values():
            counts[c] = counts.get(c, 0) + 1
        # sharpened attribution: a color certified iff its count >= req
        certified = next((c for c, k in counts.items() if k >= req), None)
        extra[f"a{a:g}_f{f:g}"] = {
            "n_planned": 5, "n_resolved": n, "required_pass_4of5": req,
            "seed_colors": colors, "color_counts": counts,
            "certified_color": certified,
            "certified": certified is not None,
            "note": "boundary sharpened; still no intact -> band remains empty; this refines drop<->damage attribution only",
        }
        print(f"  cell a={a} F={f}: 5-seed {counts} -> required {req}/5 -> certified={certified}")

    band_path = os.path.join(ROOT, "ralph", "results", "e1_band_2000.json")
    doc = io_schemas.read_json(band_path)
    doc["payload"]["extra_replications"] = extra
    io_schemas.write_json(band_path, doc)
    json.dump(extra, open(os.path.join(ROOT, "reports", "logs", "stageC-summary.json"), "w"), indent=2)
    print(f"folded {len(extra)} extra-replication cells into {band_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
