"""Aggregate per-trial e1.v1 JSONs -> e1_band.v1 per material + coverage map.

- Band estimator: planned-seed denominator, required_pass = ceil(2n/3),
  certification, contiguity/censoring, a_star (src.bands - frozen).
- Coverage: the exact unique 360-coordinate formal universe; every coordinate
  gets {status, reason}; done coordinates cross-link their trial artifact.
- Stage-A output also runs the pre-registered shape-checkpoint router.

Usage:
  newton/.venv/bin/python scripts/aggregate_bands.py [--results ralph/results] [--sigma 3333]
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src import bands, io_schemas

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SIGMAS = [2000.0, 3333.0, 6000.0]
ACCELS = [1.0, 2.5, 5.0, 10.0, 15.0]
GRIPS_FULL = [0.3, 0.5, 0.8, 1.2, 1.8, 2.5, 3.5, 5.0]
SEEDS = [0, 1, 2]


def load_trials(results_dir):
    trials = {}
    for path in glob.glob(os.path.join(results_dir, "trials", "*.json")):
        doc = io_schemas.read_json(path)
        p = doc["payload"]
        key = (float(p["sigma_y_pa"]), float(p["a_peak_cmd_ms2"]), float(p["f_g_n"]), int(p["seed"]))
        trials[key] = {"labels": p["labels"], "artifact": os.path.relpath(path, ROOT),
                       "healthy": p["health"]["clean"]}
    return trials


def outcome_of(trial):
    if trial is None or not trial["healthy"]:
        return None  # unresolved / invalid
    labels = set(trial["labels"])
    labels.discard("intact")
    return labels


def aggregate_material(sigma, trials, state, config):
    per_accel = {}
    for a in ACCELS:
        cells = {}
        for f in GRIPS_FULL:
            outs = [outcome_of(trials.get((sigma, a, f, s))) for s in SEEDS]
            attempted = any((sigma, a, f, s) in trials for s in SEEDS)
            if attempted:
                cells[f] = bands.reduce_cell(outs, n_planned=len(SEEDS))
        if cells:
            per_accel[a] = bands.estimate_band(cells)
    a_star, a_star_status = bands.estimate_a_star(per_accel) if per_accel else (None, "no_vanishing_observed")

    rows = []
    for a in ACCELS:
        b = per_accel.get(a, {"F_min": None, "F_max": None, "band_width_n": None,
                              "band_status": "empty", "censored_low": False,
                              "censored_high": False, "interior_failures": []})
        rows.append({"sigma_Y": sigma, "a_peak": a, "F_min": b["F_min"], "F_max": b["F_max"],
                     "band_width_n": b["band_width_n"], "band_status": b["band_status"],
                     "censored_low": b["censored_low"], "censored_high": b["censored_high"],
                     "interior_failures": b["interior_failures"]})

    coverage = {}
    for a in ACCELS:
        for f in GRIPS_FULL:
            for s in SEEDS:
                key = f"s{int(sigma)}_a{a:g}_f{f:g}_seed{s}"
                t = trials.get((sigma, a, f, s))
                if t is not None and t["healthy"]:
                    coverage[key] = {"status": "done", "reason": t["artifact"]}
                elif t is not None:
                    coverage[key] = {"status": "skipped_failed", "reason": "unresolved_after_retries"}
                else:
                    st = state.get(key, "not_run")
                    coverage[key] = {"status": st if st in ("skipped_time_budget",) else "skipped_not_authorized",
                                     "reason": "stage_not_authorized_or_not_yet_run"}
    payload = {"rows": rows, "a_star": a_star, "a_star_status": a_star_status,
               "coverage": coverage, "extra_replications": {}}
    doc = io_schemas.make("e1_band.v1", payload, config)
    checkpoint = None
    if sigma == 3333.0 and per_accel:
        checkpoint = bands.shape_checkpoint(per_accel, GRIPS_FULL, a_star=a_star)
    return doc, checkpoint


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", default=os.path.join(ROOT, "ralph", "results"))
    ap.add_argument("--sigma", type=float, default=None)
    args = ap.parse_args()

    trials = load_trials(args.results)
    if not trials:
        print("no trials found; nothing to aggregate")
        return 1
    # config block from any trial (protocol constants are uniform per sweep)
    any_path = os.path.join(ROOT, next(iter({t["artifact"] for t in trials.values()})))
    config = io_schemas.read_json(any_path)["config"]

    sigmas = [args.sigma] if args.sigma else SIGMAS
    for sigma in sigmas:
        doc, checkpoint = aggregate_material(sigma, trials, {}, config)
        out = os.path.join(args.results, f"e1_band_{int(sigma)}.json")
        io_schemas.write_json(out, doc)
        n_done = sum(1 for c in doc["payload"]["coverage"].values() if c["status"] == "done")
        print(f"sigma={int(sigma)}: {n_done}/120 done; a_star={doc['payload']['a_star']} "
              f"({doc['payload']['a_star_status']}) -> {out}")
        if checkpoint:
            cp_path = os.path.join(args.results, "e1_shape_checkpoint.json")
            json.dump(checkpoint, open(cp_path, "w"), indent=2)
            print(f"shape checkpoint: outcome={checkpoint['outcome']} branch={checkpoint['branch']} -> {cp_path}")
            if checkpoint["outcome"] != "structure_present":
                print("STOP: non-structure outcome requires external review before further stages (pre-registered).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
