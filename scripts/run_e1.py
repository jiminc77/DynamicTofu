"""E1 staged sweep driver: Stage A -> shape checkpoint -> B -> C, fail-closed.

- Per-cell checkpoint/resume via ralph/results/e1_state.json (attempt-logged,
  bounded retries: 3 attempts total per seed, then `unresolved`).
- PM-4 guard: the state file freezes brief_sha256 at first start; a mismatch
  on resume aborts unless DECISIONS.md acknowledges the change.
- Stage atomicity: trimming happens at stage granularity only (the selector
  decided the stage set at G-N3; passed in via --stages).
- After Stage A: bands + pre-registered shape-checkpoint router; any branch
  other than structure_present STOPS the driver (external sign-off gate).
- Throughput monitor at 10 cells and every stage boundary.

Usage (post G-N3 only; --scratch for engineering shakedowns):
  cd newton && uv run --no-sync python ../scripts/run_e1.py --stages A [B2000 B6000 C1 C2]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

ACCELS = [1.0, 2.5, 5.0, 10.0, 15.0]
GRIPS_SUBSET = [0.3, 0.8, 1.8, 3.5, 5.0]
GRIPS_FULL = [0.3, 0.5, 0.8, 1.2, 1.8, 2.5, 3.5, 5.0]
SEEDS = [0, 1, 2]


def stage_cells(stage: str):
    if stage == "A":
        return [(3333.0, a, f, s) for a in ACCELS for f in GRIPS_SUBSET for s in SEEDS]
    if stage == "B2000":
        return [(2000.0, a, f, s) for a in ACCELS for f in GRIPS_SUBSET for s in SEEDS]
    if stage == "B6000":
        return [(6000.0, a, f, s) for a in ACCELS for f in GRIPS_SUBSET for s in SEEDS]
    if stage == "C1":
        extra = [f for f in GRIPS_FULL if f not in GRIPS_SUBSET]
        return [(3333.0, a, f, s) for a in ACCELS for f in extra for s in SEEDS]
    raise ValueError(f"unknown stage {stage} (C2 boundary replication is planned separately)")


def cell_key(sigma, a, f, s):
    return f"s{int(sigma)}_a{a:g}_f{f:g}_seed{s}"


def brief_sha():
    return hashlib.sha256(open(os.path.join(ROOT, "BRIEF_WS.md"), "rb").read()).hexdigest()


def load_state(path):
    if os.path.exists(path):
        return json.load(open(path))
    return {"brief_sha256": brief_sha(), "cells": {}, "started": time.time()}


def save_state(path, state):
    tmp = path + ".tmp"
    json.dump(state, open(tmp, "w"), indent=1)
    os.replace(tmp, path)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--stages", nargs="+", default=["A"])
    ap.add_argument("--scratch", action="store_true",
                    help="engineering shakedown: write under reports/logs/e1-scratch, never ralph/results")
    ap.add_argument("--max-cells", type=int, default=None)
    ap.add_argument("--calibration-json", default=os.path.join(ROOT, "reports", "logs", "gn2-calibration.json"))
    args = ap.parse_args()

    base = os.path.join(ROOT, "reports", "logs", "e1-scratch") if args.scratch else os.path.join(ROOT, "ralph", "results")
    trials_dir = os.path.join(base, "trials")
    os.makedirs(trials_dir, exist_ok=True)
    state_path = os.path.join(base, "e1_state.json")
    state = load_state(state_path)

    if state["brief_sha256"] != brief_sha():
        print("FATAL: BRIEF_WS.md hash changed since this sweep state was created (PM-4).")
        print("Acknowledge the protocol change in ralph/DECISIONS.md and start a fresh state.")
        return 2

    cal = {"slope": 1.0, "intercept": 0.0, "residual": 0.0, "hysteresis": 0.0, "status": "scratch"}
    if os.path.exists(args.calibration_json) and not args.scratch:
        cj = json.load(open(args.calibration_json))
        cal = {"slope": cj["slope"], "intercept": cj["intercept_n"],
               "residual": cj["max_residual_n"], "hysteresis": cj["hysteresis_n"]}

    from src.trial import run_trial  # heavy import deferred

    done_count, t_start = 0, time.time()
    for stage in args.stages:
        cells = stage_cells(stage)
        print(f"== stage {stage}: {len(cells)} trials")
        for sigma, a, f, s in cells:
            key = cell_key(sigma, a, f, s)
            cell = state["cells"].get(key, {"attempts": []})
            if cell.get("status") in ("done", "unresolved"):
                continue
            if args.max_cells is not None and done_count >= args.max_cells:
                print("max-cells reached; stopping (resumable)")
                save_state(state_path, state)
                return 0
            attempt = len(cell["attempts"]) + 1
            out_json = os.path.join(trials_dir, f"{key}.json")
            t0 = time.time()
            try:
                doc = run_trial(sigma, a, f, s, calibration=cal, out_json=out_json)
                healthy = doc["payload"]["health"]["clean"]
                status = "done" if healthy else "invalid"
                reason = "" if healthy else "health_predicate_failed"
            except Exception as exc:  # noqa: BLE001 - every failure is an attempt record
                status, reason, doc = "invalid", repr(exc)[:300], None
            cell["attempts"].append({"n": attempt, "status": status, "reason": reason,
                                     "wall_s": round(time.time() - t0, 1)})
            if status == "done":
                cell["status"] = "done"
                cell["artifact"] = os.path.relpath(out_json, ROOT)
                p = doc["payload"]
                print(f"[{key}] attempt {attempt}: {p['labels']} peak_dmg={p['peak_damage_fraction']:.3f} "
                      f"a_real={p['a_peak_realized_ms2']:.2f} wall={p['wall_time_s']:.0f}s")
                done_count += 1
            elif attempt >= 3:
                cell["status"] = "unresolved"
                print(f"[{key}] UNRESOLVED after 3 attempts: {reason}")
            else:
                print(f"[{key}] attempt {attempt} invalid ({reason}); retrying")
            state["cells"][key] = cell
            save_state(state_path, state)
            if done_count in (10,) or done_count % 25 == 0 and done_count:
                rate = done_count / max(time.time() - t_start, 1e-9) * 3600
                print(f"-- throughput: {done_count} cells, {rate:.0f} cells/h")
        print(f"== stage {stage} complete")
        if stage == "A" and not args.scratch:
            print("Stage A finished: run aggregate_bands.py + shape checkpoint BEFORE any further stage.")
            break
    save_state(state_path, state)
    return 0


if __name__ == "__main__":
    sys.exit(main())
