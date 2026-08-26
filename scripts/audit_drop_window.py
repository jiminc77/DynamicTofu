"""Item-2 empirical audit: re-run all 225 E1 coordinates, confirm every drop
label is supported by in-window evidence only (drop_evidence_in_window=True).

Non-mutating: does NOT touch ralph/results/trials/. Writes incremental results
to reports/logs/drop-window-audit.json. run_trial hard-asserts the in-window
invariant, so any AssertionError is recorded as a VIOLATION (fail closed).

Usage: cd newton && uv run --no-sync python ../scripts/audit_drop_window.py
"""

from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SIGMAS = [2000.0, 3333.0, 6000.0]
ACCELS = [1.0, 2.5, 5.0, 10.0, 15.0]
GRIPS = [0.3, 0.8, 1.8, 3.5, 5.0]
SEEDS = [0, 1, 2]
CAL = {"slope": 0.99984, "intercept": 0.00027, "residual": 0.00034, "hysteresis": 0.0254,
       "status": "audit re-run"}
OUT = os.path.join(ROOT, "reports", "logs", "drop-window-audit.json")


def main() -> int:
    from src.trial import run_trial

    state = {"checked": [], "violations": [], "started": time.time()}
    if os.path.exists(OUT):
        state = json.load(open(OUT))
    done_keys = {r["key"] for r in state["checked"]}

    coords = [(s, a, f, seed) for s in SIGMAS for a in ACCELS for f in GRIPS for seed in SEEDS]
    for (s, a, f, seed) in coords:
        key = f"s{int(s)}_a{a:g}_f{f:g}_seed{seed}"
        if key in done_keys:
            continue
        rec = {"key": key}
        try:
            doc = run_trial(s, a, f, seed, calibration=CAL, out_json=None)  # no write to trials/
            p = doc["payload"]
            has_drop = "drop" in p["labels"]
            rec.update({"has_drop_label": has_drop,
                        "drop_evidence_in_window": p["drop_evidence_in_window"],
                        "drop_t_rel": p["drop_t_rel"],
                        "window_ok": (not has_drop) or bool(p["drop_evidence_in_window"])})
            if has_drop and not p["drop_evidence_in_window"]:
                state["violations"].append(key)
        except AssertionError as exc:
            rec.update({"violation": True, "error": str(exc), "window_ok": False})
            state["violations"].append(key)
        state["checked"].append(rec)
        json.dump(state, open(OUT, "w"), indent=1)
        print(f"[{len(state['checked'])}/225] {key}: window_ok={rec.get('window_ok')} "
              f"violations={len(state['violations'])}")

    n = len(state["checked"])
    v = len(state["violations"])
    state["summary"] = {"n_checked": n, "n_violations": v,
                        "verdict": "PASS" if v == 0 else "FAIL_CLOSED",
                        "wall_s": time.time() - state["started"]}
    json.dump(state, open(OUT, "w"), indent=1)
    print(f"\nAUDIT COMPLETE: {n}/225 checked, {v} violations -> {state['summary']['verdict']}")
    return 0 if v == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
