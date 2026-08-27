"""V-2 extended diagnostic runs (persisted) — beyond the 5 prescribed forces.

Persists the force/friction/lift-speed variants cited in the V-2 report with
full time series, per-run config, metrics (peak + post-lift-hold min COM rise,
bbox growth, finite/health), and provenance (git SHA, controller mode), so the
escalation package is fully artifact-backed.

Run: cd newton && uv run --no-sync python ../scripts/vbd/v2_extended.py
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import numpy as np

from src.vbd_rig import VbdTofuRig, VbdConfig, FPS, BLOCK_EDGE_M

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

# (grip_force_n, soft_contact_mu, lift_duration_s)
RUNS = [
    (5.0, 0.5, 1.0), (8.0, 1.0, 1.0), (2.0, 1.0, 1.0),
    (1.2, 0.7, 3.0), (2.0, 0.7, 3.0), (2.0, 2.0, 1.0), (1.2, 2.0, 1.0),
]


def git_sha():
    try:
        return subprocess.check_output(["git", "-C", ROOT, "rev-parse", "HEAD"], text=True).strip()
    except Exception:
        return "unknown"


def run(force, mu, lift_dur):
    cfg = VbdConfig(grip_force_n=force, soft_contact_mu=mu, lift_duration_s=lift_dur,
                    hold_after_lift_s=5.0, lift_height_m=0.06)
    rig = VbdTofuRig(cfg)
    n = int((cfg.t_hold + lift_dur + 5.0) * FPS)
    lift_done = cfg.t_hold + lift_dur
    series = []
    for f in range(n):
        rig.step()
        if f % 12 == 0:
            m = rig.metrics()
            series.append({"t": round(m["t"], 3), "com_rise": round(m["com_rise"], 5),
                           "bbox": [round(b, 4) for b in m["bbox"]], "finite": m["finite"]})
    peak = max(r["com_rise"] for r in series)
    post = [r for r in series if r["t"] >= lift_done + 0.5]
    post_min = min((r["com_rise"] for r in post), default=0.0)
    finite_all = all(r["finite"] for r in series)
    max_bbox_growth = max((max(abs(b - BLOCK_EDGE_M) for b in r["bbox"]) for r in post), default=0.0) / BLOCK_EDGE_M
    return {
        "config": {"grip_force_n": force, "soft_contact_mu": mu, "lift_duration_s": lift_dur,
                   "E_pa": 25e3, "nu": 0.45, "hold_after_lift_s": 5.0, "lift_height_m": 0.06},
        "peak_com_rise_m": round(peak, 4), "post_lift_hold_min_com_rise_m": round(post_min, 4),
        "max_bbox_growth_frac": round(max_bbox_growth, 3), "finite_all": finite_all,
        "held_5s": bool(finite_all and post_min >= 0.04),
        "series": series,
    }


def main() -> int:
    t0 = time.time()
    out = {"gate": "V2_extended_diagnostic", "git_sha": git_sha(),
           "controller_mode": "force_limited_effort_prismatic", "runs": {}}
    for force, mu, lift_dur in RUNS:
        key = f"F{force:g}_mu{mu:g}_lift{lift_dur:g}s"
        out["runs"][key] = run(force, mu, lift_dur)
        r = out["runs"][key]
        print(f"{key}: peak={r['peak_com_rise_m']*1000:.1f}mm hold_min={r['post_lift_hold_min_com_rise_m']*1000:.1f}mm "
              f"bbox_growth={r['max_bbox_growth_frac']:.2f} held={r['held_5s']} finite={r['finite_all']}", flush=True)
    out["any_held_5s"] = any(r["held_5s"] for r in out["runs"].values())
    out["wall_s"] = time.time() - t0
    os.makedirs(os.path.join(ROOT, "reports", "logs", "vbd"), exist_ok=True)
    json.dump(out, open(os.path.join(ROOT, "reports", "logs", "vbd", "v2_extended.json"), "w"), indent=2, default=str)
    print("\nany_held_5s:", out["any_held_5s"], "-> reports/logs/vbd/v2_extended.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
