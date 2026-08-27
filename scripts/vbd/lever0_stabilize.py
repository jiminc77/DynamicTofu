"""Lever 0 branch (b): stabilization ladder on the VBD Coulomb oracle.

The oracle (E=1 MPa stiff block) EJECTS a particle at finger-close (bbox y ->
0.155 m) at contact ke=5e4 -- a normal-contact penetration/tunneling instability
(kf ladder did not help). Test the branch-(b) suspects that govern normal-contact
penetration: contact ke, particle_radius (surface resolution), substeps, VBD
iterations. Goal: a param set that STOPS the ejection AND holds the oracle
(Coulomb-correct), before touching tofu.

Run: cd newton && uv run --no-sync python ../scripts/vbd/lever0_stabilize.py
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

# (label, ke, particle_radius, substeps, vbd_iterations)
LADDER = [
    ("ke5e5", 5.0e5, 0.007, 12, 30),
    ("pr012", 5.0e4, 0.012, 12, 30),
    ("sub24", 5.0e4, 0.007, 24, 30),
    ("it60", 5.0e4, 0.007, 12, 60),
    ("combo", 5.0e5, 0.010, 24, 60),
    ("combo_max", 1.0e6, 0.012, 24, 60),
]


def git_sha():
    try:
        return subprocess.check_output(["git", "-C", ROOT, "rev-parse", "HEAD"], text=True).strip()
    except Exception:
        return "unknown"


def run(label, ke, pr, sub, it):
    cfg = VbdConfig(grip_force_n=0.45, soft_contact_mu=1.0, E_pa=1.0e6, nu=0.30,
                    soft_contact_ke=ke, particle_radius=pr, substeps=sub, vbd_iterations=it,
                    lift_duration_s=1.0, hold_after_lift_s=5.0, lift_height_m=0.06)
    rig = VbdTofuRig(cfg)
    n = int((cfg.t_hold + 1.0 + 5.0) * FPS)
    lift_done = cfg.t_hold + 1.0
    series = []
    max_y_ext = 0.0
    for f in range(n):
        rig.step()
        if f % 12 == 0:
            m = rig.metrics()
            series.append({"t": round(m["t"], 3), "com_rise": round(m["com_rise"], 5), "bbox": m["bbox"], "finite": m["finite"]})
            max_y_ext = max(max_y_ext, m["bbox"][1])
    peak = max(r["com_rise"] for r in series)
    post = [r for r in series if r["t"] >= lift_done + 0.5]
    post_min = min((r["com_rise"] for r in post), default=0.0)
    finite_all = all(r["finite"] for r in series)
    ejected = max_y_ext > 0.08   # y-extent explosion = particle ejection
    held = finite_all and (not ejected) and post_min >= 0.04
    return {"label": label, "ke": ke, "particle_radius": pr, "substeps": sub, "vbd_iterations": it,
            "peak_com_rise_m": round(peak, 4), "post_lift_hold_min_com_rise_m": round(post_min, 4),
            "max_y_extent_m": round(max_y_ext, 4), "ejected": bool(ejected), "finite_all": finite_all,
            "held_5s": bool(held)}


def main() -> int:
    t0 = time.time()
    out = {"gate": "V_lever0_stabilization_ladder", "git_sha": git_sha(),
           "baseline": "ke5e4/pr007/sub12/it30 EJECTS (y-ext 0.155) and slips", "runs": {}}
    for label, ke, pr, sub, it in LADDER:
        r = run(label, ke, pr, sub, it)
        out["runs"][label] = r
        print(f"{label}: held={r['held_5s']} ejected={r['ejected']} max_y={r['max_y_extent_m']:.3f} "
              f"peak={r['peak_com_rise_m']*1000:.1f}mm hold_min={r['post_lift_hold_min_com_rise_m']*1000:.1f}mm finite={r['finite_all']}", flush=True)
    holders = [k for k, r in out["runs"].items() if r["held_5s"]]
    out["stabilizing_configs"] = holders
    out["coulomb_restored"] = len(holders) > 0
    out["wall_s"] = time.time() - t0
    json.dump(out, open(os.path.join(ROOT, "reports", "logs", "vbd", "lever0_stabilize.json"), "w"), indent=2, default=str)
    print("\nCOULOMB RESTORED:", out["coulomb_restored"], "by", holders, "-> reports/logs/vbd/lever0_stabilize.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
