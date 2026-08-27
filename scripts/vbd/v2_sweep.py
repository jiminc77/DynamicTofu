"""V-2 force sweep + hard-milestone hold clip.

Sweeps per-finger grip force {0.3,0.5,0.8,1.2,2.0} N at soft_contact_mu 0.5,
records the lift trajectory, and captures a snapshot clip of the best holding
run. HARD MILESTONE: a run that lifts and holds the tofu intact >= 5 s.
If none holds -> STOP and escalate with traces.

Run: cd newton && uv run --no-sync python ../scripts/vbd/v2_sweep.py
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
FORCES = [0.3, 0.5, 0.8, 1.2, 2.0]
HOLD_AFTER_LIFT_S = 5.0
LIFT_TARGET_M = 0.06


def git_sha():
    try:
        return subprocess.check_output(["git", "-C", ROOT, "rev-parse", "HEAD"], text=True).strip()
    except Exception:
        return "unknown"


def run_force(force, snap_dir=None):
    cfg = VbdConfig(grip_force_n=force, hold_after_lift_s=HOLD_AFTER_LIFT_S, lift_height_m=LIFT_TARGET_M)
    rig = VbdTofuRig(cfg)
    n = int((cfg.t_hold + cfg.lift_duration_s + HOLD_AFTER_LIFT_S) * FPS)
    lift_done_t = cfg.t_hold + cfg.lift_duration_s
    traj = []
    snaps = 0
    for f in range(n):
        rig.step()
        if f % 6 == 0:
            m = rig.metrics()
            traj.append({"t": round(m["t"], 3), "com_rise": m["com_rise"], "bbox": m["bbox"], "finite": m["finite"]})
        if snap_dir and f % 10 == 0:
            os.makedirs(snap_dir, exist_ok=True)
            s0 = rig.state_0
            np.savez_compressed(os.path.join(snap_dir, f"f_{snaps:04d}.npz"),
                                particle_q=s0.particle_q.numpy()[rig.soft_start:rig.soft_end].astype(np.float32),
                                body_q=s0.body_q.numpy().astype(np.float32), t=np.float64(rig.sim_time))
            snaps += 1
    # metrics over the post-lift hold window
    post = [r for r in traj if r["t"] >= lift_done_t]
    finite_all = all(r["finite"] for r in traj)
    if post:
        rises = [r["com_rise"] for r in post]
        min_rise = min(rises); mean_rise = float(np.mean(rises))
        # bbox growth = max over hold of |bbox - original|/original per axis
        bbox0 = BLOCK_EDGE_M
        max_bbox_growth = max(max(abs(b - bbox0) for b in r["bbox"]) for r in post) / bbox0
    else:
        min_rise = mean_rise = 0.0; max_bbox_growth = 0.0
    held = finite_all and min_rise >= 0.04 and max_bbox_growth < 0.6
    outcome = "hold" if held else ("blowup" if not finite_all else "slip")
    return {
        "grip_force_n": force, "outcome": outcome,
        "min_com_rise_hold_m": round(min_rise, 4), "mean_com_rise_hold_m": round(mean_rise, 4),
        "max_bbox_growth_frac": round(max_bbox_growth, 3), "finite_all": finite_all,
        "lift_target_m": LIFT_TARGET_M, "hold_after_lift_s": HOLD_AFTER_LIFT_S,
        "traj_tail": traj[-6:],
    }, rig


def main() -> int:
    t0 = time.time()
    results = {}
    best_hold = None
    for force in FORCES:
        res, _rig = run_force(force)
        results[f"{force:g}N"] = res
        print(f"force={force}N: outcome={res['outcome']} min_rise={res['min_com_rise_hold_m']*1000:.1f}mm "
              f"bbox_growth={res['max_bbox_growth_frac']:.2f} finite={res['finite_all']}", flush=True)
        if res["outcome"] == "hold" and best_hold is None:
            best_hold = force

    out = {"gate": "V2_force_sweep", "git_sha": git_sha(), "controller_mode": "force_limited_effort_prismatic",
           "soft_contact_mu": 0.5, "E_pa": 25e3, "nu": 0.45, "forces_n": FORCES,
           "results": results, "best_holding_force_n": best_hold,
           "hard_milestone_met": best_hold is not None, "wall_s": time.time() - t0}
    os.makedirs(os.path.join(ROOT, "reports", "logs", "vbd"), exist_ok=True)
    json.dump(out, open(os.path.join(ROOT, "reports", "logs", "vbd", "v2_sweep.json"), "w"), indent=2, default=str)

    if best_hold is not None:
        snap_dir = os.path.join(ROOT, "reports", "media", "frames", "v2_hold")
        print(f"capturing hold clip at {best_hold}N ...", flush=True)
        run_force(best_hold, snap_dir=snap_dir)
        print("HARD MILESTONE MET:", best_hold, "N holds the tofu >=5s")
    else:
        print("HARD MILESTONE NOT MET: no force in the sweep holds -> STOP + escalate (traces in v2_sweep.json)")
    print("\nsummary ->", os.path.join(ROOT, "reports", "logs", "vbd", "v2_sweep.json"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
