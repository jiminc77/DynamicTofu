"""Lever 0 — VBD Coulomb oracle (analog of MPM Gate A) + kf ladder branch.

A STIFF elastic block (E=1 MPa, nu=0.3, same 4 cm / 64 g) grasped by the same
floating VBD gripper at 0.45 N/finger, mu=1.0, 1 s lift + 5 s hold. Rigid
Coulomb says it MUST hold (capacity 0.9 N vs 0.63 N weight).

(a) holds -> VBD contact stack fine; failure is material-side -> caller runs
    the E sweep.
(b) slips -> contact stack/params are the problem -> run a kf ladder
    {1e3, 1e4, 1e5} and report which restores Coulomb behavior.

Run: cd newton && uv run --no-sync python ../scripts/vbd/lever0_oracle.py
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


def git_sha():
    try:
        return subprocess.check_output(["git", "-C", ROOT, "rev-parse", "HEAD"], text=True).strip()
    except Exception:
        return "unknown"


def run_oracle(kf, snap_dir=None):
    cfg = VbdConfig(grip_force_n=0.45, soft_contact_mu=1.0, E_pa=1.0e6, nu=0.30,
                    soft_contact_kf=kf, lift_duration_s=1.0, hold_after_lift_s=5.0, lift_height_m=0.06)
    rig = VbdTofuRig(cfg)
    n = int((cfg.t_hold + 1.0 + 5.0) * FPS)
    lift_done = cfg.t_hold + 1.0
    series = []
    si = 0
    for f in range(n):
        rig.step()
        if f % 12 == 0:
            m = rig.metrics()
            series.append({"t": round(m["t"], 3), "com_rise": round(m["com_rise"], 5),
                           "bbox": [round(b, 4) for b in m["bbox"]], "finite": m["finite"]})
        if snap_dir and f % 10 == 0:
            os.makedirs(snap_dir, exist_ok=True)
            s0 = rig.state_0
            np.savez_compressed(os.path.join(snap_dir, f"f_{si:04d}.npz"),
                                particle_q=s0.particle_q.numpy()[rig.soft_start:rig.soft_end].astype(np.float32),
                                body_q=s0.body_q.numpy().astype(np.float32), t=np.float64(rig.sim_time)); si += 1
    peak = max(r["com_rise"] for r in series)
    post = [r for r in series if r["t"] >= lift_done + 0.5]
    post_min = min((r["com_rise"] for r in post), default=0.0)
    finite_all = all(r["finite"] for r in series)
    max_bbox_growth = max((max(abs(b - BLOCK_EDGE_M) for b in r["bbox"]) for r in post), default=0.0) / BLOCK_EDGE_M
    return {
        "kf": kf, "E_pa": 1.0e6, "nu": 0.30, "grip_force_n": 0.45, "soft_contact_mu": 1.0,
        "coulomb_capacity_n": 0.9, "weight_n": 0.628,
        "peak_com_rise_m": round(peak, 4), "post_lift_hold_min_com_rise_m": round(post_min, 4),
        "max_bbox_growth_frac": round(max_bbox_growth, 3), "finite_all": finite_all,
        "held_5s": bool(finite_all and post_min >= 0.04),
        "series": series,
    }


def main() -> int:
    t0 = time.time()
    out = {"gate": "V_lever0_vbd_coulomb_oracle", "git_sha": git_sha(),
           "controller_mode": "force_limited_effort_prismatic",
           "friction_model": "VBD rigid-particle contact = impulse/displacement-level ALM Coulomb with cone projection (contact_tangent_rho ~ soft_contact_kf); NOT velocity-level",
           "runs": {}}
    base = run_oracle(1.0e3, snap_dir=os.path.join(ROOT, "reports", "media", "frames", "lever0_oracle_kf1e3"))
    out["runs"]["kf1e3"] = base
    print(f"oracle kf=1e3: held={base['held_5s']} peak={base['peak_com_rise_m']*1000:.1f}mm "
          f"hold_min={base['post_lift_hold_min_com_rise_m']*1000:.1f}mm finite={base['finite_all']}", flush=True)

    branch = None
    if base["held_5s"]:
        branch = "(a) oracle HOLDS -> VBD contact stack fine; failure is material-side -> run E sweep {25,50,100,200 kPa}"
    else:
        branch = "(b) oracle SLIPS/blowup at kf=1e3 -> contact-stack/param problem, not tofu -> kf ladder"
        for kf in (1.0e4, 1.0e5):
            r = run_oracle(kf)
            out["runs"][f"kf{kf:g}"] = r
            print(f"oracle kf={kf:g}: held={r['held_5s']} peak={r['peak_com_rise_m']*1000:.1f}mm "
                  f"hold_min={r['post_lift_hold_min_com_rise_m']*1000:.1f}mm finite={r['finite_all']}", flush=True)
        restored = next((r["kf"] for k, r in out["runs"].items() if r["held_5s"]), None)
        out["kf_that_restores_coulomb"] = restored

    out["branch"] = branch
    out["wall_s"] = time.time() - t0
    os.makedirs(os.path.join(ROOT, "reports", "logs", "vbd"), exist_ok=True)
    json.dump(out, open(os.path.join(ROOT, "reports", "logs", "vbd", "lever0_oracle.json"), "w"), indent=2, default=str)
    print("\nBRANCH:", branch, "-> reports/logs/vbd/lever0_oracle.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
