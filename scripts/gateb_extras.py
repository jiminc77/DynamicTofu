"""Gate B extras: it=8 iteration-robustness duplicate of the valid effort arms
(safeguard 2) + snapshot capture for the deliverable clips (holding vs dropping).

Run: cd newton && uv run --no-sync python ../scripts/gateb_extras.py
"""

from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np

from scripts.probes.diag_rig import DiagConfig, run_diag

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MEDIA = os.path.join(ROOT, "reports", "media")


def main() -> int:
    # --- it=8 duplicate of the valid effort arms (B1 stock, B4 sensor) --------
    it8 = {}
    for name, pad in (("B1_it8", "stock"), ("B4_it8", "sensor")):
        t0 = time.time()
        cfg = DiagConfig(name=name, sigma_y=6000.0, mu=1.0, target_Nf=0.60, voxel=0.005,
                         proxy_iterations=8, control="effort", pad=pad, viscosity=20.0,
                         lift_s=1.0, hold_s=10.0)
        res, log, _ = run_diag(cfg)
        bz = [r.get("block_centroid", [0, 0, 0])[2] for r in log]
        it8[name] = {"pad": pad, "outcome": res["outcome"], "health": res["health_clean"],
                     "preFn": res["preload"].get("Fn_L"), "finFn": res["final"].get("Fn_L"),
                     "z_max": round(max(bz), 4), "nodes": res["final"].get("nodes_L"), "wall": round(time.time() - t0)}
        print(f"{name}: {it8[name]}", flush=True)
    json.dump(it8, open(os.path.join(ROOT, "reports", "logs", "gateB-it8.json"), "w"), indent=2, default=str)

    # --- deliverable clips: one HOLDING run + one DROPPING run ---------------
    # HOLDING: the Gate A validation config that holds (elastic block, effort it=4).
    hold_cfg = DiagConfig(name="clip_hold", E_pa=20e3, nu=0.30, sigma_y=1e6, viscosity=100.0,
                          control="effort", pad="stock", mu=1.5, target_Nf=0.8, voxel=0.005,
                          proxy_iterations=4, lift_s=1.0, hold_s=3.0)
    run_diag(hold_cfg, snap_dir=os.path.join(MEDIA, "frames", "clip_hold"), snap_every=20)
    # DROPPING: Gate B tofu, stock effort (B1-like) -- the empty-band drop.
    drop_cfg = DiagConfig(name="clip_drop", sigma_y=6000.0, viscosity=20.0, control="effort",
                          pad="stock", mu=1.0, target_Nf=0.60, voxel=0.005, proxy_iterations=4,
                          lift_s=1.0, hold_s=3.0)
    run_diag(drop_cfg, snap_dir=os.path.join(MEDIA, "frames", "clip_drop"), snap_every=20)
    print("clips frames captured", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
