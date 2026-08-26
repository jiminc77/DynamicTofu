"""G-N2 lift + Jp probe: gentle grasp-and-lift 5 cm (health clean, volume drift)
vs deliberate crush (damage fraction rising). Records frames for the gate media.

Run: cd newton && uv run --no-sync python ../scripts/probes/gn2_lift_jp.py
Outputs: reports/logs/gn2-lift-jp.json, reports/media/frames/{gentle,crush}/*.npz
(particle+body snapshots for rendering; rendering itself never blocks physics).
"""

from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import numpy as np

from src.scene import BLOCK_CENTER
from src.health import block_volume_estimate, volume_drift
from scripts.probes.gn2_ar_probe import FRAME_DT, GRASP_Z, PREGRASP_Z, Rig

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
DAMAGE_JP_DEV = 0.05   # judgment v1, frozen
LIFT_M = 0.05
F_GENTLE = 0.5
F_CRUSH = 5.0


def damage_fraction(jp: np.ndarray) -> float:
    return float(np.mean(np.abs(jp - 1.0) > DAMAGE_JP_DEV))


def snapshot(rig, outdir, tag, idx):
    os.makedirs(outdir, exist_ok=True)
    np.savez_compressed(
        os.path.join(outdir, f"{tag}_{idx:04d}.npz"),
        particle_q=rig.state.particle_q.numpy().astype(np.float32),
        jp=rig.state.mpm.particle_Jp.numpy().astype(np.float32),
        body_q=rig.state.body_q.numpy().astype(np.float32),
        t=np.float64(rig.t),
    )


def run(f_g: float, tag: str, lift: bool):
    rig = Rig(include_block=True)
    frames_dir = os.path.join(ROOT, "reports", "media", "frames", tag)
    frac_trace, snap_i = [], 0

    def record(n_ticks, snap_every=None):
        nonlocal snap_i
        for i in range(n_ticks):
            rig.step(1)
            if (i + 1) % 20 == 0:
                jp = rig.state.mpm.particle_Jp.numpy()
                frac_trace.append({"t": round(rig.t, 3), "damage_fraction": damage_fraction(jp)})
            if snap_every and (i + 1) % snap_every == 0:
                snapshot(rig, frames_dir, tag, snap_i)
                snap_i += 1

    rig.step(int(0.5 / FRAME_DT))
    rig.move_ee((BLOCK_CENTER[0], BLOCK_CENTER[1], PREGRASP_Z), 1.5)
    rig.move_ee((BLOCK_CENTER[0], BLOCK_CENTER[1], GRASP_Z), 1.0)
    rig.step(int(0.3 / FRAME_DT))
    vol_ref = block_volume_estimate(rig.state.particle_q.numpy())
    rig.fingers.apply(rig.control, f_g)
    record(int(0.5 / FRAME_DT), snap_every=10)          # close + hold
    if lift:
        # lift 5 cm over 0.3 s, then hold: snapshot for media
        start = GRASP_Z
        n = int(0.3 / FRAME_DT)
        for k in range(n):
            z = start + LIFT_M * (k + 1) / n
            rig.move_ee((BLOCK_CENTER[0], BLOCK_CENTER[1], z), FRAME_DT)
            if (k + 1) % 6 == 0:
                snapshot(rig, frames_dir, tag, snap_i)
                snap_i += 1
        record(int(1.0 / FRAME_DT), snap_every=20)      # gentle hold aloft
    else:
        record(int(2.0 / FRAME_DT), snap_every=20)      # keep crushing

    jp = rig.state.mpm.particle_Jp.numpy()
    pq = rig.state.particle_q.numpy()
    vol_now = block_volume_estimate(pq)
    peak_frac = max(r["damage_fraction"] for r in frac_trace) if frac_trace else 0.0
    return rig, {
        "tag": tag,
        "f_g_n": f_g,
        "lifted": lift,
        "final_damage_fraction": damage_fraction(jp),
        "peak_damage_fraction": peak_frac,
        "damage_trace": frac_trace,
        "block_z_mean": float(pq[:, 2].mean()),
        "volume_drift_since_pregrasp": volume_drift(vol_ref, vol_now),
        "health": rig.health.report(),
        "n_snapshots": snap_i,
    }


def main() -> int:
    t0 = time.time()
    rig_g, gentle = run(F_GENTLE, "gentle", lift=True)
    rig_c, crush = run(F_CRUSH, "crush", lift=False)

    # lift acceptance: block airborne by ~5 cm; gentle-hold health + drift; low damage
    block_rise = gentle["block_z_mean"] - (BLOCK_CENTER[2])
    sep = (crush["peak_damage_fraction"] / gentle["peak_damage_fraction"]) if gentle["peak_damage_fraction"] > 0 else float("inf")
    out = {
        "gentle": gentle,
        "crush": crush,
        "lift_rise_m": float(block_rise),
        "separation_ratio_crush_over_gentle": sep,
        "wall_s": time.time() - t0,
    }
    with open(os.path.join(ROOT, "reports", "logs", "gn2-lift-jp.json"), "w") as fh:
        json.dump(out, fh, indent=2)
    ok_lift = gentle["health"]["clean"] and block_rise > 0.04 and gentle["volume_drift_since_pregrasp"] <= 0.02
    ok_jp = crush["peak_damage_fraction"] > 0.10 and gentle["peak_damage_fraction"] < 0.10 and sep >= 2.0
    print(json.dumps({k: out[k] for k in ("lift_rise_m", "separation_ratio_crush_over_gentle")}, indent=2))
    print("gentle: peak_frac", gentle["peak_damage_fraction"], "drift", gentle["volume_drift_since_pregrasp"], "health", gentle["health"]["clean"])
    print("crush : peak_frac", crush["peak_damage_fraction"], "health", crush["health"]["clean"])
    print("LIFT:", "PASS" if ok_lift else "FAIL", "| JP:", "PASS" if ok_jp else "FAIL")
    return 0 if (ok_lift and ok_jp) else 1


if __name__ == "__main__":
    sys.exit(main())
