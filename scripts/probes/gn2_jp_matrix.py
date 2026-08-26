"""Sign-off condition 1: crush-vs-gentle probe on ALL THREE materials.

Approved material: yield_pressure = 2 x sigma_Y. Per material:
- CRUSH: 5.0 N ramped close, 2.0 s hold -> damage predicate must fire
  (peak damage fraction well above 10%). Also confirms condition 4:
  the extrusion failure mode registers as damage.
- GENTLE: 1.5 N ramped close, 0.5 s hold, 5 cm smoothstep lift, 1.0 s hold
  -> damage predicate must NOT fire (peak fraction < 10%).

Damage fraction = judgment v1 predicate |Jp-1| > 0.05, evaluated on the MPM
entry state (the parent-state Jp buffer is never updated by the wrapper).

Outputs: reports/logs/gn2-jp-probe-<sigma>.json (per material, full traces)
and a combined verdict on stdout. Media frames saved for the receipt clips.

Run: cd newton && uv run --no-sync python ../scripts/probes/gn2_jp_matrix.py
"""

from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import numpy as np

from src.scene import BLOCK_CENTER, YIELD_PRESSURE_FACTOR
from scripts.probes.gn2_ar_probe import FRAME_DT, GRASP_Z, PREGRASP_Z, Rig
from scripts.probes.gn2_lift_jp import snapshot

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
SIGMAS = [2000.0, 3333.0, 6000.0]
F_CRUSH = 5.0
F_GENTLE = 1.5
LIFT_M = 0.05


def run_case(sigma: float, mode: str):
    rig = Rig(include_block=True, sigma_y=sigma, material_completion=True)
    frames_dir = os.path.join(ROOT, "reports", "media", "frames", f"{mode}_{int(sigma)}")
    trace, snap_i = [], 0

    def sample(tag):
        nonlocal snap_i
        jp = rig.jp()
        pq = rig.state.particle_q.numpy()
        trace.append({
            "t": round(rig.t, 3),
            "phase": tag,
            "damage_fraction": float(np.mean(np.abs(jp - 1.0) > 0.05)),
            "jp_min": float(jp.min()),
            "jp_max": float(jp.max()),
            "ext": [round(float(pq[:, i].max() - pq[:, i].min()), 4) for i in range(3)],
            "z_mean": float(pq[:, 2].mean()),
        })

    def record(n_ticks, tag, snap_every=20):
        nonlocal snap_i
        for i in range(n_ticks):
            rig.step(1)
            if (i + 1) % 10 == 0:
                sample(tag)
            if snap_every and (i + 1) % snap_every == 0:
                snapshot(rig, frames_dir, f"{mode}_{int(sigma)}", snap_i)
                snap_i += 1

    rig.step(int(0.5 / FRAME_DT))
    rig.move_ee((BLOCK_CENTER[0], BLOCK_CENTER[1], PREGRASP_Z), 1.5)
    rig.move_ee((BLOCK_CENTER[0], BLOCK_CENTER[1], GRASP_Z), 1.5)
    rig.move_ee_converge((BLOCK_CENTER[0], BLOCK_CENTER[1], GRASP_Z))
    sample("pregrasp")

    f_target = F_CRUSH if mode == "crush" else F_GENTLE
    n_ramp = int(0.3 / FRAME_DT)
    for k in range(n_ramp):
        rig.fingers.apply(rig.control, f_target * (k + 1) / n_ramp)
        rig.step(1)
    sample("close")

    if mode == "crush":
        record(int(2.0 / FRAME_DT), "crush_hold")
    else:
        record(int(0.5 / FRAME_DT), "hold")
        n = int(0.3 / FRAME_DT)
        for k in range(n):
            s = (k + 1) / n
            s = s * s * (3.0 - 2.0 * s)
            rig.move_ee((BLOCK_CENTER[0], BLOCK_CENTER[1], GRASP_Z + LIFT_M * s), FRAME_DT)
            if (k + 1) % 10 == 0:
                sample("lift")
                snapshot(rig, frames_dir, f"{mode}_{int(sigma)}", snap_i)
                snap_i += 1
        record(int(1.0 / FRAME_DT), "aloft")

    peak = max(r["damage_fraction"] for r in trace)
    final = trace[-1]
    return {
        "sigma_y_pa": sigma,
        "yield_pressure_pa": YIELD_PRESSURE_FACTOR * sigma,
        "mode": mode,
        "f_g_n": f_target,
        "peak_damage_fraction": peak,
        "final": final,
        "health": rig.health.report(),
        "n_frames": snap_i,
        "trace": trace,
    }


def main() -> int:
    t0 = time.time()
    verdicts = {}
    for sigma in SIGMAS:
        crush = run_case(sigma, "crush")
        gentle = run_case(sigma, "gentle")
        sep = crush["peak_damage_fraction"] / max(gentle["peak_damage_fraction"], 1e-9)
        ok = (
            crush["peak_damage_fraction"] > 0.10
            and gentle["peak_damage_fraction"] < 0.10
            and crush["health"]["clean"]
            and gentle["health"]["clean"]
        )
        out = {"crush": crush, "gentle": gentle, "separation_ratio": sep, "pass": bool(ok)}
        path = os.path.join(ROOT, "reports", "logs", f"gn2-jp-probe-{int(sigma)}.json")
        with open(path, "w") as fh:
            json.dump(out, fh, indent=2)
        verdicts[int(sigma)] = {
            "crush_peak": round(crush["peak_damage_fraction"], 3),
            "gentle_peak": round(gentle["peak_damage_fraction"], 3),
            "separation": round(sep, 1) if np.isfinite(sep) else "inf",
            "pass": bool(ok),
        }
        print(f"sigma={int(sigma)}: crush_peak={crush['peak_damage_fraction']:.3f} "
              f"gentle_peak={gentle['peak_damage_fraction']:.3f} pass={ok}")

    all_ok = all(v["pass"] for v in verdicts.values())
    print(json.dumps(verdicts, indent=2))
    print(f"wall {time.time()-t0:.0f}s")
    print("JP MATRIX:", "PASS" if all_ok else "FAIL")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
