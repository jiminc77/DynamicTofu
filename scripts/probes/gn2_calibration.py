"""G-N2 F_g -> joint_f calibration (8 levels, ascending+descending) + mimic probe.

F_g convention (frozen): per_finger_normal_mean - commanded per-finger normal
closure force [N], measured as the bilateral mean of the realized per-finger
normal resultants from the MPM collider reduction.

Frozen limits: slope in [0.90, 1.10]; |intercept| <= 0.05 N; max residual
<= max(0.05 N, 10% of commanded); monotone across all 8; hysteresis <= 0.05 N.

Mimic probe: convention 'dual' (both finger DOFs commanded) vs 'master'
(fr3_finger_joint1 only) at 1.2 N - detects factor-of-two errors from the
mimic equality constraint. The surviving convention is frozen.

Run: cd newton && uv run --no-sync python ../scripts/probes/gn2_calibration.py
"""

from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import numpy as np

from src.scene import BLOCK_CENTER
from src.coupling import node_reduction_per_body
from scripts.probes.gn2_ar_probe import FRAME_DT, GRASP_Z, PREGRASP_Z, Rig

LEVELS = [0.3, 0.5, 0.8, 1.2, 1.8, 2.5, 3.5, 5.0]
SETTLE_S = 0.5
MEASURE_S = 0.2


def realized_normal(rig) -> float:
    """Bilateral mean of per-finger normal components (pad-outward convention)."""
    bq = rig.state.body_q.numpy()
    reduced = node_reduction_per_body(rig.mpm, rig.state, bq, rig.model.body_com.numpy(), FRAME_DT)
    normals = rig.pad_normals_world()
    vals = []
    for b in rig.meta.finger_body_indices:
        F, _T, _n = reduced.get(b, (np.zeros(3), np.zeros(3), 0))
        vals.append(abs(float(np.dot(F, normals[b]))))
    return float(np.mean(vals))


def measure_level(rig, f_g: float) -> float:
    rig.fingers.apply(rig.control, f_g)
    rig.step(int(SETTLE_S / FRAME_DT), health_every=0)
    samples = []
    for _ in range(int(MEASURE_S / FRAME_DT)):
        rig.step(1, health_every=0)
        samples.append(realized_normal(rig))
    return float(np.mean(samples))


def grasp_setup(convention: str) -> Rig:
    rig = Rig(include_block=True)
    rig.fingers.convention = convention
    rig.step(int(0.5 / FRAME_DT))
    rig.move_ee((BLOCK_CENTER[0], BLOCK_CENTER[1], PREGRASP_Z), 1.5)
    rig.move_ee((BLOCK_CENTER[0], BLOCK_CENTER[1], GRASP_Z), 1.5)
    rig.move_ee_converge((BLOCK_CENTER[0], BLOCK_CENTER[1], GRASP_Z))
    return rig

def main() -> int:
    t0 = time.time()

    # --- mimic probe at 1.2 N -------------------------------------------------
    mimic = {}
    for conv in ("dual", "master"):
        rig = grasp_setup(conv)
        n_ramp = int(0.3 / FRAME_DT)
        for k in range(n_ramp):
            rig.fingers.apply(rig.control, 1.2 * (k + 1) / n_ramp)
            rig.step(1)
        mimic[conv] = measure_level(rig, 1.2)
    # surviving convention: realized closest to commanded 1.2
    convention = min(mimic, key=lambda c: abs(mimic[c] - 1.2))

    # --- 8-level calibration, ascending then descending ----------------------
    rig = grasp_setup(convention)
    n_ramp = int(0.3 / FRAME_DT)
    for k in range(n_ramp):
        rig.fingers.apply(rig.control, LEVELS[0] * (k + 1) / n_ramp)
        rig.step(1)
    ascending = {f: measure_level(rig, f) for f in LEVELS}
    descending = {f: measure_level(rig, f) for f in reversed(LEVELS)}
    hysteresis = max(abs(ascending[f] - descending[f]) for f in LEVELS)

    x = np.array(LEVELS)
    y = np.array([ascending[f] for f in LEVELS])
    slope, intercept = np.polyfit(x, y, 1)
    resid = np.abs(y - (slope * x + intercept))
    resid_limit = np.maximum(0.05, 0.10 * x)
    monotone = bool(np.all(np.diff(y) > 0))

    checks = {
        "slope_ok": bool(0.90 <= slope <= 1.10),
        "intercept_ok": bool(abs(intercept) <= 0.05),
        "residual_ok": bool(np.all(resid <= resid_limit)),
        "monotone_ok": monotone,
        "hysteresis_ok": bool(hysteresis <= 0.05),
    }
    out = {
        "mimic_probe_realized_at_1p2": mimic,
        "frozen_convention": convention,
        "ascending": ascending,
        "descending": descending,
        "slope": float(slope),
        "intercept_n": float(intercept),
        "max_residual_n": float(resid.max()),
        "hysteresis_n": float(hysteresis),
        "checks": checks,
        "health": rig.health.report(),
        "wall_s": time.time() - t0,
    }
    path = os.path.join(os.path.dirname(__file__), "..", "..", "reports", "logs", "gn2-calibration.json")
    with open(path, "w") as fh:
        json.dump(out, fh, indent=2)
    print(json.dumps({k: out[k] for k in ("mimic_probe_realized_at_1p2", "frozen_convention", "slope", "intercept_n", "max_residual_n", "hysteresis_n", "checks")}, indent=2))
    ok = all(checks.values())
    print("CALIBRATION:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
