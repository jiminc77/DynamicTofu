"""G-N2 kinematic reach probe (frozen criteria, pulled forward from G-N3).

Criteria (pending-approval.md "Frozen tolerances"):
1. Joint-space margin: at every waypoint, all 7 arm joints have
   min(q - lower, upper - q) >= 0.10 rad.
2. Cartesian perturbation margin: path stays IK-solvable under +/-0.02 m
   perturbations along each axis at every waypoint, IK residual <= 2 mm,
   with criterion 1 still satisfied.

Path: transport at z = GRASP_Z + 0.05, y from -0.5 out +0.3 m toward the
workspace centre (y = -0.2) and back (the +/-y routing from the brief),
sampled at 25 waypoints, plus the grasp/lift column.

Run: cd newton && uv run --no-sync python ../scripts/probes/gn2_reach.py
"""

from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import numpy as np
import warp as wp

import newton

from src.scene import BLOCK_CENTER, build_scene
from src.control import ArmIK, EE_TOOL_OFFSET

GRASP_QUAT = (1.0, 0.0, 0.0, 0.0)
Z_TRANSPORT = 0.27
MARGIN_RAD = 0.10
IK_RESID_M = 0.002
PERTURB_M = 0.02


def tool_fk(model, state, meta, q_full):
    jq = wp.array(q_full, dtype=float, device=model.device)
    newton.eval_fk(model, jq, model.joint_qd, state)
    bq = state.body_q.numpy()[meta.ee_body_index]
    x, y, z, w = bq[3:7]
    u = np.array([x, y, z]); v = np.array(EE_TOOL_OFFSET)
    return bq[:3] + 2 * np.dot(u, v) * u + (w * w - np.dot(u, u)) * v + 2 * w * np.cross(u, v)


def check_point(model, state, meta, ikw, target, lo, hi, arm_coords):
    sol = ikw.solve_to_targets(target, GRASP_QUAT)
    realized = tool_fk(model, state, meta, sol)
    resid = float(np.linalg.norm(realized - np.asarray(target)))
    margins = np.minimum(sol[arm_coords] - lo[arm_coords], hi[arm_coords] - sol[arm_coords])
    return resid, float(margins.min()), sol


def main() -> int:
    model, meta, _ = build_scene(3333.0, seed=0, include_block=False)
    state = model.state()
    newton.eval_fk(model, model.joint_q, model.joint_qd, state)
    ikw = ArmIK(model, meta, (BLOCK_CENTER[0], BLOCK_CENTER[1], Z_TRANSPORT), GRASP_QUAT)
    lo = model.joint_limit_lower.numpy()
    hi = model.joint_limit_upper.numpy()
    arm = np.asarray(meta.arm_coord_indices, dtype=int)

    waypoints = [(BLOCK_CENTER[0], y, Z_TRANSPORT) for y in np.linspace(-0.5, -0.2, 25)]
    waypoints += [(BLOCK_CENTER[0], BLOCK_CENTER[1], z) for z in np.linspace(0.22, 0.32, 6)]

    rows, worst = [], {"resid": 0.0, "margin": np.inf, "pert_resid": 0.0, "pert_margin": np.inf}
    ok = True
    for wpt in waypoints:
        resid, margin, _sol = check_point(model, state, meta, ikw, wpt, lo, hi, arm)
        p_res, p_marg = 0.0, np.inf
        for axis in range(3):
            for sign in (+1.0, -1.0):
                pt = np.asarray(wpt, dtype=float)
                pt[axis] += sign * PERTURB_M
                r2, m2, _ = check_point(model, state, meta, ikw, pt.tolist(), lo, hi, arm)
                p_res, p_marg = max(p_res, r2), min(p_marg, m2)
        rows.append({"wp": [round(v, 4) for v in wpt], "resid_m": resid, "margin_rad": margin,
                     "pert_max_resid_m": p_res, "pert_min_margin_rad": p_marg})
        worst["resid"] = max(worst["resid"], resid)
        worst["margin"] = min(worst["margin"], margin)
        worst["pert_resid"] = max(worst["pert_resid"], p_res)
        worst["pert_margin"] = min(worst["pert_margin"], p_marg)
        if resid > IK_RESID_M or margin < MARGIN_RAD or p_res > IK_RESID_M or p_marg < MARGIN_RAD:
            ok = False

    out = {
        "criteria": {"margin_rad": MARGIN_RAD, "ik_resid_m": IK_RESID_M, "perturb_m": PERTURB_M},
        "n_waypoints": len(waypoints),
        "worst": {k: float(v) for k, v in worst.items()},
        "pass": bool(ok),
        "rows": rows,
    }
    path = os.path.join(os.path.dirname(__file__), "..", "..", "reports", "logs", "gn2-reach.json")
    with open(path, "w") as fh:
        json.dump(out, fh, indent=2)
    print(json.dumps({k: out[k] for k in ("n_waypoints", "worst", "pass")}, indent=2))
    print("REACH PROBE:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
