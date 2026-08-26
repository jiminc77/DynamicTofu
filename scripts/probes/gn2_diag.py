"""Diagnostic: finger effort sign, grasp geometry, contact evolution during close.

Run: cd newton && uv run --no-sync python ../scripts/probes/gn2_diag.py
"""

from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import numpy as np

from src.scene import BLOCK_CENTER, BLOCK_EDGE_M, VOXEL_SIZE_M
from scripts.probes.gn2_ar_probe import FRAME_DT, GRASP_Z, PREGRASP_Z, Rig


def effort_sign_test():
    out = {}
    for sign, name in ((+1.0, "positive"), (-1.0, "negative")):
        rig = Rig(include_block=False)
        rig.step(int(0.3 / FRAME_DT))
        jf = rig.control.joint_f.numpy()
        jf[:] = 0.0
        for d in rig.meta.finger_dof_indices:
            jf[d] = sign * 2.0
        rig.control.joint_f.assign(jf)
        qs = []
        for _ in range(int(1.0 / FRAME_DT)):
            rig.step(1)
            qs.append(rig.finger_q().tolist())
        out[name] = {"q_start": qs[0], "q_mid": qs[len(qs) // 2], "q_end": qs[-1]}
    return out


def close_evolution():
    rig = Rig(include_block=True)
    rig.step(int(0.5 / FRAME_DT))
    rig.move_ee((BLOCK_CENTER[0], BLOCK_CENTER[1], PREGRASP_Z), 1.5)
    rig.move_ee((BLOCK_CENTER[0], BLOCK_CENTER[1], GRASP_Z), 1.0)
    rig.step(int(0.3 / FRAME_DT))

    # grasp geometry before closing
    bq = rig.state.body_q.numpy()
    left, right = rig.meta.finger_body_indices
    pq = rig.state.particle_q.numpy()
    geometry = {
        "pad_left_pos": bq[left][:3].tolist(),
        "pad_right_pos": bq[right][:3].tolist(),
        "block_aabb_lo": pq.min(axis=0).tolist(),
        "block_aabb_hi": pq.max(axis=0).tolist(),
        "block_center_nominal": list(BLOCK_CENTER),
        "block_edge_m": BLOCK_EDGE_M,
    }

    rig.fingers.apply(rig.control, 0.5)  # gentler close for the diagnostic
    trace = []
    for i in range(int(2.0 / FRAME_DT)):
        rig.step(1)
        if (i + 1) % 20 == 0:
            imp, pos, cid = rig.mpm.collect_collider_impulses(rig.state)
            impn = imp.numpy()
            cidn = cid.numpy().astype(int)
            body_of = rig.mpm.collider_body_index.numpy().astype(int)
            mags = np.linalg.norm(impn, axis=1)
            active = mags > 0
            finger_nodes = 0
            finger_force = 0.0
            for k in np.nonzero(active)[0]:
                c = cidn[k]
                if 0 <= c < len(body_of) and body_of[c] in (left, right):
                    finger_nodes += 1
                    finger_force += mags[k] / FRAME_DT
            trace.append(
                {
                    "t": round(rig.t, 3),
                    "finger_q": rig.finger_q().tolist(),
                    "active_nodes_total": int(active.sum()),
                    "finger_contact_nodes": int(finger_nodes),
                    "finger_force_sum_n": float(finger_force),
                }
            )
    return geometry, trace


def main():
    sign = effort_sign_test()
    geometry, trace = close_evolution()
    report = {"effort_sign": sign, "grasp_geometry": geometry, "close_trace": trace[::2]}
    print(json.dumps(report, indent=2))
    path = os.path.join(os.path.dirname(__file__), "..", "..", "reports", "logs", "gn2-diag.json")
    with open(path, "w") as fh:
        json.dump({"effort_sign": sign, "grasp_geometry": geometry, "close_trace": trace}, fh, indent=2)


if __name__ == "__main__":
    main()
