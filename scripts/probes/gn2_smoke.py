"""G-N2 smoke probe #1: construct scene + coupled solver, settle 1 s, health-check.

Run: cd newton && uv run --no-sync python ../scripts/probes/gn2_smoke.py
"""

from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import numpy as np
import warp as wp

import newton

from src.scene import VOXEL_SIZE_M, build_scene
from src.coupling import build_coupled_solver, harvested_body_wrenches
from src.control import assert_control_contract
from src.health import HealthAccumulator, block_volume_estimate

FRAME_DT = 0.01  # 100 Hz smoke tick


def main() -> int:
    t0 = time.time()
    model, meta, _builder = build_scene(3333.0, seed=0)
    print(f"model: bodies={model.body_count} joints={model.joint_count} "
          f"coords={model.joint_coord_count} dofs={model.joint_dof_count} particles={model.particle_count}")
    print(f"meta: arm_dofs={meta.arm_dof_indices} finger_dofs={meta.finger_dof_indices} "
          f"finger_bodies={meta.finger_body_indices} ee={meta.ee_body_index}")
    assert_control_contract(model, meta)
    print("control contract: OK")

    solver = build_coupled_solver(model, meta, voxel_size=VOXEL_SIZE_M)
    mpm = solver.solver("mpm")
    state = model.state()
    control = model.control()
    contacts_pipeline = newton.CollisionPipeline(model, soft_contact_max=0)
    contacts = contacts_pipeline.contacts()
    newton.eval_fk(model, model.joint_q, model.joint_qd, state)

    health = HealthAccumulator()
    vol0 = None
    n_ticks = int(1.0 / FRAME_DT)
    for i in range(n_ticks):
        state.clear_forces()
        contacts_pipeline.collide(state, contacts)
        solver.step(state, state, control, contacts, FRAME_DT)
        if (i + 1) % 10 == 0:
            q = state.particle_q.numpy()
            qd = state.particle_qd.numpy()
            jp_arr = getattr(state, "mpm_particle_Jp", None)
            jp = jp_arr.numpy() if jp_arr is not None else None
            health.check_tick(q, qd, jp, mpm_solver=mpm)
            if vol0 is None:
                vol0 = block_volume_estimate(q)
    qf = state.particle_q.numpy()
    drift = abs(block_volume_estimate(qf) - vol0) / vol0 if vol0 else 0.0
    wr = harvested_body_wrenches(solver)
    imp, pos, cid = mpm.collect_collider_impulses(state)
    report = {
        "health": health.report(),
        "settle_volume_drift": float(drift),
        "block_z_range": [float(qf[:, 2].min()), float(qf[:, 2].max())],
        "harvested_wrench_shape": list(wr.shape),
        "collider_arrays": [list(imp.shape), list(pos.shape), list(cid.shape)],
        "jp_attr_present": hasattr(state, "mpm_particle_Jp") or "via_getattr_check",
        "wall_s": time.time() - t0,
    }
    print(json.dumps(report, indent=2))
    ok = health.clean and drift < 0.05
    print("SMOKE:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
