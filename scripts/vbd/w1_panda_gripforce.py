"""Measure suspended-hold Panda pad forces with the validated R3 collector.

GPU driver (from newton/):
  PYTHONPATH=/home/simx2204/Workspace/DynamicTofu uv run --no-sync python ../scripts/vbd/w1_panda_gripforce.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.pad_wrench import capture_pre_step, collect_pad_wrench
from src.vbd_rig2 import FPS, GRAB_Z, Vbd2Config
from src.vbd_rig_panda import PandaRig

CASES = ((7000.0, 2.0), (15000.0, 1.2))


def _cfg(E_pa: float, force: float) -> Vbd2Config:
    return Vbd2Config(
        E_pa=E_pa, nu=0.45, grip_force_n=force, cell_m=0.005,
        particle_radius=0.0025, contact_ke=1e3, contact_kd=1.0,
        mu_pair=1.0, friction_epsilon=2e-4, soft_contact_margin=1e-3,
        substeps=80, lift_s=2.5, hold_s=5.0, lift_height_m=0.05, seed=0,
    )


def _frame(rig: PandaRig, force: float, lift_target: float):
    """PandaRig.step equivalent retaining the final substep's pre-state."""
    rig.set_control(force, lift_target)
    pre = None
    for k in range(rig.sim_substeps):
        rig.state_0.clear_forces()
        pre = capture_pre_step(rig.state_0)
        rig.collision_pipeline.collide(rig.state_0, rig.contacts)
        rig.solver.step(rig.state_0, rig.state_1, rig.control,
                        rig.contacts, rig.sim_dt)
        rig.state_0, rig.state_1 = rig.state_1, rig.state_0
        if rig.couple:
            # Keep this byte-for-byte equivalent to PandaRig.step's coupling.
            bq = rig.state_0.body_q.numpy()
            half_gap = 0.5 * (bq[rig.b_left, 1] - bq[rig.b_right, 1])
            palm_y = bq[rig.b_palm, 1]
            bq[rig.b_left, 1] = palm_y + half_gap
            bq[rig.b_right, 1] = palm_y - half_gap
            rig.state_0.body_q.assign(bq)
        for fn in rig._substep_hooks:
            fn(rig, k)
    rig.sim_time += rig.frame_dt
    wrench = collect_pad_wrench(
        rig, pre_state=pre, post_state=rig.state_0,
        contacts=rig.contacts, dt=rig.sim_dt,
    )
    # R3's frozen-rig convenience normals assume two identity-oriented pad
    # bodies. Panda's right finger body is rotated pi about z, so derive each
    # outward normal from the actual replacement pad's local +y axis.
    body_q = rig.state_0.body_q.numpy()
    shape_xform = rig.model.shape_transform.numpy()
    for name, body, shape in (
        ("left", rig.b_left, rig.s_left), ("right", rig.b_right, rig.s_right)
    ):
        q_body = body_q[body, 3:7]
        q_shape = shape_xform[shape, 3:7]
        normal = _rotate(q_body, _rotate(q_shape, np.array([0.0, 1.0, 0.0])))
        normal /= np.linalg.norm(normal)
        force_world = np.asarray(wrench[name]["force_world"], dtype=np.float64)
        fn = float(np.dot(force_world, normal))
        wrench[name]["Fn"] = fn
        wrench[name]["Ft"] = float(np.linalg.norm(force_world - fn * normal))
        wrench[name]["outward_normal_world"] = normal.tolist()
    jf = rig.control.joint_f.numpy()
    return wrench, [float(jf[rig.l_dof]), float(jf[rig.r_dof])]


def _rotate(quat, vector):
    axis = np.asarray(quat[:3], dtype=np.float64)
    vector = np.asarray(vector, dtype=np.float64)
    return (vector + 2.0 * float(quat[3]) * np.cross(axis, vector)
            + 2.0 * np.cross(axis, np.cross(axis, vector)))


def _median(rows, pad: str, key: str) -> float:
    return float(np.median([row["wrench"][pad][key] for row in rows]))


def run_case(E_pa: float, force: float) -> dict:
    rig = PandaRig(_cfg(E_pa, force), couple=True)
    cfg = rig.cfg
    t_pre = cfg.ramp_s + cfg.preload_s
    t_lift = t_pre + cfg.lift_s
    t_end = t_lift + cfg.hold_s
    rows = []
    for _ in range(round(t_end * FPS)):
        t = rig.sim_time
        command = force * min(1.0, t / cfg.ramp_s)
        lift_fraction = min(1.0, max(0.0, t - t_pre) / cfg.lift_s)
        lift_target = GRAB_Z + cfg.lift_height_m * lift_fraction
        wrench, joint_f = _frame(rig, command, lift_target)
        if rig.sim_time >= t_end - 1.0:
            rows.append({"t": float(rig.sim_time), "wrench": wrench,
                         "joint_f_applied": joint_f})
    return {
        "E_pa": E_pa,
        "commanded_F": force,
        "Fn_left": _median(rows, "left", "Fn"),
        "Fn_right": _median(rows, "right", "Fn"),
        "Ft_left": _median(rows, "left", "Ft"),
        "Ft_right": _median(rows, "right", "Ft"),
        "n_contacts": {
            "left": int(round(np.median([r["wrench"]["left"]["n_contacts"] for r in rows]))),
            "right": int(round(np.median([r["wrench"]["right"]["n_contacts"] for r in rows]))),
        },
        "joint_f_applied": {
            "left": float(np.median([r["joint_f_applied"][0] for r in rows])),
            "right": float(np.median([r["joint_f_applied"][1] for r in rows])),
        },
        "hold_samples": len(rows),
    }


def main() -> int:
    result = {
        "schema": "panda_gripforce.v1",
        "rig": "panda",
        "coupling": "maximal-coordinate per-substep symmetric projection",
        "collector": "src.pad_wrench.collect_pad_wrench",
        "cases": [run_case(E, F) for E, F in CASES],
        "frozen_reference": "Validated R3 static suspended hold: per-pad Fn approximately F_cmd",
    }
    path = ROOT / "reports/logs/vbd/panda/panda_gripforce.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps(result, indent=2, allow_nan=False))
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
