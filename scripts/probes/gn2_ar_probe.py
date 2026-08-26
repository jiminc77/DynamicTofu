"""G-N2 AR probe (blocker): per-finger action-reaction gate AR-1..AR-4.

Sequence: IK the arm to the grasp pose over the block, descend, close the
EFFORT-mode fingers at F_g, hold; evaluate AR-1/AR-2 on the held grasp;
repeat block-absent for AR-3; AR-4 logged. Optional sensitivity across
proxy iterations {1,2,4}.

Run: cd newton && uv run --no-sync python ../scripts/probes/gn2_ar_probe.py [--sensitivity]
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import numpy as np
import warp as wp

import newton

from src.scene import BLOCK_CENTER, VOXEL_SIZE_M, build_scene
from src.coupling import ar_check, build_coupled_solver, harvested_body_wrenches, mpm_entry_state, node_reduction_per_body
from src.control import ArmIK, FingerForceCommand, assert_control_contract
from src.health import HealthAccumulator

FRAME_DT = 0.005  # 200 Hz probe tick (same as the E2 tick contract)
GRASP_QUAT_WXYZ = (1.0, 0.0, 0.0, 0.0)  # template convention: vec4 (x,y,z,w)=(1,0,0,0), tool down
PREGRASP_Z = 0.32
GRASP_Z = 0.22  # tool point == block centre; pad surfaces straddle the block
F_G_PROBE = 0.5  # gentle probe grasp; crush behaviour is a separate probe


class Rig:
    def __init__(self, include_block: bool, proxy_iterations: int = 1, sigma_y: float = 3333.0, seed: int = 0, material_completion: bool = False):
        self.model, self.meta, _ = build_scene(
            sigma_y, seed=seed, include_block=include_block, material_completion=material_completion
        )
        assert_control_contract(self.model, self.meta)
        if include_block:
            self.solver = build_coupled_solver(
                self.model, self.meta, voxel_size=VOXEL_SIZE_M, proxy_iterations=proxy_iterations
            )
            self.mpm = self.solver.solver("mpm")
            self.rigid_substeps = 1  # substepping handled inside the coupled entry
        else:
            # block-absent twin: plain SolverMuJoCo, 4 manual substeps to mirror
            # the coupled entry's rigid_substeps=4. (The coupled wrapper does not
            # forward joint_f when it has no proxies - verified empirically.)
            from newton.solvers import SolverMuJoCo

            self.solver = SolverMuJoCo(model=self.model, use_mujoco_contacts=False, njmax=256, nconmax=256)
            self.mpm = None
            self.rigid_substeps = 4
        self.state = self.model.state()
        self.control = self.model.control()
        # tight speculative gap: the default 0.1 m velocity-adapted extension
        # engages table-fingertip contacts centimetres early during the descend
        spec = newton.CollisionPipeline.SpeculativeContactConfig(max_speculative_extension=0.005)
        self.pipeline = newton.CollisionPipeline(
            self.model, soft_contact_max=0, rigid_contact_max=512, speculative_config=spec
        )
        self.contacts = self.pipeline.contacts()
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state)
        self.ik = ArmIK(self.model, self.meta, (BLOCK_CENTER[0], BLOCK_CENTER[1], PREGRASP_Z), GRASP_QUAT_WXYZ)
        self.fingers = FingerForceCommand(self.meta)
        self.health = HealthAccumulator()
        self.t = 0.0
        # initial arm targets = current coords (hold pose)
        tq = self.control.joint_target_q.numpy()
        tq[:] = self.model.joint_q.numpy()
        self.control.joint_target_q.assign(tq)
        # hold the gripper open until a grasp is commanded
        self.fingers.apply_open(self.control)

    def step(self, n: int, health_every: int = 20):
        for i in range(n):
            self.state.clear_forces()
            self.pipeline.collide(self.state, self.contacts, dt=FRAME_DT)
            if self.mpm is None:
                sub_dt = FRAME_DT / self.rigid_substeps
                for _ in range(self.rigid_substeps):
                    self.solver.step(self.state, self.state, self.control, self.contacts, sub_dt)
            else:
                self.solver.step(self.state, self.state, self.control, self.contacts, FRAME_DT)
            self.t += FRAME_DT
            if health_every and (i + 1) % health_every == 0 and self.model.particle_count and self.mpm is not None:
                q = self.state.particle_q.numpy()
                qd = self.state.particle_qd.numpy()
                jp = self.jp()
                self.health.check_tick(q, qd, jp, mpm_solver=self.mpm)

    def jp(self):
        """Jp from the MPM entry state (parent-state Jp is never updated)."""
        return mpm_entry_state(self.solver).mpm.particle_Jp.numpy()

    def move_ee(self, pos, duration_s: float):
        """Solve IK once for the target and linearly interpolate arm coord targets."""
        start = self.control.joint_target_q.numpy().copy()
        sol = self.ik.solve_to_targets(pos, GRASP_QUAT_WXYZ)
        arm = np.asarray(self.meta.arm_coord_indices, dtype=int)
        n = max(1, int(duration_s / FRAME_DT))
        for k in range(n):
            alpha = (k + 1) / n
            tq = self.control.joint_target_q.numpy()
            tq[arm] = (1 - alpha) * start[arm] + alpha * sol[arm]
            self.control.joint_target_q.assign(tq)
            self.step(1)

    def realized_tool(self) -> np.ndarray:
        """Realized tool point from the current dynamic state (FK on link7)."""
        from src.control import EE_TOOL_OFFSET

        bq = self.state.body_q.numpy()[self.meta.ee_body_index]
        x, y, z, w = bq[3:7]
        u = np.array([x, y, z]); v = np.array(EE_TOOL_OFFSET)
        return bq[:3] + 2 * np.dot(u, v) * u + (w * w - np.dot(u, u)) * v + 2 * w * np.cross(u, v)

    def move_ee_converge(self, pos, tol_m: float = 0.003, max_iters: int = 12):
        """Closed-loop settle onto `pos`: re-solve IK on a bias-corrected target
        until the REALIZED tool point is within tol (kills gravity sag)."""
        target = np.asarray(pos, dtype=float)
        bias = np.zeros(3)
        err = None
        arm = np.asarray(self.meta.arm_coord_indices, dtype=int)
        for _ in range(max_iters):
            sol = self.ik.solve_to_targets((target + bias).tolist(), GRASP_QUAT_WXYZ)
            tq = self.control.joint_target_q.numpy()
            tq[arm] = sol[arm]
            self.control.joint_target_q.assign(tq)
            self.step(int(0.25 / FRAME_DT))
            realized = self.realized_tool()
            err = target - realized
            if np.linalg.norm(err) <= tol_m:
                break
            bias += 0.8 * err
        return float(np.linalg.norm(err)) if err is not None else float("nan")

    def finger_q(self) -> np.ndarray:
        return self.state.joint_q.numpy()[np.asarray(self.meta.finger_coord_indices, dtype=int)]

    def pad_normals_world(self) -> dict:
        """Declared pad normals, PAD-OUTWARD (block -> pad) convention.

        The harvested F_b is the reaction force ON the finger; a squeezing
        grasp pushes each finger outward, so compressive contact satisfies
        F_b . n_outward > 0. Convention recorded in the config block as
        pad_normal_convention: block_to_pad_outward.
        """
        bq = self.state.body_q.numpy()
        left, right = self.meta.finger_body_indices
        p_l, p_r = bq[left][:3], bq[right][:3]
        axis = p_l - p_r
        norm = np.linalg.norm(axis)
        axis = axis / norm if norm > 1e-9 else np.array([0.0, -1.0, 0.0])
        return {left: axis, right: -axis}


def run_grasp(include_block: bool, proxy_iterations: int = 1):
    rig = Rig(include_block, proxy_iterations)
    rig.step(int(0.5 / FRAME_DT))                       # settle
    rig.move_ee((BLOCK_CENTER[0], BLOCK_CENTER[1], PREGRASP_Z), 1.5)
    rig.move_ee((BLOCK_CENTER[0], BLOCK_CENTER[1], GRASP_Z), 1.0)
    resid = rig.move_ee_converge((BLOCK_CENTER[0], BLOCK_CENTER[1], GRASP_Z))
    print(f"grasp-pose servo residual: {resid*1000:.2f} mm")
    rig.step(int(0.3 / FRAME_DT))                       # settle at grasp pose
    deflection_trace = []
    n_ramp = int(0.3 / FRAME_DT)                        # ramp the close to avoid slamming the soft block
    for k in range(n_ramp):
        rig.fingers.apply(rig.control, F_G_PROBE * (k + 1) / n_ramp)
        rig.step(1)
        deflection_trace.append(rig.finger_q().tolist())
    n_close = int(1.7 / FRAME_DT)
    for _ in range(n_close):
        rig.step(1)
        deflection_trace.append(rig.finger_q().tolist())
    return rig, np.array(deflection_trace)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sensitivity", action="store_true")
    args = ap.parse_args()
    t0 = time.time()

    rig, defl_block = run_grasp(include_block=True)
    bq = rig.state.body_q.numpy()
    reduced = node_reduction_per_body(
        rig.mpm, rig.state, bq, rig.model.body_com.numpy(), FRAME_DT
    )
    harvested = harvested_body_wrenches(rig.solver)
    ar = ar_check(reduced, harvested, rig.meta.finger_body_indices, rig.pad_normals_world())
    health_report = rig.health.report()

    # AR-3: block-absent twin
    rig_nb, defl_nb = run_grasp(include_block=False)
    # deflection = finger q trajectory; noise floor = std of block-absent trace
    final_block = defl_block[-1]
    final_nb = defl_nb[-1]
    noise_floor = float(np.std(defl_nb[len(defl_nb) // 2 :], axis=0).max())
    deflection_delta = float(np.abs(final_block - final_nb).max())
    ar3_pass = deflection_delta > 10.0 * max(noise_floor, 1e-9)

    out = {
        "ar1_ar2": ar.to_dict(),
        "ar3": {
            "deflection_delta_m": deflection_delta,
            "noise_floor_m": noise_floor,
            "ratio": deflection_delta / max(noise_floor, 1e-12),
            "pass": bool(ar3_pass),
            "finger_q_block": final_block.tolist(),
            "finger_q_absent": final_nb.tolist(),
        },
        "health": health_report,
        "f_g_probe_n": F_G_PROBE,
        "frame_dt_s": FRAME_DT,
        "wall_s": time.time() - t0,
    }

    if args.sensitivity:
        sens = {}
        for it in (2, 4):
            r2, _ = run_grasp(include_block=True, proxy_iterations=it)
            bq2 = r2.state.body_q.numpy()
            red2 = node_reduction_per_body(r2.mpm, r2.state, bq2, r2.model.body_com.numpy(), FRAME_DT)
            harv2 = harvested_body_wrenches(r2.solver)
            ar2r = ar_check(red2, harv2, r2.meta.finger_body_indices, r2.pad_normals_world())
            fl, fr = r2.meta.finger_body_indices
            sens[f"iterations_{it}"] = {
                "normal_L_n": ar2r.per_finger[str(fl)]["normal_component_n"],
                "normal_R_n": ar2r.per_finger[str(fr)]["normal_component_n"],
                "ar1_pass": ar2r.ar1_pass,
            }
        out["sensitivity"] = sens

    os.makedirs(os.path.join(os.path.dirname(__file__), "..", "..", "reports", "logs"), exist_ok=True)
    out_path = os.path.join(os.path.dirname(__file__), "..", "..", "reports", "logs", "gn2-ar-probe.json")
    with open(out_path, "w") as fh:
        json.dump(out, fh, indent=2)
    print(json.dumps({k: out[k] for k in ("ar3", "health", "wall_s")}, indent=2))
    print("AR1:", "PASS" if ar.ar1_pass else "FAIL", "| AR2:", "PASS" if ar.ar2_pass else "FAIL",
          "| AR3:", "PASS" if ar3_pass else "FAIL", "| AR4 residual:", ar.global_residual_n)
    ok = ar.ar1_pass and ar.ar2_pass and ar3_pass and health_report["clean"]
    print("AR PROBE:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
