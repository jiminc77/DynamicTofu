"""Control contract: arm-only IK writes, EFFORT-mode finger force, phase machine.

Contract (pending-approval.md "Control contract and F_g definition"):
- F_g := commanded per-finger normal closure force [N], convention
  `per_finger_normal_mean` (bilateral mean of realized per-finger normals).
- IK writes MASKED to the 7 arm coordinates; finger coords never written by IK.
- control.joint_f zero everywhere except finger DOFs.
- Mimic convention (single-master vs dual-finger) probed at G-N2, then frozen.
- Phases: settle 0.5 / close+hold 0.5 / lift 5 cm in 0.3 / hold 0.2 /
  transport / settle 0.5.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import warp as wp

import newton
from newton import ik

LIFT_HEIGHT_M = 0.05
PHASE_SETTLE_S = 0.5
PHASE_CLOSE_HOLD_S = 0.5
PHASE_LIFT_S = 0.3
PHASE_POSTLIFT_HOLD_S = 0.2
PHASE_FINAL_SETTLE_S = 0.5

# Tool-point offset from LINK7 (hand collapses into link7 at import):
# link7 -> flange 0.107 m + flange -> TCP 0.1034 m ~= 0.2104 m along tool z,
# putting the fingertip grip centre at the tool point (fr3 TCP convention).
# With tool-down quat (1,0,0,0), world tool z = link7 z - offset.
EE_TOOL_OFFSET = (0.0, 0.0, 0.2104)


class ArmIK:
    """IK position+rotation tracking with writes masked to arm coordinates."""

    def __init__(self, model: newton.Model, meta, initial_target_pos, initial_target_quat_wxyz):
        self.model = model
        self.meta = meta
        self.n_coords = model.joint_coord_count
        self.ik_joint_q = wp.array(model.joint_q, shape=(1, self.n_coords))
        self.target_pos = wp.array([wp.vec3(*initial_target_pos)], dtype=wp.vec3)
        self.target_rot = wp.array([wp.vec4(*initial_target_quat_wxyz)], dtype=wp.vec4)
        self.pos_obj = ik.IKObjectivePosition(
            link_index=meta.ee_body_index,
            link_offset=wp.vec3(*EE_TOOL_OFFSET),
            target_positions=self.target_pos,
        )
        self.rot_obj = ik.IKObjectiveRotation(
            link_index=meta.ee_body_index,
            link_offset_rotation=wp.quat_identity(),
            target_rotations=self.target_rot,
        )
        self.limit_obj = ik.IKObjectiveJointLimit(
            joint_limit_lower=model.joint_limit_lower,
            joint_limit_upper=model.joint_limit_upper,
            weight=10.0,
        )
        self.solver = ik.IKSolver(
            model=model,
            n_problems=1,
            objectives=[self.pos_obj, self.rot_obj, self.limit_obj],
            lambda_initial=0.1,
            jacobian_mode=ik.IKJacobianType.ANALYTIC,
        )
        self.iters = 24
        self._arm_coords = np.asarray(meta.arm_coord_indices, dtype=int)

    def solve_to_targets(self, pos_xyz, quat_wxyz) -> np.ndarray:
        """Solve IK and return the FULL coord vector solution (host)."""
        self.target_pos.assign([wp.vec3(*pos_xyz)])
        self.target_rot.assign([wp.vec4(*quat_wxyz)])
        self.solver.step(self.ik_joint_q, self.ik_joint_q, iterations=self.iters)
        return self.ik_joint_q.numpy().reshape(-1)

    def write_arm_targets(self, control: newton.Control, solution: np.ndarray) -> None:
        """Write ONLY the 7 arm coordinates into control.joint_target_q (never fingers)."""
        tq = control.joint_target_q.numpy()
        tq[self._arm_coords] = solution[self._arm_coords]
        control.joint_target_q.assign(tq)


@dataclass
class FingerForceCommand:
    """F_g -> joint_f mapping. Nominal: each finger DOF commanded with -F_g
    (closing direction along the prismatic axis); the calibration constant
    (G-N2) rescales it. Mimic convention: 'dual' commands both finger DOFs;
    'master' commands only fr3_finger_joint1 (probed at G-N2, then frozen)."""

    meta: object
    calibration_gain: float = 1.0
    convention: str = "dual"
    close_sign: float = -1.0  # prismatic axis opens positive; closing is negative effort

    def apply(self, control: newton.Control, f_g_newton: float) -> None:
        jf = control.joint_f.numpy()
        jf[:] = 0.0
        cmd = self.close_sign * self.calibration_gain * float(f_g_newton)
        dofs = self.meta.finger_dof_indices if self.convention == "dual" else self.meta.finger_dof_indices[:1]
        for d in dofs:
            jf[d] = cmd
        control.joint_f.assign(jf)

    def apply_open(self, control: newton.Control, f_open_newton: float = 1.0) -> None:
        """Hold the gripper OPEN (approach/descend phases). EFFORT-mode fingers
        have no holding stiffness and drift shut under arm motion otherwise
        (observed: q 0.04 -> 0.006 during descend, plowing the block)."""
        self.apply(control, -abs(f_open_newton))


@dataclass
class PhaseSchedule:
    """Pre-registered phase machine. Transport profile duration supplied per trial."""

    transport_duration_s: float = 0.0
    timestamps: dict = field(default_factory=dict)

    def __post_init__(self):
        t = 0.0
        self.timestamps = {}
        for name, dur in (
            ("settle", PHASE_SETTLE_S),
            ("close_hold", PHASE_CLOSE_HOLD_S),
            ("lift", PHASE_LIFT_S),
            ("postlift_hold", PHASE_POSTLIFT_HOLD_S),
            ("transport", self.transport_duration_s),
            ("final_settle", PHASE_FINAL_SETTLE_S),
        ):
            self.timestamps[name] = (t, t + dur)
            t += dur
        self.total_s = t
        # judgment window: lift-complete -> settle-end (inclusive)
        self.window = (self.timestamps["lift"][1], self.timestamps["final_settle"][1])

    def phase_at(self, t: float) -> str:
        for name, (a, b) in self.timestamps.items():
            if a <= t < b:
                return name
        return "final_settle"


def assert_control_contract(model: newton.Model, meta) -> dict:
    """Post-finalize assertions (plan): arm POSITION, fingers EFFORT."""
    ke = model.joint_target_ke.numpy()
    kd = model.joint_target_kd.numpy()
    checks = {"finger_effort": True, "arm_position": True}
    for d in meta.finger_dof_indices:
        if not (ke[d] == 0.0 and kd[d] == 0.0):
            checks["finger_effort"] = False
    for d in meta.arm_dof_indices:
        if not ke[d] > 0.0:
            checks["arm_position"] = False
    if not all(checks.values()):
        raise AssertionError(f"control contract violated: {checks}")
    return checks
