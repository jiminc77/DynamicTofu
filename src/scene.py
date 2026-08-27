"""Scene builder: 4x4x4 cm elastoplastic block + table + full Franka Panda.

Contract (pending-approval.md, "File-level changes" + control contract):
- SolverImplicitMPM.register_custom_attributes(builder) BEFORE particles.
- Full Franka fr3_franka_hand.urdf (download_asset), floating=False,
  collapse_fixed_joints=True; finger pads later registered as DISTINCT
  MPM collider ids (coupling.py).
- Block: 4x4x4 cm, E=7 kPa, nu=0.45, rho=1000 kg/m^3, per-particle
  mpm:yield_stress = sigma_Y (deviatoric, Pa).
- DOFs resolved BY LABEL, never index arithmetic. Fingers set to EFFORT
  (target_ke == target_kd == 0) pre-finalize; arm gets explicit POSITION
  gains (URDF carries no stiffness).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import warp as wp

import newton
from newton.solvers import SolverImplicitMPM

# --- frozen scene constants (recorded in every config block) ---------------
BLOCK_EDGE_M = 0.04
BLOCK_E_PA = 7.0e3
BLOCK_NU = 0.45
BLOCK_RHO = 1000.0
BLOCK_MPM_DAMPING = 0.001          # elastic damping relaxation time (s); template default
VOXEL_SIZE_M = 0.005               # 8 cells across the block edge
PARTICLES_PER_CELL_AXIS = 2.0      # ~16 particles across the edge, ~4.1k total

TABLE_TOP_Z = 0.20
TABLE_HALF = (0.35, 0.45, 0.10)    # table slab half-extents; top at TABLE_TOP_Z
BLOCK_CENTER = (0.0, -0.5, TABLE_TOP_Z + 0.5 * BLOCK_EDGE_M)
FRANKA_BASE_XFORM_P = (-0.5, -0.5, -0.1)     # template placement
FRANKA_HOME_Q6 = [0.0, 0.0, 0.0, -1.59695, 0.0, 2.5307]

ARM_TARGET_KE = 3000.0
ARM_TARGET_KD = 150.0
PAD_FRICTION_MU = 1.0  # rubber fingertip pads; recorded in every config block

ARM_JOINT_LABELS = [f"fr3_joint{i}" for i in range(1, 8)]
FINGER_JOINT_LABELS = ["fr3_finger_joint1", "fr3_finger_joint2"]
FINGER_BODY_SUBSTRINGS = ("leftfinger", "rightfinger")

_DOF_PER_TYPE = {
    newton.JointType.REVOLUTE: 1,
    newton.JointType.PRISMATIC: 1,
    newton.JointType.FIXED: 0,
    newton.JointType.FREE: 6,
    newton.JointType.BALL: 3,
    newton.JointType.D6: 6,
}


@dataclass
class SceneMeta:
    asset_dir: str = ""
    urdf_path: str = ""
    arm_joint_indices: list = field(default_factory=list)
    finger_joint_indices: list = field(default_factory=list)
    arm_dof_indices: list = field(default_factory=list)      # qd-space
    finger_dof_indices: list = field(default_factory=list)
    arm_coord_indices: list = field(default_factory=list)    # q-space (== dof for 1-dof joints)
    finger_coord_indices: list = field(default_factory=list)
    finger_body_indices: list = field(default_factory=list)  # [left, right]
    ee_body_index: int = -1
    rigid_body_range: tuple = (0, 0)
    particle_count: int = 0
    sigma_y_pa: float = 0.0
    scene_constants: dict = field(default_factory=dict)


def _joint_dof_offsets(builder: newton.ModelBuilder) -> list[int]:
    """Cumulative qd-offsets per joint, derived from joint types (order of add)."""
    offsets, acc = [], 0
    for jt in builder.joint_type:
        offsets.append(acc)
        acc += _DOF_PER_TYPE.get(jt, 1)
    return offsets


def _joint_coord_offsets(builder: newton.ModelBuilder) -> list[int]:
    coord_per_type = dict(_DOF_PER_TYPE)
    coord_per_type[newton.JointType.FREE] = 7
    coord_per_type[newton.JointType.BALL] = 4
    offsets, acc = [], 0
    for jt in builder.joint_type:
        offsets.append(acc)
        acc += coord_per_type.get(jt, 1)
    return offsets


# APPROVED material completion P2 (external GO 2026-08-27, DECISIONS.md):
# yield_pressure = 0.85 x sigma_Y, tensile_yield_ratio = 1.0, viscosity = 20 Pa*s.
# Protocol constants; enter every JSON config block and every gate receipt.
YIELD_PRESSURE_FACTOR = 0.85
TENSILE_YIELD_RATIO = 1.0
VISCOSITY_PA_S = 20.0


def _add_block(builder, lo, res, cell, particle_mass, radius, sigma_y_pa, approved_material: bool):
    attrs = {
        "mpm:young_modulus": BLOCK_E_PA,
        "mpm:poisson_ratio": BLOCK_NU,
        "mpm:yield_stress": float(sigma_y_pa),
        "mpm:damping": BLOCK_MPM_DAMPING,
    }
    if approved_material:
        attrs["mpm:yield_pressure"] = YIELD_PRESSURE_FACTOR * float(sigma_y_pa)
        attrs["mpm:tensile_yield_ratio"] = TENSILE_YIELD_RATIO
        attrs["mpm:viscosity"] = VISCOSITY_PA_S
    builder.add_particle_grid(
        pos=wp.vec3(*lo.tolist()),
        rot=wp.quat_identity(),
        vel=wp.vec3(0.0),
        dim_x=res + 1,
        dim_y=res + 1,
        dim_z=res + 1,
        cell_x=cell,
        cell_y=cell,
        cell_z=cell,
        mass=particle_mass,
        jitter=0.0,  # deterministic lattice; seed variation enters via pose jitter + solver sampling
        radius_mean=radius,
        custom_attributes=attrs,
    )


# sensor_format_pad (user ruling 2026-08-27): a tactile-sensor-format flat
# fingertip face (Paxini-class), 30x30 mm x 3 mm, placed 1 mm inward of the
# stock pad's innermost face (finger-local +y is the inward contact axis;
# stock inner face ~= local y 0.026). Diagnostics only.
SENSOR_PAD_HALF = (0.015, 0.0015, 0.015)
SENSOR_PAD_LOCAL_P = (0.0, 0.0255, 0.028)


def build_scene(sigma_y_pa: float, *, seed: int = 0, pose_jitter_m: float = 0.0, include_block: bool = True, material_completion: bool = True, sensor_pad: bool = False):
    """material_completion=True applies the SIGNED-OFF yield_pressure = 2*sigma_Y;
    False reproduces the pre-sign-off baseline (archival probes only).
    sensor_pad=True adds the diagnostic sensor_format_pad to each finger."""
    """Build the model. Returns (builder-finalized model, SceneMeta, builder)."""
    rng = np.random.default_rng(np.random.SeedSequence([1234, int(seed)]))

    builder = newton.ModelBuilder()
    builder.default_shape_cfg.mu = 0.5

    # MPM custom attributes MUST be registered before any particles are added.
    SolverImplicitMPM.register_custom_attributes(builder)

    # --- Franka (rigid articulation) ---------------------------------------
    asset_path = newton.utils.download_asset("franka_emika_panda")
    urdf_path = str(asset_path / "urdf" / "fr3_franka_hand.urdf")
    rigid_body_start = builder.body_count
    builder.add_urdf(
        urdf_path,
        xform=wp.transform(FRANKA_BASE_XFORM_P, wp.quat_identity()),
        floating=False,
        scale=1.0,
        enable_self_collisions=False,
        collapse_fixed_joints=True,
        force_show_colliders=False,
    )
    rigid_body_end = builder.body_count
    builder.joint_q[: len(FRANKA_HOME_Q6)] = FRANKA_HOME_Q6
    # fingers start OPEN (0.04 m) so the grasp is a commanded act, not an initial condition
    builder.joint_q[-2:] = [0.04, 0.04]

    # --- label-resolved joint/DOF masks (never index arithmetic) -----------
    label_to_joint = {lbl.split("/")[-1]: i for i, lbl in enumerate(builder.joint_label)}
    missing = [l for l in ARM_JOINT_LABELS + FINGER_JOINT_LABELS if l not in label_to_joint]
    if missing:
        raise RuntimeError(f"joint labels not found in URDF import: {missing}; have {builder.joint_label}")
    arm_joints = [label_to_joint[l] for l in ARM_JOINT_LABELS]
    finger_joints = [label_to_joint[l] for l in FINGER_JOINT_LABELS]

    dof_off = _joint_dof_offsets(builder)
    coord_off = _joint_coord_offsets(builder)
    arm_dofs = [dof_off[j] for j in arm_joints]
    finger_dofs = [dof_off[j] for j in finger_joints]
    arm_coords = [coord_off[j] for j in arm_joints]
    finger_coords = [coord_off[j] for j in finger_joints]

    # --- actuation modes ----------------------------------------------------
    # Fingers: EFFORT := drive present with both gains zero (enums.py:351-372).
    for d in finger_dofs:
        builder.joint_target_ke[d] = 0.0
        builder.joint_target_kd[d] = 0.0
    # Arm: URDF has damping only (no stiffness); set explicit POSITION gains.
    for d in arm_dofs:
        builder.joint_target_ke[d] = ARM_TARGET_KE
        builder.joint_target_kd[d] = ARM_TARGET_KD

    # --- finger bodies / EE body -------------------------------------------
    finger_bodies = []
    for key in FINGER_BODY_SUBSTRINGS:
        matches = [i for i, lbl in enumerate(builder.body_label) if key in lbl]
        if len(matches) != 1:
            raise RuntimeError(f"expected exactly one body matching '{key}', got {matches}")
        finger_bodies.append(matches[0])
    # After collapse_fixed_joints the hand merges into link7; anchor IK there.
    ee_matches = [i for i, lbl in enumerate(builder.body_label) if "link7" in lbl]
    if len(ee_matches) != 1:
        raise RuntimeError(f"expected exactly one link7 body, got {ee_matches}: {builder.body_label}")
    ee_body = ee_matches[0]

    # --- sensor_format_pad (diagnostics only) ------------------------------
    sensor_pad_shapes = []
    if sensor_pad:
        for fb in finger_bodies:
            sid = builder.add_shape_box(
                fb,
                xform=wp.transform(SENSOR_PAD_LOCAL_P, wp.quat_identity()),
                hx=SENSOR_PAD_HALF[0], hy=SENSOR_PAD_HALF[1], hz=SENSOR_PAD_HALF[2],
            )
            sensor_pad_shapes.append(sid)

    # --- table + ground -----------------------------------------------------
    builder.add_shape_box(
        -1,
        xform=wp.transform((BLOCK_CENTER[0], BLOCK_CENTER[1], TABLE_TOP_Z - TABLE_HALF[2]), wp.quat_identity()),
        hx=TABLE_HALF[0],
        hy=TABLE_HALF[1],
        hz=TABLE_HALF[2],
    )
    builder.add_ground_plane()

    # --- MPM block (AFTER register_custom_attributes) -----------------------
    jitter_xy = rng.uniform(-pose_jitter_m, pose_jitter_m, size=2) if pose_jitter_m > 0 else np.zeros(2)
    lo = np.array(BLOCK_CENTER) - 0.5 * BLOCK_EDGE_M
    lo[:2] += jitter_xy
    res = int(np.ceil(PARTICLES_PER_CELL_AXIS * BLOCK_EDGE_M / VOXEL_SIZE_M))
    cell = BLOCK_EDGE_M / res
    particle_mass = float(cell**3 * BLOCK_RHO)
    radius = 0.5 * cell
    particle_start = builder.particle_count
    if include_block:
        _add_block(builder, lo, res, cell, particle_mass, radius, sigma_y_pa, material_completion)

    particle_count = builder.particle_count - particle_start

    model = builder.finalize()

    # finger-pad friction (rubber pads); MPM collider discovery reads shape materials
    mu = model.shape_material_mu.numpy()
    sb = model.shape_body.numpy()
    for i in range(model.shape_count):
        if sb[i] in finger_bodies:
            mu[i] = PAD_FRICTION_MU
    model.shape_material_mu.assign(mu)

    # --- post-finalize control-contract assertions (plan, control contract) --
    ke = model.joint_target_ke.numpy()
    kd = model.joint_target_kd.numpy()
    for d in finger_dofs:
        assert ke[d] == 0.0 and kd[d] == 0.0, f"finger dof {d} not EFFORT: ke={ke[d]} kd={kd[d]}"
    for d in arm_dofs:
        assert ke[d] > 0.0, f"arm dof {d} has no position gain"

    meta = SceneMeta(
        asset_dir=str(asset_path),
        urdf_path=urdf_path,
        arm_joint_indices=arm_joints,
        finger_joint_indices=finger_joints,
        arm_dof_indices=arm_dofs,
        finger_dof_indices=finger_dofs,
        arm_coord_indices=arm_coords,
        finger_coord_indices=finger_coords,
        finger_body_indices=finger_bodies,
        ee_body_index=ee_body,
        rigid_body_range=(rigid_body_start, rigid_body_end),
        particle_count=particle_count,
        sigma_y_pa=float(sigma_y_pa),
        scene_constants={
            "block_edge_m": BLOCK_EDGE_M,
            "block_E_pa": BLOCK_E_PA,
            "block_nu": BLOCK_NU,
            "block_rho": BLOCK_RHO,
            "mpm_damping_s": BLOCK_MPM_DAMPING,
            "voxel_size_m": VOXEL_SIZE_M,
            "particles_per_cell_axis": PARTICLES_PER_CELL_AXIS,
            "particle_count": particle_count,
            "table_top_z_m": TABLE_TOP_Z,
            "block_center_m": list(BLOCK_CENTER),
            "pose_jitter_m": pose_jitter_m,
            "seed": int(seed),
            "seed_rng_derivation": "np.random.SeedSequence([1234, seed]) -> xy pose jitter",
            "arm_target_ke": ARM_TARGET_KE,
            "arm_target_kd": ARM_TARGET_KD,
            "pad_friction_mu": PAD_FRICTION_MU,
            "default_shape_mu": 0.5,  # frozen protocol constant (external sign-off 2026-08-27)
            "sensor_format_pad": (dict(half_extents_m=list(SENSOR_PAD_HALF), local_p=list(SENSOR_PAD_LOCAL_P),
                                       face_mm=[30, 30]) if sensor_pad else None),
            "yield_pressure_pa": YIELD_PRESSURE_FACTOR * float(sigma_y_pa) if material_completion else None,
            "yield_pressure_factor": YIELD_PRESSURE_FACTOR if material_completion else None,
            "tensile_yield_ratio": TENSILE_YIELD_RATIO if material_completion else None,
            "viscosity_pa_s": VISCOSITY_PA_S if material_completion else None,
        },
    )
    return model, meta, builder
