# User-supplied consult: rebuild tofu grasping on Newton VBD (2026-08-27 evening)

User decision: the MPM-based result cannot be the basis for the paper — tofu MUST be properly implemented in the simulator. Rebuild on the VBD soft-body path. Use this consult selectively ("필요한 부분만 발췌"). Fracture is explicitly OUT OF SCOPE: use a threshold criterion instead ("임계값 이상이면 깨진다 치면 된다").

## Starting point

`newton/examples/multiphysics/example_proxy_joint_gripper.py` — already a MuJoCo-driven 2-finger gripper grasping a VBD soft body via SolverCoupledProxy (MuJoCo owns rigid gripper; SolverVBD owns the deformable; proxy coupling connects them). Related examples worth reading: `softbody_franka`, `softbody_hanging`, `rigid_soft_contact` (rigid–FEM contact setup).

Run first:
```bash
uv run -m newton.examples proxy_joint_gripper
```
The example already builds a palm + two prismatic fingers and closes them on an `add_soft_grid()` block with a time-ramped target and an effort limit — i.e., it is already "two fingers squeezing a soft box".

## Scene (4 elements)

1. tofu: `add_soft_grid()` (or `add_soft_mesh()`), SolverVBD.
2. gripper: rigid bodies, left/right prismatic joints (MuJoCo or Newton rigid).
3. rigid–soft contact between fingers and tofu.
4. grasp sequence: approach → close → hold → lift → check the tofu comes up.

## Tofu body (prototype values, NOT final material)

```python
builder.add_soft_grid(
    pos=..., rot=wp.quat_identity(), vel=wp.vec3(0,0,0),
    dim_x=..., dim_y=..., dim_z=...,      # our frozen block: 4x4x4 cm
    cell_x=..., cell_y=..., cell_z=...,
    density=1000.0,
    k_mu=8.0e3, k_lambda=8.0e4, k_damp=1.0,
    particle_radius=0.006..0.008,
)
```

Material anchors: firm/extra-firm tofu linear stiffness ≈ 26–28 kPa (Nature s41538-024-00330-6); other preparations E ≈ 4–12 kPa; tofu is nonlinear + viscoelastic (2026, S0022509626001328) — "tofu" is not one parameter set. First target: a **calibrated soft body that reproduces grasp behavior**, not perfect physics.

Poisson ratio matters (high water content → nearly incompressible → side-squeeze should bulge, not lose volume). Lamé conversion: mu = E/(2(1+nu)), lambda = E*nu/((1+nu)(1-2nu)). E=25 kPa, nu=0.45 → mu ≈ 8.6 kPa, lambda ≈ 77.6 kPa (hence the 8e3/8e4 starting values). Do NOT start at nu=0.499 (over-stiff solver); tune within nu = 0.40–0.47.

## Gripper and control

Keep the example's palm + prismatic finger structure and time-ramped close. KEY: for tofu, **force limiting matters more than position target** — treat `effort_limit` as the experimental variable (example default 250 is far too high). Start around `target_ke=2.0e4, target_kd=200.0, effort_limit=20.0` and sweep grip force; with our 64 g block the relevant per-finger range is sub-Newton to a few N.

Friction dominates outcomes: sweep `soft_contact_mu` ∈ {0.3, 0.5, 0.7, 1.0} (example uses soft_contact_mu=2.0 and finger shape mu=0.7).

## Gravity, table, lift DOF

The example has gravity=(0,0,0) — add gravity=(0,0,-9.81) and `builder.add_ground_plane()`, place tofu just above z=0. First prototype: NO robot arm — floating gripper:
```text
world → prismatic Z (lift) → palm → left/right prismatic fingers   (3 DOF total)
```
This is far easier to debug than a 7-DOF arm. (Full-Franka port comes after the material is validated.)

## Grasp state machine

Phase 1 approach (t<1.0) → Phase 2 close (t<2.0) → Phase 3 hold (t<2.5) → Phase 4 lift. Keep as an explicit state machine, not a single close ramp.

## Solver / stability starting values

```python
SolverVBD(model=v, iterations=30,
          particle_enable_self_contact=False,
          particle_enable_tile_solve=False,
          rigid_compliant_alm=True,
          rigid_body_particle_contact_buffer_size=1024)
soft_contact_ke=5.0e4; soft_contact_kd=1.0e-3; soft_contact_kf=1.0e3; soft_contact_mu=0.5
fps = 60; substeps = 8–16   # stiff contact needs substeps
```
SolverCoupledProxy(entries=[SolverMuJoCo(...), SolverVBD(...)]) as in the example.

## Damage WITHOUT fracture (user decision)

VBD tetrahedral connectivity does not fracture (no topology change). Do NOT implement fracture. Staged observables instead:
- Stage 1: grasp success / slip / deformation.
- Stage 2: maximum principal strain (per-tet Green strain).
- Stage 3: damage criterion = threshold on that strain ("임계값 이상이면 깨진 것으로 판정"). Anchor the threshold to measured tofu failure strains (tension ~10–20%; compression fracture strain ~45–54%) and pre-register it.

## Metrics

lift_success (tofu COM z above threshold after lift), COM height, max deformation, max principal strain, gripper force, slip distance, permanent deformation. Target phase picture: grip force axis with slip (low) / successful grasp (middle) / damage-threshold exceeded (high) — find the operating region: minimum force + no slip + minimal deformation.

## Recommended implementation order

1. Run `proxy_joint_gripper` as-is. 2. Replace soft object with tofu params. 3. gravity + ground plane. 4. keep gripper close logic. 5. add vertical lift joint. 6. control state machine. 7. record COM / finger separation / max deformation / contact force. First milestone: **"pick up the tofu without crushing it" clip**. Fracture explicitly deferred.
