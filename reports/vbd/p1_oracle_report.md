# V-track Day-1 (second consult): P0 discriminators + P1 pure-VBD Coulomb oracle

Spec: reports/consult-vbd2.md. Rig: src/vbd_rig2.py (pure SolverVBD, no proxy). Artifacts: reports/logs/vbd/p0_discriminators.json, p1_oracle.json, clip reports/media/p1_oracle_E100_fail.mp4.

## Root cause (STRONGLY SUPPORTED, not fully isolated) and the fix

The consult diagnoses the V-2 stiff-oracle ejection as a **SolverCoupledProxy actuator-semantics mismatch**: the finger joints were passed as PROXY JOINTS to the pinned SolverVBD, which does NOT honor joint_effort_limit -> an UNCAPPED PD (target_ke 2e4) drove ~200-340 N/finger toward a deep close target, ejecting the block (peak 0.3-2.5 m; reports/logs/vbd/lever0_*.json). Switching to **pure SolverVBD** (finger force via Control.joint_f target_ke=0) + enable_rigid_soft_full_surface_contact=True **removes the ejection** (block gripped + lifted 30-43 mm at 2 N, never launched).

**Isolation caveat (per review):** the pure-VBD run changes THREE things at once vs V-2 (control mode, proxy removal, full-surface contact), so it demonstrates the *combined* fix but does NOT isolate proxy-joint actuation as the sole cause. The decisive **body-proxy-only discriminator (SolverCoupledProxy with joints=())** and the per-run source-vs-proxy finger q / per-pad Fn / penetration / COM-velocity logs the P0 spec requires were NOT run -> **UNRESOLVED**. Claim scoped to "strongly supported by the eject-vs-no-eject contrast," not "confirmed isolated."

## Corrected facts (recorded in DECISIONS)

- soft_contact_kf is DEAD (unused in the VBD rigid-soft path; my kf {1e3-1e6} ladder was bit-identical). Axis retired.
- soft_contact_mu is NOT the friction knob; friction uses avg_mu = sqrt(pad_shape_mu0 * mu1) (geometric mean). ke mixes arithmetically with pad shape ke. BOTH sides set to the pair values.
- friction_epsilon (SolverVBD ctor, velocity-regularized Coulomb) DOES work and is the creep knob: at E=100 kPa/2 N, slip 50 mm (eps 1e-2 default) -> **16.7 mm (eps 2e-4, optimal)** -> 30.9 mm (eps 2e-6, too small). The consult's 2e-4 is confirmed optimal.
- Resolution: BLOCK_CELL 10 mm at dim 4 (5 verts/edge); the P1 oracle uses h=8 mm (dim 5). Architecture precedent: examples/vbd/example_vbd_gripper_soft_grid.py.
- joint_f sign (first-frame check): +joint_f along the inward finger axes CLOSES.

## P1 HARD GATE: FAIL

| E | grip force | hold slip (5 s) | peak rise | final rise | held <2 mm |
|---|---|---|---|---|---|
| 100 kPa | 0.45 N (recipe) | 50.1 mm | 3.9 mm | 0.9 mm (drop) | no |
| 100 kPa | 2.0 N | ~16.7-21 mm | 30.9 mm | 29.1 mm | no |
| 200 kPa | 0.45 N | 50.1 mm | 4.8 mm | 1.0 mm (drop) | no |
| 200 kPa | 2.0 N | 20.5 mm | 42.6 mm | 29.7 mm | no |

**At the exact recipe (substeps=20), neither 100 nor 200 kPa holds <2 mm slip** -> hard gate FAILS as specified. Per the directive: STOP, do not touch tofu, escalate.

**BUT the substep-doubling check is decisive (p1_oracle_full.json):** at E=100 kPa/2 N, doubling substeps 20 -> 40 drops the hold slip from **17.68 mm to 1.01 mm** (UNDER the 2 mm bar). So the residual creep is largely a **temporal-resolution artifact**, and the pure-VBD architecture CAN meet the acceptance with adequate substeps + grip force. The substep-doubling INVARIANCE criterion itself fails at 20 (17.68 vs 1.01), flagging substeps=20 as under-resolved for this stiff contact. Viable operating point for review: substeps >= 40 and grip force above the 0.45 N recipe (0.45 N still drops via the ~3 mm margin-hover; 2 N engages).

## Diagnosis of the residual failure

1. The **0.45 N recipe force is too light** for this contact stack: with soft_contact_margin=0.01, the pads hover ~3 mm off the block (in the margin) and the block is only weakly held -> it drops (50 mm slip). Higher force (2 N) makes the pads engage and the block partially lifts (30-43 mm) but still creeps ~17-21 mm over the 5 s hold.
2. The residual ~17 mm creep at eps=2e-4 is above the <2 mm bar. Candidate levers (for external decision): smaller contact margin so 0.45 N actually grips; larger/deeper pads; higher iterations/substeps; or accept a higher grip force than 0.45 N as the operating point. All are architecture/parameter choices for review.

## Unresolved acceptance measurements (per review)

The committed p1_oracle.json holds summary metrics + hold-slip; the reproducible runner scripts/vbd/p1_oracle.py (-> p1_oracle_full.json) persists full per-run series + pre-lift XY excursion + substep-doubling. Still NOT separately measured/reported: per-pad Fn (0.45 +/-10% acceptance), max penetration / margin-hover distance, and the body-proxy-only P0 discriminator. These are stiff-oracle-only measurements deferred to the external review's chosen lever; none touches tofu.

## Status

The pure-VBD architecture removes the ejection and the friction_epsilon=2e-4 fix is confirmed optimal, but the oracle does not meet the <2 mm acceptance at E=100-200 kPa, and P0 isolation + several acceptance measurements are UNRESOLVED. **STOPPING for external review before the Day-2 tofu sweep**, per the hard-gate directive.
