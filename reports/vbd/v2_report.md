# V-2 VBD tofu prototype — force sweep + hard-milestone result

Rig: src/vbd_rig.py (4x4x4 cm VBD tofu, E=25 kPa nu=0.45 -> k_mu 8.6e3 k_lambda 77.6e3, density 1000, k_damp 1.0; floating 3-DOF gripper world->Z-prismatic->palm->2 finger prismatics; gravity -9.81 + ground; SolverCoupledProxy MuJoCo+VBD; VBD it=30 substeps 12; contact ke5e4 kd1e-3 kf1e3). Controller: force-limited effort on the prismatic fingers (effort_limit = per-finger grip force). Artifacts: reports/logs/vbd/v2_sweep.json, clip reports/media/v2_slip_2N.mp4.

## HARD MILESTONE: NOT MET (tofu lifts but does not hold >= 5 s)

Spec sweep (per-finger effort, soft_contact_mu 0.5), + extended checks:

| grip force | mu | lift | peak COM rise | post-lift hold (min) | outcome |
|---|---|---|---|---|---|
| 0.3-2.0 N | 0.5 | 1 s | ~41 mm | 4.6 mm (rest) | slip |
| 5.0 N | 0.5 | 1 s | 41 mm | 11.5 mm | slip (bulge) |
| 8.0 N | 1.0 | 1 s | **59 mm (full)** | 4.5 mm | slip (bulge, bbox 0.057) |
| 2.0 N | 1.0 | 1 s | 41 mm | 4.6 mm | slip |
| 1.2/2.0 N | 0.7 | 3 s (gentle) | 17-18 mm | 4.6 mm | slip (worse) |
| 1.2/2.0 N | 2.0 (example) | 1 s | 17 mm | 4.6 mm | slip |

The tofu is grasped and **lifts** (peak 17-59 mm depending on force/friction — already beyond MPM, which never lifted), but in every case it **slips/extrudes out of the grip during the hold** and returns to the ground (post-lift COM rise ~4.6 mm = rest). No sweet spot exists in the swept space.

## Failure mode (from traces + reports/media/v2_slip_2N.mp4)

- **Low force**: friction slip — the tofu slides down out of the grip under gravity.
- **High force**: elastic **extrusion/bulge** — the tofu is compressed in the grip direction and bulges around the finger pads (bbox y grows 0.04 -> 0.059 at 8 N), then the bulged material escapes the grip. Higher friction (mu up to 2.0) and slower lift do not rescue it (slower lift is worse: more time to slip).

## Critical scientific implication

The VBD rebuild was expected to hold the tofu where MPM could not. Instead, a **proper elastic model at a FIRMER stiffness (E=25 kPa vs MPM 7 kPa) ALSO cannot fingertip-pinch-and-hold the tofu**, across a wide force/friction/speed range. **Two independent solvers now agree** that soft-tofu fingertip pinch-lift fails — this CONFIRMS (rather than refutes) the empty-band difficulty and is a strong cross-solver result.

## Candidate levers for the external decision (STOP + escalate per directive)

1. **Firmer material (E variants / material axis)**: E=25 kPa is the "firm anchor"; extra-firm tofu or higher E may hold. The minimum E that yields a static hold is the key unknown — a short E sweep {25, 50, 100, 200 kPa} at a fixed moderate force would locate it.
2. **Geometry**: larger/wrapping pads. (NB: the user previously REJECTED the bottom-shelf/spatula support for MPM as bypassing the pinch research question — flagging that constraint.)
3. **Contact/solver tuning**: soft_contact_ke/kd/kf, VBD iterations, proxy coupling.
4. **Accept the cross-solver-confirmed finding**: soft-tofu fingertip pinch-lift is hard in BOTH MPM and VBD -> the "no intact band" result is physically robust and defensible.

STOP at the human-blocked terminus for the user's direction on which lever to pursue.
