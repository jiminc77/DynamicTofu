# G-N2 Gate Receipt — Physics Smoke (DRAFT: gate call pending P3/P4 ruling)

- Gate: G-N2 (deadline 2026-08-29 23:59:59 Asia/Seoul — cutoff per external confirmation 2026-08-27)
- Status: **all rig criteria measured; two protocol rulings pending (P3 ramp-gate operationalization, P4 calibration acceptance range — `ralph/DECISIONS.md`). Fail-closed: no gate call is made here until both are decided.**
- Material under test: pre-registered E=7 kPa, ν=0.45, ρ=1000, σ_Y ∈ {2000, 3333, 6000} Pa **plus the externally signed-off completion** (P2 GO): `yield_pressure = 0.85σ_Y`, `tensile_yield_ratio = 1.0`, `viscosity = 20 Pa·s`. Frozen contact constants: `default_shape_mu = 0.5`, `pad_friction_mu = 1.0`.

## Composition (plan contract)

`SolverCoupledProxy` — MuJoCo entry (substeps 4, njmax/nconmax 256) + ImplicitMPM entry (in_place, voxel 5 mm), lagged proxy, iterations 1, mass_scale 1.0. EFFORT-mode fingers (post-finalize asserted: gains exactly 0), arm POSITION (ke 3000/kd 150), IK writes masked to the 7 arm coordinates, closed-loop EE servo (residual 1.7 mm at the grasp pose). Speculative contact gap tightened to 5 mm (the 0.1 m default parked the arm centimetres high — deviation recorded in DECISIONS.md).

## Acceptance evidence

| Criterion | Result | Evidence |
|---|---|---|
| AR-1 per-finger force+torque attribution | **PASS** — errors ~1e-7 N / 3e-8 N·m vs tolerances max(2%, 0.01 N) / max(2%, 1e-4 N·m), both fingers | `reports/logs/gn2-ar-probe.json` |
| AR-2 grasp balance, compressive sign | **PASS** — balance residual 0.0079 N ≤ tol; both normals compressive (pad-outward convention, frozen) | same |
| AR-3 feedback presence | **PASS** — deflection ratio 602× (>10× floor) | same |
| AR-4 global residual | logged only: 1.52 N | same |
| Coupling sensitivity (iterations {1,2,4}) | recorded | same, `sensitivity` block |
| Reach (both frozen criteria) | **PASS** — worst IK residual 2.9e-6 m (≤2 mm); worst joint margin 0.735 rad (≥0.10); all ±0.02 m perturbations solvable | `reports/logs/gn2-reach.json` |
| Mimic convention | **frozen: dual** — realized 1.2003 N at commanded 1.2 (master = exactly half) | `reports/logs/gn2-calibration.json` |
| 8-level calibration | mapping **exact** through 2.5 N (slope 0.99984, intercept 0.0003 N, residual 0.0003 N, hysteresis 0.025 N); saturates at bearing capacity ≈ **2.84 N** (3.5→2.710, 5.0→2.843) — frozen all-8 fit fails on saturation ⇒ **P4 ruling pending** | `gn2-calibration.json`, `gn2-calibration-presaturation.json` |
| Grasp + 5 cm lift, health clean | lift executed, health clean throughout every probe/trial (no NaN/inf, max particle speed ≤5 m/s, grid ok); block response = coherent held **elongation** (z-extent 0.082 m, bottom near table) — a rigid 5 cm carry does not occur at this softness; shown frame-by-frame in the gate clips | `gn2-jp-probe-3333.json`, media below |
| particle_Jp readout (condition 1, per-material contract) | **PASS all three** — crush fires: 2000: 0.319, 3333: 0.140; 6000: 0.025 = censored_high above grid top (authorized); gentle clean: 0.095 / 0.064 / 0.012; separations 3.3× / 2.2× / 2.2× | `gn2-jp-probe-{2000,3333,6000}.json` |
| Condition 4 (extrusion registers as damage) | **confirmed** — crush damage latches while grasped (`cell_color = damage`) | same |
| Condition 3 (window scaling) | window functions (fires 0.14–0.32 vs clean 0.01–0.095); **no rescale proposal needed** | same |
| σ_Y monotonicity gate | quasi-static ramp **inconclusive by construction**: 2000 onsets at 2.69 ± 0.06 N; 3333/6000 saturate (realized 5.1/6.95 N, fractions frozen at 0.049/0.019 even at 12 N command) ⇒ **P3 dynamic-ladder proposal pending** | `gn2-ramp-gate.json` |

## Video evidence (user-ordered amendment)

- `reports/media/gn2_gentle_lift.mp4` + `gn2_gentle_lift_key{0..4}.png` — gentle 1.5 N grasp-and-lift, σ=3333, damage fraction peaks 0.064.
- `reports/media/gn2_crush.mp4` + `gn2_crush_key{0..4}.png` — 5 N crush, damage core (|Jp−1|>0.05, red) growing in a grasped block to fraction 0.14.
- Render-path deviation (recorded per amendment): offscreen GL was not exercised; clips use the fallback orthographic PIL renderer (side y-z + front x-z views, damaged particles in red) at 200 Hz physics unaffected. Decode-verified (ffprobe: 21/20 frames, 1280×640).

## Deviations & corrections

- D1 Jp read path: the coupled wrapper never syncs `mpm:particle_Jp` to the parent state; all reads use the MPM entry state (`src/coupling.py:mpm_entry_state`). An earlier "Jp ≡ 1.0" claim was partly this read bug — corrected in DECISIONS.md before sign-off.
- D2 rigid-only `SolverCoupledProxy` drops `joint_f`; block-absent AR-3 twin uses plain SolverMuJoCo with 4 manual substeps.
- D3 `tensile_yield_ratio` engine default 0.0 (zero tensile strength) — root cause of universal lift tearing; fixed by the signed-off material completion.
- D4 finger hold-open (+1 N) during approach; free EFFORT fingers drift shut and plow the block otherwise.
- D5 fingertip pad friction 1.0 (rubber pads), flagged and frozen with `default_shape_mu = 0.5`.

## Pending before the gate call

1. **P3** — dynamic-ladder σ_Y onset gate (quasi-static operationalization unreachable for viscoplastic v2).
2. **P4** — calibration limits over the pre-saturation range + `f_bearing_capacity_n` as a measured observable.

On sign-off: run the dynamic ladder (~25 min), apply the calibration ruling, finalize this receipt with the gate call, commit, checkpoint G002.
