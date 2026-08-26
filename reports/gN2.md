# G-N2 Gate Receipt — Physics Smoke (DRAFT: gate call pending P3/P4 ruling)

- Gate: G-N2 (deadline 2026-08-29 23:59:59 Asia/Seoul — cutoff per external confirmation 2026-08-27)
- Status: **all rig criteria measured; P3 + P4 externally APPROVED and executed; one narrow ruling pending (P5: per-material downward ladder extension + rate-check re-scope — `ralph/DECISIONS.md`). Fail-closed: no gate call until P5 is decided.**
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
| 8-level calibration (**P4 approved**: pre-saturation acceptance) | **PASS** — mapping exact through 2.5 N (slope 0.99984, intercept 0.0003 N, residual 0.0003 N, hysteresis 0.025 N); measured bearing capacities (P4 first-class observable, realized bilateral sums): **5.61 / 6.11 / 7.18 N** for σ_Y 2000/3333/6000; `f_g_realized_n` recorded in every E1 trial JSON | `gn2-calibration.json`, `gn2-calibration-presaturation.json`, `gn2-dynamic-ladder.json` |
| Grasp + 5 cm lift, health clean | lift executed, health clean throughout every probe/trial (no NaN/inf, max particle speed ≤5 m/s, grid ok); block response = coherent held **elongation** (z-extent 0.082 m, bottom near table) — a rigid 5 cm carry does not occur at this softness; shown frame-by-frame in the gate clips | `gn2-jp-probe-3333.json`, media below |
| particle_Jp readout (condition 1, per-material contract) | **PASS all three** — crush fires: 2000: 0.319, 3333: 0.140; 6000: 0.025 = censored_high above grid top (authorized); gentle clean: 0.095 / 0.064 / 0.012; separations 3.3× / 2.2× / 2.2× | `gn2-jp-probe-{2000,3333,6000}.json` |
| Condition 4 (extrusion registers as damage) | **confirmed** — crush damage latches while grasped (`cell_color = damage`) | same |
| Condition 3 (window scaling) | window functions (fires 0.14–0.32 vs clean 0.01–0.095); **no rescale proposal needed** | same |
| σ_Y monotonicity gate (**P3 approved**: dynamic ladder) | executed in full (54 trials, health clean): σ=3333 onset **5.575 N** observed (per-seed sd 0.132); σ=6000 censored_high at the top (allowed); σ=2000 censored_low at the approved ladder bottom ⇒ **P5 pending** (down-ladder E1-grid points measured: 0.8 N→0.053, 1.2 N→0.098 → onset ≈ **2.43 N** interior once {0.8, 1.2} join its ladder). Rate-sensitivity recorded: 0.6 s close never crosses 0.10 (0.037–0.066) — genuine viscoplastic rate dependence; E1's own close is 0.3 s. Under P5 the gate **passes**: 2.43 < 5.57 < censored-above-7.18 N, monotone, separation 3.14 N ≫ tolerance | `gn2-dynamic-ladder.json`, `gn2-ladder-2000-extension.json` |

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

## Rate-dependent crushability (physical note, dataset preserved per P3 approval)

Quasi-static loading cannot crush the firmer materials: at the frozen 0.483 N/s ramp, realized force saturates (5.1 N at σ=3333, 6.95–7.27 N at σ=6000, even commanding 12 N) while damage fractions freeze at 0.049/0.019–0.021 (`gn2-ramp-gate.json`, `gn2-ramp-extended-12n.json`); σ=2000 onsets quasi-statically at 2.69 ± 0.06 N. The dynamic 0.3 s close — the regime E1 actually probes — crushes monotonically in σ_Y. Rate-dependent crushability of the viscoplastic material is a legitimate physical result.

## Recording flag (external order)

σ=2000's gentle probe peaked at **0.0955 — 0.5 pp under the 10% latch**. If E1 later shows the σ=2000 band nearly empty even at a=1, that is a reportable pre-registered outcome, not a failure.

## Pending before the gate call

1. **P5** — per-material downward ladder extension (E1-grid forces only) + rate-adequacy re-scope to a recorded observable. All supporting numbers are already measured; under P5 every G-N2 criterion is satisfied.

On P5 sign-off: finalize this receipt with the gate call, commit, checkpoint G002, then proceed G-N3 → E1 Stage A under the standing authorization.
