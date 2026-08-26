# E1 Cross-Material Band Summary (Stage A + Stage B, 225 trials)

Completed 2026-08-27 ~08:20 KST. 225/225 trials done, 0 unresolved, health clean throughout; seeds unanimous in 73/75 coordinate cells (the two mixed cells sit exactly on the σ=2000 damage boundary). Coverage maps: 75/120 formal coordinates per material `done` and artifact-cross-linked; the 45 remaining per material are the un-run C(i) grip columns, listed `skipped_not_authorized` pending the storyline decision.

## Phase grids (cell_color per the pre-registered precedence; 3 seeds per cell)

Force columns: 0.3 / 0.8 / 1.8 / 3.5 / 5.0 N (commanded per-finger; realized saturates at the per-material bearing capacity).

| σ_Y | a=1 | a=2.5 | a=5 | a=10 | a=15 |
|---|---|---|---|---|---|
| 2000 | D D **X X X** | D **B X X X** | D **B X X X** | D **B X X X** | D **B X X X** |
| 3333 | D D D **X X** | D D D **X X** | D D D **X X** | D D D **X X** | D D D **X X** |
| 6000 | D D D D D | D D D D D | D D D D D | D D D D D | D D D D D |

D = drop (grip cannot carry), X = damage-while-grasped (crush), B = boundary (seeds split damage/drop). **Intact bands are EMPTY at every (material, acceleration)** — `e1_band_{2000,3333,6000}.json`, all rows `empty`, a_star = 1.0 (`observed`, trivially on the empty ordering).

## The monotonicity claim IS in the data — via the damage-onset boundary

| σ_Y | damage onset (commanded F, grid resolution) | ladder-gate onset (realized bilateral, G-N2) |
|---|---|---|
| 2000 | 0.8–1.8 N (0.8 is the split cell) | 2.429 N |
| 3333 | 1.8–3.5 N | 5.575 N |
| 6000 | > 5.0 N (never crushes in-grid) | censored above 7.18 N |

Strictly monotone increasing in σ_Y, in both independent measurements. The phase diagram's structure lives on the drop→damage boundary; the intact window between "too weak to carry" and "strong enough to crush" is empty for all three materials at this (E=7 kPa, fingertip-pad, pre-registered lift) configuration — robustness confirmed by the externally-ordered gentle-lift diagnostic (0/6 intact at 1.0 s lift).

## Acceleration axis

Cell colors are invariant across the accel ladder in every material (grasp/lift stresses dominate transport stresses at these parameters). Realized-accel annotation (external ruling): commanded {10, 15} form ONE realized level ≈ 6.49 m/s² (`ralph/results/e1_realized_accel_table.json`); commanded ≤5 tracked within 5.1%.

## Artifacts

- Bands + coverage: `ralph/results/e1_band_2000.json`, `e1_band_3333.json`, `e1_band_6000.json`
- Router record (Stage A): `ralph/results/e1_shape_checkpoint.json`; escalation + rulings: `reports/shape_checkpoint.md`, `ralph/DECISIONS.md`
- Realized-accel table: `ralph/results/e1_realized_accel_table.json`; per-level tracking: `reports/logs/e1-stageA-tracking.json`
- 225 per-trial JSONs: `ralph/results/trials/` (each with realized force/accel, damage/drop timing, precedence fields, full config block)
- Diagnostic (non-E1): `reports/logs/diagnostic-gentle-lift/`

## Deferred for the storyline decision (external)

E2 mid-band selection (no usable band exists → the protocol's `censored_no_band` path, or a storyline-driven alternative selection rule with sign-off), E3 triplet sourcing (no intact/slip cells exist → fail-closed unavailability report, or a re-scoped demo), Stage C relevance (boundary densification at σ=2000's split cell would sharpen the one non-unanimous boundary).
