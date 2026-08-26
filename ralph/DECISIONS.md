# DECISIONS

## 2026-08-27 — PM-2 fired: damage observable degenerate; threshold-revision proposal (AWAITING EXTERNAL SIGN-OFF)

**Observation (G-N2 Jp probe, evidence `reports/logs/gn2-lift-jp.json` + crush diagnostic):**
Under a deliberate 5.0 N crush (2 s, block fully extruded across the table — y-extent grew from 0.04 m to 1.94 m), `mpm:particle_Jp` remained **exactly 1.0 for every particle (std = 0.0)**. The pre-registered damage predicate `|Jp−1| > 0.05` can never fire.

**Mechanism (source-grounded, pinned b74df53):** `mpm:yield_stress` drives **deviatoric** (volume-preserving) plasticity; `mpm:particle_Jp` is the **volumetric** plastic determinant. Volumetric plasticity activates only below `mpm:yield_pressure`, whose default is **1e15 Pa** (`solver_implicit_mpm.py:1268-1274`) — i.e. crush compaction is disabled unless the material declares a finite yield pressure. This is precisely the PM-2 mechanism the plan anticipated ("deviatoric-`yield_stress` / volumetric-`particle_Jp` mismatch").

**Proposal (requires external sign-off BEFORE any sweep, per brief):**
Complete the block's material model with a finite volumetric crush yield:
`mpm:yield_pressure = 2 × σ_Y` per material (i.e. 4.0 / 6.67 / 12.0 kPa for σ_Y = 2000 / 3333 / 6000 Pa), keeping the single-parameter material family and the monotone-crushing-force axis intact. `tensile_yield_ratio` stays at default. The judgment v1 damage threshold value (`|Jp−1| > 0.05`, fraction > 10%, latched) is then re-validated by the crush-vs-gentle probe; if the 0.05 window itself proves mis-scaled, a revised value will be proposed in a follow-up entry with the probe distributions.

**Status:** BLOCKED on external sign-off. No sweep, ramp-gate, or judgment-dependent run executes until signed off. Non-dependent G-N2 work (calibration, reach probe, gentle-lift media) continues.

## 2026-08-27 — Gentle-grasp force raised to 1.5 N (probe parameter, not protocol)

F_g = 0.5 N cannot statically hold the 0.63 N block at μ = 0.5 (friction capacity = μ·2·F_n = 0.5 N < mg). The gentle-lift probe uses **F_g = 1.5 N** (capacity 1.5 N, margin 2.4×). E1 grid forces are unchanged — trials at low F_g are *expected* to drop/slip; that is the phase diagram working as designed.

## 2026-08-27 — Coupled-wrapper deviation record

The rigid-only `SolverCoupledProxy` configuration (no proxies) does not forward `Control.joint_f` to its MuJoCo entry (verified empirically). The AR-3 block-absent twin therefore uses plain `SolverMuJoCo` with 4 manual substeps mirroring the coupled entry's `rigid_substeps=4`. Recorded as a deviation, not worked around silently.

## 2026-08-27 — Pad-normal convention frozen

`pad_normal_convention: block_to_pad_outward` — AR-2's compressive-contact predicate is `F_reaction · n_outward > 0` (the harvested wrench is the force ON the finger). Frozen into the config block.
