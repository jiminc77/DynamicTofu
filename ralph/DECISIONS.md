# DECISIONS

## 2026-08-27 — PM-2 fired: damage observable degenerate; threshold-revision proposal (AWAITING EXTERNAL SIGN-OFF)

**Observation (G-N2 Jp probe, evidence `reports/logs/gn2-lift-jp.json` + crush diagnostic):**
Under a deliberate 5.0 N crush (2 s, block fully extruded across the table — y-extent grew from 0.04 m to 1.94 m), `mpm:particle_Jp` remained **exactly 1.0 for every particle (std = 0.0)**. The pre-registered damage predicate `|Jp−1| > 0.05` can never fire.

**Mechanism (source-grounded, pinned b74df53):** `mpm:yield_stress` drives **deviatoric** (volume-preserving) plasticity; `mpm:particle_Jp` is the **volumetric** plastic determinant. Volumetric plasticity activates only below `mpm:yield_pressure`, whose default is **1e15 Pa** (`solver_implicit_mpm.py:1268-1274`) — i.e. crush compaction is disabled unless the material declares a finite yield pressure. This is precisely the PM-2 mechanism the plan anticipated ("deviatoric-`yield_stress` / volumetric-`particle_Jp` mismatch").

**Proposal (requires external sign-off BEFORE any sweep, per brief):**
Complete the block's material model with a finite volumetric crush yield:
`mpm:yield_pressure = 2 × σ_Y` per material (i.e. 4.0 / 6.67 / 12.0 kPa for σ_Y = 2000 / 3333 / 6000 Pa), keeping the single-parameter material family and the monotone-crushing-force axis intact. `tensile_yield_ratio` stays at default. The judgment v1 damage threshold value (`|Jp−1| > 0.05`, fraction > 10%, latched) is then re-validated by the crush-vs-gentle probe; if the 0.05 window itself proves mis-scaled, a revised value will be proposed in a follow-up entry with the probe distributions.

**Status:** BLOCKED on external sign-off. No sweep, ramp-gate, or judgment-dependent run executes until signed off. Non-dependent G-N2 work (calibration, reach probe, gentle-lift media) continues.

### 2026-08-27 — EVIDENCE CORRECTION (read bug, recorded before sign-off arrived)

The "Jp identically 1.0" observation above was partially an instrumentation artifact: the coupled wrapper does **not** sync custom state attributes back to the parent state, so `parent_state.mpm.particle_Jp` stays at its initial value forever. Reading the **MPM entry state** (`solver._entries["mpm"].state_0`) shows Jp responding strongly — via tension/softening (Jp up to ~480 under tearing) even at the baseline material, and via compaction once a finite `yield_pressure` exists. All Jp reads now route through `src/coupling.py:mpm_entry_state()`. The core PM-2 finding stands (the baseline material has no volumetric crush yield: `yield_pressure` default 1e15 disables compaction), but the "exactly 1.0 everywhere" phrasing was over-strong — it described the stale parent buffer.

### 2026-08-27 — EXTERNAL SIGN-OFF RECEIVED: yield_pressure = 2 × σ_Y APPROVED

Approved by the external authority (user, this date): `mpm:yield_pressure = 2 × σ_Y` → 4.0 / 6.67 / 12.0 kPa for σ_Y = 2000 / 3333 / 6000 Pa. **Binding conditions:**
1. Before ANY E1 cell counts, the crush-vs-gentle probe re-runs on ALL THREE materials with clean separation: 5 N crush fires the damage predicate (fraction well above 10%); gentle 1.5 N lift does not. Probe JSONs ship in the G-N2 receipt.
2. `yield_pressure` is a protocol constant in every JSON config block and the receipt.
3. Any rescaling of the |Jp−1| > 0.05 window requires a follow-up proposal with probe distributions BEFORE sweeps.
4. The previously observed extrusion failure mode must be confirmed to register as damage.

Additionally ordered: contact friction is frozen NOW as a protocol constant in configs (it was not pre-registered in the brief). Frozen values actually in use, recorded per condition: `default_shape_mu = 0.5` (all non-pad contacts) and `pad_friction_mu = 1.0` (fingertip pads; rubber-pad rationale, introduced pre-sweep and recorded above). If the pad value must also be 0.5, it is a one-line change plus probe re-run — flagged in the G-N2 receipt for confirmation.

Implementation: `src/scene.py` `YIELD_PRESSURE_FACTOR = 2.0`, applied by default to every block; config blocks carry `yield_pressure_pa`, `yield_pressure_factor`, `default_shape_mu`, `pad_friction_mu`.

## 2026-08-27 — Gentle-grasp force raised to 1.5 N (probe parameter, not protocol)

F_g = 0.5 N cannot statically hold the 0.63 N block at μ = 0.5 (friction capacity = μ·2·F_n = 0.5 N < mg). The gentle-lift probe uses **F_g = 1.5 N** (capacity 1.5 N, margin 2.4×). E1 grid forces are unchanged — trials at low F_g are *expected* to drop/slip; that is the phase diagram working as designed.

## 2026-08-27 — Coupled-wrapper deviation record

The rigid-only `SolverCoupledProxy` configuration (no proxies) does not forward `Control.joint_f` to its MuJoCo entry (verified empirically). The AR-3 block-absent twin therefore uses plain `SolverMuJoCo` with 4 manual substeps mirroring the coupled entry's `rigid_substeps=4`. Recorded as a deviation, not worked around silently.

## 2026-08-27 — Pad-normal convention frozen

`pad_normal_convention: block_to_pad_outward` — AR-2's compressive-contact predicate is `F_reaction · n_outward > 0` (the harvested wrench is the force ON the finger). Frozen into the config block.
