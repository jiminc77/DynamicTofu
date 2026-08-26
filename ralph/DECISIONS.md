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

## 2026-08-27 — FOLLOW-UP PROPOSAL P2 (condition-3 path; AWAITING EXTERNAL SIGN-OFF)

**New root cause found while executing condition 1:** `mpm:tensile_yield_ratio` defaults to **0.0** — tensile yield = ratio × yield_pressure = **zero tensile strength** at any yield_pressure. The material cannot carry tension, so every lift tears the block at the grip plane (window damage fraction 0.90–1.0 at ALL σ_Y and ALL forces — uniformity that flagged the artifact). This held for the pre-sign-off baseline too (0 × 1e15 = 0): the block never had tensile strength.

**Measured landscape (σ=3333, judgment predicate |Jp−1|>0.05, fraction>10%, window = lift-complete→settle-end; full JSONs `reports/logs/gn2-material-window.json`):**

| yield_pressure | tensile ratio | crush 5 N fires | gentle 1.5 N lift clean |
|---|---|---|---|
| 2.0σ (signed off) | 0 (default) | 0.83 yes | 0.955 NO (tears) |
| 2.0σ | 1.0 | 0.010–0.015 **NO** | 0.006 yes |
| 0.85σ | 0.75–1.0 | **0.148–0.153 yes** | **0.088–0.091 yes** |
| 0.75σ | 0.75–1.0 | 0.20 yes | 0.119–0.137 NO |
| 0.6σ | 1.0 | 0.29 yes | 0.198 NO |

Cross-material at (0.85σ, 0.75): σ=2000 gentle-1.5N fires 0.250 (1.5 N is not gentle FOR the softest material); σ=6000 crush-5N only 0.023 (5 N is not a crush FOR the firmest). **The literal condition-1 force pair cannot separate all three materials with one parameter set — material-dependent damage-onset force is the experiment's own headline hypothesis.**

**Carry limitation (physics, not a bug):** with tensile strength restored, gentle lifts no longer tear (fractions 0.005–0.03) but the block is carried only 2–5 mm: at E=7 kPa the pads indent ~1 cm (contact pressure ≈ 0.3–1× σ_Y → strain ~27%), and the 5 cm/0.3 s lift carves the pads up through the material. Real-world correlate: fingertip-pinching silken tofu. The E1 protocol handles this as drop/slip labels; the intact band may be small or empty at low F_g — that is data, and the pre-registered shape checkpoint classifies it.

**Options for sign-off:**
1. **(Recommended) Material completion v2:** `yield_pressure = 0.85σ_Y`, `tensile_yield_ratio = 1.0`, `viscosity = 20 Pa·s`; damage window unchanged. Condition 1 passes verbatim at σ=3333 (crush 0.148 / gentle 0.088 incl. the 1.5 N lift); for 2000/6000 the probe forces become per-material E1-bracketing pairs (2000: gentle 0.3–0.4 N; 6000: crush at grid-top 5 N registers 0.067 — its damage onset lies above the grid, reported censored_high).
2. Keep signed-off `yield_pressure = 2σ_Y` + `tensile_yield_ratio = 1.0` and rescale the damage window value (condition-3 clause) — would propose |Jp−1| > ~0.015 from the distribution percentiles; requires one more distribution pass to freeze.
3. Reduced scope: validate the damage axis on σ=3333 only (Stage-A material), record 2000/6000 damage observables as exploratory.

No sweep or gate-decisive run executes until one option is signed off.

## 2026-08-27 — P2 GO (external): Option 1 authorized in full

The external authority confirmed the earlier sign-off authorizes executing P2 in full — three-material probe re-run, ramp gate, final calibration, and gate media — with no further user gate before those. Adopted protocol constants (src/scene.py, every config block):

- `yield_pressure = 0.85 × σ_Y` (1700 / 2833 / 5100 Pa)
- `tensile_yield_ratio = 1.0` (tensile yield = yield_pressure)
- `viscosity = 20 Pa·s`

Condition 3 remains binding: if probe distributions show the |Jp−1| > 0.05 window mis-scaled, submit a follow-up proposal with distributions BEFORE any sweep; never tune silently.

## 2026-08-27 — PRE-REGISTRATION (external order, before any Stage-A data): cell-outcome precedence

Every trial JSON records the TIME of the damage latch (`damage_latch_t`) and of drop/grasp-loss (`drop_t`). Phase-diagram cell attribution (`cell_color`):

- damage latched AFTER grasp loss → cell colors as **drop** (slip side), `damage_after_drop: true` (impact compaction on the table is a fall artifact, not a grasp outcome);
- damage while still grasped → cell colors as **damage**;
- both raw judgment-v1 labels remain in the JSON unchanged either way.

Implemented in `src/judgment.py` + `src/trial.py`; covered by 3 unit tests (23 green). The band estimator's intact predicate is untouched.

## 2026-08-27 — Gentle-grasp force raised to 1.5 N (probe parameter, not protocol)

F_g = 0.5 N cannot statically hold the 0.63 N block at μ = 0.5 (friction capacity = μ·2·F_n = 0.5 N < mg). The gentle-lift probe uses **F_g = 1.5 N** (capacity 1.5 N, margin 2.4×). E1 grid forces are unchanged — trials at low F_g are *expected* to drop/slip; that is the phase diagram working as designed.

## 2026-08-27 — Coupled-wrapper deviation record

The rigid-only `SolverCoupledProxy` configuration (no proxies) does not forward `Control.joint_f` to its MuJoCo entry (verified empirically). The AR-3 block-absent twin therefore uses plain `SolverMuJoCo` with 4 manual substeps mirroring the coupled entry's `rigid_substeps=4`. Recorded as a deviation, not worked around silently.

## 2026-08-27 — Pad-normal convention frozen

`pad_normal_convention: block_to_pad_outward` — AR-2's compressive-contact predicate is `F_reaction · n_outward > 0` (the harvested wrench is the force ON the finger). Frozen into the config block.
