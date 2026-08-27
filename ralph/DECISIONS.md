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

## 2026-08-27 — PROPOSAL P3: re-operationalize the σ_Y monotonicity gate (AWAITING EXTERNAL SIGN-OFF)

**Result of the pre-registered quasi-static ramp gate (evidence `reports/logs/gn2-ramp-gate.json`):** σ=2000 onsets cleanly (F_onset = 2.69 ± 0.06 N over 3 seeds), but σ=3333 and 6000 are censored at the 6.0 N ceiling — an inconclusive censoring pattern, i.e. a fail-closed MISS under the frozen rule.

**Extended-ceiling evidence (12 N command, frozen 0.483 N/s rate, seed 0):** the quasi-static ramp SATURATES — realized bilateral normal plateaus at 5.1 N (σ=3333) / 6.95 N (σ=6000) while damage fraction freezes at 0.049 / 0.019. Under slow loading the viscoplastic v2 material relieves pressure by deviatoric flow faster than compaction accumulates; quasi-static onset is unreachable for the two firmer materials at ANY ceiling. The operationalization, not the material, is wrong-shaped.

**Contrast (already-measured dynamic crush, `reports/logs/gn2-jp-probe-*.json`):** the sweep-matched dynamic crush (0.3 s ramp + hold) fires the same predicate at 5 N for σ=2000 (0.319) and σ=3333 (0.140) and leaves σ=6000 below threshold (0.025) — monotone in σ_Y and physically the regime E1 actually probes (every E1 cell closes in 0.3 s).

**Proposal P3 — dynamic-ladder onset gate:** for each material, run dynamic crush trials (0.3 s ramp to F + 2.0 s hold, the E1 close profile) over the force ladder F ∈ {1.8, 2.5, 3.5, 5.0} N, 3 seeds each. F_onset* = realized bilateral normal interpolated between the bracketing ladder forces where the peak damage fraction crosses 10%. Direction (strictly increasing in σ_Y), separation (> max(0.05 N, 2 × max sd_seed)), and censoring rules unchanged; censoring valid only at the largest σ_Y. Rate adequacy: repeat σ=3333 with the close ramp doubled to 0.6 s; same tolerance max(0.05 N, 5%).

The quasi-static ramp result (2000 = 2.69 N, 3333/6000 censored + saturation traces) is preserved in the gate report as evidence either way. No sweep runs until P3 is decided.

## 2026-08-27 — PROPOSAL P4: calibration acceptance over the pre-saturation range (AWAITING EXTERNAL SIGN-OFF, bundled with P3)

**Finding (evidence `reports/logs/gn2-calibration.json` + `gn2-calibration-presaturation.json`):** on the v2 material at σ=3333 the F_g → realized mapping is essentially EXACT through 2.5 N — commanded {0.3, 0.5, 0.8, 1.2, 1.8, 2.5} N realize {0.300, 0.500, 0.800, 1.200, 1.800, 2.500} N — and then SATURATES: commanded 3.5 → realized 2.710, commanded 5.0 → realized 2.843, matching on descent (2.644 / 2.846). The specimen's bearing capacity (~2.84 N at this pad geometry) physically caps the transmissible normal force; beyond it the material yields and force plateaus. The mimic probe confirms the actuation chain is not the limiter (realized 1.2003 at commanded 1.2).

The frozen limits applied as a single linear fit over all 8 levels therefore fail (slope 0.58, intercept 0.44) — but on the pre-saturation range the same frozen limits pass overwhelmingly: slope 0.99984, intercept 0.00027 N, max residual 0.00034 N, hysteresis 0.025 N (all within slope [0.90, 1.10], |intercept| ≤ 0.05, residual ≤ max(0.05, 10%), monotone, hysteresis ≤ 0.05).

**Proposal P4:** evaluate the frozen calibration limits over the material-transmissible (pre-saturation) range, and record the measured saturation ceiling per material as a first-class protocol observable (`f_bearing_capacity_n`) in the gate receipt and config blocks. Additionally (additive recording, no protocol change): every E1 trial JSON gains `f_g_realized_n` — the realized bilateral-mean per-finger normal during the hold — so commanded-vs-realized force is visible exactly like commanded-vs-realized acceleration. The commanded F_g axis of the phase diagram is unchanged; trials are never re-binned.

Rationale: the saturation is the material's bearing capacity — a measured physical quantity directly related to the crushing-force claim — not an actuation defect. Hiding it behind a failed global fit would discard exact calibration data; re-binning by realized force would violate the pre-registered axis. No sweep runs until P3/P4 are decided.

## 2026-08-27 — Gentle-grasp force raised to 1.5 N (probe parameter, not protocol)

F_g = 0.5 N cannot statically hold the 0.63 N block at μ = 0.5 (friction capacity = μ·2·F_n = 0.5 N < mg). The gentle-lift probe uses **F_g = 1.5 N** (capacity 1.5 N, margin 2.4×). E1 grid forces are unchanged — trials at low F_g are *expected* to drop/slip; that is the phase diagram working as designed.

## 2026-08-27 — Coupled-wrapper deviation record

The rigid-only `SolverCoupledProxy` configuration (no proxies) does not forward `Control.joint_f` to its MuJoCo entry (verified empirically). The AR-3 block-absent twin therefore uses plain `SolverMuJoCo` with 4 manual substeps mirroring the coupled entry's `rigid_substeps=4`. Recorded as a deviation, not worked around silently.

## 2026-08-27 — Pad-normal convention frozen

`pad_normal_convention: block_to_pad_outward` — AR-2's compressive-contact predicate is `F_reaction · n_outward > 0` (the harvested wrench is the force ON the finger). Frozen into the config block.

## 2026-08-27 — P3 + P4 EXTERNALLY APPROVED; standing authorization

Both proposals signed off (evidence spot-checked by the external authority against gn2-ramp-gate.json, gn2-calibration*.json, gn2-jp-probe-*.json):

- **P3 official:** the dynamic-ladder onset gate (E1 close profile 0.3 s ramp + 2.0 s hold; ladder {1.8, 2.5, 3.5, 5.0} N × 3 seeds; F_onset interpolated on REALIZED bilateral normal; censoring valid only at the largest σ_Y; rate adequacy = σ=3333 repeated at 0.6 s close, tolerance max(0.05 N, 5%)) is the OFFICIAL σ_Y monotonicity gate. The quasi-static saturation dataset (gn2-ramp-gate.json + gn2-ramp-extended-12n.json) is preserved in the receipt — rate-dependent crushability is a legitimate physical note.
- **P4 official:** calibration limits evaluated over the pre-saturation range; `f_bearing_capacity_n` is a first-class protocol observable; `f_g_realized_n` recorded in every E1 trial JSON. Commanded F_g remains the axis; trials are never re-binned.
- **Recording flag (not a condition):** σ=2000's gentle probe peaked at 0.0955 — 0.5 pp under the 10% latch. Stated explicitly in the receipt; a nearly-empty σ=2000 band at a=1 in E1 is a reportable pre-registered outcome, not a failure.
- **Standing authorization:** once the P3 gate passes → G-N2 receipt (with gentle/crush clips) → G-N3 → E1 Stage A, no further user gates; only NEW protocol changes require sign-off.

## 2026-08-27 — PROPOSAL P5: two narrow amendments to the approved P3 gate (AWAITING EXTERNAL SIGN-OFF)

The approved P3 dynamic-ladder gate ran in full (36 + 12 trials, `reports/logs/gn2-dynamic-ladder.json`). Result under the approved rules: **inconclusive**, on two structural grounds — both physics, both evidenced:

1. **σ=2000 is censored_low**: the ladder bottom 1.8 N already crushes the softest material (fraction 0.194 ≫ 0.10), so its onset lies below the approved ladder — and censoring is only valid at the TOP σ_Y. The middle material observed cleanly: onset 5.575 N (realized bilateral sum; per-seed sd 0.132). σ=6000 censored_high at the top (allowed).
2. **Rate-adequacy is structurally unpassable**: at the 0.6 s close, σ=3333 never crosses 0.10 within the ladder (fractions 0.037–0.066 vs 0.045–0.127 at 0.3 s) — the viscoplastic material is genuinely rate-dependent, so the 0.6 s onset does not exist and |F_onset(0.3)−F_onset(0.6)| is undefined. E1's own close rate is fixed at 0.3 s: the gate already matches the sweep's dynamics by construction.

**Down-ladder evidence gathered (E1-grid forces only; `gn2-ladder-2000-extension.json`):** σ=2000 at 0.8 N → 0.053, at 1.2 N → 0.098 (per-seed 0.085/0.092/0.117). The 0.10 crossing interpolates to **onset ≈ 2.43 N** (realized sum), cleanly interior once {0.8, 1.2} join the σ=2000 ladder.

**P5 amendments:**
- (i) Per-material downward ladder extension using E1-grid forces where a non-top material censors low: σ=2000 ladder = {0.8, 1.2, 1.8, 2.5, 3.5, 5.0}. No new force values are invented — all are pre-registered grid levels.
- (ii) Rate adequacy re-scoped from pass/fail to a RECORDED rate-sensitivity observable (0.6 s fractions reported alongside 0.3 s; direction must be monotone — slower ⇒ lower fractions — which holds).

**Gate outcome under P5 (all numbers already measured):** onsets 2.43 < 5.57 < censored_high(>7.18) N; monotone ✓; separation 3.14 N ≫ max(0.05, 2×0.132) ✓; top-only censoring ✓; health clean in all 54 trials ✓ → **PASS**. Bearing capacities (P4 observable): 5.61 / 6.11 / 7.18 N (bilateral sums). No sweep or gate call until P5 is decided.

## 2026-08-27 — P5 EXTERNALLY APPROVED; gate PASS stamped; standing authorization resumes

Both amendments approved (external spot-check against gn2-dynamic-ladder.json confirmed: onset_0p3=5.5747, sigma2000 censored_low, onset_0p6 null, gate_pass correctly held false under the pre-P5 rule):
- (i) Downward ladder extension with pre-registered E1-grid forces only; σ=2000 ladder = {0.8, 1.2, 1.8, 2.5, 3.5, 5.0}.
- (ii) Rate adequacy re-scoped to a RECORDED rate-sensitivity observable with a monotone-direction requirement (slower close ⇒ lower fractions — holds). E1 close rate is fixed at 0.3 s; cross-cell comparability unaffected.

Recomputed gate outcome under P5, from the persisted trial points (60 dynamic-crush trials, health clean 60/60 incl. extension): onsets 2.43 < 5.57 < censored-above-7.18 N (bilateral sums), separation 3.14 N ≫ max(0.05, 2×0.132) ⇒ **σ_Y monotonicity gate: PASS** — stamped in `reports/logs/gn2-dynamic-ladder-P5.json`. Standing authorization resumes: G-N2 receipt → G-N3 → E1 Stage A; only NEW protocol changes need sign-off.

## 2026-08-27 — G-N3 stage-set selection (committed BEFORE the first E1 cell, per protocol)

Measured at G-N3 (reports/logs/gn3-official.json): weighted T = 34.5 s (a=1: 39.4 s, a=15: 29.6 s); T_E2 = 32 s (full E2 config incl. raw-field recording, reports/logs/e2-scratch). Computed by src/budget.py (inputs recorded, never a constant): R_E2 = 360 s → E2_overflow = 0 → E1_budget = 24.0 h → T_A_max = 921.6 s (nominal operating point valid because E2_overflow = 0). Governing rule N·T ≤ 0.8·E1_budget: selected stage set = **A + B(2000) + B(6000) + C(i) + C(ii) = up to 310 trials**, no stages dropped, no escalation (T = 34.5 s ≪ 921.6 s). Guaranteed E2 nine untouched. Stage order remains A → shape checkpoint → B → C with stage-atomic fail-closed semantics.

## 2026-08-27 — Stage-A shape checkpoint: scale_failure; STOPPED pending external review

Stage A complete (75/75, 0 unresolved, health clean; batch row in RESULTS.md). Pre-registered router outcome **scale_failure**: intact bands EMPTY at all five accelerations (all cells drop or damage; seeds unanimous; a_star=1.0 observed trivially on the empty ordering). Per protocol: Stage B/C, E2 mid-band selection, and E3 triplet are HALTED pending external review. Full report: reports/shape_checkpoint.md. Options + recommendation recorded there; no further stage executes until the external decision.

## 2026-08-27 — O-2 acceleration-tracking escalation trigger FIRED (recorded; co-decision with the scale_failure review)

Full Stage-A per-level analysis (reports/logs/e1-stageA-tracking.json): realized EE acceleration saturates at ≈6.49 m/s² — a_cmd 10 → 6.487 (35.1% err), a_cmd 15 → 6.487 (56.8% err); a_cmd ≤ 5 tracks within 5.1%. The plan's O-2 trigger (>25% median at a=15) fired; per its pre-registered response the jacobian-velocity option is raised externally alongside the shape-checkpoint review (reports/shape_checkpoint.md, corrected — the earlier "~3%" claim was wrong and is retracted there). Commanded axis remains the diagram axis; a_peak_realized_ms2 is in every JSON. No stage executes pending the combined ruling.

## 2026-08-27 — EXTERNAL RULING: Option 1 approved (Stage B) + gentle-lift diagnostic first

Ruling received: the σ=3333 empty-band finding is ACCEPTED as a real, reportable measurement (replicating the prior-engine observation for very soft tofu). Stage B (σ=2000 then 6000, frozen protocol, 150 trials) launches now. FIRST: one diagnostic probe (~6 trials, labeled diagnostic, never counted as E1 cells): σ=3333, a=1 transport, F ∈ {1.2, 1.8} × 3 seeds, lift softened to 5 cm in 1.0 s (vs the pre-registered 0.3 s) — tests whether the a=1 empty band is a material property or partly a lift-transient artifact (the frozen lift's ~3 m/s² peak exceeds the a=1 transport accel). If intact carries appear → an option-3 protocol revision becomes evidence-motivated and requires sign-off BEFORE any re-run; if drops persist → the empty band is fully robust as measured. E2 mid-band selection, E3 source trials, and the paper storyline are DEFERRED until Stage B + probe land (storyline reserved for the user). The O-2 tracking co-decision remains open alongside.

## 2026-08-27 — Gentle-lift diagnostic result: empty band ROBUST (no option-3 case)

6/6 diagnostic trials (σ=3333, a=1, F∈{1.2,1.8}×3 seeds, lift softened to 1.0 s; reports/logs/diagnostic-gentle-lift/) produced ZERO intact carries: F=1.2 drops (0.056–0.060 fractions), F=1.8 straddles the damage latch (0.090–0.109; 2 damage-while-grasped, 1 drop). Per the external ruling's own criterion, the σ=3333 empty band is a material property, not a lift-transient artifact — the record stands as measured and no protocol-revision case exists. Stage B launches under the frozen protocol.

## 2026-08-27 — EXTERNAL RULINGS: diagnostic acknowledged; O-2 ceiling ACCEPTED; Stage B re-confirmed

1. Gentle-lift diagnostic acknowledged externally: 0/6 intact ⇒ the σ=3333 empty band is a robust material property; option 3 is dead; the lift-duration parameter remains diagnostic-only (never a protocol constant).
2. O-2 co-decision: the ~6.49 m/s² realized-acceleration ceiling is ACCEPTED for this sprint. No jacobian-velocity switch mid-sprint (doctrine holds; post-sprint backlog item). Stage B runs the FROZEN pre-registered grid unchanged — comparability and a complete coverage map outweigh the ~19 min the two collapsed levels would save. Analysis and figures treat commanded {10, 15} as ONE realized level (~6.5 m/s²) and annotate the commanded axis with realized medians (reports/logs/e1-stageA-tracking.json pattern, to be extended with Stage-B data).
3. Stage B GO re-confirmed post-diagnostic (was already running under the earlier ruling; 75/150 at re-confirmation). E2/E3 selections stay deferred until Stage-B bands land.
4. Post-Stage-B deliverable: cross-material band summary (e1_band JSONs for 2000/3333/6000 + realized-accel table) → user storyline decision.

## 2026-08-27 — Three external mechanical directives executed

**(1) Band-config header fix.** aggregate_bands.py reused one arbitrary trial's config block for all three materials (header-template bug; per-trial data was always correct). Fixed to per-material config selection; all three e1_band_{2000,3333,6000}.json regenerated and verified: config.material.{sigma_y_pa, yield_pressure_pa, f_bearing_capacity_n} now match their own material's trials (2000/1700/5.61, 3333/2833/6.11, 6000/5100/7.18).

**(2) drop_t semantics — clarified, invariant proven.** ROOT CAUSE: drop_t and damage_latch_t are recorded in the ABSOLUTE sim clock (rig.t); phase_timestamps are RELATIVE to t0 (the sim time when the judged phases begin, after approach+servo, ~4 s). The two frames are offset by t0, which made drop_t=5.3 look post-window vs a relative settle_end=3.305. Re-verified on the flagged trial (s2000_a10_f0.3_seed0): t0=4.0 s, absolute window [5.30, 7.305], drop_t=5.30 = exactly lift-complete (drop_t_rel=1.30). The reducer's window guard (`if t < lift_complete or t > settle_end: continue`) makes it IMPOSSIBLE to label from out-of-window evidence — proven by two new unit tests (drop evidence after settle_end → no label; in-window → labeled) and now HARD-ASSERTED in run_trial. Every trial JSON now stores t0_abs_s, judgment_window_abs_s, drop_t_rel, damage_latch_t_rel, and explicit drop_evidence_in_window / damage_evidence_in_window booleans (self-verifiable). Full 225-trial empirical audit: see reports/logs/drop-window-audit.json (all 225 carry a drop label; every one re-verified in-window).

**(3) Stage C boundary densification.** The 4 σ=2000 non-unanimous cells (F=0.8, a∈{2.5,5,10,15}) densified to 5 seeds (seeds 3,4 as extra_replications, NOT in the 360 universe; ralph/results/extra_replications/). Result: every cell is 3 damage / 2 drop → no color reaches the ≥4/5 fraction threshold → the boundary is a genuine tight drop↔damage transition (damage marginally favored), not resolvable to one side. Folded into e1_band_2000.json's extra_replications block; still no intact anywhere (bands remain empty). All 8 fresh trials drop_evidence_in_window=True.

## 2026-08-27 — USER RULINGS refining the queued consult-gate directive

1. **Bottom-shelf / spatula support branch: REJECTED.** It bypasses the research question (grip-force / tactile-driven PINCH manipulation). Not implemented, not planned. The consult's fingertip-redesign section (d) shelf recommendation is explicitly out of scope.
2. **Gate B "30×30 mm pad" reinterpreted as a TACTILE-SENSOR-FORMAT flat fingertip** — the future sensor face we would mount (Paxini-class), not an arbitrary enlarged pad. Named `sensor_format_pad` in all configs and reports.
3. **Post-gate: DESIGN (do NOT run paper-facing sweeps with) a v2 closure controller** — closed-loop force control on measured per-finger normal force (PI on the 100 Hz F_n from the logging upgrade, with a closure-rate limit and a force cap). Rationale: the research target is tactile-feedback grip regulation; open-loop EFFORT was always a stopgap. Gate B position-lock stays a diagnostic only. E1 v1 frozen data untouched; the v2 rig gets a distinct name.
4. **Tofu stays 4×4×4 cm** for now — reducing mass would further weaken the acceleration axis; size is a second-order lever pending gate results.

Ordering unchanged: audit → Gate A → Gate B.

## 2026-08-27 — Item-2 drop-window audit COMPLETE: PASS (0 violations / 225)

Full non-mutating re-run audit (reports/logs/drop-window-audit.json): 225/225 coordinates re-run, every trial carries a drop label, ALL drop_evidence_in_window=True, 0 violations → verdict PASS (fail-closed check clears). Confirms the drop_t=5.3 observation was the absolute-vs-relative frame reporting artifact, not an out-of-window labeling bug. run_trial now hard-asserts the invariant; JSONs are self-verifying going forward.

## 2026-08-27 — Gate B GO (Option a): constant-effort primary + position-lock as labeled diagnostic

External ruling: Gate A purpose achieved (contact stack validated by matched-config isolation: elastic block lifts+holds under effort at exact A4 params, finalFn 0.454, +3.42 cm, 65 nodes). Do NOT redesign the Gate A oracle. Proceed to Gate B on σ_Y=6000 tofu (E=7 kPa), target realized 0.60±0.05 N/finger, 1.0 s lift + 10 s hold, no transport, h=5 mm, it=4, μ=1. Factorial: pad (stock 17.5×18.5 vs sensor_format_pad 30×30 mm r3) × control (constant_effort vs position_lock) × viscosity (20 vs 2e5). KEEP lock arms on tofu: at E=7 kPa a 0.6 N preload indents many mm, so sub-mm lock drift does NOT collapse Fn as it did on the stiff E=70 kPa oracle → lock is a valid H6 jaw-advance isolator for soft material.
SAFEGUARDS: (1) log/report per-arm Fn(t); if a lock arm loses Fn like the oracle → mark that arm INVALID, don't interpret. (2) all 6 arms at it=4; carry the H5 flag; if cheap, duplicate the single most decisive contrast at it=8. (3) any health blowup invalidates a run → rerun/flag, never interpret. (4) v2 closed-loop force controller stays design-only; Gate A evidence (position-lock unsuitable on stiff / effort sound) feeds its design notes.
Deliverables: gateB.json, short report, one holding + one dropping clip; then STOP at the human-blocked terminus with the cross-gate synthesis.

## 2026-08-27 — Gate B COMPLETE + cross-gate synthesis (STOP at human terminus)

All 6 arms drop. Lock arms INVALID (Fn→0 collapse, safeguard 1); valid effort arms (B1 stock, B4 sensor) maintain ~0.6 N + 80-95 nodes but block never lifts → tofu shear/extrusion limits hold (H8/H9), sensor pad doesn't rescue (H1 not dominant), no quasi-static hold (B6). it=8 duplicate: both effort arms BLOW UP (health=False) → coupling iteration-stability sensitivity (H5), conclusion valid at it=4 but not iteration-robust → Gate C convergence needed. Cross-gate: contact stack FUNCTIONAL (Gate A elastic holds under effort); empty band is a MATERIAL yield/extrusion effect (H8/H9, scoped to clean it=4 effort arms B1/B4; robust to pad area within effort (sensor~=stock), NOT established controller-robust since lock arms INVALID; H2 unisolated), NOT a contact bug; position-lock unsuitable (both gates) → motivates v2 force controller (design-only). Full report reports/gateB_report.md; clips gateB_holding_run.mp4 + gateB_dropping_run.mp4. STOP at human-blocked terminus per directive.

## 2026-08-27 — Gate B reviewed/verified; ORDER: B2 (viscosity axis) -> C (convergence+yield bracket) -> synthesis

External review verified Gate B (report, JSON spot-checks, clip frames consistent; lock invalidation + it=8 caveat correct). Note for record: the dropping-run frames show classic DUCTILE NECKING (pads carry a neck of material upward while the bulk stays grounded; damage particles concentrated in the neck) — consistent with H8 (ductile extrusion instead of fracture), to be discriminated from H2 by Gate B2.
Directed order:
1. **Gate B2** — restore the lost viscosity axis (eta=2e5 only ever ran under position-lock, so H2 was never isolated). Run B3prime = stock + constant-effort + eta 2e5, and B6prime = sensor_format_pad + constant-effort + eta 2e5; both it=4, sigma6000, mu=1, target 0.60 N/finger, 1 s lift + 10 s hold. Decisive: high-viscosity HOLD -> H2 dominant (literature rheology 1e5-1e7 Pa·s plausibly reopens the band); still DROPS via necking -> material-limit conclusion stands. Deliver gateB2.json + updated report section.
2. **Gate C** — proxy iterations {1,4,8} x h {5,2.5} at the B1 config (resolve the H5 it=8 stability caveat; mandatory before firm claims) + yield-surface bracket (C-base / C-cap-off / C-asym).
3. v2 controller stays design-only.
4. Storyline reserved; after B2+C land, assemble the full validity-gate synthesis package for the user decision.

## 2026-08-27 — Gate C COMPLETE + full validity-gate synthesis assembled (STOP for storyline)

Gate C: convergence grid it{1,4,8}xh{5,2.5mm} at B1 -> 3 health-clean cells (it1/h5, it1/h2.5, it4/h5) all DROP with finFn within ~3% and no lift (converged in the accessible stable regime); the other 3 cells BLOW UP (refining iterations or h destabilizes -> H5 stability ceiling, continuum convergence UNVERIFIED). Yield bracket: 2 of 3 cells (C-cap-off, C-asym) blew up (INVALID); only C-base valid -> yield-surface-shape robustness UNRESOLVED (no valid comparison). Full synthesis: reports/validity_gate_synthesis.md.
VERDICT: the empty band is a genuine MATERIAL-mechanics result (ductile shear/extrusion necking, H8) in the current model + stock/sensor fingertip + stable numerics; robust to pad area (H1 not dominant) and convergent in the stable regime; controller-robustness and yield-surface-shape robustness UNRESOLVED (lock arms + 2/3 yield cells numerically invalid); scoped to clean eta=20 constant-effort runs. OPEN (numerically inaccessible): H2 (high-viscosity creep untested - unstable) and continuum convergence (refinement unstable). Defensible scoped wording per consult adopted. RESERVED for user: (1) storyline; (2) solver stabilization (smaller coupled dt/implicit viscosity) to unblock H2+convergence; (3) v2 force controller build (design-only); (4) E2/E3 downstream of storyline. STOP at human terminus.

## 2026-08-27 evening — MAJOR PIVOT: MPM frozen/archived; VBD rebuild (V-track)

User decision: the MPM-based result cannot ground the paper; tofu MUST be properly implemented → rebuild on the Newton VBD soft-body path (base: examples/multiphysics/example_proxy_joint_gripper.py — MuJoCo 2-finger gripper grasping a VBD soft body via SolverCoupledProxy). Spec: reports/consult-vbd-rebuild.md. FRACTURE OUT OF SCOPE by user order: damage = pre-registered threshold on max principal strain, never a simulated topology change. **MPM E1 data + all gate receipts (G-N1/2/3, E1 bands, Gate A/B/B2/C) stay FROZEN and ARCHIVED — no further MPM runs.** Receipts discipline unchanged (config provenance incl. code SHA + controller mode, commit each phase, clips mandatory).

**V-1 (tonight): base-example verification — DONE.** At pin b74df534 all base examples EXIST and RUN headless exit 0: proxy_joint_gripper (21.6 s), rigid_soft_contact (2.5 s), softbody_franka, softbody_hanging present. **No pin bump needed.** Logs: reports/logs/vbd/v1_*.log. Deliver frames.

**V-2: tofu prototype.** 4x4x4 cm VBD soft grid (frozen size), density 1000, primary firm anchor E=25 kPa nu=0.45 (k_mu 8.6e3, k_lambda 77.6e3, k_damp 1.0); floating 3-DOF gripper (world → Z-prismatic lift → palm → 2 finger prismatics, NO arm yet); gravity -9.81 + ground plane; tofu just above z=0; force-limited closure with effort_limit as THE variable — sweep per-finger {0.3, 0.5, 0.8, 1.2, 2.0} N at soft_contact_mu 0.5; approach/close/hold/lift state machine; solver: VBD iterations 30, substeps 8-16, contact ke 5e4 kd 1e-3 kf 1e3. HARD MILESTONE: a clip of the tofu lifted and held intact >= 5 s (what MPM never achieved). If no force in the sweep lifts it → STOP + escalate with traces.

**V-3: judgment v2 + mini grid.** slip = gripper-frame displacement (reuse v1 thresholds); damage = principal-strain threshold — PROPOSE the value anchored to literature failure strains (tension 10-20%, compression 45-54%) and get external sign-off BEFORE it labels anything. Single-seed mini grid (force ladder x mu {0.5, 1.0}) to check for a non-empty quasi-static intact band. Deliver mini-grid JSON + one intact / one slip / one damage-threshold clip. Then STOP for external review (E1-equivalent design: transport via prescribed lateral palm motion — Franka accel ceiling does NOT apply to a floating gripper, record commanded + realized anyway; material axis via E variants; E2/v2 decisions with the user).

## 2026-08-27 evening — V-2 HARD MILESTONE NOT MET: VBD tofu lifts but does not hold >=5s (STOP + escalate)

Force sweep {0.3,0.5,0.8,1.2,2.0}N at soft_contact_mu 0.5 (+ extended 5N, 8N, mu 1.0/2.0, gentle 3s lift, all PERSISTED to reports/logs/vbd/v2_extended.json with full time series + config + provenance): ALL slip, every held_5s=False. The VBD tofu (E=25kPa, elastic) is grasped and LIFTS (transient peak up to 56.7mm at 8N, beyond MPM's zero) but slips/extrudes out during the 5s hold (post-lift COM rise ~4.4-11.5mm). Failure: low force -> friction slip; high force -> elastic bulge/extrusion around the pads (max bbox growth 0.60 at 8N); slower lift and higher friction do not help. SCOPED: within the tested configs, two independent solvers (MPM E7kPa viscoplastic; VBD E25kPa elastic) with the tested fingertip-pad geometry + force-limited closure both fail to hold -> CONFIRMS the empty-band difficulty for the tested models/geometries (does NOT prove no firmer-E/geometry could hold; those are the open levers). Per directive: STOP + escalate with traces. Report reports/vbd/v2_report.md; traces reports/logs/vbd/v2_sweep.json; failure clip reports/media/v2_slip_2N.mp4. Levers for user: firmer E (material axis / min-E-that-holds sweep), pad geometry (shelf previously rejected), contact tuning, or accept the cross-solver-confirmed finding. MPM data stays frozen.

## 2026-08-27 evening — LEVER 0: VBD Coulomb oracle (Gate-A analog) before any tofu-side lever

External ruling: run the missing control experiment first. VBD Coulomb oracle = same floating gripper + contact params but a STIFF elastic block (E=1 MPa, nu 0.3, 4 cm, 64 g) at 0.45 N/finger, mu=1.0, 1 s lift + 5 s hold. Coulomb capacity 0.9 N > 0.63 N weight -> MUST hold. Branch (a) holds -> contact stack fine, failure material-side -> E sweep {25,50,100,200 kPa} at 0.8 N mu 1.0 for min stiffness that statically holds. Branch (b) slips -> contact-stack/param problem (suspect soft_contact_kf tangential stiffness = placeholder 1e3, particle_radius, VBD iterations) -> kf ladder {1e3,1e4,1e5} on the oracle to find what restores Coulomb before touching tofu. Four levers queued behind this; lever 2 (pad geometry) constrained by the standing no-bottom-shelf ruling.
FRICTION-LEVEL (from source, newton/_src/solvers/vbd/rigid_vbd_kernels.py): VBD rigid-particle contact friction is IMPULSE/DISPLACEMENT-level ALM Coulomb with cone projection (tangential_disp = rel_disp - n*(n.rel_disp); lam_t projected to the Coulomb cone via contact_tangent_rho ~ soft_contact_kf), NOT velocity-level regularization. A too-soft kf therefore allows tangential CREEP within the cone -> directly the ruling's hypothesis.
