# DynamicTofu — Handoff State (2026-08-28)

Session handoff for a fresh continuation. **Read this first**, then `ralph/DECISIONS.md` (full audit) and `reports/vbd/tofu_band_summary.md` (current result).

## Where we are

The project pivoted from MPM to a **VBD** tofu-grasping rig after MPM could not hold soft tofu and its "empty band" result was shown to be contaminated by solver/contact artifacts (see the MPM archive below, FROZEN). The VBD rig now produces a **non-empty intact grasp-force band** with a validated Coulomb oracle. Day-2 grid + judgment-v2 damage labeling is the current phase; the **acceleration (transport) axis has NOT been started** (do not start it without reading "Open items").

## Validated rig (USE THIS)

`src/vbd_rig2.py` — pure `SolverVBD` (NO SolverCoupledProxy; the proxy path drove an uncapped PD = the V-2 ejection root cause). Key facts baked in:
- Floating 3-DOF gantry gripper: world -> Z-prismatic (position PD, lift) -> palm -> 2 finger prismatics FORCE-controlled via `Control.joint_f` (target_ke=0). **Closing sign: +joint_f along the inward axes closes** (verified).
- `enable_rigid_soft_full_surface_contact=True` (per-particle contact alone slips).
- **friction_epsilon = 2e-4** (velocity-regularized Coulomb; default 1e-2 creeps; 2e-6 is worse). `soft_contact_kf` is DEAD/unused. Friction mu = geometric mean of the two shape mus; contact ke = arithmetic sum of both sides -> set `contact_ke`/`mu_pair` on BOTH.
- **substeps = 80 REQUIRED** (substeps=40 is chaotic/non-deterministic; 80 is reproducible/unanimous across seeds).
- **MASS FIX**: `correct_mass=True` rescales soft particle_mass to density*volume (add_soft_grid over-lumps +42%). The block is the intended **64 g / 0.628 N** (4 cm cube, density 1000).
- Tofu production params: E in {7,15,25} kPa, nu 0.45, h=5mm (cell_m=0.005), r=2.5mm, ke=pad=1e3, kd=1.0, mu_pair=1.0, eps=2e-4, margin=1e-3, lift 50mm/2.5s, hold 5s.
- Instrumentation: per-tet Green strain (`strain_stats`, `strain_field` -> per-tet max principal), contact count, gripper-relative slip, applied Fn + equilibrium check.

Runners: `scripts/vbd/{tofu_probe,tofu_grid,tofu_grid_confirm,tofu_finalize,tofu_label,p1_oracle,p1_final}.py`.

## Judgment v2 (VBD) — approved

- **slip** = >2 mm gripper-relative displacement over the 5 s hold.
- **damage** = volume-weighted damaged-volume fraction (DVF = tets with eps1 > 0.15 max principal Green strain) **>= 0.5%**, LATCHED over the trial (temporal-max field). A single-tet exceedance never flips a label.
- **intact** otherwise. Precedence: damage>slip only if damage latches before drop (v1 rule).
- eps_damage = **0.15** (tensile failure anchor ~15%; compression 45-54% is higher, so tensile-anchored is conservative).

## Band result (current)

Non-empty intact band at substeps=80, 64 g tofu: all E {7,15,25} kPa **slip at F<=0.6 N, intact at F>=0.8 N** (block lifts ~48 mm, slip<2mm). Boundary UNANIMOUS across 3 seeds (`tofu_grid_confirm.json`). Mesh-invariant h5->h4mm (`tofu_meshconv.json`). Damage branch at high force (P99 principal strain rises; E7/F2.0 peak ~0.26). Final judgment-v2 labels/phase diagram: `reports/logs/vbd/tofu_labels_v2.json` + `reports/vbd/tofu_band_summary.md`.

## Artifact paths

- Band: `reports/logs/vbd/tofu_grid.json` (21 cells), `tofu_grid_confirm.json` (3-seed boundary), `tofu_meshconv.json`, `tofu_labels_v2.json`, per-tet strain fields `reports/logs/vbd/strain_fields/*.npz` (temporal-max).
- Oracle: `reports/logs/vbd/{p1_oracle,p1_final,lever0_*}.json`, `reports/vbd/p1_oracle_report.md`.
- Clips: `reports/media/tofu_{hold,slip,highstrain}.mp4`, `p1_pass_E100_2N.mp4`, `tofu_hold_E15_F12_sub80.mp4`.
- Summaries: `reports/vbd/tofu_band_summary.md`, `reports/HANDOFF_STATE.md` (this).
- Ledger: `ralph/DECISIONS.md`, `ralph/RESULTS.md`.

## Open items (NOT started; needs external go per phase)

1. **Acceleration / transport axis (E1-equivalent)**: prescribe lateral palm motion (the floating gantry has no Franka accel ceiling; record commanded AND realized accel). Sweep accel x (E, F) within the intact band. DO NOT START without the external go.
2. **E2 tactile source** = the per-pad CONTACT RECORDS (already logged as `contacts` count; extend to per-pad Fn/Ft time series and contact positions for tactile traces).
3. **E3 clips**: intact / slip / damage exemplars (three exist; add per storyline).
4. **Damage-branch coverage above 2 N** if the band's upper (damage) boundary needs resolving (current grid tops out at 2 N).
5. Full Franka-arm port (after the floating-gripper science is locked).

## Standing rules (carry forward)

- **Files-not-chat**: numbers live in `reports/logs/**` + `ralph/results/**`; decisions/gate-calls in `ralph/DECISIONS.md`; one RESULTS.md row per batch. Never assert a number that isn't in an artifact.
- **Pre-registration**: thresholds/grids frozen before running; protocol changes need external sign-off (recorded before use).
- **Clips are mandatory** for verification of any hold/slip/damage claim.
- **MPM archive is FROZEN** — no further MPM runs; MPM E1 data + gate receipts (`ralph/results/`, `reports/gN*.md`, `gateA/B/B2/C`, `validity_gate_synthesis.md`) are read-only history.
- **Commit per phase**; terminal-critic gate before any pause; fail-closed on gate miss.
- Newton pinned at `b74df534`; run via `cd newton && uv run --no-sync python ...`.
