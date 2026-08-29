# BRIEF_WS_V2 — IROS26 workshop sprint, session 2 (accel axis → E2 → E3 → freeze)

Deadline: paper submission 2026-09-01 (Google Form, user submits). Experiment freeze: 2026-08-31 12:00 KST. Today is 2026-08-28 evening. Everything in this brief runs on the ALREADY-VALIDATED VBD rig — do not re-derive or re-tune the foundation.

## Read first (in this order)
1. `reports/HANDOFF_STATE.md` — validated rig, judgment v2, band result, artifact map, standing rules.
2. `ralph/DECISIONS.md` — full ruling history (binding).
3. `reports/vbd/tofu_band_summary.md` — the quasi-static band (final labeled version).
4. `reports/consult-vbd2.md` §(d)(f) — friction/contact scaling rationale behind the frozen params.

## Frozen foundation (violations are gate failures)
- Rig: `src/vbd_rig2.py` — pure SolverVBD, full-surface rigid-soft contact, force closure via `Control.joint_f`, floating 3-DOF gripper (Z prismatic → palm → 2 finger prismatics).
- Params: substeps=80 (40 is CHAOTIC — forbidden), friction_epsilon=2e-4, mu both sides 1.0, ke both sides 1e3, kd both sides 1.0, soft_contact_margin=1e-3, h=5 mm, r=2.5 mm, tofu 4×4×4 cm, mass-corrected 64 g, E ∈ {7, 15, 25} kPa, ν=0.45.
- Judgment v2: slip = >2 mm gripper-relative displacement; damage = volume-weighted damaged-volume fraction ≥0.5% of tets with temporal-max principal Green strain >0.15 (latched); intact otherwise; damage-after-drop precedence as v1.
- Discipline: files-not-chat (numbers only via JSON receipts with config provenance incl. git SHA); pre-register grids/windows/labels before running; STOP+escalate on gate failure or protocol ambiguity; 3-seed unanimity required at every label boundary; mp4 + key frames for every gate and every claimed behavior; commit every phase; MPM archive stays frozen; closure wording is "effort/force-controlled" per DECISIONS.

## W1 — Accel-axis sweep (completes E1; THE paper axis)
Add the transport phase to the validated protocol: after lift+settle, translate the palm along ±y with a trapezoidal profile (out, full stop, return — one reversal), then final settle. Pre-register phase timestamps and judgment windows BEFORE running (same fixed-window rule as v1).
- Grid: commanded a_peak ∈ {1, 2.5, 5, 10, 20, 30} m/s² × grip F ∈ {0.4, 0.6, 0.8, 1.0, 1.2, 1.5, 2.0} N × E ∈ {7, 15, 25} kPa. Single-seed screen → 3-seed confirmation at every label boundary.
- The floating gripper has no arm saturation: record commanded AND realized palm acceleration; verify realized tracks commanded within 5% at every level (this is a gate — if tracking fails, stop and escalate).
- Hypothesis under test (pre-registered, all outcomes publishable): the intact band contracts with a — F_slip should rise like m·sqrt(g²+a²)/(2μ_eff); the damage boundary may descend via transient strains at reversal. Pivots: P-A contraction observed / P-B closure observed (a* where band vanishes) / P-C no accel effect (then characterize why with the realized-load traces).
- Deliverables: `e1v2_band_{7,15,25}.json` (schema mirroring e1_band.v1 + realized accel + per-phase strain maxima), phase-diagram tables in a report md, realized-accel table, boundary 3-seed receipts, one clip per outcome class.

## W2 — Tactile-proxy comparison (E2)
At fixed mid-band grip (pick the mid-band cell per material from W1's intact band; pre-register the choice), compare across a_peak levels: per-pad contact-record time series — normal resultant, tangential resultant, contact count, contact centroid, left-right asymmetry (all already computable from the VBD contact records; extend logging if a field is missing).
- Claim under test: same commanded grip, measurably different contact state under motion (transients at reversal).
- Deliverables: `e2v2_tactile.json` (raw per-frame series + summary stats: peak tangential ratio vs a, centroid excursion vs a), a small md report, and one overlay plot per material.

## W3 — Demo clips (E3)
Three scenes from actual W1 trials (no new physics): quasi-static intact transport / same-grip high-accel failure (if W1 produces one) / high-grip damage-branch. Standard projection clips + key frames are acceptable; a nicer render via the Newton viewer is welcome ONLY if it costs <1 h total.

## W4 — Freeze support (by 08-31 12:00)
RESULTS.md batch rows for W1/W2; all receipts committed; `reports/HANDOFF_STATE.md` updated to final; flag anything unresolved as fail-closed rather than papering over.

## Ops
- Escalations via the ultragoal ledger classification, as before. External judge (Mac session) polls ~2 min and answers dialogs.
- GPU budget guidance: W1 full grid ≈ 126 cells + confirmations at ~4–6 min/cell — batch by material, commit after each row; if wall-clock threatens the freeze, escalate with a pre-registered reduction proposal (drop a=2.5 first, then E=15) rather than silently truncating.
