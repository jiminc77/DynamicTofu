# HANDOFF_STATE — IROS26 workshop sprint session-2 (W1–W4) — FINAL

Run 01a046eb. Newton pin b74df534. Freeze target 2026-08-31 12:00 KST. Transport is **effort/force-controlled** (position + velocity feed-forward on the world-x carriage DOF; acceleration feed-forward was evaluated and disabled — see below).

## Rig (final)
Frozen VBD rig extended with a **world-x transport DOF**: chain world → j_x (prismatic, world-x) → carriage (mass 0.050 kg, full-rank inertia diag 1e-4) → j_z (lift) → palm → 2 fingers. Rig SHA256 b8c4768. The frozen contact/material/solver foundation is **untouched** (substeps=80, friction_epsilon=2e-4, mu=1.0, ke=1e3, kd=1.0, soft_contact_margin=1e-3, cell_m=0.005, particle_radius=0.0025, correct_mass→64 g, E∈{7,15,25} kPa, nu=0.45). The transport-off extension was proven **non-perturbing** by the G0′ equivalence gate (9/9, amended damage-DVF criterion). Carriage default-inert; velocity feed-forward mandatory (else the j_z-style PD fights transport).

## Judgment v2.3 (FROZEN — any further metric anomaly is a STOP)
- **slip** iff (hold-window z-creep over [4.3,9.3] s vs the pre-lift grip > 2 mm) OR (transport-window permanent x–z displacement at settle-end [11.30,11.60] vs t=9.30 > 2 mm) OR drop/eject OR grasp-frame lateral escape > 10 mm.
- **damage** iff post-grip-completion (1.80 s) DVF ≥ 0.005 (latched before drop).
- else **intact**. Grasp-frame y (block vs finger-midpoint) labels; palm-frame y is common-mode **assembly_drift** (demoted rig artifact — the force-controlled finger pair has no restoring force on its common y mode).
- Transport windows/timebase: 4800 Hz; transport [9.30,11.60]; four accel plateaus (9.35-9.45/9.65-9.75/10.15-10.25/10.45-10.55). Realized-acceleration axis (frozen G-TRK): commanded {1,2.5,5,10,20,30} → realized {0.681,1.647,3.183,6.402,12.889,19.846} m/s².

## W1 — acceleration-dependent grasp-stability band (result)
126-cell screen (E{7,15,25}×F{0.4..2.0}×a{1,2.5,5,10,20,30}, 0 failures) + 56 boundary cells 3-seed-confirmed (112 seeds, two-thirds; **0 UNRESOLVED**). The **intact "safe grasp" band contracts monotonically with realized acceleration and grows with stiffness**: intact-cell count E7 3→2→0, E15 5→4→0, E25 5→4→1→0 across a=1..30. Phase diagram: reports/vbd/w1_accel_phase_diagram.md; final bands reports/logs/vbd/final/e1v2_band_{7,15,25}.json. **T-EXT: 0 rows triggered** (no certified deciding cells + slip boundary within the frozen F grid). Contraction claims are measured **within-rig** against the a=1 reference row.

## W2 — tactile-proxy (geometry-only; ATTR probe = GEOMETRY_ONLY)
Per-pad **contact-centroid excursion (pad-frame)** increases monotonically with realized accel (~0.3 mm at a≈0.68 → ~10 mm at a≈13). **Falsifier PASS**: 3-seed endpoint ranges strictly non-overlap (a=1 [0.25,0.30,0.29] mm vs a=10 [9.15,9.19,9.07] mm; signed median diff +8.86 mm). Peak tangential/normal **ratio UNAVAILABLE** (per-pad contact forces not attributable under geometry-only; never fabricated). Artifacts: reports/logs/vbd/e2v2_tactile.json + 18 raw NPZ + overlays + reports/vbd/w2_tactile.md + e2v2_falsifier.json.

## W3 — demo clips
Three scenes from actual confirmed W1 trials, labels reproduced on re-run: intact E15/F1.2/a1, same-grip high-accel slip E15/F1.2/a30 (strict same-grip predicate with the intact scene), high-grip damage E7/F2.0/a5. reports/vbd/clips/w3_{intact,slip,damage}.mp4 + 7 key frames/scene + manifest reports/vbd/w3_clips.md.

## Gates (all recorded, files-not-chat)
G0′ equivalence 9/9 (g0_equivalence.json); G-TRK realized-axis repeatability PASS (g_trk_ladder.json + g_trk_axis.json); ATTR = GEOMETRY_ONLY (w2_attr_probe.json). Pre-registration reports/vbd/PREREG_W1.md + ralph/results/prereg_w1.json (4 spec amendments). Full decision log ralph/DECISIONS.md. Manifest reports/MANIFEST.json (sha256 + git commit).

## Unresolved / fail-closed
1. **Peak tangential/normal force ratio (W2): UNAVAILABLE.** The P3c ATTR probe was GEOMETRY_ONLY — SolverVBD.body_forces reflects the finger closing effort, not the isolated block-pad contact reaction (block-absent control: 1.2 N per pad with the block moved +10 m away). W2 falls back to geometry (centroid excursion). Pre-registered + user-accepted; never fabricated.
2. **VG universal uncertification.** Every screen cell is validity-gate uncertified via vg2 (contact-record continuity: ~11-15% zero-record substeps from VBD soft-contact flicker), even intact cells with tiny vg1 displacement (0.07-0.12 mm). Primary slip/damage labels are unaffected. Consequence: T-EXT eligibility was disabled (no certified deciding cells) — moot because the slip boundary sits within the frozen F grid. Flagged for review; NOT silently altered (metric frozen).
3. **F0.8 within-rig offset.** At E7/F0.8 the transport-rig suspended-hold creep is ~2.0 mm (reproducible 3-seed) vs 1.32 mm on the frozen quasi-static rig — a ~+0.7 mm rig-chain offset (G0′ anchors did not cover this cell). Accepted by external ruling; all W1 contraction claims are within-rig vs the a=1 reference, never vs the frozen band. reports/vbd/w1_rig_offset_note.md.
4. **VBD run-to-run non-determinism** (transport trials are NOT deterministic, unlike the quasi-static hold; e.g. damage DVF varied 0.0273↔0.0342 same seed). Mitigated by the mandatory 3-seed two-thirds rule at every label boundary; all 56 boundaries resolved unanimously or by two-thirds.

Everything above is committed; nothing is papered over.
