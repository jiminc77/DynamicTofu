# W1 Pre-registration (Frozen at P0)

The machine-readable authority is `ralph/results/prereg_w1.json`; this document mirrors it. Provenance: pre-edit `src/vbd_rig2.py` SHA-256 `11011fb9e53544d4da75f1ad1e17932ccfc9d867e81eb8c146eb994209156475`; Newton `b74df534`.

## Profile and exact timebase

Axis is world-x. The one-reversal jerk-limited trapezoid uses `T_j=0.05 s`, `T_a=0.10 s`, `T_c=0.10 s`, dwell `0.30 s`, settle `1.00 s`; transport is always `1.30 s`. It travels out (+x), fully stops, returns (-x), and settles. `Δv=0.15a`, each leg displaces `0.045a` metres, and jerk is `a/T_j`.

The absolute phase machine is ramp `[0,0.8)`, preload `[0.8,1.8)`, lift `[1.8,4.3)` (50 mm in 2.5 s), hold `[4.3,9.3)` (5 s), transport `[9.3,10.6)`, settle `[10.6,11.6]`; trial length is 11.6 s.

Time is indexed by `k=round(t*4800)` (60 fps × 80 substeps). Membership uses integer comparisons without float epsilon. All windows are half-open except final settle, closed at 11.60.

| Window | seconds | indices |
|---|---:|---:|
| accel_out | [9.30,9.50) | [44640,45600) |
| cruise_out | [9.50,9.60) | [45600,46080) |
| decel_out | [9.60,9.80) | [46080,47040) |
| dwell | [9.80,10.10) | [47040,48480) |
| accel_back | [10.10,10.30) | [48480,49440) |
| cruise_back | [10.30,10.40) | [49440,49920) |
| decel_back | [10.40,10.60) | [49920,50880) |
| settle | [10.60,11.60] | [50880,55680] |

Strictly-interior fit windows are 9.35–9.45 `[44880,45360)`, 9.65–9.75 `[46320,46800)`, 10.15–10.25 `[48720,49200)`, and 10.45–10.55 `[50160,50640)`.

## Grid, extension, and gates

Primary grid: `a={1,2.5,5,10,20,30} m/s²`, `F={0.4,0.6,0.8,1.0,1.2,1.5,2.0} N`, `E={7,15,25} kPa`: 126 cells. T-EXT adds `{2.5,3.0} N` for at most 8 rows, priority descending a then ascending E.

| Certified row topology | Action |
|---|---|
| all slip | extend to 2.5, 3.0 |
| slip→intact, intact at ceiling | extend upper edge only |
| intact then slip re-entry at ceiling | no extension |
| damage at ceiling | no extension |
| multiple flips | STOP/escalate |
| uncertified deciding coordinate | no extension; unresolved |

New flips and a closure-supporting 3.0 N censoring endpoint require three seeds.

G-TRK fits `body_qd[b_palm][0]` and requires maximum relative acceleration error ≤5% at every a, at least 5 samples in every window, and zero-command noise ≤0.01 m/s² (no absolute a=1 tolerance floor). Any miss fails closed.

VG uses device-side evidence every solver substep against the contacted soft feature: relative displacement ≤0.5 mm, zero record-dropout substeps per pad, zero overflow substeps, finite state, and four valid fits. Reset is per frame; download is end-of-frame and every 20 transport substeps; coverage is all substeps. A first isolated interior failure is recorded uncertified and censored from all inference while the row continues. A deciding-coordinate or second row failure stops the row and escalates. Uncertified evidence never supports a boundary, extension, closure, or classifier; substeps remain frozen.

## Judgment and classification

At `t_ref=9.30 s` (`k=44640`), transport slip is the maximum over `[9.30,11.60]` of `||(COM-p_palm)-(COM_ref-p_palm_ref)|| > 0.002 m`, with `p_palm=body_q[b_palm][:3]` and translation-only frame. Legacy hold is `[4.30,9.30)`. Damage latches when at least `DVF_MIN=0.005` of tets have temporal-maximum principal Green strain above `eps_damage=0.15`; twelve phase maxima are retained. Damage precedes slip only when `damage_latch_t < drop_t`.

Classifier first-match order: **INCONCLUSIVE → P-B CLOSURE → P-A CONTRACTION → P-C NO EFFECT → MIXED/NON-MONOTONE**. Inconclusive means deciding evidence is uncertified/unconfirmed. P-B needs a confirmed empty band (`a*` only for persistent closure at all higher tested a). P-A needs no closure, non-decreasing `F_lo`, confirmed endpoints, and ≥0.2 N endpoint rise. P-C needs identical label vectors across a. Everything else is mixed.

## Prediction

`F_slip(a)=m sqrt(g²+a²)/(2 mu_eff)`, with `m=0.0640 kg`, weight `0.628 N`, `g=9.81`, and `mu_eff=0.449`. The 0.70 N intercept was calibrated from the frozen `(0.6,0.8]` bracket (`0.628/1.40`); acceleration dependence is out of sample and no post-hoc refit is allowed.

| a | 1 | 2.5 | 5 | 10 | 20 | 30 |
|---:|---:|---:|---:|---:|---:|---:|
| prediction N | 0.70 | 0.72 | 0.79 | 1.00 | 1.59 | 2.25 |
| bracket N | 0.60–0.80 | 0.62–0.83 | 0.67–0.90 | 0.86–1.14 | 1.36–1.82 | 1.93–2.57 |

Observed edge is `(F_last_slip,F_first_intact]`; intersecting bracket is a hit, non-intersection a miss, censoring unscored.

## W2 and W3

For W2, `INT(E)` intersects confirmed certified intact F across compared a. Choose its median (lower tie), then enforce `F* ≥ F_lo(E,a_max)+0.2`: choose the smallest satisfying element, or the maximum and declare margin failure. If empty, use the maximal contiguous a subset beginning at 1 with nonempty intersection; fewer than two levels skips the material. Falsifier `R` is peak tangential/normal resultant. It is SUPPORTED only when the three-seed range at maximum a is strictly disjoint from and above the minimum-a range; otherwise NOT SUPPORTED. Always report signed median difference. It is UNAVAILABLE if either endpoint slips/is uncertified or >20% samples are excluded (normal <0.05 N or no contact).

W3 uses actual W1 trials only. Scene A searches E15, E25, E7 at `F*`, a=1, intact/certified, then confirmed certified intact cells by lowest a and closest F. Scene B uses exactly Scene A's E/F and the highest certified slip a. Scene C is the lowest-a certified damage cell at highest available F. Missing scenes are unavailable, never substituted. Reruns preserve seed/config and reproduce labels; frames are grasp, lift-complete, transport-start, plateau-peak, reversal, settle-end.

## Reduction ladder and acceptance

Order is E7→E25→E15 and, within material, a `[1,5,10,20,30,2.5]`; drop order is a=2.5 then remaining E15. PF includes median runtime × remaining approved target, confirms, worst-case capped T-EXT, and 6 h W2–W4 reserve. LT-1 (2026-08-29 09:00 KST) proposes dropping a=2.5 if PF exceeds freeze−2 h; LT-2 (21:00) proposes remaining E15; LT-3 (2026-08-30 12:00) is a hard acquisition cutoff after the in-flight row.

Acceptance requires: pre-registration provenance on every receipt; three-anchor/three-seed G0′ equivalence; complete G-TRK and structural evidence; complete judgment/VG/health receipts; unanimous three-seed boundary manifests; valid reproducible band schemas; phase diagram/classifier/prediction report; confirmed or explicitly unresolved T-EXT; ATTR-gated, bitwise-recomputable W2 plus overlays/equilibrium/falsifier; source-traceable same-grip W3 with unavailable scenes documented; hashed G0′/G-TRK/ATTR media; complete W4 results/handoff/decisions/manifest/unresolved list; frozen allowlist proof with no MPM run; and structural body/joint/DOF/pad/snapshot-render tests.


## Post-registration spec fixes

### 2026-08-28 — G0'-prime damage-branch DVF criterion (external ruling)
The ±20% per-seed DVF equivalence tolerance is incoherent with the damage branch's intrinsic ~60% relative seed spread. Amended (external sign-off): a damage-branch cell's DVF PASSES iff label-equivalence across the 0.5% threshold AND the extended per-seed DVF is within the baseline seed range widened by 20% of that range, OR |delta| <= 0.01 absolute, whichever is looser. All other G0'-prime criteria (label, hold-slip ±0.15 mm, COM-z RMS ≤0.5 mm, P99 ±0.02) unchanged. See ralph/DECISIONS.md 2026-08-28.


### 2026-08-28 — Realized-acceleration axis + G-TRK repeatability gate (external ruling, option A)
D3-C feed-forward is clamped by the j_x position PD (ineffective at any mass; free-carriage receipt in reports/vbd/g_trk_gate.md: realized 3.311/3.301/2.986 m/s^2 at FF 0/0.73/25 N). Amendment: (1) commanded levels are exposure levels; the scientific axis uses per-level REALIZED MEDIANS. (2) G-TRK redefined as a repeatability gate -- per commanded level, realized plateau accel monotone + well-separated + CV<=5% across cells/materials/grasp-states; r^2 shape bound and zero-command noise floor <=0.01 retained; any level failing -> STOP. (3) FF code path disabled (transport_ff_mass=0.0). (4) Expected realized ladder ~{0.6,1.6,3.2,6.4,12.8,19} m/s^2; top realized level keeps the predicted slip boundary (~1.5 N) inside the frozen grid so T-EXT is unchanged. See ralph/DECISIONS.md 2026-08-28.
