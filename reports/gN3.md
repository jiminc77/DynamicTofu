# G-N3 Gate Receipt — One Full E1 Cell End-to-End

- Gate: G-N3 (deadline 2026-08-30 12:00 Asia/Seoul)
- Executed: 2026-08-27 ~05:40 KST — **2 days 6 h ahead of the cutoff**
- Verdict: **PASS** — one complete E1 cell end-to-end with judgment v1 labels, schema-valid `e1.v1` JSON, measured wall-times, computed schedule selection committed before any sweep.

## The official cell

`ralph/results/trials/s3333_a2.5_f0.8_seed0.json` (schema `e1.v1`, validated on write; full config block incl. `brief_sha256`, Newton commit, asset hash, all P2/P4/P5 protocol constants, calibration, coupling params, judgment thresholds, seed-RNG derivation):

| Quantity | Value |
|---|---|
| Phases executed | settle 0.5 / close+hold 0.5 (ramp 0.3) / lift 5 cm in 0.3 / hold 0.2 / **trapezoid transport with reversal** (t=2.632 s) / settle 0.5 — timestamps monotone, recorded |
| Labels (judgment v1, window lift-complete→settle-end) | `drop`; `cell_color = drop` (precedence rule recorded; `damage_latch_t = None`, `drop_t` recorded) |
| Peak damage fraction | 0.0464 (always recorded) |
| `a_peak_cmd` vs `a_peak_realized_ms2` | 2.5 vs **2.424** (3.1% tracking error — far inside the 25% escalation trigger) |
| `f_g_n` vs `f_g_realized_n` (P4 observable) | 0.8 vs **0.812** (1.5%) |
| Solver health | clean (no NaN/inf, max particle speed ≤ 5 m/s, sparse grid ok) |
| Wall time | 34.4 s |

## Dynamic-path reach

The transport executed the full ±y trapezoid with 3.1% acceleration tracking and 2.42 m/s² realized peak; the G-N2 kinematic probe (both frozen criteria, worst joint margin 0.735 rad, perturbation-solvable at every waypoint, `gn2-reach.json`) plus the executed dynamic path with clean IK servoing confirm reach on the dynamic path.

## Determinism (frozen tolerances) — `reports/logs/gn3-official.json`

- Same seed re-run: labels **identical** (`drop`/`drop`), peak-fraction delta 0.0039 (≤ 0.02), realized-accel delta 0.03% (≤ 5%) — **PASS**.
- Different-seed divergence (frozen-RNG guard): at this benign cell, seed effects (peak fractions 0.0464/0.0389/0.0415 — 19% relative spread) sit *inside* the conservative frozen tolerances, so the literal beyond-tolerance divergence does not trigger here. The RNG is proven live **structurally**: seed-derived pose jitter yields distinct block centroids (offsets 1.66 mm / 0.39 mm, spec ±1 mm; `gn3-jitter-structural.json`), and knife-edge cells show label-set sensitivity (below). Recorded as a deviation-with-evidence, not a pass-by-assertion.
- Knife-edge note (from `gn3-rehearsal/`): at (a=2.5, F=1.8) the peak fraction rides exactly on the 10% latch (0.1008 vs 0.0959 across same-seed replays) and the RAW label set flips (`{damage, drop}` vs `{drop}`), while the pre-registered `cell_color` precedence yields **drop in both** — the phase-diagram outcome is replay-stable even at the boundary.

## Wall-time and schedule (computed, inputs recorded — never a constant)

| Input | Value |
|---|---|
| T(a=1) / T(a=15) | 39.4 s / 29.6 s → weighted **T = 34.5 s** |
| T_E2 (full E2 config, raw fields, 200 Hz verified) | **32 s** |
| R_E2 = 9 × T_E2 × 1.25 | 360 s → **E2_overflow = 0** |
| E1_budget | 24.0 h |
| **T_A_max = 0.8 × E1_budget / 75** | **921.6 s** (nominal point, valid because E2_overflow = 0) |
| Selection (N·T ≤ 0.8·E1_budget) | **A + B(2000) + B(6000) + C(i) + C(ii) — up to 310 trials, nothing dropped, no escalation** (34.5 ≪ 921.6) |

Stage-set selection committed to `ralph/DECISIONS.md` **before any E1 cell**. Stage A alone projects to ~45 min.

## Video evidence (user-ordered amendment)

`reports/media/gn3_transport_cell.mp4` (30 frames, decode-verified 1280×640) + `gn3_transport_cell_key{0..4}.png` — full-cell transport incl. grasp, lift, reversal, settle. Fallback orthographic render path (deviation recorded in gN2.md).

## GPU-free suite

23/23 unit tests green (judgment boundaries incl. slip-drop precedence and cell-color rule, band estimator + checkpoint router fixtures, schedule selector incl. the reduced-budget 768 s escalation fixture, 4-profile analytic contract, schema round-trip/rejection, tactile bitwise recompute).
