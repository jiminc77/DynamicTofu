# W1 transport slip metric -- reliability failure (STOP + escalate)

Run 01a046eb / G3 P4 (W1 screen). The pre-registered transport slip metric is `slip3d = max over [9.30,11.60] s of || (COM_block(t) - p_palm(t)) - ref_9.30 || > 2 mm`. Early screen cells (E7, a=1) exposed that this metric is non-reproducible and does not measure grip failure.

## Evidence
Same deterministic cell E7 / F0.8 / a=1 / seed 0, three runs:
- screen batch run:      slip3d = 7.60 mm -> label slip
- isolated `--cell` run: slip3d = 5.30 mm -> label slip
- clean inline diagnostic: MAX slip3d = 1.81 mm, SETTLE-mean(last 0.3 s) = 1.73 mm, **x_settle = 0.00 mm** -> block returns to grip

Clean diagnostic across the a=1 row (settle-end / permanent):
- E7/F0.6/a1: max 1.42, settle 1.31, **x_settle -0.01** (intact)
- E7/F0.8/a1: max 1.81, settle 1.73, **x_settle 0.00** (intact)
- E7/F1.2/a1: max 0.34, settle 0.34, **x_settle 0.00** (intact)

## Diagnosis
1. **The grip HOLDS at a=1**: the block returns to the palm along the transport axis (x_settle ~ 0 for all F). There is NO permanent transport-axis slip -- these cells are physically INTACT.
2. The ~1.7 mm 3D settle residual is a Y/Z grip re-seating offset (transport is along x), not slip.
3. `slip3d = max-3D` captures the chaotic TRANSIENT wobble during acceleration (1.8-7.6 mm near the marginal grip) plus the off-axis Y/Z offset -- not grip failure.
4. VBD has run-to-run GPU non-determinism (already seen at G0': damage DVF varied 0.0273<->0.0342 same seed). Near the marginal grip the transient wobble amplifies this -> slip3d varies 5.3-7.6 mm across identical runs. Single-seed screen labels near boundaries are therefore NON-REPRODUCIBLE, and 3-seed confirmation would be non-unanimous (unconfirmable boundaries).

## Proposed fix (needs external sign-off; pre-registered metric change)
Redefine the transport slip label as PERMANENT slip: the block-palm relative displacement ALONG THE TRANSPORT AXIS (world-x) at settle-end (after the block returns and the wobble damps), i.e. `x_perm = | (COM_block_x - p_palm_x)(settle) - (COM_block_x - p_palm_x)(9.30) | > 2 mm`. Rationale:
- Measures actual grip failure (block permanently displaced out of grip) vs recoverable elastic wobble.
- Reproducible: x_settle is robust (~0 held, large ejected) where max-3D is chaotic.
- Still separates INTACT (a=1, x_perm~0) from SLIP (high-a ejection, x_perm large) -> the band contraction the paper measures.
- Keep the transient max-3D and the Y/Z residual as RECORDED diagnostics (not the label), and keep DVF (damage) unchanged.

Options: A (recommended) permanent transport-axis slip at settle; B keep max-3D but raise threshold + require settle-recovery check; C characterize near-boundary cells probabilistically over many seeds. No further screen cells run until the slip label is re-operationalized and signed off.

## v2.2 acceptance-test result: FAILS the a=1 sanity row (STOP; per-axis table)

a=1 E7 row under v2.2 (slip = sqrt(x_res^2 + z_res^2) permanent tangential, grip ref t=1.80s -> settle [11.30,11.60]):

| F(N) | label | tang(mm) | x_res | z_res | y_res | dvf | legacy_hold_z(mm) |
|---|---|---|---|---|---|---|---|
| 0.4 | slip | 16.80 | -0.14 | -16.80 | 2.81 | .0013 | 6.33 |
| 0.6 | slip | 4.30 | -0.02 | -4.30 | -1.07 | .0013 | 1.36 |
| 0.8 | slip | 2.51 | 0.00 | -2.51 | 20.98 | .0013 | 0.69 |
| 1.0 | slip | 2.07 | 0.00 | -2.07 | -4.34 | .0007 | 0.48 |
| 1.2 | intact | 1.85 | 0.00 | -1.85 | 24.31 | .0007 | 0.36 |
| 1.5 | intact | 1.73 | 0.00 | -1.73 | -25.95 | .0026 | 0.28 |
| 2.0 | slip | null | 0 | 0 | 0 | .0420 | 0.17 |

Three compounding problems:
1. BAND SHIFTED UP: intact only at F>=1.2 (not 0.8-1.5). The z_res is MONOTONE (16.8,4.3,2.51,2.07,1.85,1.73) crossing 2mm between F1.0/F1.2. Root cause: v2.2 permanent z_res spans the WHOLE trial (grip 1.80s -> settle 11.5s) = lift + hold + transport + settle z-drift, which is systematically ~1.5-1.8mm larger than the frozen quasi-static HOLD-only slip (compare z_res vs legacy_hold_z: F0.8 2.51 vs 0.69, F1.0 2.07 vs 0.48). So the reference/window choice, not the physics, moved the boundary.
2. LARGE PERMANENT Y (grip-normal) MOTION even on intact cells: F0.8 +20.98mm, F1.2 +24.31mm, F1.5 -25.95mm. The soft block permanently slides/extrudes ~25mm ALONG the grip axis under the world-x transport (block is 40mm). Demoted per the ruling, but 25mm is not re-seating noise -- it may be a real gross grip failure or a soft-block artifact of the world-x transport.
3. F2.0 labeled slip (drop/eject precedence) despite dvf=0.042 (damage) -- under transport the damage-vs-drop precedence differs from the frozen quasi-static E7/F2.0=damage.

Plus VBD run-to-run non-determinism near the boundary (F0.8 z_res was 0.69 in the v2.1 run, 2.51 here).

Per the ruling, STOP with the per-axis residual table. Decision needed: (a) measure the quasi-static (hold) z-creep and the transport (post-9.30) x-z slip SEPARATELY (hold-window z + transport-window x-z, each vs its own reference) rather than a single whole-trial residual -- this would restore the frozen band at a=1; (b) how to treat the 25mm permanent y-extrusion (real slip vs demoted artifact); (c) F2.0 damage-vs-drop precedence under transport. This may warrant reconsidering the transport-slip measurement design.

## v2.3-final validation + two remaining methodology decisions (a=1 E7, seed 0 unless noted)

v2.3 items 1/2/3 IMPLEMENTED and largely validated:
- ITEM 2 CONFIRMED (rig artifact): grasp_frame_y_res = 0.0 mm on every cell while palm-frame assembly_drift = 1.8/-0.6/7.0/26.9 mm. The block stays centered between the fingers; the 21-27 mm is common-mode finger+block drift with grasp intact (force-controlled fingers have no common-mode y restoring force). escape_mode=None everywhere. Demoted as assembly_drift, per ruling.
- ITEM 3 (timestamp precedence) works structurally: F2.0 -> damage (dvf 0.0426), F1.0-1.5 intact.
- ITEM 1 (separate windows) restores the band STRUCTURE: F0.4 slip (hold_z 12.31), F0.6 slip (3.01), F1.0-1.5 intact (1.78/1.60/1.59), F2.0 damage.

Row: | F | 0.4 | 0.6 | 0.8 | 1.0 | 1.2 | 1.5 | 2.0 |
     | label | slip | slip | SLIP | intact | intact | intact | damage |
     | hold_z_mm | 12.31 | 3.01 | 2.02 | 1.78 | 1.60 | 1.59 | 1.73 |

TWO REMAINING DECISIONS (screen stays stopped):
1. F0.8 BOUNDARY SHIFT. F0.8 hold_z = 2.02/2.01/2.06 mm across seeds 0/1/2 (REPRODUCIBLE, right on the 2 mm line) -> slip. The frozen quasi-static band had E7/F0.8 = intact (hold slip 1.32 mm). So the transport rig's E7 slip/intact boundary is ~1 F-level higher (intact starts at F1.0, not F0.8). Not non-determinism (tight 3-seed). Decision: accept the shift (resume; two-thirds/3-seed confirmation handles boundaries in P5) OR investigate why the transport-rig hold creep (2.0) exceeds the frozen rig (1.32) at E7/F0.8.
2. DAMAGE-DVF CONTACT-TRANSIENT INFLATION (affects the whole screen). damage_latch_t fires during the RAMP: F0.8 seed1 dmg_t=0.033s (dvf 0.00716>0.005 -> spurious damage), F2.0 dmg_t=0.55s. The DVF temporal-max is catching finger-contact indentation strain (>0.15 local) during preload, not transport damage. E7/F0.8 DVF straddles 0.005 (0.0013-0.00716) -> intact/damage flip. The frozen band used the whole-trial DVF and labeled E7/F0.8 intact, so this contact-transient inflation is new/amplified in the transport rig. Decision: restrict the DVF/damage-latch to post-lift (exclude preload contact transients) -- a pre-registered damage-window change needing sign-off; damage threshold 0.005 unchanged.

VBD non-determinism confirmed (F0.8 seed1 dvf 0.00716 vs seed0/2 ~0.0013) -> 3-seed/two-thirds mandatory as ruled.
