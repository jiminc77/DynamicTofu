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
