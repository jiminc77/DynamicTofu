# Stage-A Shape Checkpoint — outcome: scale_failure (pre-registered router, branch 1)

Ran 2026-08-27 ~06:45 KST on the complete Stage-A dataset (75/75 trials, 0 unresolved, health clean throughout; artifacts `ralph/results/trials/`, aggregate `ralph/results/e1_band_3333.json`, router record `ralph/results/e1_shape_checkpoint.json`).

## Result

**Every intact band at σ_Y=3333 is EMPTY at all five accelerations** — zero intact cells anywhere in the 75-trial Stage-A grid. Every cell colors `drop` (low F_g: the grip cannot carry the block through lift+transport) or `damage` (high F_g: crush onset, unanimous across seeds from F=3.5 N at a=1, earlier at higher accelerations). The drop→damage transition happens with no intact window in between.

Router branch 1 (fixed order): all accels empty ⇒ **scale_failure** — "the ladder is mis-centred: a scale result, not a shape result. Report, escalate. No index arithmetic." Per the pre-registered escalation gate, **no further stage runs without external review** (Stage B/C halted; E2 mid-band selection has no usable band; E3 has no intact/slip source trials).

## Why this is physically coherent (not a rig fault)

- G-N2 measured the specimen bearing capacity at ≈2.84 N/finger (σ=3333): above it, force saturates and crushing begins; the damage onsets observed in Stage A (F≥3.5 N commanded) match the ladder-gate onset (5.575 N bilateral sum).
- Below crush forces, the E=7 kPa block cannot be rigidly carried by fingertip pads (pads indent ~1 cm and carve through under the pre-registered 5 cm/0.3 s lift; G-N2 clips show held elongation, not carry). The judgment window then registers `drop` via gripper-frame displacement.
- Seeds are unanimous in every completed cell (no boundary noise): this is a scale property of the (material, gripper, lift profile) triple, not statistical ambiguity. Realized tracking was clean throughout (a_real within ~3% of commanded; f_g_realized within ~2% below saturation).

## Decision needed (external, pre-registered)

1. **Accept the finding and complete the record**: run Stage B (σ=2000 and 6000, 150 trials ≈ 95 min) to establish whether an intact window exists for firmer material (σ=6000 predicts later crush onset; carry physics may still forbid intact transport) and publish the phase diagram as measured — drop/damage structure with empty intact bands is a legitimate, pre-registered reportable outcome (cf. the σ=2000 flag you pre-registered).
2. **Pivot within the pre-committed axes** (requires sign-off per the amended brief): the profile family (reversal sharpness / curvature) does not change liftability; a grid change (lower F_g resolution near the drop/damage transition) is a coordinate-grid change requiring sign-off and does not create an intact window by itself.
3. **Protocol revision** (e.g., gentler lift profile) — changes a pre-registered phase constant; largest scope.

Recommendation: **option 1** — the dataset is scientifically coherent and complete-able within budget; Stage B cross-material evidence strengthens whatever the paper claims about the intact-window's existence boundary.
