# Results

Frozen W1/W2/W3 results generated from the committed receipts. Labels: **I** = intact, **S** = slip, **D** = damage.

## W1 — Acceleration phase diagram

### E = 7 kPa

| commanded a (m/s²) | F=0.4 N | F=0.6 N | F=0.8 N | F=1 N | F=1.2 N | F=1.5 N | F=2 N |
|---|---|---|---|---|---|---|---|
| 1 | S | S | S | I | I | I | D |
| 5 | S | S | S | D | D | D | D |
| 10 | S | S | S | D | D | D | D |
| 20 | S | S | S | D | D | D | D |
| 30 | S | S | S | D | D | D | D |
| 2.5 | S | S | S | I | I | D | D |

### E = 15 kPa

| commanded a (m/s²) | F=0.4 N | F=0.6 N | F=0.8 N | F=1 N | F=1.2 N | F=1.5 N | F=2 N |
|---|---|---|---|---|---|---|---|
| 1 | S | S | I | I | I | I | I |
| 5 | S | S | S | S | D | D | D |
| 10 | S | S | S | D | D | D | D |
| 20 | S | S | S | S | D | D | D |
| 30 | S | S | S | S | S | D | D |
| 2.5 | S | S | S | I | I | I | I |

### E = 25 kPa

| commanded a (m/s²) | F=0.4 N | F=0.6 N | F=0.8 N | F=1 N | F=1.2 N | F=1.5 N | F=2 N |
|---|---|---|---|---|---|---|---|
| 1 | S | S | I | I | I | I | I |
| 5 | S | S | S | S | S | S | I |
| 10 | S | S | S | S | S | S | D |
| 20 | S | S | S | S | S | S | D |
| 30 | S | S | S | S | S | S | S |
| 2.5 | S | S | S | I | I | I | I |

### Realized-acceleration axis and intact-band contraction

| commanded acceleration (m/s²) | 1 | 2.5 | 5 | 10 | 20 | 30 |
|---|---:|---:|---:|---:|---:|---:|
| realized median acceleration (m/s²) | 0.681 | 1.647 | 3.183 | 6.402 | 12.889 | 19.846 |
| E7 intact cells | 3 | 2 | 0 | 0 | 0 | 0 |
| E15 intact cells | 5 | 4 | 0 | 0 | 0 | 0 |
| E25 intact cells | 5 | 4 | 1 | 0 | 0 | 0 |

The within-rig reference therefore contracts as E7 **3 → 2 → 0**, E15 **5 → 4 → 0**, and E25 **5 → 4 → 1 → 0** as realized acceleration increases (OBSERVED; labels are 3-seed-confirmed, 0 UNRESOLVED).

**Pre-registered classifier verdict = INCONCLUSIVE (result is DESCRIPTIVE/PROVISIONAL, not P-A CONTRACTION).** Per PREREG_W1.md, uncertified evidence never supports the classifier and the classifier order begins INCONCLUSIVE. All 126 cells fail VG certification via vg2 (*zero record-dropout substeps per pad*) alone — a bar not achievable with VBD soft-contact flicker (even intact cells drop ~11–15% of substeps), whereas vg1 relative displacement is met (0.07–0.12 mm ≤ 0.5 mm). The contraction above is thus reported as observed/descriptive; the formal classifier is INCONCLUSIVE pending an external ruling that amends the vg2 certification rule or root-causes vg2. Escalated (see HANDOFF_STATE Unresolved).

**Known F0.8 rig offset:** the frozen quasi-static band remains valid for its rig. The W1 phase diagram is internally consistent on the W1 rig, and contraction is measured within-rig against the W1 `a=1` row rather than against the frozen band.

**T-EXT: 0 rows triggered.**

## W2 — Pad-frame tactile geometry

Centroid excursion is reported per left/right pad from the `e2v2_tactile.json` summary. `—` denotes a missing realized-acceleration value in that summary.

| E (kPa) | commanded a (m/s²) | realized a (m/s²) | left excursion (mm) | right excursion (mm) |
|---:|---:|---:|---:|---:|
| 7 | 1 | 0.673 | 0.205 | 0.273 |
| 7 | 2.5 | 1.642 | 0.828 | 0.973 |
| 7 | 5 | 3.261 | 4.416 | 3.756 |
| 7 | 10 | 6.342 | 8.884 | 8.989 |
| 7 | 20 | 12.905 | 9.737 | 9.877 |
| 7 | 30 | — | 8.906 | 8.902 |
| 15 | 1 | 0.681 | 0.229 | 0.220 |
| 15 | 2.5 | 1.649 | 0.733 | 0.664 |
| 15 | 5 | 3.180 | 3.399 | 3.412 |
| 15 | 10 | 6.401 | 9.165 | 9.128 |
| 15 | 20 | 12.865 | 9.455 | 9.542 |
| 15 | 30 | — | 8.101 | 8.175 |
| 25 | 1 | 0.680 | 0.145 | 0.361 |
| 25 | 2.5 | 1.594 | 0.599 | 0.498 |
| 25 | 5 | 3.213 | 3.357 | 3.251 |
| 25 | 10 | 6.401 | 8.983 | 8.942 |
| 25 | 20 | 12.970 | 10.438 | 10.495 |
| 25 | 30 | — | 8.503 | 8.445 |

**Falsifier: PASS.** At `a=1`, centroid excursion spans **0.25–0.30 mm**; at `a=10`, it spans **9.07–9.19 mm**. The ranges do not overlap; median difference: **8.86 mm**.

**Peak tangential/normal ratio: UNAVAILABLE (ATTR=GEOMETRY_ONLY).**

## W3 — Outcome clips

The frozen clip bundle contains reproduced intact, slip, and damage outcomes with the keys and labels documented in `reports/vbd/w3_clips.md`.

## Closure

These results characterize the frozen VBD rig and its **effort/force-controlled** transport. Scientific acceleration claims use the realized-acceleration axis; W2 force attribution is unavailable and remains geometry-only.
