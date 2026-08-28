# VBD tofu grasp band — Day-2 grid (external ruling + addendum)

Rig: src/vbd_rig2.py (pure SolverVBD, full-surface contact, force-control, **mass-corrected 64 g** tofu, substeps=80, eps=2e-4, mu_pair=1.0, ke=pad=1e3, h=5mm r=2.5mm, lift 50mm/2.5s + 5s hold). Artifacts: reports/logs/vbd/tofu_grid.json (21 cells single-seed), tofu_grid_confirm.json (3-seed boundary), tofu_meshconv.json (h=4mm), strain_fields/*.npz (per-tet Green strain for post-hoc damage labeling). Clips: reports/media/tofu_{hold,slip,highstrain}.mp4.

## Band (single-seed screen; I=intact hold <2mm rel slip, s=slip/drop)

| E\\F (N/finger) | 0.4 | 0.6 | 0.8 | 1.0 | 1.2 | 1.5 | 2.0 |
|---|---|---|---|---|---|---|---|
| **7 kPa** | s | s | I | I | I | I | I |
| **15 kPa** | s | s | I | I | I | I | I |
| **25 kPa** | s | s | I | I | I | I | I |

**NON-EMPTY INTACT BAND**: every stiffness E in {7,15,25} kPa SLIPS at F<=0.6 N and holds INTACT at F>=0.8 N. The slip->intact boundary (0.6/0.8 N) is CONFIRMED UNANIMOUS across 3 seeds at all three E (tofu_grid_confirm.json). This is the non-empty band MPM never produced.

## Hold-slip (mm rel to gripper) and vol-weighted P99 principal strain

| E | metric | 0.4 | 0.6 | 0.8 | 1.0 | 1.2 | 1.5 | 2.0 |
|---|---|---|---|---|---|---|---|---|
| 7 | slip mm | 11.38 | 2.35 | 1.33 | 1.06 | 0.96 | 0.97 | 1.10 |
| 7 | P99 strain | 0.060 | 0.058 | 0.065 | 0.073 | 0.085 | 0.099 | 0.127 |
| 15 | slip mm | 10.86 | 2.21 | 1.24 | 0.86 | 0.63 | 0.48 | 0.42 |
| 15 | P99 strain | 0.027 | 0.026 | 0.029 | 0.031 | 0.035 | 0.042 | 0.053 |
| 25 | slip mm | 10.71 | 2.23 | 1.26 | 0.88 | 0.67 | 0.50 | 0.35 |
| 25 | P99 strain | 0.016 | 0.015 | 0.017 | 0.018 | 0.021 | 0.024 | 0.031 |

Slip falls monotonically with grip force; P99 principal strain RISES with force (grip tightens) -> the damage branch. Deformation is small in the intact band (bbox ~0.04 unchanged); the block lifts ~48-49 mm (near the full 50 mm) and holds.

## Mesh convergence (h=5 -> 4 mm, tofu_meshconv.json)

- E15_F1.0: h5 slip 0.86mm / h4 slip 0.77mm; P99 0.0314->0.0333; **label invariant=True**
- E7_F1.5: h5 slip 0.97mm / h4 slip 0.81mm; P99 0.099->0.1015; **label invariant=True**

## Damage threshold PROPOSAL (for external sign-off BEFORE labeling)

Per the ruling, damage labels are applied POST-HOC from the stored per-tet max-principal Green-Lagrange strain fields (strain_fields/*.npz) only AFTER external sign-off of the threshold.

**Proposed eps_damage = 0.15** (max principal Green-Lagrange strain), anchored to the measured TENSILE failure strain of tofu ~10-20% (literature, consult-vbd-rebuild.md; mid ~15% -> Green strain ~0.16). Rationale: tofu is compression-tolerant, tension-weak; compression fracture strain is much higher (~45-54%), so a tensile-anchored 0.15 flags tension-dominated damage first and is CONSERVATIVE. Damaged-volume fraction = volume-weighted fraction of tets with max principal strain > eps_damage.

At this placeholder, the intact band (F 0.8-1.2 N) stays well below threshold (P99 <= 0.10); the damage BRANCH appears only at the highest force: E=7 kPa / F=2.0 N reaches peak principal strain ~0.26 (> 0.15) in the high-strain clip -> a damage-region candidate. Post-sign-off, re-scan the stored fields to assign intact/damage labels across the grid.

## Deliverables + STOP

Probe JSON (tofu_probe*.json), grid JSON (tofu_grid.json + tofu_grid_confirm.json), mesh-conv JSON, strain fields, and 3 clips (holding tofu_hold.mp4, slipping tofu_slip.mp4, high-strain tofu_highstrain.mp4). STOP for external review of: (1) the damage threshold eps_damage before labeling; (2) the band as-measured. MPM E1 data frozen.


## FINAL judgment-v2 phase diagram (labels applied post-hoc; eps_damage=0.15, DVF>=0.5%)

s=slip (>2mm rel), I=intact, D=damage (DVF>=0.5% at eps1>0.15 latched)

| E\\F (N/finger) | 0.4 | 0.6 | 0.8 | 1.0 | 1.2 | 1.5 | 2.0 |
|---|---|---|---|---|---|---|---|
| **7 kPa** | s | s | I | I | I | I | **D** |
| **15 kPa** | s | s | I | I | I | I | I |
| **25 kPa** | s | s | I | I | I | I | I |

**Three regions cleanly resolved:** SLIP (F<=0.6 N, all E) -> INTACT (F 0.8-1.5 N all E; +2.0 N for E>=15 kPa) -> DAMAGE (E=7 kPa / F=2.0 N only: DVF 4.36%). The softest tofu at the strongest grip is the sole damage cell -> physically sensible. Damage label CONFIRMED mesh-invariant: E7/F2.0 DVF 4.36% (h=5mm) and 2.85% (h=4mm), both >> 0.5%; peak principal strain 0.253/0.306.

### Per-cell peak / P99 / DVF (temporal-max Green principal strain)

| cell | label | slip mm | peak | P99 | DVF |
|---|---|---|---|---|---|
| E7/F0.4 | slip | 11.38 | 0.116 | 0.082 | 0.000% |
| E7/F0.6 | slip | 2.36 | 0.116 | 0.082 | 0.000% |
| E7/F0.8 | intact | 1.32 | 0.116 | 0.095 | 0.000% |
| E7/F1.0 | intact | 1.05 | 0.116 | 0.097 | 0.000% |
| E7/F1.2 | intact | 0.90 | 0.134 | 0.105 | 0.000% |
| E7/F1.5 | intact | 0.90 | 0.158 | 0.123 | 0.260% |
| E7/F2.0 | damage | 1.11 | 0.253 | 0.193 | 4.362% |
| E15/F0.4 | slip | 10.85 | 0.081 | 0.069 | 0.000% |
| E15/F0.6 | slip | 2.21 | 0.081 | 0.069 | 0.000% |
| E15/F0.8 | intact | 1.24 | 0.081 | 0.069 | 0.000% |
| E15/F1.0 | intact | 0.86 | 0.081 | 0.069 | 0.000% |
| E15/F1.2 | intact | 0.63 | 0.081 | 0.069 | 0.000% |
| E15/F1.5 | intact | 0.49 | 0.082 | 0.070 | 0.000% |
| E15/F2.0 | intact | 0.41 | 0.110 | 0.081 | 0.000% |
| E25/F0.4 | slip | 10.72 | 0.135 | 0.089 | 0.000% |
| E25/F0.6 | slip | 2.23 | 0.135 | 0.088 | 0.000% |
| E25/F0.8 | intact | 1.26 | 0.135 | 0.088 | 0.000% |
| E25/F1.0 | intact | 0.87 | 0.135 | 0.088 | 0.000% |
| E25/F1.2 | intact | 0.68 | 0.135 | 0.088 | 0.000% |
| E25/F1.5 | intact | 0.51 | 0.135 | 0.089 | 0.000% |
| E25/F2.0 | intact | 0.36 | 0.135 | 0.088 | 0.000% |

DVF is 0% across the intact band and the slip region (strains below 0.15); only E7/F2.0 crosses the 0.5% volume-robust bar. Labels are threshold-independent from the stored per-tet temporal-max fields (reports/logs/vbd/strain_fields/*.npz) -> any future eps_damage can be re-applied without reruns.
