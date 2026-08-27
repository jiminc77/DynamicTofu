# RESULTS — one row per batch

| # | Date (KST) | Batch | Trials | Done / Unresolved | Labels summary | Artifacts | Notes |
|---|---|---|---|---|---|---|---|
| 1 | 2026-08-27 | E1 Stage A (sigma=3333, 5 accel x 5 grip x 3 seeds) | 75 | 75 / 0 | drop: 45, damage: 30, intact: 0 | ralph/results/trials/, e1_band_3333.json, e1_shape_checkpoint.json | shape checkpoint: scale_failure -> STOPPED for external review |
| 2 | 2026-08-27 | E1 Stage B (sigma=2000 then 6000, frozen grid) | 150 | 150 / 0 | 2000: drop 21 / damage 51 / boundary 3; 6000: drop 75 | e1_band_2000.json, e1_band_6000.json, trials/ | intact bands empty everywhere; damage-onset monotone in sigma_Y; cross-material summary reports/e1_cross_material_summary.md |
| 3 | 2026-08-27 | Stage C boundary densification (sigma=2000, F=0.8, a in {2.5,5,10,15}, seeds 3-4) | 8 | 8 / 0 | all cells 3 damage / 2 drop (boundary, not >=4/5 certified) | extra_replications/, e1_band_2000.json | drop<->damage transition sharpened; no intact; extra seeds outside 360 universe |
| 4 | 2026-08-27 | Gate B tofu factorial (diagnostics, sigma6000) | 6 (+2 it8, +2 clips) | 6 completed; 2 interpretable (B1/B4 effort), 4 lock INVALID (Fn-collapse) | all drop; effort maintains force but no lift; lock Fn-collapse INVALID; it8 blowup (H5) | gateB.json, gateB-it8.json, gateB_*_run.mp4 | contact stack functional; empty band = material shear/extrusion, not contact bug |
