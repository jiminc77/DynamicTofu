# W3 transport clips

These scenes re-run selected cells from the final W1 bands with the frozen transport rig; they introduce no new physics. The intact and slip scenes satisfy the strict same-grip comparison: identical E15 material, gripper geometry, grip force (1.2 N), seed, and protocol, with only commanded acceleration changed. Across all three scenes the gripper setup/protocol is unchanged; the damage branch uses the pre-registered higher force and E7 material.

| scene | material | commanded a (m/s²) | F (N) | source label | rerun label | label reproduced | realized a (m/s²) | projection |
|---|---:|---:|---:|---|---|---|---:|---|
| intact | E15 | 1 | 1.2 | intact | intact | true | 0.681 | `reports/vbd/clips/w3_intact.mp4` |
| slip | E15 | 30 | 1.2 | slip | slip | true | 19.85 | `reports/vbd/clips/w3_slip.mp4` |
| damage | E7 | 5 | 2 | damage | damage | true | 3.183 | `reports/vbd/clips/w3_damage.mp4` |

Each projection is the standard side `(y,z)` plus front `(x,z)` view. Key frames are under `reports/vbd/clips/w3_<scene>_keys/` at grip, lift, hold, accel-out peak, dwell, return, and settle boundaries. If ffmpeg/libx264 is unavailable, the PNG sequence remains authoritative and a key-frame contact sheet is emitted.

## Solid-surface render (presentation)

| scene | presentation render |
|---|---|
| intact | `reports/vbd/clips/w3_intact_solid.mp4` |
| slip | `reports/vbd/clips/w3_slip_solid.mp4` |
| damage | `reports/vbd/clips/w3_damage_solid.mp4` |

Surface = boundary triangles of the frozen tet mesh, shaded by depth, physics unchanged (reused label-reproduced snapshots), same seed/config; `rerun_label==source` per `w3_manifest.json`. The two solid pads use the simulated box geometry `(hx, hy, hz) = (0.022, 0.006, 0.022) m`, and the ground is the simulated plane at `z=0`. Palm = presentation proxy, non-physical; only the two pads and the tofu are simulated. Each frame presents side `(x,z)`, looking along `-y`, beside front `(y,z)`, looking along `+x`; seven nearest-snapshot key frames are under `reports/vbd/clips/w3_<scene>_solid_keys/`.

## Professor-facing W3 render

`scripts/vbd/w3_pro_render.py` produces 1280×720, 30 fps
`reports/vbd/clips/w3_<scene>_pro.mp4` and seven phase images under
`w3_<scene>_pro_keys/`. It first attempts offscreen pyrender with
`PYOPENGL_PLATFORM=egl`, and falls back per scene to matplotlib 3D with
per-frame painter sorting if EGL initialization fails. The selected path is
printed by the CLI. The single perspective camera is world-fixed; its framing
uses the complete recorded palm x range (outbound and reversal) plus 10 cm at
each end. The light ground has 5 cm x-grid ticks and a contact-shadow hint.

Only the two pad boxes are physical gripper colliders. The palm housing and
finger brackets are explicitly **render-only presentation dressing**, rigidly
attached to recorded palm/pad poses; no robot arm is shown. Damage is colored
by per-vertex averaged maximum-principal Green strain (tet deformation
gradient from the first dense frame as the rest reference), opaque tofu
amber-to-red over [0.10, 0.22], with a nonlinear ramp that retains amber at
mid strain and reserves red for failure; intact and slip are neutral. Slip playback repeats
the real dense frames from 9.20–9.40 s four times and shows a `SLOW MOTION x4`
tag. This changes playback timing only.

Physics is untouched: all images reuse the stored dense trajectories, whose
`rerun_label==source` and `label_reproduced=true` are checked fail-closed
against `w3_manifest.json`. Render environment: `.venv-render`; install with
`.venv-render/bin/pip install -r requirements-render.txt`. Smoke:
`.venv-render/bin/python scripts/vbd/w3_pro_render.py --smoke`. Full render:
`PYOPENGL_PLATFORM=egl .venv-render/bin/python scripts/vbd/w3_pro_render.py --render`.

Every composed frame also has two tactile inset panels side by side inside the
bottom-right corner. These are
`ATTR=GEOMETRY_ONLY` geometric proxies, **not simulated pressure**. For each
recorded pad pose, the inner face is the local plane
`y = sign(tofu_center_local_y) * 0.006 m`. A boundary-surface vertex is selected
exactly when its perpendicular distance
`abs(vertex_local_y - inner_face_y) <= 0.003 m`, and both
`abs(vertex_local_x) <= 0.022 m` and `abs(vertex_local_z) <= 0.022 m`.
The frozen `soft_contact_margin=1e-3 m` from `src/frozen_config.py` was tested
first but yielded no surface vertices because the recorded soft surface sits
about 1.85 mm proud of the collider face. The documented 3 mm visualization
proximity band is the smallest whole-millimetre band that gives a stable,
non-empty intact-hold footprint; it does not alter contact physics.
Selected vertices are projected into pad-local transport-x/gravity-z, with a
centroid cross and selected-vertex count. The overlay is post-composited in 2D,
so its behavior is identical for EGL and matplotlib rendering.

The non-physical palm housing and finger brackets use alpha 0.35, while the
simulated tofu and two physical pads remain opaque. Thus presentation dressing
cannot visually masquerade as, or fully occlude, simulated geometry. Video and
key-frame PNGs use the same final HUD/tactile post-composite path; all seven
keys are overwritten on each encode to prevent stale pre-overlay images.
