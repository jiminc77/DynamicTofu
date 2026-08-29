# W3 transport clips

These scenes re-run selected cells from the final W1 bands with the frozen transport rig; they introduce no new physics. The intact and slip scenes satisfy the strict same-grip comparison: identical E15 material, gripper geometry, grip force (1.2 N), seed, and protocol, with only commanded acceleration changed. Across all three scenes the gripper setup/protocol is unchanged; the damage branch uses the pre-registered higher force and E7 material.

| scene | material | commanded a (m/s²) | F (N) | final-band label | realized a (m/s²) | projection |
|---|---:|---:|---:|---|---:|---|
| intact | E15 | 1 | 1.2 | intact | 0.681 | `reports/vbd/clips/w3_intact.mp4` |
| slip | E15 | 30 | 1.2 | slip | 19.85 | `reports/vbd/clips/w3_slip.mp4` |
| damage | E7 | 5 | 2 | damage | 3.183 | `reports/vbd/clips/w3_damage.mp4` |

Each projection is the standard side `(y,z)` plus front `(x,z)` view. Key frames are under `reports/vbd/clips/w3_<scene>_keys/` at grip, lift, hold, accel-out peak, dwell, return, and settle boundaries. If ffmpeg/libx264 is unavailable, the PNG sequence remains authoritative and a key-frame contact sheet is emitted.
