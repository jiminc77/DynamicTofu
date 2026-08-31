# Judge directive — W3 demo v10 (render-only revision)

Date: 2026-08-31 (external judge, after frame-by-frame inspection of v9)
Scope: RENDER ONLY. No sim re-runs, no physics/label/band changes. Reuse the saved
force40 frame data and rig geometry (reports/vbd/clips/panda/w3_*_force40/,
panda_rig_geometry.npz). This directive supersedes nothing about the frozen rig.

## Inspection verdict on v9 (what the judge actually saw in the frames)

- V9-D1 (CRITICAL, slip): the gripper NEVER appears in w3_slip_v9.mp4. From sim
  t=0.98s to the end the wide fixed camera shows an empty track; grasp, accel and
  the release moment all happen off-frame; the tofu flies in from off-screen and
  lands. The wide framing excludes the track start / gripper position entirely.
- V9-D2 (MAJOR, all three): the v9 commit message and your completion summary say
  "white body / black fingers", but every rendered frame shows the ENTIRE hand +
  fingers in uniform dark charcoal. The claimed material change did not reach the
  encoded pixels. (Claims must be verified against final-frame state, not code.)
- V9-D3 (MAJOR, intact): the camera is nearly aligned with the track axis, so the
  0.3 m out-and-back reads as a few tens of pixels of drift — motion is almost
  invisible. The damage clip's semi-side framing shows motion well; intact does not.
- V9-D4 (MODERATE, intact+damage): sim 1.8s→~8.6s is a motionless hold played 1:1,
  so >half of each clip is a static frame. Boring for a professor demo.
- OK (no action): taxel insets (continuous bottom-row band, mirrored L/R maxima
  0.075/0.074 N intact, 0.120/0.121 N damage, shear arrows tilt under accel),
  damage strain tint + deformation, damage framing, intro/outro cards, slip landing.

## The user has now supplied the ground-truth look (real FRANKA EMIKA hand photo)

White rounded shell body; light-gray recessed band around the shell; silver metal
finger brackets; BLACK fingertip covers; black connector on top. Match this.

## v10 requirements

1) Hand colors (V9-D2):
   - shell/body: off-white ~#F2F2F0, soft sheen; recessed band (if separable): ~#C8C8C8
   - finger brackets/carriers: silver metal ~#B0B0B5
   - fingertip covers: near-black ~#2A2A2A
   - pads: UNCHANGED (matte gray, blue-L / orange-R trim)
   - Find out which submeshes of the fr3 asset are shell vs fingers; if the asset is
     a single fused mesh and per-part coloring is impossible, color it all white-shell
     and REPORT that limitation instead of silently shipping charcoal again.
   - Acceptance is pixel-level: in a rendered still, sample the hand-body region and
     assert mean luminance > 200/255. A material assignment that doesn't survive to
     the encoded frame is a failure.

2) slip camera (V9-D1), two-shot structure:
   - Shot A (grasp → release +0.2s): world-fixed semi-side framing like damage v9,
     gripper large and fully in frame. Keep SLOW MOTION x4 over accel→release.
   - Shot B (release +0.2s → landing +1s): wide full-track view; hard cut is fine.
     Extend/shift the wide framing so the gripper start position is also visible.
   - Acceptance: gripper fully in frame for all of Shot A; the tofu-leaves-pads
     moment on screen; landing on screen.

3) intact camera (V9-D3): use the same semi-side world-fixed framing as damage so
   the 0.3 m out-and-back is clearly visible.

4) Hold compression (V9-D4), all three clips: play sim 2.2s → (motion_start − 0.5s)
   at 8x with a visible "FAST-FORWARD x8" chip (same style as the slow-mo chip);
   grasp and motion→ending at existing speed rules. Target clip length 8–10s.

5) Pre-encode still-check gate (existing procedure): before encoding, render 3
   stills per clip {grasp, mid-motion, ending} (9 total) and self-check: (a) gripper
   fully in frame, (b) white shell confirmed by the luminance assertion in (1),
   (c) insets present and mirrored. All pass → proceed automatically and encode.
   Any fail → STOP and report; do not ship.

6) Deliverables: three mp4 (w3_{intact,slip,damage}_v10.mp4) + the 9 stills + the
   luminance-check numbers into reports/vbd/clips/panda/; update the manifest noting
   v10 is render-only (no sim re-run, labels untouched — no new label_reproduced
   assertion needed, but SAY SO in the manifest). Commit with receipts.
