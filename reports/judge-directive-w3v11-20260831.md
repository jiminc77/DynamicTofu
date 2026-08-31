# Judge directive — W3 demo v11 (render-only, supersedes v10 output)

Date: 2026-08-31 (external judge, after v10 frame inspection + user's callout)
Scope: RENDER ONLY again. Physics/labels/bands frozen; contact stays pads-only.

## v10 verdict first (credit where due)

Fixed and verified by the judge on pixels: white shell (luminance ~248-250 ✓),
slip two-shot (grasp/accel/release all on screen, hard cut to wide, landing on
screen ✓), FAST-FORWARD x8 chip ✓, durations 8.4/9.57/8.4 s ✓, insets intact ✓.

## Two defects remain — the user caught the first one

- V10-D1 (CRITICAL, all clips): **the white hand shell is rotated 90 degrees.**
  On the real FRANKA EMIKA hand the shell is a WIDE, squat body sitting over the
  fingers: its long axis is HORIZONTAL (parallel to the jaw/finger-opening axis),
  the gray recessed logo band runs horizontally along that long axis, and the
  silver circular flange faces UP (it is the arm-mount face; the connector sits
  on top). In v10 the shell stands VERTICALLY like a column: long axis vertical,
  band vertical, circular flange facing sideways. The fingers/pads themselves are
  correct (gate-verified physics, untouched) — this is the visual shell
  attachment transform only. Rotate the shell's local frame 90° so that:
  flange up, band horizontal, shell long axis parallel to the jaw axis.
  NOTE: this was present in v9 too (invisible because the shell was charcoal);
  do not assume any prior pose was validated.

- V10-D2 (MAJOR, intact): the intact camera still hides the motion. Judge
  measurement on the encoded frames (blue-pad centroid x, all 252 frames):
  damage swings 460→713→460 px (254 px amplitude) for the 0.3 m out-and-back,
  intact only 566→618→566 px (52 px). Same translation, 5x less on screen —
  the intact camera's yaw/position is NOT the same as damage's. Copy the damage
  camera parameters for intact verbatim (same yaw/pitch/distance/height,
  position offset only to center the intact start pose).

## Acceptance (self-check before encoding; STOP + report on any fail)

1. Shell orientation stills: from the grasp still of each clip assert visually:
   (a) shell bounding extent along the jaw axis > its vertical extent,
   (b) circular flange on TOP of the shell facing up, (c) recessed band horizontal.
   Additionally render ONE side-view still matching the user's reference photo
   angle and save it as reference_pose_check.png for side-by-side comparison.
2. Motion visibility: re-measure the blue-pad screen-x amplitude on the encoded
   intact clip; require >= 150 px. Report the number.
3. Keep everything already passing (white-shell luminance >200, slip shot
   structure, x8 chip, 9 stills gate, insets).
4. Deliverables: w3_{intact,slip,damage}_v11.mp4 + stills + the two check numbers
   into reports/vbd/clips/panda/, manifest updated (render-only, no sim re-run),
   then commit everything (v10+v11 receipts).
