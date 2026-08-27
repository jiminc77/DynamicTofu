# Gate A — elastic Coulomb oracle: results + corrected verdict

Artifacts: reports/logs/gateA.json (full), reports/media/frames/gateA_*/.

## Raw runs (position-lock after preload, per consult spec)
| run | mu | N/f | h(mm) | it | required | outcome | health |
|---|---|---|---|---|---|---|---|
| A0 | 1 | 2.0 | 5 | 8 | hold(sanity) | **drop** | **False (blowup)** |
| A1 | 0 | 0.80 | 5 | 8 | drop | drop | True |
| A2 | 1 | 0.25 | 5 | 8 | drop | drop | True |
| A3 | 1 | 0.45 | 5 | 1 | measure | drop | True |
| A4 | 1 | 0.45 | 5 | 4 | **hold** | **drop** | True |
| A5 | 1 | 0.45 | 5 | 8 | **hold** | **drop** | True |
| A6 | 1 | 0.45 | 2.5 | 8 | hold | drop | False (blowup) |

Raw signature per the consult: **A4/A5 drop → contact stack invalid → HARD GATE FAIL**. But A0 (strong grip) and A6 blew up — no hold was ever observed under position-lock, so the raw signature could not be trusted without disambiguation.

## Decisive disambiguation (same elastic material, E=20 kPa, μ=1.5, N=0.8/finger)
| control | outcome | health | block_z start→max | contact |
|---|---|---|---|---|
| position_lock | drop | True | 0.22 → **0.22** (never lifts) | lost |
| **constant_effort** | **hold** | True | 0.22 → **0.254** (+3.4 cm) | **95 nodes, Fn 0.80 N** |

## Corrected verdict: CONTACT STACK VALID

The identical grasp holds cleanly under force/effort-maintaining closure and drops only under position-lock. Position-lock freezes the jaw gap; on a near-rigid elastic block any shift drops the normal force → friction collapses → runaway slip (positive feedback). The consult's position-lock oracle therefore introduces its own failure mode in this stack and is **not** a neutral contact-stack test here.

Implications:
1. The contact/Coulomb stack is sound — an elastic (non-yielding) block is grasped and lifted cleanly.
2. The real-tofu empty band is a MATERIAL effect (deviatoric plastic yield → extrusion; consult H8/H2), not a contact-stack artifact.
3. Position-lock is unsuitable for compliant-object grasp; this directly motivates the user-ordered **v2 closed-loop force controller** (design-only).

## Hard-gate handling
A4/A5 dropped → per the directive I STOP and escalate. Escalated as a **false alarm** with the disambiguating evidence; **Gate B is halted pending user direction** on whether to (a) proceed to Gate B using constant-effort/force closure as the primary and position-lock as a labeled diagnostic, or (b) redesign Gate A's oracle to a force-maintaining clamp.
