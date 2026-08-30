# W2 tactile force-channel: R3 collector re-promotion + v8 force-inset plan

## Status: force channel READY-FOR-REPROMOTION (convergence-based acceptance)

The R3 pad-wrench collector (`src/pad_wrench.py`, wrapping the Newton
`_harvest_vbd_body_particle_contact_forces_on_proxy_bodies_kernel`) is VALIDATED
under the respecced (convergence) acceptance from GPT-Pro consult #3 follow-up.

The original momentum-balance test required equality with mg (0.628 N) at the
frozen 10 VBD iterations and failed (1.05 N). The consult diagnosed this as
finite-iteration staggered-readback amplified by the steep low-speed friction
slope, NOT a collector bug, and respecced acceptance to: the mass-weighted
momentum residual R_z = F_collector_z + F_gravity_z - dP_z/dt must CONVERGE
toward 0 as VBD iterations increase.

Receipt `reports/logs/vbd/r3_momentum_ladder.json` (iterations ladder 10/20/40,
frozen Vbd2Rig suspended hold, all contact/material frozen):

| iterations | vertical support (N) | |R_z| (N) |
|---|---|---|
| 10 | 1.054 | 0.426 |
| 20 | 0.710 | 0.082 |
| 40 | 0.665 | 0.037 |

- Support walks toward mg=0.628 N; |R_z| strictly decreasing (ratio 0.088);
  atomic sum == float64-stable sum (< 1e-3 N). VERDICT: READY-FOR-REPROMOTION.

## Consequence for the demo videos

At the FROZEN iterations=10 the tangential readback carries a ~0.4 N bias, so
force-magnitude taxel insets MUST be sourced from iterations=40 reruns, not the
frozen-10 production trajectories.

## Plan (judge ruling 12)

1. v7 ships NOW with the geometry-proxy penetration-DEPTH insets (current demo).
2. v8 (after v7): rerun the 3 demo cells at iterations=40 with a
   label-reproduction assertion (frozen config otherwise), capture per-contact
   forces via the validated collector, and render taxel FORCE insets:
   per-taxel normal force as fill, per-taxel shear as small arrows, one net
   shear arrow per pad, provenance label "validated collector at 40 iterations".
3. The 2026-09-01 paper stays GEOMETRY-ONLY (W2 tactile). The W2 force-channel
   promotion itself is a POST-SUBMISSION item; this receipt + plan record that
   the collector is ready when the paper revs.
