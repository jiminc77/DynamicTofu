# G-TRK actuation gate — MISS at first level (E-1 escalation)

Run 01a046eb / G2 phase P3. Rig: extended world-x transport DOF (src/vbd_rig2.py sha b8c4768, G0'-prime PASS 9/9). Smoke cell E15 kPa / a_cmd = 5 m/s^2 / F = 1.2 N, seed 0, substeps=80.

## Phase machine: PASS (exact)
All eight transport sub-phase boundaries hit within ~1e-13 s of the pre-registered timestamps (accel_out 9.30, cruise_out 9.50, decel_out 9.60, dwell 9.80, accel_back 10.10, cruise_back 10.30, decel_back 10.40, settle 10.60). The trapezoid-reversal profile and 4800 Hz timebase are exactly as pre-registered.

## G-TRK: FAIL
Realized palm acceleration (least-squares slope of palm_vx over the four pre-registered plateau windows), commanded a = 5.0 m/s^2:

| plateau | a_fit (m/s^2) | |a_fit-a_cmd|/a_cmd | n_samples | r2 |
|---|---|---|---|---|
| accel_out  |  3.275 | 34.5% | 6 | 0.9986 |
| decel_out  | -3.077 | 38.5% | 6 | 0.9995 |
| accel_back | -3.291 | 34.2% | 6 | 0.9989 |
| decel_back |  3.066 | 38.7% | 6 | 0.9985 |

max relative error = **38.7% > 5% gate**; samples_valid = True (n>=5). The realized palm acceleration is a consistent ~62-66% of commanded on every plateau (clean fits, r2>=0.998). This is a systematic under-actuation, NOT noise and NOT a logging artifact.

## Root cause (not a bug)
Velocity feed-forward is correctly wired and consumed: w1_transport.py:111-112 passes x_vel=profile.v_cmd(t); vbd_rig2.py:290-292 writes control.joint_target_qd[x_dof]=x_vel; VBD consumes it as axis.target_vel. Without the feed-forward the failure would be near-total (~900 N spurious drag); the ~35% residual shortfall is the actuation-dynamics limit the pre-mortem anticipated: the j_x position-PD (target_ke=1e4, target_kd=2e2, same as the validated j_z lift) is OVERDAMPED for the transported mass (carriage 0.05 + palm 0.05 + 2 fingers ~0.046 + grasped block 0.064 ~= 0.21 kg): damping ratio zeta ~= kd/(2*sqrt(ke*m)) ~= 2.2, settling time ~0.1 s ~= the entire 0.10 s accel plateau, so the palm never reaches steady acceleration-tracking within a plateau.

## Pre-registered contingency (E-1): propose D3-C for sign-off
D3-C (locked-decisions of the approved plan, "requires sign-off before use"): add an acceleration feed-forward EFFORT on j_x, control.joint_f[x_dof] += m_tot * a_cmd(t), on top of the existing position target + velocity feed-forward. This directly injects the accelerating force, bypassing the overdamped PD settling. m_tot convention to confirm: transported rigid gripper mass (carriage+palm+fingers ~0.146 kg) vs gripper+grasped-block (~0.21 kg). Frozen contact/material/solver params and the j_x PD gains are untouched; D3-C only adds a feed-forward effort term.

## STOP
Per E-1, no production screen (P4) or further gate runs proceed until D3-C is signed off and G-TRK re-passes (max plateau error <=5% at every a in {1,2.5,5,10,20,30}, plus the zero-command noise floor <=0.01 m/s^2). Awaiting external sign-off on D3-C and the m_tot convention.
