# BRIEF_WS — Newton-based characterization sweeps for the IROS26 workshop abstract

Mission: build a lean Newton-based simulation rig and produce the pre-registered
characterization data (E1 phase diagram, E2 tactile-proxy traces, E3 renders) for a
2-page workshop abstract due 2026-09-01. No learning, no RL, no reward design:
everything is scripted. The paper is written elsewhere; this workspace produces
data files only.

## Hard context

- Machine: this host (RTX PRO 6000 Blackwell 96GB). Workspace: `~/Workspace/DynamicTofu`
  (this directory; currently fresh). `~/Workspace/DynamicTofu_old` is the previous
  Genesis-based project: **read-only reference** (judgment predicate ideas, sweep
  patterns). Never modify it.
- Engine: **Newton**, github.com/newton-physics/newton, **pinned commit `b74df53`**.
  A depth-1 clone already exists at `/tmp/newton-audit` (may be re-cloned into the
  workspace properly). Relevant, verified in source: `SolverImplicitMPM` supports
  elastoplastic materials with per-particle `mpm:young_modulus`, `mpm:poisson_ratio`,
  `mpm:yield_stress` (deviatoric, Pa), and state `mpm:particle_Jp` (plastic
  deformation determinant); examples `softbody/example_softbody_franka.py`,
  `mpm/*`, `sensors/example_sensor_contact.py`.
- Absolute rules: no sudo; never touch processes you did not start; no model-config
  changes anywhere; commit early and often in this workspace's own git repo; every
  claim of success must be backed by a state log or artifact, never a flag or a
  visual impression alone.

## Deadline gates (Asia/Seoul). Fail-closed: on a miss, STOP, write the failure
report, and wait — the fallback switch (to the Genesis stack) is decided externally.

- **G-N1 by 2026-08-28 night** — environment: pinned Newton installed in a fresh venv;
  `example_softbody_franka` and at least one `mpm/` example run headless on the GPU;
  receipt records versions, wall-clock, and any deviation.
- **G-N2 by 2026-08-29 night** — physics smoke: a 4×4×4 cm elastoplastic block
  (E=7 kPa, ν=0.45, density 1000 kg/m³) is grasped by a **full Franka Panda arm with
  its parallel gripper under force control** (template:
  `examples/softbody/example_softbody_franka.py`; user decision 2026-08-27: full
  Franka from the start, for downstream reusability) and lifted 5 cm without solver
  blow-up; `particle_Jp` readout works (a deliberate crush shows damage_fraction
  rising; a gentle hold does not); σ_Y ∈ {2000, 3333, 6000} Pa changes the crushing
  force monotonically.
- **G-N3 by 2026-08-30 noon** — one full E1 cell end-to-end: trapezoidal transport
  with reversal, judgment v1 labels emitted, per-trial JSON written, wall-time
  per trial measured and reported (this number sizes the sweep schedule).

## Judgment v1 (pre-registered; no post-hoc widening of any window or threshold)

Phases: settle 0.5 s → close to commanded force, hold 0.5 s → lift 5 cm in 0.3 s →
hold 0.2 s → transport profile → settle 0.5 s. Labels evaluated from lift-complete
to settle-end:
- **damage**: fraction of particles with |Jp − 1| > 0.05 exceeds 10% at any time
  (latched). Always also record the peak fraction.
- **drop**: after grasp established, bilateral finger contact lost > 0.2 s, or
  object–gripper relative displacement > 2 cm.
- **slip**: net object displacement in the gripper frame > 5 mm, or peak > 8 mm,
  without drop.
- **intact**: none of the above.
All label reductions run host-side in a fixed order (GPU MPM is not assumed
bit-deterministic; seeds are the replication unit).
If a threshold proves physically ill-posed during G-N2/G-N3 (e.g., Jp scale differs
from expectation), propose a revised value with evidence in the gate report and get
external sign-off BEFORE any sweep runs; never tune thresholds after E1 has started.

## E1 — phase diagram (the headline data)

Grid: σ_Y ∈ {2000, 3333, 6000} Pa × a_peak ∈ {1, 2.5, 5, 10, 15} m/s² ×
F_g ∈ {0.3, 0.5, 0.8, 1.2, 1.8, 2.5, 3.5, 5.0} N × seeds {0,1,2}
(particle sampling + ±1 mm pose jitter). Motion: the Franka **end effector** tracks a
jerk-limited trapezoid, 0.3 m out, full stop, 0.3 m back (one reversal), same path
all cells. Route the path laterally (±y) through the workspace center — the old
project hit Franka reach limits at +x extremes; verify reach margin at G-N3. The arm
tracks, it does not teleport: record commanded AND realized EE acceleration; the
phase-diagram axis is commanded a_peak, and every JSON carries the realized peak
(`a_peak_realized_ms2`) so tracking error is visible, never hidden.
Order: seed 0 full grid first → localize band boundaries → remaining seeds
prioritizing boundary-adjacent cells → fill as time allows. **Every skipped cell is
listed in the aggregate JSON coverage map; nothing is silently absent.**
Per-trial JSON (schema `e1.v1`) and per-material aggregate (schema `e1_band.v1`,
with F_min/F_max per accel by the ≥2/3-seed rule, band widths, a_star or null,
coverage map) go to `ralph/results/`; one row per batch in `ralph/RESULTS.md`.
The `config` block of every JSON records all protocol constants actually used
(dt, substeps, particle count, contact params, windows) — the paper's protocol
sentences are audited against these blocks.

## E2 — tactile-proxy traces

At per-material fixed mid-band force (declared in the JSON, chosen from E1 seed-0
at a_peak=1): a_peak ∈ {1, 5, 15}, 3 seeds. Record at ≥200 Hz effective: per-finger
normal resultant, tangential (shear) resultant, contact centroid in the finger frame,
left–right normal asymmetry; mark phase timestamps (reversal). Output: npz per trial
+ `e2_summary.json` (peak-shear ratio a15/a1, max centroid excursion mm, per-seed).
Preferred source: solver collider contact data; if per-collider impulses are not
exposed, use near-finger particle stress sampling and name it a proxy in the JSON
(`"signal_source": "particle_proxy"`).

## E3 — renders

(a) One high-quality still of the actual rig (Franka + gripper + block) for the
paper's setup panel. (b) 2–3 short mp4 clips from real E1 trials (intact vs slip vs
damage at one accel) for the project record. Renders never substitute for labels.

## Deliverable layout (this workspace)

```
BRIEF_WS.md                this file
src/ (or dynamic_tofu_ws/) lean rig: scene builder, Franka EE trajectory tracking
                           (IK-based) + gripper force control, profiles,
                           judgment.py, io_schemas.py
scripts/                   run_e1.py, run_e2.py, render_e3.py, aggregate_bands.py
ralph/results/*.json|npz   the only channel for numbers
ralph/RESULTS.md           one row per batch
ralph/DECISIONS.md         gate calls, deviations, threshold proposals
reports/gN*.md             gate receipts (commands, versions, timings, evidence)
```

## Non-goals (do not build)

Learning of any kind (scripted control only); tactile sensor emulation beyond
contact force fields; parameter identification against real tofu; any paper text.
Keep the rig minimal — the previous project died of over-implementation, and this
one is measured by data files delivered per day.
