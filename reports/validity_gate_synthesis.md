# Validity-gate synthesis (Gates A / B / B2 / C) — for the storyline decision

Assembled 2026-08-27 from the diagnostics ordered off the GPT-5 Pro consult (reports/consult-gptpro-20260827.md). **E1 v1 frozen data was never touched**; all runs here are diagnostics. Artifacts: reports/logs/gate{A,B,B2,C}.json, reports/gate{A,B}_report.md, clips reports/media/gateB_{holding,dropping}_run.mp4.

## One-paragraph verdict

The E1 "empty band" (no intact grasp-and-carry region for silken-tofu-stiffness material) is, in the current model + stock/sensor fingertip + accessible stable numerics, a **genuine material-mechanics result, not a contact-stack bug**: the contact/Coulomb stack demonstrably lifts and holds a non-yielding elastic block (Gate A), and the σ6000 tofu fails by **ductile shear/extrusion necking** (Gate B, visible in the dropping clip — pads carry a neck upward while the bulk stays grounded). This holds under the valid constant-effort arms and is robust to pad area (sensor ~ stock -> H1 not dominant) and convergent across the accessible stable numerics. **Controller-robustness and yield-surface-shape robustness are UNRESOLVED**: the four position-lock arms and 2 of 3 yield-bracket cells (C-cap-off, C-asym) were numerically invalid (health blowups), so those comparisons have no valid second point. The finding is therefore scoped to clean eta=20 constant-effort runs at accessible stable numerics. **HOWEVER two questions are numerically inaccessible and remain open:** (H2) whether literature-realistic tofu creep viscosity (1e5–1e7 Pa·s) reopens the band — the high-viscosity regime blows up under effort (Gate B2); and continuum convergence — refining iterations or grid blows up (Gate C stability ceiling). So the defensible claim must be **scoped to the current constitutive model, fingertip geometry, and stable numerical regime**, and must flag rheology + high-fidelity convergence as untested levers.

## Gate-by-gate

**Gate A — contact-stack oracle: PASS (contact stack functional).** Raw signature (A4/A5 drop under position-lock) fired, but matched-config isolation at the exact A4 params (only controller changed) showed constant-effort **holds** an elastic block (+3.4 cm, Fn 0.454, 65 nodes) where position-lock never lifts it. The contact/Coulomb stack works; position-lock is an unsuitable controller (freezes gap → grip-force collapse on a shifting body). `gateA.json`, `gateA_report.md`.

**Gate B — tofu factorial (H1/H6): all 6 drop.** Only the 2 constant-effort arms are interpretable (4 position-lock arms Fn-collapsed → INVALID). Under effort, both pads keep ~0.6 N and 80–95 contact nodes yet the block never lifts → holdable tangential load is limited by **material shear-yield, not μ·Fn** (H8/H9 ductile necking). Sensor_format_pad ≈ stock → **H1 (pad pressure) not dominant** at 0.6 N. `gateB.json`, `gateB_report.md`, clips.

**Gate B2 — viscosity axis (H2): INCONCLUSIVE.** η=2e5 under constant-effort **blows up** (health=False) at baseline h5/it4 — the physically realistic high-viscosity regime is numerically unstable in this stack. Whether literature-calibrated rheology reopens the band is **UNTESTED**. `gateB2.json`.

**Gate C — convergence + yield bracket.** Convergence grid it{1,4,8}×h{5,2.5 mm}: the 3 health-clean cells (it1/h5, it1/h2.5, it4/h5) all DROP with finFn within ~3% and no lift → **converged in the accessible stable regime**; but the other 3 cells BLOW UP (refining iterations or grid destabilizes) → **continuum convergence unverified (H5 stability ceiling)**. Yield-surface bracket: 2 of 3 cells (C-cap-off deviatoric-dominated, and C-asym) blew up (health=False, INVALID); only C-base is valid, so there is **no valid comparison -> yield-surface-shape robustness is UNRESOLVED** (the alternative surfaces are numerically unstable in this stack). `gateC.json`.

## Hypothesis disposition (consult H1–H9)

| H | claim | disposition |
|---|---|---|
| H1 | pad pressure gates the band | **Not dominant** — sensor_format_pad (larger area) ≈ stock at 0.6 N |
| H2 | viscosity/rheology mismatch | **UNTESTED** — η=2e5 numerically unstable under effort (Gate B2) |
| H4 | discretization | partly implicated — h=2.5 mm destabilizes at it≥4 (Gate C) |
| H5 | proxy-coupling convergence | **Confirmed as a stability ceiling** — it=8 (and fine-h) blow up |
| H6 | jaw advancement | effort maintains force (good); position-lock collapses (invalid isolator) |
| H8 | ductile VM plasticity instead of fracture | **Leading mechanism** — dropping clip shows ductile necking, no crack model (scoped to clean eta=20 constant-effort runs) |
| yield-shape | does yield-surface shape gate the band? | **UNRESOLVED** — 2/3 bracket cells (C-cap-off, C-asym) numerically unstable; only C-base valid |
| H9 | μ uncalibrated | grip is shear-yield-limited, not μ·Fn-limited — μ is not the lever |

## Defensible paper wording (unchanged from the consult, now evidence-backed)

> "Under the stock Franka fingertip geometry (and a sensor-format flat fingertip) and the current von-Mises/capped-viscoplastic MPM model, no intact transport region was observed; the material fails by ductile shear/extrusion necking. The result is robust to pad area within the numerically stable regime; closure-controller and yield-surface-shape robustness could not be established (the alternative position-lock controller was invalid via grip-force collapse, and the alternative yield surfaces were numerically unstable). Quasi-static grasp validity remains contingent on **rheology calibration** (high-viscosity creep could not be tested — numerically unstable) and on **proxy-coupling / grid convergence** (high-fidelity runs are unstable), which are open."

**Avoid:** "Real silken tofu has no feasible grasp-force band" (unscoped).

## Decisions reserved for you

1. **Storyline** (reserved): adopt the scoped material-limit result as the paper's finding, with the H2/convergence caveats stated? Or pursue the open levers first?
2. **Solver stabilization** (would unblock H2 + continuum convergence): invest in a smaller coupled dt / implicit-viscosity treatment so the high-viscosity and high-fidelity regimes become runnable? This is the single change that could reopen the band per the consult's H2.
3. **v2 closed-loop force controller** (design-only so far; design notes in gateB_report.md): build it? It is the right closure (position-lock rejected by both gates) but will not by itself overcome ductile necking at 0.6 N — its value is tactile-regulated grip within the material's shear limit.
4. **E2/E3** remain downstream of the storyline (E2 has no usable band → censored path; E3 has no intact/slip source trials → fail-closed report), per the frozen protocol.
