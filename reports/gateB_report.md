# Gate B — H1/H2/H6 factorial on sigma_Y=6000 tofu + cross-gate synthesis

Artifacts: reports/logs/gateB.json, reports/logs/gateB-it8.json, reports/media/frames/gateB_B*/, clips reports/media/gateB_holding_run.mp4 + gateB_dropping_run.mp4 (+ key frames). All arms: E=7 kPa, sigma_Y=6000, mu=1, target 0.60 N/finger, it=4, 1.0 s lift + 10 s hold, no transport, h=5 mm.

## Factorial (constant-effort PRIMARY; position-lock LABELED diagnostic)

| arm | pad | control | eta | outcome | preFn | finFn | z_max | nodes | valid |
|---|---|---|---|---|---|---|---|---|---|
| B1 | stock  | effort | 20  | drop | 0.641 | 0.595 | 0.2229 | 95 | **yes** |
| B2 | stock  | lock   | 20  | drop | 0.640 | 0.000 | 0.2201 | 0  | no (Fn→0) |
| B3 | stock  | lock   | 2e5 | drop | 0.637 | 0.000 | 0.2201 | 0  | no (Fn→0) |
| B4 | sensor | effort | 20  | drop | 0.638 | 0.611 | 0.2226 | 80 | **yes** |
| B5 | sensor | lock   | 20  | drop | 0.638 | 0.000 | 0.2202 | 0  | no (Fn→0) |
| B6 | sensor | lock   | 2e5 | drop | 0.639 | 0.000 | 0.2202 | 0  | no (Fn→0) |

**All 6 drop.** The 4 position-lock arms collapse Fn to 0 (contact lost, block never lifts) — the same failure mode as the Gate A oracle — so per safeguard 1 they are **INVALID isolators** (contra the a-priori expectation that soft-tofu indentation would keep lock valid; empirically it does not). Only the constant-effort arms (B1, B4) are valid.

## What the valid (effort) arms show

Under force-maintaining closure both pads **keep ~0.6 N and 80–95 contact nodes to the end**, yet the block **never lifts** (z_max ~0.2226 vs 0.22 rest). With mu=1 and Fn=0.6 the Coulomb capacity (1.2 N bilateral) exceeds the 0.628 N weight, so a rigid grip *should* lift it. It does not → the holdable tangential load is limited by the **tofu shear-yield at the pad interface, not by mu*Fn** (H8/H9): the material shears/extrudes under lift load. The **sensor_format_pad does not rescue it** (B4 finFn 0.611 ≈ B1 0.595, same z_max) → **pad pressure (H1) is not the dominant lever** at this force. B6 does not hold → quasi-static grasp is **not demonstrated possible** in this stack at 0.6 N.

## Iteration robustness (safeguard 2): NOT robust — H5 stability sensitivity

Duplicating the valid effort arms at it=8 (reports/logs/gateB-it8.json): **both BLOW UP** (health=False, block ejected to z~0.37–0.41) where it=4 ran clean. The coupling has an iteration-dependent **stability** sensitivity, so the "no hold / material-limited" conclusion holds **at it=4** but is **not confirmed iteration-robust** — proxy-iteration × h convergence must be resolved in Gate C before firm quasi-static claims.

## Signatures
- Position-lock collapses on tofu (all lock arms Fn→0) → lock is not a valid isolator here; effort/force closure required.
- Sensor pad ≈ stock under effort → H1 (pad pressure) not dominant at 0.6 N.
- eta 20 vs 2e5 uncomparable (both under lock, invalid) → H2 not isolated this pass.
- No B6 hold → quasi-static grasp not shown possible.

## Cross-gate synthesis (Gate A + Gate B)

1. **Contact stack is FUNCTIONAL** (Gate A matched-config isolation): an elastic, non-yielding block lifts and holds under effort closure at the exact A4 params (+3.4 cm, Fn 0.454, 65 nodes). The raw "contact stack invalid" hard-gate signature was a **position-lock artifact**.
2. **The empty band is a MATERIAL effect, not a contact bug**: on real sigma_Y=6000 tofu, even with maintained normal force and sustained contact, the block is not carried — the tofu shears/extrudes at the interface (**H8/H9, scoped to the clean it=4 constant-effort arms B1/B4**, the only interpretable runs). This is robust to **pad area** within effort control (sensor ≈ stock → H1 not dominant). It is **NOT** established as controller-robust: the four position-lock arms Fn-collapsed and are INVALID isolators, so lock provides no independent confirmation (and H2 viscosity is unisolated, only appearing under lock).
3. **Position-lock is unsuitable** for compliant/soft-object grasp in this stack (Gate A stiff + Gate B soft) → strengthens the case for the v2 closed-loop **force** controller.
4. **Open caveat (H5)**: the coupling is iteration-stability-sensitive (it=8 blowups). Firm quasi-static conclusions require the Gate C convergence bracket (proxy-iterations {1,4,8} × h {5,2.5} + yield-surface bracket) the consult specified.

## v2 controller design notes (design-only, per user ruling)
- Position-lock rejected (both gates); PI force control on the 100 Hz per-finger Fn with a closure-rate limit + force cap is the right closure. It will maintain grip force robustly, but Gate B shows it will **not overcome material extrusion** at 0.6 N on sigma6000 — the research value is tactile-regulated grip within the material's shear limit, and identifying whether any (force, rate, pad) regime yields intact transport (currently none found).
- Operate in the it=4-validated stability regime (or resolve H5 convergence first); the controller must not command closure rates that trigger the it=8-class instability.

## Clips
- Holding run: reports/media/gateB_holding_run.mp4 (Gate A elastic effort-it4 — the validated hold; block rises and is held).
- Dropping run: reports/media/gateB_dropping_run.mp4 (Gate B sigma6000 tofu, stock effort — the empty-band drop; pads rise, block shears and stays).

## Gate B2 — viscosity axis under constant-effort (H2 isolation)

eta=2e5 only ran under position-lock in Gate B (invalid), so H2 was unisolated. Re-run under the valid constant-effort closure (reports/logs/gateB2.json; frames gateB2_B{3,6}prime):

| arm | pad | control | eta | it | outcome | health | preFn | finFn | z_max | valid |
|---|---|---|---|---|---|---|---|---|---|---|
| B3prime | stock  | effort | 2e5 | 4 | drop | **False (blowup)** | 0.637 | 0.000 | 0.399 | no |
| B6prime | sensor | effort | 2e5 | 4 | drop | **False (blowup)** | 0.639 | 0.000 | 0.427 | no |
| (B1 baseline) | stock  | effort | 20 | 4 | drop | True | 0.641 | 0.595 | 0.223 | yes |
| (B4 baseline) | sensor | effort | 20 | 4 | drop | True | 0.638 | 0.611 | 0.223 | yes |

**H2 verdict: INCONCLUSIVE (not refuted).** Both eta=2e5 effort arms **blow up** (health=False, block ejected to z~0.40-0.43) at baseline h=5 mm/it=4 — the same numerical-instability class as the it=8 blowups. Per safeguard a blowup is INVALID, so H2 cannot be isolated at baseline: the physically realistic high-viscosity regime (literature tofu creep 1e5-1e7 Pa·s) is exactly where the current coupled stack becomes unstable under effort. Whether literature-calibrated rheology REOPENS the band is therefore **UNTESTED**; it requires first stabilizing the coupling (smaller coupled dt / implicit viscosity / the Gate C regime). The eta=20 effort material-limit result stands independently. **Action:** re-test eta=2e5 in any stable (it,h) regime that Gate C identifies before drawing an H2 conclusion.
