# G-N1 Gate Receipt — Environment

- Gate: G-N1 (deadline 2026-08-28 23:59:59 Asia/Seoul; cutoff per user confirmation 2026-08-27)
- Executed: 2026-08-27 02:11–02:25 KST
- Verdict: **PASS** — pinned Newton installed in a fresh venv; `softbody_franka` and two `mpm/` examples plus the coupled-solver template ran headless on the GPU to completion.

## Machine

- Host GPU: NVIDIA RTX PRO 6000 Blackwell Workstation Edition, 97887 MiB, driver 590.48.01 (`nvidia-smi`, log: `reports/logs/gn1-versions.log`)
- Warp-reported CUDA: Toolkit 12.9, Driver 13.1, device `cuda:0` sm_120, mempool enabled

## Environment (fresh venv)

| Component | Version / pin | Evidence |
|---|---|---|
| Newton | `1.6.0.dev0` @ commit `b74df534bee62a17e0e57cc9cdfd1a67d91ca817` (required pin `b74df53`) | `reports/logs/gn1-clone.log` |
| Clone | `git clone /tmp/newton-audit newton`; origin reset to `https://github.com/newton-physics/newton.git` | same |
| venv | `uv sync --extra examples` (uv 0.11.28, lockfile `newton/uv.lock`), venv at `newton/.venv` | `reports/logs/gn1-uv-sync.log` |
| Python | 3.12.13 (uv-managed, per lock resolution) | `reports/logs/gn1-versions.log` |
| warp-lang | 1.17.0.dev20260807 (nvidia index, per lock) | same |
| mujoco | 3.11.0 | same |
| mujoco-warp | 3.11.0 | same |
| uv sync wall-clock | 18.13 s | `reports/logs/gn1-uv-sync.log` |

## Franka asset

- `newton.utils.download_asset("franka_emika_panda")` → `~/.cache/newton/newton-assets_franka_emika_panda_cdcfe7a8_a96f0973/franka_emika_panda`
- Asset repo commit: `a96f0973b6ae69c90609f9fafef9d2c1db2d6431` (matches the pin verified during planning)
- `urdf/fr3_franka_hand.urdf` sha256: `2a270e19a9b9c7ca5eb62ec9d503d779281605b6bba881f5ac6e8090aa382497`
- Download wall-clock: 3.70 s. Log: `reports/logs/gn1-asset.log`

## Headless GPU runs (all `--viewer null --quiet`, default `--num-frames 100`)

| Command (`cd newton && uv run --no-sync python -m newton.examples …`) | Wall-clock | Exit | Log |
|---|---|---|---|
| `softbody_franka --viewer null --quiet` | 1:38.71 | 0 (see deviation D3) | `reports/logs/gn1-softbody-franka.log` |
| `mpm_beam_twist --viewer null --quiet` | 0:28.07 | 0 | `reports/logs/gn1-mpm-beam-twist.log` |
| `mpm_anymal --viewer null --quiet` (M2 de-risk: MuJoCo+MPM composition) | 0:41.09 | 0 | `reports/logs/gn1-mpm-anymal.log` |
| `mujoco_mpm_coupled_solver --viewer null --quiet` (M2 de-risk: SolverCoupledProxy two-way template) | 0:18.08 | 0 | `reports/logs/gn1-mujoco-mpm-coupled.log` |

## Deviations

- **D1** — Python resolved to 3.12.13 (uv lock), not the system 3.13; within `requires-python >=3.10`. No action.
- **D2** — `warp-lang` resolved to `1.17.0.dev20260807` from the lockfile's nvidia index; recorded as the frozen version for all sprint runs.
- **D3** — The `softbody_franka` log captured only the output tail; the explicit `exit=$?` echo was added from the next run onward (with `pipefail`). Evidence of clean exit for this run: GNU time printed the wall-clock line with no `Command exited with non-zero status` marker, and the run produced no traceback. All subsequent runs record explicit exit codes.
- **D4** — `newton/` (engine clone incl. `.venv`) is gitignored, not vendored into this repo; the pin (`b74df534…` + `uv.lock` inside the clone) makes the environment reproducible via the exact commands in this receipt.
- **D5** — Non-manifold-edge `UserWarning` from the rubber-duck mesh in `softbody_franka` (upstream example asset); benign, does not affect our rig.
