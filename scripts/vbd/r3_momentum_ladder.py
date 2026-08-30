"""R3 pad-wrench momentum-residual convergence ladder (GPU driver).

Run from ``newton/``::

    PYTHONPATH=/home/simx2204/Workspace/DynamicTofu uv run --no-sync \
      python ../scripts/vbd/r3_momentum_ladder.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.vbd.tofu_probe import tofu_cfg
from src.pad_wrench import capture_pre_step, collect_pad_wrench
from src.vbd_rig2 import FPS, GRAB_Z, Vbd2Rig

F_CMD_N = 1.2
ITERATIONS = (10, 20, 40)
SUBSTEPS = 80
SAMPLE_WINDOW_S = 1.0


def _soft_momentum_z(rig, state, masses):
    velocities = state.particle_qd.numpy()[rig.soft_start:rig.soft_end, 2]
    return float(np.dot(masses, velocities.astype(np.float64)))


def _run(iterations):
    cfg = tofu_cfg(15e3, F_CMD_N, substeps=SUBSTEPS)
    cfg.vbd_iterations = iterations
    rig = Vbd2Rig(cfg)
    if not np.isclose(rig.sim_dt, 1.0 / 4800.0):
        raise RuntimeError(f"expected frozen dt=1/4800, got {rig.sim_dt}")

    masses = rig.model.particle_mass.numpy()[rig.soft_start:rig.soft_end].astype(np.float64)
    mass_kg = float(masses.sum())
    gravity_z_n = -mass_kg * 9.81
    t_preload_end = cfg.ramp_s + cfg.preload_s
    t_lift_end = t_preload_end + cfg.lift_s
    t_end = t_lift_end + cfg.hold_s
    sample_start = t_end - SAMPLE_WINDOW_S
    support_atomic = []
    support_stable = []
    residual_z = []

    for _frame_index in range(int(round(t_end * FPS))):
        t = rig.sim_time
        closing = F_CMD_N * min(1.0, t / cfg.ramp_s)
        lift_fraction = min(1.0, max(0.0, t - t_preload_end) / cfg.lift_s)
        lift_target = GRAB_Z + cfg.lift_height_m * lift_fraction
        rig.set_control(closing, lift_target)

        for _substep_index in range(rig.sim_substeps):
            rig.state_0.clear_forces()
            p_before = _soft_momentum_z(rig, rig.state_0, masses)
            pre_state = capture_pre_step(rig.state_0)
            rig.collision_pipeline.collide(rig.state_0, rig.contacts)
            rig.solver.step(rig.state_0, rig.state_1, rig.control, rig.contacts, rig.sim_dt)
            rig.state_0, rig.state_1 = rig.state_1, rig.state_0

            # Sample every solver substep in the final settled second. The collector
            # reports tofu-on-pad, so its negative is pad-on-tofu contact support.
            if t + (_substep_index + 1) * rig.sim_dt >= sample_start:
                p_after = _soft_momentum_z(rig, rig.state_0, masses)
                wrench = collect_pad_wrench(
                    rig,
                    pre_state=pre_state,
                    post_state=rig.state_0,
                    contacts=rig.contacts,
                    dt=rig.sim_dt,
                )
                atomic = -sum(wrench[pad]["force_world"][2] for pad in ("left", "right"))
                stable = -sum(wrench[pad]["force_world_stable"][2]
                              for pad in ("left", "right"))
                dp_dt = (p_after - p_before) / rig.sim_dt
                support_atomic.append(float(atomic))
                support_stable.append(float(stable))
                residual_z.append(float(atomic + gravity_z_n - dp_dt))

        rig.sim_time += rig.frame_dt

    atomic = np.asarray(support_atomic, dtype=np.float64)
    stable = np.asarray(support_stable, dtype=np.float64)
    residual = np.asarray(residual_z, dtype=np.float64)
    if len(residual) < int(0.9 * SAMPLE_WINDOW_S / rig.sim_dt):
        raise RuntimeError(f"insufficient settled substep samples: {len(residual)}")
    return {
        "iterations": iterations,
        "dt_s": float(rig.sim_dt),
        "substeps": rig.sim_substeps,
        "sample_count": int(len(residual)),
        "soft_mass_kg": mass_kg,
        "gravity_force_z_n": gravity_z_n,
        "support_atomic_n": float(np.median(atomic)),
        "support_stable_n": float(np.median(stable)),
        "atomic_stable_abs_diff_n": float(abs(np.median(atomic) - np.median(stable))),
        "atomic_stable_max_sample_diff_n": float(np.max(np.abs(atomic - stable))),
        "R_z_median_n": float(np.median(residual)),
        "abs_R_z_median_n": float(abs(np.median(residual))),
        "R_z_median_abs_samples_n": float(np.median(np.abs(residual))),
    }


def main():
    rows = [_run(iterations) for iterations in ITERATIONS]
    abs_residual = [row["abs_R_z_median_n"] for row in rows]
    monotone = all(later < earlier for earlier, later in zip(abs_residual, abs_residual[1:]))
    reduction = abs_residual[-1] / abs_residual[0] if abs_residual[0] > 0.0 else 0.0
    atomic_match = all(row["atomic_stable_abs_diff_n"] < 1e-3 for row in rows)
    accepted = bool(monotone and atomic_match)
    receipt = {
        "test": "R3 momentum residual iterations ladder",
        "frozen_settings": {
            "force_n": F_CMD_N,
            "substeps": SUBSTEPS,
            "dt_s": 1.0 / 4800.0,
            "iterations_ladder": list(ITERATIONS),
            "sample_window_s": SAMPLE_WINDOW_S,
        },
        "residual_definition": (
            "R_z = support_atomic_z + gravity_z - (P_z_post-P_z_pre)/dt; "
            "P_z=sum soft_particle_mass*particle_velocity_z"
        ),
        "per_iteration": rows,
        "abs_R_z_trend_n": abs_residual,
        "abs_R_z_strictly_decreasing": monotone,
        "final_to_initial_abs_R_z_ratio": reduction,
        "atomic_stable_match_below_1e-3_n": atomic_match,
        "pass": accepted,
        "verdict": "READY-FOR-REPROMOTION" if accepted else "NOT-READY",
    }
    path = ROOT / "reports/logs/vbd/r3_momentum_ladder.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt, indent=2))
    return 0 if accepted else 1


if __name__ == "__main__":
    raise SystemExit(main())
