"""GPU acceptance tests for the R3 soft/rigid pad-wrench collector.

Run from newton/ with the command documented in main().
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.pad_wrench import capture_pre_step, collect_pad_wrench
from src.vbd_rig2 import FPS, GRAB_Z, Vbd2Rig
from scripts.vbd.tofu_probe import tofu_cfg

F_CMD = 1.2
WEIGHT_N = 0.064 * 9.81


def _rig(absent=False):
    cfg = tofu_cfg(15e3, F_CMD, substeps=80)
    rig = Vbd2Rig(cfg)
    if absent:
        for state in (rig.state_0, rig.state_1):
            q = state.particle_q.numpy()
            q[rig.soft_start:rig.soft_end, 0] += 10.0
            state.particle_q.assign(q)
    return rig


def _frame(rig, force, lift_target=GRAB_Z):
    """Equivalent to rig.step, retaining the pre-state of its final substep."""
    rig.set_control(force, lift_target)
    pre = None
    for _ in range(rig.sim_substeps):
        rig.state_0.clear_forces()
        pre = capture_pre_step(rig.state_0)
        rig.collision_pipeline.collide(rig.state_0, rig.contacts)
        rig.solver.step(rig.state_0, rig.state_1, rig.control, rig.contacts, rig.sim_dt)
        rig.state_0, rig.state_1 = rig.state_1, rig.state_0
    rig.sim_time += rig.frame_dt
    return collect_pad_wrench(rig, pre_state=pre, post_state=rig.state_0,
                              contacts=rig.contacts, dt=rig.sim_dt)


def _settle(absent=False):
    rig = _rig(absent)
    last = None
    # Frozen schedule: 0.8 s force ramp plus 1.0 s preload, then 1.0 s static sampling.
    end = rig.cfg.ramp_s + rig.cfg.preload_s + 1.0
    samples = []
    for _ in range(int(end * FPS)):
        force = F_CMD * min(1.0, rig.sim_time / rig.cfg.ramp_s)
        last = _frame(rig, force)
        if rig.sim_time >= rig.cfg.ramp_s + rig.cfg.preload_s:
            samples.append(last)
    return samples


_SUSPENDED_CACHE = None


def _suspended_settle():
    """Run ramp, preload, lift, and hold; return the final second of suspended data."""
    global _SUSPENDED_CACHE
    if _SUSPENDED_CACHE is not None:
        return _SUSPENDED_CACHE

    rig = _rig(False)
    cfg = rig.cfg
    initial_com_z = float(rig.state_0.particle_q.numpy()[rig.soft_start:rig.soft_end, 2].mean())
    t_pre = cfg.ramp_s + cfg.preload_s
    t_lift = t_pre + cfg.lift_s
    t_end = t_lift + cfg.hold_s
    rows = []
    for _ in range(int(t_end * FPS)):
        t = rig.sim_time
        force = F_CMD * min(1.0, t / cfg.ramp_s)
        lift_fraction = min(1.0, max(0.0, t - t_pre) / cfg.lift_s)
        lift_target = GRAB_Z + cfg.lift_height_m * lift_fraction
        wrench = _frame(rig, force, lift_target)
        if rig.sim_time >= t_end - 1.0:
            com_z = float(rig.state_0.particle_q.numpy()[rig.soft_start:rig.soft_end, 2].mean())
            rows.append({"time_s": float(rig.sim_time), "com_z_m": com_z, "wrench": wrench})

    com_z = np.asarray([row["com_z_m"] for row in rows], dtype=np.float64)
    velocity = np.diff(com_z) * FPS
    acceleration = np.diff(velocity) * FPS
    rise = float(np.median(com_z) - initial_com_z)
    median_abs_velocity = float(np.median(np.abs(velocity)))
    median_abs_acceleration = float(np.median(np.abs(acceleration)))
    max_abs_velocity = float(np.max(np.abs(velocity)))
    max_abs_acceleration = float(np.max(np.abs(acceleration)))
    velocity_limit = 5e-3
    acceleration_limit = 0.5
    rise_tolerance = 0.01
    checks = {
        "initial_com_z_m": initial_com_z,
        "median_hold_com_z_m": float(np.median(com_z)),
        "com_rise_m": rise,
        "commanded_lift_m": float(cfg.lift_height_m),
        "rise_tolerance_m": rise_tolerance,
        "median_abs_com_velocity_m_s": median_abs_velocity,
        "max_abs_com_velocity_m_s": max_abs_velocity,
        "velocity_limit_m_s": velocity_limit,
        "median_abs_com_acceleration_m_s2": median_abs_acceleration,
        "max_abs_com_acceleration_m_s2": max_abs_acceleration,
        "acceleration_limit_m_s2": acceleration_limit,
        "airborne_pass": bool(abs(rise - cfg.lift_height_m) <= rise_tolerance),
        # Median absolute finite differences reject isolated float32/solver jitter
        # while requiring the entire settled window to be centered near rest.
        "static_pass": bool(median_abs_velocity <= velocity_limit
                            and median_abs_acceleration <= acceleration_limit),
    }
    _SUSPENDED_CACHE = ([row["wrench"] for row in rows], rows, checks)
    return _SUSPENDED_CACHE


def _median(samples, pad, key):
    return float(np.median([s[pad][key] for s in samples]))


def test_absent():
    samples = _settle(True)
    measured = {}
    passed = True
    for pad in ("left", "right"):
        force_norm = max(float(np.linalg.norm(s[pad]["force_world"])) for s in samples)
        fn_abs = max(abs(float(s[pad]["Fn"])) for s in samples)
        measured[pad] = {"max_force_norm_n": force_norm, "max_abs_Fn_n": fn_abs}
        passed &= force_norm < 1e-6 and fn_abs < 1e-6
    return {"pass": bool(passed), "limit_n": 1e-6, "measured": measured}


def test_static():
    samples, _rows, suspension = _suspended_settle()
    fn_l, fn_r = _median(samples, "left", "Fn"), _median(samples, "right", "Fn")
    rtol = 0.15
    passed = (abs(fn_l - F_CMD) <= rtol * F_CMD
              and abs(fn_r - F_CMD) <= rtol * F_CMD
              and suspension["airborne_pass"] and suspension["static_pass"])
    return {"pass": bool(passed), "Fn_left_n": fn_l, "Fn_right_n": fn_r,
            "F_cmd_n": F_CMD, "relative_tolerance": rtol,
            "suspension_checks": suspension}


def test_momentum():
    samples, rows, suspension = _suspended_settle()
    # force_world is force tofu exerts on pads, so negate summed pad world-z to get
    # the upward force pads exert on tofu.
    support_series = np.asarray(
        [-s["left"]["force_world"][2] - s["right"]["force_world"][2] for s in samples],
        dtype=np.float64,
    )
    support = float(np.median(support_series))
    atol = 0.10
    series_indices = np.linspace(0, len(rows) - 1, min(10, len(rows)), dtype=int)
    short_series = [{"time_s": rows[i]["time_s"],
                     "vertical_support_n": float(support_series[i])}
                    for i in series_indices]
    passed = (abs(support - WEIGHT_N) <= atol
              and suspension["airborne_pass"] and suspension["static_pass"])
    return {"pass": bool(passed),
            "component": "negative sum of left/right pad force_world[z] (world z is up)",
            "vertical_support_n": support, "weight_n": WEIGHT_N,
            "absolute_tolerance_n": atol, "support_time_series": short_series,
            "suspension_checks": suspension}


def main(argv=None):
    parser = argparse.ArgumentParser(description="R3 pad-wrench GPU acceptance tests")
    parser.add_argument("--test", choices=("absent", "static", "momentum", "all"), default="all")
    args = parser.parse_args(argv)
    funcs = {"absent": test_absent, "static": test_static, "momentum": test_momentum}
    selected = funcs if args.test == "all" else {args.test: funcs[args.test]}
    receipt = {"collector": "soft-rigid VBD contact only", "tests": {}}
    try:
        for name, fn in selected.items():
            receipt["tests"][name] = fn()
    except Exception as exc:
        receipt["error"] = f"{type(exc).__name__}: {exc}"
    receipt["pass"] = "error" not in receipt and all(v["pass"] for v in receipt["tests"].values())
    path = ROOT / "reports/logs/vbd/r3_pad_wrench.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt, indent=2))
    return 0 if receipt["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
