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


def _frame(rig, force):
    """Equivalent to rig.step, retaining the pre-state of its final substep."""
    rig.set_control(force, GRAB_Z)
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
    samples = _settle(False)
    fn_l, fn_r = _median(samples, "left", "Fn"), _median(samples, "right", "Fn")
    rtol = 0.15
    passed = abs(fn_l - F_CMD) <= rtol * F_CMD and abs(fn_r - F_CMD) <= rtol * F_CMD
    return {"pass": bool(passed), "Fn_left_n": fn_l, "Fn_right_n": fn_r,
            "F_cmd_n": F_CMD, "relative_tolerance": rtol}


def test_momentum():
    samples = _settle(False)
    # force_world is force tofu exerts on pads, so negate summed pad world-z to get
    # the upward force pads exert on tofu.
    support = float(np.median([-s["left"]["force_world"][2] - s["right"]["force_world"][2]
                               for s in samples]))
    atol = 0.10
    return {"pass": bool(abs(support - WEIGHT_N) <= atol),
            "component": "negative sum of left/right pad force_world[z] (world z is up)",
            "vertical_support_n": support, "weight_n": WEIGHT_N,
            "absolute_tolerance_n": atol}


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
