"""P3c attribution gate for SolverVBD's per-body contact reaction channel.

CLI (GPU): cd newton && uv run --no-sync python ../scripts/vbd/w2_attr_probe.py --probe
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (str(ROOT), str(Path(__file__).resolve().parent)):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def project_pad_reactions(body_forces, left_body, right_body):
    """Project world-frame force rows onto pad outward normals and tangent planes."""
    forces = np.asarray(body_forces, dtype=float)
    if forces.ndim != 2 or forces.shape[1] != 3:
        raise ValueError("body_forces must have shape (body_count, 3)")
    result = {}
    for name, body, normal in (("left", left_body, np.array([0.0, 1.0, 0.0])),
                                ("right", right_body, np.array([0.0, -1.0, 0.0]))):
        force = forces[int(body)]
        normal_n = float(np.dot(force, normal))
        tangent_n = float(np.linalg.norm(force - normal_n * normal))
        result[name] = {"force_world_n": force.tolist(), "normal_n": normal_n,
                        "tangential_n": tangent_n}
    return result


def decide_attr(seed_runs, absent_run, commanded_force, absent_limit=0.02, equilibrium_rtol=0.15):
    """Apply checks A--E to already-reduced synthetic or measured samples."""
    def seed_checks(run):
        normals = np.array([run[p]["normal_n"] for p in ("left", "right")], float)
        penalty = np.array([run[p]["penalty_n"] for p in ("left", "right")], float)
        finger_vy = float(run["finger_vy"])
        finite = bool(np.all(np.isfinite(normals)))
        a = finite and bool(np.all(normals > 0.0)) and bool(np.all(normals < 5.0 * commanded_force))
        d = (finite and bool(np.all(np.abs(normals - commanded_force) <= equilibrium_rtol * commanded_force))
             and bool(np.all(np.abs(normals - penalty) <= equilibrium_rtol * commanded_force))
             and abs(finger_vy) <= 0.01)
        return {"A_sign_units": a, "D_equilibrium": d, "normals_n": normals.tolist(),
                "penalty_n": penalty.tolist(), "finger_vy_m_s": finger_vy}

    per_seed = [seed_checks(run) for run in seed_runs]
    absent_normals = np.array([abs(absent_run[p]["normal_n"]) for p in ("left", "right")])
    b = bool(np.all(absent_normals < absent_limit))
    # The absent run retains commanded joint effort, so vanishing output distinguishes
    # the contact accumulator from joint_f. Positive penetration/contact correlation is
    # cross-checked by the penalty estimate in D.
    c = b and all(check["A_sign_units"] and check["D_equilibrium"] for check in per_seed)
    a_all = all(check["A_sign_units"] for check in per_seed)
    d_all = all(check["D_equilibrium"] for check in per_seed)
    e = len(seed_runs) == 3 and a_all and d_all and b and c
    checks = {
        "A_sign_units": {"pass": a_all, "per_seed": per_seed},
        "B_block_absent": {"pass": b, "normal_abs_n": absent_normals.tolist(), "limit_n": absent_limit},
        "C_contact_not_joint_effort": {"pass": c, "control_joint_effort_n": commanded_force,
                                         "control_normal_abs_n": absent_normals.tolist()},
        "D_equilibrium": {"pass": d_all, "relative_tolerance": equilibrium_rtol},
        "E_three_seed_reproducibility": {"pass": e, "seed_count": len(seed_runs)},
    }
    available = all(item["pass"] for item in checks.values())
    return {"checks": checks, "verdict": "AVAILABLE" if available else "GEOMETRY_ONLY",
            "failed_checks": [name for name, item in checks.items() if not item["pass"]]}


def _production_cfg(E, F, seed):
    sys.path.insert(0, str(ROOT))
    from scripts.vbd.tofu_probe import tofu_cfg
    cfg = tofu_cfg(E, F, substeps=80)
    cfg.seed = seed
    return cfg


def _run_one(E, F, seed, block_absent=False):
    from src.vbd_rig2 import FPS, GRAB_Z, Vbd2Rig

    rig = Vbd2Rig(_production_cfg(E, F, seed))
    solver_forces = getattr(rig.solver, "body_forces", None)
    if solver_forces is None or tuple(solver_forces.shape) != (rig.model.body_count,):
        raise RuntimeError("SolverVBD.body_forces is unavailable or has an unexpected layout")
    if block_absent:
        # Keep the full commanded closing joint effort but translate the soft block far
        # along x in both ping-pong states, making pad/block contact impossible.
        for state in (rig.state_0, rig.state_1):
            q = state.particle_q.numpy()
            q[rig.soft_start:rig.soft_end, 0] += 10.0
            state.particle_q.assign(q)

    cfg = rig.cfg
    t_pre = cfg.ramp_s + cfg.preload_s
    t_lift = t_pre + cfg.lift_s
    t_end = t_lift + cfg.hold_s
    samples = []
    for _ in range(int(t_end * FPS)):
        t = rig.sim_time
        closing = F * min(1.0, t / cfg.ramp_s)
        lift = GRAB_Z + cfg.lift_height_m * min(1.0, max(0.0, t - t_pre) / cfg.lift_s)
        rig.step(closing, lift)
        if rig.sim_time >= t_lift:
            projected = project_pad_reactions(rig.solver.body_forces.numpy(), rig.b_left, rig.b_right)
            metrics = rig.metrics()
            projected["left"]["penalty_n"] = float(metrics["fn_left_n"])
            projected["right"]["penalty_n"] = float(metrics["fn_right_n"])
            projected["finger_vy"] = float(metrics["finger_vy_linear"])
            samples.append(projected)
    if not samples:
        raise RuntimeError("hold produced no samples")
    reduced = {}
    for pad in ("left", "right"):
        reduced[pad] = {key: float(np.median([s[pad][key] for s in samples]))
                        for key in ("normal_n", "tangential_n", "penalty_n")}
    reduced["finger_vy"] = float(np.mean([s["finger_vy"] for s in samples]))
    reduced["sample_count"] = len(samples)
    return reduced


def run_attr(E=15e3, F=1.2, seed=0):
    """Run one contacted production-config seed and return reduced hold evidence."""
    return _run_one(E, F, seed, block_absent=False)


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--probe", action="store_true", help="run three seeds and block-absent control")
    args = parser.parse_args(argv)
    if not args.probe:
        parser.error("--probe is required")
    prereg = ROOT / "ralph/results/prereg_w1.json"
    provenance = {
        "git_sha": subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip(),
        "prereg_sha256": _sha256(prereg),
        "solver_attribute": "SolverVBD.body_forces",
        "solver_layout_units": "wp.array[wp.vec3], shape (model.body_count,), world-frame newtons",
    }
    try:
        seeds = [_run_one(15e3, 1.2, seed) for seed in range(3)]
        absent = _run_one(15e3, 1.2, 0, block_absent=True)
        result = decide_attr(seeds, absent, 1.2)
    except (AttributeError, RuntimeError) as exc:
        seeds, absent = [], None
        result = {"checks": {}, "verdict": "GEOMETRY_ONLY", "failed_checks": ["force_channel_unavailable"],
                  "error": str(exc)}
    receipt = {"gate": "P3c_ATTR", "inputs": {"E_pa": 15e3, "grip_force_n": 1.2,
               "seeds": [0, 1, 2], "substeps": 80}, "provenance": provenance,
               "seed_runs": seeds, "block_absent_control": absent, **result}
    output = ROOT / "reports/logs/vbd/w2_attr_probe.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt, indent=2))
    return 0 if result["verdict"] == "AVAILABLE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
