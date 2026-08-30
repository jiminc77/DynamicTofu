"""Panda-hand variant of the frozen W1 transport runner.

The frame loop and judgment are delegated verbatim to w1_transport; only its rig
class is replaced process-locally with PandaRig.
"""
from __future__ import annotations

import argparse
import inspect
import json
import shutil
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from scripts.vbd import w1_transport
from src.vbd_rig_panda import PandaRig

PANDA_DEMO_SCENES = (
    {"scene": "intact", "E_kpa": 15.0, "F": 1.2, "a": 1.0, "seed": 0,
     "expected_label": "intact"},
    {"scene": "slip", "E_kpa": 15.0, "F": 1.2, "a": 30.0, "seed": 0,
     "expected_label": "slip"},
    {"scene": "damage", "E_kpa": 7.0, "F": 2.0, "a": 5.0, "seed": 0,
     "expected_label": "damage"},
)


def _panda_pad_shapes(rig):
    """Select the sole particle-colliding shape on each visualized finger."""
    import newton

    bodies = rig.model.shape_body.numpy()
    flags = rig.model.shape_flags.numpy()
    particle = int(newton.ShapeFlags.COLLIDE_PARTICLES)
    left = np.flatnonzero((bodies == rig.b_left) & ((flags & particle) != 0))
    right = np.flatnonzero((bodies == rig.b_right) & ((flags & particle) != 0))
    if len(left) != 1 or len(right) != 1:
        raise RuntimeError("expected exactly one particle-colliding pad per finger")
    return int(left[0]), int(right[0])


def run_panda_cell(E: float, F: float, a_peak: float, seed: int, couple: bool = True):
    import src.vbd_rig2 as frozen_rig

    original = frozen_rig.Vbd2Rig
    original_pad_shapes = w1_transport._pad_shapes
    frozen_rig.Vbd2Rig = lambda cfg: PandaRig(cfg, couple=couple)
    w1_transport._pad_shapes = _panda_pad_shapes
    try:
        receipt = w1_transport.run_transport_cell(
            E, F, a_peak, seed, substeps=80, cell_m=0.005
        )
    finally:
        frozen_rig.Vbd2Rig = original
        w1_transport._pad_shapes = original_pad_shapes
    receipt["rig"] = "panda"
    receipt["finger_coupling_enabled"] = bool(couple)
    receipt["commanded_per_pad_force_n"] = float(F)
    receipt["commanded_coupled_dof_force_n"] = float(F)
    receipt["finger_coupling"] = (
        "servo/software symmetric per-substep projection; Newton b74df534 "
        "SolverVBD lacks mechanical mimic/equality constraints"
    )
    return receipt


def _shape_center(rig, body_q, shape_index):
    """Compose a Newton body transform with a shape's local translation."""
    body = int(rig.model.shape_body.numpy()[shape_index])
    local = rig.model.shape_transform.numpy()[shape_index, :3]
    quat = body_q[body, 3:7]
    axis = quat[:3]
    rotated = (local + 2.0 * quat[3] * np.cross(axis, local)
               + 2.0 * np.cross(axis, np.cross(axis, local)))
    return body_q[body, :3] + rotated


def run_grip_diagnostic(rig_name: str, couple: bool = True):
    """Record the frozen E7/F2 grip trajectory without changing simulation."""
    from src.vbd_rig2 import GRAB_Z, Vbd2Config, Vbd2Rig

    cfg = Vbd2Config(
        E_pa=7000.0, nu=0.45, grip_force_n=2.0, cell_m=0.005,
        particle_radius=0.0025, contact_ke=1e3, contact_kd=1.0,
        mu_pair=1.0, friction_epsilon=2e-4, soft_contact_margin=1e-3,
        substeps=80, lift_s=2.5, hold_s=5.0, lift_height_m=0.05, seed=0,
    )
    rig = PandaRig(cfg, couple=couple) if rig_name == "panda" else Vbd2Rig(cfg)
    shape_body = rig.model.shape_body.numpy()
    left_shapes = np.flatnonzero(shape_body == rig.b_left)
    right_shapes = np.flatnonzero(shape_body == rig.b_right)
    if len(left_shapes) != 1 or len(right_shapes) != 1:
        raise RuntimeError("diagnostic requires exactly one pad shape per finger")
    left_shape, right_shape = int(left_shapes[0]), int(right_shapes[0])
    qstarts = rig.model.joint_q_start.numpy()
    left_qi = int(qstarts[rig.j_left])
    right_qi = int(qstarts[rig.j_right])
    t_pre = cfg.ramp_s + cfg.preload_s
    rows = []
    # Through the complete frozen hold, stopping before transport begins.
    while rig.sim_time < w1_transport.TRANSPORT_START - 0.5 / w1_transport.FPS:
        t = rig.sim_time
        close_force = cfg.grip_force_n * min(1.0, t / cfg.ramp_s)
        lift_fraction = min(1.0, max(0.0, t - t_pre) / cfg.lift_s)
        lift_target = GRAB_Z + cfg.lift_height_m * lift_fraction
        rig.step(close_force, lift_target, x_target=0.0, x_vel=0.0)
        body_q = rig.state_0.body_q.numpy()
        particle_q = rig.state_0.particle_q.numpy()[rig.soft_start:rig.soft_end]
        left_y = float(_shape_center(rig, body_q, left_shape)[1])
        right_y = float(_shape_center(rig, body_q, right_shape)[1])
        ymin, ymax = float(particle_q[:, 1].min()), float(particle_q[:, 1].max())
        pen_left = ymax - (left_y - 0.006)
        pen_right = (right_y + 0.006) - ymin
        joint_q = rig.state_0.joint_q.numpy()
        rows.append({
            "t": float(rig.sim_time),
            "phase": w1_transport.phase_for_time(rig.sim_time),
            "commanded_close_force_n": float(close_force),
            "pad_left_y_m": left_y, "pad_right_y_m": right_y,
            "block_ymin_m": ymin, "block_ymax_m": ymax,
            "pen_left_mm": float(pen_left * 1000.0),
            "pen_right_mm": float(pen_right * 1000.0),
            "fn_left_n": float(cfg.contact_ke * max(0.0, pen_left)),
            "fn_right_n": float(cfg.contact_ke * max(0.0, pen_right)),
            # Diagnostic evidence only: pure VBD integrates body_q, so joint_q
            # may be stale and must not be treated as authoritative motion.
            "joint_q_left": float(joint_q[left_qi]),
            "joint_q_right": float(joint_q[right_qi]),
        })
    out = ROOT / "reports/logs/vbd/panda"
    out.mkdir(parents=True, exist_ok=True)
    suffix = "_no_couple" if rig_name == "panda" and not couple else ""
    path = out / f"diag_E7_F2_a5_seed0_{rig_name}{suffix}.json"
    payload = {"rig": rig_name, "E_pa": 7000.0, "grip_force_n": 2.0,
               "commanded_a_peak_m_s2": 5.0, "seed": 0,
               "finger_coupling_enabled": bool(couple), "frames": rows}
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n")
    return path, rows


def _capture_runner(extended_slip=False):
    """Clone the production loop, changing capture cadence/termination only."""
    source = inspect.getsource(w1_transport.run_transport_cell)
    cadence = "if snap_dir and frame_index % 8 == 0:"
    if source.count(cadence) != 1:
        raise RuntimeError("production snapshot condition changed; refusing dense capture")
    source = source.replace(cadence, "if snap_dir and frame_index % 1 == 0:")
    if extended_slip:
        stop = ('            if gross_slip_mm(m, transport_reference) > GROSS_SLIP_MM:\n'
                '                series.append(m)\n'
                '                ejected = True\n'
                '                if drop_t is None:\n'
                '                    drop_t = float(m["t"])\n'
                '                break')
        keep = ('            if gross_slip_mm(m, transport_reference) > GROSS_SLIP_MM:\n'
                '                if not ejected:\n'
                '                    ejected = True\n'
                '                    if drop_t is None:\n'
                '                        drop_t = float(m["t"])\n'
                '                # Dense render capture continues after first ejection.')
        if source.count(stop) != 1:
            raise RuntimeError("production ejection block changed; refusing extended capture")
        source = source.replace(stop, keep)
    namespace = dict(w1_transport.__dict__)
    namespace["_pad_shapes"] = _panda_pad_shapes
    exec(compile(source, str(Path(w1_transport.__file__)), "exec"), namespace)
    return namespace["run_transport_cell"]


def dense_capture():
    """Capture every Panda simulation frame for the three fail-closed demos."""
    import src.vbd_rig2 as frozen_rig

    out = ROOT / "reports/vbd/clips/panda"
    out.mkdir(parents=True, exist_ok=True)
    metadata = {}

    def panda_factory(cfg):
        rig = PandaRig(cfg)
        if not metadata:
            metadata["body_index_to_label"] = {
                str(i): label for i, label in enumerate(rig.model.body_label)
            }
            metadata["body_rows"] = {
                "fr3_hand": rig.b_palm,
                "fr3_leftfinger": rig.b_left,
                "fr3_rightfinger": rig.b_right,
                "carriage": rig.b_carriage,
            }
            metadata["pad_shape_half_extents_m"] = {
                "hx": 0.022, "hy": 0.006, "hz": 0.022
            }
        return rig

    normal_run = _capture_runner(False)
    slip_run = _capture_runner(True)
    original = frozen_rig.Vbd2Rig
    entries = []
    try:
        frozen_rig.Vbd2Rig = panda_factory
        for spec in PANDA_DEMO_SCENES:
            scene = spec["scene"]
            target = out / f"w3_{scene}_dense"
            if target.exists():
                shutil.rmtree(target)
            target.mkdir(parents=True)
            runner = slip_run if scene == "slip" else normal_run
            receipt = runner(
                spec["E_kpa"] * 1000.0, spec["F"], spec["a"], spec["seed"],
                substeps=80, cell_m=0.005, snap_dir=target,
            )
            reproduced = receipt["label"] == spec["expected_label"]
            frames = sorted(target.glob("f_*.npz"))
            entry = {
                **spec, "rerun_label": receipt["label"],
                "label_reproduced": reproduced, "n_frames": len(frames),
                "drop_t": receipt.get("drop_t"),
                "capture_dir": str(target.relative_to(ROOT)),
                "continued_after_ejection": scene == "slip",
            }
            entries.append(entry)
            if not reproduced:
                raise RuntimeError(
                    f"{scene}: Panda dense label {receipt['label']!r} != "
                    f"{spec['expected_label']!r}"
                )
    finally:
        frozen_rig.Vbd2Rig = original
    manifest = {
        "schema": "w3_panda_manifest.v1", "rig": "panda",
        "frame_contents": ["particle_q", "body_q", "t"],
        **metadata, "scenes": entries,
    }
    path = out / "w3_panda_manifest.json"
    path.write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n")
    print(json.dumps(manifest, indent=2, allow_nan=False))
    return path


def write_receipt(receipt: dict) -> Path:
    out = ROOT / "reports/logs/vbd/panda"
    out.mkdir(parents=True, exist_ok=True)
    path = out / "E{:.0f}_F{:g}_a{:g}_seed{}.json".format(
        receipt["E_pa"] / 1000, receipt["grip_force_n"],
        receipt["commanded_a_peak_m_s2"], receipt["seed"]
    )
    path.write_text(json.dumps(w1_transport._json_safe(receipt), indent=2,
                               allow_nan=False) + "\n")
    return path


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--cell", nargs=4, metavar=("E_KPA", "F_N", "A", "SEED"))
    group.add_argument("--smoke", action="store_true")
    group.add_argument("--diag", choices=("panda", "frozen"))
    group.add_argument("--dense-capture", action="store_true")
    parser.add_argument(
        "--no-couple", action="store_true",
        help="disable Panda per-substep symmetry projection for diagnosis",
    )
    args = parser.parse_args()
    if args.dense_capture:
        if args.no_couple:
            parser.error("--no-couple is not valid for the fidelity capture")
        dense_capture()
        return 0
    if args.diag:
        if args.no_couple and args.diag != "panda":
            parser.error("--no-couple applies only to the Panda rig")
        path, rows = run_grip_diagnostic(args.diag, couple=not args.no_couple)
        final = rows[-1]
        print(json.dumps({
            "rig": args.diag, "receipt": str(path),
            "final_pen_left_mm": final["pen_left_mm"],
            "final_pen_right_mm": final["pen_right_mm"],
            "final_fn_left_n": final["fn_left_n"],
            "final_fn_right_n": final["fn_right_n"],
        }, indent=2))
        return 0
    if args.smoke:
        E_kpa, force, accel, seed = 15.0, 1.2, 5.0, 0
    else:
        E_kpa, force, accel, seed = (float(args.cell[0]), float(args.cell[1]),
                                      float(args.cell[2]), int(args.cell[3]))
    receipt = run_panda_cell(
        E_kpa * 1000.0, force, accel, seed, couple=not args.no_couple
    )
    path = write_receipt(receipt)
    print(json.dumps({"label": receipt["label"], "health": receipt["health"],
                      "commanded_per_pad_force_n": receipt["commanded_per_pad_force_n"],
                      "realized_F_g_n": receipt["realized_F_g_n"],
                      "receipt": str(path)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
