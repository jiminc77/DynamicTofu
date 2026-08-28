"""Production W1 world-x transport runner and P3 tracking gates.

GPU entry points (run from the Newton environment):
  python scripts/vbd/w1_transport.py --smoke
  python scripts/vbd/w1_transport.py --tracking-ladder [--noise-floor]
  python scripts/vbd/w1_transport.py --cell E_kPa F_N a_m_s2 seed
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
sys.path.insert(0, str(ROOT))

from src.frozen_config import assert_frozen, frozen_provenance
from src.judgment_vbd import (PHASE_NAMES, label, latched_dvf,
                              per_phase_strain_maxima, phase_for_time, slip3d)
from src.transport import g_trk, realized_accel, trapezoid_reversal

FPS = 60
T_END = 11.6
TRANSPORT_START = 9.30
PLATEAU_WINDOWS = {
    "accel_out": {"start": 9.35, "end": 9.45, "a_cmd": 0.0},
    "decel_out": {"start": 9.65, "end": 9.75, "a_cmd": 0.0},
    "accel_back": {"start": 10.15, "end": 10.25, "a_cmd": 0.0},
    "decel_back": {"start": 10.45, "end": 10.55, "a_cmd": 0.0},
}


def phase_timestamp_table(series, expected):
    """Return first sampled timestamp and error for each expected phase."""
    first = {}
    for frame in series:
        phase = frame.get("phase")
        if phase is not None and phase not in first:
            first[phase] = float(frame["t"])
    return {name: {"expected_s": float(bounds[0]), "hit_s": first.get(name),
                   "error_s": None if name not in first else first[name] - float(bounds[0])}
            for name, bounds in expected.items()}


def tracking_receipt(times, velocities, windows):
    """CPU-only assembly of the stable tracking receipt surface."""
    gate = g_trk(times, velocities, windows)
    return {"plateaus": gate["fits"], "max_abs_relative_error": gate["max_abs_err"],
            "samples_valid": gate["samples_valid"], "pass": gate["pass"]}


def _pad_shapes(rig):
    shape_body = rig.model.shape_body.numpy()
    left = np.flatnonzero(shape_body == rig.b_left)
    right = np.flatnonzero(shape_body == rig.b_right)
    if len(left) != 1 or len(right) != 1:
        raise RuntimeError("expected exactly one collision shape on each finger pad")
    return int(left[0]), int(right[0])


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _write_receipt(receipt):
    out = ROOT / "reports/logs/vbd/w1_cells"
    out.mkdir(parents=True, exist_ok=True)
    name = "E{:.0f}_F{:g}_a{:g}_seed{}.json".format(receipt["E_pa"] / 1000, receipt["grip_force_n"],
                                                   receipt["commanded_a_peak_m_s2"], receipt["seed"])
    path = out / name
    path.write_text(json.dumps(receipt, indent=2, allow_nan=False) + "\n")
    return path


def run_transport_cell(E, F, a_peak, seed, substeps=80, cell_m=0.005,
                       snap_dir=None, save_field=None):
    from src.contact_validity import ValidityAccumulator
    from src.vbd_rig2 import GRAB_Z, Vbd2Config, Vbd2Rig

    cfg = Vbd2Config(E_pa=float(E), nu=0.45, grip_force_n=float(F), cell_m=cell_m,
                     particle_radius=0.0025, contact_ke=1e3, contact_kd=1.0,
                     mu_pair=1.0, friction_epsilon=2e-4, soft_contact_margin=1e-3,
                     substeps=substeps, lift_s=2.5, hold_s=5.0,
                     lift_height_m=0.05, seed=int(seed))
    assert_frozen(cfg)
    rig = Vbd2Rig(cfg)
    left_shape, right_shape = _pad_shapes(rig)
    validity = ValidityAccumulator(left_shape, right_shape, (rig.soft_start, rig.soft_end),
                                   rig.sim_dt, margin=cfg.soft_contact_margin)
    rig.add_substep_hook(validity)
    profile = trapezoid_reversal(a_peak) if a_peak > 0 else None

    series = []
    phase_fields = {}
    temporal_max = None
    rest_vol = None
    snap_index = 0
    t_pre = cfg.ramp_s + cfg.preload_s
    for frame_index in range(round(T_END * FPS)):
        t = rig.sim_time
        cf = F * min(1.0, t / cfg.ramp_s)
        lift_fraction = min(1.0, max(0.0, t - t_pre) / cfg.lift_s)
        lt = GRAB_Z + cfg.lift_height_m * lift_fraction
        active = TRANSPORT_START <= t <= T_END and profile is not None
        rig.step(cf, lt, x_target=profile.x_cmd(t) if active else 0.0,
                 x_vel=profile.v_cmd(t) if active else 0.0,
                 x_accel=profile.a_cmd(t) if active else 0.0)  # D3-C accel feed-forward
        validity.fold_frame()
        m = rig.metrics()
        m["phase"] = phase_for_time(m["t"])
        m["contacts"] = rig.contact_count()
        field, rest_vol = rig.strain_field()
        temporal_max = field.copy() if temporal_max is None else np.maximum(temporal_max, field)
        if m["phase"] is not None:
            previous = phase_fields.get(m["phase"])
            phase_fields[m["phase"]] = field.copy() if previous is None else np.maximum(previous, field)
        if m["t"] >= TRANSPORT_START or m["phase"] in ("ramp", "preload", "lift", "hold"):
            series.append(m)
        if snap_dir and frame_index % 8 == 0:
            Path(snap_dir).mkdir(parents=True, exist_ok=True)
            np.savez_compressed(Path(snap_dir) / f"f_{snap_index:04d}.npz",
                                particle_q=rig.state_0.particle_q.numpy()[rig.soft_start:rig.soft_end].astype(np.float32),
                                body_q=rig.state_0.body_q.numpy().astype(np.float32), t=np.float64(rig.sim_time))
            snap_index += 1

    if save_field:
        Path(save_field).parent.mkdir(parents=True, exist_ok=True)
        final_field, _ = rig.strain_field()
        np.savez_compressed(save_field, temporal_max_principal_strain=temporal_max,
                            final_principal_strain=final_field, tet_rest_vol=rest_vol)
    transport = [m for m in series if m["t"] >= TRANSPORT_START]
    times = [m["t"] for m in transport]
    velocities = [m["palm_vx"] for m in transport]
    if profile is not None:
        tracking = tracking_receipt(times, velocities, profile.plateau_windows)
        expected_phases = profile.phase_timestamps
        zero_command_accel = None
    else:
        tracking = None
        unit_profile = trapezoid_reversal(1.0)
        expected_phases = unit_profile.phase_timestamps
        zero_windows = {name: {**window, "a_cmd": 1.0}
                        for name, window in unit_profile.plateau_windows.items()}
        zero_fits = realized_accel(times, velocities, zero_windows)
        zero_command_accel = max(abs(fit["a_fit"]) for fit in zero_fits.values())
    hold = [m for m in series if m["phase"] == "hold"]
    hold_ref = None
    legacy_slip = 0.0
    for m in hold:
        rel = m["com_z"] - m["palm_z"]
        hold_ref = rel if hold_ref is None else hold_ref
        legacy_slip = max(legacy_slip, abs(rel - hold_ref) * 1000)
    dvf, damaged = latched_dvf(temporal_max, rest_vol)
    slip = slip3d(series, TRANSPORT_START)
    strain_maxima = per_phase_strain_maxima(phase_fields)
    vg = validity.summary()
    certified = all(p["certified"] for p in vg["per_pad"].values())
    stats = rig.strain_stats(0.15)
    finite = all(m["finite"] for m in series)
    fn_hold = [0.5 * (m["fn_left_n"] + m["fn_right_n"]) for m in hold]
    receipt = {
        "E_pa": float(E), "grip_force_n": float(F), "seed": int(seed),
        "commanded_a_peak_m_s2": float(a_peak), "tracking": tracking,
        "realized_F_g_n": float(np.mean(fn_hold)) if fn_hold else float("nan"),
        "slip3d_mm": slip, "legacy_hold_slip_mm": legacy_slip,
        "dvf": dvf, "damage_latched": damaged, "label": label({"dvf": dvf, "damage_latched": damaged, "slip3d_mm": slip}),
        "p99_strain": stats["p99_vol_weighted_strain"], "peak_strain": float(np.max(temporal_max)),
        "per_phase_strain_maxima": strain_maxima,
        "validity_gate": {"summary": vg, "certified": certified},
        "health": {"finite": finite,
                   "finger_vy_hold_mean": float(np.mean([m["finger_vy"] for m in hold])) if hold else float("nan"),
                   "contact_count_min": min((m["contacts"] for m in series), default=0),
                   "cfl_max_substep_mm": vg["cfl_max_substep_mm"],
                   "zero_command_realized_accel_magnitude": zero_command_accel},
        "phase_timestamps": phase_timestamp_table(series, expected_phases),
        "git_sha": subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip(),
        "prereg_sha256": _sha256(ROOT / "ralph/results/prereg_w1.json"),
        **frozen_provenance(),
    }
    return receipt


def _print_tracking(receipt):
    print(json.dumps(receipt["tracking"], indent=2))


def main(argv=None):
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--smoke", action="store_true")
    mode.add_argument("--tracking-ladder", action="store_true")
    mode.add_argument("--cell", nargs=4, metavar=("E_KPA", "F_N", "A", "SEED"))
    parser.add_argument("--noise-floor", action="store_true")
    args = parser.parse_args(argv)
    if args.cell:
        E, F, a, seed = args.cell
        receipt = run_transport_cell(float(E) * 1000, float(F), float(a), int(seed))
        path = _write_receipt(receipt)
        print(path)
        return 0
    if args.smoke:
        receipt = run_transport_cell(15e3, 1.2, 5.0, 0)
        _write_receipt(receipt)
        print(json.dumps({"phase_timestamps": receipt["phase_timestamps"], "tracking": receipt["tracking"],
                          "label": receipt["label"], "provenance": receipt["frozen_config"]}, indent=2))
        timestamps_ok = all(row["hit_s"] is not None and abs(row["error_s"]) <= 1 / FPS
                            for row in receipt["phase_timestamps"].values())
        return 0 if timestamps_ok and receipt["tracking"]["pass"] else 1
    ladder = []
    passed = True
    for acceleration in (1, 2.5, 5, 10, 20, 30):
        receipt = run_transport_cell(15e3, 1.2, acceleration, 0)
        _write_receipt(receipt)
        ladder.append({"a_peak_m_s2": acceleration, **receipt["tracking"]})
        passed &= receipt["tracking"]["pass"]
    noise_receipt = run_transport_cell(15e3, 1.2, 0.0, 0)
    _write_receipt(noise_receipt)
    transport_note = noise_receipt["phase_timestamps"]
    # Fit stationary-command slopes in the same four immutable windows.
    # The full series is intentionally not persisted in production receipts; run_transport_cell
    # records the result below when a=0 through the health telemetry.
    noise = float(noise_receipt["health"]["zero_command_realized_accel_magnitude"])
    output = {"ladder": ladder, "noise_floor_m_s2": noise, "noise_floor_pass": noise <= 0.01,
              "zero_phase_timestamps": transport_note}
    out = ROOT / "reports/logs/vbd/g_trk_ladder.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(output, indent=2, allow_nan=False) + "\n")
    print(json.dumps(output, indent=2))
    return 0 if passed and noise <= 0.01 else 1


if __name__ == "__main__":
    raise SystemExit(main())
