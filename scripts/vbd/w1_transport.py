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
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from src.frozen_config import assert_frozen, frozen_provenance
from src.judgment_vbd import (LIFT_END, PHASE_NAMES, grasp_frame_y_res_mm,
                              hold_slip_z_mm, label_v23, latched_dvf,
                              per_phase_strain_maxima, phase_for_time,
                              slip3d_max_mm, slip_perm_tangential_mm,
                              slip_perm_x_mm, transport_slip_xz_mm,
                              transport_x_res_mm, transport_y_res_mm,
                              transport_z_res_mm, x_res_mm, y_res_mm,
                              yz_residual_mm, z_res_mm)
from src.transport import g_trk, realized_accel, trapezoid_reversal

FPS = 60
T_END = 11.6
TRANSPORT_START = 9.30
GROSS_SLIP_MM = 15.0
PLATEAU_WINDOWS = {
    "accel_out": {"start": 9.35, "end": 9.45, "a_cmd": 0.0},
    "decel_out": {"start": 9.65, "end": 9.75, "a_cmd": 0.0},
    "accel_back": {"start": 10.15, "end": 10.25, "a_cmd": 0.0},
    "decel_back": {"start": 10.45, "end": 10.55, "a_cmd": 0.0},
}
MATERIAL_ORDER_KPA = (7, 25, 15)
SCREEN_FORCES = (0.4, 0.6, 0.8, 1.0, 1.2, 1.5, 2.0)
SCREEN_A_ORDER = (1, 5, 10, 20, 30, 2.5)


def screen_grid(material=None):
    """Return the pre-registered screen order as (E_kPa, a, F, seed)."""
    materials = (int(material),) if material is not None else MATERIAL_ORDER_KPA
    return [(E, a, F, 0) for E in materials
            for a in SCREEN_A_ORDER for F in SCREEN_FORCES]


def pending_screen_cells(grid, receipt_dir, resume):
    """Filter already persisted cells only when idempotent resume is requested."""
    receipt_dir = Path(receipt_dir)
    return [cell for cell in grid if not (
        resume and (receipt_dir / screen_receipt_name(*cell)).exists()
    )]


def expand_confirm_plan(plan):
    """Expand confirmation coordinates into screen-compatible seed cells."""
    expanded = []
    for item in plan:
        E = int(item["E_kPa"])
        acceleration = float(item["commanded_a_peak_m_s2"])
        force = float(item["grip_force_n"])
        seeds = item["seeds_to_run"]
        if not isinstance(seeds, list) or not seeds:
            raise ValueError("each confirmation cell requires non-empty seeds_to_run")
        expanded.extend((E, acceleration, force, int(seed)) for seed in seeds)
    return expanded


def screen_receipt_name(E_kpa, a, F, seed=0):
    return f"E{E_kpa}_F{F:g}_a{a:g}_s{seed}.json"


def load_axis_map(path):
    data = json.loads(Path(path).read_text())
    if data.get("result") != "PASS":
        raise RuntimeError("G-TRK realized-axis gate has not passed")
    return {float(row["commanded_a_peak"]): float(row["realized_median_m_s2"])
            for row in data["axis_map_commanded_to_realized"]}


def screen_cell_receipt(cell, axis_map):
    """Project a full runner receipt onto the stable W1 screen contract."""
    commanded = float(cell["commanded_a_peak_m_s2"])
    measured = (realized_from_fits(cell["tracking"]["plateaus"])
                if cell.get("tracking") else None)
    realized = measured if measured is not None else float(axis_map[commanded])
    return {
        "schema": "w1_screen_cell.v1", "status": cell.get("status", "ok"),
        "E_pa": cell["E_pa"], "grip_force_n": cell["grip_force_n"],
        "seed": cell["seed"], "commanded_a_peak_m_s2": commanded,
        "realized_accel_m_s2": realized,
        "realized_source": "cell" if measured is not None else "axis_map",
        "axis_map_realized_accel_m_s2": float(axis_map[commanded]),
        "hold_slip_z_mm": cell.get("hold_slip_z_mm"),
        "transport_slip_xz_mm": cell.get("transport_slip_xz_mm"),
        "transport_x_res_mm": cell.get("transport_x_res_mm"),
        "transport_z_res_mm": cell.get("transport_z_res_mm"),
        "transport_y_res_mm": cell.get("transport_y_res_mm"),
        "grasp_frame_y_res_mm": cell.get("grasp_frame_y_res_mm"),
        "assembly_drift_mm": cell.get("assembly_drift_mm"),
        "escape_mode": cell.get("escape_mode"),
        "slip_perm_tangential_mm": cell.get("slip_perm_tangential_mm"),
        "x_res_mm": cell.get("x_res_mm"), "z_res_mm": cell.get("z_res_mm"),
        "y_res_mm": cell.get("y_res_mm"), "slip_perm_x_mm": cell.get("slip_perm_x_mm"),
        "slip3d_max_mm": cell.get("slip3d_max_mm"),
        "yz_residual_mm": cell.get("yz_residual_mm"),
        "legacy_hold_slip_mm": cell.get("legacy_hold_slip_mm"),
        "settle_end_block_x": cell.get("settle_end_block_x"),
        "settle_end_palm_x": cell.get("settle_end_palm_x"),
        "ref_block_x": cell.get("ref_block_x"), "ref_palm_x": cell.get("ref_palm_x"),
        "grip_ref_com": cell.get("grip_ref_com"), "grip_ref_palm": cell.get("grip_ref_palm"),
        "transport_ref_com": cell.get("transport_ref_com"),
        "transport_ref_palm": cell.get("transport_ref_palm"),
        "settle_end_com": cell.get("settle_end_com"),
        "settle_end_palm": cell.get("settle_end_palm"),
        "label": cell.get("label", "error"), "dvf": cell.get("dvf"),
        "dvf_wholetrial": cell.get("dvf_wholetrial"),
        "damage_window": cell.get("damage_window"), "lift_end_s": cell.get("lift_end_s"),
        "damage_latch_t": cell.get("damage_latch_t"), "drop_t": cell.get("drop_t"),
        "damage_after_drop": cell.get("damage_after_drop", False),
        "p99_strain": cell.get("p99_strain"), "peak_strain": cell.get("peak_strain"),
        "validity_gate": cell.get("validity_gate"), "ejected": cell.get("ejected", False),
        "per_phase_strain_maxima": cell.get("per_phase_strain_maxima"),
        "health": cell.get("health", {"finite": False}), "git_sha": cell.get("git_sha"),
        "prereg_sha256": cell.get("prereg_sha256"),
        "frozen_config": cell.get("frozen_config"), "frozen_check": cell.get("frozen_check"),
        "rig_pre_edit_sha256": cell.get("rig_pre_edit_sha256"),
        "newton_commit": cell.get("newton_commit"), "msg": cell.get("msg"),
    }


def update_band(band, receipt):
    """Incrementally add/replace one cell in an e1v2 material band."""
    if band is None:
        band = {"schema": "e1v2_band.v1", "E_kPa": int(receipt["E_pa"] / 1000),
                "a_order": list(SCREEN_A_ORDER), "F_order_N": list(SCREEN_FORCES),
                "realized_accel_by_commanded": {}, "label_matrix": {}, "cells": {},
                "coverage": {"completed": [], "failed": []}}
    a = f"{receipt['commanded_a_peak_m_s2']:g}"
    F = f"{receipt['grip_force_n']:g}"
    key = f"a{a}_F{F}"
    band["realized_accel_by_commanded"][a] = receipt["axis_map_realized_accel_m_s2"]
    band["label_matrix"].setdefault(a, {})[F] = receipt["label"]
    band["cells"][key] = {
        "label": receipt["label"], "realized_accel_m_s2": receipt["realized_accel_m_s2"],
        "realized_source": receipt["realized_source"],
        "hold_slip_z_mm": receipt["hold_slip_z_mm"],
        "transport_slip_xz_mm": receipt["transport_slip_xz_mm"],
        "transport_x_res_mm": receipt["transport_x_res_mm"],
        "transport_z_res_mm": receipt["transport_z_res_mm"],
        "transport_y_res_mm": receipt["transport_y_res_mm"],
        "grasp_frame_y_res_mm": receipt["grasp_frame_y_res_mm"],
        "assembly_drift_mm": receipt["assembly_drift_mm"],
        "escape_mode": receipt["escape_mode"],
        "slip_perm_tangential_mm": receipt["slip_perm_tangential_mm"],
        "x_res_mm": receipt["x_res_mm"], "z_res_mm": receipt["z_res_mm"],
        "y_res_mm": receipt["y_res_mm"], "slip_perm_x_mm": receipt["slip_perm_x_mm"],
        "slip3d_max_mm": receipt["slip3d_max_mm"],
        "yz_residual_mm": receipt["yz_residual_mm"],
        "dvf": receipt["dvf"], "dvf_wholetrial": receipt["dvf_wholetrial"],
        "damage_window": receipt["damage_window"], "lift_end_s": receipt["lift_end_s"],
        "damage_latch_t": receipt["damage_latch_t"],
        "drop_t": receipt["drop_t"], "damage_after_drop": receipt["damage_after_drop"],
        "ejected": receipt["ejected"],
        "finite": receipt["health"].get("finite"), "certified": (
            receipt.get("validity_gate") or {}).get("certified"),
    }
    pair = [float(a), float(F)]
    # Robust to a band reloaded with a different/legacy coverage schema.
    cov = band.setdefault("coverage", {})
    cov.setdefault("completed", [])
    cov.setdefault("failed", [])
    target = "failed" if receipt["status"] == "error" else "completed"
    other = "completed" if target == "failed" else "failed"
    band["coverage"][other] = [item for item in band["coverage"][other] if item != pair]
    if pair not in band["coverage"][target]:
        band["coverage"][target].append(pair)
    return band


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


def realized_from_fits(fits, min_samples=5):
    """Median magnitude of completed (sufficiently sampled) plateau slopes."""
    values = [abs(item["a_fit"]) for item in fits.values()
              if item["n_samples"] >= min_samples and np.isfinite(item["a_fit"])]
    return float(np.median(values)) if values else None


def gross_slip_mm(frame, reference):
    """Current block-to-palm relative displacement from a 3-vector reference."""
    relative = np.asarray(frame["com"], dtype=float) - np.asarray(frame["palm_pos"], dtype=float)
    reference = np.asarray(reference, dtype=float)
    if relative.shape != (3,) or reference.shape != (3,):
        raise ValueError("slip inputs must be 3-vectors")
    return float(np.linalg.norm(relative - reference) * 1000.0)


def summarize_level(commanded_a_peak, cells):
    """Reduce three seed receipts into the repeatability-gate level row."""
    usable_cells = [cell for cell in cells if cell.get("status", "ok") == "ok"
                    and cell.get("tracking") is not None]
    realized_values = [realized_from_fits(cell["tracking"]["plateaus"])
                       for cell in usable_cells]
    realized = np.asarray([value for value in realized_values if value is not None], dtype=float)
    cell_flags = [{"seed": cell.get("seed"), "status": cell.get("status", "ok"),
                   "ejected": cell.get("ejected"), "finite": cell.get("health", {}).get("finite"),
                   "realized_accel": (realized_from_fits(cell["tracking"]["plateaus"])
                                      if cell.get("tracking") else None)}
                  for cell in cells]
    sufficient = realized.size >= 2
    if not sufficient:
        return {"commanded_a_peak": float(commanded_a_peak),
                "realized_per_cell": realized.tolist(), "realized_median": None,
                "realized_cv": None, "r2_min": None, "status": "insufficient",
                "cells": cell_flags, "level_pass": False}
    mean = float(np.mean(realized))
    median = float(np.median(realized))
    cv = float(np.std(realized) / mean) if mean > 0.0 else float("inf")
    r2_values = [
        fit["r2"] for cell in usable_cells
        for fit in cell["tracking"]["plateaus"].values()
        if fit["n_samples"] >= 5 and np.isfinite(fit["r2"])
    ]
    r2_min = float(min(r2_values)) if r2_values else None
    return {
        "commanded_a_peak": float(commanded_a_peak),
        "realized_per_cell": realized.tolist(),
        "realized_median": median,
        "realized_cv": cv,
        "r2_min": r2_min,
        "status": "ok",
        "cells": cell_flags,
        "level_pass": bool(cv <= 0.05 and r2_min is not None and r2_min >= 0.99),
    }


def ladder_shape_gate(levels):
    """Evaluate strict monotonicity and spread-aware adjacent separation."""
    if any(level["realized_median"] is None for level in levels):
        return False, False
    medians = [float(level["realized_median"]) for level in levels]
    monotone = all(right > left for left, right in zip(medians, medians[1:]))
    separated = True
    for left, right in zip(levels, levels[1:]):
        left_values = np.asarray(left["realized_per_cell"], dtype=float)
        right_values = np.asarray(right["realized_per_cell"], dtype=float)
        left_spread = float(left_values.max() - left_values.min())
        right_spread = float(right_values.max() - right_values.min())
        required = max(0.1, 2.0 * max(left_spread, right_spread))
        separated &= float(right["realized_median"]) - float(left["realized_median"]) > required
    return bool(monotone), bool(separated)


def run_cell_isolated(E, F, a_peak, seed, **kwargs):
    """Run one cell without allowing a simulation failure to abort a batch."""
    try:
        return run_transport_cell(E, F, a_peak, seed, **kwargs)
    except Exception as exc:
        return {"status": "error", "E_pa": float(E), "grip_force_n": float(F),
                "a_peak": float(a_peak), "commanded_a_peak_m_s2": float(a_peak),
                "seed": int(seed), "ejected": False, "finite": False,
                "msg": f"{type(exc).__name__}: {exc}"}


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
    path.write_text(json.dumps(_json_safe(receipt), indent=2, allow_nan=False) + "\n")
    return path


def _json_safe(value):
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, (float, np.floating)) and not np.isfinite(value):
        return None
    return value


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
    post_lift_temporal_max = None
    rest_vol = None
    snap_index = 0
    transport_reference = None
    ejected = False
    finite = True
    grip_relative_z = None
    grip_relative_finger_y = None
    damage_latch_t = None
    drop_t = None
    t_pre = cfg.ramp_s + cfg.preload_s
    for frame_index in range(round(T_END * FPS)):
        t = rig.sim_time
        cf = F * min(1.0, t / cfg.ramp_s)
        lift_fraction = min(1.0, max(0.0, t - t_pre) / cfg.lift_s)
        lt = GRAB_Z + cfg.lift_height_m * lift_fraction
        active = TRANSPORT_START <= t <= T_END and profile is not None
        rig.step(cf, lt, x_target=profile.x_cmd(t) if active else 0.0,
                 x_vel=profile.v_cmd(t) if active else 0.0)
        validity.fold_frame()
        m = rig.metrics()
        m["phase"] = phase_for_time(m["t"])
        m["contacts"] = rig.contact_count()
        if not m["finite"] or not np.all(np.isfinite(m["com"])) or not np.all(np.isfinite(m["palm_pos"])):
            finite = False
            series.append(m)
            break
        finger_mid_y = 0.5 * (m["left_y"] + m["right_y"])
        m["finger_mid_y"] = finger_mid_y
        if grip_relative_z is None and abs(m["t"] - 1.80) <= 0.5 / FPS:
            grip_relative_z = m["com"][2] - m["palm_pos"][2]
            grip_relative_finger_y = m["com"][1] - finger_mid_y
        if grip_relative_z is not None and drop_t is None:
            hold_drop = (4.30 <= m["t"] <= 9.30
                         and abs((m["com"][2] - m["palm_pos"][2])
                                 - grip_relative_z) * 1000.0 > 2.0)
            lateral_drop = (abs((m["com"][1] - finger_mid_y)
                                - grip_relative_finger_y) * 1000.0 > 10.0)
            if hold_drop or lateral_drop:
                drop_t = float(m["t"])
        if m["t"] >= TRANSPORT_START:
            relative = np.asarray(m["com"]) - np.asarray(m["palm_pos"])
            if transport_reference is None:
                transport_reference = relative
            if gross_slip_mm(m, transport_reference) > GROSS_SLIP_MM:
                series.append(m)
                ejected = True
                if drop_t is None:
                    drop_t = float(m["t"])
                break
        field, rest_vol = rig.strain_field()
        temporal_max = field.copy() if temporal_max is None else np.maximum(temporal_max, field)
        if m["t"] >= LIFT_END:
            post_lift_temporal_max = (
                field.copy() if post_lift_temporal_max is None
                else np.maximum(post_lift_temporal_max, field)
            )
        if damage_latch_t is None and post_lift_temporal_max is not None:
            current_dvf, current_latched = latched_dvf(post_lift_temporal_max, rest_vol)
            if current_latched:
                damage_latch_t = float(m["t"])
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
                            post_lift_temporal_max_principal_strain=post_lift_temporal_max,
                            final_principal_strain=final_field, tet_rest_vol=rest_vol)
    transport = [m for m in series if m["t"] >= TRANSPORT_START]
    times = [m["t"] for m in transport]
    velocities = [m["palm_vx"] for m in transport]
    if profile is not None:
        tracking = tracking_receipt(times, velocities, profile.plateau_windows)
        tracking["realized_accel_m_s2"] = realized_from_fits(tracking["plateaus"])
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
    if post_lift_temporal_max is None:
        raise RuntimeError("post-lift damage window contains no strain field")
    dvf, damaged = latched_dvf(post_lift_temporal_max, rest_vol)
    dvf_wholetrial, _ = latched_dvf(temporal_max, rest_vol)
    finite_series = [m for m in series if m["finite"]
                     and np.all(np.isfinite(m["com"])) and np.all(np.isfinite(m["palm_pos"]))]
    slip_max = slip3d_max_mm(finite_series, TRANSPORT_START) if transport_reference is not None else 0.0
    settled = [m for m in finite_series if 11.30 <= m["t"] <= 11.60]
    grip_frame = min(finite_series, key=lambda m: abs(m["t"] - 1.80))
    if abs(grip_frame["t"] - 1.80) > 0.0500001:
        raise ValueError("series contains no frame near t_grip=1.80")
    grip_ref_com = [float(value) for value in grip_frame["com"]]
    grip_ref_palm = [float(value) for value in grip_frame["palm_pos"]]
    hold_slip = hold_slip_z_mm(finite_series)
    legacy_slip = hold_slip
    if transport_reference is not None:
        ref_frame = min(finite_series, key=lambda m: abs(m["t"] - TRANSPORT_START))
        transport_ref_com = [float(value) for value in ref_frame["com"]]
        transport_ref_palm = [float(value) for value in ref_frame["palm_pos"]]
    else:
        ref_frame = None
        transport_ref_com = transport_ref_palm = None
    if not ejected and finite and settled:
        transport_slip = transport_slip_xz_mm(finite_series)
        transport_x_res = transport_x_res_mm(finite_series)
        transport_y_res = transport_y_res_mm(finite_series)
        transport_z_res = transport_z_res_mm(finite_series)
        slip_tangential = slip_perm_tangential_mm(finite_series)
        x_res = x_res_mm(finite_series)
        y_res = y_res_mm(finite_series)
        z_res = z_res_mm(finite_series)
        grasp_y_res = grasp_frame_y_res_mm(finite_series)
        slip_perm = slip_perm_x_mm(finite_series)
        yz_residual = yz_residual_mm(finite_series)
        settle_end_com = np.mean([m["com"] for m in settled], axis=0).astype(float).tolist()
        settle_end_palm = np.mean([m["palm_pos"] for m in settled], axis=0).astype(float).tolist()
        settle_end_block_x = float(np.mean([m["com"][0] for m in settled]))
        settle_end_palm_x = float(np.mean([m["palm_pos"][0] for m in settled]))
        ref_block_x = float(ref_frame["com"][0])
        ref_palm_x = float(ref_frame["palm_pos"][0])
    else:
        grasp_y_res = None
        transport_slip = transport_x_res = transport_y_res = transport_z_res = None
        slip_tangential = x_res = y_res = z_res = None
        slip_perm = yz_residual = None
        settle_end_com = settle_end_palm = None
        settle_end_block_x = settle_end_palm_x = None
        if ref_frame is not None:
            ref_block_x, ref_palm_x = float(ref_frame["com"][0]), float(ref_frame["palm_pos"][0])
        else:
            ref_block_x = ref_palm_x = None
    strain_maxima = (per_phase_strain_maxima(phase_fields)
                      if set(phase_fields) == set(PHASE_NAMES)
                      else {name: (float(np.max(phase_fields[name])) if name in phase_fields else None)
                            for name in PHASE_NAMES})
    vg = validity.summary()
    certified = all(p["certified"] for p in vg["per_pad"].values())
    stats = rig.strain_stats(0.15) if finite else {"p99_vol_weighted_strain": None}
    finite = finite and all(m["finite"] for m in series)
    fn_hold = [0.5 * (m["fn_left_n"] + m["fn_right_n"]) for m in hold]
    escape_mode = "lateral" if grasp_y_res is not None and grasp_y_res > 10.0 else None
    if escape_mode is not None and drop_t is None:
        # Normally captured incrementally; retain fail-closed timing evidence if
        # only the settle-window mean crosses the criterion.
        drop_t = 11.30
    damage_after_drop = bool(
        damage_latch_t is not None and drop_t is not None
        and damage_latch_t >= drop_t
    )
    classification = {
        "damage_latch_t": damage_latch_t, "drop_t": drop_t,
        "dropped": drop_t is not None, "ejected": ejected,
        "hold_slip_z_mm": hold_slip,
        "transport_slip_xz_mm": transport_slip,
        "grasp_frame_y_res_mm": grasp_y_res,
    }
    receipt = {
        "status": "ok", "E_pa": float(E), "grip_force_n": float(F), "seed": int(seed),
        "ejected": ejected, "finite": finite,
        "commanded_a_peak_m_s2": float(a_peak), "tracking": tracking,
        "realized_F_g_n": float(np.mean(fn_hold)) if fn_hold else float("nan"),
        "hold_slip_z_mm": hold_slip, "transport_slip_xz_mm": transport_slip,
        "transport_x_res_mm": transport_x_res, "transport_z_res_mm": transport_z_res,
        "transport_y_res_mm": transport_y_res,
        "grasp_frame_y_res_mm": grasp_y_res,
        "assembly_drift_mm": y_res,
        "escape_mode": escape_mode,
        "slip_perm_tangential_mm": slip_tangential,
        "x_res_mm": x_res, "z_res_mm": z_res, "y_res_mm": y_res,
        "slip_perm_x_mm": slip_perm, "slip3d_max_mm": slip_max,
        "yz_residual_mm": yz_residual, "legacy_hold_slip_mm": legacy_slip,
        "grip_ref_com": grip_ref_com, "grip_ref_palm": grip_ref_palm,
        "transport_ref_com": transport_ref_com, "transport_ref_palm": transport_ref_palm,
        "settle_end_com": settle_end_com, "settle_end_palm": settle_end_palm,
        "settle_end_block_x": settle_end_block_x, "settle_end_palm_x": settle_end_palm_x,
        "ref_block_x": ref_block_x, "ref_palm_x": ref_palm_x,
        "dvf": dvf, "dvf_wholetrial": dvf_wholetrial,
        "damage_window": "grip_completion_preload_end", "lift_end_s": LIFT_END,
        "damage_latched": damaged,
        "damage_latch_t": damage_latch_t, "drop_t": drop_t,
        "damage_after_drop": damage_after_drop,
        "label": ("nonfinite" if not finite else label_v23(classification)),
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


def run_screen(material=None, resume=False):
    axis_path = ROOT / "reports/logs/vbd/g_trk_axis.json"
    if not axis_path.exists():
        print(f"STOP: missing passed G-TRK axis map: {axis_path}")
        return 1
    try:
        axis_map = load_axis_map(axis_path)
    except (OSError, KeyError, ValueError, RuntimeError) as exc:
        print(f"STOP: invalid G-TRK axis map: {exc}")
        return 1
    receipt_dir = ROOT / "reports/logs/vbd/w1_screen"
    receipt_dir.mkdir(parents=True, exist_ok=True)
    grid = screen_grid(material)
    pending = pending_screen_cells(grid, receipt_dir, resume)
    done = len(grid) - len(pending)
    started = time.monotonic()
    for E_kpa, acceleration, force, seed in pending:
        raw = run_cell_isolated(E_kpa * 1000, force, acceleration, seed)
        receipt = screen_cell_receipt(raw, axis_map)
        receipt_path = receipt_dir / screen_receipt_name(E_kpa, acceleration, force, seed)
        receipt_path.write_text(json.dumps(_json_safe(receipt), indent=2, allow_nan=False) + "\n")
        band_path = ROOT / f"reports/logs/vbd/e1v2_band_{E_kpa}.json"
        band = json.loads(band_path.read_text()) if band_path.exists() else None
        band = update_band(band, receipt)
        band_path.write_text(json.dumps(_json_safe(band), indent=2, allow_nan=False) + "\n")
        done += 1
        elapsed = time.monotonic() - started
        cert = (receipt.get("validity_gate") or {}).get("certified")
        realized = receipt["realized_accel_m_s2"]
        print(f"E{E_kpa} a{acceleration:g} F{force:g} -> {receipt['label']} "
              f"realized={realized:.4g} slip={max(receipt['hold_slip_z_mm'] or 0, receipt['transport_slip_xz_mm'] or 0)} dvf={receipt['dvf']} "
              f"ejected={receipt['ejected']} cert={cert} ({done}cells done, {elapsed:.1f}s)")
    return 0


def run_confirm(resume=False):
    axis_path = ROOT / "reports/logs/vbd/g_trk_axis.json"
    plan_path = ROOT / "reports/logs/vbd/w1_confirm_list.json"
    if not axis_path.exists():
        print(f"STOP: missing passed G-TRK axis map: {axis_path}")
        return 1
    if not plan_path.exists():
        print(f"STOP: missing confirmation plan: {plan_path}")
        return 1
    try:
        axis_map = load_axis_map(axis_path)
        plan = json.loads(plan_path.read_text())
        if isinstance(plan, dict):
            plan = plan.get("cells", [])  # w1_confirm_list.json is {schema, coverage, cells:[...]}
        if not isinstance(plan, list):
            raise ValueError("confirmation plan must be a list")
        grid = expand_confirm_plan(plan)
    except (OSError, KeyError, TypeError, ValueError, RuntimeError) as exc:
        print(f"STOP: invalid confirmation input: {exc}")
        return 1
    receipt_dir = ROOT / "reports/logs/vbd/w1_screen"
    receipt_dir.mkdir(parents=True, exist_ok=True)
    pending = pending_screen_cells(grid, receipt_dir, resume)
    done = len(grid) - len(pending)
    total = len(grid)
    started = time.monotonic()
    for E_kpa, acceleration, force, seed in pending:
        raw = run_cell_isolated(E_kpa * 1000, force, acceleration, seed)
        receipt = screen_cell_receipt(raw, axis_map)
        path = receipt_dir / screen_receipt_name(E_kpa, acceleration, force, seed)
        path.write_text(json.dumps(_json_safe(receipt), indent=2, allow_nan=False) + "\n")
        done += 1
        elapsed = time.monotonic() - started
        print(f"CONFIRM E{E_kpa} a{acceleration:g} F{force:g} s{seed} -> "
              f"{receipt['label']} realized={receipt['realized_accel_m_s2']:.4g} "
              f"({done}/{total} done, {elapsed:.1f}s)")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--smoke", action="store_true")
    mode.add_argument("--tracking-ladder", action="store_true")
    mode.add_argument("--cell", nargs=4, metavar=("E_KPA", "F_N", "A", "SEED"))
    mode.add_argument("--screen", action="store_true")
    mode.add_argument("--confirm", action="store_true")
    parser.add_argument("--noise-floor", action="store_true")
    parser.add_argument("--material", type=int, choices=(7, 15, 25))
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args(argv)
    if args.confirm:
        return run_confirm(args.resume)
    if args.screen:
        return run_screen(args.material, args.resume)
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
        fits = receipt["tracking"]["plateaus"].values()
        shape_ok = all(fit["n_samples"] >= 5 and np.isfinite(fit["r2"])
                       and fit["r2"] >= 0.99 for fit in fits)
        return 0 if timestamps_ok and shape_ok else 1
    ladder = []
    for acceleration in (1, 2.5, 5, 10, 20, 30):
        cells = []
        for seed in (0, 1, 2):
            receipt = run_cell_isolated(15e3, 1.2, acceleration, seed)
            _write_receipt(receipt)
            cells.append(receipt)
        ladder.append(summarize_level(acceleration, cells))
    monotone, separated = ladder_shape_gate(ladder)
    level_10 = next(level for level in ladder if level["commanded_a_peak"] == 10.0)
    reference = level_10["realized_median"]
    spot_cells = []
    for E, F in ((7e3, 0.6), (7e3, 1.2), (25e3, 1.2)):
        receipt = run_cell_isolated(E, F, 10.0, 0)
        _write_receipt(receipt)
        realized = (realized_from_fits(receipt["tracking"]["plateaus"])
                    if receipt.get("tracking") else None)
        within = bool(realized is not None and reference is not None and reference > 0
                      and abs(realized - reference) / reference <= 0.05)
        spot_cells.append({"E_pa": E, "grip_force_n": F, "seed": 0,
                           "status": receipt.get("status", "ok"),
                           "ejected": receipt.get("ejected"),
                           "finite": receipt.get("health", {}).get("finite"),
                           "realized_accel": realized, "within_5pct": within})
    independence = all(cell["within_5pct"] for cell in spot_cells)
    noise_receipt = run_cell_isolated(15e3, 1.2, 0.0, 0)
    _write_receipt(noise_receipt)
    noise = noise_receipt.get("health", {}).get("zero_command_realized_accel_magnitude")
    noise_pass = bool(noise is not None and np.isfinite(noise) and noise <= 0.01)
    overall = bool(all(level["level_pass"] for level in ladder)
                   and monotone and separated and noise_pass and independence)
    output = {
        "levels": ladder,
        "spot_check": {"reference_realized_median": reference, "cells": spot_cells,
                       "independence_pass": independence},
        "monotone_pass": monotone,
        "separated_pass": separated,
        "noise_floor_m_s2": noise,
        "noise_floor_pass": noise_pass,
        "overall_pass": overall,
        "provenance": {
            "git_sha": subprocess.check_output(
                ["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True
            ).strip(),
            "prereg_sha256": _sha256(ROOT / "ralph/results/prereg_w1.json"),
        },
    }
    out = ROOT / "reports/logs/vbd/g_trk_ladder.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    output = _json_safe(output)
    out.write_text(json.dumps(output, indent=2, allow_nan=False) + "\n")
    print(json.dumps(output, indent=2))
    print("PASS" if overall else "STOP")
    return 0 if overall else 1


if __name__ == "__main__":
    raise SystemExit(main())
