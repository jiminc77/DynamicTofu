"""CPU-only reducers for the pre-registered VBD judgment protocol."""

from __future__ import annotations

from collections.abc import Mapping, Sequence

import numpy as np


SUBSTEPS_PER_SECOND = 4800
SLIP_START_K = 44640
SLIP_END_K = 55680
SLIP_THRESHOLD_MM = 2.0
DAMAGE_DVF_THRESHOLD = 0.005

PHASE_WINDOWS = (
    ("ramp", 0, 3840, False),
    ("preload", 3840, 8640, False),
    ("lift", 8640, 20640, False),
    ("hold", 20640, 44640, False),
    ("accel_out", 44640, 45600, False),
    ("cruise_out", 45600, 46080, False),
    ("decel_out", 46080, 47040, False),
    ("dwell", 47040, 48480, False),
    ("accel_back", 48480, 49440, False),
    ("cruise_back", 49440, 49920, False),
    ("decel_back", 49920, 50880, False),
    ("settle", 50880, 55680, True),
)
PHASE_NAMES = tuple(window[0] for window in PHASE_WINDOWS)


def substep_index(t: float) -> int:
    """Convert an absolute time to its protocol integer substep index."""
    value = float(t)
    if not np.isfinite(value):
        raise ValueError("time must be finite")
    return int(round(value * SUBSTEPS_PER_SECOND))


def phase_for_time(t: float) -> str | None:
    """Return phase membership using the exact half-open integer partition."""
    k = substep_index(t)
    for name, start, end, closed_end in PHASE_WINDOWS:
        if start <= k < end or (closed_end and k == end):
            return name
    return None


def slip3d_max_mm(series: Sequence[Mapping], t_ref: float = 9.30) -> float:
    """Maximum block-to-palm relative displacement in the transport window, mm."""
    ref_k = substep_index(t_ref)
    reference = None
    displacements = []
    for frame in series:
        k = substep_index(frame["t"])
        relative = np.asarray(frame["com"], dtype=float) - np.asarray(
            frame["palm_pos"], dtype=float
        )
        if relative.shape != (3,) or not np.all(np.isfinite(relative)):
            raise ValueError("frame com and palm_pos must be finite 3-vectors")
        if k == ref_k:
            if reference is not None:
                raise ValueError("series contains more than one reference frame")
            reference = relative
        if SLIP_START_K <= k <= SLIP_END_K:
            displacements.append((k, relative))
    if reference is None:
        raise ValueError("series must contain a frame at t_ref")
    if not displacements:
        raise ValueError("series contains no frames in the judgment window")
    return float(max(np.linalg.norm(relative - reference) for _, relative in displacements) * 1000.0)


def _permanent_residual(series, t_ref, settle_lo, settle_hi):
    frames = list(series)
    if not frames:
        raise ValueError("series must not be empty")
    reference_frame = min(frames, key=lambda frame: abs(float(frame["t"]) - t_ref))
    reference = np.asarray(reference_frame["com"], dtype=float) - np.asarray(
        reference_frame["palm_pos"], dtype=float
    )
    settled = [
        np.asarray(frame["com"], dtype=float) - np.asarray(frame["palm_pos"], dtype=float)
        for frame in frames if settle_lo <= float(frame["t"]) <= settle_hi
    ]
    if reference.shape != (3,) or not np.all(np.isfinite(reference)):
        raise ValueError("reference com and palm_pos must be finite 3-vectors")
    if not settled or any(value.shape != (3,) or not np.all(np.isfinite(value))
                          for value in settled):
        raise ValueError("settle window must contain finite 3-vector frames")
    return np.mean(settled, axis=0) - reference


def slip_perm_x_mm(series: Sequence[Mapping], t_ref: float = 9.30,
                   settle_lo: float = 11.30, settle_hi: float = 11.60) -> float:
    """Permanent transport-axis residual after settling, in millimetres."""
    return float(abs(_permanent_residual(series, t_ref, settle_lo, settle_hi)[0]) * 1000.0)


def yz_residual_mm(series: Sequence[Mapping], t_ref: float = 9.30,
                   settle_lo: float = 11.30, settle_hi: float = 11.60) -> float:
    """Permanent non-transport-axis residual norm after settling, in millimetres."""
    return float(np.linalg.norm(_permanent_residual(series, t_ref, settle_lo, settle_hi)[1:]) * 1000.0)


# Compatibility name for the recorded maximum-3D observable.
slip3d = slip3d_max_mm


def is_slip(series: Sequence[Mapping], t_ref: float = 9.30) -> bool:
    """Return whether transport relative displacement strictly exceeds 2 mm."""
    return slip3d(series, t_ref=t_ref) > SLIP_THRESHOLD_MM


def latched_dvf(temporal_max_field, rest_vol, eps: float = 0.15) -> tuple[float, bool]:
    """Return volume-weighted damaged-volume fraction and its whole-trial latch."""
    strain = np.asarray(temporal_max_field, dtype=float).reshape(-1)
    volume = np.asarray(rest_vol, dtype=float).reshape(-1)
    if strain.size == 0 or strain.shape != volume.shape:
        raise ValueError("strain and rest_vol must be non-empty and have equal shape")
    if not np.all(np.isfinite(strain)) or not np.all(np.isfinite(volume)):
        raise ValueError("strain and rest_vol must be finite")
    if np.any(volume < 0.0) or float(volume.sum()) <= 0.0:
        raise ValueError("rest_vol must be non-negative with positive total volume")
    dvf = float(volume[strain > eps].sum() / volume.sum())
    return dvf, dvf >= DAMAGE_DVF_THRESHOLD


def per_phase_strain_maxima(field_by_phase: Mapping[str, object]) -> dict[str, float]:
    """Reduce principal-strain fields to maxima in protocol phase order."""
    missing = set(PHASE_NAMES) - set(field_by_phase)
    extra = set(field_by_phase) - set(PHASE_NAMES)
    if missing or extra:
        raise ValueError(f"phase keys mismatch; missing={sorted(missing)}, extra={sorted(extra)}")
    maxima = {}
    for phase in PHASE_NAMES:
        field = np.asarray(field_by_phase[phase], dtype=float)
        if field.size == 0 or not np.all(np.isfinite(field)):
            raise ValueError(f"phase {phase!r} must contain finite strain values")
        maxima[phase] = float(np.max(field))
    return maxima


def label(cell: Mapping[str, object]) -> str:
    """Classify a cell with pre-registered causal damage/slip precedence."""
    damage = bool(
        cell.get(
            "damage_latched",
            cell.get("latched", float(cell.get("dvf", 0.0)) >= DAMAGE_DVF_THRESHOLD),
        )
    )
    slip_mm = float(cell.get("slip3d_mm", cell.get("slip3d", 0.0)))
    damage_latch_t = cell.get("damage_latch_t")
    drop_t = cell.get("drop_t")

    damage_precedes_drop = damage and (
        drop_t is None
        or (damage_latch_t is not None and float(damage_latch_t) < float(drop_t))
    )
    if damage_precedes_drop:
        return "damage"
    if slip_mm > SLIP_THRESHOLD_MM:
        return "slip"
    return "intact"


def label_v21(cell: Mapping[str, object]) -> str:
    """Classify by permanent x slip, with drop/ejection taking precedence."""
    if (bool(cell.get("dropped", cell.get("ejected", False)))
            or cell.get("drop_t") is not None):
        return "slip"
    permanent = cell.get("slip_perm_x_mm")
    if permanent is not None and float(permanent) > SLIP_THRESHOLD_MM:
        return "slip"
    damage = bool(cell.get(
        "damage_latched",
        cell.get("latched", float(cell.get("dvf", 0.0)) >= DAMAGE_DVF_THRESHOLD),
    ))
    return "damage" if damage else "intact"
