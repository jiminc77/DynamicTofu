"""CPU-side reduction and durable storage for VBD tactile observations.

Force resultants and contact geometry are deliberately separate channels.  In
particular, contact centroids are geometric (unweighted) centroids.
"""

from __future__ import annotations

from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

RAW_SCHEMA = "e2v2_tactile_raw.v1"
PAD_COUNT = 2
_SUMMARY_KEYS = (
    "normal_resultant_n", "tangential_resultant_n", "lr_normal_asymmetry_n",
    "contact_count", "contact_centroid_pad_m", "contact_extent_m",
    "peak_tangential_normal_ratio", "peak_tangential_normal_ratio_t_s",
    "centroid_excursion_m", "centroid_excursion_t_s",
    "peak_lr_asymmetry_n", "peak_lr_asymmetry_t_s",
)


def decompose_pad_force(force: np.ndarray, outward_normal: np.ndarray) -> tuple[float, float]:
    """Return signed outward-normal projection and in-plane magnitude."""
    force = np.asarray(force, dtype=np.float64)
    normal = np.asarray(outward_normal, dtype=np.float64)
    if force.shape != (3,) or normal.shape != (3,):
        raise ValueError("force and outward_normal must each have shape (3,)")
    norm = float(np.linalg.norm(normal))
    if not np.isfinite(norm) or norm == 0.0:
        raise ValueError("outward_normal must be finite and nonzero")
    unit = normal / norm
    normal_resultant = float(np.dot(force, unit))
    tangential_resultant = float(np.linalg.norm(force - normal_resultant * unit))
    return normal_resultant, tangential_resultant


def geometry_for_pad(positions_pad: np.ndarray) -> tuple[int, np.ndarray, float]:
    """Return count, unweighted centroid, and maximum pairwise extent."""
    positions = np.asarray(positions_pad, dtype=np.float64)
    if positions.size == 0:
        return 0, np.full(3, np.nan, dtype=np.float64), 0.0
    if positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError("contact positions must have shape (n, 3)")
    centroid = positions.mean(axis=0)
    if len(positions) < 2:
        extent = 0.0
    else:
        delta = positions[:, None, :] - positions[None, :, :]
        extent = float(np.sqrt(np.max(np.sum(delta * delta, axis=2))))
    return len(positions), centroid, extent


def _raw_arrays(raw_frames: Sequence[Mapping]) -> dict[str, np.ndarray]:
    n = len(raw_frames)
    times = np.empty(n, np.float64)
    forces = np.zeros((n, PAD_COUNT, 3), np.float64)
    normals = np.zeros((n, PAD_COUNT, 3), np.float64)
    force_available = np.zeros(n, np.bool_)
    positions_pad: list[np.ndarray] = []
    positions_world: list[np.ndarray] = []
    pad_ids: list[np.ndarray] = []
    offsets = [0]
    for i, frame in enumerate(raw_frames):
        times[i] = frame["t_s"]
        available = bool(frame.get("force_channel_available", "pad_force_vectors" in frame))
        force_available[i] = available
        if available:
            forces[i] = np.asarray(frame["pad_force_vectors"], np.float64)
            normals[i] = np.asarray(frame["pad_outward_normals"], np.float64)
            if forces[i].shape != (PAD_COUNT, 3) or normals[i].shape != (PAD_COUNT, 3):
                raise ValueError("pad force vectors and normals must have shape (2, 3)")
        for side in range(PAD_COUNT):
            per_pad = frame.get("contact_positions_pad", ((), ()))[side]
            pos = np.asarray(per_pad, np.float64).reshape((-1, 3))
            positions_pad.append(pos)
            world_by_pad = frame.get("contact_positions_world")
            world = pos if world_by_pad is None else np.asarray(world_by_pad[side], np.float64).reshape((-1, 3))
            if len(world) != len(pos):
                raise ValueError("world and pad contact-position counts differ")
            positions_world.append(world)
            pad_ids.append(np.full(len(pos), side, np.int8))
        offsets.append(offsets[-1] + sum(len(x) for x in positions_pad[-2:]))
    return {
        "schema": np.asarray(RAW_SCHEMA), "sample_t_s": times,
        "pad_force_vectors": forces, "pad_outward_normals": normals,
        "force_channel_available": force_available,
        "contact_pos_pad": np.concatenate(positions_pad) if positions_pad else np.empty((0, 3), np.float64),
        "contact_pos_world": np.concatenate(positions_world) if positions_world else np.empty((0, 3), np.float64),
        "contact_pad_id": np.concatenate(pad_ids) if pad_ids else np.empty(0, np.int8),
        "sample_offsets": np.asarray(offsets, np.int64),
    }


def _compute(raw: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
    times = np.asarray(raw["sample_t_s"], np.float64)
    forces = np.asarray(raw["pad_force_vectors"], np.float64)
    normals = np.asarray(raw["pad_outward_normals"], np.float64)
    available = np.asarray(raw["force_channel_available"], np.bool_)
    offsets = np.asarray(raw["sample_offsets"], np.int64)
    pos = np.asarray(raw["contact_pos_pad"], np.float64)
    ids = np.asarray(raw["contact_pad_id"], np.int8)
    n = len(times)
    normal = np.zeros((n, 2), np.float64)
    tangential = np.zeros((n, 2), np.float64)
    counts = np.zeros((n, 2), np.int64)
    centroids = np.full((n, 2, 3), np.nan, np.float64)
    extents = np.zeros((n, 2), np.float64)
    for sample in range(n):
        if available[sample]:
            for side in range(2):
                normal[sample, side], tangential[sample, side] = decompose_pad_force(forces[sample, side], normals[sample, side])
        lo, hi = offsets[sample:sample + 2]
        for side in range(2):
            selected = pos[lo:hi][ids[lo:hi] == side]
            counts[sample, side], centroids[sample, side], extents[sample, side] = geometry_for_pad(selected)
    asymmetry = normal[:, 0] - normal[:, 1]
    denominator = np.abs(normal).sum(axis=1)
    numerator = tangential.sum(axis=1)
    ratios = np.full(n, np.nan, np.float64)
    valid_ratio = available & (denominator > 0.0)
    ratios[valid_ratio] = numerator[valid_ratio] / denominator[valid_ratio]

    excursion = np.full((n, 2), np.nan, np.float64)
    for side in range(2):
        valid = np.flatnonzero(counts[:, side] > 0)
        if len(valid):
            excursion[valid, side] = np.linalg.norm(centroids[valid, side] - centroids[valid[0], side], axis=1)

    def peak(values: np.ndarray, absolute: bool = False) -> tuple[np.float64, np.float64]:
        if values.size == 0 or np.all(np.isnan(values)):
            return np.float64(np.nan), np.float64(np.nan)
        flat_index = int(np.nanargmax(np.abs(values) if absolute else values))
        index = np.unravel_index(flat_index, values.shape)
        return np.float64(values[index]), np.float64(times[index[0]])

    ratio_peak, ratio_t = peak(ratios)
    excursion_peak, excursion_t = peak(excursion)
    asym_signed, asym_t = peak(asymmetry, absolute=True)
    return {
        "normal_resultant_n": normal, "tangential_resultant_n": tangential,
        "lr_normal_asymmetry_n": asymmetry, "contact_count": counts,
        "contact_centroid_pad_m": centroids, "contact_extent_m": extents,
        "peak_tangential_normal_ratio": ratio_peak,
        "peak_tangential_normal_ratio_t_s": ratio_t,
        "centroid_excursion_m": excursion_peak, "centroid_excursion_t_s": excursion_t,
        "peak_lr_asymmetry_n": np.float64(abs(asym_signed)), "peak_lr_asymmetry_t_s": asym_t,
    }


def compute_aggregates_vbd(raw_frames: Sequence[Mapping]) -> dict[str, np.ndarray]:
    """Reduce frame records into per-sample channels and peak summaries."""
    return _compute(_raw_arrays(raw_frames))


def write_raw_vbd(raw_frames: Sequence[Mapping], E, a, seed: int,
                  root: str | Path = "reports/logs/vbd/e2v2_raw") -> Path:
    """Write the versioned raw schema and its reproducible stored summary."""
    raw = _raw_arrays(raw_frames)
    summary = _compute(raw)
    directory = Path(root)
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"e2v2_{E}_a{a}_s{seed}.npz"
    np.savez(path, **raw, **{f"summary_{key}": value for key, value in summary.items()})
    return path


def recompute_aggregates_vbd(npz_path: str | Path | Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
    """Recompute aggregates solely from durable raw fields in an NPZ."""
    if isinstance(npz_path, (str, Path)):
        with np.load(npz_path, allow_pickle=False) as archive:
            if str(archive["schema"]) != RAW_SCHEMA:
                raise ValueError("unsupported tactile raw schema")
            return _compute(archive)
    if str(npz_path["schema"]) != RAW_SCHEMA:
        raise ValueError("unsupported tactile raw schema")
    return _compute(npz_path)


def stored_aggregates_vbd(npz_path: str | Path) -> dict[str, np.ndarray]:
    """Read the stored summary, primarily for bitwise audit checks."""
    with np.load(npz_path, allow_pickle=False) as archive:
        return {key: np.asarray(archive[f"summary_{key}"]) for key in _SUMMARY_KEYS}
