"""Substep-resolution evidence for the high-acceleration contact-validity gate."""

from __future__ import annotations

from typing import Mapping, Sequence

import numpy as np

try:
    import warp as wp
except ImportError:  # CPU-only analysis and unit tests do not require Warp.
    wp = None

_PAD_NAMES = ("left", "right")


def _array(data, *names, required=True):
    for name in names:
        if name in data:
            return np.asarray(data[name])
    if required:
        raise KeyError(f"missing contact array (one of {names})")
    return None


def reduce_validity(
    contacts_arrays: Mapping[str, object],
    pad_shapes: Sequence[int],
    particle_q: np.ndarray,
    particle_qd: np.ndarray,
    sim_dt: float,
) -> dict:
    """Reduce one synthetic/host substep using the Newton soft-contact contract.

    ``soft_contact_body_vel`` is the velocity of the rigid-side contact point.
    The soft point is reconstructed from ``indices`` and ``barycentric``; its
    previous position is reconstructed as ``q - qd * sim_dt``.  Optional
    ``block_particle_range=(start, end)`` filters records to the tofu feature.
    """
    if len(pad_shapes) != 2:
        raise ValueError("exactly two pad shape ids are required")
    if not np.isfinite(sim_dt) or sim_dt <= 0.0:
        raise ValueError("sim_dt must be positive and finite")

    shapes = _array(contacts_arrays, "soft_contact_shape", "shape")
    indices = _array(contacts_arrays, "soft_contact_indices", "indices")
    bary = _array(contacts_arrays, "soft_contact_barycentric", "barycentric")
    body_vel = _array(contacts_arrays, "soft_contact_body_vel", "body_vel")
    count_value = _array(contacts_arrays, "soft_contact_count", "count")
    count = int(count_value.reshape(-1)[0])
    capacity = int(contacts_arrays.get("soft_contact_max", len(shapes)))
    usable = min(max(count, 0), capacity, len(shapes))
    q = np.asarray(particle_q, dtype=float)
    qd = np.asarray(particle_qd, dtype=float)
    if q.shape != qd.shape or q.ndim != 2 or q.shape[1] != 3:
        raise ValueError("particle_q and particle_qd must have matching (n, 3) shapes")

    particle_range = contacts_arrays.get("block_particle_range", (0, len(q)))
    start, end = map(int, particle_range)
    per_pad = {}
    cfl_max = 0.0
    for name, pad_shape in zip(_PAD_NAMES, pad_shapes):
        record_count = 0
        max_rel = 0.0
        for record in range(usable):
            if int(shapes[record]) != int(pad_shape):
                continue
            ids = np.asarray(indices[record], dtype=int)
            valid = ids >= 0
            if not np.any(valid) or np.any(ids[valid] < start) or np.any(ids[valid] >= end):
                continue
            weights = np.asarray(bary[record], dtype=float)[valid]
            feature_now = np.sum(q[ids[valid]] * weights[:, None], axis=0)
            feature_prev = np.sum((q[ids[valid]] - qd[ids[valid]] * sim_dt) * weights[:, None], axis=0)
            feature_delta = feature_now - feature_prev
            pad_delta = np.asarray(body_vel[record], dtype=float) * sim_dt
            max_rel = max(max_rel, float(np.linalg.norm(feature_delta - pad_delta)))
            cfl_max = max(cfl_max, float(np.linalg.norm(pad_delta)))
            record_count += 1
        per_pad[name] = {
            "max_rel_disp_m": max_rel,
            "record_count": record_count,
            "zero_record_substeps": int(record_count == 0),
            "overflow_substeps": int(count > capacity),
            "certified": bool(np.isfinite(max_rel) and max_rel <= 0.5e-3
                              and record_count > 0 and count <= capacity),
        }
    return {
        "per_pad": per_pad,
        "cfl_max_substep_m": cfl_max,
        "certified": all(pad["certified"] for pad in per_pad.values()),
    }


def disposition(certified: bool, is_deciding_coordinate: bool, prior_failures_in_row: int) -> str:
    """Return the pre-registered response to a failed validity gate."""
    if certified:
        raise ValueError("disposition is only defined for uncertified cells")
    if is_deciding_coordinate:
        return "stopped_deciding_coordinate"
    if prior_failures_in_row >= 1:
        return "stopped_second_in_row"
    return "censored_interior"


if wp is not None:
    @wp.kernel
    def _accumulate_kernel(
        count: wp.array(dtype=wp.int32),
        shapes: wp.array(dtype=wp.int32),
        indices: wp.array(dtype=wp.vec3i),
        bary: wp.array(dtype=wp.vec3),
        body_vel: wp.array(dtype=wp.vec3),
        particle_qd: wp.array(dtype=wp.vec3),
        pad_left: wp.int32,
        pad_right: wp.int32,
        particle_start: wp.int32,
        particle_end: wp.int32,
        capacity: wp.int32,
        dt: wp.float32,
        maxima_mm: wp.array(dtype=wp.float32),
        records: wp.array(dtype=wp.int32),
        cfl_mm: wp.array(dtype=wp.float32),
    ):
        tid = wp.tid()
        n = count[0]
        if tid >= n or tid >= capacity:
            return
        shape = shapes[tid]
        pad = wp.int32(-1)
        if shape == pad_left:
            pad = 0
        elif shape == pad_right:
            pad = 1
        if pad < 0:
            return
        ids = indices[tid]
        weights = bary[tid]
        feature_vel = wp.vec3(0.0)
        valid = wp.int32(0)
        for slot in range(3):
            particle = ids[slot]
            if particle >= particle_start and particle < particle_end:
                feature_vel += particle_qd[particle] * weights[slot]
                valid += 1
            elif particle >= 0:
                return
        if valid == 0:
            return
        pad_delta = body_vel[tid] * dt
        relative_mm = wp.length(feature_vel * dt - pad_delta) * 1000.0
        wp.atomic_max(maxima_mm, pad, relative_mm)
        wp.atomic_add(records, pad, 1)
        wp.atomic_max(cfl_mm, 0, wp.length(pad_delta) * 1000.0)


class ValidityAccumulator:
    """Device-side substep accumulator, callable as a rig substep hook."""

    def __init__(self, pad_left_shape, pad_right_shape, block_particle_range, sim_dt, margin=1e-3):
        if wp is None:
            raise RuntimeError("Warp is required to attach ValidityAccumulator to a rig")
        self.pad_shapes = (int(pad_left_shape), int(pad_right_shape))
        self.block_particle_range = tuple(map(int, block_particle_range))
        self.sim_dt = float(sim_dt)
        self.margin = float(margin)
        if self.sim_dt <= 0.0 or self.margin <= 0.0:
            raise ValueError("sim_dt and margin must be positive")
        self._device = None
        self._maxima = self._step_records = self._zero = self._overflow = self._cfl = None
        self._trial_max = np.zeros(2)
        self._trial_zero = np.zeros(2, dtype=np.int64)
        self._trial_overflow = np.zeros(2, dtype=np.int64)
        self._trial_cfl = 0.0

    def _ensure_device(self, device):
        if self._device == device:
            return
        self._device = device
        self._maxima = wp.zeros(2, dtype=wp.float32, device=device)
        self._step_records = wp.zeros(2, dtype=wp.int32, device=device)
        self._zero = wp.zeros(2, dtype=wp.int32, device=device)
        self._overflow = wp.zeros(1, dtype=wp.int32, device=device)
        self._cfl = wp.zeros(1, dtype=wp.float32, device=device)

    def __call__(self, rig, k):
        """Accumulate one solver substep; exact hook signature: ``fn(rig, k)``."""
        contacts = rig.contacts
        state = rig.state_0
        device = state.particle_q.device
        self._ensure_device(device)
        capacity = int(contacts.soft_contact_shape.shape[0])
        wp.launch(
            _accumulate_kernel,
            dim=capacity,
            inputs=[contacts.soft_contact_count, contacts.soft_contact_shape,
                    contacts.soft_contact_indices, contacts.soft_contact_barycentric,
                    contacts.soft_contact_body_vel, state.particle_qd,
                    self.pad_shapes[0], self.pad_shapes[1],
                    self.block_particle_range[0], self.block_particle_range[1],
                    capacity, self.sim_dt],
            outputs=[self._maxima, self._step_records, self._cfl],
            device=device,
        )
        # These tiny kernels keep continuity/overflow evidence on the device.
        wp.launch(_finish_substep_kernel, dim=1,
                  inputs=[contacts.soft_contact_count, capacity],
                  outputs=[self._step_records, self._zero, self._overflow], device=device)

    def fold_frame(self):
        """Download and fold the current frame, then reset device frame state."""
        if self._device is None:
            return self.summary()
        maxima = self._maxima.numpy().astype(float)
        zero = self._zero.numpy().astype(np.int64)
        overflow = int(self._overflow.numpy()[0])
        cfl = float(self._cfl.numpy()[0])
        self._trial_max = np.maximum(self._trial_max, maxima)
        self._trial_zero += zero
        self._trial_overflow += overflow
        self._trial_cfl = max(self._trial_cfl, cfl)
        for array in (self._maxima, self._step_records, self._zero, self._overflow, self._cfl):
            array.zero_()
        return self.summary()

    def summary(self):
        per_pad = {}
        for i, name in enumerate(_PAD_NAMES):
            max_mm = float(self._trial_max[i])
            zero = int(self._trial_zero[i])
            overflow = int(self._trial_overflow[i])
            per_pad[name] = {
                "vg1_max_rel_disp_mm": max_mm,
                "vg2_zero_record_substeps": zero,
                "vg3_overflow_substeps": overflow,
                "certified": bool(np.isfinite(max_mm) and max_mm <= 0.5 * self.margin * 1000.0
                                  and zero == 0 and overflow == 0),
            }
        return {"per_pad": per_pad, "cfl_max_substep_mm": float(self._trial_cfl)}


if wp is not None:
    @wp.kernel
    def _finish_substep_kernel(
        count: wp.array(dtype=wp.int32),
        capacity: wp.int32,
        records: wp.array(dtype=wp.int32),
        zero: wp.array(dtype=wp.int32),
        overflow: wp.array(dtype=wp.int32),
    ):
        if records[0] == 0:
            wp.atomic_add(zero, 0, 1)
        if records[1] == 0:
            wp.atomic_add(zero, 1, 1)
        records[0] = 0
        records[1] = 0
        if count[0] > capacity:
            wp.atomic_add(overflow, 0, 1)
