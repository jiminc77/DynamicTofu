"""E2 tactile capture: raw per-node contact field + host-derived aggregates.

Contract (pending-approval.md, E2 + stage-03 intent):
- The top-level coupled tick is the sample (entry substeps are not observable).
- Raw field per sample: node positions (world AND pad frame), 3-axis force per
  node (impulse / exact MPM dt), finger id, per-sample pad poses; ragged
  samples encoded as concatenated arrays + per-sample offsets.
- Aggregates (priority: shear per finger > centroid in finger frame > contact
  area > L-R asymmetry, plus per-finger normal) are DERIVED from the stored
  field by one code path, so recompute_aggregates(npz) == stored aggregates
  BITWISE by construction.
- Sensor emulation (taxel layout, noise, hysteresis) is out of scope.
"""

from __future__ import annotations

import numpy as np

IMPULSE_EPS = 1e-8


def _rot_matrix(q_xyzw: np.ndarray) -> np.ndarray:
    """Rotation matrix R such that v_world = R @ v_body for quat (x,y,z,w)."""
    x, y, z, w = (float(v) for v in q_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def _rotate_rows_to_frame(q_xyzw: np.ndarray, rows: np.ndarray) -> np.ndarray:
    """Rotate world-frame row vectors into the body frame (R^T @ v)."""
    return rows.astype(np.float64) @ _rot_matrix(q_xyzw)  # (R^T v)^T = v^T R


class TactileRecorder:
    """Accumulates the raw per-node field per top-level tick."""

    def __init__(self, rig, pad_normal_local=(0.0, 1.0, 0.0)):
        self.rig = rig
        self.left, self.right = rig.meta.finger_body_indices
        self.pad_normal_local = np.asarray(pad_normal_local, dtype=np.float32)
        self.node_pos_world: list[np.ndarray] = []
        self.node_pos_pad: list[np.ndarray] = []
        self.node_force_world: list[np.ndarray] = []
        self.node_finger_id: list[np.ndarray] = []
        self.sample_offsets = [0]
        self.pad_pose_left: list[np.ndarray] = []
        self.pad_pose_right: list[np.ndarray] = []
        self.sample_t_s: list[float] = []
        self.dt_mpm_s: list[float] = []
        self.phase_marks: dict[str, float] = {}

    def mark(self, name: str):
        self.phase_marks[name] = self.rig.t

    def capture(self, dt_mpm: float):
        imp, pos, cid = self.rig.mpm.collect_collider_impulses(self.rig.state)
        impn = imp.numpy().astype(np.float64)
        posn = pos.numpy().astype(np.float64)
        cidn = cid.numpy().astype(int)
        body_of = self.rig.mpm.collider_body_index.numpy().astype(int)
        bq = self.rig.state.body_q.numpy()

        mags = np.linalg.norm(impn, axis=1)
        keep, fids = [], []
        for k in np.nonzero(mags > IMPULSE_EPS)[0]:
            c = cidn[k]
            if 0 <= c < len(body_of):
                b = body_of[c]
                if b == self.left:
                    keep.append(k); fids.append(0)
                elif b == self.right:
                    keep.append(k); fids.append(1)
        keep = np.asarray(keep, dtype=int)
        n = len(keep)

        pose_l = bq[self.left][:7].astype(np.float32)
        pose_r = bq[self.right][:7].astype(np.float32)
        if n:
            f_world = (impn[keep] / dt_mpm).astype(np.float32)
            p_world = posn[keep].astype(np.float32)
            fid = np.asarray(fids, dtype=np.int16)
            p_pad = np.empty_like(p_world)
            for side, pose in ((0, pose_l), (1, pose_r)):
                m = fid == side
                if m.any():
                    rel = p_world[m].astype(np.float64) - pose[:3].astype(np.float64)
                    p_pad[m] = _rotate_rows_to_frame(pose[3:7].astype(np.float64), rel).astype(np.float32)
        else:
            f_world = np.zeros((0, 3), np.float32)
            p_world = np.zeros((0, 3), np.float32)
            p_pad = np.zeros((0, 3), np.float32)
            fid = np.zeros(0, np.int16)

        self.node_pos_world.append(p_world)
        self.node_pos_pad.append(p_pad)
        self.node_force_world.append(f_world)
        self.node_finger_id.append(fid)
        self.sample_offsets.append(self.sample_offsets[-1] + n)
        self.pad_pose_left.append(pose_l)
        self.pad_pose_right.append(pose_r)
        self.sample_t_s.append(self.rig.t)
        self.dt_mpm_s.append(dt_mpm)

    def arrays(self, voxel_size: float) -> dict:
        raw = {
            "node_pos_world": np.concatenate(self.node_pos_world) if self.node_pos_world else np.zeros((0, 3), np.float32),
            "node_pos_pad": np.concatenate(self.node_pos_pad) if self.node_pos_pad else np.zeros((0, 3), np.float32),
            "node_force_world": np.concatenate(self.node_force_world) if self.node_force_world else np.zeros((0, 3), np.float32),
            "node_finger_id": np.concatenate(self.node_finger_id) if self.node_finger_id else np.zeros(0, np.int16),
            "sample_offsets": np.asarray(self.sample_offsets, np.int32),
            "pad_pose_left": np.asarray(self.pad_pose_left, np.float32),
            "pad_pose_right": np.asarray(self.pad_pose_right, np.float32),
            "sample_t_s": np.asarray(self.sample_t_s, np.float64),
            "dt_mpm_s": np.asarray(self.dt_mpm_s, np.float64),
            "voxel_size": np.float64(voxel_size),
            "pad_normal_local": self.pad_normal_local,
        }
        raw.update(compute_aggregates(raw))
        return raw


def compute_aggregates(raw: dict) -> dict:
    """THE aggregate code path: derives all channels from the raw field only.

    Reduction per sample and finger: sum force vectors FIRST, decompose
    normal/tangential AFTER summation using the declared pad normal in the pad
    frame; impulse-weighted centroid over surviving nodes; contact area =
    n_active_nodes * voxel_size^2; explicit no-contact (centroid NaN).
    """
    offsets = np.asarray(raw["sample_offsets"], np.int32)
    n_samples = len(offsets) - 1
    fid = np.asarray(raw["node_finger_id"])
    f_pad_all = _forces_in_pad_frames(raw)
    p_pad = np.asarray(raw["node_pos_pad"], np.float64)
    normal_local = np.asarray(raw["pad_normal_local"], np.float64)
    voxel = float(raw["voxel_size"])

    normal = np.zeros((n_samples, 2), np.float64)
    shear = np.zeros((n_samples, 2), np.float64)
    centroid = np.full((n_samples, 2, 3), np.nan, np.float64)
    area = np.zeros((n_samples, 2), np.float64)
    n_nodes = np.zeros((n_samples, 2), np.int32)
    for s in range(n_samples):
        lo, hi = offsets[s], offsets[s + 1]
        for side in (0, 1):
            m = fid[lo:hi] == side
            idx = np.nonzero(m)[0] + lo
            n_nodes[s, side] = len(idx)
            if not len(idx):
                continue
            fsum = f_pad_all[idx].sum(axis=0)
            sign = 1.0 if side == 0 else -1.0
            n_comp = float(np.dot(fsum, sign * normal_local))
            tang = fsum - n_comp * sign * normal_local
            normal[s, side] = n_comp
            shear[s, side] = float(np.linalg.norm(tang))
            w = np.linalg.norm(f_pad_all[idx], axis=1)
            wsum = w.sum()
            if wsum > 0:
                centroid[s, side] = (p_pad[idx] * w[:, None]).sum(axis=0) / wsum
            area[s, side] = len(idx) * voxel * voxel
    asym = normal[:, 0] - normal[:, 1]
    return {
        "agg_normal_n": normal, "agg_shear_n": shear, "agg_centroid_pad_m": centroid,
        "agg_area_m2": area, "agg_n_nodes": n_nodes, "agg_lr_asymmetry_n": asym,
        "agg_in_contact": (n_nodes > 0),
    }


def _forces_in_pad_frames(raw: dict) -> np.ndarray:
    fid = np.asarray(raw["node_finger_id"])
    f_world = np.asarray(raw["node_force_world"], np.float64)
    offsets = np.asarray(raw["sample_offsets"], np.int32)
    poses = {0: np.asarray(raw["pad_pose_left"], np.float64), 1: np.asarray(raw["pad_pose_right"], np.float64)}
    out = np.zeros_like(f_world)
    for s in range(len(offsets) - 1):
        lo, hi = offsets[s], offsets[s + 1]
        for side in (0, 1):
            m = np.nonzero(fid[lo:hi] == side)[0] + lo
            if len(m):
                q = poses[side][s][3:7]
                out[m] = _rotate_rows_to_frame(q, f_world[m])
    return out


def recompute_aggregates(npz) -> dict:
    """Recompute all aggregates from a stored npz's raw field (bitwise hook)."""
    raw = {k: npz[k] for k in (
        "node_pos_world", "node_pos_pad", "node_force_world", "node_finger_id",
        "sample_offsets", "pad_pose_left", "pad_pose_right", "sample_t_s",
        "dt_mpm_s", "voxel_size", "pad_normal_local",
    )}
    return compute_aggregates(raw)
