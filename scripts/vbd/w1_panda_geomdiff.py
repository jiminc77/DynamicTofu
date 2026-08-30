"""Compare frozen and Panda contact geometry/strain at E7/F2.

GPU driver (from newton/):
  PYTHONPATH=/home/simx2204/Workspace/DynamicTofu uv run --no-sync python ../scripts/vbd/w1_panda_geomdiff.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.judgment_vbd import LIFT_END, latched_dvf
from src.pad_wrench import capture_pre_step, collect_pad_contacts, collect_pad_wrench
from src.vbd_rig2 import FPS, GRAB_Z, Vbd2Config, Vbd2Rig
from src.vbd_rig_panda import PandaRig


def cfg():
    return Vbd2Config(E_pa=7000.0, nu=0.45, grip_force_n=2.0, cell_m=0.005,
                      particle_radius=0.0025, contact_ke=1e3, contact_kd=1.0,
                      mu_pair=1.0, friction_epsilon=2e-4,
                      soft_contact_margin=1e-3, substeps=80, lift_s=2.5,
                      hold_s=5.0, lift_height_m=0.05, seed=0)


def rotate(q, v):
    a = np.asarray(q[:3], dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    return v + 2.0 * float(q[3]) * np.cross(a, v) + 2.0 * np.cross(a, np.cross(a, v))


def inverse_rotate(q, v):
    qi = np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)
    return rotate(qi, v)


def shape_pose(rig, shape, body_q):
    body = int(rig.model.shape_body.numpy()[shape])
    local = rig.model.shape_transform.numpy()[shape]
    center = body_q[body, :3] + rotate(body_q[body, 3:7], local[:3])
    # Only identity local rotations are expected for the frozen replacement pads.
    normal = rotate(body_q[body, 3:7], rotate(local[3:7], (0.0, 1.0, 0.0)))
    normal /= np.linalg.norm(normal)
    return center, normal, body_q[body, 3:7]


def frame(rig, force, lift_target):
    rig.set_control(force, lift_target)
    pre = None
    for k in range(rig.sim_substeps):
        rig.state_0.clear_forces()
        pre = capture_pre_step(rig.state_0)
        rig.collision_pipeline.collide(rig.state_0, rig.contacts)
        rig.solver.step(rig.state_0, rig.state_1, rig.control, rig.contacts, rig.sim_dt)
        rig.state_0, rig.state_1 = rig.state_1, rig.state_0
        if isinstance(rig, PandaRig) and rig.couple:
            bq = rig.state_0.body_q.numpy()
            half_gap = 0.5 * (bq[rig.b_left, 1] - bq[rig.b_right, 1])
            palm_y = bq[rig.b_palm, 1]
            bq[rig.b_left, 1] = palm_y + half_gap
            bq[rig.b_right, 1] = palm_y - half_gap
            rig.state_0.body_q.assign(bq)
        for hook in rig._substep_hooks:
            hook(rig, k)
    rig.sim_time += rig.frame_dt
    records = collect_pad_contacts(rig, pre_state=pre, post_state=rig.state_0,
                                   contacts=rig.contacts, dt=rig.sim_dt)
    wrench = collect_pad_wrench(rig, pre_state=pre, post_state=rig.state_0,
                                contacts=rig.contacts, dt=rig.sim_dt)
    return records, wrench


def histogram(field, volumes):
    edges = np.array([-np.inf, 0.05, 0.10, 0.15, 0.20, 0.30, np.inf])
    total = float(volumes.sum())
    return [{"lo": None if not np.isfinite(edges[i]) else float(edges[i]),
             "hi": None if not np.isfinite(edges[i + 1]) else float(edges[i + 1]),
             "tet_count": int(np.count_nonzero((field >= edges[i]) & (field < edges[i + 1]))),
             "volume_fraction": float(volumes[(field >= edges[i]) & (field < edges[i + 1])].sum() / total)}
            for i in range(len(edges) - 1)]


def summarize(name):
    rig = PandaRig(cfg(), couple=True) if name == "panda" else Vbd2Rig(cfg())
    shape_body = rig.model.shape_body.numpy()
    shapes = {side: int(np.flatnonzero(shape_body == body)[0])
              for side, body in (("left", rig.b_left), ("right", rig.b_right))}
    c = rig.cfg
    t_pre, t_end = c.ramp_s + c.preload_s, c.ramp_s + c.preload_s + c.lift_s + c.hold_s
    temporal = None
    records = wrench = None
    while rig.sim_time < t_end - 0.5 / FPS:
        t = rig.sim_time
        force = c.grip_force_n * min(1.0, t / c.ramp_s)
        fraction = min(1.0, max(0.0, t - t_pre) / c.lift_s)
        records, wrench = frame(rig, force, GRAB_Z + c.lift_height_m * fraction)
        field, volumes = rig.strain_field()
        if rig.sim_time >= LIFT_END:
            temporal = field.copy() if temporal is None else np.maximum(temporal, field)
    bq = rig.state_0.body_q.numpy()
    pq = rig.state_0.particle_q.numpy()[rig.soft_start:rig.soft_end]
    block_center = pq.mean(axis=0)
    block_ymin, block_ymax = float(pq[:, 1].min()), float(pq[:, 1].max())
    contacts = {}
    poses = {}
    for side in ("left", "right"):
        shape = shapes[side]
        center, outward, body_quat = shape_pose(rig, shape, bq)
        points = np.asarray([r["contact_point_world"] for r in records if r["pad_id"] == side], dtype=float)
        if points.size:
            local = np.asarray([inverse_rotate(body_quat, p - center) for p in points])
            footprint = {"centroid_world_m": points.mean(axis=0).tolist(),
                         "centroid_pad_frame_m": local.mean(axis=0).tolist(),
                         "x_extent_m": float(np.ptp(local[:, 0])),
                         "z_extent_m": float(np.ptp(local[:, 2])),
                         "area_bbox_m2": float(np.ptp(local[:, 0]) * np.ptp(local[:, 2]))}
        else:
            footprint = {"centroid_world_m": None, "centroid_pad_frame_m": None,
                         "x_extent_m": 0.0, "z_extent_m": 0.0, "area_bbox_m2": 0.0}
        contacts[side] = {"n_contacts": int(len(points)), **footprint,
                          "Fn_collector_n": float(wrench[side]["Fn"]),
                          "Ft_collector_n": float(wrench[side]["Ft"])}
        inner_face_y = float(center[1] - 0.006) if side == "left" else float(center[1] + 0.006)
        block_face_y = block_ymax if side == "left" else block_ymin
        poses[side] = {"pad_center_world_m": center.tolist(),
                       "pad_outward_normal_world": outward.tolist(),
                       "normal_alignment_abs_world_y": float(abs(outward[1])),
                       "tilt_from_world_y_deg": float(np.degrees(np.arccos(np.clip(abs(outward[1]), -1, 1)))),
                       "inner_face_y_m": inner_face_y, "block_face_y_m": block_face_y,
                       "inner_face_minus_block_face_mm": float((inner_face_y - block_face_y) * 1000),
                       "center_z_minus_block_center_z_mm": float((center[2] - block_center[2]) * 1000)}
    ti = rig.tet_idx
    centroids = pq[ti].mean(axis=1)
    peak_i = int(np.argmax(temporal))
    peak_pos = centroids[peak_i]
    distances = {"left_pad_center_m": float(np.linalg.norm(peak_pos - np.asarray(poses["left"]["pad_center_world_m"]))),
                 "right_pad_center_m": float(np.linalg.norm(peak_pos - np.asarray(poses["right"]["pad_center_world_m"]))) }
    nearest_face = min(abs(peak_pos[1] - block_ymin), abs(peak_pos[1] - block_ymax))
    edge_distance = min(peak_pos[0] - pq[:, 0].min(), pq[:, 0].max() - peak_pos[0],
                        peak_pos[2] - pq[:, 2].min(), pq[:, 2].max() - peak_pos[2])
    region = "pad_face_edge" if nearest_face < 0.008 and edge_distance < 0.008 else ("pad_face_center" if nearest_face < 0.008 else "interior")
    dvf, latched = latched_dvf(temporal, volumes)
    return {"rig": name, "settled_time_s": float(rig.sim_time), "block_center_world_m": block_center.tolist(),
            "contacts": contacts, "pad_block_pose": poses,
            "strain": {"peak": float(temporal[peak_i]), "peak_tet_index": peak_i,
                       "peak_location_world_m": peak_pos.tolist(), "peak_region": region,
                       "peak_distances": distances, "dvf_eps_0p15": float(dvf),
                       "damage_latched": bool(latched), "histogram": histogram(temporal, volumes)}}


def numeric_diff(frozen, panda):
    out = {}
    for path, a, b in (
        ("dvf_eps_0p15", frozen["strain"]["dvf_eps_0p15"], panda["strain"]["dvf_eps_0p15"]),
        ("peak_strain", frozen["strain"]["peak"], panda["strain"]["peak"]),
        ("left_contact_count", frozen["contacts"]["left"]["n_contacts"], panda["contacts"]["left"]["n_contacts"]),
        ("right_contact_count", frozen["contacts"]["right"]["n_contacts"], panda["contacts"]["right"]["n_contacts"]),
        ("left_area_bbox_m2", frozen["contacts"]["left"]["area_bbox_m2"], panda["contacts"]["left"]["area_bbox_m2"]),
        ("right_area_bbox_m2", frozen["contacts"]["right"]["area_bbox_m2"], panda["contacts"]["right"]["area_bbox_m2"]),
        ("left_height_offset_mm", frozen["pad_block_pose"]["left"]["center_z_minus_block_center_z_mm"], panda["pad_block_pose"]["left"]["center_z_minus_block_center_z_mm"]),
        ("right_height_offset_mm", frozen["pad_block_pose"]["right"]["center_z_minus_block_center_z_mm"], panda["pad_block_pose"]["right"]["center_z_minus_block_center_z_mm"]),
    ):
        out[path] = {"frozen": float(a), "panda": float(b), "panda_minus_frozen": float(b - a)}
    return out


def main():
    frozen, panda = summarize("frozen"), summarize("panda")
    result = {"schema": "panda_geomdiff.v1", "case": {"E_pa": 7000, "F_n": 2.0, "seed": 0},
              "frozen": frozen, "panda": panda, "diff": numeric_diff(frozen, panda)}
    path = ROOT / "reports/logs/vbd/panda/panda_geomdiff.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps(result["diff"], indent=2)); print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
