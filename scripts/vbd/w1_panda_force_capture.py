"""Dense per-taxel Panda force capture for the three frozen W3 demo cells."""
from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from scripts.vbd import w1_panda, w1_transport
from src.pad_wrench import capture_pre_step, collect_pad_contacts, collect_pad_wrench
from src.vbd_rig_panda import PandaRig

PAD_HALF_XZ_M = 0.022
TAXEL_SHAPE = (8, 8)


def _rotate_inverse(quaternion, vectors):
    """Rotate world vectors into the body's local frame."""
    q = np.asarray(quaternion, dtype=np.float64)
    v = np.asarray(vectors, dtype=np.float64)
    qv = -q[:3]
    return v + 2.0 * np.cross(qv, np.cross(qv, v) + q[3] * v)


def _rotate(quaternion, vectors):
    q = np.asarray(quaternion, dtype=np.float64)
    v = np.asarray(vectors, dtype=np.float64)
    return v + 2.0 * np.cross(q[:3], np.cross(q[:3], v) + q[3] * v)


class ForceCapturePandaRig(PandaRig):
    """PandaRig with read-only R3 collection on the final substep."""

    def step(self, close_force, lift_target, x_target=None, x_vel=0.0, x_accel=0.0):
        self.set_control(close_force, lift_target, x_target=x_target,
                         x_vel=x_vel, x_accel=x_accel)
        for k in range(self.sim_substeps):
            self.state_0.clear_forces()
            pre = capture_pre_step(self.state_0) if k == self.sim_substeps - 1 else None
            self.collision_pipeline.collide(self.state_0, self.contacts)
            self.solver.step(self.state_0, self.state_1, self.control,
                             self.contacts, self.sim_dt)
            self.state_0, self.state_1 = self.state_1, self.state_0
            if k == self.sim_substeps - 1:
                # Call the validated reduction as well as its per-contact companion.
                self.frame_wrench = collect_pad_wrench(
                    self, pre_state=pre, post_state=self.state_0,
                    contacts=self.contacts, dt=self.sim_dt,
                )
                self.frame_contacts = collect_pad_contacts(
                    self, pre_state=pre, post_state=self.state_0,
                    contacts=self.contacts, dt=self.sim_dt,
                )
            if self.couple:
                bq = self.state_0.body_q.numpy()
                half_gap = 0.5 * (bq[self.b_left, 1] - bq[self.b_right, 1])
                palm_y = bq[self.b_palm, 1]
                bq[self.b_left, 1] = palm_y + half_gap
                bq[self.b_right, 1] = palm_y - half_gap
                self.state_0.body_q.assign(bq)
            for fn in self._substep_hooks:
                fn(self, k)
        self.sim_time += self.frame_dt


def _binned(rig):
    grids = {
        "left": (np.zeros(TAXEL_SHAPE), np.zeros((*TAXEL_SHAPE, 2))),
        "right": (np.zeros(TAXEL_SHAPE), np.zeros((*TAXEL_SHAPE, 2))),
    }
    body_q = rig.state_0.body_q.numpy()
    shape_xforms = rig.model.shape_transform.numpy()
    for rec in rig.frame_contacts:
        name = rec["pad_id"]
        body = rig.b_left if name == "left" else rig.b_right
        shape = rig.s_left if name == "left" else rig.s_right
        pose = body_q[body]
        # Renderer uses this same pad-shape origin (the body's visual origin is
        # displaced along local z by PAD_MOUNT_Z_OFFSET).
        pad_origin = pose[:3] + _rotate(pose[3:7], shape_xforms[shape, :3])
        point = _rotate_inverse(pose[3:7],
                                np.asarray(rec["contact_point_world"]) - pad_origin)
        force = _rotate_inverse(pose[3:7], rec["force_on_body_world"])
        ix = int(np.clip((point[0] + PAD_HALF_XZ_M)
                         / (2 * PAD_HALF_XZ_M) * 8, 0, 7))
        iz = int(np.clip((point[2] + PAD_HALF_XZ_M)
                         / (2 * PAD_HALF_XZ_M) * 8, 0, 7))
        inward = 1.0 if name == "left" else -1.0
        grids[name][0][iz, ix] += max(0.0, inward * force[1])
        grids[name][1][iz, ix] += force[[0, 2]]
    return grids


def capture(iterations):
    if iterations != 40:
        raise ValueError("force inset provenance requires --iter 40")
    import src.vbd_rig2 as frozen_rig

    root = ROOT / "reports/vbd/clips/panda"
    root.mkdir(parents=True, exist_ok=True)
    active = None

    def factory(cfg):
        nonlocal active
        active = ForceCapturePandaRig(cfg)
        return active

    original_rig = frozen_rig.Vbd2Rig
    original_save = np.savez_compressed

    def save_force_frame(path, **arrays):
        grids = _binned(active)
        left, right = grids["left"], grids["right"]
        original_save(
            path, **arrays, taxel_fn_left=left[0].astype(np.float32),
            taxel_fn_right=right[0].astype(np.float32),
            taxel_ft_left=left[1].astype(np.float32),
            taxel_ft_right=right[1].astype(np.float32),
            net_shear_left=left[1].sum(axis=(0, 1)).astype(np.float32),
            net_shear_right=right[1].sum(axis=(0, 1)).astype(np.float32),
        )

    entries = []
    try:
        frozen_rig.Vbd2Rig = factory
        np.savez_compressed = save_force_frame
        for spec in w1_panda.PANDA_DEMO_SCENES:
            target = root / f"w3_{spec['scene']}_force40"
            if target.exists():
                shutil.rmtree(target)
            target.mkdir()
            runner = w1_panda._capture_runner(spec["scene"] == "slip")
            receipt = runner(
                spec["E_kpa"] * 1000.0, spec["F"], spec["a"], spec["seed"],
                substeps=80, cell_m=0.005, snap_dir=target,
                iterations=iterations,
            )
            reproduced = receipt["label"] == spec["expected_label"]
            frames = sorted(target.glob("f_*.npz"))
            entries.append({
                "scene": spec["scene"], "iterations": iterations,
                "rerun_label": receipt["label"],
                "expected_label": spec["expected_label"],
                "label_reproduced": reproduced, "n_frames": len(frames),
                "provenance": "validated collector at 40 iterations",
                "taxel_extent": {
                    "shape": [8, 8], "local_axes": ["z", "x"],
                    "x_m": [-PAD_HALF_XZ_M, PAD_HALF_XZ_M],
                    "z_m": [-PAD_HALF_XZ_M, PAD_HALF_XZ_M],
                    "layout": "grid[z,x], floor-to-bin with upper edge clipped to 7",
                },
            })
            if not reproduced:
                raise RuntimeError(
                    f"{spec['scene']}: iter=40 label {receipt['label']!r} != "
                    f"frozen {spec['expected_label']!r}"
                )
    finally:
        np.savez_compressed = original_save
        frozen_rig.Vbd2Rig = original_rig
    manifest = root / "w3_force40_manifest.json"
    manifest.write_text(json.dumps({"scenes": entries}, indent=2) + "\n")
    return manifest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--iter", type=int, default=40)
    args = parser.parse_args()
    print(capture(args.iter))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
