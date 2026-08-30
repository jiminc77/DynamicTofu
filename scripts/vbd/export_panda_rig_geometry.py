"""Export visible geometry exactly as stored in the finalized PandaRig model."""
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from src.vbd_rig2 import Vbd2Config
from src.vbd_rig_panda import PandaRig


def main():
    cfg = Vbd2Config(
        E_pa=7000.0, nu=0.45, grip_force_n=2.0, cell_m=0.005,
        particle_radius=0.0025, contact_ke=1e3, contact_kd=1.0,
        mu_pair=1.0, friction_epsilon=2e-4, soft_contact_margin=1e-3,
        substeps=80, lift_s=2.5, hold_s=5.0, lift_height_m=0.05, seed=0,
    )
    model = PandaRig(cfg).model
    bodies = model.shape_body.numpy()
    kinds = model.shape_type.numpy()
    flags = model.shape_flags.numpy()
    scales = model.shape_scale.numpy()
    colors = model.shape_color.numpy()
    local_poses = model.shape_transform.numpy()
    shape_indices = [i for i in range(model.shape_count)
                     if bodies[i] >= 0 and (flags[i] & 1)]
    vertices = []
    faces = []
    vertex_ranges = []
    face_ranges = []
    for index in shape_indices:
        vertex_start, face_start = len(vertices), len(faces)
        source = model.shape_source[index]
        if int(kinds[index]) == 8 and source is not None:
            shape_vertices = np.asarray(source.vertices, dtype=np.float32) * scales[index]
            shape_faces = np.asarray(source.indices, dtype=np.int32).reshape(-1, 3)
            vertices.extend(shape_vertices)
            faces.extend(shape_faces + vertex_start)
        vertex_ranges.append((vertex_start, len(vertices)))
        face_ranges.append((face_start, len(faces)))
    output = ROOT / "reports/vbd/clips/panda/panda_rig_geometry.npz"
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output,
        shape_index=np.asarray(shape_indices, dtype=np.int32),
        shape_body=bodies[shape_indices].astype(np.int32),
        shape_kind=kinds[shape_indices].astype(np.int32),
        shape_flags=flags[shape_indices].astype(np.int32),
        shape_transform=local_poses[shape_indices].astype(np.float32),
        shape_scale=scales[shape_indices].astype(np.float32),
        shape_color=colors[shape_indices].astype(np.float32),
        vertex_range=np.asarray(vertex_ranges, dtype=np.int32),
        face_range=np.asarray(face_ranges, dtype=np.int32),
        vertices=np.asarray(vertices, dtype=np.float32),
        faces=np.asarray(faces, dtype=np.int32),
        body_labels=np.asarray(model.body_label, dtype="U64"),
    )
    print(output)


if __name__ == "__main__":
    main()
