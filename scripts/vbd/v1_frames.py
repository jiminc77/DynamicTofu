"""V-1 frames: run the base proxy_joint_gripper example and dump snapshots.

Renders a few orthographic frames (PIL) to visually confirm the MuJoCo
2-finger gripper closing on the VBD soft grid at pin b74df534.

Run: cd newton && uv run --no-sync python ../scripts/vbd/v1_frames.py
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import numpy as np
import newton
import newton.examples
from newton.examples.multiphysics.example_proxy_joint_gripper import Example

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
OUT = os.path.join(ROOT, "reports", "media", "frames", "v1_proxy_gripper")


def main() -> int:
    parser = Example.create_parser()
    args = parser.parse_args(["--viewer", "null", "--num-frames", "120", "--quiet"])
    viewer, args = newton.examples.init(parser, args=args) if False else (_NullViewer(), args)
    ex = Example(viewer, args)
    os.makedirs(OUT, exist_ok=True)
    snaps = []
    for f in range(120):
        ex.step()
        if f % 8 == 0:
            s0 = ex.state_0
            pq = s0.particle_q.numpy()[ex.soft_particle_start:ex.soft_particle_end]
            bq = s0.body_q.numpy()
            np.savez_compressed(os.path.join(OUT, f"f_{len(snaps):04d}.npz"),
                                particle_q=pq.astype(np.float32), body_q=bq.astype(np.float32),
                                t=np.float64(ex.sim_time))
            snaps.append(f)
    print(f"captured {len(snaps)} snapshots -> {OUT}")

    # render (x,y top view showing the fingers close along y; and (y,z) side)
    from PIL import Image, ImageDraw
    W = 480
    xr, yr, zr = (-0.14, 0.10), (-0.16, 0.16), (0.02, 0.24)

    def px(u, v, ur, vr):
        return ((u - ur[0]) / (ur[1] - ur[0]) * (W - 1),
                (W - 1) - (v - vr[0]) / (vr[1] - vr[0]) * (W - 1))

    files = sorted(os.listdir(OUT))
    files = [x for x in files if x.endswith(".npz")]
    for name in files:
        d = np.load(os.path.join(OUT, name))
        pq, bq, t = d["particle_q"], d["body_q"], float(d["t"])
        img = Image.new("RGB", (2 * W, W), (16, 16, 20))
        dr = ImageDraw.Draw(img)
        # top (x,y) and side (y,z)
        for i in range(len(pq)):
            x0, y0 = px(pq[i, 0], pq[i, 1], xr, yr)
            dr.ellipse([x0 - 2, y0 - 2, x0 + 2, y0 + 2], fill=(235, 235, 210))
            x1, y1 = px(pq[i, 1], pq[i, 2], yr, zr)
            dr.ellipse([W + x1 - 2, y1 - 2, W + x1 + 2, y1 + 2], fill=(235, 235, 210))
        for b in range(len(bq)):
            col = (90, 200, 90) if b == 0 else (240, 130, 60)
            x0, y0 = px(bq[b, 0], bq[b, 1], xr, yr)
            dr.ellipse([x0 - 4, y0 - 4, x0 + 4, y0 + 4], outline=col, width=2)
            x1, y1 = px(bq[b, 1], bq[b, 2], yr, zr)
            dr.ellipse([W + x1 - 4, y1 - 4, W + x1 + 4, y1 + 4], outline=col, width=2)
        dr.text((8, 6), f"t={t:.2f}s  top(x,y) | side(y,z)  proxy_joint_gripper @ b74df53", fill=(200, 200, 255))
        img.save(os.path.join(OUT, name.replace(".npz", ".png")))
    print(f"rendered {len(files)} frames")
    return 0


class _NullViewer:
    show_particles = False

    def set_camera(self, *a, **k):
        pass

    def begin_frame(self, *a, **k):
        pass

    def end_frame(self, *a, **k):
        pass

    def log_state(self, *a, **k):
        pass

    def log_shapes(self, *a, **k):
        pass

    def __getattr__(self, name):
        return lambda *a, **k: None


if __name__ == "__main__":
    sys.exit(main())
