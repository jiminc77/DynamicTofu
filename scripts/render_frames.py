"""Fallback orthographic frame renderer (PIL): particles + finger/body markers.

Two panels per frame: (y, z) side view and (x, z) front view. Used for gate
media key frames and for frame-by-frame sim-quality inspection; never blocks
physics. Usage:
  newton/.venv/bin/python scripts/render_frames.py reports/media/frames/gentle --out reports/media/gentle
"""

from __future__ import annotations

import argparse
import glob
import os

import numpy as np
from PIL import Image, ImageDraw

W, H = 640, 640
WORLD_Y = (-0.62, -0.38)   # side view horizontal range
WORLD_X = (-0.12, 0.12)    # front view horizontal range
WORLD_Z = (0.14, 0.38)


def _to_px(u, v, urange, vrange):
    x = (u - urange[0]) / (urange[1] - urange[0]) * (W - 1)
    y = (H - 1) - (v - vrange[0]) / (vrange[1] - vrange[0]) * (H - 1)
    return x, y


def render_frame(npz_path: str, out_path: str) -> None:
    data = np.load(npz_path)
    pq = data["particle_q"]
    jp = data.get("jp")
    bq = data["body_q"]
    labels = [str(x.decode() if isinstance(x, bytes) else x) for x in data["body_labels"]] if "body_labels" in data else []
    finger_indices = {i for i, label in enumerate(labels) if label in {"left", "right"}}
    if not finger_indices:
        finger_indices = {i for i in (7, 8) if i < len(bq)}
    t = float(data["t"])

    img = Image.new("RGB", (2 * W, H), (16, 16, 20))
    draw = ImageDraw.Draw(img)
    all_pos = np.concatenate((pq[:, :3], bq[:, :3]), axis=0)

    def view_range(axis, minimum_span):
        lo, hi = float(all_pos[:, axis].min()), float(all_pos[:, axis].max())
        pad = max(0.02, 0.08 * (hi - lo))
        mid = 0.5 * (lo + hi)
        half = max(0.5 * minimum_span, 0.5 * (hi - lo) + pad)
        return mid - half, mid + half

    world_y = view_range(1, 0.10)
    world_x = view_range(0, 0.10)
    world_z = view_range(2, 0.12)

    # table line
    for panel, urange in ((0, world_y), (1, world_x)):
        x0, y0 = _to_px(urange[0], 0.0, urange, world_z)
        x1, y1 = _to_px(urange[1], 0.0, urange, world_z)
        draw.line([(panel * W + x0, y0), (panel * W + x1, y1)], fill=(80, 80, 90), width=2)

    dmg = np.abs(jp - 1.0) > 0.05 if jp is not None else np.zeros(len(pq), bool)
    for panel, ucol, urange in ((0, 1, world_y), (1, 0, world_x)):
        us, vs = pq[:, ucol], pq[:, 2]
        inside = (us > urange[0]) & (us < urange[1]) & (vs > world_z[0]) & (vs < world_z[1])
        for i in np.nonzero(inside)[0]:
            x, y = _to_px(us[i], vs[i], urange, world_z)
            c = (240, 80, 80) if dmg[i] else (235, 235, 210)
            draw.point((panel * W + x, y), fill=c)
        n_out = int((~inside).sum())
        draw.text((panel * W + 8, 24), f"outside view: {n_out}", fill=(255, 160, 60))

    # Bodies: label-resolved fingers (cyan), other links (green).
    for panel, ucol, urange in ((0, 1, world_y), (1, 0, world_x)):
        for bi in range(len(bq)):
            u, v = bq[bi][ucol], bq[bi][2]
            if urange[0] < u < urange[1] and world_z[0] < v < world_z[1]:
                x, y = _to_px(u, v, urange, world_z)
                col = (80, 220, 255) if bi in finger_indices else (90, 200, 90)
                r = 5 if bi in finger_indices else 3
                draw.ellipse([panel * W + x - r, y - r, panel * W + x + r, y + r], outline=col, width=2)

    draw.text((8, 6), f"t={t:.2f}s  side(y,z) | front(x,z)", fill=(200, 200, 255))
    img.save(out_path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("frames_dir")
    ap.add_argument("--out", required=True)
    ap.add_argument("--every", type=int, default=1)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    files = sorted(glob.glob(os.path.join(args.frames_dir, "*.npz")))[:: args.every]
    for f in files:
        out = os.path.join(args.out, os.path.basename(f).replace(".npz", ".png"))
        render_frame(f, out)
    print(f"rendered {len(files)} frames -> {args.out}")


if __name__ == "__main__":
    main()
