"""Assemble gate media: npz frame dumps -> PNG frames -> mp4 + key frames.

Fallback orthographic render path (PIL + ffmpeg); never blocks physics.
Usage:
  newton/.venv/bin/python scripts/make_media.py reports/media/frames/gentle_3333 \
      --mp4 reports/media/gn2_gentle_lift.mp4 --keyframes 5 --fps 10
"""

from __future__ import annotations

import argparse
import glob
import os
import shutil
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scripts.render_frames import render_frame


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("frames_dir")
    ap.add_argument("--mp4", required=True)
    ap.add_argument("--keyframes", type=int, default=5)
    ap.add_argument("--fps", type=int, default=10)
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.frames_dir, "*.npz")))
    if not files:
        print(f"no frames in {args.frames_dir}")
        return 1

    os.makedirs(os.path.dirname(args.mp4), exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        for i, f in enumerate(files):
            render_frame(f, os.path.join(tmp, f"f_{i:05d}.png"))
        cmd = [
            "ffmpeg", "-y", "-loglevel", "error", "-framerate", str(args.fps),
            "-i", os.path.join(tmp, "f_%05d.png"),
            "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", args.mp4,
        ]
        subprocess.run(cmd, check=True)

        # key frames: evenly spaced incl. first/last
        base = os.path.splitext(args.mp4)[0]
        n = len(files)
        picks = sorted({0, n - 1, *[round(k * (n - 1) / (args.keyframes - 1)) for k in range(args.keyframes)]})
        for j, idx in enumerate(picks[: args.keyframes]):
            shutil.copy(os.path.join(tmp, f"f_{idx:05d}.png"), f"{base}_key{j}.png")

    # verify: decode check + frame count
    probe = subprocess.run(
        ["ffprobe", "-v", "error", "-count_frames", "-select_streams", "v:0",
         "-show_entries", "stream=nb_read_frames,width,height", "-of", "csv=p=0", args.mp4],
        capture_output=True, text=True, check=True,
    )
    print(f"{args.mp4}: streams {probe.stdout.strip()} | key frames: {min(args.keyframes, len(picks))}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
