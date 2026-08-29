"""Render the three pre-registered W3 scenes from frozen W1 transport cells.

Run in the Newton environment (CPU authoring only):
  python scripts/vbd/w3_clips_v2.py --render
  python scripts/vbd/w3_clips_v2.py --scene intact
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from scripts.render_frames import render_frame
from scripts.vbd.w1_transport import run_transport_cell

BAND_DIR = ROOT / "reports/logs/vbd/final"
OUT_DIR = ROOT / "reports/vbd/clips"
REPORT = ROOT / "reports/vbd/w3_clips.md"

# Requested coordinates and required final-band labels. Selection is checked at
# runtime so this authoring script cannot silently misrepresent a changed band.
REQUESTS = {
    "intact": {"E": 15, "a": 1.0, "F": 1.2, "label": "intact"},
    "slip": {"E": 15, "a": 20.0, "F": 1.2, "label": "slip"},
    "damage": {"E": 7, "a": 5.0, "F": 2.0, "label": "damage"},
}
KEY_TIMES = {
    "grip": 1.80,
    "lift": 4.30,
    "hold": 9.30,
    "accel_out_peak": 9.40,
    "dwell": 9.80,
    "return": 10.10,
    "settle": 10.60,
}


def _number_key(value: float) -> str:
    return f"{value:g}"


def select_cell(scene: str) -> dict:
    """Confirm a requested cell, or choose its nearest same-label band cell."""
    request = REQUESTS[scene]
    band = json.loads((BAND_DIR / f"e1v2_band_{request['E']}.json").read_text())
    a_key, f_key = _number_key(request["a"]), _number_key(request["F"])
    matrix = band["label_matrix"]
    if matrix.get(a_key, {}).get(f_key) == request["label"]:
        selected_a, selected_f = request["a"], request["F"]
    else:
        candidates = []
        for a, row in matrix.items():
            for force, label in row.items():
                if label == request["label"]:
                    # Preserve force first (the same-grip predicate), then find
                    # the nearest acceleration and finally nearest force.
                    candidates.append((float(force) != request["F"],
                                       abs(float(a) - request["a"]),
                                       abs(float(force) - request["F"]),
                                       float(a), float(force)))
        if not candidates:
            raise ValueError(f"E{request['E']} final band contains no {request['label']} cell")
        _, _, _, selected_a, selected_f = min(candidates)
    cell = band["cells"][f"a{_number_key(selected_a)}_F{_number_key(selected_f)}"]
    return {"scene": scene, "E": request["E"], "a": selected_a, "F": selected_f,
            "label": request["label"], "realized": cell["realized_accel_m_s2"]}


def _nearest_snapshot(files: list[Path], target: float) -> Path:
    return min(files, key=lambda path: abs(float(np.load(path)["t"]) - target))


def _contact_sheet(key_paths: list[Path], output: Path) -> None:
    images = [Image.open(path).convert("RGB") for path in key_paths]
    thumb_w, thumb_h = 480, 240
    sheet = Image.new("RGB", (thumb_w * 2, thumb_h * 4), (16, 16, 20))
    draw = ImageDraw.Draw(sheet)
    for index, (path, image) in enumerate(zip(key_paths, images)):
        image.thumbnail((thumb_w, thumb_h - 22))
        x, y = index % 2 * thumb_w, index // 2 * thumb_h
        sheet.paste(image, (x, y + 22))
        draw.text((x + 6, y + 4), path.stem, fill=(230, 230, 240))
    sheet.save(output)


def render_scene(cell: dict) -> dict:
    scene = cell["scene"]
    raw_dir = OUT_DIR / f"w3_{scene}_snapshots"
    frame_dir = OUT_DIR / f"w3_{scene}.png-seq"
    key_dir = OUT_DIR / f"w3_{scene}_keys"
    for directory in (raw_dir, frame_dir, key_dir):
        if directory.exists():
            shutil.rmtree(directory)
        directory.mkdir(parents=True)

    receipt = run_transport_cell(cell["E"] * 1000, cell["F"], cell["a"], 0,
                                 snap_dir=raw_dir)
    snapshots = sorted(raw_dir.glob("*.npz"))
    if not snapshots:
        raise RuntimeError("transport rerun produced no snapshots")
    for snapshot in snapshots:
        render_frame(str(snapshot), str(frame_dir / f"{snapshot.stem}.png"))

    keys = []
    for name, target in KEY_TIMES.items():
        snapshot = _nearest_snapshot(snapshots, target)
        output = key_dir / f"{name}.png"
        render_frame(str(snapshot), str(output))
        keys.append(output)

    mp4 = OUT_DIR / f"w3_{scene}.mp4"
    ffmpeg = shutil.which("ffmpeg")
    encoded = False
    if ffmpeg:
        result = subprocess.run(
            [ffmpeg, "-y", "-framerate", "7.5", "-i", str(frame_dir / "f_%04d.png"),
             "-c:v", "libx264", "-pix_fmt", "yuv420p", str(mp4)],
            capture_output=True, text=True,
        )
        encoded = result.returncode == 0
    if not encoded:
        _contact_sheet(keys, OUT_DIR / f"w3_{scene}_contact_sheet.png")
    return {**cell, "rerun_label": receipt["label"], "encoded": encoded,
            "frames": frame_dir, "keys": key_dir}


def _write_report(results: list[dict]) -> None:
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    for result in results:
        media = (f"`reports/vbd/clips/w3_{result['scene']}.mp4`"
                 if result["encoded"] else
                 f"`reports/vbd/clips/w3_{result['scene']}.png-seq/` and contact sheet")
        rows.append(f"| {result['scene']} | E{result['E']} | {result['a']:g} | "
                    f"{result['F']:g} | {result['label']} | {result['realized']:.4g} | {media} |")
    REPORT.write_text(
        "# W3 transport clips\n\n"
        "These scenes re-run selected cells from the final W1 bands with the frozen "
        "transport rig; they introduce no new physics. The intact and slip scenes satisfy "
        "the strict same-grip comparison: identical E15 material, gripper geometry, grip "
        "force (1.2 N), seed, and protocol, with only commanded acceleration changed. "
        "Across all three scenes the gripper setup/protocol is unchanged; the damage branch "
        "uses the pre-registered higher force and E7 material.\n\n"
        "| scene | material | commanded a (m/s²) | F (N) | final-band label | realized a (m/s²) | projection |\n"
        "|---|---:|---:|---:|---|---:|---|\n" + "\n".join(rows) +
        "\n\nEach projection is the standard side `(y,z)` plus front `(x,z)` view. Key "
        "frames are under `reports/vbd/clips/w3_<scene>_keys/` at grip, lift, hold, "
        "accel-out peak, dwell, return, and settle boundaries. If ffmpeg/libx264 is "
        "unavailable, the PNG sequence remains authoritative and a key-frame contact "
        "sheet is emitted.\n"
    )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--render", action="store_true", help="render all three scenes")
    mode.add_argument("--scene", choices=tuple(REQUESTS), help="render one scene")
    args = parser.parse_args(argv)
    names = list(REQUESTS) if args.render else [args.scene]
    results, failures = [], []
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for name in names:
        try:
            cell = select_cell(name)
            result = render_scene(cell)
            results.append(result)
            print(f"{name}: E{cell['E']} a{cell['a']:g} F{cell['F']:g} "
                  f"band={cell['label']} rerun={result['rerun_label']}")
        except Exception as exc:
            failures.append((name, exc))
            print(f"{name}: ERROR {type(exc).__name__}: {exc}", file=sys.stderr)
    if results:
        _write_report(results)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
