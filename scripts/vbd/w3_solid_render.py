"""CPU-only solid-surface renderer for the frozen W3 snapshot sequences.

Usage:
  python scripts/vbd/w3_solid_render.py --render
  python scripts/vbd/w3_solid_render.py --scene intact
"""
from __future__ import annotations

import argparse
import inspect
import json
import shutil
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = ROOT / "reports/vbd/clips"
TOPOLOGY = OUT_DIR / "tofu_topology.npz"
SCENES = ("intact", "slip", "damage")
KEY_TIMES = {
    "grip": 1.80, "lift": 4.30, "hold": 9.30,
    "accel_out_peak": 9.40, "dwell": 9.80, "return": 10.10,
    "settle": 10.60,
}
PAD_HALF_EXTENTS = np.array((0.022, 0.006, 0.022))
# vbd_rig2 has no palm collider. This small non-physical presentation proxy
# visually joins the pads; the real palm body pose still drives its placement.
PALM_PROXY_HALF_EXTENTS = np.array((0.008, 0.060, 0.008))
GROUND_Z = 0.0
EXPECTED_DENSE_FRAMES = 696  # 11.6 s * 60 simulation frames/s
BOX_FACES = np.array(((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1),
                      (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)))


def boundary_triangles(tet_idx: np.ndarray, vertices: np.ndarray | None = None) -> np.ndarray:
    """Return deduplicated boundary faces, wound outward when vertices are given."""
    tets = np.asarray(tet_idx, dtype=np.int64)
    if tets.ndim != 2 or tets.shape[1] != 4:
        raise ValueError("tet_idx must have shape (N, 4)")
    faces: dict[tuple[int, int, int], list[tuple[tuple[int, int, int], int]]] = defaultdict(list)
    for a, b, c, d in tets:
        for face, opposite in (((a, b, c), d), ((a, b, d), c),
                               ((a, c, d), b), ((b, c, d), a)):
            faces[tuple(sorted(face))].append((face, int(opposite)))
    boundary = [entries[0] for entries in faces.values() if len(entries) == 1]
    triangles = np.asarray([face for face, _ in boundary], dtype=np.int64)
    if vertices is not None and len(triangles):
        xyz = np.asarray(vertices)
        for index, (face, opposite) in enumerate(boundary):
            a, b, c = xyz[list(face)]
            # A normal pointing toward the opposite tet vertex is inward.
            if np.dot(np.cross(b - a, c - a), xyz[opposite] - a) > 0.0:
                triangles[index, 1], triangles[index, 2] = triangles[index, 2], triangles[index, 1]
    return triangles


def _rotation(quat: np.ndarray) -> np.ndarray:
    """Convert Newton/Warp's xyzw quaternion to a 3x3 rotation matrix."""
    x, y, z, w = np.asarray(quat, dtype=float)
    norm = np.linalg.norm((x, y, z, w))
    if norm == 0:
        return np.eye(3)
    x, y, z, w = np.array((x, y, z, w)) / norm
    return np.array(((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                     (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                     (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y))))


def _box(pose: np.ndarray, half: np.ndarray) -> np.ndarray:
    signs = np.array(((-1,-1,-1), (-1,-1,1), (-1,1,-1), (-1,1,1),
                      (1,-1,-1), (1,-1,1), (1,1,-1), (1,1,1)), dtype=float)
    return signs * half @ _rotation(pose[3:]).T + pose[:3]


def _add_solid(ax, triangles: np.ndarray, view: str, base=(0.91, 0.72, 0.30),
               zorder: int = 1) -> None:
    direction = np.array((0.0, 1.0, 0.0)) if view == "side" else np.array((-1.0, 0.0, 0.0))
    depth = triangles.mean(axis=1) @ direction
    order = np.argsort(depth)  # far to near for painter's algorithm
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    light = np.clip(0.35 + 0.65 * np.abs(normals @ np.array((0.35, -0.45, 0.82))), 0.25, 1.0)
    colors = np.clip(np.asarray(base)[None, :] * light[:, None], 0, 1)
    ax.add_collection3d(Poly3DCollection(triangles[order], facecolors=colors[order],
                                         edgecolors=(0.28, 0.20, 0.08, 0.20),
                                         linewidths=0.18, zorder=zorder))


def _draw_frame(snapshot: Path, boundary: np.ndarray, output: Path) -> None:
    with np.load(snapshot) as data:
        particles = np.asarray(data["particle_q"], dtype=float)
        bodies = np.asarray(data["body_q"], dtype=float)
        time_s = float(data["t"])
    if particles.shape[0] <= int(boundary.max()) or bodies.shape != (4, 7):
        raise ValueError(f"unexpected snapshot shapes: particles={particles.shape}, bodies={bodies.shape}")
    tofu = particles[boundary]
    boxes = ((_box(bodies[1], PALM_PROXY_HALF_EXTENTS), (0.28, 0.30, 0.34)),
             (_box(bodies[2], PAD_HALF_EXTENTS), (0.85, 0.35, 0.27)),
             (_box(bodies[3], PAD_HALF_EXTENTS), (0.25, 0.38, 0.85)))
    centers = np.vstack((particles, bodies[:, :3]))
    xmid = (centers[:, 0].min() + centers[:, 0].max()) / 2
    span = max(0.075, np.ptp(centers[:, 0]) + 0.045)
    limits = ((xmid-span/2, xmid+span/2), (-0.085, 0.085), (-0.008, 0.115))
    fig = plt.figure(figsize=(12, 5), dpi=120, facecolor="#101218")
    for number, (view, title) in enumerate((("side", "SIDE  x–z"), ("front", "FRONT  y–z")), 1):
        ax = fig.add_subplot(1, 2, number, projection="3d",
                             facecolor="#101218", computed_zorder=False)
        for corners, color in boxes:
            side = view == "side"
            ax.add_collection3d(Poly3DCollection(corners[BOX_FACES], facecolors=color,
                                                 edgecolors=color if side else (0.08, 0.08, 0.10),
                                                 linewidths=1.15 if side else 0.5,
                                                 alpha=0.10 if side else 1.0,
                                                 zorder=1 if side else 2))
        # In side view the 44 mm near pad would occlude the 40 mm tofu. Draw
        # translucent pad outlines first and force the shaded tofu above them.
        _add_solid(ax, tofu, view, zorder=10 if view == "side" else 1)
        gx = np.linspace(*limits[0], 2); gy = np.linspace(*limits[1], 2)
        xx, yy = np.meshgrid(gx, gy)
        ax.plot_surface(xx, yy, np.full_like(xx, GROUND_Z), color=(0.25, 0.27, 0.30), alpha=0.45)
        ax.set(xlim=limits[0], ylim=limits[1], zlim=limits[2], title=title)
        ax.set_box_aspect((limits[0][1]-limits[0][0], .17, .123))
        ax.view_init(elev=0, azim=-90 if view == "side" else 0)
        ax.set_proj_type("ortho")
        ax.set_axis_off()
        ax.title.set_color("white")
    fig.suptitle(f"W3 {snapshot.parent.name.removeprefix('w3_').removesuffix('_snapshots')}  t={time_s:.2f} s",
                 color="white", fontsize=14)
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, facecolor=fig.get_facecolor())
    plt.close(fig)


def render_scene(scene: str, boundary: np.ndarray) -> Path:
    dense_dir = OUT_DIR / f"w3_{scene}_dense"
    snapshot_dir = dense_dir if dense_dir.exists() else OUT_DIR / f"w3_{scene}_snapshots"
    snapshots = sorted(snapshot_dir.glob("f_*.npz"))
    if not snapshots:
        raise FileNotFoundError(f"no frozen snapshots for {scene}")
    frame_dir = OUT_DIR / f"w3_{scene}_solid.png-seq"
    key_dir = OUT_DIR / f"w3_{scene}_solid_keys"
    frame_dir.mkdir(parents=True, exist_ok=True)
    key_dir.mkdir(parents=True, exist_ok=True)
    times = []
    for snapshot in snapshots:
        _draw_frame(snapshot, boundary, frame_dir / f"{snapshot.stem}.png")
        with np.load(snapshot) as data:
            times.append(float(data["t"]))
    for name, target in KEY_TIMES.items():
        index = int(np.argmin(np.abs(np.asarray(times) - target)))
        shutil.copy2(frame_dir / f"{snapshots[index].stem}.png", key_dir / f"{name}.png")
    output = OUT_DIR / f"w3_{scene}_solid.mp4"
    # Dense captures are real 60 Hz simulation frames; ffmpeg drops alternate
    # frames for a 30 fps presentation without changing physical playback time.
    input_fps = "60" if snapshot_dir == dense_dir else "7.5"
    result = subprocess.run(("/usr/bin/ffmpeg", "-y", "-framerate", input_fps, "-i",
                             str(frame_dir / "f_%04d.png"), "-vf", "fps=30",
                             "-c:v", "libx264",
                             "-pix_fmt", "yuv420p", str(output)), capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError(f"ffmpeg failed for {scene}: {result.stderr[-1000:]}")
    return output


def _dense_run_transport_cell():
    """Clone the frozen runner in memory, changing only snapshot cadence 8 -> 1.

    The source module remains untouched. The exact-match replacement fails
    closed if its saving condition changes, rather than silently altering any
    solver, material, contact, protocol, or judgment behavior.
    """
    from scripts.vbd import w1_transport

    source = inspect.getsource(w1_transport.run_transport_cell)
    old = "if snap_dir and frame_index % 8 == 0:"
    if source.count(old) != 1:
        raise RuntimeError("frozen run_transport_cell snapshot condition changed")
    namespace = dict(w1_transport.__dict__)
    exec(compile(source.replace(old, "if snap_dir and frame_index % 1 == 0:"),
                 str(Path(w1_transport.__file__)), "exec"), namespace)
    return namespace["run_transport_cell"]


def dense_capture() -> None:
    """Re-run the three frozen cells and save every real simulation frame."""
    manifest = json.loads((OUT_DIR / "w3_manifest.json").read_text())
    entries = {entry["scene"]: entry for entry in manifest["scenes"]}
    run_transport_cell = _dense_run_transport_cell()
    for scene in SCENES:
        entry = entries[scene]
        if not entry.get("label_reproduced") or entry["rerun_label"] != entry["source_final_band_label"]:
            raise RuntimeError(f"{scene}: source manifest does not reproduce its label")
        dense_dir = OUT_DIR / f"w3_{scene}_dense"
        if dense_dir.exists():
            shutil.rmtree(dense_dir)
        dense_dir.mkdir(parents=True)
        receipt = run_transport_cell(float(entry["E"]) * 1000.0, float(entry["F"]),
                                     float(entry["a"]), int(entry["seed"]),
                                     snap_dir=dense_dir)
        if receipt["label"] != entry["source_final_band_label"]:
            raise RuntimeError(
                f"{scene}: dense rerun label {receipt['label']!r} != "
                f"source label {entry['source_final_band_label']!r}"
            )
        count = len(list(dense_dir.glob("f_*.npz")))
        print(f"{scene}: {count} dense simulation frames (maximum {EXPECTED_DENSE_FRAMES})")


def _extended_slip_run_transport_cell():
    """Clone the frozen runner: dense cadence AND continue past gross-slip ejection.

    Per-step physics is UNTOUCHED. Two output/termination-only changes:
      (a) snapshot cadence 8 -> 1 (every real simulation frame);
      (b) the render capture no longer TERMINATES the trial at the 15 mm
          gross-slip threshold, so the real post-ejection separation and fall
          are captured. The ejection event (ejected=True, drop_t) is still
          recorded at first crossing, preserving the slip mechanism/label.
    Fails closed if either exact source block changed.
    """
    from scripts.vbd import w1_transport

    source = inspect.getsource(w1_transport.run_transport_cell)
    cad = "if snap_dir and frame_index % 8 == 0:"
    brk = ('            if gross_slip_mm(m, transport_reference) > GROSS_SLIP_MM:\n'
           '                series.append(m)\n'
           '                ejected = True\n'
           '                if drop_t is None:\n'
           '                    drop_t = float(m["t"])\n'
           '                break')
    brk_new = ('            if gross_slip_mm(m, transport_reference) > GROSS_SLIP_MM:\n'
               '                if not ejected:\n'
               '                    ejected = True\n'
               '                    if drop_t is None:\n'
               '                        drop_t = float(m["t"])\n'
               '                # extended render capture: do not terminate at ejection')
    if source.count(cad) != 1 or source.count(brk) != 1:
        raise RuntimeError("frozen runner capture/termination block changed; refuse to guess")
    modified = source.replace(cad, "if snap_dir and frame_index % 1 == 0:").replace(brk, brk_new)
    namespace = dict(w1_transport.__dict__)
    exec(compile(modified, str(Path(w1_transport.__file__)), "exec"), namespace)
    return namespace["run_transport_cell"]


def dense_capture_extended(scene: str = "slip") -> None:
    """Capture the slip scene continued past ejection for the v3 aftermath/fall."""
    manifest = json.loads((OUT_DIR / "w3_manifest.json").read_text())
    entry = {e["scene"]: e for e in manifest["scenes"]}[scene]
    if not entry.get("label_reproduced") or entry["rerun_label"] != entry["source_final_band_label"]:
        raise RuntimeError(f"{scene}: source manifest does not reproduce its label")
    run = _extended_slip_run_transport_cell()
    dense_dir = OUT_DIR / f"w3_{scene}_dense_ext"
    if dense_dir.exists():
        shutil.rmtree(dense_dir)
    dense_dir.mkdir(parents=True)
    receipt = run(float(entry["E"]) * 1000.0, float(entry["F"]), float(entry["a"]),
                  int(entry["seed"]), snap_dir=dense_dir)
    if not receipt.get("ejected"):
        raise RuntimeError(f"{scene}: extended capture did not reproduce ejection "
                           f"(label={receipt['label']!r}); refuse to ship a non-reproducing clip")
    frames = sorted(dense_dir.glob("f_*.npz"))
    times = [float(np.load(p)["t"]) for p in frames]
    meta = {"scene": scene, "frames": len(frames),
            "t_first": times[0] if times else None, "t_last": times[-1] if times else None,
            "ejected": bool(receipt["ejected"]), "drop_t": receipt["drop_t"],
            "label": receipt["label"], "source_label": entry["source_final_band_label"],
            "note": ("render-only capture continued past the 15 mm gross-slip termination; "
                     "per-step physics identical to the frozen runner; ejection event recorded "
                     "at drop_t. No renderer-side temporal interpolation.")}
    (dense_dir / "capture_meta.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(json.dumps(meta, indent=2))


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--render", action="store_true", help="render all three scenes")
    group.add_argument("--scene", choices=SCENES, help="render one scene")
    group.add_argument("--dense-capture", action="store_true",
                       help="GPU rerun all frozen scenes, saving every simulation frame")
    group.add_argument("--dense-capture-ext", action="store_true",
                       help="GPU rerun the slip scene continued past ejection (v3 fall/aftermath)")
    args = parser.parse_args()
    if args.dense_capture:
        dense_capture()
        return 0
    if args.dense_capture_ext:
        dense_capture_extended("slip")
        return 0
    with np.load(TOPOLOGY) as topology:
        tets = topology["tet_idx"]
        n_particles = int(topology["n_particles"])
    first = next((OUT_DIR / "w3_intact_snapshots").glob("f_*.npz"))
    with np.load(first) as data:
        reference = data["particle_q"]
    if len(reference) != n_particles:
        raise ValueError(f"topology has {n_particles} particles, snapshots have {len(reference)}")
    boundary = boundary_triangles(tets, reference)
    failures = []
    for scene in SCENES if args.render else (args.scene,):
        try:
            print(render_scene(scene, boundary))
        except Exception as exc:  # isolate scenes in an all-scene presentation render
            failures.append((scene, exc))
            print(f"ERROR {scene}: {exc}", file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
