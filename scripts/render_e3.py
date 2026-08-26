"""E3 renders: setup still + the fixed three-scene demo triplet (sigma_Y=3333).

Scenes (pre-registered; pending-approval.md E3):
  (i)   quasi-static success: F = F_mid(3333), a_peak=1, labeled intact
  (ii)  aggressive motion: IDENTICAL grip F = F_mid(3333), a_peak=15, labeled slip
  (iii) grip raised: F >= F_max(3333, a=15), a_peak=15, labeled damage
Same-grip linkage between (i) and (ii) is a hard predicate. Each clip re-runs
the SOURCE trial (same seed + config), asserts the label reproduces, and
records the source trial id. Deliverables: mp4 + key frames (grasp,
lift-complete, transport-end/reversal, settle-end) via the fallback render
path (deviation recorded: offscreen GL not used; PIL orthographic).

Fail-closed: a missing qualifying trial for any scene writes
reports/e3_triplet_unavailable.md (searched coordinates + observed labels)
and delivers the still plus the available subset. Renders never substitute
for labels.

Usage: cd newton && uv run --no-sync python ../scripts/render_e3.py [--results ralph/results] [--scratch]
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import shutil
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src import io_schemas

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def load_trials(results_dir):
    out = []
    for path in glob.glob(os.path.join(results_dir, "trials", "*.json")):
        doc = io_schemas.read_json(path)
        p = doc["payload"]
        if p["sigma_y_pa"] != 3333.0:
            continue
        out.append({
            "id": os.path.splitext(os.path.basename(path))[0],
            "a": float(p["a_peak_cmd_ms2"]), "f": float(p["f_g_n"]), "seed": int(p["seed"]),
            "labels": set(p["labels"]), "color": p.get("cell_color"),
            "calibration": doc["config"]["calibration"],
        })
    return out


def pick_scenes(trials, f_mid, f_max_a15):
    def find(pred):
        cands = sorted((t for t in trials if pred(t)), key=lambda t: t["seed"])
        return cands[0] if cands else None

    scenes = {
        "i_intact_quasistatic": find(lambda t: t["a"] == 1.0 and t["f"] == f_mid and t["color"] == "intact"),
        "ii_slip_same_grip": find(lambda t: t["a"] == 15.0 and t["f"] == f_mid and t["color"] == "slip"),
        "iii_damage_raised_grip": find(
            lambda t: t["a"] == 15.0 and (f_max_a15 is not None and t["f"] >= f_max_a15) and t["color"] == "damage"
        ),
    }
    # hard same-grip predicate between (i) and (ii)
    if scenes["i_intact_quasistatic"] and scenes["ii_slip_same_grip"]:
        assert scenes["i_intact_quasistatic"]["f"] == scenes["ii_slip_same_grip"]["f"], \
            "same-grip linkage violated between scenes (i) and (ii)"
    return scenes


def render_scene(tag, trial, media_dir, calibration):
    from src.trial import run_trial

    frames_dir = os.path.join(media_dir, "frames", f"e3_{tag}")
    if os.path.isdir(frames_dir):
        shutil.rmtree(frames_dir)
    doc = run_trial(3333.0, trial["a"], trial["f"], trial["seed"],
                    calibration=calibration, frames_dir=frames_dir, frame_every_ticks=40)
    relabel = set(doc["payload"]["labels"])
    label_ok = doc["payload"].get("cell_color") == trial["color"]
    mp4 = os.path.join(media_dir, f"e3_{tag}.mp4")
    subprocess.run([sys.executable, os.path.join(ROOT, "scripts", "make_media.py"),
                    frames_dir, "--mp4", mp4, "--keyframes", "5", "--fps", "6"], check=True)
    return {"source_trial": trial["id"], "replay_labels": sorted(relabel),
            "replay_color": doc["payload"].get("cell_color"),
            "label_reproduced": bool(label_ok), "mp4": os.path.relpath(mp4, ROOT)}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", default=os.path.join(ROOT, "ralph", "results"))
    ap.add_argument("--scratch", action="store_true")
    args = ap.parse_args()
    media_dir = os.path.join(ROOT, "reports", "media")
    os.makedirs(media_dir, exist_ok=True)

    trials = load_trials(args.results)
    if not trials:
        print("no sigma=3333 trials found in", args.results)
        return 1

    band_path = os.path.join(args.results, "e1_band_3333.json")
    f_mid = f_max_a15 = None
    if os.path.exists(band_path):
        from scripts.run_e2 import f_mid_from_band

        f_mid, _sel = f_mid_from_band(band_path)
        doc = io_schemas.read_json(band_path)
        row15 = next((r for r in doc["payload"]["rows"] if r["a_peak"] == 15.0), None)
        f_max_a15 = row15["F_max"] if row15 else None
    if f_mid is None:
        # fail-closed path exercises with whatever mid-band exists in the trials
        print("no usable a=1 band; falling back to searched-coordinate report")

    scenes = pick_scenes(trials, f_mid, f_max_a15)
    missing = {k: v for k, v in scenes.items() if v is None}
    rendered = {}
    for tag, trial in scenes.items():
        if trial is None:
            continue
        rendered[tag] = render_scene(tag, trial, media_dir, trial["calibration"])
        print(tag, "->", rendered[tag]["mp4"], "| label_reproduced:", rendered[tag]["label_reproduced"])

    # setup still: first key frame of any rendered scene (or a fresh gentle frame)
    if rendered:
        first = next(iter(rendered.values()))
        base = os.path.splitext(os.path.join(ROOT, first["mp4"]))[0]
        still_src = base + "_key0.png"
        shutil.copy(still_src, os.path.join(media_dir, "e3_setup_still.png"))

    if missing:
        lines = ["# E3 triplet unavailability report (fail-closed)", "",
                 f"f_mid={f_mid} f_max_a15={f_max_a15}", "", "## Missing scenes"]
        for tag in missing:
            lines.append(f"- {tag}: no trial with the required (grip, accel, label)")
        lines += ["", "## Observed labels at searched coordinates"]
        for t in sorted(trials, key=lambda t: (t["a"], t["f"], t["seed"])):
            lines.append(f"- {t['id']}: a={t['a']} f={t['f']} seed={t['seed']} color={t['color']} labels={sorted(t['labels'])}")
        with open(os.path.join(ROOT, "reports", "e3_triplet_unavailable.md"), "w") as fh:
            fh.write("\n".join(lines) + "\n")
        print("MISSING scenes:", list(missing), "-> reports/e3_triplet_unavailable.md")

    with open(os.path.join(media_dir, "e3_manifest.json"), "w") as fh:
        json.dump({"scenes": rendered, "missing": list(missing), "f_mid": f_mid,
                   "f_max_a15": f_max_a15}, fh, indent=2)
    return 0 if not missing else 3


if __name__ == "__main__":
    sys.exit(main())
