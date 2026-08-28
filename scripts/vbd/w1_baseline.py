#!/usr/bin/env python3
"""Pre-edit baseline harness (G0').

--check : CPU-only guard. Asserts src/vbd_rig2.py matches the pinned pre-edit
          sha256 and that the frozen production config is intact. No simulation.
--capture : GPU. Runs the three frozen anchor cells (E15/F0.6 slip, E15/F1.2
          intact, E7/F2.0 damage) x 3 seeds on the UNMODIFIED rig at the frozen
          production config (substeps=80) and writes g0_baseline.json + per-seed
          strain fields and COM-z trajectories. This is the reference the post-edit
          G0' equivalence gate compares against. Fails closed if the rig hash pin
          does not match (the rig MUST be unmodified when the baseline is captured).

Run capture: cd newton && uv run --no-sync python ../scripts/vbd/w1_baseline.py --capture
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = Path(__file__).resolve().parent
for _p in (str(ROOT), str(SCRIPT_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from src.frozen_config import FROZEN_PRODUCTION, RIG_PRE_EDIT_SHA256, assert_frozen

NEWTON_COMMIT = "b74df534"
SUBSTEPS = 80
CELL_M = 0.005
EPS = 2.0e-4
EPS_DAMAGE = 0.15
DVF_MIN = 0.005
SLIP_MM = 2.0

# (name, E_pa, grip_force_n, expected_label)
ANCHORS = (
    ("E15_F0.6", 15_000.0, 0.6, "slip"),
    ("E15_F1.2", 15_000.0, 1.2, "intact"),
    ("E7_F2.0", 7_000.0, 2.0, "damage"),
)
SEEDS = (0, 1, 2)

FIELD_DIR = ROOT / "reports" / "logs" / "vbd" / "g0_baseline_fields"
OUT_JSON = ROOT / "reports" / "logs" / "vbd" / "g0_baseline.json"


def _rig_hash() -> str:
    return hashlib.sha256((ROOT / "src" / "vbd_rig2.py").read_bytes()).hexdigest()


def check() -> None:
    actual = _rig_hash()
    assert actual == RIG_PRE_EDIT_SHA256, (
        f"src/vbd_rig2.py sha256 mismatch: expected {RIG_PRE_EDIT_SHA256}, got {actual}"
    )
    config = dict(FROZEN_PRODUCTION)
    config["E_pa"] = 15_000.0
    assert_frozen(config)


def _vol_weighted_p99(field, vol) -> float:
    import numpy as np

    order = np.argsort(field)
    cw = np.cumsum(vol[order]) / vol.sum()
    idx = min(len(field) - 1, int(np.searchsorted(cw, 0.99)))
    return float(field[order][idx])


def _capture_cell(E, F, seed, substeps=SUBSTEPS):
    """Run one anchor cell on the frozen rig; return the reduced baseline record."""
    import numpy as np
    from tofu_probe import run_cell  # repo convention: script dir on sys.path

    field_path = FIELD_DIR / f"{E:.0f}_{F}_s{seed}.npz"
    res, series = run_cell(
        E, F, eps=EPS, thr=EPS_DAMAGE, substeps=substeps, cell_m=CELL_M,
        save_field=str(field_path), seed=seed,
    )
    d = np.load(field_path)
    tmax = d["temporal_max_principal_strain"]
    vol = d["tet_rest_vol"]
    from src.judgment_vbd import latched_dvf

    dvf, latched = latched_dvf(tmax, vol, eps=EPS_DAMAGE)
    p99 = _vol_weighted_p99(tmax, vol)
    peak = float(tmax.max())
    hold_slip = float(res["hold_slip_mm"])
    finite = bool(res["finite"])
    # judgment v2 (quasi-static baseline: no drop -> damage precedence trivial)
    if not finite:
        lab = "nonfinite"
    elif hold_slip > SLIP_MM:
        lab = "slip"
    elif dvf >= DVF_MIN:
        lab = "damage"
    else:
        lab = "intact"
    # downsampled COM-z trajectory (10 Hz logged) -> durable npz
    t = np.array([s["t"] for s in series], dtype=np.float64)
    com_z = np.array([s["com_z"] for s in series], dtype=np.float64)
    traj_path = FIELD_DIR / f"{E:.0f}_{F}_s{seed}_traj.npz"
    np.savez_compressed(traj_path, t=t, com_z=com_z)
    return {
        "seed": seed,
        "label": lab,
        "hold_slip_mm": round(hold_slip, 4),
        "dvf": round(float(dvf), 6),
        "p99": round(p99, 4),
        "peak_principal_strain": round(peak, 4),
        "final_com_rise_mm": res["final_com_rise_mm"],
        "finite": finite,
        "strain_field": str(field_path.relative_to(ROOT)),
        "com_z_trajectory": str(traj_path.relative_to(ROOT)),
    }


def smoke() -> int:
    """Fast code-path validation: 1 cell at low substeps, isolated output."""
    check()
    FIELD_DIR.mkdir(parents=True, exist_ok=True)
    rec = _capture_cell(15_000.0, 1.2, 0, substeps=8)
    ok = rec["finite"] and rec["label"] in {"slip", "intact", "damage"}
    print(f"SMOKE {'OK' if ok else 'FAIL'}: label={rec['label']} slip={rec['hold_slip_mm']}mm "
          f"dvf={rec['dvf']} p99={rec['p99']} finite={rec['finite']}", flush=True)
    return 0 if ok else 1


def capture() -> int:
    import numpy as np

    assert SUBSTEPS == 80, f"baseline capture MUST use substeps=80, got {SUBSTEPS}"
    check()  # fail closed: rig MUST be unmodified when the baseline is captured
    FIELD_DIR.mkdir(parents=True, exist_ok=True)
    git_sha = subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip()
    from src.frozen_config import frozen_provenance

    anchors_out = []
    all_unanimous = True
    all_expected = True
    t_start = time.time()
    for name, E, F, expected in ANCHORS:
        seed_recs = []
        for seed in SEEDS:
            print(f"[baseline] {name} seed {seed} ...", flush=True)
            rec = _capture_cell(E, F, seed)
            print(f"[baseline]   -> label={rec['label']} slip={rec['hold_slip_mm']}mm "
                  f"dvf={rec['dvf']} p99={rec['p99']} ({time.time()-t_start:.0f}s)", flush=True)
            seed_recs.append(rec)
        labels = {r["label"] for r in seed_recs}
        unanimous = len(labels) == 1
        unanimous_label = seed_recs[0]["label"] if unanimous else None
        all_unanimous = all_unanimous and unanimous
        all_expected = all_expected and (unanimous_label == expected)
        slips = [r["hold_slip_mm"] for r in seed_recs]
        dvfs = [r["dvf"] for r in seed_recs]
        p99s = [r["p99"] for r in seed_recs]
        anchors_out.append({
            "name": name, "E_pa": E, "grip_force_n": F, "expected_label": expected,
            "unanimous_label": unanimous_label, "unanimous": unanimous,
            "hold_slip_mm_mean": round(float(np.mean(slips)), 4),
            "hold_slip_mm_spread": round(float(np.ptp(slips)), 4),
            "dvf_mean": round(float(np.mean(dvfs)), 6),
            "p99_mean": round(float(np.mean(p99s)), 4),
            "seeds": seed_recs,
        })
    out = {
        "gate": "G0_baseline_pre_edit",
        "provenance": {
            "git_sha": git_sha,
            "newton_commit": NEWTON_COMMIT,
            "pre_edit_rig_sha256": RIG_PRE_EDIT_SHA256,
            "rig_sha256_at_capture": _rig_hash(),
            "substeps": SUBSTEPS, "cell_m": CELL_M, "friction_epsilon": EPS,
            "eps_damage": EPS_DAMAGE, "dvf_min": DVF_MIN, "slip_threshold_mm": SLIP_MM,
            "production_config": frozen_provenance(),
            "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "wall_clock_s": round(time.time() - t_start, 1),
        },
        "anchors": anchors_out,
        "summary": {
            "all_anchors_unanimous": all_unanimous,
            "labels_match_expected": all_expected,
        },
    }
    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    json.dump(out, open(OUT_JSON, "w"), indent=2, default=str)
    print(f"[baseline] wrote {OUT_JSON.relative_to(ROOT)} "
          f"unanimous={all_unanimous} expected_match={all_expected}", flush=True)
    return 0 if (all_unanimous and all_expected) else 2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="CPU-only source pin + frozen config guard")
    parser.add_argument("--capture", action="store_true", help="GPU: capture the pre-edit baseline (3 anchors x 3 seeds)")
    parser.add_argument("--smoke", action="store_true", help="GPU: fast 1-cell low-substep code-path check (never writes g0_baseline.json)")
    args = parser.parse_args()
    if args.smoke:
        return smoke()
    if args.capture:
        return capture()
    if args.check:
        try:
            check()
        except (AssertionError, OSError) as exc:
            print(f"FAIL: {exc}")
            return 1
        print("PASS: pre-edit rig hash and frozen production config")
        return 0
    parser.error("choose --check (CPU) or --capture (GPU)")


if __name__ == "__main__":
    raise SystemExit(main())
