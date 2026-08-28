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
EQUIV_FIELD_DIR = ROOT / "reports" / "logs" / "vbd" / "g0_equivalence_fields"
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


def _capture_cell(E, F, seed, substeps=SUBSTEPS, field_dir=FIELD_DIR):
    """Run one anchor cell on the frozen rig; return the reduced baseline record."""
    import numpy as np
    from tofu_probe import run_cell  # repo convention: script dir on sys.path

    field_path = field_dir / f"{E:.0f}_{F}_s{seed}.npz"
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
    palm_z = np.array([s.get("palm_z", 0.0) for s in series], dtype=np.float64)
    traj_path = field_dir / f"{E:.0f}_{F}_s{seed}_traj.npz"
    np.savez_compressed(traj_path, t=t, com_z=com_z, palm_z=palm_z)
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


# G0' equivalence tolerances (empirical non-perturbation; the extension is NOT bit-identical).
SLIP_TOL_MM = 0.15
RMS_TOL_MM = 0.5
P99_TOL = 0.02
# Amended damage-branch DVF criterion (external ruling 2026-08-28, ralph/DECISIONS.md):
# the pre-registered +-20% per-seed DVF tolerance is incoherent with the damage branch's
# intrinsic ~60% relative seed spread. PASS a damage cell's DVF iff label-equivalence across
# the 0.5% threshold AND (extended per-seed DVF within the baseline seed range widened by 20%
# of that range) OR (|delta| <= 1 percentage point absolute), whichever is looser.
DVF_RANGE_WIDEN = 0.20
DVF_ABS_TOL = 0.01
EQUIV_OUT_JSON = ROOT / "reports" / "logs" / "vbd" / "g0_equivalence.json"


def equivalence() -> int:
    """GPU: compare the extended, transport-off rig against the pre-edit baseline."""
    import numpy as np

    assert SUBSTEPS == 80
    config = dict(FROZEN_PRODUCTION)
    config["E_pa"] = 15_000.0
    assert_frozen(config)
    from src.frozen_config import frozen_provenance

    baseline = json.loads(OUT_JSON.read_text())
    by_anchor = {anchor["name"]: anchor for anchor in baseline["anchors"]}
    EQUIV_FIELD_DIR.mkdir(parents=True, exist_ok=True)
    git_sha = subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip()
    all_pass = True
    n_pass = 0
    n_total = 0
    anchors_out = []
    print("anchor     seed label slip   com-z-rms dvf    p99    result")
    for name, E, F, _expected in ANCHORS:
        base_anchor = by_anchor[name]
        base_seeds = {int(rec["seed"]): rec for rec in base_anchor["seeds"]}
        base_dvfs = [s["dvf"] for s in base_anchor["seeds"]]
        d_lo, d_hi = min(base_dvfs), max(base_dvfs)
        d_range = d_hi - d_lo
        widened = [d_lo - DVF_RANGE_WIDEN * d_range, d_hi + DVF_RANGE_WIDEN * d_range]
        seed_out = []
        for seed in SEEDS:
            rec = _capture_cell(E, F, seed, field_dir=EQUIV_FIELD_DIR)
            base = base_seeds[seed]
            old_traj = np.load(ROOT / base["com_z_trajectory"])
            new_traj = np.load(ROOT / rec["com_z_trajectory"])
            new_z = np.interp(old_traj["t"], new_traj["t"], new_traj["com_z"])
            rms_mm = float(np.sqrt(np.mean((new_z - old_traj["com_z"]) ** 2)) * 1000.0)
            label_ok = rec["label"] == base["label"]
            slip_ok = abs(rec["hold_slip_mm"] - base["hold_slip_mm"]) <= SLIP_TOL_MM
            rms_ok = rms_mm <= RMS_TOL_MM
            same_side = (rec["dvf"] >= DVF_MIN) == (base["dvf"] >= DVF_MIN)
            if base["dvf"] == 0.0:
                dvf_pass = same_side and (rec["dvf"] == 0.0)
            else:
                within_widened = widened[0] <= rec["dvf"] <= widened[1]
                within_abs = abs(rec["dvf"] - base["dvf"]) <= DVF_ABS_TOL
                dvf_pass = same_side and (within_widened or within_abs)
            p99_ok = abs(rec["p99"] - base["p99"]) <= P99_TOL
            passed = label_ok and slip_ok and rms_ok and dvf_pass and p99_ok
            all_pass &= passed
            n_total += 1
            n_pass += int(passed)
            seed_out.append({
                "seed": seed, "label": rec["label"], "base_label": base["label"], "label_ok": label_ok,
                "hold_slip_mm": rec["hold_slip_mm"], "base_hold_slip_mm": base["hold_slip_mm"], "slip_ok": slip_ok,
                "com_z_rms_mm": round(rms_mm, 4), "rms_ok": rms_ok,
                "dvf": rec["dvf"], "base_dvf": base["dvf"], "dvf_widened_range": [round(widened[0], 6), round(widened[1], 6)],
                "dvf_pass": bool(dvf_pass), "p99": rec["p99"], "base_p99": base["p99"], "p99_ok": p99_ok,
                "cell_pass": bool(passed),
            })
            print(f"{name:<10} {seed:>4} {str(label_ok):<5} {str(slip_ok):<6} "
                  f"{rms_mm:>8.3f} {str(dvf_pass):<6} {str(p99_ok):<6} "
                  f"{'PASS' if passed else 'FAIL'}", flush=True)
        anchors_out.append({"name": name, "E_pa": E, "grip_force_n": F,
                            "dvf_widened_range": [round(widened[0], 6), round(widened[1], 6)], "seeds": seed_out})
    out = {
        "gate": "G0_prime_equivalence",
        "result": "PASS" if all_pass else "FAIL",
        "n_pass": n_pass, "n_total": n_total,
        "criterion": {
            "label": "identical per seed", "hold_slip_mm_tol": SLIP_TOL_MM, "com_z_rms_mm_tol": RMS_TOL_MM,
            "p99_tol": P99_TOL, "dvf_nondamage": "exact zero",
            "dvf_damage_amended": "same-side-of-0.5% AND (within baseline seed range widened by 20% of range OR |delta|<=0.01 abs), whichever looser",
            "amendment_ref": "ralph/DECISIONS.md 2026-08-28 G0' DVF tolerance mis-spec (external ruling)",
        },
        "provenance": {
            "git_sha": git_sha, "newton_commit": NEWTON_COMMIT,
            "pre_edit_rig_sha256": RIG_PRE_EDIT_SHA256, "post_edit_rig_sha256": _rig_hash(),
            "substeps": SUBSTEPS, "production_config": frozen_provenance(),
            "baseline_json": str(OUT_JSON.relative_to(ROOT)),
            "evaluated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        },
        "anchors": anchors_out,
    }
    EQUIV_OUT_JSON.write_text(json.dumps(out, indent=2, default=str))
    print(f"[equivalence] wrote {EQUIV_OUT_JSON.relative_to(ROOT)} result={out['result']} {n_pass}/{n_total}", flush=True)
    return 0 if all_pass else 2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="CPU-only source pin + frozen config guard")
    parser.add_argument("--capture", action="store_true", help="GPU: capture the pre-edit baseline (3 anchors x 3 seeds)")
    parser.add_argument("--equivalence", action="store_true", help="GPU: compare extended transport-off rig with baseline")
    parser.add_argument("--smoke", action="store_true", help="GPU: fast 1-cell low-substep code-path check (never writes g0_baseline.json)")
    args = parser.parse_args()
    if args.smoke:
        return smoke()
    if args.capture:
        return capture()
    if args.equivalence:
        return equivalence()
    if args.check:
        try:
            check()
        except (AssertionError, OSError) as exc:
            print(f"FAIL: {exc}")
            return 1
        print("PASS: pre-edit rig hash and frozen production config")
        return 0
    parser.error("choose --check, --capture, --equivalence, or --smoke")


if __name__ == "__main__":
    raise SystemExit(main())
