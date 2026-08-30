"""Panda-hand variant of the frozen W1 transport runner.

The frame loop and judgment are delegated verbatim to w1_transport; only its rig
class is replaced process-locally with PandaRig.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from scripts.vbd import w1_transport
from src.vbd_rig_panda import PandaRig


def run_panda_cell(E: float, F: float, a_peak: float, seed: int):
    import src.vbd_rig2 as frozen_rig

    original = frozen_rig.Vbd2Rig
    frozen_rig.Vbd2Rig = PandaRig
    try:
        receipt = w1_transport.run_transport_cell(
            E, F, a_peak, seed, substeps=80, cell_m=0.005
        )
    finally:
        frozen_rig.Vbd2Rig = original
    receipt["rig"] = "panda"
    receipt["finger_coupling"] = (
        "servo/software symmetric per-substep projection; Newton b74df534 "
        "SolverVBD lacks mechanical mimic/equality constraints"
    )
    return receipt


def write_receipt(receipt: dict) -> Path:
    out = ROOT / "reports/logs/vbd/panda"
    out.mkdir(parents=True, exist_ok=True)
    path = out / "E{:.0f}_F{:g}_a{:g}_seed{}.json".format(
        receipt["E_pa"] / 1000, receipt["grip_force_n"],
        receipt["commanded_a_peak_m_s2"], receipt["seed"]
    )
    path.write_text(json.dumps(w1_transport._json_safe(receipt), indent=2,
                               allow_nan=False) + "\n")
    return path


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--cell", nargs=4, metavar=("E_KPA", "F_N", "A", "SEED"))
    group.add_argument("--smoke", action="store_true")
    args = parser.parse_args()
    if args.smoke:
        E_kpa, force, accel, seed = 15.0, 1.2, 5.0, 0
    else:
        E_kpa, force, accel, seed = (float(args.cell[0]), float(args.cell[1]),
                                      float(args.cell[2]), int(args.cell[3]))
    receipt = run_panda_cell(E_kpa * 1000.0, force, accel, seed)
    path = write_receipt(receipt)
    print(json.dumps({"label": receipt["label"], "health": receipt["health"],
                      "receipt": str(path)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
