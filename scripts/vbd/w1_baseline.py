#!/usr/bin/env python3
"""CPU-only pre-edit baseline guard; no simulation is performed."""

from __future__ import annotations

import argparse
import hashlib
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.frozen_config import FROZEN_PRODUCTION, RIG_PRE_EDIT_SHA256, assert_frozen


def check() -> None:
    rig_path = ROOT / "src" / "vbd_rig2.py"
    actual = hashlib.sha256(rig_path.read_bytes()).hexdigest()
    assert actual == RIG_PRE_EDIT_SHA256, (
        f"src/vbd_rig2.py sha256 mismatch: expected {RIG_PRE_EDIT_SHA256}, got {actual}"
    )
    config = dict(FROZEN_PRODUCTION)
    config["E_pa"] = 15_000.0
    assert_frozen(config)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="check source pin and production config")
    args = parser.parse_args()
    if not args.check:
        parser.error("baseline capture is not implemented in this CPU-only scaffold; use --check")
    try:
        check()
    except (AssertionError, OSError) as exc:
        print(f"FAIL: {exc}")
        return 1
    print("PASS: pre-edit rig hash and frozen production config")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
