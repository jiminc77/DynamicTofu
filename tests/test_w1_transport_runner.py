import importlib.util
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1]))
from src.transport import trapezoid_reversal

SPEC = importlib.util.spec_from_file_location(
    "w1_transport_runner", Path(__file__).parents[1] / "scripts/vbd/w1_transport.py"
)
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)
phase_timestamp_table = RUNNER.phase_timestamp_table
tracking_receipt = RUNNER.tracking_receipt


def test_phase_timestamp_table_reports_first_hit_and_missing():
    expected = {"one": (1.0, 2.0), "two": (2.0, 3.0)}
    series = [{"t": 1.0, "phase": "one"}, {"t": 1.5, "phase": "one"}]
    table = phase_timestamp_table(series, expected)
    assert table["one"] == {"expected_s": 1.0, "hit_s": 1.0, "error_s": 0.0}
    assert table["two"]["hit_s"] is None
    assert table["two"]["error_s"] is None


def test_tracking_receipt_from_exact_synthetic_velocity():
    profile = trapezoid_reversal(5.0)
    times = np.arange(9.30, 10.61, 1 / 60)
    velocities = profile.v_cmd(times)
    receipt = tracking_receipt(times, velocities, profile.plateau_windows)
    assert receipt["pass"] is True
    assert receipt["samples_valid"] is True
    assert receipt["max_abs_relative_error"] < 1e-12
    assert all(fit["n_samples"] >= 5 for fit in receipt["plateaus"].values())
