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
summarize_level = RUNNER.summarize_level
ladder_shape_gate = RUNNER.ladder_shape_gate
gross_slip_mm = RUNNER.gross_slip_mm
realized_from_fits = RUNNER.realized_from_fits


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


def _cell(realized, r2=0.995):
    signs = (1, -1, -1, 1)
    return {"tracking": {"plateaus": {
        str(index): {"a_fit": sign * realized, "r2": r2, "n_samples": 6}
        for index, sign in enumerate(signs)
    }}}


def test_level_median_cv_and_shape_gates():
    low = summarize_level(1.0, [_cell(1.00), _cell(1.02), _cell(0.98)])
    high = summarize_level(2.5, [_cell(2.48), _cell(2.50), _cell(2.52)])
    assert low["realized_median"] == 1.0
    assert low["realized_cv"] < 0.05
    assert low["r2_min"] == 0.995
    assert low["level_pass"] is True
    assert ladder_shape_gate([low, high]) == (True, True)


def test_shape_gate_rejects_nonmonotone_and_insufficient_separation():
    first = summarize_level(1.0, [_cell(1.0), _cell(1.0), _cell(1.0)])
    close = summarize_level(2.5, [_cell(1.05), _cell(1.05), _cell(1.05)])
    lower = summarize_level(5.0, [_cell(0.9), _cell(0.9), _cell(0.9)])
    assert ladder_shape_gate([first, close]) == (True, False)
    assert ladder_shape_gate([first, lower]) == (False, False)


def test_gross_slip_and_partial_plateau_median():
    reference = np.array([0.0, 0.0, 0.0])
    frame = {"com": [0.016, 0.0, 0.0], "palm_pos": [0.0, 0.0, 0.0]}
    assert gross_slip_mm(frame, reference) == 16.0
    fits = {
        "done_a": {"a_fit": -3.0, "n_samples": 6},
        "thin": {"a_fit": 100.0, "n_samples": 4},
        "done_b": {"a_fit": 5.0, "n_samples": 5},
        "missing": {"a_fit": float("nan"), "n_samples": 0},
    }
    assert realized_from_fits(fits) == 4.0


def test_level_with_fewer_than_two_usable_cells_is_insufficient():
    level = summarize_level(10, [_cell(6.0), {"status": "error", "seed": 1},
                                 {"status": "error", "seed": 2}])
    assert level["status"] == "insufficient"
    assert level["level_pass"] is False
