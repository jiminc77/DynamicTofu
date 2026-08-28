from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from src.frozen_config import FROZEN_PRODUCTION, assert_frozen


ROOT = Path(__file__).resolve().parents[1]
PREREG = ROOT / "ralph" / "results" / "prereg_w1.json"


def load_prereg():
    return json.loads(PREREG.read_text(encoding="utf-8"))


def test_transport_windows_form_exact_integer_partition():
    data = load_prereg()
    windows = data["transport_windows"]
    assert len(windows) == 8
    assert windows[0]["start_k"] == round(9.30 * 4800)
    assert windows[-1]["end_k"] == round(11.60 * 4800)
    assert all(left["end_k"] == right["start_k"] for left, right in zip(windows, windows[1:]))
    assert all(not window["end_closed"] for window in windows[:-1])
    assert windows[-1]["end_closed"] is True


def test_plateaus_are_strictly_inside_their_phases():
    data = load_prereg()
    phases = {window["name"]: window for window in data["transport_windows"]}
    assert len(data["plateau_windows"]) == 4
    for plateau in data["plateau_windows"]:
        phase = phases[plateau["phase"]]
        assert phase["start_k"] < plateau["start_k"] < plateau["end_k"] < phase["end_k"]
        assert plateau["start_k"] == round(plateau["start_s"] * data["timebase"]["hz"])
        assert plateau["end_k"] == round(plateau["end_s"] * data["timebase"]["hz"])


def test_grid_and_extension_cap():
    data = load_prereg()
    grid = data["grid"]
    assert (len(grid["a_peak_m_s2"]), len(grid["F_N"]), len(grid["E_kPa"])) == (6, 7, 3)
    assert len(grid["a_peak_m_s2"]) * len(grid["F_N"]) * len(grid["E_kPa"]) == grid["primary_cells"] == 126
    assert len(data["t_ext"]["topology_table"]) == 6
    assert data["t_ext"]["cap_rows"] == 8


def test_assert_frozen_rejects_mutated_substeps_and_lists_key():
    config = dict(FROZEN_PRODUCTION)
    config["E_pa"] = 7000.0
    config["substeps"] = 79
    with pytest.raises(AssertionError, match="substeps"):
        assert_frozen(config)


def test_baseline_check_passes_without_importing_gpu_stack():
    result = subprocess.run(
        [sys.executable, "scripts/vbd/w1_baseline.py", "--check"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS" in result.stdout
