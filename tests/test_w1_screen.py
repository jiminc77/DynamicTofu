import importlib.util
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))
SPEC = importlib.util.spec_from_file_location(
    "w1_transport_screen", Path(__file__).parents[1] / "scripts/vbd/w1_transport.py"
)
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)


def _receipt(realized=None):
    tracking = None if realized is None else {"plateaus": {
        str(i): {"a_fit": sign * realized, "n_samples": 6, "r2": 1.0}
        for i, sign in enumerate((1, -1, -1, 1))
    }}
    return {
        "status": "ok", "E_pa": 7000.0, "grip_force_n": 0.4, "seed": 0,
        "commanded_a_peak_m_s2": 10.0, "tracking": tracking,
        "hold_slip_z_mm": 0.2, "transport_slip_xz_mm": 0.1,
        "transport_x_res_mm": 0.1, "transport_z_res_mm": 0.0,
        "transport_y_res_mm": 0.0,
        "grasp_frame_y_res_mm": 0.0, "assembly_drift_mm": 0.1,
        "escape_mode": None, "damage_latch_t": None, "drop_t": None,
        "damage_after_drop": False,
        "dvf_wholetrial": 0.0, "damage_window": "post_lift_hold_onset",
        "lift_end_s": 4.3,
        "slip_perm_tangential_mm": 0.2, "x_res_mm": 0.1, "z_res_mm": 0.1,
        "y_res_mm": 0.1, "slip_perm_x_mm": 0.2,
        "slip3d_max_mm": 1.2, "yz_residual_mm": 0.1,
        "legacy_hold_slip_mm": 0.2, "label": "intact",
        "dvf": 0.0, "p99_strain": 0.1, "peak_strain": 0.2,
        "validity_gate": {"certified": True}, "ejected": False,
        "per_phase_strain_maxima": {}, "health": {"finite": True},
    }


def test_grid_material_and_acceleration_order():
    grid = RUNNER.screen_grid()
    assert len(grid) == 126
    assert [grid[i][0] for i in (0, 42, 84)] == [7, 25, 15]
    assert [grid[i][1] for i in range(0, 42, 7)] == [1, 5, 10, 20, 30, 2.5]
    assert len(RUNNER.screen_grid(25)) == 42


def test_band_incremental_update():
    cell = RUNNER.screen_cell_receipt(_receipt(6.4), {10.0: 6.4019})
    band = RUNNER.update_band(None, cell)
    assert band["schema"] == "e1v2_band.v1"
    assert band["label_matrix"]["10"]["0.4"] == "intact"
    assert band["cells"]["a10_F0.4"]["certified"] is True
    assert band["coverage"]["completed"] == [[10.0, 0.4]]


def test_resume_skips_existing_receipt(tmp_path):
    grid = [(7, 1, 0.4, 0), (7, 1, 0.6, 0)]
    (tmp_path / RUNNER.screen_receipt_name(*grid[0])).write_text("{}")
    assert RUNNER.pending_screen_cells(grid, tmp_path, True) == [grid[1]]
    assert RUNNER.pending_screen_cells(grid, tmp_path, False) == grid


def test_realized_falls_back_to_axis_map():
    cell = RUNNER.screen_cell_receipt(_receipt(None), {10.0: 6.4019})
    assert cell["realized_accel_m_s2"] == 6.4019
    assert cell["realized_source"] == "axis_map"


def test_confirm_plan_expansion_names_and_resume(tmp_path):
    plan = [
        {"E_kPa": 7, "commanded_a_peak_m_s2": 2.5,
         "grip_force_n": 0.8, "seeds_to_run": [1, 2]},
        {"E_kPa": 25, "commanded_a_peak_m_s2": 10,
         "grip_force_n": 1.2, "seeds_to_run": [2]},
    ]
    grid = RUNNER.expand_confirm_plan(plan)
    assert grid == [(7, 2.5, 0.8, 1), (7, 2.5, 0.8, 2),
                    (25, 10.0, 1.2, 2)]
    assert RUNNER.screen_receipt_name(*grid[0]) == "E7_F0.8_a2.5_s1.json"
    existing = tmp_path / RUNNER.screen_receipt_name(*grid[1])
    existing.write_text("{}")
    assert RUNNER.pending_screen_cells(grid, tmp_path, True) == [grid[0], grid[2]]
    assert RUNNER.pending_screen_cells(grid, tmp_path, False) == grid
