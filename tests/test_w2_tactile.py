import pytest

from scripts.vbd.w2_tactile_vbd import (
    UNAVAILABLE_REASON,
    centroid_excursion_mm,
    material_summary,
    unavailable_tangential_ratio,
)


def _frame(t, left, right):
    return {"t": t, "pads": {
        "left": {"centroid": left},
        "right": {"centroid": right},
    }}


def test_centroid_excursion_uses_first_available_centroid_per_pad():
    series = [
        _frame(9.30, [0, 0, 0], None),
        _frame(9.31, [0.003, 0.004, 0], [1, 1, 1]),
        _frame(9.32, [0, 0, 0], [1, 1, 1.002]),
    ]
    assert centroid_excursion_mm(series) == pytest.approx({"left": 5.0, "right": 2.0})


def test_tangential_ratio_is_explicitly_unavailable():
    assert unavailable_tangential_ratio() == {"value": None, "reason": UNAVAILABLE_REASON}


def test_material_summary_groups_and_orders_successful_cells():
    cells = [
        {"E_kPa": 7, "a": 5.0, "realized_accel": 4.8,
         "centroid_excursion_mm": {"left": 2, "right": 3}},
        {"E_kPa": 15, "a": 1.0, "realized_accel": 0.9,
         "centroid_excursion_mm": {"left": 1, "right": 1}},
        {"E_kPa": 7, "a": 1.0, "realized_accel": 0.95,
         "centroid_excursion_mm": {"left": 1, "right": 2}},
        {"status": "error", "E_kPa": 7, "a": 2.5, "realized_accel": None,
         "centroid_excursion_mm": {"left": None, "right": None}},
    ]
    summary = material_summary(cells)
    assert [row["commanded_accel_m_s2"] for row in summary["7"]] == [1.0, 5.0]
    assert summary["15"][0]["realized_accel_m_s2"] == 0.9
    assert summary["25"] == []
