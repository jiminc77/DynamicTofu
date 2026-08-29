import pytest

from scripts.vbd.w2_tactile_vbd import (
    UNAVAILABLE_REASON,
    centroid_excursion_mm,
    material_summary,
    reduce_falsifier,
    unavailable_tangential_ratio,
)


def _frame(t, left, right):
    return {"t": t, "pads": {
        "left": {"centroid_pad": left},
        "right": {"centroid_pad": right},
    }}


def test_centroid_excursion_uses_first_available_centroid_per_pad():
    series = [
        _frame(9.30, [0, 0, 0], None),
        _frame(9.31, [0.003, 0.004, 0], [1, 1, 1]),
        _frame(9.32, [0, 0, 0], [1, 1, 1.002]),
    ]
    assert centroid_excursion_mm(series) == pytest.approx({"left": 5.0, "right": 2.0})


def test_centroid_excursion_rejects_rigid_world_translation():
    translation = ([0, 0, 0], [0.111, 0, 0], [0.222, 0, 0])
    series = []
    for index, shift in enumerate(translation):
        pad = [shift[0], shift[1], shift[2]]
        world_contact = [shift[0] + 0.002, shift[1] - 0.003, shift[2] + 0.004]
        relative = [contact - origin for contact, origin in zip(world_contact, pad)]
        series.append(_frame(9.30 + index / 60, relative, relative))
    assert centroid_excursion_mm(series) == pytest.approx({"left": 0.0, "right": 0.0},
                                                           abs=1e-10)


def test_tangential_ratio_is_explicitly_unavailable():
    assert unavailable_tangential_ratio() == {"value": None, "reason": UNAVAILABLE_REASON}


def test_material_summary_groups_and_orders_successful_cells():
    cells = [
        {"E_kPa": 7, "a": 5.0, "realized_accel": 4.8,
         "centroid_excursion_mm": {"left": 2, "right": 3}, "peak_lr_asymmetry": {}},
        {"E_kPa": 15, "a": 1.0, "realized_accel": 0.9,
         "centroid_excursion_mm": {"left": 1, "right": 1}, "peak_lr_asymmetry": {}},
        {"E_kPa": 7, "a": 1.0, "realized_accel": 0.95,
         "centroid_excursion_mm": {"left": 1, "right": 2}, "peak_lr_asymmetry": {}},
        {"status": "error", "E_kPa": 7, "a": 2.5, "realized_accel": None,
         "centroid_excursion_mm": {"left": None, "right": None}},
    ]
    summary = material_summary(cells)
    assert [row["commanded_accel_m_s2"] for row in summary["7"]] == [1.0, 5.0]
    assert summary["15"][0]["realized_accel_m_s2"] == 0.9
    assert summary["25"] == []


def test_falsifier_reducer_detects_non_overlap_and_signed_median():
    cells = []
    for acceleration, values in ((1, [1.0, 2.0, 3.0]), (10, [5.0, 7.0, 9.0])):
        for value in values:
            cells.append({"a": acceleration,
                          "centroid_excursion_mm": {"left": value, "right": value - 0.5}})
    result = reduce_falsifier(cells)
    assert result["endpoints"]["1"]["range_mm"] == [1.0, 3.0]
    assert result["endpoints"]["10"]["range_mm"] == [5.0, 9.0]
    assert result["strict_non_overlap"] is True
    assert result["signed_median_difference_mm_high_minus_low"] == pytest.approx(5.0)
    assert result["peak_tangential_ratio"]["reason"] == UNAVAILABLE_REASON
