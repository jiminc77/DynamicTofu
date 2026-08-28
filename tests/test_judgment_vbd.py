import numpy as np

from src.judgment_vbd import (
    PHASE_NAMES,
    label,
    label_v21,
    latched_dvf,
    per_phase_strain_maxima,
    phase_for_time,
    slip3d,
    slip_perm_x_mm,
    yz_residual_mm,
)


def _frame(t, com, palm):
    return {"t": t, "com": np.asarray(com), "palm_pos": np.asarray(palm)}


def test_slip3d_translation_invariance_and_relative_shift():
    times = [9.3, 9.8, 10.6, 11.6]
    rigid = [_frame(t, [100 * t, -7 * t, 4 * t], [100 * t, -7 * t, 4 * t]) for t in times]
    assert slip3d(rigid) == 0.0

    shifted = list(rigid)
    shifted[-1] = _frame(11.6, [1160.003, -81.2, 46.4], [1160.0, -81.2, 46.4])
    assert np.isclose(slip3d(shifted), 3.0)

    palm_z_motion = [
        _frame(t, [t, 0.0, 10.0 * t + 0.25], [t, 0.0, 10.0 * t]) for t in times
    ]
    assert slip3d(palm_z_motion) == 0.0


def test_v21_permanent_x_ignores_transient_max_and_labels_residual_or_ejection():
    returned = [
        _frame(9.3, [0, 0, 0], [0, 0, 0]),
        _frame(10.0, [0.010, 0, 0], [0, 0, 0]),
        _frame(11.3, [0, 0, 0], [0, 0, 0]),
        _frame(11.6, [0, 0, 0], [0, 0, 0]),
    ]
    assert slip3d(returned) == 10.0
    assert slip_perm_x_mm(returned) == 0.0
    assert yz_residual_mm(returned) == 0.0
    assert label_v21({"slip_perm_x_mm": slip_perm_x_mm(returned)}) == "intact"

    displaced = returned[:2] + [
        _frame(11.3, [0.003, 0, 0], [0, 0, 0]),
        _frame(11.6, [0.003, 0, 0], [0, 0, 0]),
    ]
    assert np.isclose(slip_perm_x_mm(displaced), 3.0)
    assert label_v21({"slip_perm_x_mm": slip_perm_x_mm(displaced)}) == "slip"
    assert label_v21({"slip_perm_x_mm": None, "ejected": True}) == "slip"


def test_dvf_latches_transient_accel_back_damage():
    fields = {phase: np.array([0.01, 0.02, 0.03]) for phase in PHASE_NAMES}
    fields["accel_back"] = np.array([0.01, 0.20, 0.03])
    fields["settle"] = np.array([0.01, 0.02, 0.03])
    maxima = per_phase_strain_maxima(fields)
    temporal_max = np.maximum.reduce([fields[phase] for phase in PHASE_NAMES])
    dvf, latched = latched_dvf(temporal_max, [1.0, 1.0, 198.0])
    assert maxima["accel_back"] == 0.20
    assert maxima["settle"] == 0.03
    assert dvf == 0.005
    assert latched


def test_damage_drop_precedence_both_orders():
    assert label({"damage_latched": True, "damage_latch_t": 9.7, "drop_t": 9.8,
                  "slip3d_mm": 3.0}) == "damage"
    assert label({"damage_latched": True, "damage_latch_t": 9.9, "drop_t": 9.8,
                  "slip3d_mm": 3.0}) == "slip"
    assert label({"damage_latched": False, "slip3d_mm": 2.0}) == "intact"


def test_phase_boundaries_form_exact_half_open_partition():
    boundaries = [
        (9.30, "accel_out"), (9.50, "cruise_out"),
        (9.60, "decel_out"), (9.80, "dwell"),
        (10.10, "accel_back"), (10.30, "cruise_back"),
        (10.40, "decel_back"), (10.60, "settle"),
        (11.60, "settle"),
    ]
    for t, expected in boundaries:
        assert phase_for_time(t) == expected
    assert phase_for_time(11.60 + 1 / 4800) is None
