import numpy as np

from src.judgment_vbd import (
    PHASE_NAMES,
    grasp_frame_y_res_mm,
    label,
    label_v21,
    label_v22,
    label_v23,
    hold_slip_z_mm,
    latched_dvf,
    per_phase_strain_maxima,
    phase_for_time,
    slip3d,
    slip_perm_tangential_mm,
    slip_perm_x_mm,
    transport_slip_xz_mm,
    x_res_mm,
    y_res_mm,
    yz_residual_mm,
    z_res_mm,
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


def test_v22_permanent_tangential_slip_and_normal_reseating():
    def series(settle):
        return [
            _frame(1.8, [0, 0, 0], [0, 0, 0]),
            _frame(9.3, [0, 0, 0], [0, 0, 0]),
            _frame(10.0, [0.010, 0, -0.006], [0, 0, 0]),
            _frame(11.3, settle, [0, 0, 0]),
            _frame(11.6, settle, [0, 0, 0]),
        ]

    vertical = series([0, 0, -0.006])
    assert np.isclose(slip_perm_tangential_mm(vertical), 6.0)
    assert np.isclose(z_res_mm(vertical), -6.0)
    assert label_v22({"slip_perm_tangential_mm": 6.0}) == "slip"

    held = series([0, 0, 0])
    assert slip_perm_tangential_mm(held) == 0.0
    assert label_v22({"slip_perm_tangential_mm": 0.0}) == "intact"

    x_only = series([0.003, 0, 0])
    assert np.isclose(x_res_mm(x_only), 3.0)
    assert label_v22({"slip_perm_tangential_mm":
                      slip_perm_tangential_mm(x_only)}) == "slip"

    y_only = series([0, 0.004, 0])
    assert np.isclose(y_res_mm(y_only), 4.0)
    assert slip_perm_tangential_mm(y_only) == 0.0
    assert label_v22({"slip_perm_tangential_mm": 0.0}) == "intact"
    assert label_v22({"slip_perm_tangential_mm": None, "ejected": True}) == "slip"


def test_v23_separate_hold_and_transport_windows():
    def series(hold_z=0.0, settle_x=0.0, settle_z=None, settle_y=0.0):
        settle_z = hold_z if settle_z is None else settle_z
        return [
            _frame(1.8, [0, 0, 0], [0, 0, 0]),
            _frame(4.3, [0, 0, hold_z], [0, 0, 0]),
            _frame(9.3, [0, 0, hold_z], [0, 0, 0]),
            _frame(11.3, [settle_x, settle_y, settle_z], [0, 0, 0]),
            _frame(11.6, [settle_x, settle_y, settle_z], [0, 0, 0]),
        ]

    hold_creep = series(hold_z=-0.006)
    assert np.isclose(hold_slip_z_mm(hold_creep), 6.0)
    assert transport_slip_xz_mm(hold_creep) == 0.0
    assert label_v23({"hold_slip_z_mm": 6.0, "transport_slip_xz_mm": 0.0}) == "slip"

    transport = series(settle_x=0.003)
    assert hold_slip_z_mm(transport) == 0.0
    assert np.isclose(transport_slip_xz_mm(transport), 3.0)
    assert label_v23({"hold_slip_z_mm": 0.0, "transport_slip_xz_mm": 3.0}) == "slip"

    assert label_v23({"hold_slip_z_mm": 1.0, "transport_slip_xz_mm": 1.0}) == "intact"
    y_only = series(settle_y=0.010)
    assert transport_slip_xz_mm(y_only) == 0.0
    assert label_v23({"hold_slip_z_mm": 0.0, "transport_slip_xz_mm": 0.0}) == "intact"
    assert label_v23({"ejected": True, "damage_latched": True}) == "slip"
    assert label_v23({"hold_slip_z_mm": 0.0, "transport_slip_xz_mm": 0.0,
                      "damage_latch_t": 8.0}) == "damage"


def test_v23_lateral_escape_and_timestamped_damage_precedence():
    common_mode = [
        {**_frame(1.8, [0, 0.020, 0], [0, 0, 0]),
         "left_y": 0.03, "right_y": 0.01},
        {**_frame(11.3, [0, 0.030, 0], [0, 0, 0]),
         "left_y": 0.04, "right_y": 0.02},
        {**_frame(11.6, [0, 0.030, 0], [0, 0, 0]),
         "left_y": 0.04, "right_y": 0.02},
    ]
    assert np.isclose(grasp_frame_y_res_mm(common_mode), 0.0)
    base = {"hold_slip_z_mm": 0.0, "transport_slip_xz_mm": 0.0}
    assert label_v23({**base, "grasp_frame_y_res_mm": 0.0}) == "intact"

    escaped = [dict(frame) for frame in common_mode]
    for frame in escaped[1:]:
        frame["com"] = np.array([0, 0.042, 0])
    assert np.isclose(grasp_frame_y_res_mm(escaped), 12.0)
    assert label_v23({**base, "grasp_frame_y_res_mm": 12.0,
                      "drop_t": 11.3}) == "slip"

    assert label_v23({**base, "drop_t": 9.0, "damage_latch_t": 8.0,
                      "ejected": True}) == "damage"
    assert label_v23({**base, "drop_t": 9.0, "damage_latch_t": 9.0,
                      "ejected": True}) == "slip"


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
