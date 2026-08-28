import numpy as np
import pytest

from src.contact_validity import disposition, reduce_validity


DT = 0.01
PADS = (10, 20)


def contacts(shapes=(10, 20), *, count=None, capacity=None, body_vel=None):
    n = len(shapes)
    return {
        "soft_contact_count": np.array([n if count is None else count], np.int32),
        "soft_contact_max": n if capacity is None else capacity,
        "soft_contact_shape": np.asarray(shapes, np.int32),
        "soft_contact_indices": np.array([[0, -1, -1], [1, 2, -1]][:n], np.int32),
        "soft_contact_barycentric": np.array([[1, 0, 0], [0.25, 0.75, 0]][:n], np.float32),
        "soft_contact_body_vel": np.zeros((n, 3), np.float32) if body_vel is None else np.asarray(body_vel),
        "block_particle_range": (0, 4),
    }


def test_co_moving_pad_and_contacted_features_are_certified():
    q = np.array([[0, 0, 0], [0, 1, 0], [0, 2, 0], [0, 3, 0]], float)
    qd = np.tile([0.1, 0.0, 0.0], (4, 1))
    result = reduce_validity(
        contacts(body_vel=[[0.1, 0, 0], [0.1, 0, 0]]), PADS, q, qd, DT
    )
    assert result["certified"] is True
    assert result["per_pad"]["left"]["max_rel_disp_m"] == pytest.approx(0.0, abs=1e-12)
    assert result["per_pad"]["right"]["max_rel_disp_m"] == pytest.approx(0.0, abs=1e-12)


def test_actual_contacted_feature_sliding_one_mm_fails():
    q = np.zeros((4, 3))
    qd = np.zeros((4, 3))
    qd[0, 0] = 0.001 / DT
    result = reduce_validity(contacts(), PADS, q, qd, DT)
    assert result["per_pad"]["left"]["max_rel_disp_m"] >= 0.001
    assert result["certified"] is False


def test_particle_edge_and_face_barycentric_reconstruction():
    q = np.zeros((4, 3))
    qd = np.array([[0.1, 0, 0], [0.2, 0, 0], [0.4, 0, 0], [0.8, 0, 0]])
    data = {
        "soft_contact_count": np.array([3]), "soft_contact_max": 3,
        "soft_contact_shape": np.array([10, 10, 20]),
        "soft_contact_indices": np.array([[0, -1, -1], [1, 2, -1], [1, 2, 3]]),
        "soft_contact_barycentric": np.array([[1, 0, 0], [.25, .75, 0], [.2, .3, .5]]),
        "soft_contact_body_vel": np.zeros((3, 3)), "block_particle_range": (0, 4),
    }
    result = reduce_validity(data, PADS, q, qd, DT)
    assert result["per_pad"]["left"]["max_rel_disp_m"] == pytest.approx(0.0035)
    assert result["per_pad"]["right"]["max_rel_disp_m"] == pytest.approx(0.0056)


def test_zero_filtered_pad_records_increment_vg2_and_fail():
    q = np.zeros((4, 3)); qd = np.zeros_like(q)
    data = contacts(shapes=(10,))
    result = reduce_validity(data, PADS, q, qd, DT)
    assert result["per_pad"]["right"]["zero_record_substeps"] == 1
    assert result["certified"] is False


def test_overflow_increments_vg3():
    q = np.zeros((4, 3)); qd = np.zeros_like(q)
    result = reduce_validity(contacts(count=3, capacity=2), PADS, q, qd, DT)
    assert result["per_pad"]["left"]["overflow_substeps"] == 1
    assert result["per_pad"]["right"]["overflow_substeps"] == 1
    assert result["certified"] is False


def test_graded_disposition_policy():
    assert disposition(False, False, 0) == "censored_interior"
    assert disposition(False, True, 0) == "stopped_deciding_coordinate"
    assert disposition(False, False, 1) == "stopped_second_in_row"
    with pytest.raises(ValueError):
        disposition(True, False, 0)
