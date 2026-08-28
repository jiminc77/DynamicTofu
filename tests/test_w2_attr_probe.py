import numpy as np

from scripts.vbd.w2_attr_probe import decide_attr, project_pad_reactions


def _run(normal=1.2, penalty=1.2, finger_vy=0.0):
    return {
        "left": {"normal_n": normal, "penalty_n": penalty},
        "right": {"normal_n": normal, "penalty_n": penalty},
        "finger_vy": finger_vy,
    }


def test_projection_uses_outward_normals_and_tangent_plane():
    forces = np.array([[0.0, 0.0, 0.0], [0.3, 1.2, 0.4], [0.0, -1.2, 0.5]])
    projected = project_pad_reactions(forces, 1, 2)
    assert projected["left"]["normal_n"] == 1.2
    assert projected["right"]["normal_n"] == 1.2
    assert projected["left"]["tangential_n"] == 0.5
    assert projected["right"]["tangential_n"] == 0.5


def test_available_requires_all_checks_across_three_seeds():
    absent = _run(normal=0.0, penalty=0.0)
    result = decide_attr([_run(1.19), _run(1.2), _run(1.21)], absent, 1.2)
    assert result["verdict"] == "AVAILABLE"
    assert result["failed_checks"] == []


def test_absent_force_or_seed_mismatch_fails_closed():
    contaminated = decide_attr([_run(), _run(), _run()], _run(normal=1.2), 1.2)
    assert contaminated["verdict"] == "GEOMETRY_ONLY"
    assert "B_block_absent" in contaminated["failed_checks"]
    assert "C_contact_not_joint_effort" in contaminated["failed_checks"]

    mismatch = decide_attr([_run(), _run(), _run(normal=0.8)], _run(normal=0.0), 1.2)
    assert mismatch["verdict"] == "GEOMETRY_ONLY"
    assert "D_equilibrium" in mismatch["failed_checks"]
    assert "E_three_seed_reproducibility" in mismatch["failed_checks"]
