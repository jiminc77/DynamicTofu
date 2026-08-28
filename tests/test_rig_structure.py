import numpy as np
import warp as wp

from src.frozen_config import assert_frozen
from src.vbd_rig2 import Vbd2Config, Vbd2Rig


def production_config():
    return Vbd2Config(
        E_pa=15_000.0,
        nu=0.45,
        contact_ke=1.0e3,
        contact_kd=1.0,
        mu_pair=1.0,
        friction_epsilon=2.0e-4,
        soft_contact_margin=1.0e-3,
        cell_m=0.005,
        particle_radius=0.0025,
        substeps=80,
        correct_mass=True,
    )


def test_extended_rig_indices_mass_and_snapshot_labels(tmp_path):
    wp.set_device("cpu")
    cfg = production_config()
    assert_frozen(cfg)
    rig = Vbd2Rig(cfg)

    labels = list(rig.model.body_label)
    assert [labels[i] for i in (rig.b_carriage, rig.b_palm, rig.b_left, rig.b_right)] == [
        "carriage", "palm", "left", "right"
    ]
    qs = rig.model.joint_q_start.numpy()
    qds = rig.model.joint_qd_start.numpy()
    targets = rig.model.joint_target_q_start.numpy()
    assert (rig.x_qi, rig.x_ti, rig.x_dof) == (
        int(qs[rig.j_x]), int(targets[rig.j_x]), int(qds[rig.j_x])
    )
    assert (rig.z_qi, rig.z_ti) == (int(qs[rig.j_z]), int(targets[rig.j_z]))
    assert (rig.l_qi, rig.l_dof) == (int(qs[rig.j_left]), int(qds[rig.j_left]))
    assert (rig.r_qi, rig.r_dof) == (int(qs[rig.j_right]), int(qds[rig.j_right]))

    bq = rig.state_0.body_q.numpy()
    assert bq[rig.b_left, 1] > bq[rig.b_palm, 1]
    assert bq[rig.b_right, 1] < bq[rig.b_palm, 1]
    assert np.isclose(rig.model.body_mass.numpy()[rig.b_carriage], 0.05)
    inertia = rig.model.body_inertia.numpy()[rig.b_carriage]
    assert np.linalg.matrix_rank(inertia) == 3
    assert np.all(np.linalg.eigvalsh(inertia) > 0.0)
    assert np.all(np.isfinite(rig.model.body_inv_inertia.numpy()[rig.b_carriage]))

    assert rig.model.joint_q.numpy()[rig.x_qi] == 0.0
    rig.set_control(0.0, rig.grab_z)
    assert rig.control.joint_target_q.numpy()[rig.x_ti] == 0.0
    assert rig.control.joint_target_qd.numpy()[rig.x_dof] == 0.0

    path = tmp_path / "snapshot.npz"
    np.savez_compressed(path, body_q=bq, body_labels=np.asarray(labels))
    saved = np.load(path)
    assert saved["body_labels"].tolist() == labels
