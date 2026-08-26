import io
import unittest

import numpy as np

from src import tactile


def synthetic_raw():
    # two samples: s0 both fingers contact, s1 left only
    node_pos_world = np.array(
        [[0.0, -0.52, 0.22], [0.0, -0.48, 0.22], [0.01, -0.52, 0.23]], np.float32
    )
    node_pos_pad = np.array(
        [[0.0, 0.002, 0.05], [0.0, -0.002, 0.05], [0.01, 0.003, 0.06]], np.float32
    )
    node_force_world = np.array(
        [[0.0, 0.4, 0.05], [0.0, -0.38, 0.04], [0.0, 0.2, 0.01]], np.float32
    )
    node_finger_id = np.array([0, 1, 0], np.int16)
    sample_offsets = np.array([0, 2, 3], np.int32)
    pad_pose = np.array([[0.0, -0.54, 0.31, 0.0, 0.0, 0.0, 1.0]], np.float32)
    return {
        "node_pos_world": node_pos_world,
        "node_pos_pad": node_pos_pad,
        "node_force_world": node_force_world,
        "node_finger_id": node_finger_id,
        "sample_offsets": sample_offsets,
        "pad_pose_left": np.repeat(pad_pose, 2, axis=0),
        "pad_pose_right": np.repeat(pad_pose, 2, axis=0),
        "sample_t_s": np.array([0.0, 0.005], np.float64),
        "dt_mpm_s": np.array([0.005, 0.005], np.float64),
        "voxel_size": np.float64(0.005),
        "pad_normal_local": np.array([0.0, 1.0, 0.0], np.float32),
    }


class TestTactileAggregates(unittest.TestCase):
    def test_recompute_is_bitwise_identical(self):
        raw = synthetic_raw()
        agg1 = tactile.compute_aggregates(raw)
        raw.update(agg1)
        buf = io.BytesIO()
        np.savez(buf, **raw)
        buf.seek(0)
        npz = np.load(buf)
        agg2 = tactile.recompute_aggregates(npz)
        for k, v in agg1.items():
            stored = npz[k]
            self.assertTrue(np.array_equal(stored, np.asarray(agg2[k]), equal_nan=True),
                            f"aggregate {k} not bitwise identical")

    def test_no_contact_is_explicit(self):
        raw = synthetic_raw()
        agg = tactile.compute_aggregates(raw)
        # sample 1 right finger: no nodes -> zero resultants, NaN centroid
        self.assertEqual(agg["agg_n_nodes"][1, 1], 0)
        self.assertFalse(agg["agg_in_contact"][1, 1])
        self.assertEqual(agg["agg_normal_n"][1, 1], 0.0)
        self.assertTrue(np.isnan(agg["agg_centroid_pad_m"][1, 1]).all())

    def test_area_is_grid_quantized(self):
        raw = synthetic_raw()
        agg = tactile.compute_aggregates(raw)
        self.assertAlmostEqual(agg["agg_area_m2"][0, 0], 1 * 0.005 * 0.005)
        self.assertAlmostEqual(agg["agg_area_m2"][1, 0], 1 * 0.005 * 0.005)

    def test_sum_before_decomposition(self):
        # two opposing tangential forces on one pad must cancel in shear
        raw = synthetic_raw()
        raw["node_pos_world"] = np.array([[0, -0.52, 0.22], [0, -0.52, 0.23]], np.float32)
        raw["node_pos_pad"] = np.array([[0, 0.002, 0.05], [0, 0.002, 0.06]], np.float32)
        raw["node_force_world"] = np.array([[0.3, 0.5, 0.0], [-0.3, 0.5, 0.0]], np.float32)
        raw["node_finger_id"] = np.array([0, 0], np.int16)
        raw["sample_offsets"] = np.array([0, 2], np.int32)
        raw["pad_pose_left"] = raw["pad_pose_left"][:1]
        raw["pad_pose_right"] = raw["pad_pose_right"][:1]
        raw["sample_t_s"] = raw["sample_t_s"][:1]
        raw["dt_mpm_s"] = raw["dt_mpm_s"][:1]
        agg = tactile.compute_aggregates(raw)
        self.assertAlmostEqual(agg["agg_shear_n"][0, 0], 0.0, places=6)
        self.assertAlmostEqual(agg["agg_normal_n"][0, 0], 1.0, places=6)


if __name__ == "__main__":
    unittest.main()
