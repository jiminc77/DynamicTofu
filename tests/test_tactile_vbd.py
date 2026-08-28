import tempfile
import unittest
from pathlib import Path

import numpy as np

from src import tactile_vbd


def frame(t, forces, positions, available=True):
    return {
        "t_s": t,
        "force_channel_available": available,
        "pad_force_vectors": np.asarray(forces, np.float64),
        "pad_outward_normals": np.asarray(((0, 1, 0), (0, -1, 0)), np.float64),
        "contact_positions_pad": positions,
    }


class TestTactileVBD(unittest.TestCase):
    def test_force_decomposition_and_asymmetry_sign(self):
        raw = [frame(9.3, ((3, 4, 0), (0, -2, 0)), (((0, 0, 0),), ((0, 0, 0),)))]
        result = tactile_vbd.compute_aggregates_vbd(raw)
        np.testing.assert_array_equal(result["normal_resultant_n"], ((4.0, 2.0),))
        np.testing.assert_array_equal(result["tangential_resultant_n"], ((3.0, 0.0),))
        self.assertEqual(result["lr_normal_asymmetry_n"][0], 2.0)

    def test_no_contact_is_nan_centroid_and_zero_resultants(self):
        raw = [frame(9.3, ((0, 0, 0), (0, 0, 0)), ((), ()), available=False)]
        result = tactile_vbd.compute_aggregates_vbd(raw)
        self.assertTrue(np.isnan(result["contact_centroid_pad_m"]).all())
        np.testing.assert_array_equal(result["normal_resultant_n"], np.zeros((1, 2)))
        np.testing.assert_array_equal(result["tangential_resultant_n"], np.zeros((1, 2)))
        np.testing.assert_array_equal(result["contact_count"], np.zeros((1, 2), dtype=int))

    def test_centroid_is_unweighted(self):
        positions = (((0, 0, 0), (2, 0, 0)), ())
        low = tactile_vbd.compute_aggregates_vbd([frame(0, ((1, 1, 0), (0, 0, 0)), positions)])
        high = tactile_vbd.compute_aggregates_vbd([frame(0, ((1000, 1, 0), (0, 0, 0)), positions)])
        np.testing.assert_array_equal(low["contact_centroid_pad_m"], high["contact_centroid_pad_m"])
        np.testing.assert_array_equal(low["contact_centroid_pad_m"][0, 0], (1, 0, 0))
        self.assertEqual(low["contact_extent_m"][0, 0], 2.0)

    def test_recompute_bitwise_equals_stored_summary(self):
        raw = [
            frame(9.3, ((1, 2, 0), (0, -2, 0)), (((0, 0, 0), (1, 0, 0)), ((0, 0, 0),))),
            frame(9.4, ((2, 2, 0), (0, -1, 0)), (((0, 1, 0),), ())),
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = tactile_vbd.write_raw_vbd(raw, 15, 5, 0, Path(directory))
            recomputed = tactile_vbd.recompute_aggregates_vbd(path)
            stored = tactile_vbd.stored_aggregates_vbd(path)
            self.assertEqual(recomputed.keys(), stored.keys())
            for key in recomputed:
                self.assertEqual(recomputed[key].dtype, stored[key].dtype, key)
                self.assertEqual(recomputed[key].shape, stored[key].shape, key)
                self.assertEqual(recomputed[key].tobytes(), stored[key].tobytes(), key)


if __name__ == "__main__":
    unittest.main()
