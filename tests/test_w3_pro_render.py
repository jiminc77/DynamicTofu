import numpy as np

from scripts.vbd.w3_pro_render import pad_contact_footprint, pad_taxel_depth


def test_pad_frame_projection_threshold_and_centroid():
    pose = np.array([1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0])
    # Inner face is local +y = 0.006. First two are within the 0.003 m
    # perpendicular proximity band and pad x/z extent; others miss distance
    # or extent. The first vertex lies exactly on the inner face.
    local = np.array([
        [-0.010, 0.0060, -0.010],
        [0.010, 0.0085, 0.010],
        [0.000, 0.0091, 0.000],
        [0.023, 0.0060, 0.000],
    ])

    points, centroid = pad_contact_footprint(local + pose[:3], pose, 1.0)

    np.testing.assert_allclose(points, local[:2, (0, 2)])
    np.testing.assert_allclose(centroid, [0.0, 0.0], atol=1e-15)

    grid, count = pad_taxel_depth(local + pose[:3], pose, 1.0)
    assert count == 2
    assert np.count_nonzero(grid) == 2
    assert np.isclose(grid.max(), 0.003)
