import numpy as np

from scripts.vbd.w3_solid_render import boundary_triangles


def test_cube_tets_extract_only_outward_boundary_faces():
    vertices = np.array([
        [0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0],
        [0, 0, 1], [1, 0, 1], [0, 1, 1], [1, 1, 1],
    ], dtype=float)
    # Six tetrahedra sharing the cube's body diagonal 0--7.
    tets = np.array([
        [0, 1, 3, 7], [0, 3, 2, 7], [0, 2, 6, 7],
        [0, 6, 4, 7], [0, 4, 5, 7], [0, 5, 1, 7],
    ])

    boundary = boundary_triangles(tets, vertices)

    assert boundary.shape == (12, 3)
    assert len({tuple(sorted(face)) for face in boundary}) == 12
    diagonal = {0, 7}
    assert not any(diagonal.issubset(face) for face in map(set, boundary))
    cube_center = np.array([0.5, 0.5, 0.5])
    for face in boundary:
        a, b, c = vertices[face]
        normal = np.cross(b - a, c - a)
        assert np.dot(normal, cube_center - (a + b + c) / 3) < 0
