"""MeshOcclusion (the oracle's self-occlusion test) and k_min (views a point needs to count as covered)."""
import numpy as np
import open3d as o3d

from kinetix.coverage import CoverageState, MeshOcclusion
from kinetix.geometry import camera_pose, look_at_yaw
from kinetix.types import Box, Frame, Intrinsics

K = Intrinsics(width=320, height=240, fx=200.0, fy=200.0, cx=160.0, cy=120.0)


def _pose(eye, target):
    return camera_pose(eye, look_at_yaw(eye, target))


def _two_plates(tmp_path):
    """Two 0.4 m plates facing -x, at x = 0 (front) and x = 0.3 (behind it)."""
    m = o3d.geometry.TriangleMesh()
    for x in (0.0, 0.3):
        a = o3d.geometry.TriangleMesh.create_box(0.01, 0.4, 0.4)
        a.translate([x - 0.01, -0.2, 0.8])
        m += a
    f = tmp_path / "plates.ply"
    o3d.io.write_triangle_mesh(str(f), m)
    return f


def test_mesh_occlusion_hides_the_back_plate(tmp_path):
    occ = MeshOcclusion(_two_plates(tmp_path), scale=2)
    P = np.array([[-0.011, 0.0, 1.0], [0.289, 0.0, 1.0]])          # on each plate's front face
    N = np.tile([-1.0, 0.0, 0.0], (2, 1))
    roi = Box(np.array([0.15, 0.0, 1.0]), np.array([0.4, 0.5, 0.5]))
    s = CoverageState(roi, K, surface=(P, N), occlusion=occ)
    m, _ = s.visible(_pose([-1.5, 0.0, 1.0], [0.0, 0.0, 1.0]))
    assert m.tolist() == [True, False]
    t = CoverageState(roi, K, surface=(P, N))
    assert t.visible(_pose([-1.5, 0.0, 1.0], [0.0, 0.0, 1.0]))[0].tolist() == [True, True]   # without it: both


def test_k_min_needs_three_views():
    roi = Box(np.array([0.0, 0.0, 1.0]), np.array([0.4, 0.4, 0.4]))
    P, N = np.array([[-0.2, 0.0, 1.0]]), np.array([[-1.0, 0.0, 0.0]])
    s = CoverageState(roi, K, surface=(P, N), k_min=3, match_max_deg=30.0)
    eyes = [[-1.5, y, 1.0] for y in (0.0, 0.3, -0.3)]                 # ~11 deg apart
    for i, e in enumerate(eyes):
        assert s.coverage == 0.0 and s.gain(_pose(e, P[0])) > 0       # not covered before the third view
        s.add(Frame(i, 0.0, None, K, _pose(e, P[0]), _pose(e, P[0]), None, 0.0))
    assert s.coverage == 1.0 and s.pair[0] and s.count[0] == 3
