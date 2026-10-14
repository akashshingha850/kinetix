import numpy as np

from kinetix.core.geometry import camera_pose, look_at_yaw
from kinetix.core.types import Box, Frame, Intrinsics
from kinetix.recon.coverage import CoverageState, box_surface

K = Intrinsics(1280, 720, 529.8, 529.8, 640.0, 360.0)
ROI = Box(np.array([0.0, 0.0, 1.0]), np.array([0.4, 0.4, 0.4]))


def view(az_deg, r=1.5, z=1.0):
    a = np.radians(az_deg)
    p = np.array([r * np.cos(a), r * np.sin(a), z])
    return camera_pose(p, look_at_yaw(p, ROI.center))


def frame(i, pose):
    return Frame(i, 0.0, None, K, pose, pose, None, 0.0)


def test_surface_has_no_bottom():
    P, N = box_surface(ROI, 0.1)
    assert (N[:, 2] >= 0).all() and len(P) > 0


def test_one_view_sees_one_face_and_triangulates_nothing():
    s = CoverageState(ROI, K)
    m, _ = s.visible(view(180))
    assert m.any()
    assert (s.N[m][:, 0] < 0).all()            # only the -x face (camera at -x)
    s.add(frame(0, view(180)))
    assert s.coverage == 0.0 and s.seen > 0


def test_second_view_from_new_angle_triangulates_and_has_highest_gain():
    s = CoverageState(ROI, K)
    s.add(frame(0, view(180)))
    g_same = s.gain(view(180))
    g_pair = s.gain(view(160))
    g_back = s.gain(view(0))                   # opposite side: only unseen points (0.5 each)
    assert g_pair > g_same and g_pair > 0
    s.add(frame(1, view(160)))
    assert s.coverage > 0


def test_coverage_grows_around_the_object():
    s = CoverageState(ROI, K)
    for i, az in enumerate(range(0, 360, 20)):
        s.add(frame(i, view(az)))
    assert s.seen > 0.75 and s.coverage > 0.7    # sides done; the top needs higher views
