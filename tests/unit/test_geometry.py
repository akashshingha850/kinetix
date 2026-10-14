import numpy as np

from kinetix.core.geometry import (R_to_quat, body_from_camera, camera_from_body, camera_pose, camera_yaw, quat_to_R,
                                   rot_z, segment_hits_box)
from kinetix.core.types import Box


def test_quat_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        R = quat_to_R(q)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
        assert np.allclose(quat_to_R(R_to_quat(R)), R, atol=1e-9)


def test_camera_pose_axes():
    """Optical z (forward) along the yaw, optical x (right) = -left, optical y (down) = -up."""
    R = quat_to_R(camera_pose([0, 0, 1], np.pi / 2).q)
    assert np.allclose(R[:, 2], [0, 1, 0], atol=1e-9)     # looking north
    assert np.allclose(R[:, 0], [1, 0, 0], atol=1e-9)     # right = east
    assert np.allclose(R[:, 1], [0, 0, -1], atol=1e-9)    # down
    assert np.isclose(camera_yaw(camera_pose([0, 0, 0], 0.7)), 0.7)


def test_body_camera_roundtrip():
    t = np.array([0.12, 0.03, 0.05])
    cam = camera_pose([1.0, 2.0, 1.5], 1.1)
    b = body_from_camera(cam.p, 1.1, t)
    back = camera_from_body(b, R_to_quat(rot_z(1.1)), t)
    assert np.allclose(back.p, cam.p) and np.allclose(quat_to_R(back.q), quat_to_R(cam.q), atol=1e-9)


def test_segment_box():
    box = Box(np.zeros(3), np.ones(3))
    assert segment_hits_box([-2, 0, 0], [2, 0, 0], box)
    assert not segment_hits_box([-2, 2, 0], [2, 2, 0], box)
    assert not segment_hits_box([-2, 0, 0], [-1, 0, 0], box)
    assert segment_hits_box([0, 0, 0], [0, 0, 0.1], box)            # starts inside
