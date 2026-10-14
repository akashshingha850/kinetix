"""SE3 helpers in one place (interfaces.md §1): ENU `map`, camera link (x fwd, y left, z up), OpenCV optical (x right,
y down, z fwd). Quaternions are xyzw."""
from __future__ import annotations

import numpy as np

from kinetix.core.types import Box, Pose

# optical -> camera link: optical x = -link y, optical y = -link z, optical z = link x (columns = optical axes in link)
R_LINK_OPT = np.array([[0.0, 0.0, 1.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0]])


def rot_z(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def quat_to_R(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R) -> np.ndarray:
    R = np.asarray(R, dtype=float)
    t = np.trace(R)
    if t > 0:
        s = 2.0 * np.sqrt(t + 1.0)
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
        q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q)


def yaw_of_R(R) -> float:
    return float(np.arctan2(R[1, 0], R[0, 0]))


def wrap(a: float) -> float:
    return float((a + np.pi) % (2 * np.pi) - np.pi)


def camera_pose(p, yaw: float, pitch: float = 0.0) -> Pose:
    """T_map_cam of a camera at p looking along `yaw` (rad, ENU, 0 = +x), tilted down by `pitch` (rad, mount pitch)."""
    R = rot_z(yaw) @ rot_y(pitch) @ R_LINK_OPT
    return Pose(np.asarray(p, dtype=float), R_to_quat(R))


def camera_yaw(pose: Pose) -> float:
    """Heading of the optical axis projected on the ground plane."""
    z = quat_to_R(pose.q)[:, 2]
    return float(np.arctan2(z[1], z[0]))


def look_at_yaw(p, target) -> float:
    d = np.asarray(target, dtype=float) - np.asarray(p, dtype=float)
    return float(np.arctan2(d[1], d[0]))


def body_from_camera(cam_p, yaw: float, t_base_cam) -> np.ndarray:
    """Body position that puts a yaw-only-mounted camera (offset t_base_cam in the body frame) at cam_p, body level."""
    return np.asarray(cam_p, dtype=float) - rot_z(yaw) @ np.asarray(t_base_cam, dtype=float)


def camera_from_body(body_p, body_q, t_base_cam, mount_pitch: float = 0.0) -> Pose:
    """T_map_cam from the vehicle pose (map, ENU, FLU body) and the camera mount (translation + pitch, no roll/yaw)."""
    Rb = quat_to_R(body_q)
    p = np.asarray(body_p, dtype=float) + Rb @ np.asarray(t_base_cam, dtype=float)
    return Pose(p, R_to_quat(Rb @ rot_y(mount_pitch) @ R_LINK_OPT))


def point_box_distance(p, box: Box) -> float:
    """Euclidean distance from p to an axis-aligned box (0 inside)."""
    d = np.maximum(np.maximum(box.lo - p, 0.0), np.asarray(p) - box.hi)
    return float(np.linalg.norm(d))


def inside(p, box: Box) -> bool:
    p = np.asarray(p)
    return bool(np.all(p >= box.lo) and np.all(p <= box.hi))


def segment_hits_box(a, b, box: Box) -> bool:
    """Slab test: does the closed segment a->b touch the axis-aligned box?"""
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    d = b - a
    t0, t1 = 0.0, 1.0
    for i in range(3):
        if abs(d[i]) < 1e-12:
            if a[i] < box.lo[i] or a[i] > box.hi[i]:
                return False
            continue
        ta, tb = (box.lo[i] - a[i]) / d[i], (box.hi[i] - a[i]) / d[i]
        t0, t1 = max(t0, min(ta, tb)), min(t1, max(ta, tb))
        if t0 > t1:
            return False
    return True
