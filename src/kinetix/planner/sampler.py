"""Candidate camera poses: a ring stack around the ROI (architecture.md §4 sampler.py), 4-DoF (x, y, z, yaw), every view
looking at the ROI centre's vertical axis with the mount's fixed pitch."""
from __future__ import annotations

import numpy as np

from kinetix.core.geometry import camera_pose
from kinetix.core.types import Candidate, RigCaps, Scene


def ring_stack(scene: Scene, caps: RigCaps, radii, heights, n_az: int) -> list[Candidate]:
    """Cameras on horizontal rings of radius r (from the ROI's vertical axis) at map heights z, n_az azimuths each.
    Azimuth = direction from the ROI to the camera (0 = +x, east); the camera yaw points back at the ROI."""
    c = scene.roi.center
    out = []
    for r in radii:
        for z in heights:
            for k in range(n_az):
                az = 2 * np.pi * k / n_az
                p = np.array([c[0] + r * np.cos(az), c[1] + r * np.sin(az), z])
                yaw = az + np.pi
                out.append(Candidate(camera_pose(p, yaw, caps.mount_pitch_rad),
                                     pool_id=f"r{r:.2f}_z{z:.2f}_a{np.degrees(az):03.0f}"))
    return out


class RingSampler:
    def __init__(self, radii, heights, n_az: int):
        self.radii, self.heights, self.n_az = list(radii), list(heights), int(n_az)
        self._cache = None

    def sample(self, scene: Scene, recon, caps: RigCaps) -> list[Candidate]:
        if self._cache is None:
            self._cache = ring_stack(scene, caps, self.radii, self.heights, self.n_az)
        return self._cache
