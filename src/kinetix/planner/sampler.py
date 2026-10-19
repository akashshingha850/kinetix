"""Candidate camera poses, 4-DoF (x, y, z, yaw) with the mount's fixed pitch (architecture.md §4 sampler.py):
- object scenes: a ring stack around the ROI, every view looking at the ROI centre's vertical axis;
- room scenes (ADR-K11): a grid filling the workspace, every position with `n_yaw` evenly spaced headings."""
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


def grid_poses(scene: Scene, caps: RigCaps, spacing: float, heights, n_yaw: int) -> list[Candidate]:
    """Positions on an xy grid with `spacing` m, centred in the workspace box, at the given map heights (clipped to the
    workspace); n_yaw headings each (0 = +x). Infeasible ones (obstacles, altitude) are left to the constraints."""
    lo, hi = scene.workspace.lo, scene.workspace.hi
    axes = []
    for k in range(2):
        n = int(np.floor((hi[k] - lo[k]) / spacing)) + 1
        off = (hi[k] - lo[k] - (n - 1) * spacing) / 2
        axes.append(lo[k] + off + spacing * np.arange(n))
    zs = [z for z in heights if lo[2] <= z <= hi[2]]
    out = []
    for x in axes[0]:
        for y in axes[1]:
            for z in zs:
                for k in range(n_yaw):
                    yaw = 2 * np.pi * k / n_yaw
                    out.append(Candidate(camera_pose([x, y, z], yaw, caps.mount_pitch_rad),
                                         pool_id=f"x{x:+.1f}_y{y:+.1f}_z{z:.1f}_h{np.degrees(yaw):03.0f}"))
    return out


class GridSampler:
    def __init__(self, spacing: float, heights, n_yaw: int):
        self.spacing, self.heights, self.n_yaw = float(spacing), list(heights), int(n_yaw)
        self._cache = None

    def sample(self, scene: Scene, recon, caps: RigCaps) -> list[Candidate]:
        if self._cache is None:
            self._cache = grid_poses(scene, caps, self.spacing, self.heights, self.n_yaw)
        return self._cache
