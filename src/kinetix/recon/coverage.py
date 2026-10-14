"""CoverageState: a GT-free photogrammetric coverage model over the ROI proxy surface.

Stand-in for `recon/state.py` (pycolmap ReconState, K1) so the NBV loop can run before SfM exists. The planner knows only the
scene's ROI box, so the object is approximated by the box's side and top faces, sampled every `spacing` m. A surface point is
**observed** by a frame when it projects into the image, lies in the depth range and faces the camera (incidence below
`max_incidence_deg`). It is **triangulated** when two of its observing rays are at least `tri_min_deg` apart: the
condition photogrammetry needs to reconstruct it. Occlusion is ignored (the proxy is convex).
"""
from __future__ import annotations

import numpy as np

from kinetix.core.geometry import quat_to_R
from kinetix.core.types import Box, Frame, Intrinsics, Pose


def box_surface(box: Box, spacing: float, bottom: bool = False) -> tuple[np.ndarray, np.ndarray]:
    """Points and outward normals on the faces of an axis-aligned box (no bottom face unless asked)."""
    pts, nrm = [], []
    lo, hi = box.lo, box.hi
    for axis in range(3):
        for sign in (-1, 1):
            if axis == 2 and sign == -1 and not bottom:
                continue
            u, v = [a for a in range(3) if a != axis]
            nu = max(2, int(np.ceil((hi[u] - lo[u]) / spacing)) + 1)
            nv = max(2, int(np.ceil((hi[v] - lo[v]) / spacing)) + 1)
            gu, gv = np.meshgrid(np.linspace(lo[u], hi[u], nu), np.linspace(lo[v], hi[v], nv), indexing="ij")
            p = np.zeros((gu.size, 3))
            p[:, u], p[:, v] = gu.ravel(), gv.ravel()
            p[:, axis] = hi[axis] if sign > 0 else lo[axis]
            n = np.zeros(3)
            n[axis] = sign
            pts.append(p)
            nrm.append(np.repeat(n[None], len(p), axis=0))
    return np.concatenate(pts), np.concatenate(nrm)


class CoverageState:
    def __init__(self, roi: Box, K: Intrinsics, spacing: float = 0.04, depth_range=(0.2, 4.0), max_incidence_deg=75.0,
                 tri_min_deg=8.0, novel_deg=25.0, max_views=6, border=0.03):
        self.P, self.N = box_surface(roi, spacing)
        self.K = K
        self.depth_range = depth_range
        self.cos_inc = np.cos(np.radians(max_incidence_deg))
        self.cos_tri = np.cos(np.radians(tri_min_deg))
        self.cos_novel = np.cos(np.radians(novel_deg))
        self.max_views = max_views
        self.border = border
        self.rays = np.zeros((len(self.P), max_views, 3))   # unit rays point -> camera of the observing frames
        self.count = np.zeros(len(self.P), dtype=int)
        self.tri = np.zeros(len(self.P), dtype=bool)
        self.frames: list[int] = []

    # --- visibility -------------------------------------------------------------------------------------------------
    def visible(self, pose: Pose, K: Intrinsics | None = None) -> tuple[np.ndarray, np.ndarray]:
        """(mask over surface points, unit rays point->camera) for a camera at `pose`."""
        K = K or self.K
        R = quat_to_R(pose.q)
        d = pose.p[None] - self.P                       # point -> camera
        dist = np.linalg.norm(d, axis=1)
        ray = d / np.maximum(dist[:, None], 1e-9)
        Xc = (self.P - pose.p[None]) @ R                 # map -> optical: R^T (P - p)
        z = Xc[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = K.fx * Xc[:, 0] / z + K.cx
            v = K.fy * Xc[:, 1] / z + K.cy
        bw, bh = self.border * K.width, self.border * K.height
        m = (z > self.depth_range[0]) & (z < self.depth_range[1])
        m &= (u > bw) & (u < K.width - bw) & (v > bh) & (v < K.height - bh)
        m &= np.einsum("ij,ij->i", self.N, ray) > self.cos_inc
        return m, ray

    # --- state ------------------------------------------------------------------------------------------------------
    def add(self, frame: Frame) -> int:
        """Record a captured frame at its reported pose; returns the number of points it observed."""
        self.K = frame.K
        m, ray = self.visible(frame.pose, frame.K)
        for i in np.flatnonzero(m):
            c = self.count[i]
            if c and not self.tri[i]:
                self.tri[i] = bool(np.any(self.rays[i, :c] @ ray[i] < self.cos_tri))
            if c < self.max_views:
                self.rays[i, c] = ray[i]
                self.count[i] = c + 1
        self.frames.append(frame.idx)
        return int(m.sum())

    def gain(self, pose: Pose) -> float:
        """Expected value of a frame at `pose`, normalised by the surface size (0..1):
        1.0 per seen-but-untriangulated point it would triangulate, 0.5 per unseen point (first ray),
        0.1 per triangulated point it adds a novel direction to (> novel_deg from all rays, fewer than max_views)."""
        m, ray = self.visible(pose)
        idx = np.flatnonzero(m)
        if not len(idx):
            return 0.0
        c = self.count[idx]
        cos = np.einsum("ikj,ij->ik", self.rays[idx], ray[idx])        # (n, max_views)
        cos = np.where(np.arange(self.max_views)[None] < c[:, None], cos, -1.0)
        best = cos.max(axis=1)                                          # closest existing ray
        g = np.where(c == 0, 0.5, 0.0)
        g += np.where((c > 0) & ~self.tri[idx] & (best < self.cos_tri), 1.0, 0.0)
        g += np.where(self.tri[idx] & (c < self.max_views) & (best < self.cos_novel), 0.1, 0.0)
        return float(g.sum() / len(self.P))

    @property
    def coverage(self) -> float:
        """Fraction of the ROI proxy surface that is triangulated."""
        return float(self.tri.mean())

    @property
    def seen(self) -> float:
        return float((self.count > 0).mean())
