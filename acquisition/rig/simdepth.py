"""Ray-cast depth of a box room (+ box obstacles) for the dry rig and tests: what a depth camera would return, no sim needed.

The room is the inner box (rays start inside it and stop at its walls, floor and ceiling); obstacles are boxes the rays can hit
first. Depth is the optical z (metres along the optical axis), like the ZED's `depth_registered`."""
from __future__ import annotations

import numpy as np

from kinetix.geometry import quat_to_R
from kinetix.types import Box, Intrinsics, Pose


def render_depth(room: Box, obstacles, pose: Pose, K: Intrinsics, stride: int = 8, noise_frac: float = 0.0,
                 max_range: float = 12.0, rng: np.random.Generator | None = None) -> np.ndarray:
    """(H/stride, W/stride) float32 depth; NaN beyond `max_range` (no return). `K` is the full-resolution intrinsics: the
    returned image is subsampled, so a consumer scales K by shape / (height, width) (`OccupancyMap.integrate` does)."""
    v = np.arange(stride // 2, K.height, stride)
    u = np.arange(stride // 2, K.width, stride)
    uu, vv = np.meshgrid(u, v)
    dirs = np.stack([(uu - K.cx) / K.fx, (vv - K.cy) / K.fy, np.ones_like(uu, dtype=float)], axis=-1).reshape(-1, 3)
    d = dirs @ quat_to_R(pose.q).T                                      # map-frame ray directions (z-component of optical = 1)
    o = pose.p
    with np.errstate(divide="ignore", invalid="ignore"):
        t1, t2 = (room.lo - o) / d, (room.hi - o) / d
    t = np.where(d > 0, t2, t1)
    t = np.where(np.isfinite(t), t, np.inf).min(axis=1)                 # exit through the nearest wall / floor / ceiling
    for b in obstacles:
        with np.errstate(divide="ignore", invalid="ignore"):
            a1, a2 = (b.lo - o) / d, (b.hi - o) / d
        par = d == 0
        t_in = np.where(par, -np.inf, np.minimum(a1, a2)).max(axis=1)
        t_out = np.where(par, np.inf, np.maximum(a1, a2)).min(axis=1)
        outside = (par & ((o < b.lo) | (o > b.hi))).any(axis=1)
        hit = (t_in <= t_out) & (t_in > 1e-6) & ~outside
        t = np.where(hit & (t_in < t), t_in, t)
    depth = t.astype(np.float32)
    depth[(depth > max_range) | ~np.isfinite(depth)] = np.nan
    if noise_frac:
        depth *= 1.0 + noise_frac * (rng or np.random.default_rng(0)).standard_normal(depth.shape).astype(np.float32)
    return depth.reshape(len(v), len(u))
