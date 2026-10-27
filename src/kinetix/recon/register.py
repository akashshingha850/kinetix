"""DriftTracker: estimate how far the EKF2 pose has drifted by registering each depth frame against the map built so far.

EKF2 is accurate over seconds and drifts over minutes (sim: ~0.2 m in 5 min, >1 m by 10 min). A map built on the raw EKF2 pose
sees the same wall at several offsets: walls grow outward and thin duplicate sheets appear. The depth camera is an independent
witness of the room's geometry, so each new frame is slid over the map and the offset where its points best land on what is
already occupied is the drift since the map was started. Translation only (the drift that matters for walls; yaw drift is small
over a few minutes).

  map_pose = ekf_pose + t        (t starts at 0: the map frame is the EKF2 frame at take-off)

Search: coarse then fine grid of offsets around the current t. The score is the mean of a smooth "closeness to occupied" field
at the frame's points, minus a pull towards the current t, so a direction a view cannot observe (translation along the single wall it
looks at) stays where it was. A correction is accepted only when it beats the current t by `min_gain`, and is capped per frame.
"""
from __future__ import annotations

import numpy as np

from kinetix.core.geometry import quat_to_R
from kinetix.core.types import Intrinsics, Pose
from kinetix.recon.depthmap import OccupancyMap

WEIGHTS = (1.0, 0.7, 0.4, 0.2)               # field value at 0, 1, 2, 3 voxels from an occupied voxel


def depth_points(depth: np.ndarray, K: Intrinsics, pose: Pose, stride: int, dmin: float, dmax: float, n_max: int,
                 rng: np.random.Generator) -> np.ndarray:
    """Map-frame points (n, 3) of the valid near returns of a depth image (same subsampling rule as OccupancyMap.integrate)."""
    H, W = depth.shape
    s = max(1, round(stride * W / K.width))
    sx, sy = W / K.width, H / K.height
    v, u = np.arange(s // 2, H, s), np.arange(s // 2, W, s)
    uu, vv = np.meshgrid(u, v)
    d = depth[np.ix_(v, u)].ravel().astype(float)
    ok = np.isfinite(d) & (d >= dmin) & (d <= dmax)
    d, uu, vv = d[ok], uu.ravel()[ok], vv.ravel()[ok]
    if len(d) > n_max:
        pick = rng.choice(len(d), n_max, replace=False)
        d, uu, vv = d[pick], uu[pick], vv[pick]
    dirs = np.stack([(uu - K.cx * sx) / (K.fx * sx), (vv - K.cy * sy) / (K.fy * sy), np.ones_like(d)], axis=1)
    return np.asarray(pose.p, dtype=float) + (dirs * d[:, None]) @ quat_to_R(pose.q).T


class DriftTracker:
    def __init__(self, rc: dict, seed: int = 0):
        self.rc = rc
        self.t = np.zeros(3)
        self.rng = np.random.default_rng(seed)
        self.history: list[dict] = []

    def to_map(self, pose: Pose) -> Pose:
        return Pose(np.asarray(pose.p, dtype=float) + self.t, pose.q)

    def to_ekf(self, pose: Pose) -> Pose:
        return Pose(np.asarray(pose.p, dtype=float) - self.t, pose.q)

    def _field(self, omap: OccupancyMap) -> tuple[np.ndarray, np.ndarray] | None:
        """Closeness-to-occupied field over the box around the occupied voxels, and that box's lower voxel corner."""
        occ = omap.occupied
        idx = np.argwhere(occ)
        if len(idx) < self.rc["min_occupied"]:
            return None
        lo = np.maximum(idx.min(axis=0) - 4, 0)
        hi = np.minimum(idx.max(axis=0) + 5, occ.shape)
        sub = occ[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]]
        F = np.zeros(sub.shape, dtype=np.float32)
        for r in range(len(WEIGHTS) - 1, -1, -1):
            F[OccupancyMap.dilate(sub, r, r)] = WEIGHTS[r]
        return F, lo

    def _score(self, F, lo_vox, omap, pts, T) -> np.ndarray:
        """Mean field value of `pts` shifted by each offset in T (m, 3)."""
        out = np.zeros(len(T))
        shape = np.array(F.shape)
        for k in range(0, len(T), 64):                        # chunks keep the (M, N, 3) index array small
            ijk = np.floor((pts[None] + T[k:k + 64, None] - omap.lo) / omap.voxel).astype(np.int64) - lo_vox
            ok = np.all((ijk >= 0) & (ijk < shape), axis=2)
            vals = np.where(ok, F[tuple(np.where(ok[..., None], ijk, 0).transpose(2, 0, 1))], 0.0)
            out[k:k + 64] = vals.mean(axis=1)
        return out

    def update(self, omap: OccupancyMap, depth: np.ndarray, K: Intrinsics, ekf_pose: Pose, dmin: float, dmax: float,
               stride: int) -> dict:
        """Register one frame taken at `ekf_pose` against the map; moves self.t when the evidence is clear. Call BEFORE integrating
        the frame (the map must not contain it yet). Returns what happened (for the exploration log)."""
        rc = self.rc
        info = {"applied": False, "t": self.t.round(3).tolist()}
        field = self._field(omap)
        if field is None:
            return {**info, "why": "map too sparse"}
        F, lo_vox = field
        pts = depth_points(depth, K, ekf_pose, stride, dmin, dmax, rc["points"], self.rng)
        if len(pts) < 300:
            return {**info, "why": "few depth points"}
        base = self.t.copy()

        def best_around(centre, half, step):
            g = np.arange(-half, half + 1e-9, step)
            T = centre + np.stack(np.meshgrid(g, g, g, indexing="ij"), axis=-1).reshape(-1, 3)
            sc = self._score(F, lo_vox, omap, pts, T) - rc["reg"] * np.sum((T - base) ** 2, axis=1)
            k = int(np.argmax(sc))
            return T[k], sc[k]
        s0 = self._score(F, lo_vox, omap, pts, base[None])[0]
        info["overlap"] = round(float(s0), 3)
        if s0 < rc["min_overlap"]:
            return {**info, "why": "no overlap with the map yet"}
        t1, _ = best_around(base, rc["search_m"], rc["coarse_step_m"])
        t2, _ = best_around(t1, rc["coarse_step_m"], rc["fine_step_m"])
        gain = float(self._score(F, lo_vox, omap, pts, t2[None])[0] - s0)
        # observability: a view of one wall pins the offset only across that wall. Move along an axis only where the score has real
        # curvature (it drops when the frame is slid either way); the others keep their value (they are what the search cannot see)
        h = rc["curv_step_m"]
        e = np.eye(3) * h
        probe = np.concatenate([t2[None], t2 + e, t2 - e])                      # centre, +x +y +z, -x -y -z
        s = self._score(F, lo_vox, omap, pts, probe)
        curv = s[0] - 0.5 * (s[1:4] + s[4:7])
        seen = curv >= rc["min_curv"]
        info["curv"] = curv.round(3).tolist()
        step = np.where(seen, t2 - base, 0.0)
        n = np.linalg.norm(step)
        if gain >= rc["min_gain"] and seen.any() and n > 1e-6:
            if n > rc["max_step_m"]:
                step *= rc["max_step_m"] / n
            self.t = base + step
            info.update(applied=True, t=self.t.round(3).tolist(), step=step.round(3).tolist(), gain=round(gain, 3))
        else:
            info.update(gain=round(gain, 3))
        self.history.append(info)
        return info
