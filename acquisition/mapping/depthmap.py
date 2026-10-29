"""OccupancyMap: a live voxel map of the room from depth images posed by the flight controller's EKF (the rig pose).

Why this and not the ZED's own spatial map: the map is built at the **EKF2 pose** (rig pose = EKF2 ∘ camera mount), the same
estimate that flies the drone, so the map and the flight path share one frame (the ZED map lives in the ZED's own tracking
frame, which EKF2 corrects). It is a coarse, fast tier: it drives exploration and obstacle knowledge. The accurate tier stays
offline (`kinetix sfm`, bundle adjustment with the EKF2 poses as priors).

Log-odds per voxel in an int8 grid: a depth return raises its voxel (`hit`), the ray up to it lowers every voxel it crosses
(`miss`). L > 0 occupied, L < 0 free, L == 0 unknown. Rays with no return (NaN) are ignored: no return is not free space.
"""
from __future__ import annotations

import numpy as np

from kinetix.geometry import quat_to_R
from kinetix.types import Intrinsics, Pose


class OccupancyMap:
    def __init__(self, lo, hi, voxel: float = 0.2, depth_range=(0.3, 8.0), stride: int = 8, hit: int = 3, miss: int = -1,
                 lmin: int = -8, lmax: int = 8):
        self.lo = np.asarray(lo, dtype=float)
        self.voxel = float(voxel)
        self.shape = tuple(int(n) for n in np.ceil((np.asarray(hi, dtype=float) - self.lo) / self.voxel))
        self.L = np.zeros(self.shape, dtype=np.int8)
        self.probed = np.zeros(self.shape, dtype=bool)        # unknown voxels just past the range limit of a ray that returned farther
        self.depth_range, self.stride = tuple(depth_range), int(stride)
        self.hit, self.miss, self.lmin, self.lmax = hit, miss, lmin, lmax
        self.n_frames = 0

    # --- coordinates ----------------------------------------------------------------------------------------------
    def index(self, pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Integer voxel indices (n, 3) of map points and the mask of those inside the volume."""
        ijk = np.floor((np.asarray(pts, dtype=float) - self.lo) / self.voxel).astype(np.int64)
        ok = np.all((ijk >= 0) & (ijk < np.array(self.shape)), axis=1)
        return ijk, ok

    def centers(self, mask: np.ndarray) -> np.ndarray:
        """Map coordinates of the centres of the True voxels of `mask`."""
        return self.lo + (np.argwhere(mask) + 0.5) * self.voxel

    # --- integration ----------------------------------------------------------------------------------------------
    def integrate(self, depth: np.ndarray, K: Intrinsics, pose: Pose) -> int:
        """Fuse one depth image (float32 metres, optical z) taken at `pose` (T_map_cam). `K` is the intrinsics of the image the
        depth is registered to; a depth image of another size is handled by scaling K. Returns the number of rays used."""
        H, W = depth.shape
        sx, sy = W / K.width, H / K.height
        fx, fy, cx, cy = K.fx * sx, K.fy * sy, K.cx * sx, K.cy * sy
        s = max(1, round(self.stride * W / K.width))          # `stride` is in pixels of the registered image; a smaller depth image has fewer
        v = np.arange(s // 2, H, s)
        u = np.arange(s // 2, W, s)
        uu, vv = np.meshgrid(u, v)
        d = depth[np.ix_(v, u)].ravel().astype(float)
        ok = np.isfinite(d) & (d >= self.depth_range[0])
        if not ok.any():
            return 0
        d, uu, vv = d[ok], uu.ravel()[ok], vv.ravel()[ok]
        near = d <= self.depth_range[1]                          # a return beyond the trusted range is still proof that the air up
        d = np.minimum(d, self.depth_range[1])                   # to that range is empty: it carves free space but marks no wall
        dirs = np.stack([(uu - cx) / fx, (vv - cy) / fy, np.ones_like(d)], axis=1)
        R = quat_to_R(pose.q)
        o = np.asarray(pose.p, dtype=float)
        ends = o + (dirs * d[:, None]) @ R.T
        length = np.linalg.norm(ends - o, axis=1)

        step = 0.8 * self.voxel                                  # free space: every voxel the ray crosses, short of the hit
        n = np.maximum(np.floor((length - np.where(near, self.voxel, 0.0)) / step).astype(int), 0)
        if n.sum():
            rid = np.repeat(np.arange(len(d)), n)
            k = np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n) + 1
            pts = o + (ends[rid] - o) * (k * step / length[rid])[:, None]
            self._add(pts, self.miss)
        self._add(ends[near], self.hit)
        far = ~near                                               # the unknown behind the carved air is "beyond range", not a hole:
        if far.any():                                             # it must not read as frontier (a new shell around every view)
            for k in (0, 1, 2):
                ijk, ok = self.index(ends[far] + (ends[far] - o) / length[far][:, None] * (k * self.voxel))
                self.probed[tuple(ijk[ok].T)] = True
        self.n_frames += 1
        return int(len(d))

    def _add(self, pts: np.ndarray, delta: int) -> None:
        ijk, ok = self.index(pts)
        if not ok.any():
            return
        flat = np.unique(np.ravel_multi_index(tuple(ijk[ok].T), self.shape))     # one update per voxel per image
        cells = np.unravel_index(flat, self.shape)
        self.L[cells] = np.clip(self.L[cells].astype(np.int16) + delta, self.lmin, self.lmax).astype(np.int8)

    # --- queries ---------------------------------------------------------------------------------------------------
    @property
    def occupied(self) -> np.ndarray:
        return self.L > 0

    @property
    def free(self) -> np.ndarray:
        return self.L < 0

    @property
    def unknown(self) -> np.ndarray:
        return self.L == 0

    def frontier(self, probed: bool = False) -> np.ndarray:
        """Free voxels with an unknown face neighbour: the edge of what the drone has seen to be empty. `probed=True`: the range
        frontier instead, the edge of the air carved by returns from beyond the range (no wall seen there yet, only farther)."""
        unk = self.unknown & (self.probed if probed else ~self.probed)
        near = np.zeros(self.shape, dtype=bool)
        for ax in range(3):
            lo, hi = [slice(None)] * 3, [slice(None)] * 3
            a, b = list(lo), list(hi)
            a[ax], b[ax] = slice(1, None), slice(None, -1)
            near[tuple(b)] |= unk[tuple(a)]                       # neighbour at +1 is unknown
            near[tuple(a)] |= unk[tuple(b)]                       # neighbour at -1 is unknown
        return self.free & near

    @staticmethod
    def dilate(mask: np.ndarray, rxy: int, rz: int) -> np.ndarray:
        """Box dilation by rxy voxels in x and y, rz in z (separable)."""
        out = mask.copy()
        for ax, r in ((0, rxy), (1, rxy), (2, rz)):
            base = out.copy()
            for s in range(1, r + 1):
                a, b = [slice(None)] * 3, [slice(None)] * 3
                a[ax], b[ax] = slice(s, None), slice(None, -s)
                out[tuple(a)] |= base[tuple(b)]
                out[tuple(b)] |= base[tuple(a)]
        return out

    def _radii(self, clearance_m: float, clearance_z_m: float) -> tuple[int, int]:
        """Dilation radii in voxels. Centre-to-centre distances undercount the gap to an obstacle's surface by half a voxel, so a
        clearance of c metres needs ceil(c / voxel) + 1 voxels: the drone then really is c from what the planner later boxes."""
        return int(np.ceil(clearance_m / self.voxel)) + 1, int(np.ceil(clearance_z_m / self.voxel)) + 1

    def safe(self, clearance_m: float, clearance_z_m: float) -> np.ndarray:
        """Voxels where the drone may stop: free, and no occupied or unknown voxel within the clearance (it parks only where
        it has seen the space around it to be empty)."""
        rxy, rz = self._radii(clearance_m, clearance_z_m)
        return self.free & ~self.dilate(self.occupied | self.unknown, rxy, rz)

    def corridor(self, clearance_m: float, clearance_z_m: float) -> np.ndarray:
        """Voxels the drone may pass through: known free and clear of occupied space. Unknown space beside the path is allowed:
        right after a turn on the spot the cells next to the drone are unknown (the camera cone is thin close to it), and
        the depth camera sees along the way."""
        rxy, rz = self._radii(clearance_m, clearance_z_m)
        return self.free & ~self.dilate(self.occupied, rxy, rz)
