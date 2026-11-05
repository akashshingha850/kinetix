"""CoverageState: a GT-free photogrammetric coverage model over a proxy surface.

Stand-in for `recon/state.py` (pycolmap ReconState, K1) so the NBV loop can run before SfM exists. The planner knows only the
scene file, so the surface is approximated by boxes, sampled every `spacing` m:
- object scenes: the ROI box's side and top faces (outward normals);
- room scenes (ADR-K11): the room box's inner walls, floor and ceiling (inward normals) plus the outer faces of the `no_fly`
  obstacles that are not flush with the room (no bottoms, no faces against a wall). Floor under an obstacle is dropped.

A surface point is **observed** by a frame when it projects into the image, lies in the depth range, faces the camera
(incidence below `max_incidence_deg`) and, in a room, the ray to it clears every obstacle box. It is **triangulated** when two of
its observing rays are at least `tri_min_deg` and at most `match_max_deg` apart: far enough apart to triangulate, close enough for
feature matching to find the correspondence (SIFT fails beyond ~30-40 deg of viewpoint change; objects default to no limit),
and at least `k_min` frames observe it (dense MVS wants >= 3 views a point; 2 = a pair is enough).
With a `MeshOcclusion` (the coverage oracle: the GT mesh) a point is also hidden when the mesh is in front of it (self-occlusion).
"""
from __future__ import annotations

import numpy as np

from kinetix.geometry import quat_to_R
from kinetix.types import Box, Frame, Intrinsics, Pose


def box_surface(box: Box, spacing: float, bottom: bool = False, inward: bool = False) -> tuple[np.ndarray, np.ndarray]:
    """Points and normals on the faces of an axis-aligned box (no bottom face unless asked). Normals point out of the box,
    or into it with `inward` (the inside of a room)."""
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
            n[axis] = -sign if inward else sign
            pts.append(p)
            nrm.append(np.repeat(n[None], len(p), axis=0))
    return np.concatenate(pts), np.concatenate(nrm)


def room_surface(room: Box, obstacles, spacing: float, flush: float = 0.15) -> tuple[np.ndarray, np.ndarray]:
    """Inner faces of the room (floor and ceiling included) + the visible outer faces of the obstacles (ADR-K11)."""
    P, N = box_surface(room, spacing, bottom=True, inward=True)
    keep = np.ones(len(P), dtype=bool)
    for b in obstacles:                                   # floor (and walls) hidden under/behind an obstacle
        keep &= ~np.all((P >= b.lo - 1e-6) & (P <= b.hi + 1e-6), axis=1)
    P, N = P[keep], N[keep]
    for b in obstacles:
        q, n = box_surface(b, spacing)                    # no bottom face
        on_room = np.any(np.abs(q - room.lo) < flush, axis=1) | np.any(np.abs(q - room.hi) < flush, axis=1)
        facing_wall = np.zeros(len(q), dtype=bool)        # a face whose normal points at a wall closer than `flush`
        for ax in range(3):
            facing_wall |= (n[:, ax] < 0) & (q[:, ax] - room.lo[ax] < flush)
            facing_wall |= (n[:, ax] > 0) & (room.hi[ax] - q[:, ax] < flush)
        inside = np.all((q > room.lo) & (q < room.hi), axis=1) | on_room
        k = inside & ~facing_wall
        P, N = np.concatenate([P, q[k]]), np.concatenate([N, n[k]])
    return P, N


def gt_surface(scene, spacing: float) -> tuple[np.ndarray, np.ndarray]:
    """The scene's GT-visible surface (gt_observable.npz beside gt_mesh, sim/build_scene.py), thinned to one point per `spacing`
    voxel: the surface the `coverage_oracle` method plans on. Reads GT: sim only, an upper bound."""
    if scene.gt_mesh is None or not (scene.gt_mesh.parent / "gt_observable.npz").exists():
        raise FileNotFoundError(f"{scene.name}: no gt_observable.npz (rebuild the scene with sim/build_scene.py)")
    d = np.load(scene.gt_mesh.parent / "gt_observable.npz")
    P, N = d["P"].astype(float), d["N"].astype(float)
    _, first = np.unique(np.floor(P / spacing).astype(np.int64), axis=0, return_index=True)
    return P[first], N[first]


def segments_hit_box(o: np.ndarray, P: np.ndarray, box: Box, end_margin: float = 0.02) -> np.ndarray:
    """For segments o -> P[i] (vectorised slab test): does the segment pass through `box` before reaching P[i]? The last
    `end_margin` m are ignored, so a point on the box's own surface does not occlude itself."""
    d = P - o[None]
    length = np.linalg.norm(d, axis=1)
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1.0 / d
        t1 = (box.lo[None] - o[None]) * inv
        t2 = (box.hi[None] - o[None]) * inv
    tmin = np.where(np.isnan(t1), -np.inf, np.minimum(t1, t2))
    tmax = np.where(np.isnan(t1), np.inf, np.maximum(t1, t2))
    # an axis with d == 0: inside the slab -> no constraint, outside -> never hits
    par = d == 0
    outside = par & ((o[None] < box.lo[None]) | (o[None] > box.hi[None]))
    tmin = np.where(par, -np.inf, tmin)
    tmax = np.where(par, np.inf, tmax)
    t_in = np.maximum(tmin.max(axis=1), 0.0)
    t_out = tmax.min(axis=1)
    t_end = 1.0 - end_margin / np.maximum(length, 1e-9)
    return (t_in <= t_out) & (t_in < t_end) & ~outside.any(axis=1)


class MeshOcclusion:
    """Self-occlusion test against a mesh (the GT mesh: oracle only): a depth image ray-cast at 1/`scale` resolution per pose."""

    def __init__(self, mesh_file, scale: int = 4, tol: float = 0.03):
        import open3d as o3d
        m = o3d.t.io.read_triangle_mesh(str(mesh_file))
        self.scene = o3d.t.geometry.RaycastingScene()
        self.scene.add_triangles(o3d.t.geometry.TriangleMesh(m.vertex.positions.to(o3d.core.float32),
                                                              m.triangle.indices.to(o3d.core.int32)))
        self.scale, self.tol = scale, tol
        self.offset = np.zeros(3)                       # CoverageState.shift moves it with the surface

    def depth(self, pose: Pose, K: Intrinsics) -> np.ndarray:
        """(h, w) optical depth (m, inf = no hit) at 1/scale of K's resolution."""
        import open3d as o3d
        s = self.scale
        w, h = K.width // s, K.height // s
        u, v = np.meshgrid((np.arange(w) + 0.5) * s, (np.arange(h) + 0.5) * s)
        d = np.stack([(u - K.cx) / K.fx, (v - K.cy) / K.fy, np.ones_like(u)], -1).reshape(-1, 3) @ quat_to_R(pose.q).T
        o = np.broadcast_to(pose.p - self.offset, d.shape)
        rays = o3d.core.Tensor(np.hstack([o, d]).astype(np.float32))
        return self.scene.cast_rays(rays)["t_hit"].numpy().reshape(h, w)   # unnormalised d, z = 1: t = optical depth

    def hidden(self, pose: Pose, K: Intrinsics, u: np.ndarray, v: np.ndarray, z: np.ndarray) -> np.ndarray:
        D = self.depth(pose, K)
        iu = np.clip((u / self.scale).astype(int), 0, D.shape[1] - 1)
        iv = np.clip((v / self.scale).astype(int), 0, D.shape[0] - 1)
        return z > D[iv, iu] + self.tol


class CoverageState:
    def __init__(self, roi: Box, K: Intrinsics, spacing: float = 0.04, depth_range=(0.2, 4.0), max_incidence_deg=75.0,
                 tri_min_deg=8.0, novel_deg=25.0, max_views=6, border=0.03, surface=None, occluders=(), min_overlap=0.0,
                 match_max_deg=180.0, min_shared=0, k_min=2, occlusion: MeshOcclusion | None = None):
        self.P, self.N = surface if surface is not None else box_surface(roi, spacing)
        self.K = K
        self.depth_range = depth_range
        self.cos_inc = np.cos(np.radians(max_incidence_deg))
        self.cos_tri = np.cos(np.radians(tri_min_deg))
        self.cos_match = np.cos(np.radians(match_max_deg))   # rays closer than this can be matched (SfM)
        self.cos_novel = np.cos(np.radians(novel_deg))
        self.max_views = max_views
        self.border = border
        self.occluders = tuple(occluders)
        self.min_overlap = float(min_overlap)              # fraction of the view already seen matchably (SfM connectivity)
        self.min_shared = int(min_shared)                  # ... or an absolute number of such points (room scenes, ADR-K11)
        self.k_min = max(2, int(k_min))                    # frames a point needs to count as covered
        self.occlusion = occlusion
        self.rays = np.zeros((len(self.P), max_views, 3))   # unit rays point -> camera of the observing frames
        self.count = np.zeros(len(self.P), dtype=int)
        self.pair = np.zeros(len(self.P), dtype=bool)       # two of its rays are in the triangulable + matchable band
        self.tri = np.zeros(len(self.P), dtype=bool)        # pair and >= k_min rays: covered
        self.frames: list[int] = []
        self.centres: list[np.ndarray] = []                 # camera centres of the frames (the fvs baseline's distance)
        self._vis_cache: dict = {}

    @classmethod
    def for_scene(cls, scene, K: Intrinsics, cc: dict, surface=None, occlusion=None) -> "CoverageState":
        """The coverage model of a scene from the `coverage` config block (object: ROI faces, or `surface` (P, N) when given;
        room: room + obstacles)."""
        kw = dict(spacing=cc["spacing_m"], depth_range=tuple(cc["depth_range_m"]), max_incidence_deg=cc["max_incidence_deg"],
                  tri_min_deg=cc["tri_min_deg"], novel_deg=cc["novel_deg"], max_views=cc["max_views"],
                  match_max_deg=cc.get("match_max_deg", 180.0), min_shared=cc.get("min_shared", 0), k_min=cc.get("k_min", 2))
        if scene.kind == "room":
            return cls(scene.roi, K, surface=room_surface(scene.roi, scene.no_fly, cc["spacing_m"]), occluders=scene.no_fly,
                       min_overlap=cc.get("min_overlap", 0.0), **kw)
        return cls(scene.roi, K, surface=surface, min_overlap=cc.get("min_overlap", 0.0), occlusion=occlusion, **kw)

    def shift(self, d) -> None:
        """Move the proxy surface and occluders by `d` in map (types.shifted); only before the first frame."""
        assert not self.frames, "shift the coverage model before adding frames"
        d = np.asarray(d, dtype=float)
        self.P = self.P + d
        self.occluders = tuple(Box(b.center + d, b.size, b.yaw) for b in self.occluders)
        if self.occlusion is not None:
            self.occlusion.offset = self.occlusion.offset + d
        self._vis_cache.clear()

    # --- visibility -------------------------------------------------------------------------------------------------
    def visible(self, pose: Pose, K: Intrinsics | None = None) -> tuple[np.ndarray, np.ndarray]:
        """(mask over surface points, unit rays point->camera) for a camera at `pose`. Cached per pose for the planner's
        intrinsics (candidate poses repeat every step)."""
        key = None
        if K is None or K == self.K:
            key = (tuple(np.round(pose.p, 4)), tuple(np.round(pose.q, 5)))
            hit = self._vis_cache.get(key)
            if hit is not None:
                return hit
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
        if self.occluders and m.any():
            idx = np.flatnonzero(m)
            blocked = np.zeros(len(idx), dtype=bool)
            for b in self.occluders:
                blocked |= segments_hit_box(pose.p, self.P[idx], b)
            m[idx[blocked]] = False
        if self.occlusion is not None and m.any():
            idx = np.flatnonzero(m)
            m[idx[self.occlusion.hidden(pose, K, u[idx], v[idx], z[idx])]] = False
        if key is not None:
            self._vis_cache[key] = (m, ray)
        return m, ray

    # --- state ------------------------------------------------------------------------------------------------------
    def add(self, frame: Frame) -> int:
        """Record a captured frame at its reported pose; returns the number of points it observed."""
        if frame.K != self.K:
            self._vis_cache.clear()
        self.K = frame.K
        m, ray = self.visible(frame.pose, frame.K)
        for i in np.flatnonzero(m):
            c = self.count[i]
            if c and not self.pair[i]:
                cos = self.rays[i, :c] @ ray[i]
                self.pair[i] = bool(np.any((cos < self.cos_tri) & (cos >= self.cos_match)))
            if c < self.max_views:
                self.rays[i, c] = ray[i]
                self.count[i] = c + 1
            self.tri[i] = self.pair[i] and c + 1 >= self.k_min
        self.frames.append(frame.idx)
        self.centres.append(np.asarray(frame.pose.p, dtype=float))
        return int(m.sum())

    def _closest(self, idx: np.ndarray, ray: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(cos to every stored ray, -1 where unused; cos of the closest one) for the points `idx` seen along `ray`."""
        c = self.count[idx]
        cos = np.einsum("ikj,ij->ik", self.rays[idx], ray[idx])        # (n, max_views)
        cos = np.where(np.arange(self.max_views)[None] < c[:, None], cos, -1.0)
        return cos, cos.max(axis=1)

    def overlap(self, pose: Pose) -> float:
        """Fraction of what a frame at `pose` would see that is already seen from a matchable direction (<= match_max_deg):
        the share of its view that feature matching can tie to the existing images."""
        m, ray = self.visible(pose)
        idx = np.flatnonzero(m)
        if not len(idx):
            return 0.0
        _, best = self._closest(idx, ray)
        return float(((self.count[idx] > 0) & (best >= self.cos_match)).mean())

    def gain(self, pose: Pose) -> float:
        """Expected value of a frame at `pose`, normalised by the surface size (0..1):
        1.0 per uncovered point it would cover (completes a band pair and k_min rays), 0.5 per unseen point (first ray) and per
        matchable ray toward k_min,
        0.1 per triangulated point it adds a novel direction to (> novel_deg from all rays, fewer than max_views).
        0 when it shares less than `min_overlap` of its view, or fewer than `min_shared` points, with what is already seen from a
        matchable direction (it could not register in SfM)."""
        m, ray = self.visible(pose)
        idx = np.flatnonzero(m)
        if not len(idx):
            return 0.0
        c = self.count[idx]
        cos, best = self._closest(idx, ray)                             # best = closest existing ray
        matchable = (c > 0) & (best >= self.cos_match)                  # an existing ray close enough to match
        if self.frames and (matchable.mean() < self.min_overlap or matchable.sum() < self.min_shared):
            return 0.0
        used = np.arange(self.max_views)[None] < c[:, None]
        pairs = np.any(used & (cos < self.cos_tri) & (cos >= self.cos_match), axis=1)   # an existing ray to triangulate with
        g = np.where(~matchable, 0.5, 0.0)                              # first ray from a new (unmatchable) direction
        open_ = (c > 0) & ~self.tri[idx]
        g += np.where(open_ & (self.pair[idx] | pairs) & (c + 1 >= self.k_min), 1.0, 0.0)
        g += np.where(open_ & matchable & (c + 1 < self.k_min), 0.5, 0.0)
        g += np.where(self.tri[idx] & (c < self.max_views) & (best < self.cos_novel), 0.1, 0.0)
        return float(g.sum() / len(self.P))

    @property
    def coverage(self) -> float:
        """Fraction of the proxy surface that is triangulated."""
        return float(self.tri.mean())

    @property
    def seen(self) -> float:
        return float((self.count > 0).mean())
