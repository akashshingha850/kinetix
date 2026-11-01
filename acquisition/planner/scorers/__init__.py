"""Scorers: score(cands, recon) -> gains >= 0 (interfaces.md §3). Selected by name (`method`, make_scorer). A scorer with
`add(frame)` is also fed every captured frame (its own map)."""
from __future__ import annotations

import numpy as np

from kinetix.geometry import quat_to_R
from kinetix.types import Candidate


class RoiCoverageScorer:
    """Photogrammetric coverage gain on the ROI proxy surface (recon/coverage.py). GT-free: reads only the scene ROI and
    the frames captured so far. A stand-in until the K3 scorers (sfm_uncertainty, depth_frontier, combo) exist."""
    name = "roi_coverage"
    uses_gt = False

    def score(self, cands: list[Candidate], recon) -> np.ndarray:
        return np.array([recon.gain(c.pose) for c in cands])


class RandomScorer:
    name = "random"
    uses_gt = False

    def __init__(self, seed: int = 0):
        self.rng = np.random.default_rng(seed)

    def score(self, cands: list[Candidate], recon) -> np.ndarray:
        return self.rng.uniform(0.01, 1.0, len(cands))


class FvsScorer:
    """Farthest-view sampling (the strong simple baseline of NeRF Director, Xiao et al. 2024): the view whose camera centre is
    farthest from every captured one. Ignores the scene; on the ring stack every view already looks at the ROI axis."""
    name = "fvs"
    uses_gt = False

    def score(self, cands: list[Candidate], recon) -> np.ndarray:
        if not recon.centres:
            return np.ones(len(cands))
        C = np.array(recon.centres)
        P = np.array([c.pose.p for c in cands])
        return np.linalg.norm(P[:, None] - C[None], axis=2).min(axis=1) + 1e-6


class VolumetricIgScorer:
    """Classical volumetric NBV (Isler et al. 2016, Delmerico et al. 2018 family): the number of unknown voxels of the ROI a
    view would observe, cast through an occupancy map of the depth captured so far (a ray stops at its first occupied voxel).
    Needs depth on every frame (the rig's sensor depth: GT render on the pool / isaac rigs, the ZED on mavros)."""
    name = "volumetric_ig"
    uses_gt = False

    def __init__(self, roi, voxel: float = 0.04, rays: tuple[int, int] = (40, 24), depth_range=(0.2, 4.0)):
        from acquisition.mapping.depthmap import OccupancyMap
        self.map = OccupancyMap(roi.lo, roi.hi, voxel, depth_range, stride=4)
        self.rays = rays
        self.n_frames = 0

    def add(self, frame) -> None:
        if frame.depth is None:
            raise ValueError("volumetric_ig needs the frame's depth (rig.save_depth / a rig with sensor depth)")
        self.map.integrate(frame.depth, frame.K, frame.pose)
        self.n_frames += 1

    def score(self, cands: list[Candidate], recon) -> np.ndarray:
        K = recon.K
        u = np.linspace(0.5, K.width - 0.5, self.rays[0])
        v = np.linspace(0.5, K.height - 0.5, self.rays[1])
        uu, vv = np.meshgrid(u, v)
        d = np.stack([(uu.ravel() - K.cx) / K.fx, (vv.ravel() - K.cy) / K.fy, np.ones(uu.size)], 1)
        d /= np.linalg.norm(d, axis=1, keepdims=True)
        m = self.map
        lo, hi = m.lo, m.lo + np.array(m.shape) * m.voxel
        step = 0.5 * m.voxel
        out = np.zeros(len(cands))
        for j, c in enumerate(cands):
            o = np.asarray(c.pose.p, dtype=float)
            D = d @ quat_to_R(c.pose.q).T
            with np.errstate(divide="ignore", invalid="ignore"):               # slab entry/exit of the map box per ray
                t1, t2 = (lo - o) / D, (hi - o) / D
            t_in = np.nan_to_num(np.minimum(t1, t2), nan=-np.inf).max(axis=1).clip(min=m.depth_range[0])
            t_out = np.nan_to_num(np.maximum(t1, t2), nan=np.inf).min(axis=1).clip(max=m.depth_range[1])
            hit = t_out > t_in
            if not hit.any():
                continue
            ts = np.arange(0.0, (t_out[hit] - t_in[hit]).max() + step, step)
            pts = o + D[hit, None, :] * (t_in[hit, None, None] + ts[None, :, None])     # (rays, samples, 3)
            inside = ts[None] <= (t_out[hit] - t_in[hit])[:, None]
            ijk = np.clip(np.floor((pts - lo) / m.voxel).astype(np.int64), 0, np.array(m.shape) - 1)
            L = m.L[ijk[..., 0], ijk[..., 1], ijk[..., 2]]
            blocked = np.cumsum((L > 0) & inside, axis=1) > 0                # at or past the first occupied voxel
            unk = (L == 0) & inside & ~blocked
            flat = np.ravel_multi_index((ijk[..., 0][unk], ijk[..., 1][unk], ijk[..., 2][unk]), m.shape)
            out[j] = len(np.unique(flat))
        return out / max(int(np.prod(m.shape)), 1) + 1e-9 * (out > 0)


SCORERS = {"roi_coverage": RoiCoverageScorer, "random": RandomScorer, "fvs": FvsScorer, "volumetric_ig": VolumetricIgScorer}


def make_scorer(method: str, seed: int, scene, cfg: dict):
    """The scorer for `method`. depth_coverage / coverage_oracle score like roi_coverage; only their surface differs (cli.py)."""
    if method == "random":
        return RandomScorer(seed)
    if method == "volumetric_ig":
        vc = cfg["planner"]["volumetric_ig"]
        return VolumetricIgScorer(scene.roi, vc["voxel_m"], tuple(vc["rays"]), tuple(cfg["coverage"]["depth_range_m"]))
    return SCORERS.get(method, RoiCoverageScorer)()
