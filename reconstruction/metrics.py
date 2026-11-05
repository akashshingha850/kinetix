"""Geometric metrics of a reconstruction against GT (paper/benchmark.md §3). Thresholds are fractions of the ROI diagonal D,
so objects of different sizes compare. Inputs are points in the GT's frame (already aligned and cropped)."""
from __future__ import annotations

import numpy as np

TAUS = (0.005, 0.01)                                   # F-score thresholds, fraction of D


def to_mesh(points: np.ndarray, V: np.ndarray, F: np.ndarray) -> np.ndarray:
    """Distance (m) from each point to the triangle mesh (V, F)."""
    import open3d as o3d
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.core.Tensor(np.asarray(V, np.float32)), o3d.core.Tensor(np.asarray(F, np.uint32)))
    return scene.compute_distance(o3d.core.Tensor(np.asarray(points, np.float32))).numpy().astype(float)


def to_points(query: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Distance (m) from each query point to its nearest neighbour in `points`."""
    import open3d as o3d
    pc = lambda P: o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(P, float)))   # noqa: E731
    return np.asarray(pc(query).compute_point_cloud_distance(pc(points)), dtype=float)


def score(points: np.ndarray, V: np.ndarray, F: np.ndarray, gt_obs: np.ndarray, diag: float, taus=TAUS) -> dict:
    """accuracy = mean recon -> GT mesh, completeness = mean GT observable points -> recon (mm), Chamfer = their mean,
    F-score@tau = harmonic mean of precision (recon within tau of the mesh) and recall (GT within tau of the recon)."""
    out = {"n_points": int(len(points)), "diag_m": round(float(diag), 4)}
    if not len(points):
        return {**out, "accuracy_mm": None, "completeness_mm": None, "chamfer_mm": None,
                "fscore": {f"tau_{t * 100:g}pct": 0.0 for t in taus}, "precision": {f"tau_{t * 100:g}pct": 0.0 for t in taus},
                "completeness_ratio": {f"tau_{t * 100:g}pct": 0.0 for t in taus}}
    acc, comp = to_mesh(points, V, F), to_points(gt_obs, points)
    out.update(accuracy_mm=round(1000 * acc.mean(), 3), completeness_mm=round(1000 * comp.mean(), 3),
               chamfer_mm=round(500 * (acc.mean() + comp.mean()), 3), fscore={}, precision={}, completeness_ratio={})
    for t in taus:
        k = f"tau_{t * 100:g}pct"
        p, r = float((acc < t * diag).mean()), float((comp < t * diag).mean())
        out["precision"][k], out["completeness_ratio"][k] = round(p, 4), round(r, 4)
        out["fscore"][k] = round(2 * p * r / (p + r), 4) if p + r else 0.0
    return out
