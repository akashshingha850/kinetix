"""Monocular depth for planning (methodology.md M3, sensing.plan_depth `mono` / `mono_metric_noanchor`).

`MonoDepth` runs Depth Anything V2 (transformers; Small checkpoints, Apache-2.0) on one image. The relative model returns
affine-invariant inverse depth; `anchor` fits 1/z = s·r + t to the SfM points the frame sees (their depth in this camera),
robustly, which puts it in the SfM's metric frame: C1's SfM anchoring. The metric model is used as is (the no-anchor ablation)."""
from __future__ import annotations

import numpy as np

RELATIVE = "depth-anything/Depth-Anything-V2-Small-hf"
METRIC_INDOOR = "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf"


class MonoDepth:
    def __init__(self, model: str = RELATIVE, device: str | None = None):
        import torch
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation
        self.torch = torch
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.proc = AutoImageProcessor.from_pretrained(model)
        self.model = AutoModelForDepthEstimation.from_pretrained(model).to(self.device).eval()

    def __call__(self, rgb: np.ndarray) -> np.ndarray:
        """(H, W) float32 at the image's size: relative inverse depth (RELATIVE) or metres (METRIC_*)."""
        t = self.torch
        x = self.proc(images=rgb, return_tensors="pt").to(self.device)
        with t.no_grad():
            d = self.model(**x).predicted_depth[None]
        return t.nn.functional.interpolate(d, size=rgb.shape[:2], mode="bilinear", align_corners=False)[0, 0].cpu().numpy()


def anchor(rel: np.ndarray, u: np.ndarray, v: np.ndarray, z: np.ndarray, min_points: int = 20, iters: int = 3):
    """(s, t, inliers) with 1/z ≈ s·rel[v, u] + t over the anchors (pixel u, v; depth z > 0), or None below `min_points`.
    Least squares, then twice refit on the points within 3 MAD of the residual (outliers: mismatched or occluded SfM points)."""
    r = rel[np.clip(v.astype(int), 0, rel.shape[0] - 1), np.clip(u.astype(int), 0, rel.shape[1] - 1)]
    y = 1.0 / z
    keep = np.isfinite(r) & np.isfinite(y) & (z > 0)
    if keep.sum() < min_points:
        return None
    for _ in range(iters):
        A = np.c_[r[keep], np.ones(keep.sum())]
        s, t = np.linalg.lstsq(A, y[keep], rcond=None)[0]
        res = np.abs(s * r + t - y)
        mad = np.median(res[keep]) + 1e-9
        new = keep & (res <= 3 * 1.4826 * mad)
        if new.sum() < min_points or (new == keep).all():
            break
        keep = new
    return float(s), float(t), int(keep.sum())


def to_depth(rel: np.ndarray, s: float, t: float, max_m: float = 20.0) -> np.ndarray:
    """Anchored metric depth (m); NaN where s·rel + t gives no positive depth below max_m."""
    inv = s * rel + t
    with np.errstate(divide="ignore", invalid="ignore"):
        d = 1.0 / inv
    return np.where((inv > 0) & (d < max_m), d, np.nan).astype(np.float32)
