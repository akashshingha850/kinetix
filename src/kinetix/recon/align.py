"""Sim3 alignment (architecture.md §6): Umeyama on corresponding points (camera centres), and ATE after alignment."""
from __future__ import annotations

import numpy as np


def umeyama(src: np.ndarray, dst: np.ndarray, with_scale: bool = True) -> tuple[float, np.ndarray, np.ndarray]:
    """(s, R, t) minimising |dst - (s R src + t)|^2 over corresponding rows (Umeyama 1991). Needs >= 3 non-collinear points."""
    src, dst = np.asarray(src, dtype=float), np.asarray(dst, dtype=float)
    if len(src) < 3:
        raise ValueError("umeyama needs at least 3 point pairs")
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1.0
    R = U @ S @ Vt
    var_s = (xs ** 2).sum() / len(src)
    s = float(np.trace(np.diag(D) @ S) / var_s) if with_scale else 1.0
    t = mu_d - s * R @ mu_s
    return s, R, t


def apply(s: float, R: np.ndarray, t: np.ndarray, X: np.ndarray) -> np.ndarray:
    return s * np.asarray(X, dtype=float) @ R.T + t


def ate(src: np.ndarray, dst: np.ndarray) -> dict:
    """Sim3-align src onto dst, then the translation error statistics (metres) and the scale."""
    s, R, t = umeyama(src, dst)
    err = np.linalg.norm(apply(s, R, t, src) - dst, axis=1)
    return {"rmse_m": float(np.sqrt((err ** 2).mean())), "median_m": float(np.median(err)), "max_m": float(err.max()),
            "scale": s, "n": int(len(src))}
