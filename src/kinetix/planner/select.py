"""argmax of gain − λ·travel − μ·|Δyaw| over the feasible candidates (architecture.md §7)."""
from __future__ import annotations

import numpy as np

from kinetix.core.geometry import camera_yaw, wrap
from kinetix.core.types import Candidate, Pose


def utility(cands: list[Candidate], gains: np.ndarray, current: Pose, travel_w: float, yaw_w: float) -> np.ndarray:
    d = np.array([np.linalg.norm(c.pose.p - current.p) for c in cands])
    y0 = camera_yaw(current)
    dy = np.array([abs(wrap(camera_yaw(c.pose) - y0)) for c in cands])
    return gains - travel_w * d - yaw_w * dy


def select(cands: list[Candidate], gains: np.ndarray, feasible: np.ndarray, current: Pose, travel_w: float,
           yaw_w: float) -> int | None:
    """Index of the chosen candidate, or None when no feasible candidate has a positive gain."""
    u = utility(cands, gains, current, travel_w, yaw_w)
    u = np.where(feasible & (gains > 0), u, -np.inf)
    i = int(np.argmax(u))
    return None if not np.isfinite(u[i]) else i
