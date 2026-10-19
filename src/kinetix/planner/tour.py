"""Tour planner (ADR-K12): plan the whole capture before take-off, then fly it continuously.

Stop-and-shoot NBV spends its flight on two things that do not make images: zig-zag hops (the greedy next view ignores the
route: 6.6 m per hop in the Warehouse) and stopping (accelerate, turn, settle, grab: ~5 s per view). For a known room the
SOTA recipe for fast aerial reconstruction is a viewpoint set + a TSP over it + a continuous trajectory (PredRecon, Plan3D,
FC-Planner), with images taken while moving. This module is that recipe on the existing coverage model:

1. `select_viewpoints`: greedy set cover on the proxy surface (gain only, no travel cost) up to `coverage_target`;
2. `order_tour`: open TSP from the start pose, nearest neighbour + 2-opt, cost = metres + `yaw_cost_m` per radian of turn;
3. `sample_path`: the polyline through the ordered views, sampled every `step_m` with yaw interpolated per segment; what
   a continuously recording camera sees (used to score a plan on the dry rig, and as the rig's carrot path).

Pure numpy: never imports rclpy or a rig adapter (dependency rule).
"""
from __future__ import annotations

import numpy as np

from kinetix.core.geometry import camera_pose, camera_yaw, wrap
from kinetix.core.types import Candidate, Frame, Pose


def select_viewpoints(cands: list[Candidate], feasible: np.ndarray, recon, coverage_target: float, budget: int,
                      min_gain: float = 1e-4) -> list[int]:
    """Indices of a greedy covering set: repeatedly take the feasible candidate with the largest coverage gain and record it
    as a frame on `recon` (a scratch CoverageState), until the target, the budget or no gain."""
    chosen: list[int] = []
    idx = np.flatnonzero(feasible)
    while len(chosen) < budget and recon.coverage < coverage_target:
        gains = np.array([recon.gain(cands[j].pose) for j in idx])
        k = int(np.argmax(gains))
        if gains[k] < min_gain:
            break
        j = int(idx[k])
        chosen.append(j)
        p = cands[j].pose
        recon.add(Frame(len(chosen) - 1, 0.0, None, recon.K, p, p, None, 0.0))
        idx = np.delete(idx, k)
    return chosen


def _cost(a: Pose, b: Pose, yaw_cost_m: float) -> float:
    return float(np.linalg.norm(a.p - b.p) + yaw_cost_m * abs(wrap(camera_yaw(a) - camera_yaw(b))))


def order_tour(start: Pose, poses: list[Pose], yaw_cost_m: float = 1.0, iters: int = 50) -> list[int]:
    """Visiting order of `poses` for an open tour from `start` (nearest neighbour, then 2-opt until no improvement)."""
    n = len(poses)
    if n < 2:
        return list(range(n))
    nodes = [start] + list(poses)
    C = np.array([[_cost(a, b, yaw_cost_m) for b in nodes] for a in nodes])
    order = [0]
    left = set(range(1, n + 1))
    while left:
        nxt = min(left, key=lambda j: C[order[-1], j])
        order.append(nxt)
        left.remove(nxt)
    for _ in range(iters):                                       # 2-opt on the open path (node 0 stays first)
        improved = False
        for i in range(1, n):
            for k in range(i + 1, n + 1):
                a, b = order[i - 1], order[i]
                c = order[k]
                d = order[k + 1] if k + 1 <= n else None
                old = C[a, b] + (C[c, d] if d is not None else 0.0)
                new = C[a, c] + (C[b, d] if d is not None else 0.0)
                if new < old - 1e-9:
                    order[i:k + 1] = order[i:k + 1][::-1]
                    improved = True
        if not improved:
            break
    return [j - 1 for j in order[1:]]


def tour_length(start: Pose, poses: list[Pose]) -> tuple[float, float]:
    """(metres, radians of yaw) along start -> poses."""
    pts = [start] + list(poses)
    d = sum(float(np.linalg.norm(b.p - a.p)) for a, b in zip(pts, pts[1:]))
    y = sum(abs(wrap(camera_yaw(b) - camera_yaw(a))) for a, b in zip(pts, pts[1:]))
    return d, y


def sample_path(start: Pose, poses: list[Pose], step_m: float, step_yaw: float, pitch: float = 0.0) -> list[Pose]:
    """Camera poses along the polyline start -> poses, every `step_m` metres or `step_yaw` rad of turn (whichever is denser),
    yaw interpolated linearly along each segment (shorter way round); includes every waypoint."""
    out = [start]
    for a, b in zip([start] + list(poses), poses):
        ya, yb = camera_yaw(a), camera_yaw(b)
        dy = wrap(yb - ya)
        n = max(1, int(np.ceil(max(np.linalg.norm(b.p - a.p) / step_m, abs(dy) / step_yaw))))
        for k in range(1, n + 1):
            f = k / n
            out.append(camera_pose(a.p + f * (b.p - a.p), ya + f * dy, pitch))
    return out


def flight_time(dist_m: float, yaw_rad: float, n_corners: int, cruise_mps: float, yaw_rate: float,
                corner_s: float = 0.6) -> float:
    """Rough flight time of a continuous tour: cruise along the path, turns overlap the straight parts except where a turn
    is longer than its segment, plus a short slow-down per corner (no stopping)."""
    return dist_m / cruise_mps + max(0.0, yaw_rad / yaw_rate - dist_m / cruise_mps) + corner_s * n_corners
