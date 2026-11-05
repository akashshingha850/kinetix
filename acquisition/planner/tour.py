"""Tour planner (ADR-K12): plan the whole capture before take-off, then fly it continuously.

Stop-and-shoot NBV spends its flight on two things that do not make images: zig-zag hops (the greedy next view ignores the
route: 6.6 m per hop in the Warehouse) and stopping (accelerate, turn, settle, grab: ~5 s per view). For a known room the
SOTA recipe for fast aerial reconstruction is a viewpoint set + a TSP over it + a continuous trajectory (PredRecon, Plan3D,
FC-Planner), with images taken while moving. This module is that recipe on the existing coverage model:

1. `select_viewpoints`: greedy set cover on the proxy surface (gain only, no travel cost) up to `coverage_target`;
2. `order_tour`: open TSP from the start pose, nearest neighbour + 2-opt, cost = metres + `yaw_cost_m` per radian of turn;
3. `sample_path`: the polyline through the ordered views, sampled every `step_m` with yaw interpolated per segment; what
   a continuously recording camera sees (used to score a plan on the dry rig, and as the rig's carrot path);
4. `run_tour`: plan, then fly the path once and keep keyframes (after an exploration of the room with `explore.enabled`).

Pure numpy: never imports rclpy or a rig adapter (dependency rule).
"""
from __future__ import annotations

import time

import numpy as np

from acquisition.planner.constraints import BoxConstraints
from acquisition.planner.loop import apply_shift
from acquisition.planner.sampler import GridSampler
from kinetix.coverage import segments_hit_box
from kinetix.geometry import camera_pose, camera_yaw, wrap
from kinetix.types import Candidate, Frame, Pose, RigError, Scene, shifted


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


def reach_range(plan, positions: np.ndarray, pitch: float, factor: float, near_m: float) -> np.ndarray:
    """Per-point far limit for `plan` (a CoverageState): `factor` x the closest distance from which any of `positions` can see
    the point (line of sight, incidence limit, inside the vertical field of view of a camera at mount `pitch`; yaw is free),
    clipped to [near_m, plan's far limit]. A wall is then only counted as covered from close up, while the ceiling and the
    tops of the walls, which a level camera sees only from afar, keep the range they need."""
    far = float(plan.depth_range[1])
    K = plan.K
    half_v = np.arctan((0.5 - plan.border) * K.height / K.fy)
    best = np.full(len(plan.P), np.inf)
    for c in positions:
        d = c[None] - plan.P                          # point -> camera
        dist = np.linalg.norm(d, axis=1)
        ray = d / np.maximum(dist[:, None], 1e-9)
        elev = np.arcsin(np.clip(-ray[:, 2], -1.0, 1.0))                 # of the camera -> point direction
        ok = (np.einsum("ij,ij->i", plan.N, ray) > plan.cos_inc) & (np.abs(elev + pitch) < half_v)   # pitch > 0 = tilted down
        idx = np.flatnonzero(ok & (dist < best))
        for b in plan.occluders:
            if len(idx):
                idx = idx[~segments_hit_box(c, plan.P[idx], b)]
        best[idx] = dist[idx]
    return np.clip(factor * best, near_m, far)


def _cost(a: Pose, b: Pose, yaw_cost_m: float) -> float:
    return float(np.linalg.norm(a.p - b.p) + yaw_cost_m * abs(wrap(camera_yaw(a) - camera_yaw(b))))


def order_tour(start: Pose, poses: list[Pose], yaw_cost_m: float = 1.0, iters: int = 50, blocked=None) -> list[int]:
    """Visiting order of `poses` for an open tour from `start` (nearest neighbour, then 2-opt until no improvement).
    `blocked(p, q)` (optional): True for a straight leg that may not be flown; such legs cost 1 km, so they are used only
    when nothing else connects."""
    n = len(poses)
    if n < 2:
        return list(range(n))
    nodes = [start] + list(poses)
    C = np.array([[_cost(a, b, yaw_cost_m) + (1e3 if blocked is not None and blocked(a.p, b.p) else 0.0) for b in nodes]
                  for a in nodes])
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


def run_tour(cfg: dict, scene: Scene, rig, recon, log, truth: Scene | None = None) -> dict:
    """Plan the whole capture, then fly it without stopping and keep keyframes (ADR-K12, planner/tour.py). With
    `explore.enabled` (ADR-K13) the room is not read from the scene file: the drone first explores it with the ZED depth
    (planner/explore.py) and the tour is planned on the fitted scene; `truth` (sim) only scores that fit."""
    from kinetix.coverage import CoverageState
    pc, tc = cfg["planner"], cfg["planner"]["tour"]
    stop_reason, explored = None, []
    try:
        rig.start()                                   # inside the try: a failed start still runs finish()
        if cfg.get("explore", {}).get("enabled"):
            from acquisition.planner.explore import run_explore
            shift = np.asarray(getattr(rig, "map_shift", np.zeros(3)), dtype=float)
            scene, rep = run_explore(cfg, scene, rig, log, shifted(truth, shift) if truth is not None else None, explored)
            if scene is None:
                return {"stop_reason": "explore_failed", "coverage": 0.0, "seen": 0.0}
            seen = {k: v for k, v in rep["faces"].items() if k != "ceiling"}      # the ceiling is far and often out of range
            if min(seen.values()) < cfg["explore"]["tour_min_wall"]:
                log.info(f"explore: {min(seen, key=seen.get)} seen only {min(seen.values()):.0%} (stop: {rep['stop_reason']}): the "
                         "fitted room is not trustworthy, landing instead of touring it")
                return {"stop_reason": "explore_incomplete", "coverage": 0.0, "seen": 0.0}
            recon = CoverageState.for_scene(scene, recon.K, cfg["coverage"])
            for f in explored:                        # what exploration already saw counts: the tour flies the gaps
                recon.add(f)
            log.info(f"tour on the explored room (stop: {rep['stop_reason']}); {len(explored)} reused views cover {recon.coverage:.3f}")
        else:
            scene = apply_shift(rig, scene, recon, log)
        t0 = time.time()
        g = pc["grid"]
        sampler = GridSampler(g["spacing_m"], g["heights"], g["n_yaw"], g.get("layer_m"))
        cands = sampler.sample(scene, recon, rig.caps)
        start = rig.pose()
        feas = BoxConstraints(pc["clearance_m"], pc["roi_clearance_m"]).feasible(cands, start, scene)
        if not feas.any():                            # e.g. the start is inside an obstacle's keep-out box: land instead of crashing
            log.info(f"tour: none of {len(cands)} candidate viewpoints is feasible from here")
            return {"stop_reason": "no_feasible_view", "coverage": round(recon.coverage, 4), "seen": round(recon.seen, 4)}
        cc = dict(cfg["coverage"], min_shared=0, min_overlap=0.0)   # continuous capture links the views
        plan = CoverageState.for_scene(scene, recon.K, cc)
        if tc.get("range_factor"):                    # count a surface as covered only from near its closest reachable view
            pos = np.unique(np.round([cands[j].pose.p for j in np.flatnonzero(feas)], 3), axis=0)
            reach = reach_range(plan, pos, rig.caps.mount_pitch_rad, tc["range_factor"], tc["near_m"])
            plan.depth_range = (plan.depth_range[0], reach)
        for f in explored:
            plan.add(f)
        vp = select_viewpoints(cands, feas, plan, tc["coverage_target"], pc["budget"])
        poses = [cands[j].pose for j in vp]
        order = order_tour(start, poses, tc["yaw_cost_m"])
        tour = [poses[i] for i in order]
        dist, yaw = tour_length(start, tour)
        est = flight_time(dist, yaw, len(tour), cfg["rig"]["cruise_mps"], np.radians(cfg["rig"]["yaw_rate_deg_s"]))
        rec = {"step": 0, "plan": "tour", "n_cands": len(cands), "n_feasible": int(feas.sum()), "viewpoints": len(tour),
               "plan_coverage": round(plan.coverage, 4), "tour_m": round(dist, 1), "tour_yaw_deg": round(float(np.degrees(yaw))),
               "est_flight_s": round(est, 1), "t_plan_s": round(time.time() - t0, 2),
               "waypoints": [[cands[vp[i]].pool_id] for i in order]}
        log.step(rec)
        log.info(f"tour: {len(tour)} viewpoints (plan coverage {plan.coverage:.3f}), {dist:.0f} m, {np.degrees(yaw):.0f} deg "
                 f"of turn, est. flight {est / 60:.1f} min, planned in {rec['t_plan_s']} s")
        path = sample_path(start, tour, tc["path_step_m"], np.radians(tc["path_step_deg"]), rig.caps.mount_pitch_rad)

        def on_frame(frame):
            recon.add(frame)
            log.frame(frame)
            if frame.idx % 25 == 0:
                log.info(f"keyframe {frame.idx}: coverage {recon.coverage:.3f} (seen {recon.seen:.3f}), "
                         f"travel {log.travel_m:.0f} m")
        try:
            rig.fly_path(path, tc["keyframe_step_m"], np.radians(tc["keyframe_step_deg"]), on_frame)
            stop_reason = "tour_done"
        except RigError as e:
            log.info(f"tour: {e}")
            stop_reason = e.kind
    finally:
        rig.finish()
    return {"stop_reason": stop_reason, "coverage": round(recon.coverage, 4), "seen": round(recon.seen, 4)}
