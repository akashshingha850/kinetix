"""Exploration pass for an unknown room (ADR-K13): fly a few views, map the ZED depth at the EKF2 pose, fit the room.

Replaces the scene file's room box and obstacles, the one thing `tour` otherwise needs from a person. It reads from the hint scene
only the take-off point and the altitude band (the flight envelope); never its room geometry.

  1. a yaw sweep at the take-off point (the map starts with free space all round the drone);
  2. repeat: take the frontier (free voxels next to unknown ones), score reachable viewpoints by how many frontier voxels they
     would see, fly to the best, integrate its depth. The drone only flies through space the map has seen to be empty with the
     flight clearance to spare, on a straight segment, so exploration never trusts a wall it has not seen;
  3. stop when every face of the fitted room has been seen (closed), when no frontier is left, or at the step limit.

The map is built at the rig pose (EKF2 ∘ camera mount): the estimate that flies the drone, so map and path share one frame.
"""
from __future__ import annotations

import json
from dataclasses import replace

import numpy as np
import yaml

from acquisition.mapping.depthmap import OccupancyMap
from acquisition.mapping.register import DriftTracker
from acquisition.mapping.roomfit import (
    AXIS_FACE,
    compare,
    fit_scene,
    scene_to_dict,
    translated,
)
from kinetix.geometry import camera_pose, camera_yaw, wrap
from kinetix.types import Box, Intrinsics, Pose, RigError, Scene

IDX0 = 900000                       # exploration images are numbered apart from the tour's keyframes (frames/NNNNNN.png)


def hint_scene(truth: Scene, volume, band=None) -> Scene:
    """What the planner and the rig get in exploration mode: the take-off point, an altitude band (`band`, else the scene file's)
    and a search volume where its room and obstacles were. No room geometry, no obstacles."""
    box = Box(np.asarray(truth.standby, dtype=float), np.asarray(volume, dtype=float))
    alt = truth.altitude if band is None else tuple(float(x) for x in band)
    return replace(truth, roi=box, workspace=box, no_fly=(), gt_mesh=None, kind="room", altitude=alt)


def in_frustum(points: np.ndarray, p: np.ndarray, R: np.ndarray, K: Intrinsics, dmin: float, dmax: float) -> np.ndarray:
    """Which map points lie in the view cone of a camera at p with rotation R (optical -> map): inside the depth range and 90 %
    of the field of view. Occlusion is ignored: it is a cheap estimate of what a view can resolve."""
    Xc = (points - p) @ R
    z = Xc[:, 2]
    return ((z > dmin) & (z < dmax) & (np.abs(Xc[:, 0]) < 0.9 * (K.width / 2) / K.fx * z)
            & (np.abs(Xc[:, 1]) < 0.9 * (K.height / 2) / K.fy * z))


def los_clear(omap: OccupancyMap, o: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Which points can be seen from o: the segment to them crosses no occupied voxel (the voxel holding the point itself and
    its immediate neighbours do not count: a frontier voxel sits next to what is seen)."""
    d = pts - o
    L = np.linalg.norm(d, axis=1)
    ok = np.ones(len(pts), dtype=bool)
    n_s = int(np.ceil(L.max() / (0.5 * omap.voxel))) + 1 if len(L) else 0
    occupied = omap.occupied                                  # once: the property is a pass over the whole grid
    for k in range(1, n_s):
        t = k / n_s
        idx, inside = omap.index(o + d * t)
        occ = inside & occupied[tuple(np.where(inside[:, None], idx, 0).T)]
        ok &= ~(occ & (t * L < L - 1.5 * omap.voxel))
    return ok


def straight_ok(omap: OccupancyMap, corridor: np.ndarray, cur: np.ndarray, pts: np.ndarray, exempt_m: float = 0.5) -> np.ndarray:
    """Which straight segments from cur to pts stay in the corridor (the first `exempt_m` is the drone's own neighbourhood)."""
    d = pts - cur
    L = np.linalg.norm(d, axis=1)
    ok = np.ones(len(pts), dtype=bool)
    if not len(pts):
        return ok
    n_s = int(np.ceil(L.max() / (0.5 * omap.voxel))) + 1
    for k in range(n_s + 1):
        t = k / n_s
        idx, inside = omap.index(cur + d * t)
        free_here = inside & corridor[tuple(np.where(inside[:, None], idx, 0).T)]
        ok &= free_here | (t * L < exempt_m)
    return ok


class Reach:
    """Flight distance from the drone through the corridor, on a coarse grid (`cell` voxels a side, 26-connected wavefront): which
    viewpoints can be reached at all, how far the flight there is, and the first straight leg of the way. A straight segment from
    the drone is too strict in clutter (around a crate, over a low one); the drone flies the leg, takes a depth image, replans.
    A coarse cell is passable when any of its voxels is corridor (cheap, permissive): the leg that is flown is checked on the fine
    grid."""
    OFFS = np.array([(i, j, k) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1) if (i, j, k) != (0, 0, 0)])

    def __init__(self, omap: OccupancyMap, corridor: np.ndarray, cur, cell: int = 2):
        self.omap, self.corridor, self.cur, self.cell = omap, corridor, np.asarray(cur, dtype=float), cell
        s_f = np.clip(omap.index(self.cur[None])[0][0], 0, np.array(omap.shape) - 1)
        nz = np.argwhere(corridor)
        lo = np.minimum(nz.min(axis=0), s_f) // cell if len(nz) else s_f // cell
        hi = np.maximum(nz.max(axis=0), s_f) // cell + 1 if len(nz) else s_f // cell + 1
        shape = hi - lo
        f0, f1 = lo * cell, np.minimum(hi * cell, omap.shape)
        fine = np.zeros(tuple(shape * cell), dtype=bool)                  # the corridor's bounding box, padded to whole cells
        fine[tuple(slice(0, b - a) for a, b in zip(f0, f1))] = corridor[tuple(slice(a, b) for a, b in zip(f0, f1))]
        P = fine.reshape(shape[0], cell, shape[1], cell, shape[2], cell).any(axis=(1, 3, 5))
        s = tuple(s_f // cell - lo)
        P[s] = True
        D = np.full(P.shape, -1, dtype=np.int32)
        D[s] = 0
        front, k = np.zeros(P.shape, dtype=bool), 0
        front[s] = True
        while front.any():
            k += 1
            front = OccupancyMap.dilate(front, 1, 1) & P & (D < 0)
            D[front] = k
        self.lo, self.D = lo, D

    def _cells(self, pts):
        c = self.omap.index(pts)[0] // self.cell - self.lo
        ok = np.all((c >= 0) & (c < np.array(self.D.shape)), axis=1)
        return np.where(ok[:, None], c, 0), ok

    def distance(self, pts: np.ndarray) -> np.ndarray:
        """Flight distance (m) to each point, at least the straight line; inf where the corridor does not lead."""
        c, ok = self._cells(pts)
        d = self.D[tuple(c.T)]
        geo = np.where(ok & (d >= 0), d * self.cell * self.omap.voxel, np.inf)
        return np.maximum(geo, np.linalg.norm(pts - self.cur, axis=1))

    def leg(self, goal: np.ndarray, min_m: float = 0.5):
        """The point on the way to `goal` nearest to it that a straight leg from the drone reaches (fine-grid check), or None."""
        c, ok = self._cells(goal[None])
        path = [c[0]]
        if not ok[0] or self.D[tuple(path[0])] < 0:
            return None
        sh = np.array(self.D.shape)
        while self.D[tuple(path[-1])] > 0:                                 # down the wavefront to the drone
            n = path[-1] + self.OFFS
            n = n[np.all((n >= 0) & (n < sh), axis=1)]
            n = n[self.D[tuple(n.T)] == self.D[tuple(path[-1])] - 1]
            path.append(n[np.argmin(np.abs(n - path[-1]).sum(axis=1))])  # face neighbours first: no corner cutting
        pts = self.omap.lo + ((np.array(path) + self.lo) * self.cell + self.cell / 2) * self.omap.voxel
        pts[0] = goal
        idx, inside = self.omap.index(pts)
        ok = inside & self.corridor[tuple(np.where(inside[:, None], idx, 0).T)]
        ok &= np.linalg.norm(pts - self.cur, axis=1) >= min_m
        ok[ok] = straight_ok(self.omap, self.corridor, self.cur, pts[ok])
        return pts[np.argmax(ok)] if ok.any() else None


def _side_targets(omap: OccupancyMap, i0, i1, ax: int, start: int, step: int, n: int, stride: int) -> np.ndarray:
    """March every `stride`-th cell of the box face (i0..i1 across axis `ax`) from voxel `start` along `ax` in `step`s, up to n
    voxels: the first voxel that is not free, where it is unknown (an occupied one is the surface, seen)."""
    idx = start + step * np.arange(n)
    idx = idx[(idx >= 0) & (idx < omap.shape[ax])]
    if not len(idx):
        return np.zeros((0, 3))
    sl = [slice(i0[k], i1[k] + 1, stride) for k in range(3)]
    sl[ax] = slice(None)
    col = np.moveaxis(np.take(omap.L[tuple(sl)], idx, axis=ax), ax, -1)       # (cells, cells, along the march)
    k = np.argmax(col >= 0, axis=-1)                                           # first voxel that is not free
    first = np.take_along_axis(col, k[..., None], -1)[..., 0]
    cells = np.argwhere(first == 0)                                            # unknown (an all-free column gives a free 'first')
    ijk = np.insert(np.asarray(i0)[[o for o in range(3) if o != ax]] + cells * stride, ax, idx[k[tuple(cells.T)]], axis=1)
    return omap.lo + (ijk + 0.5) * omap.voxel


def face_targets(omap: OccupancyMap, rep: dict, scene: Scene | None, target: float, stride: int = 2) -> tuple[np.ndarray, float]:
    """What the fitted room still lacks, as points to look at -> (points, m^2 of surface per point).
    Walls: for every `stride`-th cell of each face seen less than `target`, the first voxel from just inside its plane outward that
    is not free, when it is unknown. Before any wall is seen the fit puts the face where the seen floor ends; once rays carve past
    that plane the target moves out with the carved air, so it keeps pointing at the wall the fit lacks. Frontier gain alone
    favours the nearest unknown air (Warehouse flights 2026-10-28: 6.7 m from the far end wall the best view looked sideways; that
    wall ended 4-10 % seen and no tour was flown).
    Obstacles: the same from just outside each side and the top of every fitted box (and of every blob still too small to be one,
    report["blobs"]) inward, so its unseen back is a target too. Without it the walls close first and a crate seen from one side
    is fitted as a thin slab, or not at all (nbv_bench rooms)."""
    if not rep.get("roi_lo"):
        return np.zeros((0, 3)), 0.0
    shape = np.array(omap.shape)
    idx = lambda p: np.clip(((np.asarray(p) - omap.lo) / omap.voxel).astype(int), 0, shape - 1)   # noqa: E731
    i0, i1 = idx(rep["roi_lo"]), idx(rep["roi_hi"])
    out = []
    for ax in range(3):
        for side, name, sgn in ((i0[ax], AXIS_FACE[ax][0], -1), (i1[ax], AXIS_FACE[ax][1], 1)):
            if rep["faces"].get(name, 1.0) < target:                            # one voxel inside the plane, then outward
                out.append(_side_targets(omap, i0, i1, ax, side - sgn, sgn, int(shape[ax]), stride))
    boxes = [(b.center - b.size / 2, b.center + b.size / 2) for b in (scene.no_fly if scene is not None else ())]
    for lo, hi in boxes + [tuple(map(np.asarray, bl)) for bl in rep.get("blobs", ())]:
        b0, b1 = idx(lo), idx(hi)
        for ax, sgn in ((0, -1), (0, 1), (1, -1), (1, 1), (2, 1)):           # not the bottom: it stands on the floor
            side = b1[ax] if sgn > 0 else b0[ax]                                # one voxel outside the face, then inward
            out.append(_side_targets(omap, b0, b1, ax, side + sgn, -sgn, int(b1[ax] - b0[ax]) + 3, stride))
    return (np.vstack(out) if out else np.zeros((0, 3))), (stride * omap.voxel) ** 2


def view_candidates(omap: OccupancyMap, frontier_pts: np.ndarray, current, hint: Scene, K: Intrinsics, ec: dict, mount_pitch: float,
                    blocked: list, rng: np.random.Generator, tried: list = (), targets=None):
    """The reachable viewpoints and their view-cone score: (scored, pos, L, F, scale, y0, reach, T, t_w) or None. `scored` holds
    (utility, position index, yaw, rotation) of every view that sees at least `min_frontier_gain` frontier voxels in its cone, or
    face area worth as much; L is the flight distance to each position through the corridor, `reach` the Reach that measured
    it. `targets` = face_targets(): T, and t_w = the utility of one target point (explore.face_weight per m^2)."""
    from kinetix.geometry import quat_to_R
    v = omap.voxel
    zc = omap.lo[2] + (np.arange(omap.shape[2]) + 0.5) * v
    band = ((zc >= hint.altitude[0]) & (zc <= hint.altitude[1]))[None, None, :]  # the flight envelope: views and every leg stay in it
    safe = omap.safe(ec["clearance_m"], ec["clearance_z_m"]) & band          # where it may stop
    corridor = omap.corridor(ec["clearance_m"], ec["clearance_z_m"]) & band  # what it may fly through
    # one safe voxel (at random) per cell of the candidate grid: a fixed lattice (`safe[::step]`) misses free space thinner than a
    # cell or off its phase, e.g. the 1 m band between crate tops and the ceiling clearance of a low cluttered room
    step = max(1, int(round(ec["cand_step_m"] / v)))
    ijk = np.argwhere(safe)
    ijk = ijk[rng.permutation(len(ijk))]
    ijk = ijk[np.unique(ijk // step, axis=0, return_index=True)[1]]
    if not len(ijk):
        return None
    pos = omap.lo + (ijk + 0.5) * v
    for b in blocked:
        pos = pos[np.linalg.norm(pos - b, axis=1) > 1.0]
    if not len(pos):
        return None
    if len(pos) > ec["max_candidates"]:
        pos = pos[rng.choice(len(pos), ec["max_candidates"], replace=False)]

    # reachable: the corridor leads there (in one straight segment or several legs); the cost is the flight through it
    reach = Reach(omap, corridor, current.p, ec["reach_cell"])
    L = reach.distance(pos)
    ok = np.isfinite(L)
    pos, L = pos[ok], L[ok]
    if not len(pos):
        return None

    F = frontier_pts
    scale = 1.0
    if len(F) > ec["max_frontier_pts"]:
        scale = len(F) / ec["max_frontier_pts"]
        F = F[rng.choice(len(F), ec["max_frontier_pts"], replace=False)]
    T, t_w = targets if targets is not None and ec["face_weight"] > 0 else (np.zeros((0, 3)), 0.0)
    t_w *= ec["face_weight"]
    if len(T) > ec["max_frontier_pts"]:
        t_w *= len(T) / ec["max_frontier_pts"]
        T = T[rng.choice(len(T), ec["max_frontier_pts"], replace=False)]
    y0 = camera_yaw(current)
    near_tried = [np.linalg.norm(pos - tp, axis=1) < 0.5 for tp, _ in tried]
    scored = []                                                        # (utility, position index, yaw, rotation)
    for k in range(ec["n_yaw"]):
        yaw = 2 * np.pi * k / ec["n_yaw"]
        R = quat_to_R(camera_pose([0, 0, 0], yaw, mount_pitch).q)
        done = np.zeros(len(pos), dtype=bool)                          # a view already taken from here at about this heading
        for near, (_, ty) in zip(near_tried, tried):
            if abs(wrap(yaw - ty)) < np.radians(25):
                done |= near
        for i, p in enumerate(pos):
            if done[i]:
                continue
            gain = in_frustum(F, p, R, K, *ec["depth_range_m"]).sum() * scale
            if len(T):
                gain += in_frustum(T, p, R, K, *ec["depth_range_m"]).sum() * t_w
            if gain >= ec["min_frontier_gain"]:
                scored.append((gain - ec["travel_weight"] * L[i] - ec["yaw_weight"] * abs(wrap(yaw - y0)), i, yaw, R))
    return scored, pos, L, F, scale, y0, reach, T, t_w


def next_view(omap: OccupancyMap, frontier_pts: np.ndarray, current, hint: Scene, K: Intrinsics, ec: dict, mount_pitch: float,
              blocked: list, rng: np.random.Generator, tried: list = (), targets=None):
    """(pose, gain) of the best reachable viewpoint, or None. Every candidate is scored on the view cone alone (cheap); the best
    few are re-scored with a line-of-sight test, so a view that stares at a crate does not win on the frontier behind it.
    `tried` = (position, yaw) of earlier views: none is chosen twice. A view the drone cannot fly to in one straight segment
    returns the first leg of the way there instead (facing along it, gain 0): it replans from there. gain = visible frontier
    voxels + the worth of the visible face targets (`targets`, see view_candidates)."""
    c = view_candidates(omap, frontier_pts, current, hint, K, ec, mount_pitch, blocked, rng, tried, targets)
    if c is None:
        return None
    scored, pos, L, F, scale, y0, reach, T, t_w = c
    cur = np.asarray(current.p, dtype=float)
    order = sorted(scored, key=lambda s: -s[0])
    n = ec["los_top"]
    # line of sight in batches down the cone ranking: when the frontier sits behind a wall (a partition) the whole top batch can
    # see nothing of it, and the next batch must get its turn instead of the exploration ending
    for b in range(0, min(len(order), ec["los_max"]), n):
        ranked = []
        for _, i, yaw, R in order[b:b + n]:
            p = pos[i]
            vis = in_frustum(F, p, R, K, *ec["depth_range_m"])
            gain = los_clear(omap, p, F[vis]).sum() * scale
            if len(T):
                tv = in_frustum(T, p, R, K, *ec["depth_range_m"])
                gain += los_clear(omap, p, T[tv]).sum() * t_w if tv.any() else 0.0
            if gain >= ec["min_frontier_gain"]:
                ranked.append((gain - ec["travel_weight"] * L[i] - ec["yaw_weight"] * abs(wrap(yaw - y0)), p, yaw, gain))
        for _, p, yaw, gain in sorted(ranked, key=lambda r: -r[0]):
            if straight_ok(omap, reach.corridor, cur, p[None])[0]:
                return camera_pose(p, yaw, mount_pitch), float(gain)
            via = reach.leg(p)
            if via is not None:
                return camera_pose(via, float(np.arctan2(*(p - via)[1::-1])), mount_pitch), 0.0
    return None


def run_explore(cfg: dict, hint: Scene, rig, log, truth: Scene | None = None,
                frames: list | None = None) -> tuple[Scene | None, dict]:
    """Explore and fit. Returns (scene or None, report); `truth` (a known scene, sim/tests) only adds an error report.
    With `explore.reuse_views` every view is a keyframe too (frames.jsonl, for `kinetix sfm`); `frames`, when given, receives them at their pose in the
    returned scene's frame, so the tour can count what exploration already covered."""
    ec, fc = dict(cfg["explore"]), cfg["explore"]["fit"]
    # never closer to anything than the tour will allow: it boxes obstacles by planner.clearance_m in every direction, and the
    # tour starts where exploration ends
    ec["clearance_m"] = max(ec["clearance_m"], cfg["planner"]["clearance_m"])
    ec["clearance_z_m"] = max(ec["clearance_z_m"], cfg["planner"]["clearance_m"])
    cam = cfg["camera"]
    K = getattr(rig, "K", None) or Intrinsics(**{k: cam[k] for k in ("width", "height", "fx", "fy", "cx", "cy")})
    sb = np.asarray(hint.standby, dtype=float)
    vol = np.asarray(ec["volume_m"], dtype=float)
    lo = np.array([sb[0] - vol[0] / 2, sb[1] - vol[1] / 2, sb[2] - ec["below_m"]])
    omap = OccupancyMap(lo, lo + vol, ec["voxel_m"], tuple(ec["depth_range_m"]), ec["ray_stride_px"])
    pitch = rig.caps.mount_pitch_rad
    rng = np.random.default_rng(0)
    out = log.dir / "explore"
    out.mkdir(exist_ok=True)
    steps = open(out / "steps.jsonl", "w")
    rc = cfg["rig"]                                   # the rig reads its tolerances from this dict at every move
    saved_rc = {k: rc[k] for k in ec["rig"] if k in rc}
    rc.update(ec["rig"])                              # exploration needs a fresh depth image, not a 10 cm / 3 deg hover
    n, blocked, tried, stop = 0, [], [], None
    tracker = DriftTracker(ec["register"]) if ec["register"]["enabled"] else None
    to_map = tracker.to_map if tracker else (lambda pose: pose)        # EKF frame -> the map's drift-corrected frame
    to_ekf = tracker.to_ekf if tracker else (lambda pose: pose)
    clock = getattr(rig, "now", None) or (lambda: rig.t)         # sim seconds (MavrosRig.now) / the dry rig's flight clock
    t_start = clock()
    hist = []                                        # (room lo, room hi, obstacle count) of recent fits
    looked = np.zeros(omap.shape, dtype=bool)        # frontier voxels some view already had in its cone (resolved or not)
    shots = []                                       # (frame, its pose in the map frame)

    def view(pose):
        """Fly to `pose` (map frame), take a depth image, correct the drift estimate with it, integrate it."""
        nonlocal n
        frame = rig.capture_at(to_ekf(pose), idx=IDX0 + n, pool_id=f"explore_{n}")   # the rig flies in the EKF frame
        if frame.depth is None:
            raise RigError("no_image", "the rig returned no depth (explore needs a depth source)")
        if tracker:                                  # slide the frame over the map first: the map must not contain it yet
            tracker.update(omap, frame.depth, frame.K, frame.pose, ec["depth_range_m"][0], ec["depth_range_m"][1], ec["ray_stride_px"])
        mpose = to_map(frame.pose)
        rays = omap.integrate(frame.depth, frame.K, mpose)
        if ec["reuse_views"]:
            log.frame(frame)
            shots.append((frame, mpose))
        # a frontier voxel at the end of the depth range stays frontier however often it is viewed: look once, then move on
        fr = omap.frontier() | omap.frontier(probed=True) if ec["range_frontier_weight"] > 0 else omap.frontier()
        ijk = np.argwhere(fr & ~looked)
        if len(ijk):
            from kinetix.geometry import quat_to_R
            pts = omap.lo + (ijk + 0.5) * omap.voxel
            seen = in_frustum(pts, mpose.p, quat_to_R(mpose.q), frame.K, ec["depth_range_m"][0], ec["look_range_m"])
            seen[seen] = los_clear(omap, mpose.p, pts[seen])
            looked[tuple(ijk[seen].T)] = True
        n += 1
        return frame, rays

    log.info(f"explore: map {omap.shape} voxels of {omap.voxel} m around the take-off point, yaw sweep of {ec['sweep_yaws']}")
    try:
        p0, y0 = to_map(rig.pose()).p, camera_yaw(rig.pose())
        for k in range(ec["sweep_yaws"]):
            view(camera_pose(p0, y0 + 2 * np.pi * k / ec["sweep_yaws"], pitch))
        scene, rep = None, {}
        for step in range(ec["max_steps"]):
            scene, rep = fit_scene(omap, hint, fc)
            rec = {"step": step, "frames": n, "occupied": rep.get("occupied_voxels"), "known": round(rep.get("known_fraction", 0), 4),
                   "faces": rep.get("faces"), "closed": rep.get("closed"), "roi_lo": rep.get("roi_lo"), "roi_hi": rep.get("roi_hi"),
                   "obstacles": rep.get("n_obstacles"), "drift": tracker.t.round(3).tolist() if tracker else None}
            stable = False
            if scene is not None:
                hist.append((np.array(rep["roi_lo"]), np.array(rep["roi_hi"]), rep["n_obstacles"]))
                last = hist[-ec["stable_steps"]:]
                stable = (len(last) == ec["stable_steps"] and all(
                    h[2] == last[0][2] and max(np.abs(h[0] - last[0][0]).max(), np.abs(h[1] - last[0][1]).max()) <= ec["stable_tol_m"]
                    for h in last) and min(rep["faces"].values()) >= ec["min_face"]
                    and all(o <= fc["max_opening_voxels"] for o in rep["openings"].values()))   # a partition stays put too
            # a blob too small to be an obstacle yet is looked at again first (face_targets), not left out of the room
            if scene is not None and (rep["closed"] or stable) and n >= ec["min_views"] and not rep.get("blobs"):
                stop = "closed" if rep["closed"] else "stable"
                steps.write(json.dumps(rec) + "\n")
                break
            if clock() - t_start > ec["max_time_s"]:                  # EKF2 drifts with flight time: bound it
                stop = "time_budget"
                steps.write(json.dumps(rec) + "\n")
                break
            fr = omap.frontier()
            F = omap.centers(fr & ~looked)
            if ec["range_frontier_weight"] > 0:          # the range frontier, at its weight (a random share of it): it leads to walls
                Fr = omap.centers(omap.frontier(probed=True) & ~fr & ~looked)                 # farther than the depth range
                F = np.vstack([F, Fr[rng.random(len(Fr)) < ec["range_frontier_weight"]]])
            rec["frontier"] = int(len(F))
            if len(F) < ec["min_frontier"]:
                stop = "no_frontier"
                steps.write(json.dumps(rec) + "\n")
                break
            targets = face_targets(omap, rep, scene, fc["closed_target"]) if scene is not None else None
            pick = next_view(omap, F, to_map(rig.pose()), hint, K, ec, pitch, blocked, rng, tried, targets)
            if pick is None:
                stop = "no_reachable_view"
                steps.write(json.dumps(rec) + "\n")
                break
            pose, gain = pick
            tried.append((np.asarray(pose.p, dtype=float), camera_yaw(pose)))
            rec.update(target=[round(float(x), 2) for x in pose.p], gain=round(gain), yaw_deg=round(float(np.degrees(camera_yaw(pose)))))
            try:
                _, rays = view(pose)
                rec["rays"] = rays
            except RigError as e:
                rec["error"] = str(e)
                if e.kind in ("aborted", "no_image"):
                    stop = e.kind
                    steps.write(json.dumps(rec) + "\n")
                    break
                blocked.append(np.asarray(pose.p, dtype=float))           # could not get there: do not pick it again
            steps.write(json.dumps(rec) + "\n")
            steps.flush()
            if step % 3 == 0 or rec.get("error"):
                log.info(f"explore {step}: -> {rec.get('target')} sees ~{rec.get('gain')} frontier voxels, known {rec['known']:.2f}, "
                         f"faces {min(rep['faces'].values()) if rep.get('faces') else 0:.2f} (min)")
        else:
            stop = "max_steps"
    finally:
        steps.close()
        rc.update(saved_rc)
    scene, rep = fit_scene(omap, hint, fc)
    rep.update(stop_reason=stop, views=n, flight_s=round(clock() - t_start, 1), drift_offset_m=tracker.t.round(3).tolist() if tracker else None)
    if scene is not None and truth is not None:
        rep["vs_truth"] = compare(scene, truth)                  # the map frame: the take-off frame, drift removed
    if scene is not None and tracker:                            # the drone flies in the EKF frame: hand the tour the room as EKF sees it now
        scene = translated(scene, -tracker.t)
    if frames is not None:                                       # the views in that same frame (map pose - the final drift estimate)
        d = tracker.t if tracker else np.zeros(3)
        frames.extend(replace(f, pose=Pose(mp.p - d, mp.q), depth=None) for f, mp in shots)
    np.savez_compressed(out / "occupancy.npz", L=omap.L, lo=omap.lo, voxel=omap.voxel)
    (out / "report.json").write_text(json.dumps(rep, indent=1))
    if scene is not None:
        (out / "scene.yaml").write_text(yaml.safe_dump(scene_to_dict(scene), sort_keys=False))
    if tracker:
        log.info(f"explore: depth registration put the EKF drift at {np.round(tracker.t, 2).tolist()} m "
                 f"({sum(h['applied'] for h in tracker.history)} corrections in {n} views)")
    log.info(f"explore done ({stop}, {n} views): " + (
        f"room {np.round(scene.roi.size, 1).tolist()} m, {len(scene.no_fly)} obstacle(s)"
        + (f"; vs truth: max wall error {rep['vs_truth']['roi_max_err_m']} m, obstacles "
           f"{rep['vs_truth']['obstacles_found']}/{rep['vs_truth']['obstacles_true']}" if "vs_truth" in rep else "")
        if scene is not None else f"no room could be fitted ({rep.get('why')})"))
    return scene, rep
