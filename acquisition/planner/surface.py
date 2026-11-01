"""Model-based two-stage capture, after Liu et al. 2026 (Geo-NeRF, related_work.md [50]), as an object baseline (method
`surface_normal`): a coarse orbit, a coarse surface from that orbit's ZED depth, one viewpoint per surface patch offset along
its normal by a standoff, KD-tree grouping down to the image budget, an ordered visiting sequence, stop-and-go capture.

Differences from the paper, by design: the coarse model is the orbit's ZED depth (oriented points binned into patches), not a
georeferenced NeRF mesh, so it is in the map frame with no extra alignment; the standoff is the scene's (indoor clearance, not
5 cm); the camera is level (mount pitch), so up-facing patches get no viewpoint; the order is nearest neighbour + 2-opt (the
paper's future work), shared with `tour`. The coarse views are flown frames too and count against `planner.budget`.
"""
from __future__ import annotations

import numpy as np

from acquisition.planner.constraints import BoxConstraints
from acquisition.planner.loop import apply_shift
from acquisition.planner.sampler import ring_stack
from acquisition.planner.tour import order_tour
from kinetix.geometry import camera_pose, quat_to_R, segment_hits_box
from kinetix.types import Candidate, RigError, Scene


def backproject(frame, roi, floor_margin: float) -> tuple[np.ndarray, np.ndarray]:
    """Map-frame points of a frame's depth inside the ROI box (minus a floor band), and the camera centre per point."""
    d = frame.depth
    h, w = d.shape
    K = frame.K
    sx, sy = w / K.width, h / K.height                       # depth may be subsampled (dry rig) or another resolution
    v, u = np.mgrid[0:h, 0:w]
    z = d.ravel()
    ok = np.isfinite(z) & (z > 0)
    x = ((u.ravel() + 0.5) / sx - K.cx) / K.fx * z
    y = ((v.ravel() + 0.5) / sy - K.cy) / K.fy * z
    P = np.stack([x, y, z], 1)[ok] @ quat_to_R(frame.pose.q).T + frame.pose.p
    keep = np.all((P >= roi.lo) & (P <= roi.hi), axis=1) & (P[:, 2] > roi.lo[2] + floor_margin)
    return P[keep], np.repeat(frame.pose.p[None], keep.sum(), 0)


def surface_patches(P: np.ndarray, cams: np.ndarray, patch_m: float, min_points: int = 10) -> tuple[np.ndarray, np.ndarray]:
    """Coarse surface as patches: per `patch_m` voxel the centroid and the plane-fit normal (smallest principal axis of its
    points), oriented toward the cameras that saw them. NumPy only: this runs in the kinetix image (no open3d there)."""
    if len(P) < min_points:
        return np.zeros((0, 3)), np.zeros((0, 3))
    _, inv, cnt = np.unique(np.floor(P / patch_m).astype(np.int64), axis=0, return_inverse=True, return_counts=True)
    order = np.argsort(inv.ravel(), kind="stable")
    C, M = [], []
    for idx in np.split(order, np.cumsum(cnt)[:-1]):
        if len(idx) < min_points:                            # a few stray returns: no trustworthy normal
            continue
        X = P[idx]
        c = X.mean(0)
        n = np.linalg.eigh((X - c).T @ (X - c))[1][:, 0]
        if n @ (cams[idx].mean(0) - c) < 0:
            n = -n
        C.append(c)
        M.append(n)
    return np.array(C).reshape(-1, 3), np.array(M).reshape(-1, 3)


def viewpoints(C: np.ndarray, M: np.ndarray, standoff: float, max_normal_z: float, scene: Scene, pitch: float) -> list[Candidate]:
    """One level camera per patch: offset horizontally along the patch normal by `standoff`, at the patch height (clipped to the
    altitude band), looking back at it. Patches facing up or down more than `max_normal_z` get none."""
    out = []
    for c, n in zip(C, M):
        h = n[:2]
        if abs(n[2]) > max_normal_z or np.linalg.norm(h) < 1e-6:
            continue
        h = h / np.linalg.norm(h)
        p = np.array([c[0] + standoff * h[0], c[1] + standoff * h[1], np.clip(c[2], *scene.altitude)])
        yaw = float(np.arctan2(-h[1], -h[0]))
        out.append(Candidate(camera_pose(p, yaw, pitch), pool_id=f"x{p[0]:.2f}_y{p[1]:.2f}_z{p[2]:.2f}_h{np.degrees(yaw) % 360:03.0f}"))
    return out


def group(cands: list[Candidate], n_out: int, pitch: float, max_turn_deg: float = 45.0) -> list[Candidate]:
    """The paper's KD-tree grouping: take an ungrouped viewpoint, merge it with its k nearest ungrouped ones, replace them by
    their centroid looking along their mean heading. Only viewpoints within `max_turn_deg` of the seed's heading join (a group
    spanning two sides of the object puts its centroid off both normals, near the corner); k grows from ceil(n / n_out) until
    at most n_out groups remain."""
    if len(cands) <= n_out:
        return list(cands)
    P = np.array([c.pose.p for c in cands])
    Y = np.array([quat_to_R(c.pose.q)[:2, 2] for c in cands])          # optical axis, ground plane
    Y /= np.maximum(np.linalg.norm(Y, axis=1, keepdims=True), 1e-9)
    D = np.linalg.norm(P[:, None] - P[None], axis=2)                 # a few hundred viewpoints: brute force is fine
    cos_max = np.cos(np.radians(max_turn_deg))
    for k in range(int(np.ceil(len(cands) / max(n_out, 1))), len(cands) + 1):
        done = np.zeros(len(cands), bool)
        groups = []
        for i in range(len(cands)):
            if done[i]:
                continue
            members = [j for j in np.argsort(D[i]) if not done[j] and Y[j] @ Y[i] >= cos_max][:k]
            done[members] = True
            groups.append(members)
        if len(groups) <= n_out:
            break
    out = []
    for members in groups:
        p = P[members].mean(0)
        y = Y[members].sum(0)
        yaw = float(np.arctan2(y[1], y[0]))
        out.append(Candidate(camera_pose(p, yaw, pitch), pool_id=f"x{p[0]:.2f}_y{p[1]:.2f}_z{p[2]:.2f}_h{np.degrees(yaw) % 360:03.0f}"))
    return out


def run_surface(cfg: dict, scene: Scene, rig, recon, log) -> dict:
    if scene.kind == "room":
        raise ValueError("surface_normal is an object method (a coarse orbit around the ROI)")
    pc, sc = cfg["planner"], cfg["planner"]["surface"]
    cons = BoxConstraints(pc["clearance_m"], pc["roi_clearance_m"])
    pitch = rig.caps.mount_pitch_rad
    n, stop_reason, timeouts = 0, None, 0
    frames = []

    def capture(c: Candidate, stage: str) -> bool:
        """Fly, settle, grab; False when the flight must stop."""
        nonlocal n, stop_reason, timeouts
        log.info(f"{stage} {n}: -> {c.pool_id}")
        rec = {"step": n, "stage": stage, "chosen": c.pool_id, "coverage": round(recon.coverage, 4), "error": None}
        try:
            frame = rig.capture_at(c.pose, idx=n, pool_id=c.pool_id)
        except RigError as e:
            rec["error"] = str(e)
            log.step(rec)
            log.info(f"{stage} {n}: {e}")
            timeouts += 1
            if e.kind in ("aborted", "no_image") or timeouts >= pc["max_consecutive_timeouts"]:
                stop_reason = e.kind if e.kind in ("aborted", "no_image") else "stalled"
                return False
            return True
        timeouts = 0
        recon.add(frame)
        frames.append(frame)
        rec["coverage"] = round(recon.coverage, 4)
        log.step(rec)
        log.frame(frame)
        log.info(f"frame {n}: coverage {recon.coverage:.3f}, pose err {np.linalg.norm(frame.pose.p - c.pose.p):.3f} m")
        n += 1
        return True

    def static_ok(cs: list[Candidate]) -> list[Candidate]:
        """Workspace, altitude band, standoff, outside the keep-out boxes: the leg to it is checked when it is flown."""
        return [c for c in cs if cons.feasible([c], c.pose, scene)[0]]

    def blocked(a, b) -> bool:
        return any(segment_hits_box(a, b, box) for box in shells)

    def route(a, b) -> list[Candidate] | None:
        """Transit poses from a to b around the object when the straight leg is blocked: out to the coarse ring at a's azimuth,
        round it in arcs of at most 30 deg (chords that clear the ROI shell), in to b. [] = fly straight, None = no way."""
        if not blocked(a, b):
            return []
        c = scene.roi.center
        az0 = np.arctan2(a[1] - c[1], a[0] - c[0])
        dz = (np.arctan2(b[1] - c[1], b[0] - c[0]) - az0 + np.pi) % (2 * np.pi) - np.pi
        k = int(np.ceil(abs(dz) / np.radians(30)))
        hops = []
        for i in range(k + 1):
            az = az0 + dz * i / k
            p = [c[0] + sc["coarse_radius"] * np.cos(az), c[1] + sc["coarse_radius"] * np.sin(az), a[2]]
            hops.append(Candidate(camera_pose(p, az + np.pi, pitch), pool_id=f"transit_a{np.degrees(az) % 360:03.0f}"))
        pts = [a] + [h.pose.p for h in hops] + [b]
        if len(static_ok(hops)) < len(hops) or any(blocked(p, q) for p, q in zip(pts, pts[1:])):
            return None
        return hops

    def fly(seq: list[Candidate], stage: str) -> None:
        """Visit `seq` in order, the next one reachable straight or around the object (route) from here."""
        nonlocal stop_reason
        left = list(seq)
        while left and stop_reason is None and n < pc["budget"]:
            here = rig.pose().p
            j, hops = next(((i, r) for i, c in enumerate(left) if (r := route(here, c.pose.p)) is not None), (None, None))
            if j is None:
                log.info(f"{stage}: {len(left)} views left, none reachable: stopping")
                stop_reason = "no_feasible_view"
                return
            try:
                for h in hops:
                    rig.move_to(h.pose)
            except RigError as e:
                log.info(f"{stage}: transit failed: {e}")
                stop_reason = e.kind if e.kind in ("aborted", "no_image") else "stalled"
                return
            if hops:
                log.info(f"{stage} {n}: round the object via {len(hops)} transit poses")
            if not capture(left.pop(j), stage):
                return

    try:
        rig.start()
        scene = apply_shift(rig, scene, recon, log)
        shells = [b.inflated(pc["clearance_m"]) for b in scene.no_fly] + [scene.roi.inflated(pc["roi_clearance_m"])]
        # stage 1: the coarse orbit (one ring, from the view nearest the drone, round the ring)
        ring = static_ok(ring_stack(scene, rig.caps, [sc["coarse_radius"]], [sc["coarse_height"]], sc["coarse_views"]))
        if ring:
            s = min(range(len(ring)), key=lambda j: np.linalg.norm(ring[j].pose.p - rig.pose().p))
            fly(ring[s:] + ring[:s], "coarse")
        if stop_reason is None:
            # the coarse surface, then one viewpoint per patch
            parts = [backproject(f, scene.roi, sc["floor_margin_m"]) for f in frames if f.depth is not None]
            if not parts:
                raise RuntimeError("surface_normal: no depth on the coarse frames (rig.save_depth)")
            P, cams = np.concatenate([p for p, _ in parts]), np.concatenate([c for _, c in parts])
            C, M = surface_patches(P, cams, sc["patch_m"])
            cands = viewpoints(C, M, sc["standoff_m"], sc["max_normal_z"], scene, pitch)
            ok = static_ok(cands)
            fine = static_ok(group(ok, max(pc["budget"] - n, 0), pitch))
            order = order_tour(rig.pose(), [c.pose for c in fine], sc["yaw_cost_m"], blocked=blocked)
            log.info(f"coarse model: {len(P)} points, {len(C)} patches -> {len(cands)} viewpoints, {len(ok)} feasible "
                     f"-> {len(fine)} after grouping (standoff {sc['standoff_m']} m)")
            # stage 2: stop-and-go along the ordered viewpoints
            fly([fine[j] for j in order], "fine")
            stop_reason = stop_reason or ("budget" if n >= pc["budget"] else "sequence_done")
    finally:
        rig.finish()
    return {"stop_reason": stop_reason, "coverage": round(recon.coverage, 4), "seen": round(recon.seen, 4)}
