"""The session loop (architecture.md §5) — the only orchestrator. Never imports rclpy or a rig adapter (dependency rule)."""
from __future__ import annotations

import time

import numpy as np

from kinetix.core.geometry import camera_yaw, wrap
from kinetix.core.types import Candidate, RigError, Scene, shifted
from kinetix.planner.constraints import BoxConstraints
from kinetix.planner.sampler import GridSampler, RingSampler
from kinetix.planner.scorers import SCORERS
from kinetix.planner.select import select, utility
from kinetix.planner.stop import Stop

METHODS = ("roi_coverage", "random", "orbit", "tour")


def orbit_order(cands: list[Candidate], current, radius: float, height: float) -> list[int]:
    """Fixed-sequence baseline: one ring (closest to radius/height), in azimuth order, starting at the view nearest
    the current camera and turning the shorter way round first."""
    key = [(abs(float(c.pool_id.split("_")[0][1:]) - radius), abs(float(c.pool_id.split("_")[1][1:]) - height)) for c in cands]
    best = min(key)
    ring = [i for i, k in enumerate(key) if k == best]
    ring.sort(key=lambda i: float(cands[i].pool_id.split("_a")[1]))
    start = min(range(len(ring)), key=lambda j: np.linalg.norm(cands[ring[j]].pose.p - current.p))
    return ring[start:] + ring[:start]


def score_feasible(scorer, cands: list[Candidate], feas: np.ndarray, recon) -> np.ndarray:
    """Scorer gains for the feasible candidates, 0 elsewhere (a room grid has thousands of candidates per step)."""
    gains = np.zeros(len(cands))
    idx = np.flatnonzero(feas)
    if len(idx):
        gains[idx] = scorer.score([cands[j] for j in idx], recon)
    return gains


def _apply_shift(rig, scene: Scene, recon, log) -> Scene:
    shift = np.asarray(getattr(rig, "map_shift", np.zeros(3)), dtype=float)
    if np.linalg.norm(shift) > 0.01:                  # the rig found the map origin elsewhere: move the scene with it
        scene = shifted(scene, shift)
        if hasattr(recon, "shift"):
            recon.shift(shift)
        log.info(f"scene shifted by {np.round(shift, 3).tolist()} m (measured map origin vs the scene's world_to_map)")
    return scene


def run_tour(cfg: dict, scene: Scene, rig, recon, log) -> dict:
    """Plan the whole capture, then fly it without stopping and keep keyframes (ADR-K12, planner/tour.py)."""
    import copy

    from kinetix.planner.tour import flight_time, order_tour, sample_path, select_viewpoints, tour_length
    from kinetix.recon.coverage import CoverageState
    pc, tc = cfg["planner"], cfg["planner"]["tour"]
    stop_reason = None
    try:
        rig.start()                                   # inside the try: a failed start still runs finish()
        scene = _apply_shift(rig, scene, recon, log)
        t0 = time.time()
        g = pc["grid"]
        sampler = GridSampler(g["spacing_m"], g["heights"], g["n_yaw"])
        cands = sampler.sample(scene, recon, rig.caps)
        start = rig.pose()
        feas = BoxConstraints(pc["clearance_m"], pc["roi_clearance_m"]).feasible(cands, start, scene)
        cc = dict(cfg["coverage"], min_shared=0, min_overlap=0.0)   # continuous capture links the views
        plan = CoverageState.for_scene(scene, recon.K, cc)
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


def run_session(cfg: dict, scene: Scene, rig, recon, method: str, log, seed: int = 0) -> dict:
    pc = cfg["planner"]
    if method == "tour":
        if scene.kind != "room":
            raise ValueError("tour is a room method (grid viewpoints + TSP + continuous capture)")
        return run_tour(cfg, scene, rig, recon, log)
    if scene.kind == "room":
        if method == "orbit":
            raise ValueError("orbit is an object baseline (rings around the ROI); a room scene has no orbit")
        sampler = GridSampler(pc["grid"]["spacing_m"], pc["grid"]["heights"], pc["grid"]["n_yaw"])
    else:
        sampler = RingSampler(pc["rings"]["radii"], pc["rings"]["heights"], pc["rings"]["n_az"])
    cons = BoxConstraints(pc["clearance_m"], pc["roi_clearance_m"])
    stop = Stop(pc["budget"], pc["coverage_target"], pc["min_gain"], pc["patience"])
    scorer = None if method == "orbit" else (SCORERS[method](seed) if method == "random" else SCORERS[method]())
    blocked: set[str] = set()
    taken: set[str] = set()
    n = 0
    stop_reason = None
    try:
        rig.start()                                   # inside the try: a failed start still runs finish() (lands, stops the bag)
        scene = _apply_shift(rig, scene, recon, log)
        cands = sampler.sample(scene, recon, rig.caps)
        queue = orbit_order(cands, rig.pose(), pc["orbit"]["radius"], pc["orbit"]["height"]) if method == "orbit" else None
        last_gain = None
        while not stop.done(recon, n, last_gain if n > pc["seed_views"] and method != "orbit" else None):
            t0 = time.time()
            current = rig.pose()
            feas = cons.feasible(cands, current, scene, blocked | taken)
            if method == "orbit":
                i = next((j for j in queue if cands[j].pool_id not in taken | blocked and feas[j]), None)
                gains = np.array([recon.gain(c.pose) for c in cands])     # logged only; the order is fixed
            elif n < pc["seed_views"]:                                     # seed: the nearest feasible view
                gains = score_feasible(scorer, cands, feas, recon)
                d = np.array([np.linalg.norm(c.pose.p - current.p) + 0.3 * abs(wrap(camera_yaw(c.pose) - camera_yaw(current)))
                              for c in cands])
                i = int(np.argmin(np.where(feas, d, np.inf))) if feas.any() else None
            else:
                gains = score_feasible(scorer, cands, feas, recon)
                i = select(cands, gains, feas, current, pc["travel_weight"], pc["yaw_weight"])
            t_score = time.time() - t0
            u = utility(cands, gains, current, pc["travel_weight"], pc["yaw_weight"])
            top = [(cands[j].pool_id, round(float(gains[j]), 4)) for j in np.argsort(-np.where(feas, u, -np.inf))[:5] if feas[j]]
            rec = {"step": n, "n_cands": len(cands), "n_feasible": int(feas.sum()), "chosen": None, "gain": None,
                   "top5": top, "t_score_s": round(t_score, 3), "coverage": round(recon.coverage, 4), "error": None}
            if i is None:
                stop_reason = "sequence_done" if method == "orbit" else "no_feasible_view"
                log.step(rec)
                break
            c = cands[i]
            rec.update(chosen=c.pool_id, gain=round(float(gains[i]), 4))
            log.info(f"step {n}: -> {c.pool_id} gain {gains[i]:.3f} ({int(feas.sum())}/{len(cands)} feasible)")
            try:
                frame = rig.capture_at(c.pose, idx=n, pool_id=c.pool_id)
            except RigError as e:
                rec["error"] = str(e)
                log.step(rec)
                log.info(f"step {n}: {e}")
                if e.kind in ("aborted", "no_image"):   # no image for image_timeout_s = the camera (and the ZED VIO) is
                    stop_reason = e.kind                 # gone: land now, while EKF2 still holds position on flow + range
                    break
                blocked.add(c.pool_id)
                continue
            seen = recon.add(frame)
            taken.add(c.pool_id)
            last_gain = float(gains[i])
            rec["coverage"] = round(recon.coverage, 4)
            log.step(rec)
            log.frame(frame)
            log.info(f"frame {n}: {seen} surface pts in view, coverage {recon.coverage:.3f} (seen {recon.seen:.3f}), "
                     f"pose err {np.linalg.norm(frame.pose.p - c.pose.p):.3f} m")
            n += 1
        stop_reason = stop_reason or stop.reason
    finally:
        rig.finish()
    return {"stop_reason": stop_reason, "coverage": round(recon.coverage, 4), "seen": round(recon.seen, 4)}
