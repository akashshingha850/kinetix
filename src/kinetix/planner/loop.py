"""The session loop (architecture.md §5) — the only orchestrator. Never imports rclpy or a rig adapter (dependency rule)."""
from __future__ import annotations

import time

import numpy as np

from kinetix.core.geometry import camera_yaw, wrap
from kinetix.core.types import Candidate, RigError, Scene
from kinetix.planner.constraints import BoxConstraints
from kinetix.planner.sampler import RingSampler
from kinetix.planner.scorers import SCORERS
from kinetix.planner.select import select, utility
from kinetix.planner.stop import Stop

METHODS = ("roi_coverage", "random", "orbit")


def orbit_order(cands: list[Candidate], current, radius: float, height: float) -> list[int]:
    """Fixed-sequence baseline: one ring (closest to radius/height), in azimuth order, starting at the view nearest
    the current camera and turning the shorter way round first."""
    key = [(abs(float(c.pool_id.split("_")[0][1:]) - radius), abs(float(c.pool_id.split("_")[1][1:]) - height)) for c in cands]
    best = min(key)
    ring = [i for i, k in enumerate(key) if k == best]
    ring.sort(key=lambda i: float(cands[i].pool_id.split("_a")[1]))
    start = min(range(len(ring)), key=lambda j: np.linalg.norm(cands[ring[j]].pose.p - current.p))
    return ring[start:] + ring[:start]


def run_session(cfg: dict, scene: Scene, rig, recon, method: str, log, seed: int = 0) -> dict:
    pc = cfg["planner"]
    sampler = RingSampler(pc["rings"]["radii"], pc["rings"]["heights"], pc["rings"]["n_az"])
    cons = BoxConstraints(pc["clearance_m"], pc["roi_clearance_m"])
    stop = Stop(pc["budget"], pc["coverage_target"], pc["min_gain"], pc["patience"])
    scorer = None if method == "orbit" else (SCORERS[method](seed) if method == "random" else SCORERS[method]())
    blocked: set[str] = set()
    taken: set[str] = set()
    n = 0
    stop_reason = None
    rig.start()
    try:
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
                gains = scorer.score(cands, recon)
                d = np.array([np.linalg.norm(c.pose.p - current.p) + 0.3 * abs(wrap(camera_yaw(c.pose) - camera_yaw(current)))
                              for c in cands])
                i = int(np.argmin(np.where(feas, d, np.inf))) if feas.any() else None
            else:
                gains = scorer.score(cands, recon)
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
                if e.kind == "aborted":
                    stop_reason = "aborted"
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
