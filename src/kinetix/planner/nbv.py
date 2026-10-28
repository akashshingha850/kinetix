"""Stepwise next-best-view (architecture.md §4, §7): score the feasible candidates, fly to the best, capture, repeat until
the stop rule fires. Methods roi_coverage, random (planner/scorers) and orbit (fixed-sequence baseline)."""
from __future__ import annotations

import time

import numpy as np

from kinetix.core.geometry import camera_yaw, wrap
from kinetix.core.types import Candidate, Pose, RigError, Scene
from kinetix.planner.constraints import BoxConstraints
from kinetix.planner.loop import apply_shift
from kinetix.planner.sampler import GridSampler, RingSampler
from kinetix.planner.scorers import SCORERS


def score_feasible(scorer, cands: list[Candidate], feas: np.ndarray, recon) -> np.ndarray:
    """Scorer gains for the feasible candidates, 0 elsewhere (a room grid has thousands of candidates per step)."""
    gains = np.zeros(len(cands))
    idx = np.flatnonzero(feas)
    if len(idx):
        gains[idx] = scorer.score([cands[j] for j in idx], recon)
    return gains


# ── selection: argmax of gain − λ·travel − μ·|Δyaw| over the feasible candidates ──────────────────────────────────────
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


def orbit_order(cands: list[Candidate], current, radius: float, height: float) -> list[int]:
    """Fixed-sequence baseline: one ring (closest to radius/height), in azimuth order, starting at the view nearest
    the current camera and turning the shorter way round first."""
    key = [(abs(float(c.pool_id.split("_")[0][1:]) - radius), abs(float(c.pool_id.split("_")[1][1:]) - height)) for c in cands]
    best = min(key)
    ring = [i for i, k in enumerate(key) if k == best]
    ring.sort(key=lambda i: float(cands[i].pool_id.split("_a")[1]))
    start = min(range(len(ring)), key=lambda j: np.linalg.norm(cands[ring[j]].pose.p - current.p))
    return ring[start:] + ring[:start]


# ── stop rule: image budget, coverage target, or gain plateau ─────────────────────────────────────────────────────────
class Stop:
    def __init__(self, budget: int, coverage_target: float = 0.95, min_gain: float = 0.005, patience: int = 2):
        self.budget, self.coverage_target, self.min_gain, self.patience = budget, coverage_target, min_gain, patience
        self._low = 0
        self.reason = None

    def done(self, recon, n_frames: int, last_gain: float | None = None) -> bool:
        if n_frames >= self.budget:
            self.reason = "budget"
        elif recon is not None and recon.coverage >= self.coverage_target:
            self.reason = "coverage_target"
        elif last_gain is not None:
            self._low = self._low + 1 if last_gain < self.min_gain else 0
            if self._low >= self.patience:
                self.reason = "gain_plateau"
        return self.reason is not None


def run_nbv(cfg: dict, scene: Scene, rig, recon, method: str, log, seed: int = 0) -> dict:
    pc = cfg["planner"]
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
    timeouts = 0                                      # consecutive failed moves; a rig that can no longer settle must not loop forever
    stop_reason = None
    try:
        rig.start()                                   # inside the try: a failed start still runs finish() (lands, stops the bag)
        scene = apply_shift(rig, scene, recon, log)
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
                timeouts += 1
                if timeouts >= pc["max_consecutive_timeouts"]:
                    log.info(f"{timeouts} moves in a row timed out: stopping")
                    stop_reason = "stalled"
                    break
                continue
            timeouts = 0
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
