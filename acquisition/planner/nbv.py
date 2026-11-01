"""Stepwise next-best-view (architecture.md §4, §7): score the feasible candidates, fly to the best, capture, repeat until
the stop rule fires. Methods roi_coverage, random, fvs, volumetric_ig (planner/scorers) and the fixed sequences orbit (one ring)
and orbit_rings (every ring, golden-angle order: each prefix is an evenly spread N-view capture, paper/benchmark.md §1).
`seed`: 0 starts at the view nearest the rig (the flight default), k > 0 at a seeded random feasible view (bench seeds)."""
from __future__ import annotations

import time

import numpy as np

from acquisition.planner.constraints import BoxConstraints
from acquisition.planner.loop import apply_shift
from acquisition.planner.sampler import GridSampler, RingSampler
from acquisition.planner.scorers import make_scorer
from kinetix.geometry import camera_yaw, wrap
from kinetix.types import Candidate, Pose, RigError, Scene


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


def spread_order(cands: list[Candidate], current, seed: int = 0) -> list[int]:
    """Fixed-sequence baseline over every ring: view k is on ring k mod n_rings (rings by radius, then height) at azimuth
    a0 + k x 137.5 deg (the golden angle), snapped to the ring's nearest untaken view; a0 faces the current camera (seed 0) or
    is seeded. Every prefix is spread evenly over azimuth and height."""
    keys = [tuple(c.pool_id.split("_a")[0].split("_")) for c in cands]
    rings = sorted(set(keys), key=lambda k: (float(k[0][1:]), float(k[1][1:])))
    by_ring = {r: [i for i, k in enumerate(keys) if k == r] for r in rings}
    az = np.array([float(c.pool_id.split("_a")[1]) for c in cands])
    if seed:
        a0 = np.random.default_rng(seed).uniform(0, 360)
    else:
        a0 = az[min(range(len(cands)), key=lambda i: np.linalg.norm(cands[i].pose.p - current.p))]
    order, taken = [], set()
    for k in range(len(cands)):
        ring = by_ring[rings[k % len(rings)]]
        free = [i for i in ring if i not in taken]
        a = (a0 + 137.50776 * k) % 360
        i = min(free, key=lambda i: abs((az[i] - a + 180) % 360 - 180))
        order.append(i)
        taken.add(i)
    return order


def depth_surface(recon, frames, scene: Scene, cfg: dict, log):
    """depth_coverage: swap the ROI-box proxy for the surface the first views' depth shows (patches of coverage.spacing_m with a
    plane-fit normal, planner/surface.py), plus the box points nothing has seen yet (the far side is still unknown, so it stays
    a box and keeps drawing views there). The frames so far are re-added, so the stop rule and the gains carry on."""
    from acquisition.planner.surface import backproject, surface_patches
    from kinetix.coverage import CoverageState
    ds = cfg["planner"]["depth_surface"]
    parts = [backproject(f, scene.roi, ds["floor_margin_m"]) for f in frames if f.depth is not None]
    if not parts:
        log.info("depth_coverage: no depth on the first views (rig.save_depth): keeping the box proxy")
        return recon
    P = np.concatenate([p for p, _ in parts])
    C, M = surface_patches(P, np.concatenate([c for _, c in parts]), cfg["coverage"]["spacing_m"], ds["min_points"])
    unseen = recon.count == 0
    surf = (np.concatenate([C, recon.P[unseen]]), np.concatenate([M, recon.N[unseen]]))
    new = CoverageState.for_scene(scene, recon.K, cfg["coverage"], surf)
    for f in frames:
        new.add(f)
    log.info(f"depth_coverage: surface from {len(frames)} views' depth: {len(C)} patches + {int(unseen.sum())} unseen box points, "
             f"coverage {recon.coverage:.3f} -> {new.coverage:.3f}")
    return new


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
        if method in ("orbit", "orbit_rings"):
            raise ValueError(f"{method} is an object baseline (rings around the ROI); a room scene has no orbit")
        sampler = GridSampler(pc["grid"]["spacing_m"], pc["grid"]["heights"], pc["grid"]["n_yaw"], pc["grid"].get("layer_m"))
    else:
        sampler = RingSampler(pc["rings"]["radii"], pc["rings"]["heights"], pc["rings"]["n_az"])
    cons = BoxConstraints(pc["clearance_m"], pc["roi_clearance_m"])
    stop = Stop(pc["budget"], pc["coverage_target"], pc["min_gain"], pc["patience"])
    fixed = method in ("orbit", "orbit_rings")
    scorer = None if fixed else make_scorer(method, seed, scene, cfg)
    rng = np.random.default_rng(seed)
    frames = []
    blocked: set[str] = set()
    taken: set[str] = set()
    n = 0
    timeouts = 0                                      # consecutive failed moves; a rig that can no longer settle must not loop forever
    stop_reason = None
    try:
        rig.start()                                   # inside the try: a failed start still runs finish() (lands, stops the bag)
        scene = apply_shift(rig, scene, recon, log)
        cands = sampler.sample(scene, recon, rig.caps)
        if method == "orbit":
            queue = orbit_order(cands, rig.pose(), pc["orbit"]["radius"], pc["orbit"]["height"])
            if seed:
                k = int(rng.integers(len(queue)))
                queue = queue[k:] + queue[:k]
        elif method == "orbit_rings":
            queue = spread_order(cands, rig.pose(), seed)
        last_gain = None
        while not stop.done(recon, n, last_gain if n > pc["seed_views"] and not fixed else None):
            t0 = time.time()
            current = rig.pose()
            feas = cons.feasible(cands, current, scene, blocked | taken)
            if fixed:
                i = next((j for j in queue if cands[j].pool_id not in taken | blocked and feas[j]), None)
                gains = np.array([recon.gain(c.pose) for c in cands])     # logged only; the order is fixed
            elif n < pc["seed_views"]:                                     # seed: the nearest feasible view
                gains = score_feasible(scorer, cands, feas, recon)
                d = np.array([np.linalg.norm(c.pose.p - current.p) + 0.3 * abs(wrap(camera_yaw(c.pose) - camera_yaw(current)))
                              for c in cands])
                if seed and feas.any():                                    # bench seeds: a seeded random first view
                    d = np.where(feas, rng.uniform(size=len(cands)), np.inf)
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
                stop_reason = "sequence_done" if fixed else "no_feasible_view"
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
            if hasattr(scorer, "add"):
                scorer.add(frame)
            frames.append(frame)
            if method == "depth_coverage" and len(frames) == pc["depth_surface"]["after_views"]:
                recon = depth_surface(recon, frames, scene, cfg, log)
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
