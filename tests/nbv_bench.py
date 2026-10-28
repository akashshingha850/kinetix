"""NBV benchmark (ADR-K13): is the exploration's next-best-view accurate, and cheap enough?

    PYTHONPATH=src python3 tests/nbv_bench.py [prediction] [e2e] [sweep] [--quick] [--out runs/nbv_bench]

Dry rig, synthetic depth cast against a true room the planner never sees, no EKF drift (the drift has its own tests: this
measures the view choice alone).

  prediction  at every exploration step the candidate views are also flown in simulation (depth cast against the true room,
              integrated into a copy of the map) and what each really achieved is compared with what the NBV predicted:
              rank correlation, the chosen view's regret against the best candidate, how often it is in the true top 3.
  e2e         the whole exploration with the NBV and with baselines (a random reachable view, the nearest one, the cone score
              without the line-of-sight re-score, an oracle that picks by the true outcome): views, metres flown, flight time,
              planning compute, room error.
  sweep       one NBV parameter at a time (candidate grid, headings, frontier sample, line-of-sight top, travel weight):
              what it buys in views and metres, what it costs in compute.
"""
from __future__ import annotations

import argparse
import json
import sys
import tempfile
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from kinetix.core.config import ROOT, load_config, load_scene  # noqa: E402
from kinetix.core.geometry import camera_pose, camera_yaw, wrap  # noqa: E402
from kinetix.core.runlog import RunLog  # noqa: E402
from kinetix.core.simdepth import render_depth  # noqa: E402
from kinetix.core.types import Box, Scene  # noqa: E402
from kinetix.planner import explore as X  # noqa: E402
from kinetix.recon.depthmap import OccupancyMap  # noqa: E402
from kinetix.rig.base import load_rig  # noqa: E402

B = lambda c, s: Box(np.array(c, dtype=float), np.array(s, dtype=float))   # noqa: E731


def scenes():
    """(scene, volume_m, start positions). Box rooms: the depth caster knows boxes only."""
    room12 = Scene("room12_2crates", B([0, 0, 2], [12, 9, 4]), B([0, 0, 2], [9, 6, 2]),
                   (B([2, 1.5, 0.6], [1.2, 1.2, 1.2]), B([-2, -1, 0.9], [1, 1, 1.8])), (0.0, np.inf), (1.0, 3.0), kind="room")
    clutter = Scene("clutter10_6crates", B([0, 0, 1.75], [10, 8, 3.5]), B([0, 0, 1.75], [7, 5, 2]),
                    (B([-3, 2, 0.5], [1, 1.5, 1]), B([-1, -2.2, 0.8], [1.2, 1, 1.6]), B([1.5, 1.8, 1.0], [0.8, 0.8, 2.0]),
                     B([3, -1.5, 0.4], [1.5, 1, 0.8]), B([0.5, 0.2, 0.6], [0.6, 0.6, 1.2]), B([-3.5, -2.5, 0.3], [1, 1, 0.6])),
                    (0.0, np.inf), (1.0, 2.6), kind="room")
    # a floor-to-ceiling partition from the south wall to 2 m short of the north one: the far half is only seen around its end
    split = Scene("split14_partition", B([0, 0, 2], [14, 10, 4]), B([0, 0, 2], [11, 7, 2]),
                  (B([0, -1, 2], [0.4, 8, 4]), B([3.5, 2.5, 0.5], [1, 1, 1])), (0.0, np.inf), (1.0, 3.0), kind="room")
    wh = load_scene(ROOT / "config" / "scenes" / "warehouse_room.yaml")
    return [
        (room12, [30, 30, 8], [[-4, -2.8, 1.5], [3.5, -2.5, 1.5]]),
        (clutter, [24, 24, 7], [[-2.5, 0, 1.4], [3, 2.5, 1.4]]),
        (split, [30, 30, 8], [[-4.5, -3, 1.5], [-2, 3, 1.5]]),
        (wh, [40, 40, 12], [[0, 0, 1.2], [4, 6, 1.2]]),            # the room must fit the +-20 m map volume
    ]


# --- what a view really achieves ------------------------------------------------------------------------------------
def map_copy(m: OccupancyMap) -> OccupancyMap:
    c = OccupancyMap.__new__(OccupancyMap)
    c.__dict__.update(m.__dict__)
    c.L, c.probed = m.L.copy(), m.probed.copy()
    return c


def is_frontier(m: OccupancyMap, ijk: np.ndarray) -> np.ndarray:
    """OccupancyMap.frontier() at the voxels ijk only."""
    sh = np.array(m.shape)
    near = np.zeros(len(ijk), dtype=bool)
    for ax in range(3):
        for s in (-1, 1):
            n = ijk.copy()
            n[:, ax] += s
            ok = (n[:, ax] >= 0) & (n[:, ax] < sh[ax])
            n = tuple(np.where(ok[:, None], n, 0).T)
            near |= ok & (m.L[n] == 0) & ~m.probed[n]
    return (m.L[tuple(ijk.T)] < 0) & near


def outcome(m: OccupancyMap, F: np.ndarray, scale: float, pose, truth: Scene, K, depth_max: float) -> dict:
    """Fly the view in simulation: frontier voxels of F it resolves (scaled like the prediction), voxels it makes known, wall /
    obstacle voxels it adds."""
    c = map_copy(m)
    c.integrate(render_depth(truth.roi, truth.no_fly, pose, K, max_range=depth_max), K, pose)
    ijk, _ = m.index(F)
    known0, occ0 = m.L != 0, m.L > 0
    return {"resolved": float((~is_frontier(c, ijk)).sum() * scale), "new_known": int(((c.L != 0) & ~known0).sum()),
            "new_occ": int(((c.L > 0) & ~occ0).sum())}


def ranks(a):
    a = np.asarray(a, dtype=float)
    r = np.empty(len(a))
    r[np.argsort(a, kind="stable")] = np.arange(len(a))
    for v in np.unique(a):                                       # average the ties
        r[a == v] = r[a == v].mean()
    return r


def spearman(a, b):
    if len(a) < 3 or np.ptp(a) == 0 or np.ptp(b) == 0:
        return float("nan")
    return float(np.corrcoef(ranks(a), ranks(b))[0, 1])


# --- strategies: drop-in replacements of explore.next_view --------------------------------------------------------------
def _cands(args, n_top=24, n_rand=16, rng=None):
    """The candidates the NBV scored on the cone, best n_top plus n_rand others, each with its line-of-sight gain and utility."""
    omap, F0, current, hint, K, ec, pitch, blocked, rng0, tried = args[:10]
    c = X.view_candidates(*args)
    if c is None or not c[0]:
        return None
    scored, pos, L, F, scale, y0 = c[:6]
    order = sorted(range(len(scored)), key=lambda k: -scored[k][0])
    pick = order[:n_top]
    rest = order[n_top:]
    if rest and n_rand:
        pick += list((rng or np.random.default_rng(0)).choice(rest, min(n_rand, len(rest)), replace=False))
    out = []
    for k in pick:
        u0, i, yaw, R = scored[k]
        p = pos[i]
        vis = X.in_frustum(F, p, R, K, *ec["depth_range_m"])
        gain = float(X.los_clear(omap, p, F[vis]).sum() * scale)
        cost = ec["travel_weight"] * L[i] + ec["yaw_weight"] * abs(wrap(yaw - y0))
        out.append({"cone_rank": order.index(k), "u_cone": float(u0), "gain": gain, "cost": float(cost), "u": gain - cost,
                    "pose": camera_pose(p, yaw, pitch), "travel": float(L[i]), "i": i, "yaw": yaw})
    return out, F, scale, len(scored), c


NEXT_VIEW = X.next_view


def go(c, i, yaw, args, gain):
    """Fly to candidate i like the NBV does: straight when the corridor allows, else the first leg of the way (gain 0)."""
    pos, reach = c[1], c[6]
    p, cur = pos[i], np.asarray(args[2].p, dtype=float)
    if X.straight_ok(args[0], reach.corridor, cur, p[None])[0]:
        return camera_pose(p, yaw, args[6]), float(gain)
    via = reach.leg(p)
    if via is None:
        return None
    return camera_pose(via, float(np.arctan2(*(p - via)[1::-1])), args[6]), 0.0                                          # the planner's own, before explore_once patches it


def nbv(*args):
    return NEXT_VIEW(*args)


def nbv_cone(*args):
    """Ablation: the cone score alone, no line-of-sight re-score."""
    c = X.view_candidates(*args)
    if c is None or not c[0]:
        return None
    u0, i, yaw, R = max(c[0], key=lambda s: s[0])
    return go(c, i, yaw, args, u0)


def make_random(seed):
    rng = np.random.default_rng(seed)

    def random_view(*args):
        c = X.view_candidates(*args)
        if c is None or not c[0]:
            return None
        u0, i, yaw, R = c[0][rng.integers(len(c[0]))]
        return go(c, i, yaw, args, u0)
    return random_view


def nearest(*args):
    """Closest reachable view that sees enough frontier (heading: the best cone score there)."""
    c = X.view_candidates(*args)
    if c is None or not c[0]:
        return None
    L = c[2]
    u0, i, yaw, R = min(c[0], key=lambda s: (round(L[s[1]], 1), -s[0]))
    return go(c, i, yaw, args, u0)


def make_oracle(truth, depth_max):
    """Upper bound: flies its 40 candidates in simulation, takes the best true utility (resolved frontier - travel - turn)."""
    def oracle(*args):
        c = _cands(args)
        if c is None:
            return None
        cands, F, scale = c[:3]
        for d in cands:
            d["true"] = outcome(args[0], F, scale, d["pose"], truth, args[4], depth_max)
        for d in sorted(cands, key=lambda d: -(d["true"]["resolved"] - d["cost"])):
            pick = go(c[4], d["i"], d["yaw"], args, d["gain"])
            if pick is not None:
                return pick
        return None
    return oracle


# --- one exploration ------------------------------------------------------------------------------------------------------
def explore_once(scene, vol, start, strategy="nbv", ec=None, analyse=False, seed=0):
    cfg = load_config([], [], kind="room")
    cfg["explore"].update({"enabled": True, "volume_m": list(vol), **(ec or {})})
    truth = replace(scene, standby=np.asarray(start, dtype=float))
    hint = X.hint_scene(truth, cfg["explore"]["volume_m"])
    depth_max = float(cfg["rig"].get("dry_depth_max_m", 15.0))
    fn = {"nbv": nbv, "nbv_cone": nbv_cone, "nearest": nearest, "random": make_random(seed),
          "oracle": make_oracle(truth, depth_max)}[strategy]
    plan_s, steps, legs = [], [], []

    def wrapped(*args):
        t = time.perf_counter()
        pick = fn(*args)
        plan_s.append(time.perf_counter() - t)
        legs.append(pick is not None and pick[1] == 0)
        if analyse and pick is not None and pick[1] > 0:          # gain 0: a leg on the way, not a view choice
            c = _cands(args)
            if c is not None:
                cands, F, scale, n_scored = c[:4]
                for d in cands:
                    d["true"] = outcome(args[0], F, scale, d["pose"], truth, args[4], depth_max)
                chosen = outcome(args[0], F, scale, pick[0], truth, args[4], depth_max)
                steps.append(step_stats(cands, pick, chosen, args[5], n_scored, len(F)))
        return pick

    sent = []
    with tempfile.TemporaryDirectory() as tmp:
        log = RunLog(tmp, truth.name, "tour", "dry", 0, cfg, uses_gt=False)
        log.info = lambda *a, **k: None
        rig = load_rig("dry", cfg, hint, log.dir, truth)
        cap = rig.capture_at
        rig.capture_at = lambda target, idx, pool_id=None: (sent.append(target), cap(target, idx, pool_id))[1]
        saved, X.next_view = X.next_view, wrapped
        t0 = time.perf_counter()
        try:
            scene_fit, rep = X.run_explore(cfg, hint, rig, log, truth)
        finally:
            X.next_view = saved
        wall = time.perf_counter() - t0
        curve = [json.loads(s) for s in (log.dir / "explore" / "steps.jsonl").read_text().splitlines()]
    p = [np.asarray(start, dtype=float)] + [s.p for s in sent]
    yaw = [0.0] + [camera_yaw(s) for s in sent]
    vs = rep.get("vs_truth", {})
    return {"scene": scene.name, "start": list(start), "strategy": strategy, "stop": rep.get("stop_reason"), "views": rep.get("views"),
            "frontier_views": len(sent) - cfg["explore"]["sweep_yaws"], "legs": int(sum(legs)), "path_m": round(float(sum(np.linalg.norm(np.diff(p, axis=0), axis=1))), 1),
            "turn_deg": round(float(np.degrees(sum(abs(wrap(b - a)) for a, b in zip(yaw, yaw[1:]))))),
            "flight_s": rep.get("flight_s"), "fitted": scene_fit is not None, "min_face": round(min(rep.get("faces", {0: 0}).values()), 2),
            "min_wall": round(min([v for k, v in rep.get("faces", {}).items() if k != "ceiling"] or [0]), 2),   # the tour's gate
            "faces": rep.get("faces"), "targets": [[round(float(x), 1) for x in s.p] for s in sent],
            "known": round(rep.get("known_fraction", 0.0), 3), "wall_err_m": vs.get("roi_max_err_m"),
            "obst": f"{vs.get('obstacles_found', '-')}/{vs.get('obstacles_true', '-')} ({vs.get('obstacles_fitted', '-')} fitted)",
            "plan_ms_mean": round(1e3 * float(np.mean(plan_s)), 1) if plan_s else None,
            "plan_ms_max": round(1e3 * float(np.max(plan_s)), 1) if plan_s else None,
            "plan_s_total": round(float(np.sum(plan_s)), 2), "wall_s": round(wall, 1),
            "curve": [(c.get("frames"), round(min(c["faces"].values()), 3) if c.get("faces") else 0.0) for c in curve],
            "steps": steps}


def step_stats(cands, pick, chosen, ec, n_scored, n_front):
    pred = [d["gain"] for d in cands]
    res = [d["true"]["resolved"] for d in cands]
    known = [d["true"]["new_known"] for d in cands]
    occ = [d["true"]["new_occ"] for d in cands]
    u_true = [d["true"]["resolved"] - d["cost"] for d in cands]
    cost_chosen = next((d["cost"] for d in cands if np.allclose(d["pose"].p, pick[0].p) and np.allclose(d["pose"].q, pick[0].q)), None)
    u_chosen = chosen["resolved"] - (cost_chosen if cost_chosen is not None else 0.0)
    best = max(u_true)
    sorted_u = sorted(u_true, reverse=True)
    return {"n_scored": n_scored, "frontier": n_front, "n_eval": len(cands),
            "pred_gain": round(pick[1]), "true_resolved": round(chosen["resolved"]), "true_new_known": chosen["new_known"],
            "rho_resolved": spearman(pred, res), "rho_known": spearman(pred, known), "rho_occ": spearman(pred, occ),
            "rho_cone_resolved": spearman([d["u_cone"] + d["cost"] for d in cands], res),
            "regret": round((best - u_chosen) / max(abs(best), 1.0), 3),
            "top3": bool(u_chosen >= sorted_u[min(2, len(sorted_u) - 1)] - 1e-6),
            "best_resolved": round(max(res)), "best_known": max(known), "pred_err": round((pick[1] - chosen["resolved"]) / max(pick[1], 1.0), 3)}


# --- reports ---------------------------------------------------------------------------------------------------------
def table(rows, cols):
    w = [max(len(c), *(len(str(r.get(c))) for r in rows)) for c in cols]
    print("  ".join(c.ljust(n) for c, n in zip(cols, w)))
    for r in rows:
        print("  ".join(str(r.get(c)).ljust(n) for c, n in zip(cols, w)))


def nanmean(x):
    x = [v for v in x if v == v]
    return round(float(np.mean(x)), 3) if x else None


def run_prediction(sc, quick):
    out = []
    for scene, vol, starts in sc:
        for start in starts[:1] if quick else starts:
            r = explore_once(scene, vol, start, "nbv", analyse=True)
            s = r["steps"]
            row = {"scene": scene.name, "start": start, "steps": len(s),
                   "rho_resolved": nanmean([x["rho_resolved"] for x in s]), "rho_known": nanmean([x["rho_known"] for x in s]),
                   "rho_occ": nanmean([x["rho_occ"] for x in s]), "rho_cone": nanmean([x["rho_cone_resolved"] for x in s]),
                   "regret_mean": nanmean([x["regret"] for x in s]), "regret_p90": round(float(np.percentile([x["regret"] for x in s], 90)), 3) if s else None,
                   "top3": f"{sum(x['top3'] for x in s)}/{len(s)}", "pred_err_mean": nanmean([x["pred_err"] for x in s]),
                   "known_vs_best": nanmean([x["true_new_known"] / max(x["best_known"], 1) for x in s])}
            out.append({**row, "detail": s})
            print(f"  prediction {scene.name} {start}: {len(s)} steps", flush=True)
    print("\nPREDICTION (per step, NBV choice vs every evaluated candidate flown in simulation)")
    table(out, ["scene", "start", "steps", "rho_resolved", "rho_known", "rho_occ", "rho_cone", "regret_mean", "regret_p90", "top3",
                "pred_err_mean", "known_vs_best"])
    return out


def run_e2e(sc, quick):
    out = []
    strategies = ["nbv", "nbv_cone", "nearest", "random", "oracle"]
    for scene, vol, starts in sc:
        for start in starts[:1] if quick else starts:
            for st in strategies:
                for seed in ((0,) if st != "random" or quick else (0, 1, 2)):
                    r = explore_once(scene, vol, start, st, seed=seed)
                    r.pop("steps")
                    out.append(r)
                    print(f"  e2e {scene.name} {start} {st} s{seed}: {r['stop']} {r['views']} views {r['path_m']} m", flush=True)
    print("\nEND TO END (exploration until the room is closed / stable)")
    table(out, ["scene", "start", "strategy", "stop", "views", "legs", "path_m", "turn_deg", "flight_s", "min_face", "wall_err_m",
                "obst", "plan_ms_mean", "plan_ms_max", "wall_s"])
    return out


SWEEP = {"cand_step_m": [0.5, 1.0, 1.5], "n_yaw": [8, 12, 16], "max_candidates": [200, 400, 800], "max_frontier_pts": [500, 1500, 4000],
         "los_top": [1, 4, 8, 16], "travel_weight": [1.0, 4.0, 10.0]}


def run_sweep(sc, quick):
    out = []
    for scene, vol, starts in sc:
        if scene.name.startswith("warehouse") and quick:
            continue
        for key, vals in SWEEP.items():
            for v in vals:
                r = explore_once(scene, vol, starts[0], "nbv", ec={key: v})
                r.pop("steps")
                out.append({"param": f"{key}={v}", **r})
                print(f"  sweep {scene.name} {key}={v}: {r['stop']} {r['views']} views {r['path_m']} m {r['plan_ms_mean']} ms", flush=True)
    print("\nSWEEP (one parameter at a time, first start of each scene)")
    table(out, ["scene", "param", "stop", "views", "legs", "path_m", "flight_s", "min_face", "wall_err_m", "obst", "plan_ms_mean", "plan_ms_max"])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("parts", nargs="*", default=["prediction", "e2e", "sweep"])
    ap.add_argument("--quick", action="store_true", help="one start per scene, no warehouse sweep, one random seed")
    ap.add_argument("--only", help="scene name prefix")
    ap.add_argument("--out", default=str(ROOT / "runs" / "nbv_bench"))
    a = ap.parse_args()
    sc = [s for s in scenes() if not a.only or s[0].name.startswith(a.only)]
    res = {}
    for part in a.parts:
        res[part] = {"prediction": run_prediction, "e2e": run_e2e, "sweep": run_sweep}[part](sc, a.quick)
    Path(a.out).mkdir(parents=True, exist_ok=True)
    f = Path(a.out) / f"{time.strftime('%Y%m%d-%H%M%S')}_{'-'.join(a.parts)}.json"
    f.write_text(json.dumps(res, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)))
    print(f"\nwritten {f}")


if __name__ == "__main__":
    main()
