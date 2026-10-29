"""Tour benchmark: what a planning knob costs in flight and what it buys in viewing geometry, on the dry rig.

    python3 acquisition/tests/tour_bench.py [--ranges 12 8 5] [--factors 1.5 2] [--explore] [--real RUN ...] [--out data/tour_bench]

For each value of `coverage.depth_range_m` (the farthest surface the planner counts as covered), and for each
`planner.tour.range_factor` (the far limit per surface point: factor x its closest reachable viewing distance, clipped to
[near_m, 12 m]), a whole tour of the Warehouse is planned and flown on the dry rig (no images, no drift), and the keyframes
are scored against the true room surface:

  cost      viewpoints, keyframes, metres, estimated flight time
  geometry  every surface point the camera really sees (up to `--see-m`, in the field of view, incidence <= 75 deg, not occluded):
            nearest viewing distance (GSD = distance / fx), `near_seen`: the share of the surface seen from within `--near-m`, and
            `near_tri`: the share with two views both within `--near-m` and 8-20 deg apart (matchable and triangulable up close).
            No triangulation angle: real SIFT tracks break after a few keyframes (mean length 6.4 on `163032`), which a ray model
            cannot predict (it chained the continuous keyframes into 59 deg tracks where SfM measured 7.5 deg); SfM measures it.

`--real RUN` scores a flown run's keyframes (ground-truth poses) the same way, to tie the dry numbers to an SfM result.
"""
from __future__ import annotations

import argparse
import json
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from acquisition.planner.explore import hint_scene
from acquisition.planner.loop import run_session
from acquisition.rig.base import load_rig
from kinetix.config import DATA, SCENES, load_config, load_scene
from kinetix.coverage import CoverageState
from kinetix.runlog import RunLog
from kinetix.types import Intrinsics, Pose

MAX_RAYS = 80                                    # rays kept per surface point for the pair test (evenly subsampled)


def intrinsics(cfg) -> Intrinsics:
    return Intrinsics(**{k: cfg["camera"][k] for k in ("width", "height", "fx", "fy", "cx", "cy")})


def read_frames(run: Path, key: str = "pose") -> list[Pose]:
    out = []
    for line in (run / "frames.jsonl").read_text().splitlines():
        d = json.loads(line)[key]
        out.append(Pose(np.asarray(d["p"], float), np.asarray(d["q"], float)))
    return out


def geometry(poses: list[Pose], scene, K: Intrinsics, see_m: float, near_m: float, spacing: float = 0.5) -> dict:
    """Viewing geometry of `poses` on the true surface of `scene` (see the module doc)."""
    ev = CoverageState.for_scene(scene, K, {"spacing_m": spacing, "depth_range_m": [0.3, see_m], "max_incidence_deg": 75.0,
                                            "tri_min_deg": 8.0, "novel_deg": 25.0, "max_views": 1})
    rays: list[list] = [[] for _ in ev.P]
    dists: list[list] = [[] for _ in ev.P]
    for pose in poses:
        m, ray = ev.visible(pose)
        d = np.linalg.norm(ev.P - pose.p[None], axis=1)
        for i in np.flatnonzero(m):
            rays[i].append(ray[i])
            dists[i].append(d[i])
    seen = np.array([len(r) > 0 for r in rays])
    nearest = np.array([min(d) for d in dists if d])
    near_tri = np.zeros(len(ev.P), dtype=bool)
    lo, hi = np.cos(np.radians(8.0)), np.cos(np.radians(20.0))
    for i, (r, d) in enumerate(zip(rays, dists)):
        r = np.asarray(r)[np.asarray(d) <= near_m]          # only the close views can form a close pair
        if len(r) < 2:
            continue
        if len(r) > MAX_RAYS:
            r = r[np.linspace(0, len(r) - 1, MAX_RAYS).astype(int)]
        c = r @ r.T
        near_tri[i] = bool(np.any((c < lo) & (c >= hi)))
    return {"surface_pts": len(ev.P), "seen": round(float(seen.mean()), 3),
            "dist_p50_m": round(float(np.median(nearest)), 2), "gsd_p50_mm": round(float(np.median(nearest)) / K.fx * 1e3, 1),
            "dist_p90_m": round(float(np.percentile(nearest, 90)), 2),
            "near_seen": round(float((nearest <= near_m).sum() / len(ev.P)), 3),
            "near_tri": round(float(near_tri.mean()), 3)}


def plan_and_fly(depth_max: float, explore: bool, overrides: list[str], tmp: Path) -> tuple[dict, list[Pose], object, Intrinsics]:
    """One dry-rig tour of the Warehouse with `coverage.depth_range_m` capped at `depth_max` (+ `overrides`)."""
    truth = load_scene(SCENES / "warehouse_room.yaml")
    ov = [f"coverage.depth_range_m=[0.3, {depth_max}]"] + (["explore.enabled=true"] if explore else []) + list(overrides)
    cfg = load_config([], ov, kind="room")
    scene = hint_scene(truth, cfg["explore"]["volume_m"]) if explore else truth
    log = RunLog(tmp, scene.name, "tour", "dry", 0, cfg, uses_gt=False)
    K = intrinsics(cfg)
    recon = CoverageState.for_scene(scene, K, cfg["coverage"])
    res = run_session(cfg, scene, load_rig("dry", cfg, scene, log.dir, truth if explore else None), recon, "tour", log, 0,
                      truth if explore else None)
    plan = next(json.loads(s) for s in (log.dir / "steps.jsonl").read_text().splitlines() if '"plan": "tour"' in s)
    poses = read_frames(log.dir)
    cost = {"stop": res["stop_reason"], "viewpoints": plan["viewpoints"], "keyframes": len(poses),
            "path_m": round(log.travel_m, 1), "est_flight_min": round(plan["est_flight_s"] / 60, 1),
            "plan_coverage": plan["plan_coverage"], "t_plan_s": plan["t_plan_s"]}
    return cost, poses, truth, K


def table(rows: list[dict]) -> str:
    cols = list(rows[0])
    w = [max(len(c), *(len(str(r.get(c, ""))) for r in rows)) for c in cols]
    line = lambda vals: "  ".join(str(v).rjust(n) for v, n in zip(vals, w))
    return "\n".join([line(cols), line(["-" * n for n in w])] + [line([r.get(c, "") for c in cols]) for r in rows])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ranges", type=float, nargs="*", default=[12.0, 8.0, 5.0], help="flat coverage.depth_range_m caps")
    ap.add_argument("--factors", type=float, nargs="*", default=[1.5, 2.0], help="planner.tour.range_factor values (12 m cap)")
    ap.add_argument("--explore", action="store_true", help="explore the room first (as flown), instead of reading the scene file")
    ap.add_argument("--real", nargs="*", default=[], help="flown run dirs to score the same way (ground-truth poses)")
    ap.add_argument("--see-m", type=float, default=20.0, help="how far the camera really sees, for the geometry")
    ap.add_argument("--near-m", type=float, default=5.0, help="close-range limit for near_tri")
    ap.add_argument("--out", default=str(DATA / "tour_bench"))
    ap.add_argument("overrides", nargs="*", help="key=value applied to every case")
    a = ap.parse_args()
    rows = []
    cfg = load_config([], [], kind="room")
    for run in a.real:
        run = Path(run)
        poses = read_frames(run, "gt_pose")
        g = geometry(poses, load_scene(SCENES / "warehouse_room.yaml"), intrinsics(cfg), a.see_m, a.near_m)
        rows.append({"case": f"flown {run.name[:15]}", "keyframes": len(poses), **g})
    with tempfile.TemporaryDirectory() as tmp:
        cases = [(f"dry range {r:g} m", r, []) for r in a.ranges]
        cases += [(f"dry reach x{f:g}", 12.0, [f"planner.tour.range_factor={f}", f"planner.tour.near_m={a.near_m}"]) for f in a.factors]
        for label, r, ov in cases:
            cost, poses, truth, K = plan_and_fly(r, a.explore, list(a.overrides) + ov, Path(tmp))
            rows.append({"case": label, **cost, **geometry(poses, truth, K, a.see_m, a.near_m)})
            print(table([rows[-1]]).splitlines()[-1], flush=True)
    keys = list(dict.fromkeys(k for r in rows for k in r))
    rows = [{k: r.get(k, "") for k in keys} for r in rows]
    print("\n" + table(rows))
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / f"tour_bench{'_explore' if a.explore else ''}.json").write_text(json.dumps(rows, indent=1))
    print(f"\nwritten: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
