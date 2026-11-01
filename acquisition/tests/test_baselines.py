"""The K2 baselines on the dry rig (fvs, volumetric_ig, orbit_rings), bench seeds, and the T0 pool rig on a tiny fake pool."""
import json

import numpy as np

from acquisition.planner.loop import run_session
from acquisition.planner.nbv import spread_order
from acquisition.planner.sampler import ring_stack
from acquisition.rig.base import load_rig
from kinetix.config import ROOT, load_config, load_scene
from kinetix.coverage import CoverageState
from kinetix.runlog import RunLog
from kinetix.types import Intrinsics, RigCaps

SCENE = ROOT / "kinetix" / "scenes" / "simple_room_blocks.yaml"


def session(tmp_path, method, rig="dry", seed=0, extra=()):
    cfg = load_config([], ["planner.budget=10", "planner.coverage_target=2.0", "planner.min_gain=-1.0", *extra])
    scene = load_scene(SCENE)
    cam = cfg["camera"]
    recon = CoverageState.for_scene(scene, Intrinsics(**{k: cam[k] for k in ("width", "height", "fx", "fy", "cx", "cy")}), cfg["coverage"])
    log = RunLog(tmp_path, scene.name, method, rig, seed, cfg, uses_gt=False)
    out = run_session(cfg, scene, load_rig(rig, cfg, scene, log.dir), recon, method, log, seed)
    return [json.loads(x) for x in (log.dir / "frames.jsonl").read_text().splitlines()], out, log.dir


def test_fvs_spreads_the_views(tmp_path):
    frames, _, _ = session(tmp_path, "fvs")
    P = np.array([f["pose"]["p"] for f in frames])
    gaps = [np.linalg.norm(P[k] - P[:k], axis=1).min() for k in range(1, 5)]
    assert len(frames) == 10 and min(gaps) > 1.0                  # each of the first views far from all before it


def test_volumetric_ig_runs_on_depth(tmp_path):
    frames, _, _ = session(tmp_path, "volumetric_ig", extra=["rig.save_depth=true"])
    assert len(frames) == 10 and len({f["pool_id"] for f in frames}) == 10


def test_orbit_rings_prefixes_are_spread():
    scene = load_scene(SCENE)
    cfg = load_config()
    rc = cfg["planner"]["rings"]
    cands = ring_stack(scene, RigCaps(False, True, 0.0, 1.0), rc["radii"], rc["heights"], rc["n_az"])
    order = spread_order(cands, cands[0].pose)
    assert sorted(order) == list(range(len(cands)))             # every view exactly once
    az = [float(cands[i].pool_id.split("_a")[1]) for i in order[:8]]
    assert len({round(a / 45) % 8 for a in az}) >= 6            # the first 8 views cover most octants
    assert len({cands[i].pool_id.split("_a")[0] for i in order[:8]}) == 8   # ... on 8 different rings


def test_seeds_change_the_first_view(tmp_path):
    starts = {session(tmp_path, "roi_coverage", seed=s)[0][0]["pool_id"] for s in (1, 2, 3)}
    assert len(starts) >= 2


def test_pool_rig_replays_a_pool(tmp_path):
    import cv2
    scene = load_scene(SCENE)
    cfg = load_config()
    rc = cfg["planner"]["rings"]
    pool = tmp_path / "pool"
    for d in ("images", "depth"):
        (pool / d).mkdir(parents=True)
    cands = ring_stack(scene, RigCaps(False, True, 0.0, 1.0), rc["radii"], rc["heights"], rc["n_az"])
    with open(pool / "views.jsonl", "w") as fh:
        for c in cands:
            cv2.imwrite(str(pool / "images" / f"{c.pool_id}.png"), np.zeros((4, 4, 3), np.uint8))
            np.save(pool / "depth" / f"{c.pool_id}.npy", np.full((9, 16), 2.0, np.float32))
            fh.write(json.dumps({"id": c.pool_id, "split": "candidate", "pose": c.pose.to_dict()}) + "\n")
    frames, _, run = session(tmp_path, "volumetric_ig", rig="pool", extra=[f"rig.pool={pool}"])
    assert len(frames) == 10 and all((run / f["image"]).exists() for f in frames)
    assert all(f["pose"] == f["gt_pose"] for f in frames)
