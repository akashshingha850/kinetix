import ast
from pathlib import Path

import numpy as np

from kinetix.core.config import ROOT, load_config, load_scene
from kinetix.core.geometry import camera_pose
from kinetix.core.runlog import RunLog
from kinetix.core.types import Intrinsics
from kinetix.planner.constraints import BoxConstraints
from kinetix.planner.loop import run_session
from kinetix.planner.sampler import ring_stack
from kinetix.recon.coverage import CoverageState
from kinetix.rig.base import load_rig

SCENE = ROOT / "config" / "scenes" / "simple_room_blocks.yaml"


def _setup(tmp_path, method, budget=20):
    cfg = load_config([], [f"planner.budget={budget}"])
    scene = load_scene(SCENE)
    cam = cfg["camera"]
    recon = CoverageState(scene.roi, Intrinsics(**{k: cam[k] for k in ("width", "height", "fx", "fy", "cx", "cy")}))
    log = RunLog(tmp_path, scene.name, method, "dry", 0, cfg, uses_gt=False)
    return cfg, scene, recon, log, load_rig("dry", cfg, scene, log.dir)


def test_constraints_respect_table_roi_and_segments():
    cfg = load_config()
    scene = load_scene(SCENE)
    rig = load_rig("dry", cfg, scene, None)
    cands = ring_stack(scene, rig.caps, [1.2], [1.0], 24)
    beside = camera_pose([2.5, 1.6, 1.0], -np.pi / 2)          # low, north of the table, looking at the ROI
    mask = BoxConstraints(0.35, 0.45).feasible(cands, beside, scene)
    table = scene.no_fly[0].inflated(0.35)
    for c, ok in zip(cands, mask):
        over_table = np.all(c.pose.p >= table.lo) and np.all(c.pose.p <= table.hi)
        assert not (ok and over_table)
    assert mask.any() and not mask.all()       # the low ring is only flyable beside the table
    # from the standby point above the spawn, every straight segment to the low ring clips the table corner
    assert not BoxConstraints(0.35, 0.45).feasible(cands, rig.pose(), scene).any()


def test_nbv_beats_orbit_and_random_on_images(tmp_path):
    res = {}
    for m in ("roi_coverage", "orbit", "random"):
        cfg, scene, recon, log, rig = _setup(tmp_path, m)
        out = run_session(cfg, scene, rig, recon, m, log)
        res[m] = (log.frames, out["coverage"])
        log.close(out["stop_reason"])
    assert res["roi_coverage"][1] >= 0.95
    assert res["roi_coverage"][0] < res["orbit"][0] and res["roi_coverage"][0] < res["random"][0]


def test_run_dir_contents(tmp_path):
    cfg, scene, recon, log, rig = _setup(tmp_path, "roi_coverage", budget=3)
    out = run_session(cfg, scene, rig, recon, "roi_coverage", log)
    log.close(out["stop_reason"])
    names = {p.name for p in log.dir.iterdir()}
    assert {"run.yaml", "frames.jsonl", "steps.jsonl", "log.txt", "frames"} <= names
    assert len((log.dir / "frames.jsonl").read_text().splitlines()) == 3


def test_dependency_rule():
    """planner/ and core/ never import rclpy, rig adapters or offline/ (architecture.md §4)."""
    bad = {"rclpy", "kinetix.rig.mavros", "kinetix.rig.dry", "kinetix.offline"}
    for sub in ("planner", "core", "recon"):
        for f in (ROOT / "src" / "kinetix" / sub).rglob("*.py"):
            for node in ast.walk(ast.parse(f.read_text())):
                names = [a.name for a in node.names] if isinstance(node, ast.Import) else \
                        [node.module or ""] if isinstance(node, ast.ImportFrom) else []
                for n in names:
                    assert not any(n == b or n.startswith(b + ".") for b in bad), f"{f}: imports {n}"
