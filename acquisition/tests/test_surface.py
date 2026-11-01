"""surface_normal (planner/surface.py) on the dry rig: coarse orbit -> ZED-like depth of the ROI box -> one view per box side,
at the standoff along the side's normal, looking at it, every leg clear of the ROI shell."""
import numpy as np

from acquisition.planner.loop import run_session
from acquisition.planner.surface import group, surface_patches, viewpoints
from acquisition.rig.base import load_rig
from kinetix.config import ROOT, load_config, load_scene
from kinetix.coverage import CoverageState
from kinetix.geometry import quat_to_R, segment_hits_box
from kinetix.runlog import RunLog
from kinetix.types import Intrinsics

SCENE = ROOT / "kinetix" / "scenes" / "simple_room_blocks.yaml"


def test_patches_and_viewpoints_of_a_box_side():
    scene = load_scene(SCENE)
    x0 = scene.roi.lo[0]                                       # the box's -x face, sampled, seen from -x
    y, z = np.meshgrid(np.linspace(-0.15, 0.15, 30), np.linspace(1.0, 1.2, 20))
    P = np.c_[np.full(y.size, x0), y.ravel(), z.ravel()]
    cams = np.tile([x0 - 1.5, 0.0, 1.1], (len(P), 1))
    C, M = surface_patches(P, cams, 0.1)
    assert len(C) and np.allclose(M, [-1, 0, 0], atol=0.05)    # oriented toward the camera
    vs = viewpoints(C, M, 0.8, 0.7, scene, 0.0)
    for v in vs:
        assert abs(v.pose.p[0] - (x0 - 0.8)) < 0.05 and quat_to_R(v.pose.q)[0, 2] > 0.99   # 0.8 m out, looking +x
    g = group(vs, 2, 0.0)
    assert len(g) <= 2 + 1 and all(quat_to_R(c.pose.q)[0, 2] > 0.99 for c in g)


def test_surface_normal_dry_run(tmp_path):
    cfg = load_config([], ["planner.budget=14", "rig.save_depth=true"])
    scene = load_scene(SCENE)
    cam = cfg["camera"]
    recon = CoverageState(scene.roi, Intrinsics(**{k: cam[k] for k in ("width", "height", "fx", "fy", "cx", "cy")}))
    log = RunLog(tmp_path, scene.name, "surface_normal", "dry", 0, cfg, uses_gt=False)
    rig = load_rig("dry", cfg, scene, log.dir)
    flown = [rig.pose().p]                                     # every pose the rig is sent to, transits included
    for name in ("move_to", "capture_at"):
        real = getattr(rig, name)
        setattr(rig, name, lambda target, *a, _real=real, **k: (flown.append(target.p), _real(target, *a, **k))[1])
    out = run_session(cfg, scene, rig, recon, "surface_normal", log)
    import json
    frames = [json.loads(line) for line in (log.dir / "frames.jsonl").read_text().splitlines()]
    steps = [json.loads(line) for line in (log.dir / "steps.jsonl").read_text().splitlines()]
    coarse = [f for f, s in zip(frames, steps) if s["stage"] == "coarse"]
    fine = [f for f, s in zip(frames, steps) if s["stage"] == "fine"]
    assert len(coarse) >= 4 and len(fine) >= 4 and len(frames) <= 14
    sc = cfg["planner"]["surface"]
    sides = set()
    for f in fine:
        p, R = np.array(f["pose"]["p"]), quat_to_R(np.array(f["pose"]["q"]))
        d = np.maximum(scene.roi.lo - p, 0) + np.maximum(p - scene.roi.hi, 0)
        assert abs(np.linalg.norm(d) - sc["standoff_m"]) < 0.15                    # at the standoff from the box
        to_box = scene.roi.center[:2] - p[:2]
        assert R[:2, 2] @ (to_box / np.linalg.norm(to_box)) > 0.7                    # looking at it
        sides.add(int(np.argmax(np.abs(p[:2] - scene.roi.center[:2]))) * 2 + int((p - scene.roi.center)[np.argmax(np.abs(p[:2] - scene.roi.center[:2]))] > 0))
    assert len(sides) >= 3                                                           # views on several sides, not one
    shell = scene.roi.inflated(cfg["planner"]["roi_clearance_m"])
    assert not any(segment_hits_box(a, b, shell) for a, b in zip(flown, flown[1:]))  # no leg through the object
    assert out["stop_reason"] in ("budget", "sequence_done")


def test_depth_coverage_switches_to_the_depth_surface(tmp_path):
    # the dry rig's depth is 8x subsampled: ~3 points per 4 cm patch (the ZED's full-resolution depth gives hundreds)
    cfg = load_config([], ["planner.budget=12", "rig.save_depth=true", "planner.depth_surface.min_points=3"])
    scene = load_scene(SCENE)
    cam = cfg["camera"]
    K = Intrinsics(**{k: cam[k] for k in ("width", "height", "fx", "fy", "cx", "cy")})
    from kinetix.coverage import CoverageState
    recon = CoverageState.for_scene(scene, K, cfg["coverage"])
    log = RunLog(tmp_path, scene.name, "depth_coverage", "dry", 0, cfg, uses_gt=False)
    out = run_session(cfg, scene, load_rig("dry", cfg, scene, log.dir), recon, "depth_coverage", log)
    text = (log.dir / "log.txt").read_text()
    assert "depth_coverage: surface from 4 views' depth" in text and log.frames > 4 and out["coverage"] > 0.1
    n_patches = int(text.split("depth: ")[1].split(" patches")[0])
    assert n_patches > 50                                       # the box sides the 4 views saw, now as depth patches


def test_gt_surface_thins_and_requires_the_npz(tmp_path):
    from types import SimpleNamespace

    import pytest

    from kinetix.coverage import gt_surface
    P = np.random.default_rng(0).uniform(0, 0.2, (5000, 3))
    np.savez(tmp_path / "gt_observable.npz", P=P, N=np.tile([0.0, 0.0, 1.0], (len(P), 1)))
    s = SimpleNamespace(name="t", gt_mesh=tmp_path / "gt_mesh.ply")
    Q, N = gt_surface(s, 0.04)
    assert len(Q) <= 5 ** 3 and len(Q) == len(N) and len(Q) > 50                  # one point per 4 cm voxel of a 20 cm cube
    with pytest.raises(FileNotFoundError):
        gt_surface(SimpleNamespace(name="t", gt_mesh=None), 0.04)
