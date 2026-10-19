"""Room scenes (ADR-K11): inside-out coverage, occlusion, matchable-angle rules, grid candidates, room constraints, Sim3."""
import ast

import numpy as np

from kinetix.core.config import ROOT, load_config, load_scene
from kinetix.core.geometry import camera_pose, look_at_yaw
from kinetix.core.runlog import RunLog
from kinetix.core.types import Box, Frame, Intrinsics, RigCaps, Scene
from kinetix.planner.constraints import BoxConstraints
from kinetix.planner.loop import run_session
from kinetix.planner.sampler import grid_poses
from kinetix.recon.align import apply, ate, umeyama
from kinetix.recon.coverage import CoverageState, room_surface, segments_hit_box
from kinetix.rig.base import load_rig

K = Intrinsics(1280, 720, 529.8, 529.8, 640.0, 360.0)
ROOM = Box(np.array([0.0, 0.0, 2.0]), np.array([10.0, 8.0, 4.0]))          # x -5..5, y -4..4, z 0..4
CRATE = Box(np.array([3.0, 0.0, 0.5]), np.array([1.0, 1.0, 1.0]))          # x 2.5..3.5, on the floor
CAPS = RigCaps(continuous=True, has_gt=True, mount_pitch_rad=0.0, speed_mps=1.0)
SCENE = Scene("room_test", ROOM, Box(np.array([0.0, 0.0, 2.0]), np.array([7.0, 5.0, 2.4])), (CRATE,), (0.0, np.inf),
              (0.9, 3.0), standby=np.array([-2.0, 0.0, 1.5]), kind="room")
COV = dict(spacing_m=0.5, depth_range_m=[0.3, 12.0], max_incidence_deg=75.0, tri_min_deg=8.0, novel_deg=25.0, max_views=6,
           match_max_deg=35.0, min_overlap=0.3)


def frame(i, pose):
    return Frame(i, 0.0, None, K, pose, pose, None, 0.0)


def test_room_surface_normals_point_inward_and_floor_under_obstacle_is_dropped():
    P, N = room_surface(ROOM, (CRATE,), 0.5)
    inner = np.all((P > ROOM.lo - 1e-6) & (P < ROOM.hi + 1e-6), axis=1)
    assert inner.all()
    centre_dir = ROOM.center[None] - P
    walls = np.isclose(np.abs(P[:, 0]), 5.0) | np.isclose(np.abs(P[:, 1]), 4.0)
    assert (np.einsum("ij,ij->i", N[walls], centre_dir[walls]) > 0).all()   # wall normals face the room centre
    under = (np.abs(P[:, 2]) < 1e-6) & (P[:, 0] > 2.6) & (P[:, 0] < 3.4) & (np.abs(P[:, 1]) < 0.4)
    assert not under.any()                                                    # no floor inside the crate footprint
    assert ((P[:, 2] > 0.99) & (P[:, 2] < 1.01) & (N[:, 2] > 0.9)).any()      # crate top is surface


def test_obstacle_occludes_the_wall_behind_it_but_not_its_own_face():
    o = np.array([0.0, 0.0, 0.5])
    behind = np.array([[5.0, 0.0, 0.5]])                                      # east wall, behind the crate
    face = np.array([[2.5, 0.0, 0.5]])                                        # the crate's west face
    assert segments_hit_box(o, behind, CRATE)[0]
    assert not segments_hit_box(o, face, CRATE)[0]
    s = CoverageState.for_scene(SCENE, K, COV)
    m, _ = s.visible(camera_pose(o, 0.0))
    hidden = np.isclose(s.P[:, 0], 5.0) & (np.abs(s.P[:, 1]) < 0.3) & (s.P[:, 2] < 0.9)
    assert hidden.any() and not m[hidden].any()


def test_triangulation_needs_a_matchable_baseline():
    s = CoverageState.for_scene(SCENE, K, COV)
    target = np.array([0.0, -4.0, 1.5])                                       # south wall point
    near = lambda p: np.argmin(np.linalg.norm(s.P - target, axis=1))          # noqa: E731
    i = near(target)
    a = np.array([-1.5, 1.0, 1.5])
    s.add(frame(0, camera_pose(a, look_at_yaw(a, target))))
    b = np.array([3.0, 0.5, 1.5])                                             # ~50 deg away: not matchable
    s.add(frame(1, camera_pose(b, look_at_yaw(b, target))))
    assert s.count[i] == 2 and not s.tri[i]
    c = np.array([-0.6, 1.0, 1.5])                                            # ~15 deg from `a`: triangulates
    s.add(frame(2, camera_pose(c, look_at_yaw(c, target))))
    assert s.tri[i]


def test_min_overlap_gates_views_that_cannot_register():
    s = CoverageState.for_scene(SCENE, K, COV)
    p = np.array([-2.0, 0.0, 1.5])
    s.add(frame(0, camera_pose(p, 0.0)))                                      # looking east
    assert s.gain(camera_pose(p + [0.5, 0.0, 0.0], 0.0)) > 0                  # same direction, small step: overlaps
    assert s.overlap(camera_pose(p, np.pi)) < 0.3                             # turned around: sees nothing matchable
    assert s.gain(camera_pose(p, np.pi)) == 0.0


def test_grid_candidates_fill_the_workspace_and_room_constraints_skip_the_roi_shell():
    cands = grid_poses(SCENE, CAPS, 1.0, [1.2, 2.5], 4)
    P = np.array([c.pose.p for c in cands])
    assert (P >= SCENE.workspace.lo - 1e-9).all() and (P <= SCENE.workspace.hi + 1e-9).all()
    feas = BoxConstraints(0.5, 0.45).feasible(cands, camera_pose(SCENE.standby, 0.0), SCENE)
    assert feas.mean() > 0.8                                                  # inside the room ROI is allowed
    near_crate = np.array([np.all(np.abs(p[:2] - CRATE.center[:2]) < 1.0) and p[2] < 1.5 for p in P])
    assert near_crate.any() and not feas[near_crate].any()                    # no-fly box + clearance


def test_umeyama_recovers_a_sim3():
    rng = np.random.default_rng(0)
    X = rng.normal(size=(20, 3))
    a = 0.7
    R = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1.0]])
    Y = apply(2.5, R, np.array([1.0, -2.0, 0.5]), X)
    s, R2, t2 = umeyama(X, Y)
    assert abs(s - 2.5) < 1e-9 and np.allclose(R2, R) and ate(X, Y)["rmse_m"] < 1e-9


def test_room_dry_run_reaches_the_target_on_the_warehouse(tmp_path):
    scene = load_scene(ROOT / "config" / "scenes" / "warehouse_room.yaml")
    cfg = load_config([], ["planner.budget=120"], kind="room")
    assert cfg["coverage"]["spacing_m"] == 0.5 and "grid" in cfg["planner"]   # room.yaml is layered
    cam = cfg["camera"]
    recon = CoverageState.for_scene(scene, Intrinsics(**{k: cam[k] for k in ("width", "height", "fx", "fy", "cx", "cy")}),
                                    cfg["coverage"])
    log = RunLog(tmp_path, scene.name, "roi_coverage", "dry", 0, cfg, uses_gt=False)
    res = run_session(cfg, scene, load_rig("dry", cfg, scene, log.dir), recon, "roi_coverage", log)
    assert res["stop_reason"] == "coverage_target" and log.frames < 80


def test_offline_never_imports_the_planner():
    for f in (ROOT / "src" / "kinetix" / "offline").rglob("*.py"):
        for node in ast.walk(ast.parse(f.read_text())):
            mod = node.module if isinstance(node, ast.ImportFrom) else None
            assert not (mod or "").startswith("kinetix.planner"), f"{f}: imports {mod}"


def test_scene_and_coverage_shift_together():
    from kinetix.core.types import shifted
    d = np.array([0.1, -0.2, 0.25])
    s2 = shifted(SCENE, d)
    assert np.allclose(s2.roi.center, ROOM.center + d) and np.allclose(s2.no_fly[0].center, CRATE.center + d)
    assert np.allclose(s2.standby, SCENE.standby + d) and s2.kind == "room"
    a = CoverageState.for_scene(SCENE, K, COV)
    b = CoverageState.for_scene(s2, K, COV)
    a.shift(d)
    assert np.allclose(a.P, b.P) and np.allclose(a.occluders[0].center, b.occluders[0].center)
    pose = camera_pose(SCENE.standby + d, 0.0)
    assert (a.visible(pose)[0] == b.visible(pose)[0]).all()


def test_tour_order_is_shorter_than_the_input_order_and_the_path_passes_every_waypoint():
    from kinetix.planner.tour import order_tour, sample_path, tour_length
    rng = np.random.default_rng(1)
    poses = [camera_pose(rng.uniform([-3, -2, 1.2], [3, 2, 2.4]), rng.uniform(-np.pi, np.pi)) for _ in range(25)]
    start = camera_pose(SCENE.standby, 0.0)
    order = order_tour(start, poses, yaw_cost_m=1.0)
    assert sorted(order) == list(range(25))
    tour = [poses[i] for i in order]
    d0, y0 = tour_length(start, poses)
    d1, y1 = tour_length(start, tour)
    assert d1 + y1 < 0.6 * (d0 + y0)
    path = sample_path(start, tour, 0.25, np.radians(3))
    P = np.array([p.p for p in path])
    assert np.max(np.linalg.norm(np.diff(P, axis=0), axis=1)) <= 0.25 + 1e-9
    for w in tour:
        assert np.min(np.linalg.norm(P - w.p, axis=1)) < 1e-9


def test_dry_tour_covers_the_warehouse_on_a_short_path(tmp_path):
    scene = load_scene(ROOT / "config" / "scenes" / "warehouse_room.yaml")
    cfg = load_config([], [], kind="room")
    cam = cfg["camera"]
    recon = CoverageState.for_scene(scene, Intrinsics(**{k: cam[k] for k in ("width", "height", "fx", "fy", "cx", "cy")}),
                                    cfg["coverage"])
    log = RunLog(tmp_path, scene.name, "tour", "dry", 0, cfg, uses_gt=False)
    res = run_session(cfg, scene, load_rig("dry", cfg, scene, log.dir), recon, "tour", log)
    assert res["stop_reason"] == "tour_done" and res["coverage"] > 0.9 and log.travel_m < 150
