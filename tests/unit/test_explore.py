"""Exploration of an unknown room (ADR-K13): ray-cast depth, the occupancy map, the room fit, safe exploration, explore-then-tour.

Everything runs on the dry rig, which casts synthetic depth against a *true* scene the planner never sees."""
import json

import numpy as np

from kinetix.core.config import ROOT, load_config, load_scene
from kinetix.core.geometry import camera_pose, segment_hits_box
from kinetix.core.runlog import RunLog
from kinetix.core.simdepth import render_depth
from kinetix.core.types import Box, Intrinsics, Scene
from kinetix.planner.explore import hint_scene, run_explore
from kinetix.planner.loop import run_session
from kinetix.recon.coverage import CoverageState
from kinetix.recon.depthmap import OccupancyMap
from kinetix.recon.roomfit import compare, fit_scene
from kinetix.rig.base import load_rig

K = Intrinsics(1280, 720, 529.8, 529.8, 640.0, 360.0)
ROOM = Box(np.array([0.0, 0.0, 2.0]), np.array([12.0, 9.0, 4.0]))            # x -6..6, y -4.5..4.5, z 0..4
CRATES = (Box(np.array([2.0, 1.5, 0.6]), np.array([1.2, 1.2, 1.2])), Box(np.array([-2.0, -1.0, 0.9]), np.array([1.0, 1.0, 1.8])))
TRUTH = Scene("explore_room", ROOM, Box(np.array([0.0, 0.0, 2.0]), np.array([9.0, 6.0, 2.0])), CRATES, (0.0, np.inf), (1.0, 3.0),
              standby=np.array([-4.0, -2.8, 1.5]), kind="room")


def explore_cfg(**ex):
    cfg = load_config([], [], kind="room")
    cfg["explore"].update({"enabled": True, "volume_m": [30.0, 30.0, 8.0], **ex})
    return cfg


def explore(tmp_path, truth=TRUTH, **ex):
    cfg = explore_cfg(**ex)
    hint = hint_scene(truth, cfg["explore"]["volume_m"])
    log = RunLog(tmp_path, truth.name, "tour", "dry", 0, cfg, uses_gt=False)
    rig = load_rig("dry", cfg, hint, log.dir, truth)
    sent = []
    capture = rig.capture_at
    rig.capture_at = lambda target, idx, pool_id=None: (sent.append(target), capture(target, idx, pool_id))[1]
    scene, rep = run_explore(cfg, hint, rig, log, truth)
    return scene, rep, sent, hint


def test_render_depth_hits_the_wall_and_the_crate_first():
    pose = camera_pose([-4.0, -2.8, 1.5], 0.0, 0.0)                        # looking along +x from x = -4
    d = render_depth(ROOM, (), pose, K)
    assert abs(d[d.shape[0] // 2, d.shape[1] // 2] - 10.0) < 0.05          # the far wall at x = 6
    crate = Box(np.array([0.0, -2.8, 1.5]), np.array([1.0, 1.0, 1.0]))     # straight ahead, front face at x = -0.5
    d = render_depth(ROOM, (crate,), pose, K)
    assert abs(d[d.shape[0] // 2, d.shape[1] // 2] - 3.5) < 0.05
    assert np.isnan(render_depth(ROOM, (), pose, K, max_range=5.0)[d.shape[0] // 2, d.shape[1] // 2])   # no return beyond range


def test_occupancy_map_marks_the_hit_occupied_the_ray_free_and_behind_unknown():
    omap = OccupancyMap([-8, -8, -1], [8, 8, 6], 0.2, depth_range=(0.3, 12.0))
    pose = camera_pose([-4.0, -2.8, 1.5], 0.0, 0.0)
    assert omap.integrate(render_depth(ROOM, (), pose, K), K, pose) > 0
    near_axis = lambda m: (lambda c: c[(np.abs(c[:, 1] + 2.8) < 0.6) & (np.abs(c[:, 2] - 1.5) < 0.6)])(omap.centers(m))   # noqa: E731
    wall = near_axis(omap.occupied)[:, 0]
    assert len(wall) and wall.min() > 5.7 and wall.max() < 6.3                  # the far wall, within a voxel of x = 6
    air = near_axis(omap.free)[:, 0]
    assert len(air) > 20 and air.max() < 6.0 and air.min() < -3.0                # air all along the way to it
    assert not (omap.centers(omap.occupied)[:, 0] > 6.4).any()                   # nothing behind the wall: never seen
    assert omap.frontier().any() and not (omap.frontier() & omap.occupied).any()


def test_safe_voxels_keep_clear_of_occupied_and_unknown_space():
    omap = OccupancyMap([-8, -8, -1], [8, 8, 6], 0.2, depth_range=(0.3, 12.0))
    for yaw in np.arange(0, 2 * np.pi, np.pi / 4):                           # a sweep so the air all round is known
        pose = camera_pose([-4.0, -2.8, 1.5], yaw, 0.0)
        omap.integrate(render_depth(ROOM, (), pose, K), K, pose)
    safe = omap.safe(0.8, 0.4)
    assert safe.any()
    pts = omap.centers(safe)
    dil = OccupancyMap.dilate(omap.occupied | omap.unknown, 4, 2)
    assert not dil[tuple(omap.index(pts)[0].T)].any()
    assert (pts[:, 0] > -6.0).all() and (pts[:, 0] < 6.0).all()


def test_exploration_closes_the_room_and_recovers_walls_and_obstacles(tmp_path):
    scene, rep, sent, _ = explore(tmp_path)
    assert scene is not None and rep["stop_reason"] in ("closed", "stable"), rep
    vs = rep["vs_truth"]
    assert vs["roi_max_err_m"] <= 0.25, vs                                   # every wall within a voxel and a bit
    assert vs["obstacles_found"] == 2 and vs["obstacles_fitted"] == 2, vs
    assert min(rep["faces"].values()) >= 0.25                              # every face seen (the 'stable' stop rule's floor)
    out = next(tmp_path.glob("*/explore"))
    assert (out / "scene.yaml").exists() and (out / "occupancy.npz").exists() and (out / "steps.jsonl").exists()
    assert json.loads((out / "report.json").read_text())["stop_reason"] in ("closed", "stable")


def test_exploration_flies_only_through_space_it_has_seen_to_be_empty(tmp_path):
    _, _, sent, _ = explore(tmp_path)
    assert len(sent) > 8
    prev = camera_pose(TRUTH.standby, 0.0, 0.0).p
    for pose in sent:
        p = pose.p
        assert np.all(p > ROOM.lo + 0.5) and np.all(p < ROOM.hi - 0.5), p      # inside the real room, off its walls
        for c in CRATES:
            assert not segment_hits_box(prev, p, c.inflated(0.4)), (prev, p)   # never through, or hugging, a real obstacle
        prev = p


def test_explore_does_not_read_the_scene_files_room():
    hint = hint_scene(TRUTH, [30.0, 30.0, 8.0])
    assert hint.no_fly == () and np.allclose(hint.roi.center, TRUTH.standby) and np.allclose(hint.roi.size, [30, 30, 8])
    assert np.allclose(hint.standby, TRUTH.standby) and hint.altitude == TRUTH.altitude


def test_fit_scene_needs_a_bounded_room():
    cfg = explore_cfg()
    omap = OccupancyMap([-8, -8, -1], [8, 8, 6], 0.2)
    scene, rep = fit_scene(omap, hint_scene(TRUTH, [16, 16, 7]), cfg["explore"]["fit"])
    assert scene is None and rep["closed"] is False                          # an empty map bounds nothing


def test_explore_then_tour_flies_a_tour_on_the_fitted_room(tmp_path):
    cfg = explore_cfg()
    hint = hint_scene(TRUTH, cfg["explore"]["volume_m"])
    cam = cfg["camera"]
    recon = CoverageState.for_scene(hint, Intrinsics(**{k: cam[k] for k in ("width", "height", "fx", "fy", "cx", "cy")}), cfg["coverage"])
    log = RunLog(tmp_path, TRUTH.name, "tour", "dry", 0, cfg, uses_gt=False)
    res = run_session(cfg, hint, load_rig("dry", cfg, hint, log.dir, TRUTH), recon, "tour", log, 0, TRUTH)
    assert res["stop_reason"] == "tour_done", res
    assert res["coverage"] > 0.5                                             # a 12x9 m room is knife-edge for the 2 m candidate grid (a 5 cm
                                                                             # workspace difference drops a row): the Warehouse test is the parity check
    steps = [json.loads(line) for line in (log.dir / "steps.jsonl").read_text().splitlines()]
    assert steps[0]["plan"] == "tour" and steps[0]["viewpoints"] > 3        # steps.jsonl still starts with the tour plan (the GCS reads it)


def test_explore_then_tour_on_the_warehouse(tmp_path):
    truth = load_scene(ROOT / "config" / "scenes" / "warehouse_room.yaml")
    scene, rep, _, _ = explore(tmp_path, truth=truth, volume_m=[40.0, 40.0, 12.0])
    assert scene is not None, rep
    vs = rep["vs_truth"]
    assert vs["roi_max_err_m"] <= 0.35, (vs, rep["stop_reason"])
    # the returned scene is in the EKF frame (what the tour flies in), vs_truth in the drift-corrected map frame: with no drift in
    # this run they agree to within what the registration estimated
    assert compare(scene, truth)["roi_max_err_m"] <= 0.35 + float(np.abs(rep["drift_offset_m"]).max())


def test_explore_then_tour_matches_the_tour_on_the_scene_file_for_the_warehouse(tmp_path):
    truth = load_scene(ROOT / "config" / "scenes" / "warehouse_room.yaml")
    cam = load_config([], [], kind="room")["camera"]
    K0 = Intrinsics(**{k: cam[k] for k in ("width", "height", "fx", "fy", "cx", "cy")})
    plans = {}
    explore_frames = {}
    for mode in ("file", "traditional", "reuse"):            # the two benchmark arms: explore then a full scan, explore views reused
        cfg = load_config([], [], kind="room")
        if mode != "file":
            cfg["explore"].update(enabled=True, reuse_views=mode == "reuse")
        scene = hint_scene(truth, cfg["explore"]["volume_m"]) if mode != "file" else truth
        log = RunLog(tmp_path / mode, truth.name, "tour", "dry", 0, cfg, uses_gt=False)
        res = run_session(cfg, scene, load_rig("dry", cfg, scene, log.dir, truth), CoverageState.for_scene(scene, K0, cfg["coverage"]),
                          "tour", log, 0, truth if mode != "file" else None)
        assert res["stop_reason"] == "tour_done" and res["coverage"] > 0.85, (mode, res)
        plans[mode] = json.loads((log.dir / "steps.jsonl").read_text().splitlines()[0])
        explore_frames[mode] = sum(str(json.loads(f)["pool_id"]).startswith("explore_") for f in (log.dir / "frames.jsonl").read_text().splitlines())
    a, t, r = plans["file"], plans["traditional"], plans["reuse"]
    # traditional: exploration only maps, the tour is the full scan of the fitted room (about the scene file's tour)
    assert explore_frames["traditional"] == 0 and abs(t["tour_m"] - a["tour_m"]) <= 0.15 * a["tour_m"], (a, t)
    # reuse: the tour starts from what exploration already covered, so it flies only the gaps
    assert explore_frames["reuse"] > 0 and r["tour_m"] < t["tour_m"] and r["viewpoints"] < t["viewpoints"], (t, r)


def test_returns_beyond_the_range_carve_air_and_mark_no_wall():
    """Real ZED depth is valid almost everywhere (14400/14400 rays in the sim). A return past the mapping range proves the air up
    to that range is empty, but marks no wall and is flagged as probed."""
    room = Box(np.array([0.0, 0.0, 2.0]), np.array([40.0, 40.0, 4.0]))              # walls all farther than the 8 m range
    pose = camera_pose([0.0, 0.0, 1.5], 0.0, 0.0)
    omap = OccupancyMap([-12, -12, -1], [12, 12, 6], 0.2, depth_range=(0.3, 8.0))
    omap.integrate(render_depth(room, (), pose, K, max_range=15.0), K, pose)
    occ = omap.centers(omap.occupied)
    assert len(occ) and ((occ[:, 2] < 0.35) | (occ[:, 2] > 3.65)).all()               # only floor and ceiling (within range), no wall
    assert omap.free.sum() > 1000 and omap.probed.sum() > 100


def test_unknown_space_past_a_range_limited_ray_is_not_frontier():
    omap = OccupancyMap([0, 0, 0], [4, 0.2, 0.2], 0.2)                              # a 20 x 1 x 1 line of voxels
    omap.L[0:10, 0, 0] = -3                                                           # air known up to voxel 9
    assert omap.frontier()[9, 0, 0]                                                   # voxel 10 is unknown: a hole to explore
    omap.probed[10, 0, 0] = True                                                      # ... unless a ray returned from beyond it
    assert not omap.frontier()[9, 0, 0]


def test_exploration_survives_ekf_drift_without_false_obstacles(tmp_path):
    """The live sim run: EKF2 drifted ~1.4 m over an 11 min exploration, the same wall was seen at several offsets, and the fit grew
    walls 1 m too far out and 30 obstacles (thin sheets beside the walls). Drift of the real size, short exploration."""
    cfg = explore_cfg()
    cfg["rig"]["dry_drift_m_per_sqrt_m"] = 0.05                                        # ~1 m of drift by the end: like the sim's first 10 min
    hint = hint_scene(TRUTH, cfg["explore"]["volume_m"])
    log = RunLog(tmp_path, TRUTH.name, "tour", "dry", 0, cfg, uses_gt=False)
    rig = load_rig("dry", cfg, hint, log.dir, TRUTH)
    scene, rep = run_explore(cfg, hint, rig, log, TRUTH)
    assert scene is not None, rep
    vs = rep["vs_truth"]
    assert vs["roi_max_err_m"] <= 0.9, vs                                              # drift-sized error, not a room 1.6 m too big
    assert vs["obstacles_fitted"] <= vs["obstacles_true"] + 1, vs                      # no pile of wall duplicates
    assert rep["drift_offset_m"] is not None                                            # the registration ran and reported an estimate


def test_drift_tracker_recovers_a_known_offset_and_leaves_unobservable_axes_alone():
    """Registration alone: a frame taken while the EKF is 0.4 m off in x, against a map built at the right poses. The end wall 4 m
    ahead pins x; little in that view fixes y or z: the estimate must find x and not invent the others."""
    from kinetix.recon.register import DriftTracker
    cfg = load_config([], [], kind="room")
    omap = OccupancyMap([-8, -8, -1], [8, 8, 6], 0.2, depth_range=(0.3, 8.0))
    for yaw in np.arange(0, 2 * np.pi, np.pi / 4):                                     # the map: a sweep at the right pose
        pose = camera_pose([2.0, -1.0, 1.5], yaw, 0.0)
        omap.integrate(render_depth(ROOM, (), pose, K, noise_frac=0.01), K, pose)
    true_pose = camera_pose([2.0, -1.0, 1.5], 0.0, 0.0)
    ekf_pose = camera_pose([2.0 + 0.4, -1.0, 1.5], 0.0, 0.0)                          # the EKF says x = 2.4; the truth is x = 2.0
    tr = DriftTracker(cfg["explore"]["register"])
    info = tr.update(omap, render_depth(ROOM, (), true_pose, K, noise_frac=0.01), K, ekf_pose, 0.3, 8.0, 8)
    assert info["applied"], info
    assert abs(tr.t[0] - (-0.4)) < 0.15, (tr.t, info)                                   # found the 0.4 m along x (the end wall pins it)
    assert abs(tr.t[1]) < 0.3, (tr.t, info)                                             # did not invent a large y shift


def test_exploration_stops_at_its_time_budget(tmp_path):
    """EKF2 drifts with flight time (live sim: 0.2 m by 5 min, >1 m by 10): exploring is bounded in time, not only in views."""
    scene, rep, _, _ = explore(tmp_path, max_time_s=20.0, min_views=100)
    assert rep["stop_reason"] == "time_budget" and rep["flight_s"] <= 60.0, rep


CLUTTER = Scene("clutter_room", Box(np.array([0.0, 0.0, 1.75]), np.array([10.0, 8.0, 3.5])),
                Box(np.array([0.0, 0.0, 1.75]), np.array([7.0, 5.0, 2.0])),
                (Box(np.array([-3, 2, 0.5]), np.array([1, 1.5, 1])), Box(np.array([-1, -2.2, 0.8]), np.array([1.2, 1, 1.6])),
                 Box(np.array([1.5, 1.8, 1.0]), np.array([0.8, 0.8, 2.0])), Box(np.array([3, -1.5, 0.4]), np.array([1.5, 1, 0.8])),
                 Box(np.array([0.5, 0.2, 0.6]), np.array([0.6, 0.6, 1.2]))),
                (0.0, np.inf), (1.0, 2.6), standby=np.array([-2.5, 0.0, 1.4]), kind="room")


def test_a_low_cluttered_room_is_explored_around_and_over_its_crates(tmp_path):
    """NBV benchmark finding (tests/nbv_bench.py): with the 0.8 m clearance in a 3.5 m room the safe space after the take-off sweep
    is a 1 m band over the crates. A fixed 1 m candidate lattice put 1 candidate in it, and a straight segment from the drone
    reached none: exploration ended after the sweep. Candidates per grid cell + legs through the corridor fix both."""
    scene, rep, sent, hint = explore(tmp_path, truth=CLUTTER, volume_m=[24.0, 24.0, 7.0])
    assert rep["stop_reason"] in ("closed", "stable"), rep
    assert rep["vs_truth"]["roi_max_err_m"] <= 0.25 and rep["vs_truth"]["obstacles_found"] == 5, rep["vs_truth"]
    prev = CLUTTER.standby
    for pose in sent:                                                          # the legs too: in the band, never through a crate
        assert hint.altitude[0] - 0.1 <= pose.p[2] <= hint.altitude[1] + 0.1, pose.p
        for c in CLUTTER.no_fly:
            assert not segment_hits_box(prev, pose.p, c.inflated(0.4)), (prev, pose.p)
        prev = pose.p


def test_a_partition_is_not_taken_for_the_rooms_wall(tmp_path):
    """NBV benchmark finding: a floor-to-ceiling partition 2 m short of the far wall covered 80 % of the face the fit put on it,
    so the room 'closed' 7 m short. Free space seen past a face now keeps it open, and the line-of-sight re-score goes past
    the first batch (its top 8 all looked at the frontier through the partition)."""
    split = Scene("split_room", Box(np.array([0.0, 0.0, 2.0]), np.array([14.0, 10.0, 4.0])),
                  Box(np.array([0.0, 0.0, 2.0]), np.array([11.0, 7.0, 2.0])), (Box(np.array([0.0, -1.0, 2.0]), np.array([0.4, 8.0, 4.0])),),
                  (0.0, np.inf), (1.0, 3.0), standby=np.array([-4.5, -3.0, 1.5]), kind="room")
    scene, rep, _, _ = explore(tmp_path, truth=split)
    assert scene is not None and rep["vs_truth"]["roi_max_err_m"] <= 0.25, (rep["stop_reason"], rep.get("vs_truth"))


def test_face_openings_count_free_space_past_a_face():
    from kinetix.recon.roomfit import face_openings
    free = np.zeros((20, 10, 10), dtype=bool)
    box = ((2, 12), (0, 9), (0, 9))
    assert all(n == 0 for n in face_openings(free, box).values())
    free[14:17, 3:6, 2:5] = True                                                    # seen through the x+ face (plane at 12)
    o = face_openings(free, box)
    assert o["x+"] == 27 and o["x-"] == 0


def test_the_tour_is_not_flown_on_a_room_exploration_did_not_fit(tmp_path):
    """Live sim, second run: a 28-view exploration left the walls at 2-8 % seen and 15 false obstacles; the tour then flew on that and
    the EKF diverged. A tour needs every wall (and the floor) seen at least `explore.tour_min_wall`."""
    cfg = explore_cfg(max_steps=3, tour_min_wall=0.9, min_views=100)
    hint = hint_scene(TRUTH, cfg["explore"]["volume_m"])
    cam = cfg["camera"]
    recon = CoverageState.for_scene(hint, Intrinsics(**{k: cam[k] for k in ("width", "height", "fx", "fy", "cx", "cy")}), cfg["coverage"])
    log = RunLog(tmp_path, TRUTH.name, "tour", "dry", 0, cfg, uses_gt=False)
    rig = load_rig("dry", cfg, hint, log.dir, TRUTH)
    res = run_session(cfg, hint, rig, recon, "tour", log, 0, TRUTH)
    assert res["stop_reason"] == "explore_incomplete", res
    frames = (log.dir / "frames.jsonl").read_text().splitlines()
    assert frames and all(json.loads(f)["pool_id"].startswith("explore_") for f in frames)      # only exploration views, no tour keyframes
