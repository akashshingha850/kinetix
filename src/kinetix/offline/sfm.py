"""Offline SfM over a run's captured frames (architecture.md §9 step 1-2; `kinetix sfm <run>`).

pycolmap (CPU): SIFT features -> matching -> incremental mapping, with one PINHOLE camera fixed to the run's K (the ZED rect
image is undistorted and calibrated, so focal length and principal point are not refined).
Matching (`pairing`): `exhaustive` (every pair) or `covis` (default when a scene is known): only pairs of frames whose rig poses
(EKF2, never GT) co-observe at least `min_shared` proxy-surface points (10: the box proxy misses real structure, so the
bar is low and the rig check below removes the wrong pairs) from directions <= match_max_deg apart. In a room of
identical bays (the Warehouse repeats every 6 m) exhaustive matching registers images into the wrong bay; co-visibility pairing
keeps only geometrically plausible pairs (2026-10-17: exhaustive 28/48 cameras within 0.75 m of GT, the rest 6-28 m off).
Rig check (`rig_check`, default on): a verified pair is dropped before mapping when its estimated relative rotation differs from
the rig's (EKF2) by more than `max_rot_deg`, or, for baselines >= `min_baseline` m, its baseline direction by more than
`max_dir_deg`. A view matched into the wrong bay has the right rotation but a baseline pointing elsewhere.
Position priors (`prior_sigma_m`, default on): every image gets its rig camera centre (EKF2) as a pose prior with that standard
deviation, and mapping uses them with a robust loss (COLMAP prior-guided incremental mapping). SfM keeps the fine geometry while
the VIO keeps the global shape and scale: a 400-keyframe tour mapped without priors registered 98 % but drifted to 4.2 m ATE
(low-parallax video, scale 1.83-2.02 along the sequence). The largest model is
Sim3-aligned to the rig's camera centres (EKF2, `frames.jsonl` pose) and, in the sim, to the ground-truth centres (`gt_pose`).
Completeness is measured against the scene's proxy surface (recon/coverage.py): a surface point counts as reconstructed when a
sparse point lies in its voxel neighbourhood after the EKF2 alignment. Reads the run directory only (dependency rule).

Writes <run>/sfm/offline/{database.db, sparse/<k>/, points.ply, metrics.json}.
"""
from __future__ import annotations

import json
import shutil
import time
from pathlib import Path

import numpy as np
import yaml

from kinetix.core.config import load_scene
from kinetix.core.types import Intrinsics
from kinetix.recon.align import apply, ate, umeyama
from kinetix.recon.coverage import CoverageState


def _frames(run: Path) -> list[dict]:
    return [json.loads(line) for line in (run / "frames.jsonl").read_text().splitlines() if line.strip()]


def _surface_classes(P: np.ndarray, N: np.ndarray, n_room: int | None) -> dict[str, np.ndarray]:
    """Masks over the proxy surface: floor / ceiling / walls (room faces) and obstacles (the rest)."""
    idx = np.arange(len(P))
    room = idx < n_room if n_room is not None else np.ones(len(P), dtype=bool)
    return {"floor": room & (N[:, 2] > 0.9), "ceiling": room & (N[:, 2] < -0.9),
            "walls": room & (np.abs(N[:, 2]) < 0.1), "obstacles": ~room}


def _near(points: np.ndarray, query: np.ndarray, voxel: float) -> np.ndarray:
    """Per query point: is any of `points` in its voxel or one of the 26 neighbours (distance < ~2 voxels)?"""
    occ = {tuple(v) for v in np.floor(points / voxel).astype(np.int64)}
    q = np.floor(query / voxel).astype(np.int64)
    offs = np.array([(i, j, k) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)])
    return np.array([any(tuple(c + o) in occ for o in offs) for c in q])


def covis_pairs(frames: list[dict], cov: CoverageState, min_shared: int = 10) -> list[tuple[str, str, int]]:
    """(image_i, image_j, shared) for frame pairs that co-observe >= min_shared surface points from matchable directions,
    using the rig poses in frames.jsonl (`pose`, EKF2 on T2)."""
    from kinetix.core.types import Pose
    vis = []
    for f in frames:
        m, ray = cov.visible(Pose(np.asarray(f["pose"]["p"], dtype=float), np.asarray(f["pose"]["q"], dtype=float)))
        vis.append((m, ray))
    out = []
    for i in range(len(frames)):
        for j in range(i + 1, len(frames)):
            both = vis[i][0] & vis[j][0]
            if not both.any():
                continue
            cos = np.einsum("ij,ij->i", vis[i][1][both], vis[j][1][both])
            n = int((cos >= cov.cos_match).sum())
            if n >= min_shared:
                out.append((frames[i]["image"], frames[j]["image"], n))
    return out


def rig_check_pairs(db_path: Path, frames: list[dict], max_rot_deg: float = 5.0, max_dir_deg: float = 30.0,
                    min_baseline: float = 0.5) -> dict:
    """Delete verified pairs whose relative pose disagrees with the rig poses (frames.jsonl `pose`). Returns counts."""
    import pycolmap

    from kinetix.core.geometry import quat_to_R
    by_name = {f["image"]: f for f in frames}
    db = pycolmap.Database.open(str(db_path))
    names = {img.image_id: img.name for img in db.read_all_images()}
    pair_ids, geoms = db.read_two_view_geometries()
    kept = dropped_rot = dropped_dir = unchecked = 0
    for pid, g in zip(pair_ids, geoms):
        if len(g.inlier_matches) == 0:
            continue
        if g.cam2_from_cam1 is None:                  # no relative pose (pure rotation / degenerate): nothing to check
            unchecked += 1
            kept += 1
            continue
        i1, i2 = pycolmap.pair_id_to_image_pair(pid)
        f1, f2 = by_name[names[i1]], by_name[names[i2]]
        R1, R2 = quat_to_R(f1["pose"]["q"]), quat_to_R(f2["pose"]["q"])          # T_map_cam rotations (OpenCV optical)
        c1, c2 = np.asarray(f1["pose"]["p"]), np.asarray(f2["pose"]["p"])
        R_rig = R2.T @ R1                                                         # cam2_from_cam1
        R_est = np.asarray(g.cam2_from_cam1.rotation.matrix())
        rot = np.degrees(np.arccos(np.clip((np.trace(R_est.T @ R_rig) - 1) / 2, -1.0, 1.0)))
        bad = rot > max_rot_deg
        if bad:
            dropped_rot += 1
        elif np.linalg.norm(c1 - c2) >= min_baseline:
            t_rig = R2.T @ (c1 - c2)
            t_est = np.asarray(g.cam2_from_cam1.translation)
            if np.linalg.norm(t_est) > 1e-9:
                cosd = float(t_rig @ t_est / (np.linalg.norm(t_rig) * np.linalg.norm(t_est)))
                if np.degrees(np.arccos(np.clip(cosd, -1.0, 1.0))) > max_dir_deg:
                    bad = True
                    dropped_dir += 1
        if bad:
            db.delete_two_view_geometry(i1, i2)
        else:
            kept += 1
    db.close()
    return {"kept": kept, "unchecked": unchecked, "dropped_rotation": dropped_rot, "dropped_direction": dropped_dir,
            "max_rot_deg": max_rot_deg, "max_dir_deg": max_dir_deg, "min_baseline_m": min_baseline}


def video_pairs(frames: list[dict], cov: CoverageState | None, window: int = 10, top_k: int = 8,
                min_shared: int = 30) -> list[tuple[str, str, int]]:
    """Pairs for a continuous capture (method `tour`): each frame with its `window` successors, plus its `top_k` most
    co-visible frames anywhere in the sequence (loop closures), from the rig poses."""
    n = len(frames)
    pairs = {(i, j): 0 for i in range(n) for j in range(i + 1, min(n, i + window + 1))}
    if cov is not None and top_k > 0:
        from kinetix.core.types import Pose
        vis = [cov.visible(Pose(np.asarray(f["pose"]["p"], dtype=float), np.asarray(f["pose"]["q"], dtype=float)))
               for f in frames]
        M = np.array([v[0] for v in vis], dtype=np.float32)                   # (n, surface points) visibility
        shared = M @ M.T
        np.fill_diagonal(shared, 0)
        for i in range(n):
            for j in np.argsort(-shared[i])[:top_k]:
                if shared[i, j] >= min_shared and abs(int(j) - i) > window:
                    a, b = sorted((i, int(j)))
                    pairs[(a, b)] = int(shared[i, j])
    return [(frames[a]["image"], frames[b]["image"], v) for (a, b), v in sorted(pairs.items())]


def write_position_priors(db_path: Path, frames: list[dict], sigma_m: float) -> int:
    """One CARTESIAN position prior per image: the rig camera centre (frames.jsonl `pose`, EKF2), covariance sigma^2 I."""
    import pycolmap
    by_name = {f["image"]: f for f in frames}
    db = pycolmap.Database.open(str(db_path))
    n = 0
    for img in db.read_all_images():
        f = by_name.get(img.name)
        if f is None:
            continue
        prior = pycolmap.PosePrior(position=np.asarray(f["pose"]["p"], dtype=float),
                                   position_covariance=np.eye(3) * sigma_m ** 2,
                                   coordinate_system=pycolmap.PosePriorCoordinateSystem.CARTESIAN,
                                   corr_data_id=img.data_id)
        db.write_pose_prior(prior)
        n += 1
    db.close()
    return n


def known_pose_model(db_path: Path, image_root: Path, out: Path, frames: list[dict], opts, ba_rounds: int = 2):
    """pose_source=prior (architecture.md §6): cameras at the rig poses (EKF2), points triangulated from the verified matches,
    then `ba_rounds` x (bundle adjustment with fixed intrinsics + re-triangulation). Registration cannot alias across
    repeated bays because no pose is searched for; BA starts near the truth and only refines."""
    import pycolmap

    from kinetix.core.geometry import quat_to_R
    by_name = {f["image"]: f for f in frames}
    db = pycolmap.Database.open(str(db_path))
    rec = pycolmap.Reconstruction()
    for cam in db.read_all_cameras():
        rec.add_camera_with_trivial_rig(cam)
    for img in db.read_all_images():
        f = by_name.get(img.name)
        if f is None:
            continue
        R = quat_to_R(f["pose"]["q"])                                   # T_map_cam rotation (OpenCV optical)
        c = np.asarray(f["pose"]["p"], dtype=float)
        cfw = pycolmap.Rigid3d(pycolmap.Rotation3d(R.T), -R.T @ c)       # cam_from_world
        kp = np.asarray(db.read_keypoints(img.image_id))[:, :2].astype(float)
        rec.add_image_with_trivial_frame(pycolmap.Image(name=img.name, keypoints=kp, camera_id=img.camera_id,
                                                        image_id=img.image_id), cfw)
    db.close()
    tri = out / "tri"
    tri.mkdir(parents=True, exist_ok=True)
    rec = pycolmap.triangulate_points(rec, db_path, image_root, tri, clear_points=True, options=opts)
    ba = pycolmap.BundleAdjustmentOptions()
    ba.refine_focal_length = False
    ba.refine_principal_point = False
    ba.refine_extra_params = False
    for _ in range(ba_rounds):
        pycolmap.bundle_adjustment(rec, ba)
        rec = pycolmap.triangulate_points(rec, db_path, image_root, tri, clear_points=False, options=opts)
    return rec


def _coverage_for(head: dict, scene_file: Path) -> CoverageState:
    scene = load_scene(scene_file)
    cfg = head["config"]
    cam = cfg["camera"]
    return CoverageState.for_scene(scene, Intrinsics(**{k: cam[k] for k in ("width", "height", "fx", "fy", "cx", "cy")}),
                                   cfg["coverage"])


def run_sfm(run: Path | str, scene_path: Path | str | None = None, max_features: int = 8192, voxel: float = 0.25,
            threads: int = -1, init_min_tri_angle: float = 4.0, init_min_num_inliers: int = 50, pairing: str | None = None,
            min_shared: int = 10, rig_check: bool = True, prior_sigma_m: float | None = 0.5,
            pose_source: str | None = None) -> dict:
    """COLMAP's default initial pair needs >= 16 deg of triangulation angle and 100 inliers. In a room most matches lie on
    walls 5-12 m away, where views a metre or two apart give a few degrees, so the defaults here are 4 deg / 50."""
    import pycolmap

    run = Path(run)
    head = yaml.safe_load((run / "run.yaml").read_text())
    frames = [f for f in _frames(run) if f.get("image")]
    if len(frames) < 3:
        raise SystemExit(f"{run}: {len(frames)} frames with images, need >= 3")
    out = run / "sfm" / "offline"
    if out.exists():
        shutil.rmtree(out)
    (out / "sparse").mkdir(parents=True)
    db = out / "database.db"
    K = frames[0]["K"]
    names = [f["image"] for f in frames]                  # "frames/000003.png", relative to the run dir
    t0 = time.time()

    reader = pycolmap.ImageReaderOptions()
    reader.camera_model = "PINHOLE"
    reader.camera_params = f"{K['fx']},{K['fy']},{K['cx']},{K['cy']}"
    fopt = pycolmap.FeatureExtractionOptions()
    fopt.sift.max_num_features = int(max_features)
    fopt.num_threads = threads
    pycolmap.extract_features(db, run, image_names=names, camera_mode=pycolmap.CameraMode.SINGLE, reader_options=reader,
                              extraction_options=fopt, device=pycolmap.Device.cpu)
    t_feat = time.time() - t0
    scene_file = Path(scene_path) if scene_path else _scene_from_run(head)
    pairing = pairing or ("video" if len(frames) > 150 else "covis" if scene_file is not None else "exhaustive")
    pose_source = pose_source or ("prior" if head.get("method") == "tour" or len(frames) > 150 else "sfm")
    if pose_source not in ("prior", "sfm"):
        raise SystemExit(f"pose_source {pose_source!r} (sfm | prior)")
    vopt = pycolmap.TwoViewGeometryOptions()
    vopt.compute_relative_pose = True                     # cam2_from_cam1 for the rig check
    n_pairs = len(names) * (len(names) - 1) // 2
    if pairing in ("covis", "video"):
        if pairing == "covis" and scene_file is None:
            raise SystemExit("pairing covis needs the scene (--scene)")
        cov = _coverage_for(head, scene_file) if scene_file is not None else None
        pairs = covis_pairs(frames, cov, min_shared) if pairing == "covis" else video_pairs(frames, cov)
        (out / "pairs.txt").write_text("".join(f"{a} {b}\n" for a, b, _ in pairs))
        popt = pycolmap.ImportedPairingOptions()
        popt.match_list_path = str(out / "pairs.txt")
        pycolmap.match_image_pairs(db, pairing_options=popt, verification_options=vopt, device=pycolmap.Device.cpu)
        n_pairs = len(pairs)
    elif pairing == "exhaustive":
        pycolmap.match_exhaustive(db, verification_options=vopt, device=pycolmap.Device.cpu)
    else:
        raise SystemExit(f"pairing {pairing!r} (exhaustive | covis | video)")
    checked = rig_check_pairs(db, frames) if rig_check else None
    n_priors = 0                                         # set below for pose_source=sfm
    t_match = time.time() - t0 - t_feat
    opts = pycolmap.IncrementalPipelineOptions()
    opts.ba_refine_focal_length = False
    opts.ba_refine_principal_point = False
    opts.ba_refine_extra_params = False
    opts.num_threads = threads
    opts.mapper.init_min_tri_angle = float(init_min_tri_angle)
    opts.mapper.init_min_num_inliers = int(init_min_num_inliers)
    opts.min_model_size = 3
    if pose_source == "sfm" and prior_sigma_m:
        n_priors = write_position_priors(db, frames, prior_sigma_m)
    if n_priors:
        opts.use_prior_position = True
        opts.use_robust_loss_on_prior_position = False   # robust: a model metres off treats its priors as outliers
    if pose_source == "prior":
        models = {0: known_pose_model(db, run, out, frames, opts)}
    else:
        models = pycolmap.incremental_mapping(db, run, out / "sparse", options=opts)
    t_map = time.time() - t0 - t_feat - t_match
    if not models:
        res = {"n_frames": len(frames), "registered": 0, "registered_frac": 0.0, "models": 0,
               "time_s": {"features": round(t_feat, 1), "matching": round(t_match, 1), "mapping": round(t_map, 1)}}
        (out / "metrics.json").write_text(json.dumps(res, indent=1))
        return res
    k_best, rec = max(models.items(), key=lambda kv: kv[1].num_reg_images())
    (out / "sparse" / str(k_best)).mkdir(parents=True, exist_ok=True)
    rec.write(out / "sparse" / str(k_best))

    by_name = {f["image"]: f for f in frames}
    sfm_c, ekf_c, gt_c = [], [], []
    for img in rec.images.values():
        f = by_name.get(img.name)
        if f is None or not img.has_pose:
            continue
        sfm_c.append(np.asarray(img.projection_center()))
        ekf_c.append(f["pose"]["p"])
        gt_c.append(f["gt_pose"]["p"] if f.get("gt_pose") else None)
    sfm_c, ekf_c = np.array(sfm_c), np.array(ekf_c)
    pts = np.array([p.xyz for p in rec.points3D.values()]) if rec.num_points3D() else np.zeros((0, 3))
    res = {"n_frames": len(frames), "pose_source": pose_source, "pairing": pairing, "pairs": n_pairs, "rig_check": checked,
           "position_priors": {"n": n_priors, "sigma_m": prior_sigma_m} if n_priors else None,
           "registered": int(rec.num_reg_images()),
           "registered_frac": round(rec.num_reg_images() / len(frames), 3), "models": len(models),
           "models_sizes": sorted((m.num_reg_images() for m in models.values()), reverse=True),
           "points3D": int(rec.num_points3D()), "mean_track_length": round(float(rec.compute_mean_track_length()), 2),
           "mean_reproj_error_px": round(float(rec.compute_mean_reprojection_error()), 3),
           "time_s": {"features": round(t_feat, 1), "matching": round(t_match, 1), "mapping": round(t_map, 1)}}
    if len(sfm_c) >= 3:
        res["ate_vs_ekf2"] = {k: round(v, 4) if isinstance(v, float) else v for k, v in ate(sfm_c, ekf_c).items()}
        if all(g is not None for g in gt_c):
            res["ate_vs_gt"] = {k: round(v, 4) if isinstance(v, float) else v for k, v in ate(sfm_c, np.array(gt_c)).items()}
            res["ekf2_vs_gt_m"] = round(float(np.sqrt((np.linalg.norm(ekf_c - np.array(gt_c), axis=1) ** 2).mean())), 4)
        s, R, t = umeyama(sfm_c, ekf_c)
        pts_map = apply(s, R, t, pts)
        _write_ply(out / "points.ply", pts_map, rec)
        if scene_file is not None and len(pts_map):
            scene = load_scene(scene_file)
            cfg = head["config"]
            cov = _coverage_for(head, scene_file)
            n_room = None
            if scene.kind == "room":
                from kinetix.recon.coverage import box_surface
                n_room = int(len(box_surface(scene.roi, cfg["coverage"]["spacing_m"], bottom=True)[0]))
                n_room -= _dropped_floor(scene, cfg["coverage"]["spacing_m"])
            near = _near(pts_map, cov.P, voxel)
            comp = {"all": round(float(near.mean()), 4)}
            for name, m in _surface_classes(cov.P, cov.N, n_room).items():
                if m.any():
                    comp[name] = round(float(near[m].mean()), 4)
            res["completeness_proxy"] = {**comp, "voxel_m": voxel, "surface_points": int(len(cov.P)), "scene": str(scene_file)}
    res["time_s"]["total"] = round(time.time() - t0, 1)
    (out / "metrics.json").write_text(json.dumps(res, indent=1))
    return res


def _dropped_floor(scene, spacing: float) -> int:
    """Room-face points removed by room_surface() because they lie inside an obstacle (keeps the class split aligned)."""
    from kinetix.recon.coverage import box_surface
    P, _ = box_surface(scene.roi, spacing, bottom=True, inward=True)
    keep = np.ones(len(P), dtype=bool)
    for b in scene.no_fly:
        keep &= ~np.all((P >= b.lo - 1e-6) & (P <= b.hi + 1e-6), axis=1)
    return int((~keep).sum())


def _scene_from_run(head: dict) -> Path | None:
    from kinetix.core.config import ROOT
    p = ROOT / "config" / "scenes" / f"{head.get('scene')}.yaml"
    return p if p.exists() else None


def _write_ply(path: Path, pts: np.ndarray, rec) -> None:
    """ASCII PLY of the sparse points in `map` (after the EKF2 Sim3), with colours."""
    cols = np.array([p.color for p in rec.points3D.values()], dtype=int) if len(pts) else np.zeros((0, 3), dtype=int)
    with open(path, "w") as fh:
        fh.write(f"ply\nformat ascii 1.0\nelement vertex {len(pts)}\nproperty float x\nproperty float y\nproperty float z\n"
                 "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
        for (x, y, z), (r, g, b) in zip(pts, cols):
            fh.write(f"{x:.4f} {y:.4f} {z:.4f} {r} {g} {b}\n")
