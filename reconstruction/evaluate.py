"""`kinetix eval <run>`: the run's SfM model scored against the scene's GT (paper/benchmark.md §3). Writes <sfm dir>/eval.json.

The model is Sim3-aligned to the frames' GT camera centres (`gt_pose`; T0/T1: the exact render poses, T2: Pegasus GT in this
run's map), cropped to the ROI box and compared with gt_mesh.ply (accuracy) and gt_observable (completeness), both moved by the
run's `map_shift`. Points: the sparse SfM points, or with `densify` (a dict: backend `colmap_mvs` + its options,
reconstruction/densify.py) the fused dense points of the same model, written to <run>/dense/ and <run>/dense_n<N>/.
`evaluate_prefix` re-maps only the first N frames from the run's feature/match database (the bench's budget curve: a greedy
session's first N views are what a budget-N session takes)."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import yaml

from kinetix.align import apply, ate, umeyama
from kinetix.config import load_scene
from reconstruction.metrics import score


def _frames(run: Path) -> list[dict]:
    return [json.loads(line) for line in (run / "frames.jsonl").read_text().splitlines() if line.strip()]


def scene_file_of(run: Path, scene: Path | str | None) -> Path:
    if scene is not None:
        return Path(scene)
    from reconstruction.sfm import _scene_from_run
    head = yaml.safe_load((run / "run.yaml").read_text())
    path = _scene_from_run(head)
    if path is None:
        raise SystemExit(f"{run}: scene unknown, pass --scene")
    return Path(path)


def best_model(sparse: Path):
    import pycolmap
    models = [pycolmap.Reconstruction(str(d)) for d in sorted(sparse.iterdir()) if (d / "images.bin").exists()]
    return max(models, key=lambda m: m.num_reg_images()) if models else None


def dense_points(rec, run: Path, work: Path, densify: dict) -> np.ndarray:
    from reconstruction.densify import colmap_mvs
    opts = {k: v for k, v in densify.items() if k != "backend"}
    if densify.get("backend", "colmap_mvs") != "colmap_mvs":
        raise SystemExit(f"densify backend {densify['backend']!r}: only colmap_mvs")
    return colmap_mvs(rec, run, work, **opts)


def evaluate_model(rec, frames: list[dict], scene_file: Path, shift, n_images: int, points=None, densify: str = "sparse") -> dict:
    """`points` (model frame) replace the model's sparse points, e.g. dense_points()."""
    scene = load_scene(scene_file)
    if scene.gt_mesh is None:
        raise SystemExit(f"{scene_file}: no gt_mesh")
    import trimesh
    shift = np.asarray(shift, float)
    gt = trimesh.load(scene.gt_mesh, process=False)
    obs_file = scene.gt_mesh.parent / "gt_observable.npz"
    obs = np.load(obs_file)["P"].astype(float) + shift if obs_file.exists() else np.asarray(gt.vertices) + shift
    lo, hi = scene.roi.lo + shift, scene.roi.hi + shift
    diag = float(np.linalg.norm(scene.roi.size))
    reg = rec.num_reg_images() if rec is not None else 0
    out = {"n_images": n_images, "registered": reg, "registered_frac": round(reg / max(n_images, 1), 4),
           "eval": {"densify": densify, "align": "sim3_gt_centres", "gt": obs_file.name, "scene": str(scene_file)}}
    by_name = {f["image"]: f for f in frames}
    pairs = [(np.asarray(img.projection_center()), by_name[img.name]["gt_pose"]["p"]) for img in (rec.images.values() if rec else [])
             if img.has_pose and img.name in by_name and by_name[img.name].get("gt_pose")]
    if len(pairs) < 3:
        return {**out, **score(np.zeros((0, 3)), gt.vertices, gt.faces, obs, diag), "eval_valid": False}
    S, G = np.array([a for a, _ in pairs]), np.array([b for _, b in pairs], float)
    s, R, t = umeyama(S, G)
    if points is None:
        points = np.array([p.xyz for p in rec.points3D.values()]) if rec.num_points3D() else np.zeros((0, 3))
    pts = apply(s, R, t, points) if len(points) else np.zeros((0, 3))
    pts = pts[np.all((pts >= lo) & (pts <= hi), axis=1)]
    a = ate(S, G)
    out["ate_cm"] = round(100 * a["rmse_m"], 3)
    out.update(score(pts, np.asarray(gt.vertices) + shift, gt.faces, obs, diag))
    out["eval_valid"] = bool(out["registered_frac"] >= 0.5 and a["rmse_m"] <= 0.05 * diag)   # benchmark.md §3 sanity guards
    return out


def evaluate(run: Path | str, scene: Path | str | None = None, sfm_dir: str = "sfm", densify: dict | None = None) -> dict:
    """Score `<run>/<sfm_dir>` (kinetix sfm's output; run it first if missing); with `densify`: its dense points -> <run>/dense/."""
    run = Path(run)
    head = yaml.safe_load((run / "run.yaml").read_text())
    frames = [f for f in _frames(run) if f.get("image")]
    scene_file = scene_file_of(run, scene)
    if not (run / sfm_dir / "metrics.json").exists():
        from reconstruction.sfm import run_sfm
        run_sfm(run, scene_file)
    rec = best_model(run / sfm_dir / "sparse")
    out = run / ("dense" if densify else sfm_dir)
    pts = dense_points(rec, run, out, densify) if densify and rec is not None else None
    res = evaluate_model(rec, frames, scene_file, (head.get("summary") or {}).get("map_shift", [0, 0, 0]), len(frames), pts,
                         densify["backend"] if densify else "sparse")
    (out / "eval.json").write_text(json.dumps(res, indent=1))
    return res


def evaluate_prefix(run: Path | str, n: int, scene: Path | str | None = None, densify: dict | None = None) -> dict:
    """Map only the first `n` frames, reusing <run>/sfm/database.db (features, matches, pose priors), then score: <run>/sfm_n<n>/
    (with `densify`: <run>/dense_n<n>/)."""
    import shutil

    import pycolmap

    from reconstruction.sfm import mapper_options
    run = Path(run)
    head = yaml.safe_load((run / "run.yaml").read_text())
    frames = [f for f in _frames(run) if f.get("image")][:n]
    sfm = json.loads((run / "sfm" / "metrics.json").read_text())
    if sfm.get("pose_source") != "sfm":
        raise SystemExit(f"{run}: prefixes need pose_source sfm (incremental mapping), not {sfm.get('pose_source')}")
    out = run / (f"dense_n{n}" if densify else f"sfm_n{n}")
    if out.exists():
        shutil.rmtree(out)
    (out / "sparse").mkdir(parents=True)
    opts = mapper_options()
    opts.image_names = [f["image"] for f in frames]
    if sfm.get("position_priors"):
        opts.use_prior_position = True
        opts.use_robust_loss_on_prior_position = False
    models = pycolmap.incremental_mapping(run / "sfm" / "database.db", run, out / "sparse", options=opts)
    rec = max(models.values(), key=lambda m: m.num_reg_images()) if models else None
    pts = dense_points(rec, run, out / "mvs", densify) if densify and rec is not None else None
    res = evaluate_model(rec, frames, scene_file_of(run, scene), (head.get("summary") or {}).get("map_shift", [0, 0, 0]), len(frames),
                         pts, densify["backend"] if densify else "sparse")
    (out / "eval.json").write_text(json.dumps(res, indent=1))
    shutil.rmtree(out / "sparse")                      # the numbers are what the bench needs; models of every prefix are large
    return res
