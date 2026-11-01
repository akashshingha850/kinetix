"""V5a (methodology.md §4): how good is SfM-anchored monocular depth on a pool, against the pool's GT depth?

For every registered image of a run's SfM model (Sim3-aligned to the GT camera centres), Depth Anything V2 relative depth is anchored
to the SfM points that image observes (`mono`), compared with the metric indoor model used as is (`mono_metric_noanchor`) and with
the relative depth anchored to GT depth at the same pixels (`affine_gt`: the best an affine fit of this network can do). Error =
AbsRel on ROI pixels (GT depth back-projected inside the ROI box). Not a pytest test (needs a pool, a run and the `ml` extra):
`uv run python -m acquisition.tests.anchor_bench <run dir> [--every 4]`."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from acquisition.mapping.monodepth import METRIC_INDOOR, RELATIVE, MonoDepth, anchor, to_depth
from kinetix.align import apply, umeyama
from kinetix.config import load_scene
from kinetix.geometry import quat_to_R
from reconstruction.evaluate import best_model, scene_file_of


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run", type=Path)
    ap.add_argument("--every", type=int, default=4, help="evaluate every k-th registered image")
    a = ap.parse_args(argv)
    run = a.run
    frames = {f["image"]: f for f in map(json.loads, (run / "frames.jsonl").read_text().splitlines()) if f.get("image")}
    scene = load_scene(scene_file_of(run, None))
    rec = best_model(run / "sfm" / "sparse")
    imgs = [im for im in rec.images.values() if im.has_pose and im.name in frames]
    S = np.array([im.projection_center() for im in imgs])
    G = np.array([frames[im.name]["gt_pose"]["p"] for im in imgs], float)
    s, R, t = umeyama(S, G)
    X = {pid: apply(s, R, t, p.xyz[None])[0] for pid, p in rec.points3D.items()}
    lo, hi = scene.roi.lo - 0.05, scene.roi.hi + 0.05
    rel_net, met_net = MonoDepth(RELATIVE), MonoDepth(METRIC_INDOOR)
    rows = []
    for im in imgs[::a.every]:
        f = frames[im.name]
        K = f["K"]
        Rw, p = quat_to_R(f["gt_pose"]["q"]), np.array(f["gt_pose"]["p"], float)
        gt = np.load(_pool_depth(run, f))
        rgb = cv2.cvtColor(cv2.imread(str(run / im.name)), cv2.COLOR_BGR2RGB)
        h, w = gt.shape
        uu, vv = np.meshgrid(np.arange(w), np.arange(h))
        P = np.stack([(uu - K["cx"]) / K["fx"] * gt, (vv - K["cy"]) / K["fy"] * gt, gt], -1) @ Rw.T + p
        roi = np.isfinite(gt) & np.all((P >= lo) & (P <= hi), axis=-1)
        if roi.sum() < 500:
            continue
        ids = [p2.point3D_id for p2 in im.points2D if p2.has_point3D()]
        Xc = (np.array([X[i] for i in ids]) - p) @ Rw if ids else np.zeros((0, 3))
        Xc = Xc[Xc[:, 2] > 0.1]
        u, v = K["fx"] * Xc[:, 0] / Xc[:, 2] + K["cx"], K["fy"] * Xc[:, 1] / Xc[:, 2] + K["cy"]
        ok = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        u, v, z = u[ok], v[ok], Xc[ok, 2]
        rel, met = rel_net(rgb), met_net(rgb)
        row = {"image": im.name, "pool_id": f.get("pool_id"), "anchors": int(len(z)), "roi_px": int(roi.sum())}
        fit = anchor(rel, u, v, z)
        if fit:
            row["mono"] = _absrel(to_depth(rel, fit[0], fit[1]), gt, roi)
            row["mono_inliers"] = fit[2]
        row["mono_metric_noanchor"] = _absrel(met, gt, roi)
        k = np.flatnonzero(roi.ravel())[:: max(1, int(roi.sum()) // 2000)]
        g = anchor(rel, uu.ravel()[k], vv.ravel()[k], gt.ravel()[k])
        row["affine_gt"] = _absrel(to_depth(rel, g[0], g[1]), gt, roi) if g else None
        rows.append(row)
        print(row, flush=True)
    summ = {k: round(float(np.median([r[k] for r in rows if r.get(k) is not None])), 4)
            for k in ("mono", "mono_metric_noanchor", "affine_gt")}
    summ.update(images=len(rows), anchors_median=int(np.median([r["anchors"] for r in rows])),
                mono_fitted=sum("mono" in r for r in rows))
    (run / "anchor_bench.json").write_text(json.dumps({"summary": summ, "rows": rows}, indent=1))
    print(json.dumps(summ, indent=1))


def _pool_depth(run: Path, f: dict) -> Path:
    import yaml
    head = yaml.safe_load((run / "run.yaml").read_text())
    from kinetix.cli import default_pool
    pool = default_pool(head["scene_path"])          # the run's pool rig read the scene's default pool
    return pool / "depth" / f"{f['pool_id']}.npy"


def _absrel(d: np.ndarray, gt: np.ndarray, m: np.ndarray) -> float | None:
    m = m & np.isfinite(d)
    return round(float(np.median(np.abs(d[m] - gt[m]) / gt[m])), 4) if m.sum() > 100 else None


if __name__ == "__main__":
    main()
