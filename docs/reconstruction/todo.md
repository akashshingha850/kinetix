# Todo: reconstruction

Owner: author B ([collaboration.md](../collaboration.md)). Code in `reconstruction/`, results in `data/<run>/<method>/metrics.json`, compared
on the frozen benchmark runs listed in collaboration.md. Acquisition work is in [acquisition/todo.md](../acquisition/todo.md); its flight notes hold the SfM numbers so far.

## Now
- [x] `reconstruction/metrics.py` + `evaluate.py` + `kinetix eval` (2026-11-01, built by acquisition for the K2 bench; review): sparse
  model Sim3-aligned to the GT centres, cropped to the ROI, accuracy / completeness / Chamfer / F@0.5 and 1 % of D, `eval_valid`
  guards; `evaluate_prefix` maps the first N frames from the same database (bench budgets). Sanity tests in `tests/test_metrics.py`
- [ ] `sfm.py`: cameras with too few points get moved far off their prior pose by bundle adjustment. Hold them at their prior pose,
  or drop them. Seen on every explore + tour flight: `123651` (900012/18, max 17 m), `134802` (900016/25/03, 9.7 m), `163032`
  (900004, 7 points, 20.6 m). Without them, ATE vs GT RMSE is 0.14-0.16 m.
- [x] `reconstruction/densify.py` (2026-11-01, built by acquisition for the V1 check of docs/methodology.md; review): COLMAP
  patch-match + fusion in `colmap/colmap:20260907.8077` (CUDA; `docker run --gpus`, host uid), `evaluate(..., densify=...)` and
  `evaluate_prefix` write `dense/` and `dense_n<N>/`. Buddha `orbit_rings`, 80 views, same sparse model: sparse F@1 % 0.615
  (9 k points, completeness 37 mm); COLMAP defaults at 1280 px 0.916 (1.04 M points, 13 mm, **25 min**); bench preset 640 px,
  8 source images, photometric 0.895 (289 k, 20 mm, 4.2 min); 640 + geometric 0.879 (9 min, no better); 512 px, 6 sources, 3
  iterations 0.876 (2.2 min). gsplat backend not built.
- [ ] Dense reconstruction (K2 `reconstruction/densify.py`: COLMAP MVS or gsplat) for a real completeness figure. `completeness_proxy`
  from the sparse cloud is a lower bound (0.41 on `163032`, 0.55 on the 2026-10-18 tour).
- [ ] Ground truth for the Warehouse: a mesh from the bisg USD (the emptied room), so accuracy and completeness are measured
  cloud-to-mesh, not against the room box. 16.5 % of `134802`'s points lie > 0.25 m outside the true box (most beyond x+), not looked into.
- [ ] Depth-aided reconstruction: the ZED depth of every keyframe is recorded (`depth/<idx>.npy`) and not used yet. Options: depth
  fusion (TSDF), depth priors in MVS / 3DGS (DN-Splatter style), scale. Plain orange wall panels read half invalid in the ZED depth (`134802`, frame 900024).
- [ ] Mono vs stereo as reconstruction input (gap 3 in the scoping review): the same frames with and without the ZED depth / right image.
- [ ] One `metrics.json` schema for every method (registered fraction, ATE vs GT, reprojection, accuracy / completeness vs GT mesh,
  F-score at fixed thresholds, time), so the paper tables come from one script.
