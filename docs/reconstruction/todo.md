# Todo: reconstruction

Owner: author B ([collaboration.md](../collaboration.md)). Code in `reconstruction/`, results in `results/<run>/<method>/metrics.json`, compared
on the frozen benchmark runs listed in collaboration.md. Acquisition work is in [acquisition/todo.md](../acquisition/todo.md); its flight notes hold the SfM numbers so far.

## Now
- [ ] `sfm.py`: cameras with too few points get moved far off their prior pose by bundle adjustment. Hold them at their prior pose,
  or drop them. Seen on every explore + tour flight: `123651` (900012/18, max 17 m), `134802` (900016/25/03, 9.7 m), `163032`
  (900004, 7 points, 20.6 m). Without them, ATE vs GT RMSE is 0.14-0.16 m.
- [ ] Dense reconstruction (K2 `reconstruction/densify.py`: COLMAP MVS or gsplat) for a real completeness figure. `completeness_proxy`
  from the sparse cloud is a lower bound (0.41 on `163032`, 0.55 on the 2026-10-18 tour).
- [ ] Ground truth for the Warehouse: a mesh from the bisg USD (the emptied room), so accuracy and completeness are measured
  cloud-to-mesh, not against the room box. 16.5 % of `134802`'s points lie > 0.25 m outside the true box (most beyond x+), not looked into.
- [ ] Depth-aided reconstruction: the ZED depth of every keyframe is recorded (`depth/<idx>.npy`) and not used yet. Options: depth
  fusion (TSDF), depth priors in MVS / 3DGS (DN-Splatter style), scale. Plain orange wall panels read half invalid in the ZED depth (`134802`, frame 900024).
- [ ] Mono vs stereo as reconstruction input (gap 3 in the scoping review): the same frames with and without the ZED depth / right image.
- [ ] One `metrics.json` schema for every method (registered fraction, ATE vs GT, reprojection, accuracy / completeness vs GT mesh,
  F-score at fixed thresholds, time), so the paper tables come from one script.
