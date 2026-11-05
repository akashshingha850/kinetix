# Reconstruction pipeline

What is made from a flight and how it is scored. Code in `reconstruction/`, todo in [todo.md](todo.md). Input: a run directory
`data/<run>/` ([data-format.md](../data-format.md) §3, read only). Output: `data/<run>/<method>/` with a `metrics.json`
(metrics: [paper/benchmark.md](../paper/benchmark.md) §3). Ownership and the frozen benchmark runs: [collaboration.md](../collaboration.md).

## Offline pipeline (identical for every method)

To keep the comparison fair, the online model is only for planning. Every method's final model is
rebuilt the same way from its captured frames alone:

1. `reconstruction/sfm.py`: pycolmap SfM over all frames (`sfm` or `prior` poses; rig SfM when
   `recon_input` is stereo).
2. Sim3-align to GT camera centres (sim) or to the rig poses (real).
3. `reconstruction/densify.py`: `colmap_mvs` (CUDA COLMAP in Docker: the pip pycolmap wheel is CPU-only)
   or `gsplat` (3DGS, then TSDF-fused rendered depth → mesh), or `tsdf_depth` (stereo modes only:
   TSDF fusion of the stored stereo depth at SfM poses).
4. `reconstruction/evaluate.py`: crop to the ROI, compare with the **observable** GT surface, write
   `metrics.json` ([paper/benchmark.md](../paper/benchmark.md) §3).

