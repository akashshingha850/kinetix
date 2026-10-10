# Benchmark protocol

What is compared, how it is measured, and which result backs each paper contribution C1–C5
([publication-plan.md](publication-plan.md)). The image saving is a *reported result*, measured against
the strongest baseline. It is not the headline claim, because ~53–72 % savings are already published
([research/reports/Kinetix novelty search.md](research/reports/Kinetix%20novelty%20search.md)). The harness is
`kinetix bench` ([interfaces.md](interfaces.md) §6), and the outputs are in [data-format.md](data-format.md) §4.

## 1. Experimental axes

| Axis | Values |
|---|---|
| Scenes | K2: 3 sim objects. Paper: 8–10 sim objects spanning texture (rich/poor), geometry (convex/concave/thin parts), size (0.3–2 m), material (matte/glossy), **at least 3 drawn from Objaverse++ / GSO / OmniObject3D** so the set overlaps community pools (ObjView-Bench), plus 3 real objects (K6). Asset licences are checked before the set is fixed |
| Pools | `rings:4x60` (main) and **`tammes:128`** (ObjView-Bench-compatible 128-view sphere with drone reachability masks: nothing below the floor or inside walls), each with a held-out test split |
| Methods | **fixed patterns:** `orbit_1ring`, `orbit_3ring` (dense multi-ring at matched GSD, the one that counts), `grid_hemisphere` (CPP-style) · **simple adaptive:** `random`, `fvs` (farthest-view sampling, strong per NeRF Director) · **classical NBV:** `volumetric_ig` (unobserved/rear-side voxel entropy on the same TSDF, Isler/Delmerico family) · **radiance-field:** `fisherrf` (RGB-only, offline over the pool) · **learned:** `ma_scvp` (public code, pool-native; `gennbv` as an alternative) · `explore_exploit` (orbit seed, then weak-region coverage; if K3 time allows) · `coverage_oracle` (upper bound, uses GT) · `kinetix_combo` + ablations. Hestia / VIN-NBV / ActMVS only if their code runs |
| Sensing (ADR-K07) | `mono` (Kinetix), `mono_noprior`, `stereo_plan`, `stereo_full`, `oracle_depth`. Baselines run with `left` and `stereo_rig` reconstruction input, so a stereo baseline exists for every stereo Kinetix row. `zed_mapping` (ZED SDK spatial-mapping mesh on an orbit) on T2/T3 only |
| Budgets | images N ∈ {5, 10, 20, 30, 40, 80} (includes ObjView-Bench's K=5/30) and `stop: gain_plateau` (free mode) |
| Tier | T0 for the full matrix. A T1 and T2 subset for transfer. T3 for real |
| Seeds | 3 (K2–K3), 5 (paper). The seed controls the seed-view phase, random tie-breaks and pose noise |

Fixed-sequence baselines take the first N poses of an evenly spread sequence (a golden-angle
ordering on the rings), so every prefix is a fair N-image baseline.

## 2. Reference and headline metric

- **Dense reference** per scene: every candidate view of the pool (~200+), reconstructed with the same
  offline pipeline. Its F-score@1 % is `F_ref`.
- **Headline metric: images-to-quality** `N@q` (contribution C3) = the smallest N at which a method reaches `q · F_ref`,
  interpolated on the budget curve, reported for **q ∈ {0.9, 0.95}**. Image reduction = `1 − N@q(kinetix) / N@q(baseline)`,
  reported against the **best fixed pattern and the best adaptive baseline**, never the weakest.
- **N@q sensitivity**: `F_ref` depends on the pool's density, so N@q is also reported against a 120-, 240- and
  360-view dense reference.
- **Comparability with prior tables**: coverage AUC over the image sequence (SCONE/GenNBV convention), and
  F-score / coverage at fixed budgets K = 5 and K = 30 (ObjView-Bench).
- **Efficiency curve**: F-score@1 % vs N per method (mean ± std over seeds), one panel per scene.
- **Mono vs stereo** ([research/mono-vs-stereo.md](research/mono-vs-stereo.md) RQ1–RQ5):
  - planning value: `N@q(mono)` vs `N@q(stereo_plan)` (same `left` reconstruction)
  - reconstruction value at equal N: `stereo_plan` vs `stereo_full`
  - gap closure of the mono prior: `(N@q(mono_noprior) − N@q(mono)) / (N@q(mono_noprior) − N@q(stereo_plan))`
  - per scene category (texture-poor, glossy, thin parts) for RQ4
- **Tier transfer (C4)**: Kendall's τ between method rankings (by N@q) at T0 and at T1, T2 and T3, plus
  ΔN@q per tier for each method.

## 3. Metrics (`offline/metrics.py`)

All geometric metrics are computed after Sim3 alignment of the reconstruction to GT camera centres,
cropped to the ROI box, against `gt_observable.ply` (GT surface visible from some feasible pose).
Thresholds are relative to the ROI diagonal `D`, so objects of different sizes are comparable.

| Metric | Definition |
|---|---|
| accuracy | mean distance recon → GT (mm) |
| completeness | mean distance GT → recon (mm) |
| Chamfer | (accuracy + completeness) / 2 |
| F-score@τ | harmonic mean of precision/recall at τ ∈ {0.5 %, 1 %} of D. Real objects additionally use one absolute τ in mm |
| coverage AUC | area under GT-surface coverage vs image index, normalised to [0, 1] |
| Kendall's τ (tiers) | rank correlation of methods between two tiers (C4) |
| completeness ratio@τ | fraction of GT points within τ of the recon |
| registered fraction | registered frames / captured frames |
| ATE | RMSE of camera centres after Sim3 (cm) |
| NVS | PSNR / SSIM / LPIPS on the pool's `test` views (3DGS densifier only) |
| cost | images, travel distance (m), flight time (T2/T3, s), planning time per step (s) |

Sanity guards are run by `evaluate.py` on every evaluation: if alignment residual > 5 % of D, or
registered fraction < 50 %, the run is marked `eval_valid: false` and kept out of means (reported as failure count).

## 4. Ablations (K3+)

| Ablation | Values |
|---|---|
| scorer terms | sfm_uncertainty only · depth_frontier only · combo |
| pose source | `sfm` · `prior` · `gt` |
| travel cost λ | 0 · default · 4× default |
| seed views | 3 · 6 · 12 |
| camera mount pitch | 0° · −20° · −35° (Q1) |
| mono depth model | DA-V2 small · base · large (scale-fit) |
| **depth anchoring (C1, decisive)** | `mono` (DA-V2 anchored to SfM) · `mono_metric_noanchor` (Metric3D v2 / Depth Pro used directly, FrontierNet-style) · `mono_stereo_anchor` (DA-V2 scale-fit to ZED depth, Next Best Sense-style) · `none`. If SfM anchoring does not beat both alternatives, C1 is engineering, not a contribution |
| next-gen depth (optional) | Depth Anything 3 or VGGT multi-view depth with drone-pose scale |
| stereo depth source (T0) | ZED SDK via the twin (pool `zed/`, default, ADR-K10) · FoundationStereo · SGBM · GT + fitted ZED noise `σ(z)` (RQ5, Q6) |
| pose noise (T0) | 0 · 2 cm/1° · 5 cm/2° (emulates EKF2 error) |

## 5. Reporting rules

- Every number in the paper comes from a `results.csv` produced by `kinetix bench`. No hand-edited tables.
- Real flights (T3) report Chamfer, F-score and N@q against scanned GT, not pictures. The minimum real
  method set is `orbit_3ring`, `fvs`, `kinetix_combo` (mono) and `stereo_plan`, so the C2 decomposition appears on hardware.
- Report mean ± std over seeds, plus failure counts. Oracle rows are visually marked as upper bounds.
- `run.yaml` provenance (git sha, versions, host, GPU) is kept for every run behind a reported number.
- Timings are reported with the host (workstation B, RTX 6000 Ada). Sim-time vs wall-time is stated for T2.
