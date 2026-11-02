# Methodology and validation architecture

Written 2026-11-01. This is the plan the object paper is built and checked against: what the method is, stage by stage, and how
each stage and each claim is validated before anything new is flown on PX4/ROS (T2). System context, tiers and module map:
[architecture.md](architecture.md). Claims C1–C5: [paper/publication-plan.md](paper/publication-plan.md) §2. Metrics and matrix:
[paper/benchmark.md](paper/benchmark.md). Phase order: [roadmap.md](roadmap.md). Rooms (`tour`, ADR-K11–K13) are outside this paper
and outside this document.

**Rule.** No new T2 (PX4 SITL + MAVROS) or T3 work for the object paper until the **flight gate** (§5) is green. T2 is used for one
thing: measuring what changes when the same method flies (C4). Everything a flight would teach us about the method itself must
already be known from T0/T1.

---

## 1. Problem

Given:

- a scene `S`: object ROI box, workspace, no-fly boxes, standoff and altitude limits (`kinetix/scenes/*.yaml`, `sim/scenes/*.yaml`)
- a camera with intrinsics `K` on a fixed mount (pitch `camera.mount_pitch_deg`, so a pose is 4-DoF: x, y, z, yaw)
- a candidate set `P`: a discrete pool (T0, `rings` today, `tammes:128` planned) or a continuous pose space (T1–T3)
- an image budget `N`, or a free stop (gain plateau)
- a fixed offline pipeline `R`: SfM, then densify (benchmark.md §3)

choose an ordered sequence `v1 … vn ∈ P`, `n ≤ N`, each feasible from the previous one, that maximises the quality
`Q(R(I(v1 … vn)))` of the final reconstruction against ground truth. Travel is a secondary cost. The planner sees only what its
**sensing mode** allows (§3); it never sees ground truth unless it is the oracle.

`Q` is the F-score at 1 % of the ROI diagonal `D` on the **dense** reconstruction. The paper's headline is N@q: the smallest `n` that
reaches `q · F_ref`, where `F_ref` is the F-score of a dense reference capture of the same object (C3).

## 2. The method, stage by stage

```
            ┌──────────────── online, per step (planner; no GT) ────────────────┐
 M0 candidates ─► M1 seed ─► capture ─► M2 online SfM ─► M3 depth ─► M4 TSDF map ─► M5 scorers ─► M6 select ─► M7 stop?
                                ▲                                                               │
                                └──────────────────────────── fly / replay ◄────────────────────┘
 offline: M8 SfM over all frames ─► M9 densify ─► evaluation (§4, V0/V1; never seen by the planner)
```

| Stage | What it does | Module | Status 2026-11-01 |
|---|---|---|---|
| M0 candidates | feasible 4-DoF poses: pool views, or ring / Tammes samples; workspace, no-fly, ROI shell, straight-leg check | `planner/sampler.py`, `constraints.py`, `rig/pool.py` | ✅ rings; ⬜ Tammes + reachability mask |
| M1 seed | first view nearest the start; seed > 0 = seeded random first view | `planner/nbv.py` | ✅ (one view; a short overlapping arc is the C1 option) |
| M2 online SfM | incremental pycolmap over the captured frames; Sim3 to the rig poses → metric `map` frame; exposes registered images, points, track length, triangulation angle, reprojection error | `mapping/state.py` (ReconState) | ⬜ the coverage proxy stands in |
| M3 depth | per-frame depth for planning, from `sensing.plan_depth`: `mono` = Depth Anything V2 relative depth **anchored** to the SfM points the frame sees (robust scale + shift in inverse depth); `mono_metric_noanchor`; `mono_stereo_anchor`; `stereo` (ZED SDK / FoundationStereo); `gt` (oracle) | `mapping/depth/` | ⬜ (`gt` only, from the pool / render) |
| M4 map | occupancy / TSDF over the ROI from M3's depth at the M2 pose | `mapping/depthmap.py` `OccupancyMap` | ✅ (fed GT depth today) |
| M5 scorers | `depth_frontier`: unknown voxels a candidate would see before the first occupied one (= `volumetric_ig` on M4). `sfm_uncertainty`: visible sparse points with short tracks, small triangulation angle or high reprojection error, plus angular gaps around the ROI. `combo` = weighted sum. All adaptive methods share the **registration constraint** (below) | `planner/scorers/` | 🟡 `volumetric_ig` ✅; `sfm_uncertainty`, `combo` ⬜ |
| M6 select | argmax of gain − λ_d·metres − λ_y·\|Δyaw\|; seeded tie-break | `planner/nbv.py` `select` | ✅ |
| M7 stop | budget, or gain below `min_gain` for `patience` steps | `planner/nbv.py` | ✅ |
| M8 SfM | final SfM over all frames: covis pairing, incremental (`pose_source: sfm`) or known pose (`prior`, `gt`) | `reconstruction/sfm.py` | ✅ (known-pose 3–6 mm offset open) |
| M9 densify | dense points or mesh from M8 | `reconstruction/densify.py` | ⬜ **blocks every C3 number** |

**Registration constraint.** A view that cannot be matched to the images already taken does not register in SfM and adds nothing,
whatever its geometric gain. The first benchmark showed this directly: `fvs`, `orbit_rings` and the coverage oracle spread their
first 20 views so widely that only 26–45 % registered. Every adaptive method therefore gets the same rule: a candidate is admissible
only if it would share at least `coverage.min_overlap` of its view with already-seen surface from within `coverage.match_max_deg`.
The planner predicts that overlap from its model (box proxy, depth map or SfM points, depending on the method). It is a feasibility
rule, not a score term, so it also applies to the baselines and does not favour C1.

**Every baseline is a configuration of these stages** (benchmark.md §1):

| Method | M3 depth | M4 map | M5 gain | Uses GT |
|---|---|---|---|---|
| `orbit`, `orbit_rings` | – | – | fixed sequence | no |
| `random`, `fvs` | – | – | random / distance to captured centres | no |
| `roi_coverage` | – | ROI box proxy | coverage gain | no |
| `depth_coverage` | gt (T0/T1), ZED (T2) | depth surface | coverage gain | yes on T0/T1 |
| `volumetric_ig` | gt (T0/T1), ZED (T2) | occupancy | unknown voxels | yes on T0/T1 |
| `coverage_oracle` | – | GT surface | coverage gain | yes (upper bound) |
| **`kinetix_combo` (C1)** | **mono, SfM-anchored** | **TSDF** | **frontier + SfM uncertainty** | **no** |
| C1 ablations | `mono_metric_noanchor`, `mono_stereo_anchor`, `none` | TSDF / – | same | no |
| sensing presets (C2) | `stereo` / `gt` | TSDF | same | `oracle_depth` only |

On T0/T1 the depth baselines read rendered GT depth, which makes them stronger than they would be on hardware. They are flagged
`uses_gt` and are reported as "depth-oracle" baselines. Their honest versions are `stereo_plan` (ZED/FoundationStereo depth) and C1.

### 2.1 Why each stage is built this way

Each design choice rests on peer-reviewed evidence; the right-hand column is what that evidence does **not** settle and Kinetix has
to measure itself (the V-level that measures it). References are numbered as in [related_work.md](paper/research/related_work.md).

| Stage | Choice | Evidence | Still ours to show |
|---|---|---|---|
| M0 | discrete pool of 4-DoF candidates, Tammes layout planned | view-sphere candidate sets are the NBV standard [[14]](paper/research/related_work.md#ref14), [[47]](paper/research/related_work.md#ref47); ObjView-Bench's 128-view Tammes sphere [[46]](paper/research/related_work.md#ref46) (preprint) | discrete vs continuous changes nothing (V7) |
| M2, M8 | incremental SfM (COLMAP/pycolmap) as the planner's model and the final pipeline | COLMAP [[1]](paper/research/related_work.md#ref1); next-view choice inside SfM to keep it well-conditioned [[1]](paper/research/related_work.md#ref1) | ReconState = offline SfM on the same frames (V5c) |
| M3 | relative depth (DA-V2) anchored by a scale + shift fit in inverse depth to the SfM points | DA-V2 [[51]](paper/research/related_work.md#ref51); the scale-and-shift-invariant formulation of relative depth [[52]](paper/research/related_work.md#ref52); SfM points guiding monocular depth, offline [[40]](paper/research/related_work.md#ref40); naive single-image depth fails inside an NBV loop [[35]](paper/research/related_work.md#ref35), [[39]](paper/research/related_work.md#ref39) | anchoring beats metric depth and stereo anchoring *online*, at 5–20 views (V5a, RQ3) |
| M4 | TSDF / occupancy over the ROI | volumetric fusion [[58]](paper/research/related_work.md#ref58); volumetric NBV maps [[16]](paper/research/related_work.md#ref16), [[17]](paper/research/related_work.md#ref17) | map from anchored depth ≈ map from GT depth (V5b) |
| M5 | frontier (unknown-voxel) gain + SfM-uncertainty gain | frontiers [[23]](paper/research/related_work.md#ref23); volumetric information gain compared [[16]](paper/research/related_work.md#ref16), [[17]](paper/research/related_work.md#ref17); uncertainty-driven view choice [[1]](paper/research/related_work.md#ref1), [[48]](paper/research/related_work.md#ref48) | each term helps alone, and combined (V6 ablation) |
| M5 | **registration constraint**: a candidate must be matchable to what is seen | detector/descriptor matching fails beyond ~25–30° viewpoint change [[53]](paper/research/related_work.md#ref53); matchability can be predicted [[54]](paper/research/related_work.md#ref54) | the predictor actually predicts SfM registration (V3 overlap half, RQ1) |
| M6 | argmax of gain − travel cost | receding-horizon NBV utility with path cost [[15]](paper/research/related_work.md#ref15); drone NBV with travel [[10]](paper/research/related_work.md#ref10), [[42]](paper/research/related_work.md#ref42) | — (fixed design, weights frozen on Buddha, §7) |
| M9 | dense MVS (COLMAP patch-match) | [[59]](paper/research/related_work.md#ref59); evaluation needs dense surfaces, sparse points cap recall (V1, measured) | F_ref ≥ 0.8 per scene (V1) |
| eval | F-score at τ = 1 % of D, accuracy/completeness | Tanks and Temples precision/recall/F [[56]](paper/research/related_work.md#ref56); Sim3 alignment [[60]](paper/research/related_work.md#ref60) | metric code correct (V0) |
| stats | paired Wilcoxon over (scene, seed), Holm correction, Kendall τ across tiers | Wilcoxon for paired comparisons over data sets [[57]](paper/research/related_work.md#ref57); Holm [[61]](paper/research/related_work.md#ref61); sim→real predictivity as a rank/correlation between tiers [[55]](paper/research/related_work.md#ref55) | — |

## 3. Information boundary

What the planner may read, by sensing preset (ADR-K07; the topic whitelist is in [interfaces.md](interfaces.md) §4):

| Preset | Planner reads | Reconstruction reads |
|---|---|---|
| `mono` (C1) | left image, `K`, rig pose | left images (+ rig pose as prior when `pose_source: prior`) |
| `mono_noprior` | left image, `K`, rig pose; no depth | left |
| `stereo_plan` | + stereo depth | left |
| `stereo_full` | + stereo depth | left + right as a rig, + stereo depth |
| `oracle_depth` | + GT depth | left |

Ground truth (GT mesh, `gt_observable`, GT depth, `gt_pose`) is used only by the evaluator, by the oracle and by the depth-oracle
baselines, and `run.yaml uses_gt` says so. Evaluation (`reconstruction/`) never imports `acquisition/`
(`kinetix/tests/test_boundaries.py`), so the method under test cannot shape its own scoring.

## 4. Validation architecture

Nine levels. Each one answers one question, has a pass criterion fixed **before** the run, and blocks the levels above it. A level
that fails is fixed at that level; it is never compensated further up.

| Level | Question | Test | Pass criterion | Tier | Status |
|---|---|---|---|---|---|
| **V0** metric correctness | do the metrics compute what they claim? | `reconstruction/tests/test_metrics.py`: GT vs itself, sphere offset by d, empty recon; Sim3 recovers a known transform | F = 1; Chamfer = d ± 1 %; no crash | unit | ✅ |
| **V1** evaluation fidelity | does F measure *surface quality* rather than point count? | dense eval (M9) on the reference prefixes | F_ref ≥ 0.8 at 1 % of D on every scene; F rises monotonically over the reference prefixes 50/75/100 %; N@0.9 exists for the reference itself | T0 | 🟡 dense (`bench/dense`, 2026-11-01): F_ref at 192 views 0.93 Buddha / 0.92 Monster / 0.86 Chair ✅; prefixes 96/144/192 rise on Buddha (0.91→0.92→0.93) and Chair (0.83→0.86→0.86), Monster falls 0.95→0.94→0.92 (one seed; dense noise ~0.03) ⚠️; N@0.9 does not exist for the Chair reference ❌ → q must be relative to F_ref (decision open) |
| **V2** reproducibility | is a cell a pure function of its inputs? | same scene, method, seed and budget twice; SfM `random_seed = 0` | identical `chosen` sequence; F within 0.01 | T0 | 🟡 sequence deterministic (tested); SfM repeatability not measured |
| **V3** proxy validity | does what the planner optimises predict what is measured? | for every bench cell and budget: occlusion-aware GT coverage of the *registered* views vs measured completeness ratio; predicted overlap vs registered fraction | Spearman ρ ≥ 0.7 for coverage → completeness; registered fraction ≥ 0.9 whenever the constraint says admissible | T0 | 🟡 dense (252 rows): coverage → completeness ρ = 0.73 (`k_min` 2) / **0.76** (3) ✅, coverage → F 0.70 / 0.73; overlap → registered ρ = **0.24** ❌ (sparse was 0.54 / 0.56 / 0.36) |
| **V4** upper bound | does the GT oracle bound every GT-free method? | `coverage_oracle` with occlusion-aware visibility and the registration constraint, on the full matrix | oracle ≥ every non-GT method at every budget, within one seed std | T0 | ❌ fails dense too: oracle ≥ best non-GT method in 1 of 24 (scene, budget, k_min) cells; registration, not coverage (§4.1) |
| **V5** component checks of C1 | is each C1 part right before it is combined? | V5a anchoring: AbsRel of anchored DA-V2 vs pool GT depth, against #SfM points in view; V5b map: IoU of the M4 map built from M3 vs from GT depth; V5c ReconState vs offline SfM on the same frames; V5d time per planning step | V5a AbsRel ≤ 0.05 with ≥ 50 points, and better than `mono_metric_noanchor`; V5b IoU ≥ 0.7; V5c same registered set, centres within 1 % of D; V5d < 2 s at 40 frames | T0 | 🟡 V5a first pass 2026-11-01 (below); V5b–d ⬜ |
| **V6** claims on T0 | do C1, C2, C3 hold on the matrix? | `config/bench/ablation.yaml`, `sensing.yaml`, `baselines.yaml`: 3 scenes × 3 seeds (K3), then 8–10 scenes × 5 seeds (paper) | per claim, the statistics of §6 | T0 | ⬜ (baselines done, sparse) |
| **V7** tier transfer T0 → T1 | do continuous rendering and on-demand views change the conclusions? | the same 4–6 methods on T1 | Kendall τ ≥ 0.8; F at 80 views within 0.05 of T0 for every method | T1 | 🟡 office_buddha τ = 0.87, 5 of 6 within 0.035 (sparse eval; repeat after V1) |
| **V8** flight robustness, emulated | do pose noise and capture error change the ranking? | T0 with pose noise 0 / 2 cm + 1° / 5 cm + 2° (benchmark.md §4) on the candidate poses and on `frames.jsonl pose` | Kendall τ ≥ 0.8 vs noise-free; C1 margin over the best baseline keeps its sign | T0 | ⬜ |

V5a, first pass (2026-11-01, `acquisition/tests/anchor_bench.py`, Buddha `orbit_rings` s0, every 4th of 80 registered images,
anchors = the points of the **final 80-view** SfM model, median 575 per image): median AbsRel on ROI pixels `mono` (DA-V2 Small,
SfM-anchored) **0.035**, `mono_metric_noanchor` (DA-V2 Metric-Indoor Small) 0.20, `affine_gt` (same network, affine fit to GT depth:
the bound) 0.023. Passes on this case. Still to show: the same with the anchors an *online* model has at 5–20 views (fewer, noisier
points), on the other two scenes, and against `mono_stereo_anchor`.

V5 and V8 are what make T2 a measurement instead of a debugging session: a T2 result that differs from T0/T1 can then only come
from dynamics, settle time and the ZED twin, which is what C4 reports.

### 4.1 Diagnosis of V4 (2026-11-01) and the fix

The oracle plans on `gt_observable` with the same `CoverageState` as `roi_coverage` (`kinetix/coverage.py`). Two faults:

1. **No self-occlusion.** `visible()` tests frustum, range and incidence, and occlusion only by scene boxes. On a concave object
   (Buddha folds, the monster's undercuts, chair legs) it counts surface that the GT depth shows is hidden, so it believes a view
   covers more than it does. Fix: a per-pose depth test against the GT mesh (Open3D ray cast at reduced resolution, cached per
   candidate), used by the oracle and by V3.
2. **Saturation at one pair.** A point counts as covered once two rays 8–30° apart see it, and further rays earn 0.1 only if they
   are 25° away from all others. Coverage reaches ~0.92 by about 25 views on Buddha, after which the gain is noise and the oracle
   picks almost arbitrary views. Dense MVS needs three or more views per point. Fix: coverage counts a point when `k_min` views
   (config, 3) see it with a pair in the matchable band. This is the target M9 needs.

Fixed oracle, dense scoring (2026-11-01, `$KINETIX_DATA/bench/oracle`, 3 seeds, mean F@1 % / registered fraction; invalid = F 0):

| Scene | k_min | 10 | 20 | 40 | 80 |
|---|---|---|---|---|---|
| Buddha | 2 | 0.54 / 0.90 | 0.43 / 0.55 | 0.67 / 0.78 | 0.90 / 1.00 |
| Buddha | 3 | 0.55 / 0.87 | 0.40 / 0.53 | 0.79 / 0.87 | 0.92 / 1.00 |
| Monster | 2 | 0.36 / 0.67 | 0.19 / 0.42 | 0.79 / 0.80 | 0.93 / 0.99 |
| Monster | 3 | 0.56 / 0.93 | 0.59 / 0.82 | 0.92 / 0.97 | 0.94 / 1.00 |
| Chair | 2 | 0.00 / 0.10 | 0.17 / 0.45 | 0.69 / 0.84 | 0.78 / 1.00 |
| Chair | 3 | 0.00 / 0.27 | 0.17 / 0.43 | 0.70 / 0.88 | 0.82 / 1.00 |

Dense scoring lifts F at 80 views to 0.78–0.94 (sparse: 0.24–0.48). `k_min` 3 is as good or better everywhere. What remains is
**registration**: at 10–20 views 27–55 % of the oracle's images do not register in SfM (a dip of F from 10 to 20 views is runs whose
evaluation turned invalid). The `min_overlap` 0.2 rule in the coverage model does not predict SfM registration, so the registration
constraint of §2 needs a model that does (V3, overlap half).

V4 against `bench/dense` (same budgets, 3 seeds; best non-GT method per cell):

| Scene | 10 | 20 | 40 | 80 |
|---|---|---|---|---|
| Buddha | k3 0.55 < `depth_coverage` 0.57 | 0.58 < `orbit` 0.69 | 0.79 < `depth_coverage` 0.85 | 0.92 = `fvs` 0.92 |
| Monster | 0.56 < `orbit` 0.57 | 0.77 < `orbit` 0.79 | 0.92 < `fvs` 0.95 | 0.94 < `orbit_rings` 0.96 |
| Chair | 0.02 < `volumetric_ig` 0.54 | 0.43 < `volumetric_ig` 0.69 | 0.70 < `volumetric_ig` 0.79 | 0.82 < `volumetric_ig` 0.87 |

The oracle is beaten almost everywhere, while V3 shows its objective (occlusion-aware `k_min` coverage) does predict completeness
(ρ 0.76). The methods that beat it either keep consecutive views close (`orbit`, `volumetric_ig`, ~0.9–1.0 registered at 10–20
views) or reach 40+ views where everything registers. The oracle spreads views for coverage and loses 13–82 % of them in SfM. So
V4 fails on one missing term: a registration model that predicts SfM registration (overlap → registered ρ 0.24). That model is the
next item of §8; it is shared by every adaptive method, so it changes `roi_coverage`, `depth_coverage` and the oracle together.

The same two faults apply to `roi_coverage` and `depth_coverage`, so the fix is in shared code (a `kinetix/` PR). V3 then tests
whether the fixed model predicts the measured completeness; V4 is re-run after V1, because sparse-point F rewards point count, which
no coverage model optimises.

## 5. Flight gate (before PX4 / ROS)

T2 runs for the object paper start when all of these hold:

1. V0–V4 green on 3 scenes (dense evaluation, valid oracle).
2. V5 green, and the C1 go/no-go (publication-plan.md §6) decided on T0.
3. V7 green with dense evaluation on at least 2 scenes.
4. V8 green: the conclusions survive 5 cm / 2° pose noise.
5. The T2 protocol is fixed in advance: methods (`orbit_rings`, `fvs`, `volumetric_ig` as `stereo_plan`, `kinetix_combo`), seeds (5), one
   fresh sim per capture, the capture-pose error and settle criterion (testing.md §3), and the success thresholds of roadmap K4.

Flights done before this gate (the 2026-10-11 demo and the 2026-10-31 `office_buddha` runs) were plumbing checks of `MavrosRig`; none
of their numbers go into the paper.

## 6. Statistics

- **Unit of analysis**: one (scene, seed) pair. Methods are compared paired on the same scene and seed.
- **Curves**: F@1 % vs N, mean ± std over seeds, per scene. Failed evaluations count as F = 0 (benchmark.md §3) and are also reported as a count.
- **N@q**: interpolated on the per-(scene, seed) curve, then averaged; 95 % bootstrap CI over (scene, seed) pairs (10 000 resamples).
  A pair that never reaches q is reported as "> max budget" and enters the ranking as the worst value.
- **Image reduction**: `1 − N@q(C1) / N@q(baseline)` against the best fixed and the best adaptive non-GT baseline, chosen per scene
  *before* C1 runs, with its bootstrap CI.
- **Method comparison**: Wilcoxon signed-rank on paired N@q (or F at a fixed N) over (scene, seed) pairs; Holm correction across
  the comparisons in one table. With 3 scenes × 3 seeds (n = 9) only large effects can be significant; the paper matrix is 8–10 × 5.
- **C1 go/no-go** (V6): SfM-anchored `mono` beats `mono_metric_noanchor` and `mono_stereo_anchor` on N@0.95, paired, with the CI of
  the difference above 0. Otherwise C1 is dropped as a contribution (publication-plan.md §6).
- **Tier transfer**: Kendall τ of method rankings by N@q (F at the largest budget while N@q is undefined), with its permutation
  p-value; ΔN@q per method per tier.
- **Determinism**: every number traces to a `results.csv` row, its `run.yaml` (git sha, seed, config) and its `cells/<cell>.json`.

## 7. Threats to validity

| Threat | Effect | Mitigation |
|---|---|---|
| Sparse evaluation | F tracks point count, not coverage (V1 fails) | densify before any C3 number |
| GT depth in the depth baselines on T0/T1 | depth baselines look better than on hardware | `uses_gt` flag; honest `stereo_plan` rows; C1 never reads GT |
| Discrete pool | NBV on 192 views is easier than continuous | T1 transfer (V7); Tammes pool as a second layout |
| Pool and reference share the views | `F_ref` depends on the pool | reference sensitivity at 50/75/100 % (C3) |
| Three objects, one room | category effects untested | 8–10 objects incl. ≥ 3 Objaverse++/GSO before the paper (K6) |
| Fixed level mount | top surfaces rarely seen; top ring weak on < 1 m objects | Q1 (mount pitch) as an ablation; per-scene rings |
| Renderer has no motion blur, sim ZED ≠ real ZED | sim conclusions about capture on the move or stereo depth may not transfer | stop-and-shoot only; depth study RQ5; T3 |
| Known-pose SfM offset (3–6 mm) | biases the `pose_source` ablation | find it before that ablation runs |
| Tuning on the test scenes | optimistic C1 numbers | C1 weights tuned on `office_buddha` only, frozen before the other scenes run; tuning runs reported |

## 8. Order of work to the flight gate

1. **V1**: `reconstruction/densify.py` with COLMAP patch-match stereo in the `colmap/colmap` CUDA image, dense eval in `evaluate.py`;
   re-run the reference and the baseline matrix dense. ✅ done 2026-11-01 (`bench/dense`, 66 cells, ~10 h, 1 GPU); open: q relative to F_ref.
2b. **Registration model**: replace the `min_overlap` rule with a predictor of SfM registration (V3 overlap half ρ ≥ 0.7), then
   re-run `oracle.yaml`; V4 green is the condition to start V6.
2. **V4 fix + V3**: occlusion-aware and `k_min`-view coverage in `kinetix/coverage.py`; `bench/validate.py` for V3; re-run the oracle.
   ✅ done 2026-11-01: V3 coverage half passes (ρ 0.76), overlap half and V4 fail (§4.1). With occlusion the oracle's GT coverage saturates at ~0.89 by
   ~25 views on Buddha (k_min 2 and 3): beyond that it has only the novel-direction term to go on.
3. **V2**: SfM repeatability (one cell, 5 repeats).
4. **V5**: ReconState, DA-V2 anchoring, the TSDF map from anchored depth, each against its GT counterpart on the pools.
5. **V6**: `sfm_uncertainty`, `combo`, the ablation and sensing matrices; C1 go/no-go.
6. **V7, V8**: T1 transfer with dense eval on 2 scenes; pose-noise ablation on T0.
7. Flight gate review (§5), then T2.
