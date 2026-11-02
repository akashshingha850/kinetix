# Roadmap

Phases have **deliverables** and an **exit test**. Week numbers map to [paper/research/work_plan.md](paper/research/work_plan.md);
the paper gates G1–G5 are in [paper/publication-plan.md](paper/publication-plan.md) §5. Day-to-day status:
[acquisition/todo.md](acquisition/todo.md) and [reconstruction/todo.md](reconstruction/todo.md).

**Reformed 2026-11-01.** The phases were planned to run in order. In practice the flight stack came first: T1 serve and T2
flights (K4) and room capture (ADR-K11/K12) were built before the K1/K2 evaluation they need. From now on the order is set by
what each paper claim needs (C1–C5, publication-plan.md §2), and nothing new is built for flights until the benchmark can score
it. ✅ = done, 🟡 = partly done, ⬜ = open.

**Method and validation architecture: [methodology.md](methodology.md)** (2026-11-01): the method's stages M0–M9, validation
levels V0–V8 with pass criteria, and the **flight gate**. No new T2 (PX4/ROS) work for the object paper until that gate is green.

| Phase | Name | Status (2026-11-01) | Gate | Feeds |
|---|---|---|---|---|
| K0 | Foundations: docs, environment, novelty search | ✅ | — | positioning |
| K1 | Core, scenes, T0 replay, first pools | 🟡 3 scenes + ring pools + pool rig; Tammes pool, online ReconState, test fixture open | G1 | C4 (T0) |
| K2 | Evaluation + benchmark harness, baselines | 🟡 sparse eval + `kinetix bench` + 9 baselines, first matrix run; densify and stereo-rig SfM open | **G1 2026-11-12** | C3 |
| K3 | NBV core: the C1 method, anchoring and sensing studies | ⬜ nothing built | **G2 2026-12-17** (C1 go/no-go) | C1, C2 |
| K4 | Closed loop in sim (T1 serve, T2 flight) | 🟡 rigs built and flown early; tier-transfer runs and exit tests open | G3 2027-01-07 | C4 (T1/T2) |
| K5 | Optimisation | ⬜ | — | only what improves C1–C5 |
| K6 | Real drone + paper | ⬜ (needs bisg Phase 5) | G4, G5 | C5, C2/C4 on hardware |

## Steps to submission (2026-11-02, ordered by research question)

Each step answers one question of [paper/research-questions.md](paper/research-questions.md) (ADR-K14, proposed), ends on an exit test
fixed in advance (V-levels: [methodology.md](methodology.md) §4) and blocks the steps after it. A step that fails is fixed there, never
compensated later. The K-phases below keep the deliverable detail; methodology.md §8 is the same order at finer grain.

| Step | Answers | Do | Exit test | Ends by |
|---|---|---|---|---|
| S1 frozen evaluation | (all) | decide N@q relative to the scene's F_ref; SfM repeatability, 1 cell × 5 repeats (V2) | V1 decision written; F within 0.01 across repeats | G1 2026-11-12 |
| S2 registration | RQ1 | a predictor of SfM registration replaces `coverage.min_overlap`, shared by every adaptive method; re-run `config/bench/oracle.yaml` | V3 overlap ρ ≥ 0.7; V4 green (oracle ≥ every GT-free method) | 2026-11-26 |
| S3 C1 components | RQ3 | ReconState, DA-V2 anchoring with online anchors at 5–20 views, TSDF from anchored depth, timing | V5a–d | 2026-12-10 |
| S4 claims on T0 | RQ2, RQ3 | `sfm_uncertainty`, `combo`; ablation + sensing matrices, 3 scenes × 3 seeds; novelty sweep #1 incl. registration-aware NBV | V6; **C1 go/no-go** | **G2 2026-12-17** |
| S5 scale-up | RQ1–RQ3 | 8–10 scenes (≥ 3 GSO/Objaverse++) × 5 seeds, weights frozen on `office_buddha`; `tammes:128` pool | the paper matrix regenerates with one command | 2027-01-21 |
| S6 transfer in sim | RQ4 | T1 transfer with dense eval on 2 scenes (V7); pose noise on T0 (V8); flight gate; T2, 4 methods × 5 seeds | V7, V8, flight gate (methodology §5); T2 τ reported | G3 2027-01-07 for V7/V8, T2 by 2027-02-11 |
| S7 real flight | RQ4 | 3 real objects with scanned GT (scanner by G3), real runs of `orbit_rings`, `fvs`, Kinetix, `stereo_plan` | quantitative Chamfer / F / N@q on T3 | G4 2027-03-25 (needs bisg Phase 5) |
| S8 paper + release | all | write, final novelty sweep, release package | publication-plan §8 checklist | G5 2027-04-22 |

S5 and S6 overlap. G3 can only freeze the method if S4 ran to plan, so S5's scale-up runs on T0 while S6 runs on the sim. Rooms
(ADR-K11–K13) are outside the object paper and outside this list.

---

## K0 — Foundations ✅

Doc set, `pyproject.toml` + `uv.lock`, `scripts/check_env.py`, novelty search (2026-10-08..10).

## K1 — Core + T0 skeleton 🟡

Deliverables
- ✅ `kinetix/` (types, geometry, config, runlog, align, coverage), `tool.uv.package = true` (2026-10-28).
- 🟡 Object set (Q2): three Smithsonian CC0 scans in the Isaac Office lobby (`sim/scenes/*.yaml` → `sim/build_scene.py`, plinth
  for the two small ones). Still to add: ≥ 3 from Objaverse++ / GSO / OmniObject3D so the set overlaps ObjView-Bench.
- ✅ `sim/render_server.py` batch mode (left + right + GT depth) and `kinetix pool make` → pool `rings` = the planner's ring stack
  (`object.yaml planner.rings`: 2 radii × 4 heights × 24 azimuths = 192 views, ~90 s per scene). ⬜ `tammes:128` with reachability
  masks, the 10 % test split, `--zed-depth` (ADR-K10).
- ✅ `acquisition/rig/pool.py` (T0), the loop (`planner/loop.py`, `nbv.py`: seed views, fixed sequences, stop rule).
- ⬜ `acquisition/mapping/state.py` v1 (online pycolmap ReconState; the coverage proxy stands in).
- ⬜ `tests/fixtures/pool_tiny/` (git-lfs); `test_baselines.py` builds a tiny fake pool for now.

Exit test: `kinetix run --rig pool --method orbit` → run dir, ≥ 90 % registered, ATE < 1 % of D. ✅ (2026-11-01 smoke: roi_coverage
20 views, 20/20 registered, ATE 2.4 mm on D = 1.82 m).

## K2 — Evaluation + benchmark harness 🟡 (G1)

Deliverables
- ✅ `reconstruction/sfm.py` (mono, incremental or known-pose), `metrics.py`, `evaluate.py`, `kinetix eval` (sparse points,
  Sim3 to the GT centres, cropped to the ROI, against `gt_observable` / `gt_mesh`).
- ⬜ **`reconstruction/densify.py`**: pycolmap here has no CUDA, so dense MVS needs the CUDA COLMAP container, or `gsplat`.
  Without it the F-score is recall-limited (step 2 above). Owner: reconstruction.
- ⬜ Stereo-rig SfM (`recon_input: stereo_rig`); baselines with `left` and `stereo_rig` input.
- ✅ `gt_observable` (scene builder), `coverage_oracle` (coverage.py on the GT surface), scorers `fvs`, `volumetric_ig`, fixed
  `orbit_rings`, bench seeds (seed k > 0 = seeded first view / sequence offset).
- ✅ `bench/matrix.py`, `bench/report.py`, `kinetix bench`: F@1 % per budget (mean ± std over seeds), N@q (q 0.9, 0.95),
  image reduction vs the best fixed and best simple adaptive baseline, Kendall's τ across bench folders. ⬜ coverage AUC.

Exit test
- ✅ Metric sanity: GT vs itself F = 1; a sphere offset by d has Chamfer d ± 1 % (`reconstruction/tests/test_metrics.py`).
- ✅ `kinetix bench --matrix config/bench/baselines.yaml` (3 scenes × 9 methods × budgets {5…80} × 3 seeds) finishes unattended
  (~35 min, 6 jobs) and writes `report.md` (2026-11-01; results in acquisition/todo.md). N@q is empty on sparse points: densify.
- ❌ The oracle is ≥ every baseline at every budget: **fails** (2026-11-01) with both rules (30° / 0.2 and 15° / 0.5);
  `volumetric_ig` beats it. The coverage model is what needs fixing (it is also the planner's gain), before K3 builds on it.
- ⬜ Stereo-rig SfM on an orbit is metric without a pose prior (scale error < 1 %).

## K3 — NBV core: the C1 method ⬜ (G2)

Deliverables (unchanged)
- `scorers/sfm_uncertainty.py`, depth sources `mono` (Depth Anything V2, scale fit to the SfM points), `stereo_learned`, `gt`,
  `scorers/depth_frontier.py`, `scorers/combo.py`, `select.py` with travel cost, gain-plateau stop.
- Sensing presets (`config/sensing/*.yaml`): mono, mono_noprior, stereo_plan, stereo_full, oracle_depth.
- Anchoring variants `mono_metric_noanchor`, `mono_stereo_anchor` (C1 ablation), `recon.pose_source: sfm | prior | gt`.
- External baselines `fisherrf`, `ma_scvp` on the K2 pools. Ablation matrix `config/bench/ablation.yaml`.
- Needs the `ml` extra (torch, transformers): not installed in the venv yet.

Exit test (unchanged): `kinetix_combo` in `mono` beats the best adaptive baseline and `orbit_rings` on N@0.95 over 3 scenes × 3 seeds;
C1 go/no-go (SfM anchoring beats both alternatives); the mono-vs-stereo table exists; planning step < 2 s at 40 frames.

## K4 — Closed loop in sim 🟡 (G3)

Built ahead of plan
- ✅ T1: `render_server.py` serve + `acquisition/rig/isaac_render.py` (2026-10-31). ✅ T2: `acquisition/rig/mavros.py`,
  constraints, Docker image, scenes loaded by bisg (`bisg.yaml` locator). Flown: object demo (2026-10-11), `office_buddha`
  roi_coverage / surface_normal (2026-10-31). ✅ `surface_normal` (explore-then-exploit baseline, T1/T2 only: it plans off the pool grid).
- Room capture (ADR-K11, K12): Warehouse room NBV + `tour`, flown 2026-10-17/18. Outside the object paper; kept as is.

Open
- 🟡 Tier transfer: T0 ↔ T1 done on office_buddha (6 methods × 3 seeds, τ = 0.87, F@80 within 0.035 for 5 of 6); T2 open (step 6); T1 needs `scripts/render_server.sh serve <scene>` per scene,
  T2 one fresh sim per capture (bisg todo: second-flight yaw oscillation).
- ⬜ Exit tests: T1 within 0.05 F of T0; T2 20-image session unattended, capture-pose error < 0.10 m / 3°, F within 0.1 of T1;
  topic whitelist per sensing preset; `stereo_plan` on T2 with ZED SDK depth.
- ⬜ Depth study (ZED half), `zed_mapping` baseline.

## K5 — Optimisation ⬜

Unchanged: each candidate (incremental ReconState, learned scorer from the oracle, capture on the move, exposure-aware scoring,
images-vs-time budget) is an isolated module with a benchmark row, kept only if it improves its metric on the K2 matrix without
costing more than 0.02 F.

## K6 — Real drone + paper ⬜

Unchanged: depends on bisg Phase 5. Real scenes with scanned GT (Q4), real pools, the full benchmark (8–10 sim objects + 3 real),
real runs for `orbit_rings`, `fvs`, `kinetix_combo` (mono) and `stereo_plan`, the release package. Exit test: every paper table
regenerates from `kinetix bench` with one command.
