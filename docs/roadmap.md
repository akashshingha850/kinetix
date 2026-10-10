# Roadmap

Phases are sequential. Each one has **deliverables** and an **exit test** that must pass before the next
phase starts. Week numbers map to [research/work_plan.md](research/work_plan.md). This file
refines that plan into buildable steps. Status of work in progress: [todo.md](todo.md). The paper
milestones each phase feeds (P-gates) are in [publication-plan.md](publication-plan.md) §5.

| Phase | Name | Weeks | Tier reached | Feeds contribution |
|---|---|---|---|---|
| K0 | Foundations: docs, environment, novelty search | 0 | — | positioning |
| K1 | Core + T0 replay skeleton + first scene/pools (rings + Tammes) | 1–3 | T0 (+ T1 batch) | C4 (T0) |
| K2 | Evaluation and benchmark harness, full baseline set | 3–5 | T0 | C3 |
| K3 | NBV core (monocular scorers) + anchoring and sensing studies | 5–10 | T0 | C1, C2 |
| K4 | Closed loop in sim (T1 serve, T2 flight) | 10–13 | T1, T2 | C4 (T1/T2) |
| K5 | Optimisation (learned scoring, online speed, capture on the move, lighting) | 13–18 | T0–T2 | (only what improves C1–C5) |
| K6 | Real drone + full benchmark + paper artifacts | 19–28 | T3 | C5, C2/C4 on hardware |

---

## K0 — Foundations (done when this doc set is reviewed)

Deliverables
- Doc set: architecture, interfaces, data-format, roadmap, testing, benchmark, decisions, setup, todo.
- `pyproject.toml` + `uv.lock` (Python 3.12, torch cu128, pycolmap, open3d, trimesh, transformers).
- `scripts/check_env.py`.

Exit test
- `uv run python scripts/check_env.py` exits 0 on workstation B.
- A reader can say from the docs which module owns each step of the loop and which topics T2 uses.

## K1 — Core + T0 skeleton (weeks 1–3)

Deliverables
- `src/kinetix/core/` (types, geometry, config + registry, runlog) and switch `tool.uv.package = true`.
- Object set decided (Q2) with licences checked: SimReady plus ≥ 3 from Objaverse++ / GSO / OmniObject3D.
- `sim/scenes/`: build **one** scene (Q2: a textured, mid-complexity object in a simple room) →
  `scene.usd`, `gt_mesh.ply`, `scene.yaml`. bisg mount for the scene (Q3).
- `sim/render_server.py` **batch mode** (with `--zed-depth`: SDK depth + confidence per view, ADR-K10) + `kinetix pool make` → first pools: `rings:4x60` (~240 views, 10 % test)
  and `tammes:128` with reachability masks (ObjView-Bench-compatible),
  rendering left + right + GT depth per view (one pool serves mono and stereo, ADR-K07).
- `core.types.Frame` carries the optional stereo fields from the start (interfaces.md §2), even though K1 runs `mono` only.
- `rig/pool.py`, `planner/loop.py`, `seed.py`, `sequence.py` (orbit, grid), `scorers/random.py`, `stop.py` (budget only).
- `recon/state.py` v1 (re-run mapping) + `recon/align.py`.
- `kinetix run` writes a complete run directory.
- Test fixture: `tests/fixtures/pool_tiny/` (24 views at 320×240 of the same scene, < 15 MB, git-lfs).

Exit test
- `kinetix run --scene <s> --method orbit --rig pool --seed 0 planner.budget=24` → run dir passes
  the schema check, ≥ 90 % frames registered, online-model ATE vs pool poses < 1 % of ROI diagonal.
- Unit + component tests green (`pytest -m "not t1 and not t2 and not t3"`), < 2 min total.

## K2 — Evaluation + benchmark harness (weeks 3–5)

Deliverables
- `offline/sfm.py` incl. **stereo-rig SfM** (`recon_input: stereo_rig`, pycolmap `Rig`),
  `offline/densify.py` (`gsplat` first; `colmap_mvs` via the CUDA COLMAP container; `tsdf_depth` for stereo),
  `offline/metrics.py`, `offline/evaluate.py`, `kinetix eval`.
- `scene` tooling: `gt_observable.ply` (raycast from feasible poses).
- `scorers/coverage_oracle.py` (upper bound), `scorers/fvs.py`, `scorers/volumetric_ig.py`,
  `bench/matrix.py`, `bench/report.py`, `kinetix bench`.
- Metrics beyond F-score: N@q at q ∈ {0.9, 0.95} with dense-reference sensitivity (120/240/360 views), coverage AUC,
  quality at K = 5 / 30, Kendall's τ between tiers (benchmark.md §2–3).
- 3 scenes with pools.
- Baselines run in both `left` and `stereo_rig` reconstruction input.

Exit test
- Metric sanity: evaluating the GT mesh against itself gives Chamfer ≈ 0 and F-score = 1. A sphere
  offset by d gives Chamfer = d ± 1 %.
- `kinetix bench --matrix config/bench/baselines.yaml` (3 scenes × {orbit, grid, random, fvs, volumetric_ig, oracle} ×
  budgets {5, 10, 20, 30, 40, 80} × 3 seeds) finishes unattended and writes `report.md` with the
  F-score-vs-images curve and the N@q table. The oracle is ≥ every baseline at every budget (otherwise the evaluation is wrong).
- Stereo-rig SfM on an orbit is metric without a pose prior: scale error < 1 % against GT.

## K3 — NBV core (weeks 5–10)

Deliverables
- `scorers/sfm_uncertainty.py`, `recon/depth/` sources `mono` (Depth Anything V2, scale fit), `stereo_learned`
  (FoundationStereo, SGBM fallback) and `gt`, `scorers/depth_frontier.py` (consumes any DepthSource), `scorers/combo.py`,
  `select.py` with travel cost, `stop.py` gain plateau.
- Sensing presets (`config/sensing/*.yaml`): mono, mono_noprior, stereo_plan, stereo_full, oracle_depth.
- **Depth study** (research/mono-vs-stereo.md §6) on sim views: `docs/research/depth-study.md`. The ZED SDK
  half needs the T2 stack, so it lands with K4. Learned stereo, mono and GT are done here.
- `recon.pose_source: sfm | prior | gt`.
- Depth-anchoring variants `mono_metric_noanchor` (Metric3D v2 / Depth Pro) and `mono_stereo_anchor` (C1 ablation).
- External baselines `planner/external/fisherrf.py` and `ma_scvp.py` (their code in its own env), run on the K2 pools.
- Ablation matrix `config/bench/ablation.yaml`.

Exit test
- On the 3 K2 scenes, `kinetix_combo` in `mono` has a lower N@0.95 than the **best adaptive baseline**
  (`fvs`, `volumetric_ig`, `fisherrf`, `ma_scvp`) and than `orbit_3ring`, mean over 3 seeds.
- **Go/no-go for C1**: `mono` (SfM-anchored) beats both `mono_metric_noanchor` and `mono_stereo_anchor` on N@0.95.
  If not, C1 is reframed per publication-plan.md §6 before K4 starts.
- The mono-vs-stereo table (planning value, reconstruction value, gap closure) is produced by
  `kinetix bench --matrix config/bench/sensing.yaml`. This exit test needs the table to exist, not mono to win.
- Planning step (sample + score + select, excluding SfM) < 2 s at 40 frames.

## K4 — Closed loop in sim (weeks 10–13)

Deliverables
- `sim/render_server.py` **serve mode** + `rig/isaac_render.py` (T1, continuous view-sphere sampler).
- `rig/mavros.py` (T2) with the per-mode topic whitelist and stereo triple pairing, `recon/depth/stereo_zed.py`,
  `planner/constraints.py` (workspace, no-fly, standoff, segment check).
- Depth study, ZED SDK half (sim). `zed_mapping` baseline script.
- `docker/Dockerfile` (ros:jazzy-ros-base + uv venv) + `docker/compose.yaml` (`kinetix` service, host network).
- Scene loaded in a bisg scenario (`world.usd_path`), drone spawned at a standby pose.

Exit test
- T1: same scene/method/seed as T0 gives F-score within 0.05 of T0.
- T2: `kinetix run --rig mavros --method kinetix_combo` against `./bisg all headless` completes a
  20-image session without operator input, lands, and the run evaluates. Capture-pose error vs Pegasus
  GT < 0.10 m / 3°. F-score within 0.1 of T1.
- The topic-whitelist test passes for every sensing preset (in `mono`, no right/depth topic is subscribed).
- `stereo_plan` on T2 runs end to end with ZED SDK depth.

## K5 — Optimisation (weeks 13–18)

Candidates, picked by K3/K4 results. Each is an isolated module plus a benchmark row:
`recon/state.py` v2 (incremental registration), a learned scorer (imitation of the oracle on T0 pools),
capture on the move (trajectory through views instead of stop-and-shoot), exposure/specularity-aware
scoring, multi-objective budget (images vs flight time).

Exit test: each kept change improves its target metric on the K2 matrix without regressing F-score
by more than 0.02.

## K6 — Real drone + paper (weeks 19–28)

Depends on bisg_isaac Phase 5 (hardware single drone) being done.

Deliverables
- Hardware bench checklist (testing.md §5) passed. Tethered indoor flights.
- Real scenes with reference GT (Q4). Real pools from dense captures (T0 on real data), recorded as
  left + right + ZED depth, so the mono-vs-stereo comparison repeats on real data. Real half of the depth study.
- Full benchmark: 8–10 sim objects + 3 real, all baselines + ablations, `report.md` → paper figures.
- Real objects scanned (turntable or structured light). Real runs report Chamfer, F-score (relative and absolute mm)
  and N@q for at least `orbit_3ring`, `fvs`, `kinetix_combo` (mono) and `stereo_plan`.
- Dataset + code release package (publication-plan.md §7).

Exit test: the paper's tables regenerate from `kinetix bench` with one command per table.
