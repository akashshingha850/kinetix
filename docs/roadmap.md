# Roadmap

Phases are sequential. Each one has **deliverables** and an **exit test** that must pass before the next
phase starts. Week numbers map to [research/work_plan.md](research/work_plan.md). This file
refines that plan into buildable steps. Status of work in progress: [todo.md](todo.md).

| Phase | Name | Weeks | Tier reached |
|---|---|---|---|
| K0 | Foundations: docs, environment | 0 | — |
| K1 | Core + T0 replay skeleton + first scene/pool | 1–3 | T0 (+ T1 batch) |
| K2 | Evaluation and benchmark harness, baselines | 3–5 | T0 |
| K3 | NBV core (monocular scorers) | 5–10 | T0 |
| K4 | Closed loop in sim (T1 serve, T2 flight) | 10–13 | T1, T2 |
| K5 | Optimisation (learned scoring, online speed, capture on the move, lighting) | 13–18 | T0–T2 |
| K6 | Real drone + full benchmark + paper artifacts | 19–28 | T3 |

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
- `sim/scenes/`: build **one** scene (Q2: a textured, mid-complexity object in a simple room) →
  `scene.usd`, `gt_mesh.ply`, `scene.yaml`. bisg mount for the scene (Q3).
- `sim/render_server.py` **batch mode** + `kinetix pool make` → first pool (rings, ~240 views, 10 % test).
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
- `offline/sfm.py`, `offline/densify.py` (`gsplat` first; `colmap_mvs` via the CUDA COLMAP container),
  `offline/metrics.py`, `offline/evaluate.py`, `kinetix eval`.
- `scene` tooling: `gt_observable.ply` (raycast from feasible poses).
- `scorers/coverage_oracle.py` (upper bound), `bench/matrix.py`, `bench/report.py`, `kinetix bench`.
- 3 scenes with pools.

Exit test
- Metric sanity: evaluating the GT mesh against itself gives Chamfer ≈ 0 and F-score = 1. A sphere
  offset by d gives Chamfer = d ± 1 %.
- `kinetix bench --matrix config/bench/baselines.yaml` (3 scenes × {orbit, grid, random, oracle} ×
  budgets {10, 20, 40, 80} × 3 seeds) finishes unattended and writes `report.md` with the
  F-score-vs-images curve. The oracle is ≥ every baseline at every budget (otherwise the evaluation is wrong).

## K3 — NBV core (weeks 5–10)

Deliverables
- `scorers/sfm_uncertainty.py`, `recon/depth_prior.py` (Depth Anything V2, scale fit),
  `scorers/depth_frontier.py`, `scorers/combo.py`, `select.py` with travel cost, `stop.py` gain plateau.
- `recon.pose_source: sfm | prior | gt`.
- Ablation matrix `config/bench/ablation.yaml`.

Exit test
- On the 3 K2 scenes, `kinetix_combo` reaches the dense reference's F-score@1 % (benchmark.md §2) with
  **≤ 50 %** of the images `orbit` needs (the first checkpoint toward the 50–80 % goal), mean over 3 seeds.
- Planning step (sample + score + select, excluding SfM) < 2 s at 40 frames.

## K4 — Closed loop in sim (weeks 10–13)

Deliverables
- `sim/render_server.py` **serve mode** + `rig/isaac_render.py` (T1, continuous view-sphere sampler).
- `rig/mavros.py` (T2), `planner/constraints.py` (workspace, no-fly, standoff, segment check).
- `docker/Dockerfile` (ros:jazzy-ros-base + uv venv) + `docker/compose.yaml` (`kinetix` service, host network).
- Scene loaded in a bisg scenario (`world.usd_path`), drone spawned at a standby pose.

Exit test
- T1: same scene/method/seed as T0 gives F-score within 0.05 of T0.
- T2: `kinetix run --rig mavros --method kinetix_combo` against `./bisg all headless` completes a
  20-image session without operator input, lands, and the run evaluates. Capture-pose error vs Pegasus
  GT < 0.10 m / 3°. F-score within 0.1 of T1.
- The monocular-rule test passes (no forbidden topic subscribed).

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
- Real scenes with reference GT (Q4). Real pools from dense captures (T0 on real data).
- Full benchmark: 8–10 sim objects + 3 real, all baselines + ablations, `report.md` → paper figures.
- Dataset + code release package.

Exit test: the paper's tables regenerate from `kinetix bench` with one command per table.
