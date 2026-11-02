# Kinetix docs: start here

One question per document. Read in the order of your role, and stop when you have what you need.

## Reading paths

| You want to | Read, in order |
|---|---|
| understand the project in 15 min | [paper/research-questions.md](paper/research-questions.md) §2 → [architecture.md](architecture.md) §1, §3, §3a → [roadmap.md](roadmap.md) "Steps to submission" |
| work on acquisition (planner, rigs, flight) | [architecture.md](architecture.md) §3a, §4 → [acquisition/architecture.md](acquisition/architecture.md) → [interfaces.md](interfaces.md) → [acquisition/todo.md](acquisition/todo.md) |
| work on reconstruction / evaluation | [data-format.md](data-format.md) §3 → [reconstruction/pipeline.md](reconstruction/pipeline.md) → [methodology.md](methodology.md) §4 → [reconstruction/todo.md](reconstruction/todo.md) |
| write or review the paper | [paper/research-questions.md](paper/research-questions.md) → [methodology.md](methodology.md) (§2.1 has the evidence for every stage) → [paper/benchmark.md](paper/benchmark.md) → [paper/publication-plan.md](paper/publication-plan.md) |
| set up a machine | [setup.md](setup.md) → [testing.md](testing.md) |

## Every document, one line each

| Doc | Answers | Kind |
|---|---|---|
| [architecture.md](architecture.md) | what the system is, tiers T0–T3, module map, the method × rig table (§3a) | reference |
| [methodology.md](methodology.md) | the method stage by stage (M0–M9), why each stage (§2.1, cited), validation levels V0–V8, statistics, threats | reference |
| [interfaces.md](interfaces.md) | Python protocols, ROS topics, frames (change here first) | contract |
| [data-format.md](data-format.md) | what a run directory, pool and bench folder contain | contract |
| [decisions.md](decisions.md) | why things are as they are (ADR-K01…K14) | record |
| [roadmap.md](roadmap.md) | the steps to submission and the K-phase deliverables | plan |
| [testing.md](testing.md) | test levels and markers | reference |
| [setup.md](setup.md) | install, environment, data root | how-to |
| [collaboration.md](collaboration.md) | who owns what, branches, data sync | how-to |
| [acquisition/](acquisition/) | planner loop, rooms (tour/explore), ROS pipeline, acquisition todo | area |
| [reconstruction/](reconstruction/) | offline pipeline, reconstruction todo | area |
| [paper/research-questions.md](paper/research-questions.md) | RQ1–RQ4, contributions, novelty status (ADR-K14, proposed) | paper |
| [paper/benchmark.md](paper/benchmark.md) | baselines, metrics, matrices | paper |
| [paper/publication-plan.md](paper/publication-plan.md) | venue, gates G1–G5, risks, release | paper |
| [paper/research/related_work.md](paper/research/related_work.md) | the bibliography (DOI-checked; [54], [58]–[61] still to check) | paper |
| [paper/research/reports/](paper/research/reports/) | novelty search 2026-10-08, room scoping review | evidence |
| [paper/research/mono-vs-stereo.md](paper/research/mono-vs-stereo.md) | the sensing study design (RQ2) | paper |
| [paper/research/idea.md](paper/research/idea.md), [work_plan.md](paper/research/work_plan.md) | the original pitch and plan (2026-10) | history, superseded |

The two `todo.md` files are lab notebooks: dated results, newest first under each heading. A number cited in a reference doc must
point to a run directory or a `results.csv`, never only to a todo entry.

## Debug a run

Each step has a gate: do not start the next until `kinetix check` says `ok` for this one.

| Debug aid | What it does | Where |
|---|---|---|
| `WARN` lines in `log.txt` | every frame and step is checked as it is written (finite poses, unit quaternions, the image/depth files exist, finite gains), and Python/numpy warnings land here too, not only on stderr; `summary.warnings` counts them | always on (kinetix/checks.py, runlog.py) |
| `stop_reason: error` | a crash still closes the run: traceback in `log.txt`, `summary.error` | always on |
| `--strict` | the first NaN (`np.seterr(invalid="raise")`), Python warning or check problem raises, so the traceback points at the line that made it. On `mavros` it ends the flight and the rig lands | `kinetix run ... --strict` |
| `kinetix check <runs>` | the same checks offline + stop reason, frame count, `uses_gt` vs the method table; notes the failed moves and the reached-vs-target and pose-vs-GT error | after every run, every tier |

1. **Same method, simplest rig first, strict.** `uv run kinetix run --scene <scene> --method <m> --rig dry --strict planner.budget=5`,
   then `kinetix check <run>`. If it fails here, the fault is in the planner, not in ROS, Isaac or PX4 ([architecture.md](architecture.md)
   §3a lists which rig each method can use).
2. **Read the run directory** ([data-format.md](data-format.md) §3), in this order: `grep -E "WARN|ERROR" log.txt`, then `log.txt` (one
   line per step: chosen view, gain, feasible count, pose error), `steps.jsonl` (`top5` candidates, `n_feasible`, `error` of a failed
   move), `frames.jsonl` (`pose` vs `gt_pose`), `run.yaml` (resolved config, `uses_gt`, git sha, `summary`).
3. **Then climb the tiers**: `pool` (T0, rendered views, `--strict`) → `isaac` (T1, `--strict`) → `mavros` on the sim (T2) → real (T3).
   On T2/T3 run without `--strict` (a warning should not land the drone) and gate on `kinetix check` afterwards. A result that changes
   between two tiers is caused by what the higher tier adds: continuous poses, then dynamics and the ZED twin, then the real sensors.
4. **Score it**: `uv run kinetix eval <run>` (SfM, then metrics against the GT). `uv run python -m acquisition.compare <runs>` puts
   runs side by side with the config keys that differ.
