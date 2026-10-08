# Publication plan

What Kinetix publishes, what each paper claims, which experiment backs each claim, and when. Grounded
in the novelty search ([research/reports/Kinetix novelty search.md](research/reports/Kinetix%20novelty%20search.md),
2026-10-08). Experiments: [benchmark.md](benchmark.md). Build phases: [roadmap.md](roadmap.md).
Week 0 = 2026-10-08.

## 1. Positioning

Kinetix is a **measurement paper with a principled monocular method attached**. It is not "a better
planner that saves images": ~53–72 % savings over fixed patterns are already published. The question
it answers, on one planner and one drone, is *what depth is actually for in object photogrammetry*
(choosing views or building the model), and how far NBV conclusions drawn in replay survive PX4 dynamics
and a real room.

- **Frame against ObjView-Bench (Bonn, 2026) as its complement, not a rival**: aerial vs tabletop arm,
  photoreal RGB + SfM/3DGS vs depth coverage, monocular vs depth sensing. The Bonn group (SCVP, MA-SCVP,
  ObjView-Bench, ActiveGS) is the likeliest reviewer pool.
- **Frame against Hestia (WACV 2026)**: Kinetix contributes quantitative real-flight geometry vs scanned GT,
  a standard photogrammetry output and a sensing study, all of which Hestia lacks.
- **Frame against ActMVS (2026)**: object scale, real flight, and scale anchored to SfM instead of naive monocular
  depth. ActMVS's own ablation (single-image depth: Chamfer ~2.4 → ~46 cm) is the motivation for C1.

## 2. Contributions (paper wording) and the evidence each needs

| # | Contribution | Evidence (benchmark matrix → table/figure) | Phase |
|---|---|---|---|
| C1 | A monocular next-best-view method for single-object photogrammetry that anchors a relative-depth foundation model to the scale of the incremental SfM reconstruction and fuses it into a TSDF frontier score, combined with an SfM-uncertainty score. It needs no depth sensor or metric-depth model for planning. | `bench/ablation`: anchoring (`mono` vs `mono_metric_noanchor` vs `mono_stereo_anchor` vs `none`), scorer terms alone vs combined → **Table 3** | K3 |
| C2 | The first controlled study, to our knowledge, of monocular versus small-baseline stereo sensing within one NBV planner. It separates the value of depth for *planning* from its value as *input to reconstruction*. | `bench/sensing`: presets `mono`, `mono_noprior`, `stereo_plan`, `stereo_full`, `oracle_depth` on T0, plus the T3 subset → **Table 2, Fig. 3** | K3, K6 |
| C3 | N@q, an images-to-quality metric: the number of images a capture strategy needs to reach a fixed fraction of the F-score of a dense capture of the same object. It is reported alongside Chamfer, F-score and coverage AUC. | sensitivity to q ∈ {0.9, 0.95} and to the reference density (120/240/360) and agreement with K = 5/30 tables → **Table 1, supp.** | K2 |
| C4 | A four-tier evaluation protocol (photoreal view-pool replay, continuous rendering, PX4 SITL flight, real indoor flight) used to quantify how NBV method rankings and image savings change as realism increases. | `bench/tiers`: Kendall's τ of rankings T0↔T1↔T2↔T3, ΔN@q per tier → **Table 4, Fig. 4** | K4, K6 |
| C5 | A real-flight evaluation on a PX4/Jetson quadrotor with scanned ground truth, plus released view pools, digital twins, evaluation code and baselines. | `bench/real` (Chamfer, F-score in relative and absolute mm, N@q for orbit_3ring / fvs / mono / stereo_plan) → **Table 5, Fig. 5**; the release (§7) | K6 |

**Result, not contribution**: "at equal quality Kinetix needs X % fewer images than the best fixed pattern
and Y % fewer than the best adaptive baseline, consistent with prior reports (~53–72 %)."

## 3. Claims we must not make

Already held by others (novelty report, claims table): "50–80 % fewer images" as a novelty; "first monocular
active (aerial) reconstruction" (ActMVS, MACARONS); "first foundation depth model in NBV/exploration" (Next Best
Sense, FrontierNet); "first RGB-only object NBV" (MACARONS, DM-OSVP++, PUN, Hestia); "first RGB-only object NBV
on a real drone" (Hestia). Every "first" in the paper carries "to our knowledge" and must survive the final
novelty sweep (§6).

## 4. Paper outline (RA-L format: 6 pages + up to 2 extra; check the current author guide)

| Section | Content | Source |
|---|---|---|
| I. Introduction | "what is depth for" question, C1–C5 | this file §1–2 |
| II. Related work | NBV/active recon, foundation depth in planning, aerial capture planning, benchmarks | `research/related_work.md` |
| III. Method | SfM-anchored monocular depth, TSDF frontier + SfM uncertainty, selection with travel cost, 4-DoF constraints | architecture.md §5–8 |
| IV. Evaluation protocol | tiers T0–T3, pools (rings + Tammes), N@q, metrics, baselines | benchmark.md |
| V. Results | Tables 1–5, Figs. 2–5 | `kinetix bench` outputs |
| VI. Discussion | H1 outcome, failure cases (texture-poor, glossy, thin), limits | study RQ4 |
| Supplement / video | per-object curves, N@q sensitivity, flight video | bench reports |

Figures: **Fig. 1** system and tiers. **Fig. 2** F-score-vs-images efficiency curves per scene category.
**Fig. 3** the 2×2 sensing result. **Fig. 4** ranking shift across tiers (bump chart). **Fig. 5** real flight:
trajectory, chosen views and error heat map vs GT. **Fig. 6** an example NBV sequence. Every table and figure is
regenerated from a `config/bench/*.yaml` matrix with one command (roadmap K6 exit test).

Baselines reviewers expect (benchmark.md §1): dense multi-ring orbit, farthest-view sampling, volumetric
information gain, FisherRF, one learned planner (MA-SCVP), and explore-then-exploit if time allows.

## 5. Timeline and gates

| Gate | Week (date) | Must be true | Decision |
|---|---|---|---|
| G1 | 5 (~2026-11-12) | K2 done: baseline matrix incl. `fvs`/`volumetric_ig`, N@q works, Tammes pool exists | — |
| G2 | 10 (~2026-12-17) | K3 done: anchoring ablation and T0 sensing table | **C1 go/no-go** (§6). Optional early arXiv/workshop (P0) |
| G3 | 13 (~2027-01-07) | K4 done: T1 and T2 runs, first tier-transfer numbers | freeze method; only bug fixes to the planner after this |
| G4 | 24 (~2027-03-25) | real flights and scans done (depends on bisg Phase 5) | final novelty sweep #1; choose plan A or B |
| G5 | 28 (~2027-04-22) | all tables regenerate, draft complete, release package staged | submit |

Venue plan:

- **P1, main paper → IEEE RA-L.** Plan A: journal-only submission at G5 (~May 2027). Plan B (if real flights
  slip past G4): RA-L with the **ICRA 2028** presentation option, whose RA-L deadline is typically mid-September
  (≈ week 49, verify on the ICRA 2028 site). The RA-L + IROS 2027 option (deadline typically ~1 March, ≈ week 21)
  is reachable only if K6 real flights finish early. Do not compress the real-flight tier to hit it.
- **P0 (optional, decided at G2) → workshop paper or arXiv technical report** on T0 results (C1–C3), at a spring 2027
  ICRA workshop on active perception/3D reconstruction. Purpose: timestamp the sensing study before ObjView-Bench
  or another group fixes the protocol. Check the workshop's dual-submission rules against RA-L first.
- **P2, follow-up → benchmark/dataset paper** (3DV / CVPR, or a datasets track) once the object set reaches hundreds
  of items with learned baselines, following the GenNBV / UrbanScene3D precedent. Late 2027.
- **P3 (optional) → ISPRS P&RS**: a photogrammetry-led extension (C2 + C3 deep dive, real-object accuracy).

arXiv: post P1 at submission (RA-L permits preprints). The code repository goes public the same day.

## 6. Risks and contingencies

| Risk | Signal | Response |
|---|---|---|
| C1 fails: SfM anchoring does not beat the metric-depth or stereo-anchored variants | G2 ablation | Drop C1 as a contribution. Keep the best variant as "the Kinetix planner" and lead with C2–C4 (still publishable as a measurement paper) |
| H1 fails: stereo helps reconstruction as much as planning | G2 sensing table | Report it as a quantified cost-of-monocular result. C2 stays novel either way |
| ObjView-Bench releases code/pools first | watch list (todo.md) | the Tammes pool and shared objects already map onto its protocol. Add its numbers as a cross-reference |
| A new preprint takes C2 or C4 | novelty sweeps at G2, G4, G5 | narrow the claim wording. The real-flight C5 remains |
| bisg hardware (Phase 5) late | G4 | Plan B venue timing. Never submit without quantitative real flights |
| No scanned GT for real objects | K6 start | arrange the scanner by G3 (Q4). A dense 300+ image reference photogrammetry is the fallback |
| Asset licences block pool release | K1 object choice | prefer redistributable assets. Release pool *generators* when the images cannot be released |

## 7. Release plan (at P1 submission)

| Artifact | Form | Notes |
|---|---|---|
| code | GitHub `kinetix` repo, permissive licence (choose at G4) | includes baselines and external wrappers |
| view pools | Zenodo (DOI), per-object archives | only redistributable assets. Others ship as scene + generator script |
| digital twins | scene USD + GT meshes + `scene.yaml` | same licence check |
| real data | real pools (left/right/ZED depth), scans, flight logs | consent/space permissions for the room |
| evaluation | `kinetix eval` / `kinetix bench` + the paper's matrices | tables regenerate with one command |
| sim environment | pointers to bisg_isaac images + the Kinetix Docker image | pinned versions in `run.yaml` |

## 8. Pre-submission checklist

- [ ] Every number traces to a `results.csv` and a `run.yaml` with git sha (benchmark.md §5).
- [ ] Image savings computed against the best fixed and best adaptive baselines.
- [ ] Each "first" survives novelty sweep #2 (G5), with arXiv searches for the last 3 months.
- [ ] Real-flight results are numeric, with scanned GT, and include the C2 subset.
- [x] `related_work.md` references cross-checked against Crossref/arXiv, DOI on every entry (2026-10-08). Re-run the check before submission.
- [ ] The supplementary video shows T2 and T3 flights.
- [ ] Release package is staged and links work from a clean machine.
- [ ] Authorship and acknowledgements agreed (TBD).
