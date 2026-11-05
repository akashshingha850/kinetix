# Research questions, contributions and novelty

Written 2026-11-02. **Proposed (ADR-K14)**: it refines ADR-K08's C1–C5 and does not replace them until both authors accept it.
It is based on the novelty search ([research/reports/kinetix-novelty-search.md](research/reports/kinetix-novelty-search.md), 2026-10-08),
the first dense benchmark results ([../methodology.md](../methodology.md) §4) and a targeted check on 2026-11-02 (below). Experiments:
[benchmark.md](benchmark.md). Venue and dates: [publication-plan.md](publication-plan.md). References `[n]` are in
[research/related_work.md](research/related_work.md).

## 1. Why refine C1–C5

1. **Five contributions are too many for six RA-L pages.** Two of them are instruments, not findings. N@q (C3) is a metric, and its
   definition is still open: the Chair reference never reaches 0.9 of its own F (methodology V1). The tier protocol (C4) has a
   peer-reviewed precedent: Kadian et al. [55] measure sim→real predictivity as the correlation of method results across sim and
   real, for navigation in RA-L. C4 is still new for NBV photogrammetry, but as a measurement, not as a protocol idea.
2. **Kinetix's own data point at a stronger finding.** The coverage oracle loses to a simple GT-free baseline (`orbit`, `fvs`,
   `volumetric_ig`, `depth_coverage`) in 23 of 24 cells (V4). Its objective does predict completeness (ρ 0.76, V3), but 13–82 % of its views at 10–20 images do not register in SfM. At
   small budgets, *registration*, not coverage, decides the result. The surveyed NBV papers score their own reconstruction, not an
   unmodified COLMAP pipeline (novelty search), so they cannot see this effect.
3. **C1 is unbuilt**, its go/no-go is at G2 (2026-12-17), and the paper must stand if it fails (publication-plan §6). The question
   set below keeps the paper alive in every branch of that decision.

## 2. The question

> **What must a next-best-view planner know to choose images for photogrammetry, and can a monocular drone know it?**

| RQ | Question | Hypothesis (falsifiable) | Measured by | Old claim |
|---|---|---|---|---|
| **RQ1** | How much of a planner's predicted gain survives SfM registration, and does a registration-aware admissibility rule recover it? | H1: at ≤ 20 images, unregistered views explain most of the gap between the coverage oracle and the best simple baseline; with a registration rule, the oracle bounds every GT-free method (V4 green). | registered fraction and F@1 % vs N, oracle with / without the rule; ρ(predicted, registered) ≥ 0.7 (V3) | new (from V4) |
| **RQ2** | What is depth for in NBV photogrammetry: choosing views or building the model? | H2: stereo depth improves *planning* (N@q) far more than *reconstruction* (F at a fixed N), because a 63 mm baseline subtends ~2.4° at 1.5 m against 15–30° between planned views. | 2×2: `plan_depth` {none, mono, stereo, gt} × `recon_input` {left, stereo_rig+depth} | C2 |
| **RQ3** | Can a monocular planner recover the planning value of depth, with scale taken from its own SfM? | H3: DA-V2 anchored to the SfM points [51, 52] reaches the N@q of `stereo_plan` within its CI, and beats a metric-depth model without anchoring [35, 39]. | anchoring ablation `mono` / `mono_metric_noanchor` / `mono_stereo_anchor` / `none`; V5a AbsRel | C1 |
| **RQ4** | Do the conclusions of RQ1–RQ3 survive PX4 dynamics and a real flight? | H4: method rankings keep Kendall τ ≥ 0.8 from T0 to T2, and the sign of each RQ effect holds on T3 with scanned GT. | τ T0↔T1↔T2↔T3 (the predictivity measure of [55], here on rankings); ΔN@q per tier; real Chamfer / F | C4 + C5 |

N@q stays the headline *metric* (the number of images needed to reach q · F_ref) and goes into the method section. It stops being
a contribution, which removes the weakest claim and the open definition from the claims list. Decision needed (V1): q relative to
the scene's own F_ref, which is what §6 of methodology.md already computes.

## 3. Contributions (paper wording, conditional where marked)

| # | Contribution | Holds if | Fallback |
|---|---|---|---|
| **K-A** | A measurement, through an unmodified COLMAP SfM→MVS pipeline, of how much few-view NBV gain is lost to image registration, and a registration-aware admissibility rule that every compared method shares. | RQ1: the rule turns V4 green and the effect is large at ≤ 20 views; it survives the novelty sweep (below) | report the loss as a finding without a rule |
| **K-B** | The first controlled study, to our knowledge, of monocular vs small-baseline stereo sensing inside one NBV planner, separating depth for planning from depth for reconstruction. | always: either sign of H2 is a result | — |
| **K-C** | A monocular NBV planner that anchors a relative-depth foundation model to the scale of its own incremental SfM, needing no depth sensor or metric-depth model. | RQ3 go at G2 | drop K-C; the best variant is "the planner", and K-A/K-B/K-D carry the paper |
| **K-D** | A tier-transfer measurement (pool replay → render → PX4 SITL → real PX4/Jetson flight with scanned GT) of how NBV rankings and savings change with realism, with the released pools, twins and code. | RQ4: real flights are quantitative (G4) | Plan B venue timing (publication-plan §5) |

The image saving stays a *result*: "X % fewer images than the best fixed and Y % fewer than the best adaptive baseline, consistent
with ~53–72 % reported before [42, 43, 44]".

## 4. Novelty status (check of 2026-11-02, extends the 2026-10-08 search)

| Claim | Status | Closest prior work | How to word it |
|---|---|---|---|
| K-A registration loss in NBV photogrammetry | **open, needs a dedicated sweep** | matchability prediction [54]; next-view choice inside SfM [1]; viewpoint limit of matching [53]; MP-SfM (2025 preprint) picks the next view by matcher scores; Mostegel et al. [2] predict MVS quality online | narrow: "measured through unmodified COLMAP, at few views, for drone NBV"; never "first registration-aware planner" |
| K-B mono vs stereo inside one planner | open (2026-10-08) | ActMVS depth ablation [35] (now ICRA 2026); Next Best Sense scale from a depth camera [38] | as in C2 |
| K-C SfM-anchored relative depth for object NBV | open (2026-10-08) | Murre [40] (offline, no planning); Next Best Sense [38] (sensor scale); FrontierNet [39] (metric depth) | as in C1; "scale from the reconstruction itself" |
| K-D tier transfer for NBV | open for NBV photogrammetry; **the idea is not new** | Kadian et al. [55] (navigation, sim→real correlation); ObjView-Bench [46] (arm, preprint) | cite [55] as the method; claim the measurement, not the protocol |

Every "first" keeps "to our knowledge" and must survive the sweeps at G2, G4 and G5 (publication-plan §3, §8). The K-A sweep
searches for: NBV or view planning with SfM registration, matchability or feature-overlap constraints, 2016–2026, robotics
(ICRA/IROS/RA-L/T-RO) and photogrammetry (ISPRS, P&RS, Remote Sensing) venues.

## 5. What each RQ needs, in order

The steps, dates and exit criteria are in [../roadmap.md](../roadmap.md) ("Steps to submission"). In short: frozen evaluation
(V1, V2) → RQ1 (registration model, V3/V4) → RQ3 components (V5) → RQ2 + RQ3 on T0 (V6, G2) → scale-up to 8–10 scenes → RQ4 in sim
(V7, V8, flight gate, T2) → RQ4 real (T3, G4) → write and release (G5). RQ1 comes first because its rule is shared by every adaptive
method: it changes every number after it.

## 6. Two papers: planning now, reconstruction later (proposed 2026-11-02, needs both authors)

The RQ2 2×2 already splits along the line between the two authors: `plan_depth` (rows) is planning, `recon_input` (columns) is
reconstruction. Each claim goes in exactly one paper.

| | **Paper A: planning** (conference) | **Paper B: reconstruction** (co-author, later) |
|---|---|---|
| Question | what must the planner know to choose images (RQ1, RQ3, RQ2 rows, RQ4 on T0–T2) | what to build the model from (RQ2 columns: left vs stereo + depth as input; densifiers; metric scale) |
| Contributions | K-A registration loss + rule; K-C if G2 is a go; K-B reduced to *depth for choosing views*; K-D on T0→T1→T2 | its own; it may use A's captures and must cite A |
| Reconstruction | **fixed and frozen at S1, not a contribution**: unmodified COLMAP incremental SfM [1] + PatchMatch MVS and fusion [59], Sim(3) alignment to GT [60], accuracy / completeness / F-score at τ [56], registered fraction | the subject of the paper: MVS vs 3DGS [26], stereo / ZED depth input, scale |
| Data | T0 pools, T1, T2 (S1–S6); T3 only if S7 lands before the deadline | the released captures + the real flights of S7 (scanned GT) |
| Must not report | a densifier or `recon_input` comparison | planner rankings or image savings as its result |

**Venue for A: IROS 2027, paper deadline 2027-03-01** (IEEE RAS listing, 2026-11-02; confirm on the IROS 2027 site; 6 pages + extra,
RA-L option open). Roadmap S6 delivers T2 by 2027-02-11, which leaves 18 days of results writing. So intro, related work, method and
protocol are written from G2 (2026-12-17), and S1–S6 slip by no more than a week. If S6 slips further, the fallback is ICRA 2028
(ICRA 2027's deadline was 2026-09-15, so expect ~2027-09-15), which also lets T3 in.

What this changes, on acceptance: publication-plan §5's P1 (RA-L, all claims, May 2027) becomes Paper A at IROS. The real flight
(C5 / RQ4-T3) moves to Paper B or to a journal extension of A. K-B loses its "separates the two uses of depth" wording, because only A + B
together show the full 2×2. A cites B's preprint, or reports the columns as future work. Dual publication: B's primary claims must be
new relative to A (IEEE policy on prior publication), and B cites A for the captures and the protocol. Authorship order for each
paper is decided by the authors (publication-plan §8).
