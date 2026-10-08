# Study: monocular vs stereo in Kinetix

Date: 2026-10-08. Question: the drone carries a **ZED Mini (stereo)**, but Kinetix was planned as
**monocular**. Does that conflict? Can we evaluate both? This study answers both questions and sets the
scope. The resulting plan changes are in [../decisions.md](../decisions.md) ADR-K07 and the docs it lists.

## 1. Facts

| Fact | Value | Source |
|---|---|---|
| Stereo baseline | 63 mm | ZED Mini datasheet; bisg contract (`-P[0,3]/P[0,0]`) |
| Depth range | 0.1–15 m (bisg config: `min_depth 0.2`, `max_depth 15`) | datasheet; `bisg_isaac/docker/zed/zed.yaml` |
| Depth accuracy | < 1 % at 2 m, < 1.5 % up to 3 m, < 7 % up to 15 m | Stereolabs support/datasheet |
| Depth vs active stereo | denser than active-stereo cameras at ~0.6 m but ~2× their RMSE | Kotlyar et al., close-range study (arXiv 1903.09169) |
| bisg depth mode | `NEURAL`, depth enabled, grab `HD1080` @ 30 fps | `zed.yaml` |
| Sim | the **real ZED SDK** runs on the Isaac ZED Mini twin, so sim stereo depth is the SDK's own algorithm on rendered images (T2 only, live stream) | bisg `docs/zed-sdk-sim.md` |
| Topics already in the contract | `right/color/rect/image`, `depth/depth_registered`, `point_cloud/...`, `mapping/fused_cloud` | bisg `interface-contract.md` |
| Stereo SfM | pycolmap 4.2.1 has `Rig`, `RigConfig`, `apply_rig_config`: a left/right pair can be one rigid rig in SfM/BA with a known baseline | checked in the Kinetix venv |
| Navigation | the drone already flies on stereo: PX4 EKF2 is fed by ZED VIO on hardware | bisg plan |

## 2. Does the stereo camera break the monocular plan?

**No.** "Monocular" is a property of the *method*, not of the hardware. The left ZED image is an
ordinary rectified RGB camera, and using only it is a valid monocular experiment. Four points still matter:

1. **Reviewer question.** With stereo on board, "why not use it?" is unavoidable. The answer has to be a
   measured comparison, not an assumption. **This makes "do both" necessary, not optional.**
2. **Physics argument for monocular.** At the planned standoff (0.8–2 m), ZED depth error is about
   8–20 mm per frame (≈ 1 %). That is comparable to the F-score threshold (1 % of a ~1 m object = 10 mm).
   The stereo baseline gives a triangulation angle of only 63 mm / 1.5 m ≈ 2.4°. Multi-view
   photogrammetry between planned views has 15–30°. **Hypothesis H1:** stereo depth helps *planning*
   (metric, immediate coarse geometry) much more than it helps *final accuracy*, which multi-view SfM/MVS
   dominates. If this holds, it is the paper's central argument for a monocular method.
3. **Scale.** Monocular SfM is up to scale. Today Kinetix gets metric scale from the vehicle pose. A
   stereo rig in SfM gives metric scale from the baseline alone. That makes a clean ablation, and a real
   advantage of stereo when VIO drifts.
4. **Navigation stays stereo** in every mode (EKF2 + ZED VIO). This was already Q5. It is now stated
   explicitly: the sensing mode only concerns *planning and reconstruction inputs*.

Other effects: the ZED Mini uses rolling-shutter sensors, which is harmless with stop-and-shoot
(ADR-K06) but matters for capture on the move (K5). bisg grabs at HD1080, so the pool camera model
must match whatever resolution Kinetix captures (it reads `camera_info`, never hard-codes `K`).

## 3. Literature update (affects the novelty claim)

| Work | What it does | Relation to Kinetix |
|---|---|---|
| **ActMVS** (Pu, Han, Lian; ICRA 2026, [DOI 10.1109/ICRA57385.2026.11696006](https://doi.org/10.1109/ICRA57385.2026.11696006)) | active **scene-level** reconstruction with **monocular** MVS + view factor graph + global depth optimisation; Replica sims; "competitive with RGB-D" | closest monocular competitor. It weakens the claim "monocular active reconstruction has no direct prior". Kinetix's distinct claims: **object-centric**, **real indoor drone**, **COLMAP/3DGS pipeline output**, **measured mono-vs-stereo trade-off on one platform** |
| **FlyCo** (Feng et al.; arXiv 2601.07558, Jan 2026) | foundation-model-guided drone scanning of open-world structures (perception → shape prediction → planning) | outdoor structure scale; sensors not confirmed in the abstract. Cite as a recent active-scanning system |
| ActiveGS (2025), FisherRF (ECCV 2024) | active view selection with Gaussian splatting / Fisher information, depth-sensor based | RGB-D upper references for the scorer design |
| **FoundationStereo** (Wen et al., CVPR 2025) | zero-shot learned stereo, strong sim-to-real | the offline stereo depth source for T0 pools (the ZED SDK cannot run on pool images) |
| **VGGT** (Wang et al., CVPR 2025) | feed-forward cameras + depth + points from 1…hundreds of views in < 1 s | optional monocular depth source / fast online recon (K5 candidate) |
| Depth Pro, UniDepth v2, Metric3D v2, MoGe-2 | monocular **metric** depth | alternatives to scale-fitted Depth Anything V2 for the mono depth source (ablation) |

Action: `related_work.md` gets ActMVS and FlyCo, and its "monocular-only … has no direct prior" gap is
reworded to the narrower, defensible claim above. Superseded in depth by the full novelty search:
[reports/Kinetix novelty search.md](reports/Kinetix%20novelty%20search.md) (contribution C2 = this study).

## 4. Can we do both? Yes, as two independent axes plus presets

The plan already has one pluggable depth consumer (`depth_frontier`) and one SfM module. Stereo needs no
new architecture, only a **sensing configuration** with two independent axes:

| Axis | Key | Values |
|---|---|---|
| Depth used for **planning** | `sensing.plan_depth` | `none` · `mono` (DA-V2 scale-fit; metric models as ablation) · `stereo` (ZED SDK on T2/T3; FoundationStereo on T0/T1 pools) · `gt` (oracle) |
| Input to **reconstruction** | `sensing.recon_input` | `left` · `stereo_rig` (left+right as a pycolmap rig: metric scale, extra views) · `stereo_rig+depth` (plus TSDF fusion of stereo depth as a densifier) |

Named presets (what the paper reports):

| Preset | plan_depth | recon_input | Question it answers |
|---|---|---|---|
| `mono` (**Kinetix**) | mono | left | main method |
| `mono_noprior` | none | left | does a mono depth prior help planning at all? |
| `stereo_plan` | stereo | left | how much does stereo help *planning*? (tests H1) |
| `stereo_full` | stereo | stereo_rig+depth | best the ZED can do with our planner |
| `oracle_depth` | gt | left | upper bound of depth-assisted planning |
| `zed_mapping` (baseline, T2/T3) | — | ZED SDK spatial-mapping mesh along an orbit | what the stereo camera gives out of the box |

Isolating the two axes is what makes the comparison informative. `mono` vs `stereo_plan` measures
planning value, and `stereo_plan` vs `stereo_full` measures reconstruction value.

## 5. Research questions and how each is measured

| # | Question | Measured by |
|---|---|---|
| RQ1 | Images-to-quality `N@q` of mono vs stereo planning vs baselines | efficiency curves, `mono` vs `stereo_plan` vs `orbit` |
| RQ2 (H1) | Does stereo depth improve final accuracy beyond multi-view SfM/MVS at equal images? | `stereo_plan` vs `stereo_full` at equal N |
| RQ3 | How much of the stereo advantage does a mono depth prior recover? | gap closure `(N_noprior − N_mono)/(N_noprior − N_stereo)` |
| RQ4 | Where does each mode fail (texture-poor, glossy, thin parts, < 0.5 m standoff)? | per-scene-category breakdown, failure counts |
| RQ5 | Is sim stereo depth representative? | depth study (§6) on the same views: ZED SDK vs FoundationStereo vs DA-V2 vs GT, sim and real |

## 6. Depth study (small, early, done once per change of depth model)

On ~50 views of the K1 scenes (sim) and later ~50 real views with a reference scan:
abs-rel, RMSE and % within 1 cm, binned by distance (0.3–3 m) and surface type, for ZED SDK `NEURAL`
(T2: through the bisg stack), FoundationStereo and SGBM on the rendered pair, DA-V2 scale-fit, and one
metric mono model. Output: `docs/research/depth-study.md` + a noise model `σ(z)` per source. This makes
the T0 `stereo` depth source defensible (RQ5), and it reuses bisg's existing `zed_depth_box` approach.

## 7. Scope

In scope: both sensing families with the same planner, loop, offline pipeline and metrics; the
presets in §4; the depth study; stereo-rig SfM.

Out of scope (unless a result asks for it): new stereo matching methods, the ZED SDK running
offline on pools (SVO synthesis), active depth sensors, multi-camera rigs beyond the ZED pair, and
using ZED VIO/mapping inside Kinetix reconstruction (except the `zed_mapping` baseline).

## 8. Cost and risks

| Item | Cost / risk | Mitigation |
|---|---|---|
| Pools render the right image too | ~2× render time and disk per pool | render once (K1). The right camera is the left pose ∘ baseline (from `camera_info`) |
| FoundationStereo install | repo + weights, research licence, GPU memory | install at K3 start. SGBM (OpenCV, already installed) is the fallback |
| Sim stereo ≠ real stereo | T0 stereo source is not the ZED SDK | depth study (RQ5) quantifies it. T2/T3 use the real SDK |
| Mono may simply lose to stereo | weakens "mono is enough" | the paper still holds as a measured trade-off (cost/weight of a stereo camera vs images saved). RQ3 framing |
| Novelty overlap (ActMVS) | reviewers | object-centric + real drone + pipeline compatibility + mono-vs-stereo study |

## Sources

- ZED Mini depth spec: [Stereolabs support](https://support.stereolabs.com/hc/en-us/articles/1500008533841-For-the-depth-sensing-what-is-the-range-for-the-ZED-cameras-in-terms-of-measuring-distances-and-the-accuracy-of-the-detection), [datasheet](https://www.generationrobots.com/media/zed-mini-camera-datasheet.pdf), [close-range study](https://arxiv.org/pdf/1903.09169)
- [ActMVS, arXiv 2606.01367](https://arxiv.org/abs/2606.01367) · [FlyCo, arXiv 2601.07558](https://arxiv.org/abs/2601.07558)
- [FoundationStereo, CVPR 2025](https://openaccess.thecvf.com/content/CVPR2025/html/Wen_FoundationStereo_Zero-Shot_Stereo_Matching_CVPR_2025_paper.html) · [VGGT, CVPR 2025](https://arxiv.org/abs/2503.11651)
- [Depth Pro](https://arxiv.org/abs/2410.02073) · [Survey on monocular metric depth](https://arxiv.org/pdf/2501.11841) · [MoGe-2](https://arxiv.org/pdf/2507.02546)
