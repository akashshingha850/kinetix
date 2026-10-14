"""Scorers: score(cands, recon) -> gains >= 0 (interfaces.md §3). Selected by name (`method`)."""
from __future__ import annotations

import numpy as np

from kinetix.core.types import Candidate


class RoiCoverageScorer:
    """Photogrammetric coverage gain on the ROI proxy surface (recon/coverage.py). GT-free: reads only the scene ROI and
    the frames captured so far. A stand-in until the K3 scorers (sfm_uncertainty, depth_frontier, combo) exist."""
    name = "roi_coverage"
    uses_gt = False

    def score(self, cands: list[Candidate], recon) -> np.ndarray:
        return np.array([recon.gain(c.pose) for c in cands])


class RandomScorer:
    name = "random"
    uses_gt = False

    def __init__(self, seed: int = 0):
        self.rng = np.random.default_rng(seed)

    def score(self, cands: list[Candidate], recon) -> np.ndarray:
        return self.rng.uniform(0.01, 1.0, len(cands))


SCORERS = {"roi_coverage": RoiCoverageScorer, "random": RandomScorer}
