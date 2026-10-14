"""Stop rule: image budget, coverage target, or gain plateau (architecture.md §4 stop.py)."""
from __future__ import annotations


class Stop:
    def __init__(self, budget: int, coverage_target: float = 0.95, min_gain: float = 0.005, patience: int = 2):
        self.budget, self.coverage_target, self.min_gain, self.patience = budget, coverage_target, min_gain, patience
        self._low = 0
        self.reason = None

    def done(self, recon, n_frames: int, last_gain: float | None = None) -> bool:
        if n_frames >= self.budget:
            self.reason = "budget"
        elif recon is not None and recon.coverage >= self.coverage_target:
            self.reason = "coverage_target"
        elif last_gain is not None:
            self._low = self._low + 1 if last_gain < self.min_gain else 0
            if self._low >= self.patience:
                self.reason = "gain_plateau"
        return self.reason is not None
