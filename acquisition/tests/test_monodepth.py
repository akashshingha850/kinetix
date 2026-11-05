"""anchor(): the scale/shift fit of relative inverse depth to SfM point depths (testing.md §1 `recon.depth.mono`)."""
import numpy as np

from acquisition.mapping.monodepth import anchor, to_depth


def test_anchor_recovers_scale_and_shift_despite_outliers():
    rng = np.random.default_rng(0)
    Z = rng.uniform(1.0, 3.0, (60, 80))
    s0, t0 = 0.7, 0.05
    rel = (1.0 / Z - t0) / s0                             # what the network would output for depth Z
    u, v = rng.integers(0, 80, 200), rng.integers(0, 60, 200)
    z = Z[v, u] * (1 + rng.normal(0, 0.005, 200))
    z[:20] *= 2.5                                         # 10 % bad SfM points (mismatches / occluded)
    s, t, n = anchor(rel, u, v, z)
    assert abs(s - s0) < 0.02 and abs(t - t0) < 0.01 and 170 <= n <= 185
    assert np.nanmedian(np.abs(to_depth(rel, s, t) - Z) / Z) < 0.01
    assert anchor(rel, u[:10], v[:10], z[:10]) is None   # too few points
