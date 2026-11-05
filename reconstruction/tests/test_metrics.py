"""reconstruction/metrics.py sanity (roadmap K2 exit test): GT against itself is perfect; a sphere offset by d has Chamfer d."""
import numpy as np
import trimesh

from reconstruction.metrics import score


def test_gt_against_itself():
    m = trimesh.creation.icosphere(subdivisions=4, radius=0.5)
    P, _ = trimesh.sample.sample_surface(m, 20000, seed=0)
    r = score(P, m.vertices, m.faces, P, diag=1.7)
    assert r["chamfer_mm"] < 0.5 and r["fscore"]["tau_1pct"] == 1.0 and r["fscore"]["tau_0.5pct"] == 1.0


def test_offset_sphere_chamfer_is_the_offset():
    m = trimesh.creation.icosphere(subdivisions=5, radius=0.5)
    gt, _ = trimesh.sample.sample_surface(m, 40000, seed=1)
    d = 0.02
    P = gt * (0.5 + d) / 0.5                                     # the same points pushed out by d
    r = score(P, m.vertices, m.faces, gt, diag=1.7)
    assert abs(r["chamfer_mm"] - 1000 * d) <= 0.01 * 1000 * d + 0.3   # d ± 1 % (+ the mesh's facetting)
    assert r["fscore"]["tau_0.5pct"] == 0.0 and r["fscore"]["tau_1pct"] == 0.0   # 8.5 / 17 mm < 20 mm


def test_no_points_scores_zero():
    m = trimesh.creation.box()
    r = score(np.zeros((0, 3)), m.vertices, m.faces, m.vertices, diag=1.7)
    assert r["fscore"]["tau_1pct"] == 0.0 and r["chamfer_mm"] is None
