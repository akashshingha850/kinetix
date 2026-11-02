"""The method table (planner/methods.py): what a run is flagged as reading, per method and rig."""
from acquisition.planner.methods import METHODS, uses_gt


def test_uses_gt_by_method_and_rig():
    assert uses_gt("coverage_oracle", "mavros")                     # the GT surface on any rig
    assert uses_gt("volumetric_ig", "pool") and uses_gt("depth_coverage", "isaac")   # rendered GT depth
    assert not uses_gt("volumetric_ig", "mavros")                   # ZED depth
    assert not any(uses_gt(m, r) for m in ("roi_coverage", "fvs", "orbit_rings", "tour") for r in ("pool", "mavros"))


def test_every_method_has_a_known_loop_and_surface():
    assert all(m.loop in ("nbv", "tour", "surface") and m.surface in ("box", "gt", "depth") for m in METHODS.values())
