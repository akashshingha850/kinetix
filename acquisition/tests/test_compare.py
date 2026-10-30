import json

import yaml

from acquisition.compare import main


def _run(root, name, exp, step, ate):
    d = root / name
    (d / "sfm").mkdir(parents=True)
    cfg = {"experiment": exp, "planner": {"tour": {"keyframe_step_m": step}, "budget": 120}}
    (d / "run.yaml").write_text(yaml.safe_dump({"config": cfg, "summary": {"frames": 10, "travel_m": 50.0, "wall_s": 300.0}}))
    (d / "sfm" / "metrics.json").write_text(json.dumps({"registered_frac": 1.0, "ate_vs_gt": {"rmse_m": ate}}))
    return d


def test_compare_sorts_by_ate_and_lists_only_the_differing_settings(tmp_path, capsys):
    a, b = _run(tmp_path, "a", "dense", 0.25, 0.4), _run(tmp_path, "b", "sparse", 0.6, 0.1)
    assert main([str(a), str(b), "--csv", str(tmp_path / "t.csv")]) == 0
    out = capsys.readouterr().out
    assert out.index("| b | sparse") < out.index("| a | dense")                 # lower ATE first
    assert "planner.tour.keyframe_step_m: 0.6 | 0.25" in out and "budget" not in out
    assert (tmp_path / "t.csv").read_text().splitlines()[1].startswith("b,sparse")
