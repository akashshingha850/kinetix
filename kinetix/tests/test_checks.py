"""Run-directory checks (kinetix/checks.py) and how RunLog reports them."""
import warnings

import numpy as np
import pytest

from kinetix.checks import frame_problems, run_problems
from kinetix.runlog import RunLog
from kinetix.types import Frame, Intrinsics, Pose

K = Intrinsics(width=640, height=480, fx=500.0, fy=500.0, cx=320.0, cy=240.0)
P = Pose(np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))


def frame(image="frames/000000.png", p=(0.0, 0.0, 0.0)):
    return Frame(idx=0, t=0.0, image=image, K=K, target=P, pose=Pose(np.array(p), P.q), gt_pose=P, travel_m=0.0)


def test_frame_problems(tmp_path):
    (tmp_path / "frames").mkdir()
    (tmp_path / "frames/000000.png").write_bytes(b"x")
    assert frame_problems(frame().to_dict(), tmp_path) == []
    probs = frame_problems(frame(image="frames/missing.png", p=(np.nan, 0, 0)).to_dict(), tmp_path)
    assert any("pose not finite" in p for p in probs) and any("missing" in p for p in probs)


def test_runlog_counts_problems_and_warnings(tmp_path):
    log = RunLog(tmp_path, "s", "orbit", "dry", 0, {}, uses_gt=False)
    log.frame(frame(image="frames/missing.png"))
    warnings.warn("kinetix test warning", stacklevel=1)
    assert log.close("budget")["warnings"] == 2
    assert "WARN UserWarning: kinetix test warning" in (log.dir / "log.txt").read_text()
    probs, _ = run_problems(log.dir)
    assert any("missing" in p for p in probs) and any("2 warnings" in p for p in probs)


def test_strict_raises_and_unclosed_run_is_caught(tmp_path):
    log = RunLog(tmp_path, "s", "orbit", "dry", 0, {}, uses_gt=False, strict=True)
    with pytest.raises(RuntimeError, match="--strict"):
        log.frame(frame(image="frames/missing.png"))
    assert "no summary" in run_problems(log.dir)[0][0]   # crashed before close
    log.close("error")
