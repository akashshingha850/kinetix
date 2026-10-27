"""A dead rig must abort the session instead of timing out forever (2026-10-22 run: the rclpy context died, /clock froze,
every move hit its 20 min wall backstop and the planner kept blocking candidates for 15 h)."""
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from kinetix.core.config import ROOT, load_config, load_scene
from kinetix.core.geometry import camera_pose
from kinetix.core.runlog import RunLog
from kinetix.core.types import Intrinsics, RigError
from kinetix.planner.loop import run_session
from kinetix.recon.coverage import CoverageState
from kinetix.rig.dry import DryRig
from kinetix.rig.mavros import MavrosRig

SCENE = ROOT / "config" / "scenes" / "simple_room_blocks.yaml"


class _StuckRig(DryRig):
    """Every move times out, like a rig whose clock froze."""
    def __init__(self, *a):
        super().__init__(*a)
        self.calls = 0

    def capture_at(self, target, idx, pool_id=None):
        self.calls += 1
        raise RigError("timeout", "not settled")


def test_consecutive_timeouts_stop_the_session(tmp_path):
    cfg = load_config([], ["planner.max_consecutive_timeouts=4", "planner.budget=50"])
    scene = load_scene(SCENE)
    cam = cfg["camera"]
    recon = CoverageState(scene.roi, Intrinsics(**{k: cam[k] for k in ("width", "height", "fx", "fy", "cx", "cy")}))
    log = RunLog(tmp_path, scene.name, "roi_coverage", "dry", 0, cfg, uses_gt=False)
    rig = _StuckRig(cfg, scene, log.dir)
    out = run_session(cfg, scene, rig, recon, "roi_coverage", log)
    log.close(out["stop_reason"])
    assert out["stop_reason"] == "stalled" and rig.calls == 4


def _rig(**kw):
    """A MavrosRig shell without ROS: only what _check_alive / _stream_loop touch."""
    r = object.__new__(MavrosRig)
    r.rc = {"silent_timeout_s": 30.0, "setpoint_hz": 200}
    r._rclpy = SimpleNamespace(ok=lambda: True)
    r._dead, r._rx_wall, r.started, r.state = None, time.time(), True, SimpleNamespace(armed=True, mode="OFFBOARD")
    r.__dict__.update(kw)
    return r


def test_check_alive_ok():
    _rig()._check_alive()


@pytest.mark.parametrize("kw, msg", [
    ({"_dead": "setpoint stream died: boom"}, "boom"),
    ({"_rclpy": SimpleNamespace(ok=lambda: False)}, "context"),
    ({"_rx_wall": time.time() - 100}, "no mavros state"),
    ({"state": SimpleNamespace(armed=True, mode="AUTO.LOITER")}, "left OFFBOARD"),
])
def test_check_alive_raises(kw, msg):
    with pytest.raises(RigError, match=msg) as ei:
        _rig(**kw)._check_alive()
    assert ei.value.kind == "aborted"


def test_silence_ignored_before_start():
    _rig(started=False, _rx_wall=time.time() - 100)._check_alive()


def test_wait_aborts_on_dead_rig():
    r = _rig(_dead="setpoint stream died: boom")
    r.rc["min_rtf"] = 0.1
    r.now = lambda: 0.0                                   # frozen clock: only the liveness check can end this wait
    t0 = time.time()
    with pytest.raises(RigError, match="boom"):
        r._wait(lambda: False, 120.0, "never")
    assert time.time() - t0 < 2.0


def test_stream_thread_crash_marks_rig_dead():
    class Pub:
        def publish(self, m):
            raise RuntimeError("publisher's context is invalid")
    r = _rig(sp=(np.array([0.0, 0.0, 1.0]), 0.0), path=None, _stream_on=True, sp_pub=Pub(), cmd=None, _cmd_t=None,
             _PoseStamped=lambda: SimpleNamespace(header=SimpleNamespace(), pose=SimpleNamespace(
                 position=SimpleNamespace(), orientation=SimpleNamespace())),
             node=SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                 to_msg=lambda: None, nanoseconds=0))))
    r.rc["cruise_mps"] = 1.0
    t = threading.Thread(target=r._stream_loop, daemon=True)
    t.start()
    t.join(timeout=2.0)
    assert not t.is_alive() and "context is invalid" in r._dead
    with pytest.raises(RigError):
        r._check_alive()


def _diverged_rig(speed_jump_m):
    """A rig whose EKF2 position jumped `speed_jump_m` within the last half second (sim, 2026-10-24: 45 m)."""
    r = _rig()
    r.rc["max_speed_mps"] = 4.0
    r.lock = threading.Lock()
    r.track = [(0.0, np.zeros(3)), (0.5, np.array([speed_jump_m, 0.0, 0.0]))]
    return r


def test_ekf_divergence_aborts_the_flight():
    with pytest.raises(RigError, match="not physical") as ei:
        _diverged_rig(45.0)._check_diverged()                    # 90 m/s
    assert ei.value.kind == "aborted"


def test_normal_flight_speed_passes_the_divergence_check():
    _diverged_rig(0.5)._check_diverged()                         # 1 m/s: cruise


def test_divergence_check_is_off_before_the_start():
    r = _diverged_rig(45.0)
    r.started = False
    r._check_diverged()
