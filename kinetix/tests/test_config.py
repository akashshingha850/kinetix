import numpy as np
import pytest

from kinetix.config import load_config, load_scene

SCENE = """name: r
kind: {kind}
roi: {{center: [0, 0, 3], size: [10, 8, 6]}}
workspace: {{center: [0, 0, 2], size: [7, 5, 2]}}
altitude_m: auto
"""


def test_auto_altitude_follows_the_room_box_and_sets_the_workspace_height(tmp_path):
    (tmp_path / "s.yaml").write_text(SCENE.format(kind="room"))
    s = load_scene(tmp_path / "s.yaml")                                        # floor 0, ceiling 6
    assert np.allclose(s.altitude, (1.1, 5.0)) and np.allclose([s.workspace.lo[2], s.workspace.hi[2]], [1.1, 5.0])
    capped = load_scene(tmp_path / "s.yaml", load_config([], ["explore.fit.altitude_above_floor_m=[1.1,3.0]"], kind="room"))
    assert np.allclose(capped.altitude, (1.1, 3.0))


def test_auto_altitude_needs_a_room_scene(tmp_path):
    (tmp_path / "s.yaml").write_text(SCENE.format(kind="object"))
    with pytest.raises(ValueError):
        load_scene(tmp_path / "s.yaml")
