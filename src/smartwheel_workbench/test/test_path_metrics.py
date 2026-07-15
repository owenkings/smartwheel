from geometry_msgs.msg import PoseStamped

import pytest

from smartwheel_workbench.node import (
    _directory_size_bytes,
    _finish_manager_action,
    _path_length,
)


def pose(x, y):
    message = PoseStamped()
    message.pose.position.x = float(x)
    message.pose.position.y = float(y)
    return message


def test_path_length_is_float_for_empty_and_single_pose_paths():
    assert _path_length([]) == 0.0
    assert isinstance(_path_length([pose(1, 2)]), float)


def test_path_length_sums_planar_segments():
    assert _path_length([pose(0, 0), pose(3, 4), pose(6, 8)]) == 10.0


def test_empty_bag_path_never_scans_current_directory(tmp_path, monkeypatch):
    (tmp_path / "large_unrelated_file").write_bytes(b"x" * 100)
    monkeypatch.chdir(tmp_path)
    assert _directory_size_bytes("") == 0


def test_bag_size_only_counts_selected_directory(tmp_path):
    bag = tmp_path / "rosbag"
    bag.mkdir()
    (bag / "metadata.yaml").write_bytes(b"1234")
    (bag / "data.db3.zstd").write_bytes(b"123456")
    assert _directory_size_bytes(str(bag)) == 10


def test_finish_accepts_manager_that_already_started_finalizing():
    assert _finish_manager_action("MAPPING") == "STOP"
    assert _finish_manager_action("LOOP_CLOSING") == "WAIT"
    assert _finish_manager_action("OPTIMIZING") == "WAIT"
    assert _finish_manager_action("READY") == "DONE"
    with pytest.raises(ValueError, match="cannot finish"):
        _finish_manager_action("FAILED")
