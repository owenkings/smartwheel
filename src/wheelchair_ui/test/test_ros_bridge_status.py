import time

from wheelchair_ui.ros_bridge import (
    WheelchairUiRosNode,
    is_map_point_navigable,
    map_point_to_cell,
    normalize_angle,
    parse_key_value_status,
    pose_delta,
)


def test_parse_key_value_status_converts_common_values():
    parsed = parse_key_value_status(
        "mode=real; real_motion_enabled=false; left_rpm=-12.50; "
        "right_rpm=0.00; cmd_age=1.25; command=configured"
    )

    assert parsed == {
        "mode": "real",
        "real_motion_enabled": False,
        "left_rpm": -12.5,
        "right_rpm": 0,
        "cmd_age": 1.25,
        "command": "configured",
    }


def test_map_point_to_cell_uses_origin_and_resolution():
    info = {
        "width": 10,
        "height": 10,
        "resolution": 0.5,
        "origin": {"x": -1.0, "y": -2.0},
        "data": [0] * 100,
    }

    assert map_point_to_cell(info, -1.0, -2.0) == (0, 0)
    assert map_point_to_cell(info, 0.1, -0.9) == (2, 2)


def test_pose_delta_handles_distance_and_wrapped_yaw():
    pose = {"x": 1.0, "y": 2.0, "yaw": -3.10}

    distance, yaw_error = pose_delta(pose, 1.3, 1.6, 3.10)

    assert round(distance, 3) == 0.5
    assert yaw_error is not None
    assert yaw_error < 0.1
    assert -3.142 <= normalize_angle(4.0) <= 3.142


def test_is_map_point_navigable_rejects_unknown_and_near_obstacle():
    data = [0] * 100
    data[5 * 10 + 5] = 100
    data[2 * 10 + 2] = -1
    info = {
        "width": 10,
        "height": 10,
        "resolution": 0.1,
        "origin": {"x": 0.0, "y": 0.0},
        "data": data,
    }

    ok, _ = is_map_point_navigable(info, 0.1, 0.1, clearance_m=0.05)
    assert ok is True
    ok, reason = is_map_point_navigable(info, 0.5, 0.5, clearance_m=0.15)
    assert ok is False
    assert "障碍物" in reason
    ok, reason = is_map_point_navigable(info, 0.2, 0.2, clearance_m=0.05)
    assert ok is False
    assert "未知区域" in reason


def test_map_snapshot_prefers_fresh_rtabmap_then_falls_back_to_map():
    node = WheelchairUiRosNode.__new__(WheelchairUiRosNode)
    now = time.monotonic()
    node.primary_map_timeout_sec = 5.0
    node._map_candidates = {
        "/rtabmap/grid_map": {
            "ok": True,
            "source_topic": "/rtabmap/grid_map",
            "_received_at": now,
            "frame_id": "map",
            "width": 2,
            "height": 2,
            "resolution": 0.1,
            "origin": {"x": 0.0, "y": 0.0},
            "data": [0, 0, 0, 0],
        },
        "/map": {
            "ok": True,
            "source_topic": "/map",
            "_received_at": now,
            "frame_id": "map",
            "width": 1,
            "height": 1,
            "resolution": 1.0,
            "origin": {"x": 0.0, "y": 0.0},
            "data": [0],
        },
    }

    assert node.map_snapshot()["source_topic"] == "/rtabmap/grid_map"

    node._map_candidates["/rtabmap/grid_map"]["_received_at"] = now - 10.0
    assert node.map_snapshot()["source_topic"] == "/map"
