from wheelchair_ui.ros_bridge import parse_key_value_status


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
