from pathlib import Path


ROOT = Path(__file__).parents[2]
MAPPING_LAUNCH = ROOT / "wheelchair_3d_mapping" / "launch"
BRINGUP_LAUNCH = ROOT / "wheelchair_bringup" / "launch"


def test_fast_lio_map_bridge_is_explicitly_switchable():
    source = (MAPPING_LAUNCH / "fast_lio_mapping.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'DeclareLaunchArgument(\n            "publish_map_tf"' in source
    assert 'condition=IfCondition(LaunchConfiguration("publish_map_tf"))' in source
    assert source.count('"map", "camera_init"') == 1


def test_fast_lio_rejects_invalid_and_blocked_right_radar_before_nodes():
    source = (MAPPING_LAUNCH / "fast_lio_mapping.launch.py").read_text(
        encoding="utf-8"
    )

    assert "INVALID_RADAR_SELECTION" in source
    assert 'if radar == "right":' in source
    assert "BLOCKED_CONFLICT: right_lidar_stage1_calibration" in source
    assert source.index('if radar == "right":') < source.index("spec = RADAR_TF[radar]")


def test_fast_lio_uses_full_inverse_imu_mount_and_bounded_lio_ranges():
    source = (MAPPING_LAUNCH / "fast_lio_mapping.launch.py").read_text(
        encoding="utf-8"
    )

    assert '"min_range": 0.3' in source
    assert '"max_range": 12.0' in source
    assert '"output_qos": "best_effort"' in source
    assert '"0.000495116765", "-0.004454081453", "-0.449977683911"' in source
    assert '"-0.004949042248", "-0.000550123085", "0.000002722616"' in source


def test_left_profile_gives_map_correction_to_loop_backend_only_when_enabled():
    source = (BRINGUP_LAUNCH / "manual_mapping_lio_left.launch.py").read_text(
        encoding="utf-8"
    )

    assert '"publish_map_tf": PythonExpression(' in source
    assert '["\'", enable_loop_backend, "\' != \'true\'"]' in source
    assert '"publish_radar_tf": "false"' in source
    assert "loop_backend_odom_to_camera_init" not in source


def test_rtabmap_explicitly_owns_the_global_map_correction():
    source = (MAPPING_LAUNCH / "rtabmap_3d_mapping.launch.py").read_text(
        encoding="utf-8"
    )

    assert '"map_frame_id": "map"' in source
    assert '"publish_tf": True' in source
