from pathlib import Path


SRC = Path(__file__).resolve().parents[2]
LAUNCH = SRC / "smartwheel_bringup" / "launch" / "manual_mapping_lio_left_session.launch.py"


def test_left_session_wrapper_starts_capture_before_manual_lio():
    source = LAUNCH.read_text(encoding="utf-8")
    assert '"map_export.launch.py"' in source
    assert '"manual_mapping_lio_left.launch.py"' in source
    assert 'get_package_share_directory("wheelchair_bringup")' in source
    assert 'os.path.join(wheelchair_bringup, "launch", "manual_mapping_lio_left.launch.py")' in source
    assert '"cloud_topic": "/cloud_registered"' in source
    assert '"odom_topic": "/Odometry"' in source
    assert '"cloud_frame_mode": "world_registered"' in source
    assert '"expected_cloud_frame": "camera_init"' in source
    assert '"require_session_stop": "true"' in source


def test_left_session_wrapper_keeps_motion_control_disabled_by_default():
    source = LAUNCH.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("motion_control_enabled", default_value="false")' in source
    assert "right-lidar entry remains blocked" in source
