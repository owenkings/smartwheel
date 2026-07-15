from pathlib import Path


def test_workbench_has_no_physical_transport_or_motor_publisher():
    package = Path(__file__).parents[1] / "smartwheel_workbench"
    source = "\n".join(path.read_text(encoding="utf-8") for path in package.glob("*.py"))
    forbidden = ("/dev/tty", "SocketCAN", "python-can", "serial.Serial", "/motor/command")
    assert all(token not in source for token in forbidden)
    safety = (
        Path(__file__).parents[2] / "smartwheel_rviz_plugins" / "src" / "mock_safety_supervisor.cpp"
    ).read_text(encoding="utf-8")
    assert 'create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safe"' in safety
    assert 'create_subscription<geometry_msgs::msg::Twist>(' in safety
    assert '"/teleop/cmd_vel"' in safety
    assert 'mode != "mock"' in safety
    assert "hardware_enabled" in safety


def test_workbench_does_not_duplicate_high_bandwidth_sensor_subscriptions():
    source = (
        Path(__file__).parents[1] / "smartwheel_workbench" / "node.py"
    ).read_text(encoding="utf-8")
    subscription_section = source.split("self.create_timer(0.5", 1)[0]
    assert "Image," not in subscription_section
    assert '"/lidar/left/points_raw", PointCloud2' not in subscription_section
    assert '"/lidar/right/points_raw", PointCloud2' not in subscription_section
    assert '"/imu/data_raw", Imu' not in subscription_section
    assert '"/wheel/odom", Odometry' not in subscription_section
    assert 'DiagnosticArray,\n            "/diagnostics"' in subscription_section
