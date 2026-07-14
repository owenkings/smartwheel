from pathlib import Path


REQUIRED_PACKAGES = {
    "smartwheel_interfaces",
    "smartwheel_description",
    "smartwheel_sensor_api",
    "xtm60_ros2_driver",
    "h30_imu_driver",
    "wheel_odom_driver",
    "camera_array_driver",
    "dual_lidar_fusion",
    "smartwheel_state_estimation",
    "smartwheel_global_mapping",
    "smartwheel_map_products",
    "smartwheel_mapping_manager",
    "smartwheel_teleop",
    "smartwheel_bringup",
    "smartwheel_sim",
    "smartwheel_tests",
}


REQUIRED_LAUNCHES = {
    "sim_mapping.launch.py",
    "record_mapping.launch.py",
    "single_lidar_mapping.launch.py",
    "dual_lidar_mapping.launch.py",
    "offline_mapping.launch.py",
    "map_export.launch.py",
    "teleop_safe.launch.py",
}


def repository_root():
    return Path(__file__).resolve().parents[3]


def test_clean_package_and_launch_structure_exists():
    root = repository_root()
    packages = {path.parent.name for path in (root / "src").glob("smartwheel*/package.xml")}
    packages.update({path.parent.name for path in (root / "src").glob("*driver/package.xml")})
    packages.add("dual_lidar_fusion") if (root / "src/dual_lidar_fusion/package.xml").is_file() else None
    packages.add("camera_array_driver") if (root / "src/camera_array_driver/package.xml").is_file() else None
    assert REQUIRED_PACKAGES <= packages
    launches = {path.name for path in (root / "src/smartwheel_bringup/launch").glob("*.launch.py")}
    assert REQUIRED_LAUNCHES <= launches


def test_mapping_v2_sources_exclude_ultrasonic():
    root = repository_root()
    for package in REQUIRED_PACKAGES:
        if package == "smartwheel_tests":
            continue
        path = root / "src" / package
        if not path.exists():
            continue
        for source in path.rglob("*"):
            if source.is_file() and source.suffix in (".py", ".yaml", ".xml", ".xacro"):
                text = source.read_text(encoding="utf-8", errors="ignore").lower()
                assert "fd07" not in text
                assert "ultrasonic" not in text


def test_only_state_selector_owns_odom_to_base_tf():
    root = repository_root()
    publishers = []
    for source in (root / "src").glob("smartwheel*/**/*.py"):
        if "smartwheel_tests" in source.parts:
            continue
        text = source.read_text(encoding="utf-8", errors="ignore")
        if "TransformBroadcaster" in text and "base_link" in text and "odom" in text:
            publishers.append(source.name)
    assert publishers == ["state_selector_node.py"]
