import re
from pathlib import Path

import yaml


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


def test_ground_truth_is_not_an_odometry_or_mapping_input():
    root = repository_root()
    forbidden = [
        root / "src/smartwheel_state_estimation/smartwheel_state_estimation/mock_lio_node.py",
        root / "src/smartwheel_global_mapping",
    ]
    for path in forbidden:
        sources = [path] if path.is_file() else path.rglob("*.py")
        for source in sources:
            assert "/sim/ground_truth/odom" not in source.read_text(encoding="utf-8")


def test_custom_map_products_do_not_compete_for_backend_map_topic():
    root = repository_root()
    source = (root / "src/smartwheel_map_products/smartwheel_map_products/node.py").read_text(
        encoding="utf-8"
    )
    assert 'create_publisher(OccupancyGrid, "/map",' not in source
    assert '"/map_products/occupancy"' in source


def test_offline_replay_cannot_export_on_recorded_completion_event():
    root = repository_root()
    source = (root / "src/smartwheel_bringup/launch/offline_mapping.launch.py").read_text(
        encoding="utf-8"
    )
    assert '"/sim/completed:=/offline/recorded_sim_completed"' in source
    assert "post_replay_settle_sec" in source


def test_rtabmap_offline_replay_rejects_accelerated_playback():
    root = repository_root()
    source = (root / "src/smartwheel_bringup/launch/offline_mapping.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'backend == "rtabmap" and replay_rate_value > 1.0' in source
    assert "silently dropped" in source


def test_third_party_build_dependencies_are_exactly_pinned_and_nested():
    root = repository_root()
    manifest = yaml.safe_load((root / "third_party/dependencies.repos").read_text(encoding="utf-8"))
    repositories = manifest["repositories"]
    assert set(repositories) == {
        "fast_lio_ros2",
        "fast_lio_ros2/include/ikd-Tree",
        "livox_ros_driver2",
    }
    for repository in repositories.values():
        assert repository["type"] == "git"
        assert re.fullmatch(r"[0-9a-f]{40}", repository["version"])

    references = yaml.safe_load(
        (root / "third_party/references.repos").read_text(encoding="utf-8")
    )["repositories"]
    assert set(references).isdisjoint(repositories)
    assert re.fullmatch(
        r"[0-9a-f]{40}", references["fast_lio_upstream_reference"]["version"]
    )
