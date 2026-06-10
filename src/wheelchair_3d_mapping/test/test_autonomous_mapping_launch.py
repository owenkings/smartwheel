import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def load_launch_module():
    path = Path(__file__).parents[1] / "launch" / "autonomous_rviz_mapping.launch.py"
    spec = importlib.util.spec_from_file_location("autonomous_rviz_mapping_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_values(profile, **overrides):
    values = {
        "hardware_profile": profile,
        "enable_motion": "true",
        "autonomous_exploration": "true",
        "bringup_sensors": "true",
        "base_mode": "real",
        "enable_xtm60_radar": "true",
        "use_colorizer": "false",
        "max_linear_speed": "auto",
        "max_angular_speed": "auto",
        "exploration_mode": "auto",
        "require_enable_signal": "true",
        "delete_db_on_start": "true",
        "database_path": "/tmp/autonomous-mapping-launch-test.db",
        "turn_trigger_distance": "0.60",
        "min_frontier_size": "8",
        "goal_timeout_sec": "45.0",
        "exploration_timeout_sec": "600.0",
        "stop_on_safety_warning": "false",
        "stop_on_safety_emergency": "true",
        "rviz": "true",
    }
    values.update(overrides)
    return values


def expand(profile, **overrides):
    context = LaunchContext()
    context.launch_configurations.update(launch_values(profile, **overrides))
    actions = load_launch_module()._setup(context)
    includes = [
        dict(action.launch_arguments)
        for action in actions
        if isinstance(action, IncludeLaunchDescription)
    ]
    nodes = [action for action in actions if isinstance(action, Node)]
    names = [node._Node__node_name for node in nodes]
    parameter_files = []
    for node in nodes:
        for parameter in node._Node__parameters:
            if hasattr(parameter, "evaluate"):
                parameter_files.append(str(parameter.evaluate(context)))
    return includes, names, parameter_files, nodes


def node_parameters(nodes, name):
    node = next(node for node in nodes if node._Node__node_name == name)
    values = {}
    for parameter in node._Node__parameters:
        if isinstance(parameter, dict):
            for key, value in parameter.items():
                if isinstance(key, tuple):
                    key = "".join(
                        item.perform(None) if hasattr(item, "perform") else str(item)
                        for item in key
                    )
                values[str(key)] = value
    return values


def test_left_lidar_lab_profile_disables_right_chain():
    includes, names, parameter_files, nodes = expand("left_lidar_lab")
    rtabmap_args = includes[0]
    reactive = node_parameters(nodes, "reactive_explorer_node")
    rviz = next(node for node in nodes if node._Node__node_name == "rviz2")

    assert rtabmap_args["enable_xtm60_left"] == "true"
    assert rtabmap_args["enable_xtm60_right"] == "false"
    assert rtabmap_args["allow_single_lidar_fallback"] == "true"
    assert "pointcloud_to_laserscan_right_node" not in names
    assert "reactive_explorer_node" in names
    assert any(path.endswith("scan_merger_left_only.yaml") for path in parameter_files)
    assert any(path.endswith("diagnostics_left_lidar_lab.yaml") for path in parameter_files)
    assert reactive["forward_speed"] == pytest.approx(0.03)
    assert reactive["turn_speed"] == pytest.approx(0.18)
    assert str(rviz._Node__arguments[-1]).endswith("left_lidar_lab_mapping.rviz")


def test_left_lidar_lab_profile_enforces_speed_caps():
    _, _, _, nodes = expand(
        "left_lidar_lab",
        max_linear_speed="0.8",
        max_angular_speed="1.2",
    )
    reactive = node_parameters(nodes, "reactive_explorer_node")

    assert reactive["forward_speed"] == pytest.approx(0.04)
    assert reactive["turn_speed"] == pytest.approx(0.18)


def test_dual_lidar_profile_keeps_both_lidars_required_for_motion():
    includes, names, parameter_files, nodes = expand("dual_lidar")
    rtabmap_args = includes[0]
    rviz = next(node for node in nodes if node._Node__node_name == "rviz2")

    assert rtabmap_args["enable_xtm60_left"] == "true"
    assert rtabmap_args["enable_xtm60_right"] == "true"
    assert rtabmap_args["allow_single_lidar_fallback"] == "false"
    assert "pointcloud_to_laserscan_right_node" in names
    assert "frontier_explorer_node" in names
    assert any(path.endswith("scan_merger.yaml") for path in parameter_files)
    assert any(path.endswith("diagnostics.yaml") for path in parameter_files)
    assert str(rviz._Node__arguments[-1]).endswith("autonomous_3d_mapping.rviz")


def test_dual_lidar_mapping_without_motion_keeps_stationary_fallback():
    includes, _, _, _ = expand("dual_lidar", enable_motion="false")

    assert includes[0]["allow_single_lidar_fallback"] == "true"


def test_left_lidar_diagnostics_only_requires_left_points():
    path = (
        Path(__file__).parents[2]
        / "wheelchair_bringup"
        / "config"
        / "diagnostics_left_lidar_lab.yaml"
    )
    parameters = yaml.safe_load(path.read_text())["sensor_watchdog_node"]["ros__parameters"]

    assert parameters["points_topics"] == ["/xtm60/left/points"]
    assert parameters["points_0_critical"] is True
    assert "points_1_critical" not in parameters
    assert parameters["startup_grace_sec"] == pytest.approx(10.0)
    assert all(parameters[f"ultrasonic_{index}_critical"] for index in range(4))


def test_right_lidar_lab_profile_is_explicitly_reserved():
    context = LaunchContext()
    context.launch_configurations.update(launch_values("right_lidar_lab"))
    with pytest.raises(RuntimeError, match="reserved but not implemented"):
        load_launch_module()._setup(context)
