import os
import subprocess
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


def _filtered_rviz_config(source: Path, context) -> str:
    toggles = {
        "SmartWheel/Camera": _as_bool(LaunchConfiguration("enable_cameras").perform(context)),
        "SmartWheel/Teleop": _as_bool(LaunchConfiguration("enable_teleop").perform(context)),
        "SmartWheel/Mapping Control": _as_bool(LaunchConfiguration("enable_mapping_control").perform(context)),
        "SmartWheel/System Status": _as_bool(LaunchConfiguration("enable_status_panel").perform(context)),
        "SmartWheel/Map Products": _as_bool(LaunchConfiguration("enable_map_products").perform(context)),
    }
    if all(toggles.values()):
        return str(source)
    data = yaml.safe_load(source.read_text(encoding="utf-8"))
    data["Panels"] = [
        panel for panel in data.get("Panels", [])
        if toggles.get(panel.get("Class"), True)
    ]
    output = Path("/tmp") / f"smartwheel_operator_workbench_{os.getpid()}.rviz"
    output.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return str(output)


def _setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    hardware_enabled = _as_bool(LaunchConfiguration("hardware_enabled").perform(context))
    backend = LaunchConfiguration("mapping_backend").perform(context)
    if mode != "mock":
        raise RuntimeError("operator_workbench.launch.py currently permits mode:=mock only")
    if hardware_enabled:
        raise RuntimeError("operator_workbench.launch.py refuses hardware_enabled:=true")
    if backend not in ("rtabmap", "slam_toolbox"):
        raise RuntimeError("mapping_backend must be rtabmap or slam_toolbox")

    bringup_share = Path(get_package_share_directory("smartwheel_bringup"))
    rviz_argument = Path(LaunchConfiguration("rviz_config").perform(context)).expanduser()
    if not rviz_argument.is_file():
        rviz_argument = bringup_share / "rviz" / "smartwheel_operator_workbench.rviz"
    rviz_config = _filtered_rviz_config(rviz_argument.resolve(), context)
    map_root = Path(LaunchConfiguration("map_root").perform(context)).expanduser().resolve()
    experiment_root = Path(LaunchConfiguration("experiment_root").perform(context)).expanduser().resolve()
    profile = Path(LaunchConfiguration("hardware_profile").perform(context)).expanduser().resolve()
    git_commit = subprocess.run(
        ["git", "rev-parse", "--short=12", "HEAD"],
        cwd=str(Path.cwd()),
        check=False,
        capture_output=True,
        text=True,
    ).stdout.strip() or "UNKNOWN"
    map_source = "/rtabmap/map" if backend == "rtabmap" else "/mapping_backend/map"
    cloud_source = "/rtabmap/optimized_cloud" if backend == "rtabmap" else "/map_products/cloud"

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_share / "launch" / "sim_mapping.launch.py")),
        launch_arguments={
            "mode": "mock",
            "hardware_enabled": "false",
            "mapping_backend": backend,
            "lidar_mode": "dual_map_only",
            "state_mode": "lio_primary",
            "record_bag": "false",
            "enable_cameras": LaunchConfiguration("enable_cameras").perform(context),
            "enable_loop_closure": "true",
            "enable_offline_colorization": "true",
            "hardware_profile": str(profile),
            "map_name": LaunchConfiguration("map_name").perform(context),
            "output_root": str(map_root),
            "sim_duration_sec": LaunchConfiguration("sim_duration_sec").perform(context),
            "playback_rate": "1.0",
            "startup_delay_sec": "3.0",
            "finalization_timeout_sec": "60.0",
            "workbench_topic_aliases": "true",
        }.items(),
    )
    backend_node = Node(
        package="smartwheel_workbench",
        executable="workbench_node",
        name="smartwheel_operator_workbench_backend",
        output="screen",
        parameters=[{
            "mode": "mock",
            "hardware_enabled": False,
            "mapping_backend": backend,
            "map_root": str(map_root),
            "experiment_root": str(experiment_root),
            "map_cloud_source": cloud_source,
            "map_source": map_source,
            "path_source": "/lio/path",
            "hardware_profile": str(profile),
            "algorithm_profile": f"mapping-v2-workbench-{backend}",
            "git_commit": git_commit,
        }],
    )
    safety_supervisor = Node(
        package="smartwheel_rviz_plugins",
        executable="mock_safety_supervisor",
        name="mock_safety_supervisor",
        output="screen",
        parameters=[{
            "mode": "mock",
            "hardware_enabled": False,
            "command_timeout_sec": 0.35,
            "command_rate_hz": 20.0,
            "linear_speed_max_mps": 0.15,
            "angular_speed_max_radps": 0.25,
        }],
    )
    rviz = TimerAction(
        period=2.0,
        actions=[Node(
            package="rviz2",
            executable="rviz2",
            name="smartwheel_operator_rviz",
            output="screen",
            arguments=["-d", rviz_config],
        )],
    )
    return [simulation, backend_node, safety_supervisor, rviz]


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("smartwheel_bringup"))
    default_profile = str(bringup_share / "config" / "hardware_profile.mock.yaml")
    default_rviz = str(bringup_share / "rviz" / "smartwheel_operator_workbench.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="mock", choices=["mock", "real"]),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        DeclareLaunchArgument("mapping_backend", default_value="rtabmap", choices=["rtabmap", "slam_toolbox"]),
        DeclareLaunchArgument("hardware_enabled", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("enable_cameras", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("enable_teleop", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("enable_mapping_control", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("enable_status_panel", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("enable_map_products", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("hardware_profile", default_value=default_profile),
        DeclareLaunchArgument("map_name", default_value="workbench_mock"),
        DeclareLaunchArgument("map_root", default_value="maps/versions"),
        DeclareLaunchArgument("experiment_root", default_value="experiments"),
        DeclareLaunchArgument("sim_duration_sec", default_value="1800.0"),
        OpaqueFunction(function=_setup),
    ])
