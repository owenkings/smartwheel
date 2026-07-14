from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ("true", "1", "yes", "on")


def _setup(context):
    bag_path = Path(LaunchConfiguration("bag_path").perform(context)).expanduser().resolve()
    if not bag_path.exists():
        raise RuntimeError(f"bag_path does not exist: {bag_path}")
    if _as_bool(LaunchConfiguration("hardware_enabled").perform(context)):
        raise RuntimeError("offline replay never enables physical hardware")
    backend = LaunchConfiguration("mapping_backend").perform(context)
    output_root = Path(LaunchConfiguration("output_root").perform(context)).expanduser().resolve()
    map_name = LaunchConfiguration("map_name").perform(context)
    version = output_root / f"{map_name}_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"
    version.mkdir(parents=True, exist_ok=False)
    config = Path(get_package_share_directory("smartwheel_global_mapping")) / "config"
    replay_rate = LaunchConfiguration("replay_rate").perform(context)
    common = {"use_sim_time": True}
    playback = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", str(bag_path), "--clock", "--rate", replay_rate,
            "--remap",
            "/tf:=/offline/recorded_tf",
            "/odom/fused:=/offline/recorded_odom",
            "/map:=/offline/recorded_map",
            "/map_cloud:=/offline/recorded_map_cloud",
            "/mapping/status:=/offline/recorded_mapping_status",
        ],
        output="screen",
    )
    export = ExecuteProcess(
        cmd=["ros2", "service", "call", "/map_export/export", "std_srvs/srv/Trigger", "{}"],
        output="screen",
    )
    actions = [
        Node(
            package="smartwheel_state_estimation",
            executable="state_selector_node",
            name="offline_state_selector",
            output="screen",
            parameters=[
                common,
                {
                    "state_mode": "lio_primary",
                    "lio_topic": "/offline/recorded_odom",
                    "output_topic": "/odom/fused",
                },
            ],
        ),
        Node(
            package="smartwheel_map_products",
            executable="map_products_node",
            name="offline_map_products",
            output="screen",
            parameters=[
                common,
                {
                    "map_name": map_name,
                    "output_root": str(output_root),
                    "version_directory": str(version),
                    "hardware_profile_path": LaunchConfiguration("hardware_profile").perform(context),
                    "mapping_backend": backend,
                    "state_mode": LaunchConfiguration("state_mode").perform(context),
                    "lidar_mode": LaunchConfiguration("lidar_mode").perform(context),
                    "bag_path": str(bag_path),
                    "export_delay_sec": 3.0,
                    "enable_offline_colorization": _as_bool(
                        LaunchConfiguration("enable_offline_colorization").perform(context)
                    ),
                },
            ],
        ),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            condition=IfCondition(PythonExpression(["'", backend, "' == 'rtabmap'"])),
            parameters=[
                str(config / "rtabmap_params.yaml"),
                common,
                {"database_path": str(version / "rtabmap.db")},
            ],
            remappings=[
                ("odom", "/odom/fused"),
                ("scan_cloud", "/lidar/merged/points"),
                ("map", "/rtabmap/map"),
                ("grid_map", "/rtabmap/grid_map"),
            ],
        ),
        Node(
            package="smartwheel_global_mapping",
            executable="cloud_to_scan_node",
            name="offline_cloud_to_scan",
            output="screen",
            condition=IfCondition(PythonExpression(["'", backend, "' == 'slam_toolbox'"])),
            parameters=[common],
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="offline_slam_toolbox",
            output="screen",
            condition=IfCondition(PythonExpression(["'", backend, "' == 'slam_toolbox'"])),
            parameters=[str(config / "slam_toolbox_params.yaml"), common],
        ),
        TimerAction(
            period=3.0,
            actions=[playback],
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=playback, on_exit=[TimerAction(period=1.0, actions=[export])])
        ),
    ]
    return actions


def generate_launch_description():
    default_profile = str(
        Path(get_package_share_directory("smartwheel_bringup")) / "config" / "hardware_profile.mock.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="mock", choices=["mock", "real"]),
            DeclareLaunchArgument("mapping_backend", default_value="rtabmap", choices=["rtabmap", "slam_toolbox"]),
            DeclareLaunchArgument("lidar_mode", default_value="dual_map_only", choices=["single", "dual_map_only", "dual_lio"]),
            DeclareLaunchArgument("state_mode", default_value="lio_primary", choices=["lio_primary", "wheel_imu_fallback"]),
            DeclareLaunchArgument("enable_cameras", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("enable_online_visual_loop", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("enable_offline_colorization", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("hardware_enabled", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("bag_path"),
            DeclareLaunchArgument("hardware_profile", default_value=default_profile),
            DeclareLaunchArgument("map_name", default_value="offline_map"),
            DeclareLaunchArgument("output_root", default_value="maps/versions"),
            DeclareLaunchArgument("replay_rate", default_value="1.0"),
            OpaqueFunction(function=_setup),
        ]
    )
