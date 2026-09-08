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
    config = Path(get_package_share_directory("smartwheel_global_mapping")) / "config"
    replay_rate = LaunchConfiguration("replay_rate").perform(context)
    replay_rate_value = float(replay_rate)
    if replay_rate_value <= 0.0:
        raise RuntimeError("replay_rate must be greater than zero")
    if backend == "rtabmap" and replay_rate_value > 1.0:
        raise RuntimeError(
            "RTAB-Map offline replay_rate must be <= 1.0 so synchronized inputs are not silently dropped"
        )
    post_replay_settle_sec = float(LaunchConfiguration("post_replay_settle_sec").perform(context))
    if post_replay_settle_sec < 2.0:
        raise RuntimeError("post_replay_settle_sec must be at least 2.0 seconds")
    maximum_accumulated_points = LaunchConfiguration(
        "maximum_accumulated_points"
    ).perform(context).strip()
    try:
        maximum_accumulated_points_value = int(maximum_accumulated_points)
    except ValueError as exc:
        raise RuntimeError("maximum_accumulated_points must be an integer") from exc
    if maximum_accumulated_points_value <= 0:
        raise RuntimeError("maximum_accumulated_points must be greater than zero")
    bag_cloud_topic = LaunchConfiguration("bag_cloud_topic").perform(context).strip()
    bag_odom_topic = LaunchConfiguration("bag_odom_topic").perform(context).strip()
    bag_tf_topic = LaunchConfiguration("bag_tf_topic").perform(context).strip()
    bag_storage = LaunchConfiguration("bag_storage").perform(context).strip()
    bag_start_offset_sec = LaunchConfiguration("bag_start_offset_sec").perform(context).strip()
    scan_frame = LaunchConfiguration("scan_frame").perform(context).strip()
    try:
        bag_start_offset_value = float(bag_start_offset_sec)
    except ValueError as exc:
        raise RuntimeError("bag_start_offset_sec must be numeric") from exc
    if bag_start_offset_value < 0.0:
        raise RuntimeError("bag_start_offset_sec must be non-negative")
    if backend == "slam_toolbox" and bag_start_offset_value > 0.0:
        raise RuntimeError(
            "slam_toolbox offline replay requires bag_start_offset_sec:=0.0 "
            "so recorded static TF is retained; the normalizer handles warm-up"
        )
    for name, value in (
        ("bag_cloud_topic", bag_cloud_topic),
        ("bag_odom_topic", bag_odom_topic),
        ("bag_tf_topic", bag_tf_topic),
    ):
        if not value.startswith("/"):
            raise RuntimeError(f"{name} must be an absolute ROS topic")
    if not bag_storage:
        raise RuntimeError("bag_storage must be non-empty")
    if not scan_frame or "/" in scan_frame:
        raise RuntimeError("scan_frame must be a non-empty TF frame without '/'")
    version.mkdir(parents=True, exist_ok=False)
    common = {"use_sim_time": True}
    state_output_topic = (
        "/offline/selected_odom" if backend == "slam_toolbox" else "/odom/fused"
    )
    playback = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", str(bag_path), "--storage", bag_storage,
            "--clock", "--rate", replay_rate, "--start-offset", bag_start_offset_sec,
            "--remap",
            f"{bag_tf_topic}:=/offline/recorded_tf",
            "/tf_static:=/offline/recorded_tf_static",
            f"{bag_odom_topic}:=/offline/recorded_odom",
            f"{bag_cloud_topic}:=/lidar/merged/points",
            "/map_products/occupancy:=/offline/recorded_map_products_occupancy",
            "/map_products/cloud:=/offline/recorded_map_products_cloud",
            "/mapping/status:=/offline/recorded_mapping_status",
            "/sim/completed:=/offline/recorded_sim_completed",
        ],
        output="screen",
    )
    stop_session = ExecuteProcess(
        cmd=["ros2", "service", "call", "/map_session/stop", "std_srvs/srv/Trigger", "{}"],
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
                    "state_mode": LaunchConfiguration("state_mode").perform(context),
                    "lio_topic": "/offline/recorded_odom",
                    "output_topic": state_output_topic,
                    "publish_tf": backend != "slam_toolbox",
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
                    "maximum_accumulated_points": maximum_accumulated_points_value,
                    "export_delay_sec": 3.0,
                    "enable_offline_colorization": _as_bool(
                        LaunchConfiguration("enable_offline_colorization").perform(context)
                    ),
                    "ground_truth_topic": "/sim/ground_truth/odom",
                    "backend_cloud_topic": "/rtabmap/optimized_cloud" if backend == "rtabmap" else "",
                    "require_backend_cloud": backend == "rtabmap",
                },
            ],
            remappings=[("/sim/completed", "/offline/recorded_sim_completed")],
        ),
        Node(
            package="smartwheel_global_mapping",
            executable="rtabmap_optimized_cloud_node",
            name="offline_rtabmap_optimized_cloud",
            output="screen",
            condition=IfCondition(PythonExpression(["'", backend, "' == 'rtabmap'"])),
            parameters=[common],
        ),
        Node(
            package="smartwheel_global_mapping",
            executable="offline_replay_normalizer_node",
            name="offline_replay_normalizer",
            output="screen",
            condition=IfCondition(PythonExpression(["'", backend, "' == 'slam_toolbox'"])),
            parameters=[
                common,
                {
                    "odom_input_topic": "/offline/selected_odom",
                    "odom_output_topic": "/odom/fused",
                    "scan_input_topic": "/offline/raw_scan",
                    "scan_output_topic": "/scan",
                    "odom_frame": "odom",
                    "child_frame": "body",
                    "max_interpolation_gap_sec": 0.25,
                    "tf_broadcast_delay_sec": 0.2,
                },
            ],
        ),
        Node(
            package="smartwheel_global_mapping",
            executable="offline_static_tf_relay_node",
            name="offline_static_tf_relay",
            output="screen",
            condition=IfCondition(PythonExpression(["'", backend, "' == 'slam_toolbox'"])),
            parameters=[
                common,
                {
                    "input_topic": "/offline/recorded_tf_static",
                    "output_topic": "/tf_static",
                    "republish_period_sec": 0.5,
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
                ("cloud_map", "/rtabmap/cloud_map"),
                ("info", "/rtabmap/info"),
            ],
        ),
        Node(
            package="smartwheel_global_mapping",
            executable="cloud_to_scan_node",
            name="offline_cloud_to_scan",
            output="screen",
            condition=IfCondition(PythonExpression(["'", backend, "' == 'slam_toolbox'"])),
            parameters=[
                common,
                {
                    "input_topic": "/lidar/merged/points",
                    "output_topic": "/offline/raw_scan",
                    "target_frame": scan_frame,
                },
            ],
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="offline_slam_toolbox",
            output="screen",
            condition=IfCondition(PythonExpression(["'", backend, "' == 'slam_toolbox'"])),
            parameters=[
                str(config / "slam_toolbox_params.yaml"),
                common,
                {
                    # The launch name is intentionally offline_slam_toolbox;
                    # inject the contract explicitly because the YAML key is
                    # retained as slam_toolbox for the standalone backend.
                    "odom_frame": "odom",
                    "map_frame": "map",
                    "base_frame": "base_link",
                    "scan_topic": "/scan",
                    "transform_timeout": 1.0,
                    "tf_buffer_duration": 30.0,
                    "scan_buffer_size": 50,
                    "scan_queue_size": 50,
                    "min_laser_range": 0.2,
                    "max_laser_range": 20.0,
                },
            ],
        ),
        TimerAction(
            period=3.0,
            actions=[playback],
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=playback,
                on_exit=[TimerAction(period=post_replay_settle_sec, actions=[stop_session])],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=stop_session,
                on_exit=[TimerAction(period=1.0, actions=[export])],
            )
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
            DeclareLaunchArgument(
                "bag_cloud_topic", default_value="/lidar/merged/points",
                description="Topic name as stored in the bag; remapped to /lidar/merged/points.",
            ),
            DeclareLaunchArgument(
                "bag_odom_topic", default_value="/odom/fused",
                description="Topic name as stored in the bag; remapped to /offline/recorded_odom.",
            ),
            DeclareLaunchArgument(
                "bag_tf_topic", default_value="/tf",
                description="Topic name as stored in the bag; remapped to /tf for offline consumers.",
            ),
            DeclareLaunchArgument(
                "bag_storage", default_value="sqlite3",
                description="rosbag2 storage plugin used for the bag (formal H1 uses sqlite3).",
            ),
            DeclareLaunchArgument(
                "bag_start_offset_sec", default_value="0.0",
                description="Seconds to skip at bag start; use a measured sensor warm-up offset only.",
            ),
            DeclareLaunchArgument(
                "scan_frame", default_value="xtm60_right_link",
                description="Frame of the recorded point cloud used to create offline scans.",
            ),
            DeclareLaunchArgument("map_name", default_value="offline_map"),
            DeclareLaunchArgument("output_root", default_value="maps/versions"),
            DeclareLaunchArgument("replay_rate", default_value="1.0"),
            DeclareLaunchArgument(
                "maximum_accumulated_points",
                default_value="500000",
                description=(
                    "Bounded voxel capacity for the offline map product; increase "
                    "only with an explicit memory budget."
                ),
            ),
            DeclareLaunchArgument("post_replay_settle_sec", default_value="5.0"),
            OpaqueFunction(function=_setup),
        ]
    )
