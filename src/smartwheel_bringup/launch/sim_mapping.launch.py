import math
from datetime import datetime
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ("true", "1", "yes", "on")


def _rad(value) -> float:
    return math.radians(float(value))


def _setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    hardware_enabled = _as_bool(LaunchConfiguration("hardware_enabled").perform(context))
    backend = LaunchConfiguration("mapping_backend").perform(context)
    lidar_mode = LaunchConfiguration("lidar_mode").perform(context)
    state_mode = LaunchConfiguration("state_mode").perform(context)
    if mode != "mock":
        raise RuntimeError("sim_mapping.launch.py supports mode:=mock only in Stage A")
    if hardware_enabled:
        raise RuntimeError("sim_mapping.launch.py refuses hardware_enabled:=true")
    if backend not in ("rtabmap", "slam_toolbox"):
        raise RuntimeError("mapping_backend must be rtabmap or slam_toolbox")
    if lidar_mode not in ("single", "dual_map_only", "dual_lio"):
        raise RuntimeError("lidar_mode must be single, dual_map_only, or dual_lio")
    if state_mode not in ("lio_primary", "wheel_imu_fallback"):
        raise RuntimeError("state_mode must be lio_primary or wheel_imu_fallback")

    bringup_share = Path(get_package_share_directory("smartwheel_bringup"))
    profile_path = Path(LaunchConfiguration("hardware_profile").perform(context))
    if not profile_path.is_file():
        profile_path = bringup_share / "config" / "hardware_profile.mock.yaml"
    with profile_path.open(encoding="utf-8") as stream:
        profile = yaml.safe_load(stream)
    if profile["profile"]["mode"] != "mock" or profile["profile"].get("hardware_enabled", True):
        raise RuntimeError("sim launch requires a mock profile with hardware_enabled=false")

    output_root = Path(LaunchConfiguration("output_root").perform(context)).expanduser().resolve()
    map_name = LaunchConfiguration("map_name").perform(context)
    version = output_root / f"{map_name}_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"
    version.mkdir(parents=True, exist_ok=False)
    bag_path = version / "raw_bag"
    database_path = version / "rtabmap.db"

    left = profile["lidar_left"]
    right = profile["lidar_right"]
    wheel = profile["wheel"]
    imu = profile["imu"]
    cameras = profile["cameras"]
    dual = profile["dual_lidar"]
    left_xyz = [left[key] for key in ("x_m", "y_m", "z_m")]
    right_xyz = [right[key] for key in ("x_m", "y_m", "z_m")]
    left_rpy = [_rad(left[key]) for key in ("roll_deg", "pitch_deg", "yaw_deg")]
    right_rpy = [_rad(right[key]) for key in ("roll_deg", "pitch_deg", "yaw_deg")]

    description_share = Path(get_package_share_directory("smartwheel_description"))
    xacro_file = description_share / "urdf" / "smartwheel.urdf.xacro"
    xacro_arguments = {
        "wheel_track_m": wheel["track_width_m"],
        "wheel_radius_m": wheel["wheel_radius_m"],
        "imu_x": imu["x_m"], "imu_y": imu["y_m"], "imu_z": imu["z_m"],
        "imu_roll": _rad(imu["roll_deg"]), "imu_pitch": _rad(imu["pitch_deg"]), "imu_yaw": _rad(imu["yaw_deg"]),
        "left_lidar_x": left["x_m"], "left_lidar_y": left["y_m"], "left_lidar_z": left["z_m"],
        "left_lidar_roll": left_rpy[0], "left_lidar_pitch": left_rpy[1], "left_lidar_yaw": left_rpy[2],
        "right_lidar_x": right["x_m"], "right_lidar_y": right["y_m"], "right_lidar_z": right["z_m"],
        "right_lidar_roll": right_rpy[0], "right_lidar_pitch": right_rpy[1], "right_lidar_yaw": right_rpy[2],
    }
    for name in ("front", "left", "right", "rear"):
        camera = cameras[name]
        xacro_arguments.update(
            {
                f"{name}_camera_x": camera["x_m"],
                f"{name}_camera_y": camera["y_m"],
                f"{name}_camera_z": camera["z_m"],
                f"{name}_camera_roll": _rad(camera["roll_deg"]),
                f"{name}_camera_pitch": _rad(camera["pitch_deg"]),
                f"{name}_camera_yaw": _rad(camera["yaw_deg"]),
            }
        )
    xacro_command = [FindExecutable(name="xacro"), " ", str(xacro_file)]
    for key, value in xacro_arguments.items():
        xacro_command.extend([" ", f"{key}:={value}"])

    integration_mode = "dual_lio" if lidar_mode == "dual_lio" else "map_only"
    fallback_topic = "/odom/fallback" if state_mode == "wheel_imu_fallback" else "/wheel/odom"
    enable_cameras = _as_bool(LaunchConfiguration("enable_cameras").perform(context))
    colorization = _as_bool(LaunchConfiguration("enable_offline_colorization").perform(context))
    record_bag = _as_bool(LaunchConfiguration("record_bag").perform(context))
    enable_loop_closure = _as_bool(LaunchConfiguration("enable_loop_closure").perform(context))
    workbench_topic_aliases = _as_bool(LaunchConfiguration("workbench_topic_aliases").perform(context))
    backend_map_topic = "/mapping_backend/map" if backend == "slam_toolbox" and workbench_topic_aliases else (
        "/rtabmap/map" if backend == "rtabmap" else "/map"
    )
    camera_params = {}
    for name in ("front", "left", "right", "rear"):
        camera = cameras[name]
        camera_params[f"{name}_camera_xyz"] = [camera[key] for key in ("x_m", "y_m", "z_m")]
        camera_params[f"{name}_camera_rpy"] = [_rad(camera[key]) for key in ("roll_deg", "pitch_deg", "yaw_deg")]

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="mapping_v2_robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": Command(xacro_command)}],
        ),
        TimerAction(
            period=float(LaunchConfiguration("startup_delay_sec").perform(context)),
            actions=[
                Node(
                    package="smartwheel_sim",
                    executable="indoor_sim_node",
                    name="stage_a_indoor_sim",
                    output="screen",
                    parameters=[
                        {
                            "seed": 20260714,
                            "duration_sec": float(LaunchConfiguration("sim_duration_sec").perform(context)),
                            "playback_rate": float(LaunchConfiguration("playback_rate").perform(context)),
                            "publisher_discovery_warmup_sec": 2.0,
                            "lidar_rate_hz": float(left["scan_rate_hz"]),
                            "wheel_radius_m": float(wheel["wheel_radius_m"]),
                            "track_width_m": float(wheel["track_width_m"]),
                            "encoder_cpr": int(wheel["encoder_cpr"]),
                            "gear_ratio": float(wheel["gear_ratio"]),
                            "left_lidar_xyz": left_xyz,
                            "left_lidar_rpy": left_rpy,
                            "right_lidar_xyz": right_xyz,
                            "right_lidar_rpy": right_rpy,
                            "enable_cameras": enable_cameras,
                            "camera_width": int(cameras["front"]["width"]),
                            "camera_height": int(cameras["front"]["height"]),
                            "enable_external_cmd_vel": workbench_topic_aliases,
                        }
                    ],
                )
            ],
        ),
        Node(
            package="wheel_odom_driver",
            executable="wheel_odom_node",
            name="stage_a_wheel_odom",
            output="screen",
            parameters=[
                {
                    "wheel_radius_m": float(wheel["wheel_radius_m"]),
                    "track_width_m": float(wheel["track_width_m"]),
                    "encoder_cpr": int(wheel["encoder_cpr"]),
                    "encoder_bits": int(wheel["encoder_bits"]),
                    "gear_ratio": float(wheel["gear_ratio"]),
                    "left_sign": int(wheel["left_sign"]),
                    "right_sign": int(wheel["right_sign"]),
                }
            ],
        ),
        Node(
            package="smartwheel_global_mapping",
            executable="lio_input_adapter",
            name="stage_a_lio_input_adapter",
            output="screen",
            parameters=[{"input_topic": "/lidar/left/points_raw", "expected_frame_id": left["frame_id"]}],
        ),
        Node(
            package="smartwheel_state_estimation",
            executable="mock_lio_node",
            name="stage_a_mock_lio",
            output="screen",
        ),
        Node(
            package="smartwheel_state_estimation",
            executable="state_selector_node",
            name="stage_a_state_selector",
            output="screen",
            parameters=[{"state_mode": state_mode, "wheel_topic": fallback_topic}],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="stage_a_wheel_imu_fallback",
            output="screen",
            condition=IfCondition(PythonExpression(["'", state_mode, "' == 'wheel_imu_fallback'"])),
            parameters=[
                str(
                    Path(get_package_share_directory("smartwheel_global_mapping"))
                    / "config"
                    / "robot_localization_fallback.yaml"
                )
            ],
            remappings=[("odometry/filtered", "/odom/fallback")],
        ),
        Node(
            package="dual_lidar_fusion",
            executable="dual_lidar_fusion_node",
            name="stage_a_dual_lidar_fusion",
            output="screen",
            parameters=[
                {
                    "pair_tolerance_sec": float(dual["max_pair_time_difference_ms"]) / 1000.0,
                    "integration_mode": integration_mode,
                    "left_xyz": left_xyz,
                    "left_rpy": left_rpy,
                    "right_xyz": right_xyz,
                    "right_rpy": right_rpy,
                }
            ],
        ),
        Node(
            package="smartwheel_map_products",
            executable="map_products_node",
            name="stage_a_map_products",
            output="screen",
            parameters=[
                {
                    "map_name": map_name,
                    "output_root": str(output_root),
                    "version_directory": str(version),
                    "hardware_profile_path": str(profile_path),
                    "mapping_backend": backend,
                    "state_mode": state_mode,
                    "lidar_mode": lidar_mode,
                    "bag_path": str(bag_path) if record_bag else "",
                    "export_delay_sec": 3.0,
                    "enable_offline_colorization": colorization and enable_cameras,
                    "ground_truth_topic": "/sim/ground_truth/odom",
                    "backend_cloud_topic": "/rtabmap/optimized_cloud" if backend == "rtabmap" else "",
                    "require_backend_cloud": backend == "rtabmap",
                    **camera_params,
                }
            ],
        ),
        Node(
            package="smartwheel_global_mapping",
            executable="cloud_to_scan_node",
            name="stage_a_cloud_to_scan",
            output="screen",
            condition=IfCondition(PythonExpression(["'", backend, "' == 'slam_toolbox'"])),
        ),
        Node(
            package="smartwheel_mapping_manager",
            executable="mapping_manager_node",
            name="stage_a_mapping_manager",
            output="screen",
            parameters=[
                {
                    "map_name": map_name,
                    "record_bag": record_bag,
                    "bag_path": str(bag_path) if record_bag else "",
                    "checking_timeout_sec": float(LaunchConfiguration("startup_delay_sec").perform(context)) + 10.0,
                    "mapping_backend": backend,
                    "backend_map_topic": backend_map_topic,
                    "require_loop_closure": enable_loop_closure if backend == "rtabmap" else False,
                    "finalization_timeout_sec": float(
                        LaunchConfiguration("finalization_timeout_sec").perform(context)
                    ),
                }
            ],
        ),
        Node(
            package="smartwheel_global_mapping",
            executable="rtabmap_optimized_cloud_node",
            name="stage_a_rtabmap_optimized_cloud",
            output="screen",
            condition=IfCondition(PythonExpression(["'", backend, "' == 'rtabmap'"])),
        ),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            condition=IfCondition(PythonExpression(["'", backend, "' == 'rtabmap'"])),
            parameters=[
                str(Path(get_package_share_directory("smartwheel_global_mapping")) / "config" / "rtabmap_params.yaml"),
                {
                    "database_path": str(database_path),
                    "RGBD/ProximityBySpace": "true" if enable_loop_closure else "false",
                    # Time-neighbor constraints are not loop-closure evidence and can
                    # overconstrain this synthetic route. Space proximity is the tested loop mode.
                    "RGBD/ProximityByTime": "false",
                },
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
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            condition=IfCondition(PythonExpression(["'", backend, "' == 'slam_toolbox'"])),
            parameters=[str(Path(get_package_share_directory("smartwheel_global_mapping")) / "config" / "slam_toolbox_params.yaml")],
            remappings=[("map", backend_map_topic)],
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "record", "-o", str(bag_path),
                "/lidar/left/points_raw", "/lidar/right/points_raw", "/imu/data_raw",
                "/wheel/encoder_counts", "/wheel/odom", "/lio/odom", "/lio/path", "/odom/fused",
                "/camera/front/image_raw", "/camera/front/camera_info",
                "/camera/left/image_raw", "/camera/left/camera_info",
                "/camera/right/image_raw", "/camera/right/camera_info",
                "/camera/rear/image_raw", "/camera/rear/camera_info",
                "/lidar/merged/points", "/scan", "/sim/ground_truth/odom", "/sim/completed",
                "/map_products/occupancy", "/map_products/cloud", "/hardware/status",
                "/tf", "/tf_static", "/diagnostics", "/mapping/status",
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("record_bag")),
        ),
    ]
    return nodes


def generate_launch_description():
    default_profile = str(Path(get_package_share_directory("smartwheel_bringup")) / "config" / "hardware_profile.mock.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="mock", choices=["mock", "real"]),
            DeclareLaunchArgument("mapping_backend", default_value="rtabmap", choices=["rtabmap", "slam_toolbox"]),
            DeclareLaunchArgument("lidar_mode", default_value="dual_map_only", choices=["single", "dual_map_only", "dual_lio"]),
            DeclareLaunchArgument("state_mode", default_value="lio_primary", choices=["lio_primary", "wheel_imu_fallback"]),
            DeclareLaunchArgument("record_bag", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("enable_cameras", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("enable_online_visual_loop", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("enable_loop_closure", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("enable_offline_colorization", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("hardware_enabled", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("motion_mode", default_value="teleop", choices=["teleop", "push"]),
            DeclareLaunchArgument("hardware_profile", default_value=default_profile),
            DeclareLaunchArgument("map_name", default_value="stage_a_sim"),
            DeclareLaunchArgument("output_root", default_value="maps/versions"),
            DeclareLaunchArgument("sim_duration_sec", default_value="12.0"),
            DeclareLaunchArgument("playback_rate", default_value="2.0"),
            DeclareLaunchArgument("startup_delay_sec", default_value="6.0"),
            DeclareLaunchArgument("finalization_timeout_sec", default_value="15.0"),
            DeclareLaunchArgument("workbench_topic_aliases", default_value="false", choices=["true", "false"]),
            OpaqueFunction(function=_setup),
        ]
    )
