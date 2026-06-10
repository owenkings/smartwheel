"""Autonomous RViz exploration 3D mapping (HIGH RISK, explicit opt-in).

The chair drives itself at very low speed, choosing unexplored frontiers and
building a 3D map, with everything visualized in RViz. Composition:
  RTAB-Map  -> builds the map (/rtabmap/cloud_map 3D, /rtabmap/grid_map 2D)
  Nav2      -> available for frontier mode and route preview/control
  safety_supervisor -> /cmd_vel_nav -> /cmd_vel_safe (never bypassed)
  reactive_explorer_node -> vacuum-like first-pass motion from /scan
  frontier_explorer_node -> optional frontier goals via NavigateToPose
  RViz      -> visualization only

SAFETY: nothing moves by default. The wheelchair only moves when BOTH
enable_motion:=true AND autonomous_exploration:=true. Off-ground test first,
clear area only, physical E-stop required.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    mapping = get_package_share_directory("wheelchair_3d_mapping")
    bringup = get_package_share_directory("wheelchair_bringup")

    def s(name):
        return LaunchConfiguration(name).perform(context).strip()

    def flag(name):
        return s(name).lower() == "true"

    enable_motion = flag("enable_motion")
    autonomous = flag("autonomous_exploration")
    use_colorizer = flag("use_colorizer")
    hardware_profile = s("hardware_profile").lower()
    if hardware_profile not in ("dual_lidar", "left_lidar_lab", "right_lidar_lab"):
        raise RuntimeError(
            "hardware_profile must be dual_lidar, left_lidar_lab, or right_lidar_lab"
        )
    if hardware_profile == "right_lidar_lab":
        raise RuntimeError(
            "hardware_profile=right_lidar_lab is reserved but not implemented in this release"
        )

    left_lidar_lab = hardware_profile == "left_lidar_lab"
    enable_left_lidar = True
    enable_right_lidar = not left_lidar_lab
    allow_single_lidar_fallback = left_lidar_lab or not enable_motion

    exploration_mode = s("exploration_mode").lower()
    if exploration_mode == "auto":
        exploration_mode = "reactive" if left_lidar_lab else "frontier"
    if exploration_mode not in ("frontier", "reactive"):
        raise RuntimeError("exploration_mode must be auto, frontier, or reactive")

    linear_arg = s("max_linear_speed").lower()
    angular_arg = s("max_angular_speed").lower()
    max_linear_speed = (0.03 if left_lidar_lab else 0.05) if linear_arg == "auto" else float(linear_arg)
    max_angular_speed = (0.18 if left_lidar_lab else 0.22) if angular_arg == "auto" else float(angular_arg)
    if left_lidar_lab:
        max_linear_speed = min(max_linear_speed, 0.04)
        max_angular_speed = min(max_angular_speed, 0.18)

    diagnostics_config = (
        "diagnostics_left_lidar_lab.yaml" if left_lidar_lab else "diagnostics.yaml"
    )
    scan_merger_config = "scan_merger_left_only.yaml" if left_lidar_lab else "scan_merger.yaml"
    rviz_config = "left_lidar_lab_mapping.rviz" if left_lidar_lab else "autonomous_3d_mapping.rviz"
    explore_active = enable_motion and autonomous

    actions = []
    if left_lidar_lab:
        actions.append(LogInfo(
            msg="[left_lidar_lab] RIGHT XT-M60 is disabled / under repair. "
                "Single-left-lidar autonomous mapping demo mode."
        ))
    actions.append(LogInfo(
        msg=f"[autonomous_rviz_mapping] hardware_profile={hardware_profile} "
            f"exploration_mode={exploration_mode} max_linear_speed={max_linear_speed:.3f} "
            f"max_angular_speed={max_angular_speed:.3f}"
    ))
    if autonomous and not enable_motion:
        actions.append(LogInfo(msg="[autonomous_rviz_mapping] ERROR: autonomous_exploration:=true "
                                   "requires enable_motion:=true. Exploration MOTION DISABLED "
                                   "(mapping + visualization still run)."))
    if enable_motion:
        actions.append(LogInfo(msg="[autonomous_rviz_mapping] *** HIGH-RISK AUTONOMOUS MAPPING ENABLED: "
                                   "the wheelchair MAY MOVE. Off-ground/clear-area only, keep the physical "
                                   "E-stop in reach. ***"))
    else:
        actions.append(LogInfo(msg="[autonomous_rviz_mapping] enable_motion:=false -> base will not write "
                                   "motor speeds (safe). Pass enable_motion:=true to allow motion."))

    # A. + C. Profile-selected XT-M60 sensors + fusion + RTAB-Map 3D.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mapping, "launch", "rtabmap_3d_mapping.launch.py")),
        launch_arguments={
            "bringup_sensors": s("bringup_sensors"),
            "enable_xtm60_left": "true" if enable_left_lidar else "false",
            "enable_xtm60_right": "true" if enable_right_lidar else "false",
            "points_topic": "/points_merged",
            "imu_topic": "/imu/data",
            # Wheel+H30 EKF is the odom->base_link source. icp_odometry is not
            # run: the 120-degree flash-LiDAR cloud has insufficient overlap for
            # reliable standalone ICP odometry during pivots.
            "odom_mode": "external",
            "odom_topic": "/odometry/filtered",
            "subscribe_scan_cloud": "true",
            "subscribe_rgb": "false",          # camera not in geometry SLAM
            "use_colorizer": "true" if use_colorizer else "false",
            "enable_ultrasonic": "true",
            "allow_single_lidar_fallback": "true" if allow_single_lidar_fallback else "false",
            "localization": "false",
            "delete_db_on_start": s("delete_db_on_start"),
            "database_path": s("database_path"),
            "rviz": "false",                   # this launch starts rviz2 separately
        }.items(),
    ))

    # B. /scan for Nav2 and safety. The lab profile never starts the absent right chain.
    scan_converters = [("pointcloud_to_laserscan_left_node", "pointcloud_to_scan_left.yaml")]
    if enable_right_lidar:
        scan_converters.append(
            ("pointcloud_to_laserscan_right_node", "pointcloud_to_scan_right.yaml")
        )
    for name, cfg in scan_converters:
        actions.append(Node(
            package="wheelchair_perception", executable="pointcloud_to_laserscan_node",
            name=name, output="screen",
            parameters=[os.path.join(bringup, "config", cfg)],
        ))
    actions.append(Node(
        package="wheelchair_perception", executable="scan_merger_node",
        name="scan_merger_node", output="screen",
        parameters=[os.path.join(bringup, "config", scan_merger_config)],
    ))

    # D. + E. Nav2 (low-speed mapping params) + safety + passability + goal mgmt.
    # require_localization_healthy:=false because map->odom is from RTAB-Map, not AMCL.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "navigation.launch.py")),
        launch_arguments={
            "params_file": os.path.join(bringup, "config", "nav2_autonomous_mapping_params.yaml"),
            "require_localization_healthy": "false",
            "enable_passability": "false",
            "enable_semantic_keepout": "true",
            "keepout_map_topic": "/rtabmap/grid_map",
            "safety_params_file": os.path.join(bringup, "config", "safety_params_mapping.yaml"),
        }.items(),
    ))

    # Fuse wheel odometry with the H30 yaw/yaw-rate in planar mode. The EKF is
    # the sole odom->base_link TF owner; RTAB-Map and Nav2 consume its output.
    actions.append(Node(
        package="robot_localization", executable="ekf_node", name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(bringup, "config", "robot_localization_ekf.yaml")],
    ))

    # Fail closed before safety can release velocity. Sensor criticality comes
    # entirely from the selected profile config; do not hard-code a right lidar.
    actions.append(Node(
        package="wheelchair_diagnostics", executable="sensor_watchdog_node",
        name="sensor_watchdog_node", output="screen",
        parameters=[os.path.join(bringup, "config", diagnostics_config)],
    ))

    # F. Base driver. Motion writes only when enable_motion:=true. The EKF owns
    # odom->base_link, so the base publishes odometry data but no TF.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "base.launch.py")),
        launch_arguments={
            "mode": s("base_mode"),
            "publish_tf": "false",
            "motion_control_enabled": "true" if enable_motion else "false",
            "hold_zero_before_motion_init": "false",
        }.items(),
    ))

    # G. Explorer. The single-left-lidar lab profile defaults to reactive mode;
    # the dual-lidar profile keeps the existing frontier default.
    if exploration_mode == "frontier":
        actions.append(Node(
            package="wheelchair_navigation", executable="frontier_explorer_node",
            name="frontier_explorer_node", output="screen",
            parameters=[{
                "auto_start": explore_active,
                "map_topic": "/rtabmap/grid_map",
                "min_frontier_size": int(s("min_frontier_size")),
                "goal_timeout_sec": float(s("goal_timeout_sec")),
                "exploration_timeout_sec": float(s("exploration_timeout_sec")),
                "stop_on_safety_warning": flag("stop_on_safety_warning"),
                "stop_on_safety_emergency": flag("stop_on_safety_emergency"),
                "require_enable_signal": flag("require_enable_signal"),
            }],
        ))
    else:
        actions.append(Node(
            package="wheelchair_navigation", executable="reactive_explorer_node",
            name="reactive_explorer_node", output="screen",
            parameters=[{
                "auto_start": explore_active,
                "forward_speed": max_linear_speed,
                "turn_speed": max_angular_speed,
                "hard_stop_distance": 0.30,
                "turn_trigger_distance": float(s("turn_trigger_distance")),
                "side_trigger_distance": 0.30,
                "ultrasonic_stale_timeout_sec": 1.0,
                "ultrasonic_min_valid_m": 0.03,
                "corridor_half_width_m": 0.45,
                "corridor_lookahead_m": 1.20,
                "require_enable_signal": flag("require_enable_signal"),
            }],
        ))

    if flag("rviz"):
        actions.append(Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", os.path.join(mapping, "rviz", rviz_config)],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "hardware_profile", default_value="dual_lidar",
            description="dual_lidar | left_lidar_lab | right_lidar_lab (reserved)",
        ),
        DeclareLaunchArgument("enable_motion", default_value="false",
                              description="HIGH RISK. true allows the base to write motor speeds."),
        DeclareLaunchArgument("autonomous_exploration", default_value="false",
                              description="HIGH RISK. true lets frontier_explorer send goals (needs enable_motion)."),
        DeclareLaunchArgument("bringup_sensors", default_value="true"),
        DeclareLaunchArgument(
            "base_mode", default_value="real",
            description="real for hardware; mock for no-hardware integration testing.",
        ),
        DeclareLaunchArgument("enable_xtm60_radar", default_value="true"),
        DeclareLaunchArgument("use_colorizer", default_value="false",
                              description="Colorize the cloud with the cameras (visual only)."),
        DeclareLaunchArgument(
            "max_linear_speed", default_value="auto",
            description="auto: 0.05 dual-lidar, 0.03 left-lidar lab (lab capped at 0.04).",
        ),
        DeclareLaunchArgument(
            "max_angular_speed", default_value="auto",
            description="auto: 0.22 dual-lidar, 0.18 left-lidar lab (lab capped at 0.18).",
        ),
        DeclareLaunchArgument(
            "exploration_mode", default_value="auto",
            description="auto | frontier | reactive. auto selects frontier for dual and reactive for left-lidar lab.",
        ),
        DeclareLaunchArgument("require_enable_signal", default_value="true",
                              description="Require an explicit true message on /autonomy/enable before exploration starts."),
        DeclareLaunchArgument("delete_db_on_start", default_value="true",
                              description="Start a fresh RTAB-Map database. Use a temporary database for preflight."),
        DeclareLaunchArgument("database_path", default_value=os.path.expanduser("~/.ros/rtabmap.db"),
                              description="RTAB-Map database path."),
        DeclareLaunchArgument("turn_trigger_distance", default_value="0.50",
                              description="Reactive explorer starts turning when front scan/ultrasonic is closer than this."),
        DeclareLaunchArgument("min_frontier_size", default_value="8"),
        DeclareLaunchArgument("goal_timeout_sec", default_value="45.0"),
        DeclareLaunchArgument("exploration_timeout_sec", default_value="600.0"),
        DeclareLaunchArgument("stop_on_safety_warning", default_value="false",
                              description="false: explorer keeps sending goals during WARNING/SLOWDOWN "
                                          "(safety still slows/zeros speed; explorer halts on STOP/EMERGENCY)."),
        DeclareLaunchArgument("stop_on_safety_emergency", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
