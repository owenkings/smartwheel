"""Autonomous RViz exploration 3D mapping (HIGH RISK, explicit opt-in).

The chair drives itself at very low speed, choosing unexplored frontiers and
building a 3D map, with everything visualized in RViz. Composition:
  RTAB-Map  -> builds the map (/rtabmap/cloud_map 3D, /rtabmap/grid_map 2D)
  Nav2      -> plans + controls (low-speed autonomous params)
  safety_supervisor -> /cmd_vel_nav -> /cmd_vel_safe (never bypassed)
  frontier_explorer_node -> picks unknown-region goals via NavigateToPose
  RViz      -> visualization only

SAFETY: nothing moves by default. The wheelchair only moves when BOTH
enable_motion:=true AND autonomous_exploration:=true. Off-ground test first,
clear area only, physical E-stop required.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
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
    explore_active = enable_motion and autonomous

    actions = []
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

    # A. + C. Sensors (dual XT-M60 + IMU + ultrasonic) + fusion (/points_merged) + RTAB-Map 3D.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mapping, "launch", "rtabmap_3d_mapping.launch.py")),
        launch_arguments={
            "bringup_sensors": "true",
            "points_topic": "/points_merged",
            "imu_topic": "/imu/data",
            # Wheel odometry is the odom->base_link source (reliable on this wheeled
            # base). icp_odometry is NOT run: the 120-deg flash-LiDAR cloud is too
            # sparse/low-overlap for reliable ICP odometry (ratio < threshold).
            "odom_mode": "external",
            "odom_topic": "/wheel/odom",
            "subscribe_scan_cloud": "true",
            "subscribe_rgb": "false",          # camera not in geometry SLAM
            "use_colorizer": "true" if use_colorizer else "false",
            "enable_ultrasonic": "true",
            "localization": "false",
            "delete_db_on_start": "true",
            "rviz": "false",                   # this launch starts rviz2 separately
        }.items(),
    ))

    # B. /scan for Nav2 local costmap + safety + passability (from the two radars).
    for name, cfg in (
        ("pointcloud_to_laserscan_left_node", "pointcloud_to_scan_left.yaml"),
        ("pointcloud_to_laserscan_right_node", "pointcloud_to_scan_right.yaml"),
    ):
        actions.append(Node(
            package="wheelchair_perception", executable="pointcloud_to_laserscan_node",
            name=name, output="screen",
            parameters=[os.path.join(bringup, "config", cfg)],
        ))
    actions.append(Node(
        package="wheelchair_perception", executable="scan_merger_node",
        name="scan_merger_node", output="screen",
        parameters=[os.path.join(bringup, "config", "scan_merger.yaml")],
    ))

    # D. + E. Nav2 (low-speed mapping params) + safety + passability + goal mgmt.
    # require_localization_healthy:=false because map->odom is from RTAB-Map, not AMCL.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "navigation.launch.py")),
        launch_arguments={
            "params_file": os.path.join(bringup, "config", "nav2_autonomous_mapping_params.yaml"),
            "require_localization_healthy": "false",
            "enable_passability": "false",
            "safety_params_file": os.path.join(bringup, "config", "safety_params_mapping.yaml"),
        }.items(),
    ))

    # F. Base driver. Motion writes ONLY when enable_motion:=true. publish_tf=true:
    # the base owns odom->base_link from wheel odometry (icp_odometry is not run in
    # this mode, so there is no TF double-publish). RTAB-Map owns map->odom.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "base.launch.py")),
        launch_arguments={
            "mode": "real",
            "publish_tf": "true",
            "motion_control_enabled": "true" if enable_motion else "false",
            "hold_zero_before_motion_init": "true" if enable_motion else "false",
        }.items(),
    ))

    # G. Frontier explorer. auto_start only when motion + exploration are both on.
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
        }],
    ))

    return actions


def generate_launch_description():
    rviz_cfg = os.path.join(
        get_package_share_directory("wheelchair_3d_mapping"), "rviz", "autonomous_3d_mapping.rviz"
    )
    return LaunchDescription([
        DeclareLaunchArgument("enable_motion", default_value="false",
                              description="HIGH RISK. true allows the base to write motor speeds."),
        DeclareLaunchArgument("autonomous_exploration", default_value="false",
                              description="HIGH RISK. true lets frontier_explorer send goals (needs enable_motion)."),
        DeclareLaunchArgument("bringup_sensors", default_value="true"),
        DeclareLaunchArgument("enable_xtm60_radar", default_value="true"),
        DeclareLaunchArgument("use_colorizer", default_value="false",
                              description="Colorize the cloud with the cameras (visual only)."),
        # Enforced low-speed caps live in nav2_autonomous_mapping_params.yaml (Nav2)
        # and safety_params.yaml (safety layer). These are advisory/record.
        DeclareLaunchArgument("max_linear_speed", default_value="0.10"),
        DeclareLaunchArgument("max_angular_speed", default_value="0.25"),
        DeclareLaunchArgument("min_frontier_size", default_value="8"),
        DeclareLaunchArgument("goal_timeout_sec", default_value="45.0"),
        DeclareLaunchArgument("exploration_timeout_sec", default_value="600.0"),
        DeclareLaunchArgument("stop_on_safety_warning", default_value="false",
                              description="false: explorer keeps sending goals during WARNING/SLOWDOWN "
                                          "(safety still slows/zeros speed; explorer halts on STOP/EMERGENCY)."),
        DeclareLaunchArgument("stop_on_safety_emergency", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_setup),
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", rviz_cfg],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
