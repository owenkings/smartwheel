"""Stage 1 (RViz-first MVP): manual-drive mapping, single LEFT XT-M60.

= manual_teleop (sensors + TF + scan + EKF + watchdog + safety + base + RViz
  TeleopPanel) PLUS dual_lidar fusion (-> /points_merged) PLUS RTAB-Map building
  the 3D cloud map and 2D projected grid while the user drives by hand.

Mapping data flow:
  /points_merged (left-only fallback) + /odometry/filtered (EKF)
     -> RTAB-Map -> /rtabmap/cloud_map (3D)  /rtabmap/grid_map (2D)

Control (safety never bypassed):
  RViz TeleopPanel -> /cmd_vel_nav -> safety_supervisor -> /cmd_vel_safe -> base

RTAB-Map runs in EXTERNAL odom mode using /odometry/filtered, so the EKF stays
the single odom->base_link owner (icp_odometry is NOT started, avoiding a TF
fight). NO Nav2, NO autonomous explorer.

SAFETY: motors move only with motion_control_enabled:=true (default false =
read-only). Off-ground/clear-area test first, keep the physical E-stop in reach.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("wheelchair_bringup")
    bringup = get_package_share_directory("wheelchair_bringup")
    mapping = get_package_share_directory("wheelchair_3d_mapping")

    motion_control_enabled = LaunchConfiguration("motion_control_enabled")
    use_rviz = LaunchConfiguration("rviz")
    delete_db = LaunchConfiguration("delete_db_on_start")
    database_path = LaunchConfiguration("database_path")

    manual_teleop_launch = os.path.join(bringup, "launch", "manual_teleop.launch.py")
    fusion_launch = os.path.join(mapping, "launch", "dual_lidar_fusion.launch.py")
    rtabmap_launch = os.path.join(mapping, "launch", "rtabmap_3d_mapping.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument(
            "motion_control_enabled", default_value="false",
            description="HIGH RISK. true lets /cmd_vel_safe write real motor speeds. "
                        "Default false = read-only (verify the chain without moving)."),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("delete_db_on_start", default_value="true",
                              description="Start a fresh map. Set false to append to an existing db."),
        DeclareLaunchArgument(
            "database_path", default_value=os.path.expanduser("~/.ros/rtabmap.db")),

        LogInfo(msg="[manual_mapping_left] single LEFT XT-M60 manual-drive mapping. "
                    "Drive from the RViz TeleopPanel; RTAB-Map builds the map as you go. "
                    "No Nav2 / no autonomous explorer."),

        # 1. Manual teleop base stack (sensors, TF, scan, EKF, watchdog, safety,
        #    base, RViz TeleopPanel). RViz is started here separately, so disable
        #    the one inside manual_teleop to avoid two RViz windows.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manual_teleop_launch),
            launch_arguments={
                "motion_control_enabled": motion_control_enabled,
                "rviz": "false",
            }.items(),
        ),

        # 2. Dual-lidar fusion (single-left fallback) -> /points_merged.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fusion_launch),
            launch_arguments={
                "allow_single_lidar_fallback": "true",
            }.items(),
        ),

        # 3. RTAB-Map in EXTERNAL odom mode (uses EKF /odometry/filtered, no icp
        #    so it does not fight the EKF for odom->base_link). Camera kept out of
        #    the geometry sync path (subscribe_rgb:=false).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch),
            launch_arguments={
                "bringup_sensors": "false",
                "points_topic": "/points_merged",
                "imu_topic": "/imu/data",
                "odom_mode": "external",
                "odom_topic": "/odometry/filtered",
                "subscribe_scan_cloud": "true",
                "subscribe_rgb": "false",
                "use_colorizer": "false",
                "localization": "false",
                "delete_db_on_start": delete_db,
                "database_path": database_path,
                "rviz": "false",
            }.items(),
        ),

        # 4. RViz: sensor view + embedded TeleopPanel + map displays.
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", PathJoinSubstitution(
                [bringup_share, "rviz", "manual_mapping_left.rviz"])],
            condition=IfCondition(use_rviz),
        ),
    ])
