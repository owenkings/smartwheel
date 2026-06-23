"""Top-level: manual-drive FAST-LIO mapping, single RIGHT XT-M60.

RIGHT-radar deployment (the LEFT radar is currently boxed/occluded — see
.kiro/steering/orin-host-ops.md). Mirrors manual_mapping_lio_left.launch.py but
selects the right radar throughout:
  - manual_teleop radar:=right (right XT-M60 driver + right scan)
  - fast_lio_mapping radar:=right (xtm60_right_lio.yaml + right extrinsic TF)
  - cloud_to_occupancy_grid fed by /cloud_registered
  - RViz: manual_mapping_lio_left.rviz layout (radar-agnostic topics:
    /cloud_registered, /Odometry, /map_2d_from_3d, /scan, cameras)

TF ownership: EKF OFF (enable_ekf:=false), base publish_tf:=false -> FAST-LIO is
the sole base_link pose owner via camera_init->body + body->base_link bridges.

SAFETY: motors move only with motion_control_enabled:=true (default false).
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

    motion = LaunchConfiguration("motion_control_enabled")
    use_rviz = LaunchConfiguration("rviz")
    enable_loop_backend = LaunchConfiguration("enable_loop_backend")

    manual_teleop_launch = os.path.join(bringup, "launch", "manual_teleop.launch.py")
    fast_lio_launch = os.path.join(mapping, "launch", "fast_lio_mapping.launch.py")
    rtabmap_launch = os.path.join(mapping, "launch", "rtabmap_3d_mapping.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument(
            "motion_control_enabled", default_value="false",
            description="HIGH RISK. true lets /cmd_vel_safe write real motor speeds."),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "enable_loop_backend", default_value="false",
            description="Optional RTAB-Map loop/export backend consuming FAST-LIO pose."),

        LogInfo(msg="[manual_mapping_lio_right] RIGHT XT-M60 FAST-LIO (LiDAR-inertial) "
                    "manual-drive mapping. EKF OFF -> FAST-LIO owns the pose."),

        # 1. Base/sensor/safety stack, RIGHT radar, EKF OFF.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manual_teleop_launch),
            launch_arguments={
                "motion_control_enabled": motion,
                "rviz": "false",
                "enable_ekf": "false",
                "radar": "right",
            }.items(),
        ),

        # 2. FAST-LIO chain for the RIGHT radar.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch),
            launch_arguments={"radar": "right", "rviz": "false"}.items(),
        ),

        # 3. 3D -> 2D occupancy projection for Nav2.
        Node(
            package="wheelchair_3d_mapping", executable="cloud_to_occupancy_grid_node",
            name="cloud_to_occupancy_grid_node", output="screen",
            parameters=[os.path.join(mapping, "config", "cloud_to_occupancy_grid.yaml"),
                        {"input_cloud_topic": "/cloud_registered",
                         "map_frame": "camera_init"}],
        ),

        # 4. Optional RTAB-Map loop/export backend (default off).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch),
            launch_arguments={
                "bringup_sensors": "false",
                "points_topic": "/cloud_registered",
                "odom_mode": "external",
                "odom_topic": "/Odometry",
                "subscribe_scan_cloud": "true",
                "rviz": "false",
            }.items(),
            condition=IfCondition(enable_loop_backend),
        ),

        # 5. RViz (radar-agnostic LIO layout).
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", PathJoinSubstitution(
                [bringup_share, "rviz", "manual_mapping_lio_left.rviz"])],
            condition=IfCondition(use_rviz),
        ),
    ])
