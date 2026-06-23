"""Top-level: manual-drive FAST-LIO mapping, single RIGHT XT-M60.

RIGHT-radar deployment (the LEFT radar is currently boxed/occluded — see
.kiro/steering/orin-host-ops.md). Mirrors manual_mapping_lio_left.launch.py but
selects the right radar throughout:
  - manual_teleop radar:=right (right XT-M60 driver + right scan)
  - fast_lio_mapping radar:=right (xtm60_right_lio.yaml + right extrinsic TF)
  - cloud_to_occupancy_grid fed by /cloud_registered
  - RViz: manual_mapping_lio_right.rviz layout (radar-agnostic topics:
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

        # 4b. Loop-backend TF alignment (Task 6). RTAB-Map external-odom expects an
        #     `odom -> base_link` chain, but FAST-LIO publishes its pose on
        #     `camera_init -> body -> base_link` (camera_init is FAST-LIO's world
        #     origin; /Odometry is expressed in camera_init). Publish an identity
        #     `odom -> camera_init` so RTAB-Map's odom frame bridges onto FAST-LIO's
        #     world and the chain odom -> camera_init -> body -> base_link is whole.
        #     Only when the backend is on (default off -> not published, no clash).
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="loop_backend_odom_to_camera_init",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "camera_init"],
            output="screen",
            condition=IfCondition(enable_loop_backend),
        ),

        # 5. RViz (right-radar LIO layout).
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", PathJoinSubstitution(
                [bringup_share, "rviz", "manual_mapping_lio_right.rviz"])],
            condition=IfCondition(use_rviz),
        ),
    ])
