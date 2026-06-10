"""Manual teleop + sensor view (single LEFT XT-M60).

Brings up exactly what is needed to drive the wheelchair by hand from the RViz
TeleopPanel while watching live sensor data. NO Nav2, NO RTAB-Map, NO autonomous
explorer.

Chain (safety never bypassed):
  RViz TeleopPanel -> /cmd_vel_nav -> safety_supervisor -> /cmd_vel_safe -> base

Nodes started:
  - robot_state_publisher (TF)
  - left XT-M60, IMU, ultrasonics, left camera (real sensors)
  - pointcloud_to_laserscan (left) + scan_merger -> /scan
  - EKF (wheel + IMU) -> odom->base_link TF + /odometry/filtered
  - sensor_watchdog with the LEFT-ONLY diagnostics profile (the broken right
    radar is NOT critical, so it cannot force a fail-closed stop)
  - safety_supervisor (manned safety profile)
  - zlac8030 base driver

SAFETY: motors move only with motion_control_enabled:=true. Default false is
read-only. Off-ground test first, clear area, keep the physical E-stop in reach.
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

    motion_control_enabled = LaunchConfiguration("motion_control_enabled")
    use_rviz = LaunchConfiguration("rviz")

    sensors_launch = os.path.join(bringup, "launch", "sensors.launch.py")
    base_launch = os.path.join(bringup, "launch", "base.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument(
            "motion_control_enabled", default_value="false",
            description="HIGH RISK. true lets /cmd_vel_safe write real motor speeds. "
                        "Default false = read-only (verify the chain without moving)."),
        DeclareLaunchArgument("rviz", default_value="true"),

        LogInfo(msg="[manual_teleop] single LEFT XT-M60, manual teleop only. "
                    "No Nav2 / RTAB-Map / explorer. Drive from the RViz TeleopPanel."),

        # Sensors + TF: left radar, IMU, ultrasonics, left camera. Right radar
        # left off (broken hardware).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch),
            launch_arguments={
                "mode": "real",
                "enable_xtm60": "false",
                "enable_xtm60_left": "true",
                "enable_xtm60_right": "false",
                "enable_imu": "true",
                "enable_ultrasonic": "true",
                "enable_camera": "true",
            }.items(),
        ),

        # Left point cloud -> /scan_left, then merged -> /scan.
        Node(
            package="wheelchair_perception", executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan_left_node", output="screen",
            parameters=[os.path.join(bringup, "config", "pointcloud_to_scan_left.yaml")],
        ),
        Node(
            package="wheelchair_perception", executable="scan_merger_node",
            name="scan_merger_node", output="screen",
            parameters=[os.path.join(bringup, "config", "scan_merger_left_only.yaml")],
        ),

        # Wheel + IMU EKF: owns odom->base_link TF and publishes /odometry/filtered.
        Node(
            package="robot_localization", executable="ekf_node", name="ekf_filter_node",
            output="screen",
            parameters=[os.path.join(bringup, "config", "robot_localization_ekf.yaml")],
        ),

        # Watchdog with the LEFT-ONLY profile so the broken right radar is not
        # treated as a critical sensor (otherwise it forces a permanent stop).
        Node(
            package="wheelchair_diagnostics", executable="sensor_watchdog_node",
            name="sensor_watchdog_node", output="screen",
            parameters=[os.path.join(bringup, "config", "diagnostics_left_lidar_lab.yaml")],
        ),

        # Software E-stop publisher (/emergency_stop_sw) used by the UI/keyboard.
        Node(
            package="wheelchair_safety", executable="emergency_stop_node",
            name="emergency_stop_node", output="screen",
            parameters=[PathJoinSubstitution([bringup_share, "config", "safety_params.yaml"])],
        ),

        # Safety supervisor: /cmd_vel_nav -> /cmd_vel_safe. Manual teleop still
        # flows through here; it slows/stops on obstacles and E-stop.
        Node(
            package="wheelchair_safety", executable="safety_supervisor_node",
            name="safety_supervisor_node", output="screen",
            parameters=[
                PathJoinSubstitution([bringup_share, "config", "safety_params.yaml"]),
                {"require_localization_healthy": False},
            ],
        ),

        # Base driver. Motors write only when motion_control_enabled:=true.
        # EKF owns odom->base_link, so the base must not also publish that TF.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                "mode": "real",
                "motion_control_enabled": motion_control_enabled,
                "publish_tf": "false",
            }.items(),
        ),

        # RViz with the sensor view + embedded TeleopPanel.
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", PathJoinSubstitution(
                [bringup_share, "rviz", "sensor_view.rviz"])],
            condition=IfCondition(use_rviz),
        ),
    ])
