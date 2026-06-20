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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _setup(context, *args, **kwargs):
    bringup_share = FindPackageShare("wheelchair_bringup")
    bringup = get_package_share_directory("wheelchair_bringup")

    def s(name):
        return LaunchConfiguration(name).perform(context).strip()

    # radar selection: left | right | both. Controls which XT-M60 drivers start,
    # which pointcloud_to_laserscan nodes run, and which scan_merger config is used.
    radar = s("radar").lower()
    if radar not in ("left", "right", "both"):
        radar = "left"
    use_left = radar in ("left", "both")
    use_right = radar in ("right", "both")

    motion_control_enabled = LaunchConfiguration("motion_control_enabled")
    use_rviz = LaunchConfiguration("rviz")
    safety_params_file = LaunchConfiguration("safety_params_file")

    sensors_launch = os.path.join(bringup, "launch", "sensors.launch.py")
    base_launch = os.path.join(bringup, "launch", "base.launch.py")

    # scan_merger config: left-only uses the single-source config; right or both
    # use the dual config (lists /scan_left + /scan_right, require_all_sources=
    # false so a stale/absent source is ignored).
    if radar == "left":
        scan_merger_cfg = os.path.join(bringup, "config", "scan_merger_left_only.yaml")
    else:
        scan_merger_cfg = os.path.join(bringup, "config", "scan_merger_dual.yaml")

    actions = [
        LogInfo(msg=f"[manual_teleop] radar={radar} (left={use_left} right={use_right}), "
                    "manual teleop only. No Nav2 / RTAB-Map / explorer. Drive from the RViz TeleopPanel."),

        # Sensors + TF: selected radar(s), IMU, ultrasonics, cameras.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch),
            launch_arguments={
                "mode": "real",
                "enable_xtm60": "false",
                "enable_xtm60_left": "true" if use_left else "false",
                "enable_xtm60_right": "true" if use_right else "false",
                "enable_imu": "true",
                "enable_ultrasonic": "true",
                "enable_camera": "true",
            }.items(),
        ),
    ]

    # Left point cloud -> /scan_left (only when the left radar is used).
    if use_left:
        actions.append(Node(
            package="wheelchair_perception", executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan_left_node", output="screen",
            parameters=[os.path.join(bringup, "config", "pointcloud_to_scan_left.yaml")],
        ))
    # Right point cloud -> /scan_right (only when the right radar is used).
    if use_right:
        actions.append(Node(
            package="wheelchair_perception", executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan_right_node", output="screen",
            parameters=[os.path.join(bringup, "config", "pointcloud_to_scan_right.yaml")],
        ))

    # Merge available scans -> /scan.
    actions.append(Node(
        package="wheelchair_perception", executable="scan_merger_node",
        name="scan_merger_node", output="screen",
        parameters=[scan_merger_cfg],
    ))

    # Wheel + IMU EKF: owns odom->base_link TF and publishes /odometry/filtered.
    actions.append(Node(
        package="robot_localization", executable="ekf_node", name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(bringup, "config", "robot_localization_ekf.yaml")],
        condition=IfCondition(LaunchConfiguration("enable_ekf")),
    ))

    # Watchdog with the LEFT-ONLY profile (broken right radar not critical).
    actions.append(Node(
        package="wheelchair_diagnostics", executable="sensor_watchdog_node",
        name="sensor_watchdog_node", output="screen",
        parameters=[os.path.join(bringup, "config", "diagnostics_left_lidar_lab.yaml")],
    ))

    # Software E-stop publisher.
    actions.append(Node(
        package="wheelchair_safety", executable="emergency_stop_node",
        name="emergency_stop_node", output="screen",
        parameters=[safety_params_file],
    ))

    # Safety supervisor: /cmd_vel_nav -> /cmd_vel_safe.
    actions.append(Node(
        package="wheelchair_safety", executable="safety_supervisor_node",
        name="safety_supervisor_node", output="screen",
        parameters=[
            safety_params_file,
            {"require_localization_healthy": False},
        ],
    ))

    # Base driver. Motors write only when motion_control_enabled:=true.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            "mode": "real",
            "motion_control_enabled": motion_control_enabled,
            "publish_tf": "false",
        }.items(),
    ))

    # RViz with the sensor view + embedded TeleopPanel.
    actions.append(Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", PathJoinSubstitution(
            [bringup_share, "rviz", "sensor_view.rviz"])],
        condition=IfCondition(use_rviz),
    ))
    return actions


def generate_launch_description():
    bringup_share = FindPackageShare("wheelchair_bringup")

    return LaunchDescription([
        DeclareLaunchArgument(
            "motion_control_enabled", default_value="false",
            description="HIGH RISK. true lets /cmd_vel_safe write real motor speeds. "
                        "Default false = read-only (verify the chain without moving)."),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "enable_ekf", default_value="true",
            description="Run robot_localization EKF as the odom->base_link owner. "
                        "Set false when an external odometry source (e.g. RTAB-Map "
                        "icp_odometry) owns that TF, to avoid a TF fight."),
        DeclareLaunchArgument(
            "radar", default_value="left",
            description="Which XT-M60(s) to use: left | right | both. Selects the "
                        "radar drivers, the pointcloud_to_laserscan nodes and the "
                        "scan_merger config. Default left."),
        DeclareLaunchArgument(
            "safety_params_file",
            default_value=PathJoinSubstitution(
                [bringup_share, "config", "safety_params_manual_mapping.yaml"]),
            description="Safety profile. Default = human-supervised manual-mapping profile "
                        "(ignores the chair's own structure on ultrasonic, tolerates brief "
                        "LiDAR drop-outs). Pass safety_params.yaml for the strict profile."),
        OpaqueFunction(function=_setup),
    ])
