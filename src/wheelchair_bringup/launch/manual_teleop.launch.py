"""Manual teleop + live hardware operator workbench.

Brings up the selected XT-M60 radar(s), H30, FD07-34R array, four-camera
qualified profile, safe teleop chain, and the hardware RViz workbench. Nav2,
RTAB-Map and the autonomous explorer are not started.

Chain (safety never bypassed):
  RViz SmartWheel/Teleop -> /teleop/cmd_vel -> safety_supervisor
  -> /cmd_vel_safe -> base

Nodes started:
  - robot_state_publisher (TF)
  - selected XT-M60(s), IMU, ultrasonics, four cameras (real sensors)
  - pointcloud_to_laserscan for each selected radar + scan_merger -> /scan
  - EKF (wheel + IMU) -> odom->base_link TF + /odometry/filtered
  - sensor_watchdog with a selectable diagnostics profile
  - safety_supervisor (manned safety profile)
  - zlac8030 base driver

SAFETY: motors move only with motion_control_enabled:=true. Default false is
read-only. Off-ground test first, clear area, keep the physical E-stop in reach.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _setup(context, *args, **kwargs):
    bringup = get_package_share_directory("wheelchair_bringup")

    def s(name):
        return LaunchConfiguration(name).perform(context).strip()

    # radar selection: left | right | both. Controls which XT-M60 drivers start,
    # which pointcloud_to_laserscan nodes run, and which scan_merger config is used.
    radar = s("radar").lower()
    if radar not in ("left", "right", "both"):
        radar = "right"
    use_left = radar in ("left", "both")
    use_right = radar in ("right", "both")
    enable_ekf = s("enable_ekf").lower() == "true"

    motion_control_enabled = LaunchConfiguration("motion_control_enabled")
    use_rviz = LaunchConfiguration("rviz")
    safety_params_file = LaunchConfiguration("safety_params_file")
    diagnostics_config = LaunchConfiguration("diagnostics_config")
    camera_config = LaunchConfiguration("camera_config")
    split_camera_processes = LaunchConfiguration("split_camera_processes")
    teleop_topic = LaunchConfiguration("teleop_topic")
    rviz_config = LaunchConfiguration("rviz_config")

    sensors_launch = os.path.join(bringup, "launch", "sensors.launch.py")
    base_launch = os.path.join(bringup, "launch", "base.launch.py")

    # Use an exact single-source config for a single-radar profile. This avoids
    # retaining a silent left-radar placeholder in the current right-only baseline.
    if radar == "left":
        scan_merger_cfg = os.path.join(bringup, "config", "scan_merger_left_only.yaml")
    elif radar == "right":
        scan_merger_cfg = os.path.join(bringup, "config", "scan_merger_right_only.yaml")
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
                "camera_config": camera_config,
                "split_camera_processes": split_camera_processes,
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

    # Hardware watchdog. The default profile follows the current right-radar
    # baseline and therefore does not wait for an intentionally disabled left unit.
    actions.append(Node(
        package="wheelchair_diagnostics", executable="sensor_watchdog_node",
        name="sensor_watchdog_node", output="screen",
        parameters=[diagnostics_config],
    ))

    # Software E-stop publisher.
    actions.append(Node(
        package="wheelchair_safety", executable="emergency_stop_node",
        name="emergency_stop_node", output="screen",
        parameters=[safety_params_file],
    ))

    # Safety supervisor: selected teleop input -> /cmd_vel_safe.
    actions.append(Node(
        package="wheelchair_safety", executable="safety_supervisor_node",
        name="safety_supervisor_node", output="screen",
        parameters=[
            safety_params_file,
            {
                "require_localization_healthy": False,
                "cmd_vel_nav_topic": teleop_topic,
            },
        ],
    ))

    # Base driver. Motors write only when motion_control_enabled:=true.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            "mode": "real",
            "motion_control_enabled": motion_control_enabled,
            "publish_tf": "false" if enable_ekf else "true",
        }.items(),
    ))

    # Optional 2D occupancy mapping. This is deliberately opt-in: the live
    # top-down /scan view works without claiming that a map has been validated.
    actions.append(TimerAction(
        period=8.0,
        actions=[Node(
            package="slam_toolbox", executable="async_slam_toolbox_node",
            name="slam_toolbox", output="screen",
            parameters=[os.path.join(bringup, "config", "slam_toolbox_params.yaml")],
            condition=IfCondition(LaunchConfiguration("enable_2d_mapping")),
        )],
    ))

    # RViz with live dual clouds, merged 2D scan, four compressed cameras and
    # the safety-chain teleop panel.
    actions.append(Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=[
            "-d", rviz_config,
            "--qwindowgeometry", LaunchConfiguration("rviz_window_geometry"),
        ],
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
            "radar", default_value="right",
            description="Which XT-M60(s) to use: left | right | both. Selects the "
                        "radar drivers, the pointcloud_to_laserscan nodes and the "
                        "scan_merger config. Current production default: right."),
        DeclareLaunchArgument(
            "teleop_topic", default_value="/teleop/cmd_vel",
            description="Input to the safety supervisor. The SmartWheel RViz panel is "
                        "locked to /teleop/cmd_vel."),
        DeclareLaunchArgument(
            "camera_config",
            default_value=PathJoinSubstitution(
                [bringup_share, "config", "camera_quad.yaml"]),
            description="Four-camera hardware profile used by the operator workbench."),
        DeclareLaunchArgument(
            "split_camera_processes", default_value="true",
            description="Isolate the four UVC streams so one damaged stream cannot stall the rest."),
        DeclareLaunchArgument(
            "diagnostics_config",
            default_value=PathJoinSubstitution(
                [bringup_share, "config", "diagnostics_right_lidar_mapping.yaml"])),
        DeclareLaunchArgument(
            "enable_2d_mapping", default_value="true",
            description="Publish the live /map preview with slam_toolbox. This enables the "
                        "2D workbench view but does not validate or save the resulting map. "
                        "Set false for a sensor-only session."),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [bringup_share, "rviz", "hardware_operator_workbench.rviz"])),
        DeclareLaunchArgument(
            "rviz_window_geometry", default_value="1920x1080+0+0",
            description="Qt main-window geometry. The Orin operator display defaults to 1920x1080; "
                        "override this argument for a different monitor."),
        DeclareLaunchArgument(
            "safety_params_file",
            default_value=PathJoinSubstitution(
                [bringup_share, "config", "safety_params_manual_mapping.yaml"]),
            description="Safety profile. Default = human-supervised manual-mapping profile "
                        "(ignores the chair's own structure on ultrasonic, tolerates brief "
                        "LiDAR drop-outs). Pass safety_params.yaml for the strict profile."),
        OpaqueFunction(function=_setup),
    ])
