"""Stage 0 (RViz-first MVP): single-LEFT-radar sensor visualization. NO motion.

Brings up exactly enough to SEE the sensors in RViz with a valid TF tree:
  - robot_state_publisher (static TF: base_link -> sensors)
  - left XT-M60, IMU, ultrasonics, left camera
  - pointcloud_to_laserscan (left) + scan_merger -> /scan, /scan_left
  - robot_localization EKF -> odom->base_link TF + /odometry/filtered
  - RViz with sensor_view.rviz (Fixed Frame = odom)

It does NOT start safety_supervisor, the base driver, Nav2, RTAB-Map or any
explorer. No /cmd_vel is published; nothing moves. This is the "what data do I
have and are the frames correct?" stage.

The EKF provides odom->base_link from wheel odometry + IMU. With the chair
stationary the frame simply stays at the origin, which is what we want for
visualization. The base driver is not started, so there is no motor risk and
the EKF is the sole odom->base_link publisher.
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

    use_rviz = LaunchConfiguration("rviz")
    sensors_launch = os.path.join(bringup, "launch", "sensors.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),

        LogInfo(msg="[stage0] single-LEFT-radar sensor visualization. "
                    "No motion, no safety/base/Nav2/RTAB-Map. Just see the data in RViz."),

        # Left radar + IMU + ultrasonics + left camera + robot_state_publisher (TF).
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

        # Left point cloud -> /scan_left, merged -> /scan.
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

        # EKF: provides odom->base_link TF so point clouds/scan resolve in RViz
        # (Fixed Frame = odom). Stationary chair stays at origin.
        Node(
            package="robot_localization", executable="ekf_node", name="ekf_filter_node",
            output="screen",
            parameters=[os.path.join(bringup, "config", "robot_localization_ekf.yaml")],
        ),

        # RViz sensor view (embedded TeleopPanel present but harmless: with no
        # safety/base running, nothing moves).
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", PathJoinSubstitution(
                [bringup_share, "rviz", "sensor_view.rviz"])],
            condition=IfCondition(use_rviz),
        ),
    ])
