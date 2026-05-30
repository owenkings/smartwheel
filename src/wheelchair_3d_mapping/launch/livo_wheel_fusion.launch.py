"""Loose-coupling wheel/LIVO fusion: robot_localization EKF + consistency monitor.

The EKF fuses /wheel/odom + /imu/data + /livo/odom into /odometry/filtered.
Whether it publishes odom->base_link is controlled by tf_owner so that exactly
one node owns that TF (see bringup_3d_slam.launch.py).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    bringup = get_package_share_directory("wheelchair_bringup")
    pkg = get_package_share_directory("wheelchair_3d_mapping")
    tf_owner = LaunchConfiguration("tf_owner").perform(context).strip()
    use_wheel = LaunchConfiguration("use_wheel_fusion").perform(context).strip().lower() == "true"
    use_sim = LaunchConfiguration("use_sim_time").perform(context).strip().lower() == "true"

    ekf_yaml = os.path.join(bringup, "config", "robot_localization_livo_wheel_ekf.yaml")
    cons_yaml = os.path.join(pkg, "config", "wheel_livo_consistency.yaml")
    ekf_owns_tf = tf_owner == "ekf"

    actions = [LogInfo(msg=f"[livo_wheel_fusion] tf_owner={tf_owner} ekf_publish_tf={ekf_owns_tf} "
                           f"use_wheel_fusion={use_wheel}")]
    if use_wheel:
        actions.append(Node(
            package="robot_localization", executable="ekf_node", name="ekf_filter_node",
            output="screen",
            parameters=[ekf_yaml, {"publish_tf": ekf_owns_tf, "use_sim_time": use_sim}],
        ))
    actions.append(Node(
        package="wheelchair_3d_mapping", executable="wheel_livo_consistency_monitor",
        name="wheel_livo_consistency_monitor", output="screen",
        parameters=[cons_yaml, {"use_sim_time": use_sim}],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("tf_owner", default_value="ekf",
                              description="ekf | livo | wheel. Only the EKF publishes odom->base_link when 'ekf'."),
        DeclareLaunchArgument("use_wheel_fusion", default_value="true",
                              description="Start the robot_localization EKF. The consistency monitor always runs."),
        OpaqueFunction(function=_setup),
    ])
