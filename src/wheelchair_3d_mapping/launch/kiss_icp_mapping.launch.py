"""KISS-ICP fallback 3D mapping.

Input /points_merged -> /kiss/odom, /kiss/path, /kiss/map_cloud + TF
odom->base_link. Use when the RTAB-Map main line is blocked (camera/RGB-D or
other). With bringup_sensors:=true also starts sensors + dual-lidar fusion
(no base/EKF) so kiss_icp_mapping_node uniquely owns odom->base_link.

Requires the kiss-icp core: pip install --user kiss-icp
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    mapping = get_package_share_directory("wheelchair_3d_mapping")
    bringup = get_package_share_directory("wheelchair_bringup")

    def s(name):
        return LaunchConfiguration(name).perform(context).strip()

    use_sim = s("use_sim_time")
    actions = []

    if s("bringup_sensors").lower() == "true":
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "sensors.launch.py")),
            launch_arguments={
                "mode": "real", "enable_xtm60": "false",
                "enable_xtm60_left": "true", "enable_xtm60_right": "true",
                "enable_imu": "true", "enable_ultrasonic": "false", "enable_camera": "false",
            }.items()))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mapping, "launch", "dual_lidar_fusion.launch.py")),
            launch_arguments={"use_sim_time": use_sim}.items()))

    actions.append(Node(
        package="wheelchair_3d_mapping", executable="kiss_icp_mapping_node",
        name="kiss_icp_mapping_node", output="screen",
        parameters=[{
            "cloud_topic": s("points_topic"),
            "odom_topic": s("odom_topic"),
            "path_topic": s("path_topic"),
            "map_cloud_topic": s("map_cloud_topic"),
            "deskew": s("deskew").lower() == "true",
            "voxel_size": float(s("voxel_size")),
            "max_range": float(s("max_range")),
            "min_range": float(s("min_range")),
            "map_publish_every": int(s("map_publish_every")),
            "publish_tf": s("publish_tf").lower() == "true",
            "use_sim_time": use_sim == "true",
        }]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("points_topic", default_value="/points_merged"),
        DeclareLaunchArgument("odom_topic", default_value="/kiss/odom"),
        DeclareLaunchArgument("path_topic", default_value="/kiss/path"),
        DeclareLaunchArgument("map_cloud_topic", default_value="/kiss/map_cloud"),
        DeclareLaunchArgument("deskew", default_value="false",
                              description="XT-M60 flash ToF has no per-point time; keep false."),
        DeclareLaunchArgument("voxel_size", default_value="0.5"),
        DeclareLaunchArgument("max_range", default_value="20.0"),
        DeclareLaunchArgument("min_range", default_value="0.3"),
        DeclareLaunchArgument("map_publish_every", default_value="5",
                              description="Publish /kiss/map_cloud every N frames (throttles the heavy cloud)."),
        DeclareLaunchArgument("publish_tf", default_value="true",
                              description="kiss_icp owns odom->base_link; set false if another owner runs."),
        DeclareLaunchArgument("bringup_sensors", default_value="false",
                              description="Also start sensors + dual-lidar fusion (no base/EKF)."),
        OpaqueFunction(function=_setup),
    ])
