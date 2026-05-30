from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("wheelchair_3d_mapping")
    use_sim_time = LaunchConfiguration("use_sim_time")
    config = LaunchConfiguration("config")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "config",
            default_value=PathJoinSubstitution([pkg, "config", "dual_lidar_fusion.yaml"]),
            description="dual_lidar_cloud_fusion_node parameter file.",
        ),
        Node(
            package="wheelchair_3d_mapping",
            executable="dual_lidar_cloud_fusion_node",
            name="dual_lidar_cloud_fusion_node",
            output="screen",
            parameters=[config, {"use_sim_time": use_sim_time}],
        ),
    ])
