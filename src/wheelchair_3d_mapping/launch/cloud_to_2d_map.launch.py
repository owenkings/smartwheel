from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("wheelchair_3d_mapping")
    use_sim_time = LaunchConfiguration("use_sim_time")
    config = LaunchConfiguration("config")
    input_cloud_topic = LaunchConfiguration("input_cloud_topic")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "config",
            default_value=PathJoinSubstitution([pkg, "config", "cloud_to_occupancy_grid.yaml"]),
        ),
        DeclareLaunchArgument(
            "input_cloud_topic",
            default_value="/livo/cloud_registered",
            description="3D cloud to project (e.g. /livo/cloud_registered or /livo/map_cloud).",
        ),
        Node(
            package="wheelchair_3d_mapping",
            executable="cloud_to_occupancy_grid_node",
            name="cloud_to_occupancy_grid_node",
            output="screen",
            parameters=[config, {"use_sim_time": use_sim_time, "input_cloud_topic": input_cloud_topic}],
        ),
    ])
