from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    navigation_share = FindPackageShare("wheelchair_navigation")
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8080"),
            DeclareLaunchArgument(
                "named_goals_path",
                default_value=PathJoinSubstitution(
                    [navigation_share, "config", "named_goals.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "semantic_map_path",
                default_value=PathJoinSubstitution(
                    [navigation_share, "config", "semantic_map.yaml"]
                ),
            ),
            DeclareLaunchArgument("enabled_cameras", default_value="left,right"),
            DeclareLaunchArgument("ultrasonic_indices", default_value="0,1,2,3"),
            Node(
                package="wheelchair_ui",
                executable="wheelchair_ui",
                name="wheelchair_web_user_map",
                output="screen",
                arguments=[
                    "--host",
                    LaunchConfiguration("host"),
                    "--port",
                    LaunchConfiguration("port"),
                    "--named-goals-path",
                    LaunchConfiguration("named_goals_path"),
                    "--semantic-map-path",
                    LaunchConfiguration("semantic_map_path"),
                    "--enabled-cameras",
                    LaunchConfiguration("enabled_cameras"),
                    "--ultrasonic-indices",
                    LaunchConfiguration("ultrasonic_indices"),
                ],
            ),
        ]
    )
