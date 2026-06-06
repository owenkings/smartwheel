from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = FindPackageShare("wheelchair_voice_agent")
    audio_config = LaunchConfiguration("audio_config")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "audio_config",
                default_value=PathJoinSubstitution([package, "config", "audio_io.yaml"]),
            ),
            Node(
                package="wheelchair_voice_agent",
                executable="audio_io_bridge_node",
                name="audio_io_bridge_node",
                output="screen",
                parameters=[audio_config],
            ),
            Node(
                package="wheelchair_voice_agent",
                executable="command_parser_node",
                name="command_parser_node",
                output="screen",
            ),
        ]
    )
