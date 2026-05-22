from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("wheelchair_bringup")
    params = PathJoinSubstitution([bringup_share, "config", "diagnostics.yaml"])
    include_localization_health = LaunchConfiguration("include_localization_health")

    return LaunchDescription(
        [
            DeclareLaunchArgument("include_localization_health", default_value="true"),
            Node(
                package="wheelchair_diagnostics",
                executable="sensor_watchdog_node",
                name="sensor_watchdog_node",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="wheelchair_diagnostics",
                executable="localization_health_node",
                name="localization_health_node",
                output="screen",
                parameters=[params],
                condition=IfCondition(include_localization_health),
            ),
        ]
    )
