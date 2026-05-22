from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("wheelchair_bringup")
    return LaunchDescription(
        [
            Node(
                package="wheelchair_diagnostics",
                executable="hardware_self_check_node",
                name="hardware_self_check_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "diagnostics.yaml"])
                ],
            )
        ]
    )
