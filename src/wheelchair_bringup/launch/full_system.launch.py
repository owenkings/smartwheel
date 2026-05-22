from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_file = LaunchConfiguration("map")
    ui_port = LaunchConfiguration("ui_port")

    bringup_share = FindPackageShare("wheelchair_bringup")
    navigation_share = FindPackageShare("wheelchair_navigation")

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=PathJoinSubstitution([bringup_share, "config", "empty_map.yaml"])),
            DeclareLaunchArgument("ui_port", default_value="8080"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "sensors.launch.py"])
                ),
                launch_arguments={"mode": "real"}.items(),
            ),
            Node(
                package="wheelchair_perception",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "pointcloud_to_scan.yaml"])
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "localization.launch.py"])
                ),
                launch_arguments={"map": map_file, "use_ekf": "false"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "navigation.launch.py"])
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "diagnostics.launch.py"])
                ),
                launch_arguments={"include_localization_health": "false"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "base.launch.py"])
                ),
                launch_arguments={"mode": "real"}.items(),
            ),
            Node(
                package="wheelchair_ui",
                executable="wheelchair_ui",
                name="wheelchair_ui",
                output="screen",
                arguments=[
                    "--host",
                    "0.0.0.0",
                    "--port",
                    ui_port,
                    "--named-goals-path",
                    PathJoinSubstitution([navigation_share, "config", "named_goals.yaml"]),
                    "--semantic-map-path",
                    PathJoinSubstitution([navigation_share, "config", "semantic_map.yaml"]),
                ],
            ),
        ]
    )
