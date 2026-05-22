from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    bringup_share = FindPackageShare("wheelchair_bringup")
    navigation_share = FindPackageShare("wheelchair_navigation")

    nav2_launch = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "nav2_params.yaml"]
                ),
            ),
            GroupAction(
                [
                    SetRemap(src="/cmd_vel", dst="/cmd_vel_nav"),
                    SetRemap(src="cmd_vel", dst="/cmd_vel_nav"),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(nav2_launch),
                        launch_arguments={
                            "use_sim_time": "false",
                            "params_file": params_file,
                            "autostart": "true",
                        }.items(),
                    ),
                ]
            ),
            Node(
                package="wheelchair_perception",
                executable="passability_analyzer_node",
                name="passability_analyzer_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "passability.yaml"])
                ],
            ),
            Node(
                package="wheelchair_safety",
                executable="safety_supervisor_node",
                name="safety_supervisor_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "safety_params.yaml"]),
                    {"require_localization_healthy": True},
                ],
            ),
            Node(
                package="wheelchair_diagnostics",
                executable="localization_health_node",
                name="localization_health_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "diagnostics.yaml"])
                ],
            ),
            Node(
                package="wheelchair_navigation",
                executable="goal_manager_node",
                name="goal_manager_node",
                output="screen",
                parameters=[
                    {
                        "named_goals_path": PathJoinSubstitution(
                            [navigation_share, "config", "named_goals.yaml"]
                        )
                    }
                ],
            ),
            Node(
                package="wheelchair_navigation",
                executable="navigation_status_node",
                name="navigation_status_node",
                output="screen",
            ),
        ]
    )
