from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    require_localization_healthy = LaunchConfiguration("require_localization_healthy")
    enable_passability = LaunchConfiguration("enable_passability")
    safety_params_file = LaunchConfiguration("safety_params_file")
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
            DeclareLaunchArgument(
                "require_localization_healthy",
                default_value="true",
                description="AMCL nav: true. RTAB-Map mapping-while-navigating: false (map->odom comes from SLAM, not AMCL).",
            ),
            DeclareLaunchArgument(
                "enable_passability",
                default_value="true",
                description="Run the scan corridor passability analyzer (hard-stops safety on BLOCKED). "
                            "Set false for the dual flash-LiDAR autonomous mapping mode where it mis-fires; "
                            "Nav2 costmap + safety scan-distance stop remain the obstacle guards.",
            ),
            DeclareLaunchArgument(
                "safety_params_file",
                default_value=PathJoinSubstitution([bringup_share, "config", "safety_params.yaml"]),
                description="Safety profile. Manned default; pass safety_params_mapping.yaml for unmanned low-speed mapping.",
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
                condition=IfCondition(enable_passability),
            ),
            Node(
                package="wheelchair_safety",
                executable="emergency_stop_node",
                name="emergency_stop_node",
                output="screen",
            ),
            Node(
                package="wheelchair_safety",
                executable="safety_supervisor_node",
                name="safety_supervisor_node",
                output="screen",
                parameters=[
                    safety_params_file,
                    {"require_localization_healthy": require_localization_healthy},
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
