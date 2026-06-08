from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_file = LaunchConfiguration("map")
    use_ekf = LaunchConfiguration("use_ekf")
    startup_localization_mode = LaunchConfiguration("startup_localization_mode")
    startup_named_goals_path = LaunchConfiguration("startup_named_goals_path")
    startup_named_goal_name = LaunchConfiguration("startup_named_goal_name")
    startup_fixed_x = LaunchConfiguration("startup_fixed_x")
    startup_fixed_y = LaunchConfiguration("startup_fixed_y")
    startup_fixed_yaw = LaunchConfiguration("startup_fixed_yaw")
    startup_anchor_topic = LaunchConfiguration("startup_anchor_topic")
    bringup_share = FindPackageShare("wheelchair_bringup")
    navigation_share = FindPackageShare("wheelchair_navigation")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "empty_map.yaml"]
                ),
                description="Path to saved OccupancyGrid yaml map.",
            ),
            DeclareLaunchArgument(
                "use_ekf",
                default_value="false",
                description="Enable robot_localization EKF after wheel odom is wired.",
            ),
            DeclareLaunchArgument(
                "startup_localization_mode",
                default_value="disabled",
                description="disabled, named_goal, fixed, or external_anchor.",
            ),
            DeclareLaunchArgument(
                "startup_named_goals_path",
                default_value=PathJoinSubstitution(
                    [navigation_share, "config", "named_goals.yaml"]
                ),
            ),
            DeclareLaunchArgument("startup_named_goal_name", default_value="charging"),
            DeclareLaunchArgument("startup_fixed_x", default_value="0.0"),
            DeclareLaunchArgument("startup_fixed_y", default_value="0.0"),
            DeclareLaunchArgument("startup_fixed_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "startup_anchor_topic",
                default_value="/localization/anchor_pose",
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "nav2_params.yaml"]),
                    {"yaml_filename": map_file},
                ],
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "nav2_params.yaml"])
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {"use_sim_time": False},
                    {"autostart": True},
                    {"node_names": ["map_server", "amcl"]},
                ],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [bringup_share, "config", "robot_localization_ekf.yaml"]
                    )
                ],
                condition=IfCondition(use_ekf),
            ),
            Node(
                package="wheelchair_navigation",
                executable="startup_localization_node",
                name="startup_localization_node",
                output="screen",
                parameters=[
                    {
                        "mode": startup_localization_mode,
                        "named_goals_path": startup_named_goals_path,
                        "named_goal_name": startup_named_goal_name,
                        "fixed_x": startup_fixed_x,
                        "fixed_y": startup_fixed_y,
                        "fixed_yaw": startup_fixed_yaw,
                        "anchor_topic": startup_anchor_topic,
                    }
                ],
            ),
        ]
    )
