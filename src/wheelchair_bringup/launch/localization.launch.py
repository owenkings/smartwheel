from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_file = LaunchConfiguration("map")
    use_ekf = LaunchConfiguration("use_ekf")
    bringup_share = FindPackageShare("wheelchair_bringup")

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
        ]
    )
