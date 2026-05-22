from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    ui_port = LaunchConfiguration("ui_port")
    obstacle_distance = LaunchConfiguration("obstacle_distance")
    cycle_obstacle = LaunchConfiguration("cycle_obstacle")

    bringup_share = FindPackageShare("wheelchair_bringup")
    description_share = FindPackageShare("wheelchair_description")
    navigation_share = FindPackageShare("wheelchair_navigation")

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    PathJoinSubstitution(
                        [description_share, "urdf", "wheelchair.urdf.xacro"]
                    ),
                ]
            ),
            value_type=str,
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("ui_port", default_value="8080"),
            DeclareLaunchArgument("obstacle_distance", default_value="2.0"),
            DeclareLaunchArgument("cycle_obstacle", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="wheelchair_sensors",
                executable="mock_sensor_node",
                name="mock_sensor_node",
                output="screen",
                parameters=[
                    {
                        "front_obstacle_distance": obstacle_distance,
                        "cycle_obstacle": cycle_obstacle,
                        "publish_scan_directly": False,
                        "publish_cmd_vel_nav": True,
                        "publish_odom": False,
                        "publish_map_to_odom_tf": True,
                        "publish_odom_to_base_tf": False,
                    }
                ],
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
            Node(
                package="wheelchair_perception",
                executable="obstacle_detector_node",
                name="obstacle_detector_node",
                output="screen",
                parameters=[{"scan_topic": "/scan", "obstacles_topic": "/obstacles"}],
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
                    PathJoinSubstitution([bringup_share, "config", "safety_params.yaml"])
                ],
            ),
            Node(
                package="wheelchair_base",
                executable="zlac8030_driver_node",
                name="zlac8030_driver_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "zlac8030_base.yaml"]),
                    {"mode": "mock"},
                ],
            ),
            Node(
                package="wheelchair_diagnostics",
                executable="sensor_watchdog_node",
                name="sensor_watchdog_node",
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
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution([bringup_share, "rviz", "wheelchair_default.rviz"]),
                ],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
