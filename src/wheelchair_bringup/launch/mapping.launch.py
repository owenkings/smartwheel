from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock = LaunchConfiguration("use_mock")
    use_rviz = LaunchConfiguration("use_rviz")
    enable_ui = LaunchConfiguration("enable_ui")
    enable_native_gui = LaunchConfiguration("enable_native_gui")
    ui_port = LaunchConfiguration("ui_port")
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
            DeclareLaunchArgument("use_mock", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("enable_ui", default_value="false"),
            DeclareLaunchArgument("enable_native_gui", default_value="false"),
            DeclareLaunchArgument("ui_port", default_value="8080"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "sensors.launch.py"])
                ),
                launch_arguments={
                    "mode": "real",
                    "publish_description": "false",
                }.items(),
                condition=UnlessCondition(use_mock),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "base.launch.py"])
                ),
                launch_arguments={"mode": "real"}.items(),
                condition=UnlessCondition(use_mock),
            ),
            Node(
                package="wheelchair_sensors",
                executable="mock_sensor_node",
                name="mock_sensor_node",
                output="screen",
                parameters=[{"cycle_obstacle": False, "publish_cmd_vel_nav": False}],
                condition=IfCondition(use_mock),
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
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "slam_toolbox_params.yaml"])
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
                condition=IfCondition(enable_ui),
            ),
            Node(
                package="wheelchair_ui",
                executable="wheelchair_native_gui",
                name="wheelchair_native_gui",
                output="screen",
                arguments=[
                    "--named-goals-path",
                    PathJoinSubstitution([navigation_share, "config", "named_goals.yaml"]),
                    "--semantic-map-path",
                    PathJoinSubstitution([navigation_share, "config", "semantic_map.yaml"]),
                ],
                condition=IfCondition(enable_native_gui),
            ),
        ]
    )
