from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    publish_description = LaunchConfiguration("publish_description")
    enable_xtm60 = LaunchConfiguration("enable_xtm60")
    enable_imu = LaunchConfiguration("enable_imu")
    enable_ultrasonic = LaunchConfiguration("enable_ultrasonic")
    enable_camera = LaunchConfiguration("enable_camera")

    bringup_share = FindPackageShare("wheelchair_bringup")
    description_share = FindPackageShare("wheelchair_description")
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
            DeclareLaunchArgument(
                "mode",
                default_value="real",
                description="real reads configured hardware; mock publishes synthetic data.",
            ),
            DeclareLaunchArgument(
                "publish_description",
                default_value="true",
                description="Start robot_state_publisher for the wheelchair URDF.",
            ),
            DeclareLaunchArgument("enable_xtm60", default_value="true"),
            DeclareLaunchArgument("enable_imu", default_value="true"),
            DeclareLaunchArgument("enable_ultrasonic", default_value="true"),
            DeclareLaunchArgument("enable_camera", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
                condition=IfCondition(publish_description),
            ),
            Node(
                package="wheelchair_sensors",
                executable="xtm60_adapter_node",
                name="xtm60_adapter_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "xtm60_sdk.yaml"]),
                    {"mode": mode},
                ],
                condition=IfCondition(enable_xtm60),
            ),
            Node(
                package="wheelchair_sensors",
                executable="imu_adapter_node",
                name="imu_adapter_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "h30_imu.yaml"]),
                    {"mode": mode},
                ],
                condition=IfCondition(enable_imu),
            ),
            Node(
                package="wheelchair_sensors",
                executable="ultrasonic_adapter_node",
                name="ultrasonic_adapter_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "ultrasonic.yaml"]),
                    {"mode": mode},
                ],
                condition=IfCondition(enable_ultrasonic),
            ),
            Node(
                package="wheelchair_sensors",
                executable="camera_adapter_node",
                name="camera_adapter_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "camera.yaml"]),
                    {"mode": mode},
                ],
                condition=IfCondition(enable_camera),
            ),
        ]
    )
