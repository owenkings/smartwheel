from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    publish_description = LaunchConfiguration("publish_description")
    enable_xtm60 = LaunchConfiguration("enable_xtm60")
    enable_xtm60_left = LaunchConfiguration("enable_xtm60_left")
    enable_xtm60_right = LaunchConfiguration("enable_xtm60_right")
    enable_imu = LaunchConfiguration("enable_imu")
    enable_ultrasonic = LaunchConfiguration("enable_ultrasonic")
    enable_camera = LaunchConfiguration("enable_camera")
    xtm60_config = LaunchConfiguration("xtm60_config")
    xtm60_left_config = LaunchConfiguration("xtm60_left_config")
    xtm60_right_config = LaunchConfiguration("xtm60_right_config")
    xtm60_left_bind_ip = LaunchConfiguration("xtm60_left_bind_ip")
    xtm60_right_bind_ip = LaunchConfiguration("xtm60_right_bind_ip")
    xtm60_bind_port = LaunchConfiguration("xtm60_bind_port")
    ultrasonic_config = LaunchConfiguration("ultrasonic_config")
    camera_config = LaunchConfiguration("camera_config")

    bringup_share = FindPackageShare("wheelchair_bringup")
    bringup_prefix = FindPackagePrefix("wheelchair_bringup")
    xt_bindshim = PathJoinSubstitution(
        [bringup_prefix, "lib", "wheelchair_bringup", "libxt_bindshim.so"]
    )
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
            DeclareLaunchArgument("enable_xtm60_left", default_value="false"),
            DeclareLaunchArgument("enable_xtm60_right", default_value="false"),
            DeclareLaunchArgument("xtm60_left_bind_ip", default_value="192.168.0.100"),
            DeclareLaunchArgument("xtm60_right_bind_ip", default_value="192.168.1.100"),
            DeclareLaunchArgument("xtm60_bind_port", default_value="7687"),
            DeclareLaunchArgument("enable_imu", default_value="true"),
            DeclareLaunchArgument("enable_ultrasonic", default_value="true"),
            DeclareLaunchArgument("enable_camera", default_value="true"),
            DeclareLaunchArgument(
                "xtm60_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "xtm60_sdk.yaml"]),
            ),
            DeclareLaunchArgument(
                "xtm60_left_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "xtm60_left.yaml"]),
            ),
            DeclareLaunchArgument(
                "xtm60_right_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "xtm60_right.yaml"]),
            ),
            DeclareLaunchArgument(
                "ultrasonic_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "ultrasonic.yaml"]),
            ),
            DeclareLaunchArgument(
                "camera_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "camera.yaml"]),
            ),
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
                    xtm60_config,
                    {"mode": mode},
                ],
                condition=IfCondition(enable_xtm60),
            ),
            Node(
                package="wheelchair_sensors",
                executable="xtm60_adapter_node",
                name="xtm60_left_adapter_node",
                output="screen",
                parameters=[
                    xtm60_left_config,
                    {"mode": mode},
                ],
                remappings=[
                    ("/xtm60/points", "/xtm60/left/points"),
                    ("/xtm60/status", "/xtm60/left/status"),
                ],
                additional_env={
                    "LD_PRELOAD": xt_bindshim,
                    "XT_BIND_IP": xtm60_left_bind_ip,
                    "XT_BIND_PORT": xtm60_bind_port,
                },
                condition=IfCondition(enable_xtm60_left),
            ),
            Node(
                package="wheelchair_sensors",
                executable="xtm60_adapter_node",
                name="xtm60_right_adapter_node",
                output="screen",
                parameters=[
                    xtm60_right_config,
                    {"mode": mode},
                ],
                remappings=[
                    ("/xtm60/points", "/xtm60/right/points"),
                    ("/xtm60/status", "/xtm60/right/status"),
                ],
                additional_env={
                    "LD_PRELOAD": xt_bindshim,
                    "XT_BIND_IP": xtm60_right_bind_ip,
                    "XT_BIND_PORT": xtm60_bind_port,
                },
                condition=IfCondition(enable_xtm60_right),
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
                    ultrasonic_config,
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
                    camera_config,
                    {"mode": mode},
                ],
                condition=IfCondition(enable_camera),
            ),
        ]
    )
