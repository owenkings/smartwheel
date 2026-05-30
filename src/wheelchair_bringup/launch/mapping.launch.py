from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock = LaunchConfiguration("use_mock")
    use_rviz = LaunchConfiguration("use_rviz")
    enable_ui = LaunchConfiguration("enable_ui")
    enable_native_gui = LaunchConfiguration("enable_native_gui")
    enable_dual_xtm60 = LaunchConfiguration("enable_dual_xtm60")
    # When use_ekf=true, robot_localization's ekf_filter_node fuses
    # /wheel/odom and /imu/data into /odometry/filtered and publishes the
    # odom -> base_link TF. zlac8030_driver_node is told to NOT publish that
    # TF (publish_tf:=false) so we don't get two competing TF publishers.
    # When use_ekf=false, the wheel driver alone publishes the TF and EKF
    # is not started. Keeping a single switch makes it easy to compare
    # mapping quality with and without IMU fusion.
    use_ekf = LaunchConfiguration("use_ekf")
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
    use_real_dual_xtm60 = PythonExpression(
        ["'", use_mock, "' == 'false' and '", enable_dual_xtm60, "' == 'true'"]
    )
    use_single_scan_projection = PythonExpression(
        ["'", use_mock, "' == 'true' or '", enable_dual_xtm60, "' == 'false'"]
    )
    # When EKF runs, base must not publish odom -> base_link TF.
    base_publish_tf = PythonExpression(
        ["'true' if '", use_ekf, "' == 'false' else 'false'"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("enable_ui", default_value="false"),
            DeclareLaunchArgument("enable_native_gui", default_value="false"),
            DeclareLaunchArgument("ui_port", default_value="8080"),
            DeclareLaunchArgument("enable_dual_xtm60", default_value="true"),
            DeclareLaunchArgument(
                "use_ekf",
                default_value="false",
                description="Enable robot_localization EKF to fuse /wheel/odom + /imu/data; EKF then publishes odom->base_link TF and zlac8030 stops publishing it.",
            ),
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
                condition=IfCondition(
                    PythonExpression(
                        ["'", use_mock, "' == 'false' and '", enable_dual_xtm60, "' == 'false'"]
                    )
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "sensors.launch.py"])
                ),
                launch_arguments={
                    "mode": "real",
                    "publish_description": "false",
                    "enable_xtm60": "false",
                    "enable_xtm60_left": "true",
                    "enable_xtm60_right": "true",
                }.items(),
                condition=IfCondition(use_real_dual_xtm60),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "base.launch.py"])
                ),
                launch_arguments={
                    "mode": "real",
                    "publish_tf": base_publish_tf,
                }.items(),
                condition=UnlessCondition(use_mock),
            ),
            # robot_localization EKF (only if use_ekf=true). It subscribes to
            # /wheel/odom and /imu/data, publishes /odometry/filtered, and
            # broadcasts the odom -> base_link TF that slam_toolbox uses as
            # its motion prior. With IMU fused yaw, scan-matching survives
            # narrow-FOV pivots that previously made the map collapse.
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
                condition=IfCondition(use_single_scan_projection),
            ),
            Node(
                package="wheelchair_perception",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan_left_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "pointcloud_to_scan_left.yaml"])
                ],
                condition=IfCondition(use_real_dual_xtm60),
            ),
            Node(
                package="wheelchair_perception",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan_right_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "pointcloud_to_scan_right.yaml"])
                ],
                condition=IfCondition(use_real_dual_xtm60),
            ),
            Node(
                package="wheelchair_perception",
                executable="scan_merger_node",
                name="scan_merger_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "scan_merger.yaml"])
                ],
                condition=IfCondition(use_real_dual_xtm60),
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
