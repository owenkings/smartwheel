from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_file = LaunchConfiguration("map")
    ui_port = LaunchConfiguration("ui_port")
    enable_web_ui = LaunchConfiguration("enable_web_ui")
    enable_native_gui = LaunchConfiguration("enable_native_gui")
    enable_dual_xtm60 = LaunchConfiguration("enable_dual_xtm60")
    # Top-level switch for whether ANY XT-M60 radar adapter starts. When false,
    # neither the single-radar adapter nor the dual left/right adapters run,
    # nor do the pointcloud_to_laserscan / scan_merger downstream nodes. The
    # rest of the bringup (UI, IMU, ultrasonic, camera, base, Nav2) still
    # comes up so the system is usable for non-radar tasks (manual driving,
    # IMU/camera inspection, parameter tuning) without leaving the radar
    # actively scanning. Set to true when mapping or autonomous navigation is
    # required.
    enable_xtm60_radar = LaunchConfiguration("enable_xtm60_radar")

    # Boolean expressions for downstream node gating. The pointcloud_to_scan
    # and scan_merger nodes only have meaningful work when the radar adapter
    # they depend on is also running.
    single_radar_active = PythonExpression(
        ["'", enable_xtm60_radar, "' == 'true' and '", enable_dual_xtm60, "' == 'false'"]
    )
    dual_radar_active = PythonExpression(
        ["'", enable_xtm60_radar, "' == 'true' and '", enable_dual_xtm60, "' == 'true'"]
    )

    bringup_share = FindPackageShare("wheelchair_bringup")
    navigation_share = FindPackageShare("wheelchair_navigation")

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=PathJoinSubstitution([bringup_share, "config", "empty_map.yaml"])),
            DeclareLaunchArgument("ui_port", default_value="8080"),
            DeclareLaunchArgument("enable_web_ui", default_value="true"),
            DeclareLaunchArgument("enable_native_gui", default_value="false"),
            DeclareLaunchArgument("enable_dual_xtm60", default_value="true"),
            DeclareLaunchArgument(
                "enable_xtm60_radar",
                default_value="false",
                description=(
                    "Start XT-M60 radar adapter(s). Default false so the radar"
                    " stays in standby (powered, not scanning) when ROS"
                    " auto-starts. Set true to enable mapping / Nav2."
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "sensors.launch.py"])
                ),
                launch_arguments={
                    "mode": "real",
                    "enable_xtm60": enable_xtm60_radar,
                }.items(),
                condition=UnlessCondition(enable_dual_xtm60),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "sensors.launch.py"])
                ),
                launch_arguments={
                    "mode": "real",
                    "enable_xtm60": "false",
                    "enable_xtm60_left": enable_xtm60_radar,
                    "enable_xtm60_right": enable_xtm60_radar,
                }.items(),
                condition=IfCondition(enable_dual_xtm60),
            ),
            Node(
                package="wheelchair_perception",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "pointcloud_to_scan.yaml"])
                ],
                condition=IfCondition(single_radar_active),
            ),
            Node(
                package="wheelchair_perception",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan_left_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "pointcloud_to_scan_left.yaml"])
                ],
                condition=IfCondition(dual_radar_active),
            ),
            Node(
                package="wheelchair_perception",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan_right_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "pointcloud_to_scan_right.yaml"])
                ],
                condition=IfCondition(dual_radar_active),
            ),
            Node(
                package="wheelchair_perception",
                executable="scan_merger_node",
                name="scan_merger_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution([bringup_share, "config", "scan_merger.yaml"])
                ],
                condition=IfCondition(dual_radar_active),
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
                condition=IfCondition(enable_web_ui),
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
