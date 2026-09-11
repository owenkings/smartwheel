from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("wheelchair_3d_mapping")
    use_sim_time = LaunchConfiguration("use_sim_time")
    config = LaunchConfiguration("config")
    allow_single_lidar_fallback = LaunchConfiguration("allow_single_lidar_fallback")
    enable_left_input = LaunchConfiguration("enable_left_input")
    enable_right_input = LaunchConfiguration("enable_right_input")
    require_synchronized_pair = LaunchConfiguration("require_synchronized_pair")
    max_pair_time_difference_sec = LaunchConfiguration("max_pair_time_difference_sec")
    require_intensity = LaunchConfiguration("require_intensity")
    require_nonzero_timestamps = LaunchConfiguration("require_nonzero_timestamps")
    left_points_topic = LaunchConfiguration("left_points_topic")
    right_points_topic = LaunchConfiguration("right_points_topic")
    output_topic = LaunchConfiguration("output_topic")
    status_topic = LaunchConfiguration("status_topic")
    target_frame = LaunchConfiguration("target_frame")
    motion_compensation = LaunchConfiguration("motion_compensation")
    motion_fixed_frame = LaunchConfiguration("motion_fixed_frame")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "config",
            default_value=PathJoinSubstitution([pkg, "config", "dual_lidar_fusion.yaml"]),
            description="dual_lidar_cloud_fusion_node parameter file.",
        ),
        DeclareLaunchArgument(
            "allow_single_lidar_fallback",
            default_value="true",
            description="Allow mapping to continue from one lidar for an approved hardware profile.",
        ),
        DeclareLaunchArgument("enable_left_input", default_value="true"),
        DeclareLaunchArgument("enable_right_input", default_value="true"),
        DeclareLaunchArgument("left_points_topic", default_value="/xtm60/left/points"),
        DeclareLaunchArgument("right_points_topic", default_value="/xtm60/right/points"),
        DeclareLaunchArgument("output_topic", default_value="/points_merged"),
        DeclareLaunchArgument("status_topic", default_value="/points_merged/status"),
        DeclareLaunchArgument("target_frame", default_value="base_link"),
        DeclareLaunchArgument(
            "motion_compensation", default_value="false", choices=["true", "false"],
            description="Transform staggered clouds into one acquisition time using continuous two-time TF.",
        ),
        DeclareLaunchArgument(
            "motion_fixed_frame", default_value="",
            description="Required with motion_compensation: continuous odometric frame, not loop-corrected map.",
        ),
        DeclareLaunchArgument(
            "require_synchronized_pair",
            default_value="false",
            choices=["true", "false"],
            description="Reject dual output when source timestamps exceed the configured pair limit.",
        ),
        DeclareLaunchArgument("max_pair_time_difference_sec", default_value="0.075"),
        DeclareLaunchArgument(
            "require_intensity",
            default_value="false",
            choices=["true", "false"],
            description="Reject output unless every participating cloud has a real intensity field.",
        ),
        DeclareLaunchArgument(
            "require_nonzero_timestamps",
            default_value="false",
            choices=["true", "false"],
            description="Reject zero-stamped source clouds in a formal acquisition route.",
        ),
        Node(
            package="wheelchair_3d_mapping",
            executable="dual_lidar_cloud_fusion_node",
            name="dual_lidar_cloud_fusion_node",
            output="screen",
            parameters=[
                config,
                {
                    "use_sim_time": use_sim_time,
                    "allow_single_lidar_fallback": allow_single_lidar_fallback,
                    "enable_left_input": enable_left_input,
                    "enable_right_input": enable_right_input,
                    "require_synchronized_pair": require_synchronized_pair,
                    "max_pair_time_difference_sec": max_pair_time_difference_sec,
                    "require_intensity": require_intensity,
                    "require_nonzero_timestamps": require_nonzero_timestamps,
                    "left_points_topic": left_points_topic,
                    "right_points_topic": right_points_topic,
                    "output_topic": output_topic,
                    "status_topic": status_topic,
                    "target_frame": target_frame,
                    "motion_compensation": motion_compensation,
                    "motion_fixed_frame": ParameterValue(motion_fixed_frame, value_type=str),
                },
            ],
        ),
    ])
