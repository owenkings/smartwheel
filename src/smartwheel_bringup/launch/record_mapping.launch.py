from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    topics = [
        "/lidar/left/points_raw", "/lidar/right/points_raw", "/imu/data_raw",
        "/wheel/encoder_counts", "/wheel/odom", "/lio/odom", "/lio/path", "/odom/fused",
        "/camera/front/image_raw", "/camera/front/camera_info", "/camera/left/image_raw",
        "/camera/left/camera_info", "/camera/right/image_raw", "/camera/right/camera_info",
        "/camera/rear/image_raw", "/camera/rear/camera_info", "/tf", "/tf_static",
        "/lidar/merged/points", "/scan", "/sim/ground_truth/odom", "/sim/completed",
        "/map_products/occupancy", "/map_products/cloud", "/hardware/status",
        "/diagnostics", "/mapping/status",
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="mock", choices=["mock", "real"]),
            DeclareLaunchArgument("record_bag", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("enable_cameras", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("hardware_enabled", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("bag_path", default_value="bags/mapping_session"),
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-o", LaunchConfiguration("bag_path"), *topics],
                output="screen",
                condition=IfCondition(LaunchConfiguration("record_bag")),
            ),
        ]
    )
