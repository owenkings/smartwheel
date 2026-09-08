"""Contract-driven FAST-LIO2 odometry for the formal 3D mapping pipeline.

This component never publishes ``map -> camera_init`` and never publishes the
sensor mount transforms.  The parent formal mapping launch owns the approved
``base_link -> sensor`` transforms, FAST-LIO owns ``camera_init -> body``, and
RTAB-Map owns the global map correction.  A missing or non-approved canonical
dual-LiDAR/H30 contract fails before any ROS node is constructed.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from wheelchair_3d_mapping.formal_lio_contract import load_formal_lio_contract


def _number(value: float) -> str:
    return f"{float(value):.12g}"


def _setup(context, *args, **kwargs):
    contract_path = LaunchConfiguration("calibration_contract").perform(context).strip()
    radar = LaunchConfiguration("radar").perform(context).strip().lower()
    if not contract_path:
        raise RuntimeError("FORMAL_LIO_BLOCKED: calibration_contract is required")
    try:
        spec = load_formal_lio_contract(contract_path, radar=radar)
    except ValueError as exc:
        raise RuntimeError(f"FORMAL_LIO_BLOCKED: {exc}") from exc

    mapping_share = get_package_share_directory("wheelchair_3d_mapping")
    config_override = LaunchConfiguration("config_file").perform(context).strip()
    input_override = LaunchConfiguration("input_topic").perform(context).strip()
    imu_topic = LaunchConfiguration("imu_topic").perform(context).strip()
    config_file = config_override or os.path.join(
        mapping_share, "config", spec["config_name"]
    )
    input_topic = input_override or spec["input_topic"]
    if not os.path.isfile(config_file):
        raise RuntimeError(f"FORMAL_LIO_BLOCKED: FAST-LIO config is missing: {config_file}")
    for name, topic in (("input_topic", input_topic), ("imu_topic", imu_topic)):
        if not topic.startswith("/"):
            raise RuntimeError(f"FORMAL_LIO_BLOCKED: {name} must be an absolute topic")

    body_xyz = [_number(value) for value in spec["body_to_base_translation"]]
    body_quaternion = [_number(value) for value in spec["body_to_base_quaternion"]]
    return [
        Node(
            package="wheelchair_3d_mapping",
            executable="lio_cloud_adapter",
            name="formal_lio_cloud_adapter",
            output="screen",
            parameters=[
                {
                    "input_topic": input_topic,
                    "output_topic": "/lio/cloud_in",
                    "restamp_to_now": False,
                    "add_zero_time_field": False,
                    "min_range": 0.3,
                    "max_range": 12.0,
                    "output_qos": "best_effort",
                }
            ],
        ),
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="formal_fastlio_mapping",
            output="screen",
            parameters=[
                config_file,
                {
                    "common.lid_topic": "/lio/cloud_in",
                    "common.imu_topic": imu_topic,
                    "mapping.extrinsic_T": spec["extrinsic_T"],
                    "mapping.extrinsic_R": spec["extrinsic_R"],
                    "mapping.extrinsic_est_en": False,
                },
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="formal_lio_body_to_base_link",
            arguments=[
                "--x",
                body_xyz[0],
                "--y",
                body_xyz[1],
                "--z",
                body_xyz[2],
                "--qx",
                body_quaternion[0],
                "--qy",
                body_quaternion[1],
                "--qz",
                body_quaternion[2],
                "--qw",
                body_quaternion[3],
                "--frame-id",
                "body",
                "--child-frame-id",
                "base_link",
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("calibration_contract", default_value=""),
            DeclareLaunchArgument("radar", default_value="right", choices=["left", "right"]),
            DeclareLaunchArgument("config_file", default_value=""),
            DeclareLaunchArgument("input_topic", default_value=""),
            DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
            OpaqueFunction(function=_setup),
        ]
    )
