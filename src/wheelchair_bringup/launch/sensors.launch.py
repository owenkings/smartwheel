import json
import math
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def _flag(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() == "true"


def _finite_runtime_transform(transform: object) -> bool:
    if not isinstance(transform, dict):
        return False
    translation = transform.get("translation", transform.get("xyz"))
    rotation = transform.get("rotation", transform.get("rpy"))
    if not isinstance(translation, (list, tuple)) or not isinstance(rotation, (list, tuple)):
        return False
    values = list(translation) + list(rotation)
    return len(translation) == 3 and len(rotation) == 3 and all(
        not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(float(value))
        for value in values
    )


def _has_expected_frame_pair(value: object, child: str) -> bool:
    return (
        isinstance(value, dict)
        and value.get("parent") == "base_link"
        and value.get("child") == child
    )


def _approved_calibration_contract(path_value: str, *, require_dual: bool = False) -> tuple[bool, str]:
    """Validate canonical dual contracts, with right-diagnostic legacy support.

    The canonical formal contract records all three base-relative transforms.
    A historic single-right contract remains valid only for right-only
    diagnostic entry points; it cannot authorize a dual-lidar launch.
    """
    path = Path(path_value).expanduser().resolve()
    if not path.is_file():
        return False, f"calibration contract does not exist: {path}"
    try:
        contract = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        return False, f"calibration contract is unreadable: {exc}"
    if not isinstance(contract, dict) or str(contract.get("status", "")).upper() != "APPROVED":
        return False, f"calibration contract status is {contract.get('status', '<missing>') if isinstance(contract, dict) else '<invalid>'}"
    transforms = contract.get("runtime_transforms")
    if transforms is not None:
        if not isinstance(transforms, dict):
            return False, "canonical runtime_transforms must be an object"
        for name, child in (
            ("xtm60_left", "xtm60_left_link"),
            ("xtm60_right", "xtm60_right_link"),
            ("imu", "imu_link"),
        ):
            item = transforms.get(name)
            if not isinstance(item, dict):
                return False, f"canonical runtime_transforms.{name} is missing"
            if not _finite_runtime_transform(item.get("runtime_transform")):
                return False, f"canonical runtime_transforms.{name} contains non-finite values"
            if not _has_expected_frame_pair(item.get("frames"), child):
                return False, f"canonical runtime_transforms.{name} has the wrong frame pair"
        return True, str(path)

    if require_dual:
        return False, "dual-lidar route requires canonical runtime_transforms for both XT-M60 sensors and H30"
    if not _finite_runtime_transform(contract.get("runtime_transform")):
        return False, "legacy runtime_transform contains non-finite values"
    if not _has_expected_frame_pair(contract.get("frames"), "xtm60_right_link"):
        return False, "legacy calibration contract must declare base_link->xtm60_right_link"
    return True, str(path)


def _block_unapproved_right_lidar(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    if mode not in ("real", "mock"):
        raise RuntimeError(f"INVALID_SENSOR_MODE: {mode!r}")

    # The legacy single-radar config points to the same right unit
    # (192.168.1.101), so it is covered by the same contract gate.
    right_requested = _flag(context, "enable_xtm60") or _flag(
        context, "enable_xtm60_right"
    )
    dual_requested = _flag(context, "enable_xtm60_left") and right_requested
    if mode == "real" and right_requested:
        if not _flag(context, "allow_approved_right_lidar"):
            raise RuntimeError(
                "BLOCKED_CONFLICT: right_lidar_stage1_calibration has no "
                "runtime-eligible transform; pass an approved contract with "
                "allow_approved_right_lidar:=true"
            )
        contract_value = LaunchConfiguration("calibration_contract").perform(context).strip()
        approved, reason = _approved_calibration_contract(
            contract_value, require_dual=dual_requested
        )
        if not approved:
            raise RuntimeError(f"BLOCKED_CONFLICT: {reason}")
    return []


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    publish_description = LaunchConfiguration("publish_description")
    enable_xtm60 = LaunchConfiguration("enable_xtm60")
    enable_xtm60_left = LaunchConfiguration("enable_xtm60_left")
    enable_xtm60_right = LaunchConfiguration("enable_xtm60_right")
    calibration_contract = LaunchConfiguration("calibration_contract")
    allow_approved_right_lidar = LaunchConfiguration("allow_approved_right_lidar")
    enable_imu = LaunchConfiguration("enable_imu")
    enable_ultrasonic = LaunchConfiguration("enable_ultrasonic")
    enable_camera = LaunchConfiguration("enable_camera")
    split_camera_processes = LaunchConfiguration("split_camera_processes")
    xtm60_config = LaunchConfiguration("xtm60_config")
    xtm60_left_config = LaunchConfiguration("xtm60_left_config")
    xtm60_right_config = LaunchConfiguration("xtm60_right_config")
    xtm60_left_bind_ip = LaunchConfiguration("xtm60_left_bind_ip")
    xtm60_right_bind_ip = LaunchConfiguration("xtm60_right_bind_ip")
    xtm60_bind_port = LaunchConfiguration("xtm60_bind_port")
    xtm60_phase_realign_interval_sec = LaunchConfiguration(
        "xtm60_phase_realign_interval_sec"
    )
    xtm60_quality_temporal_hard_reject = LaunchConfiguration(
        "xtm60_quality_temporal_hard_reject"
    )
    xtm60_timestamp_source = LaunchConfiguration("xtm60_timestamp_source")
    xtm60_use_sdk_timestamps = LaunchConfiguration("xtm60_use_sdk_timestamps")
    imu_use_device_timestamp = LaunchConfiguration("imu_use_device_timestamp")
    imu_require_device_timestamp = LaunchConfiguration(
        "imu_require_device_timestamp"
    )
    ultrasonic_config = LaunchConfiguration("ultrasonic_config")
    camera_config = LaunchConfiguration("camera_config")

    bringup_share = FindPackageShare("wheelchair_bringup")
    bringup_prefix = FindPackagePrefix("wheelchair_bringup")
    xt_bindshim = PathJoinSubstitution(
        [bringup_prefix, "lib", "wheelchair_bringup", "libxt_bindshim.so"]
    )
    description_share = FindPackageShare("wheelchair_description")
    include_mock_right_tf = PythonExpression(
        [
            "'",
            mode,
            "' == 'mock' and ('",
            enable_xtm60,
            "' == 'true' or '",
            enable_xtm60_right,
            "' == 'true')",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    PathJoinSubstitution(
                        [description_share, "urdf", "wheelchair.urdf.xacro"]
                    ),
                    " include_blocked_right_lidar_for_mock:=",
                    include_mock_right_tf,
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
            DeclareLaunchArgument(
                "enable_xtm60",
                default_value="false",
                description="Legacy single XT-M60 route. Disabled by default; select an explicit side.",
            ),
            DeclareLaunchArgument("enable_xtm60_left", default_value="false"),
            DeclareLaunchArgument(
                "enable_xtm60_right",
                default_value="false",
                description="Explicit right XT-M60 route used by the current mapping baseline.",
            ),
            DeclareLaunchArgument(
                "calibration_contract",
                default_value="",
                description="Approved calibration contract required for an explicit real right-lidar route.",
            ),
            DeclareLaunchArgument(
                "allow_approved_right_lidar",
                default_value="false",
                choices=["true", "false"],
                description="Safety gate: permit right lidar only when calibration_contract is APPROVED and finite.",
            ),
            DeclareLaunchArgument("xtm60_left_bind_ip", default_value="192.168.0.100"),
            DeclareLaunchArgument("xtm60_right_bind_ip", default_value="192.168.1.100"),
            DeclareLaunchArgument("xtm60_bind_port", default_value="7687"),
            DeclareLaunchArgument(
                "xtm60_phase_realign_interval_sec",
                default_value="0.0",
                description=(
                    "Single-radar default is 0 (no periodic stop/start). "
                    "A coordinated dual-radar profile may explicitly set 30.0."
                ),
            ),
            DeclareLaunchArgument(
                "xtm60_quality_temporal_hard_reject",
                default_value="true",
                description=(
                    "Hard-reject same-ray temporal jumps only for stationary "
                    "sensor qualification. Driving routes set false."
                ),
            ),
            DeclareLaunchArgument(
                "xtm60_timestamp_source",
                default_value="host_receive",
                choices=["host_receive", "sdk_epoch"],
                description=(
                    "Explicit cloud timestamp domain. sdk_epoch blocks output "
                    "unless the SDK stamp is already epoch-shaped."
                ),
            ),
            DeclareLaunchArgument(
                "xtm60_use_sdk_timestamps",
                default_value="false",
                choices=["true", "false"],
                description="Legacy alias; true selects the same strict sdk_epoch path.",
            ),
            DeclareLaunchArgument(
                "imu_use_device_timestamp",
                default_value="false",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "imu_require_device_timestamp",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "When true, H30 samples without the timestamp TLV are not published."
                ),
            ),
            DeclareLaunchArgument("enable_imu", default_value="true"),
            DeclareLaunchArgument("enable_ultrasonic", default_value="true"),
            DeclareLaunchArgument("enable_camera", default_value="true"),
            DeclareLaunchArgument(
                "split_camera_processes",
                default_value="false",
                description=(
                    "Run one isolated adapter process per camera. Use with "
                    "camera_quad.yaml so one damaged stream cannot stall the others."
                ),
            ),
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
            # Central fail-closed gate. It executes after launch arguments are
            # declared and before any robot_state_publisher or sensor process.
            OpaqueFunction(function=_block_unapproved_right_lidar),
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
                    {
                        "mode": mode,
                        "timestamp_source": xtm60_timestamp_source,
                        "use_sdk_timestamps": ParameterValue(
                            xtm60_use_sdk_timestamps, value_type=bool
                        ),
                    },
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
                    {
                        "mode": mode,
                        "phase_realign_interval_sec": ParameterValue(
                            xtm60_phase_realign_interval_sec, value_type=float
                        ),
                        "quality_temporal_hard_reject": ParameterValue(
                            xtm60_quality_temporal_hard_reject, value_type=bool
                        ),
                        "timestamp_source": xtm60_timestamp_source,
                        "use_sdk_timestamps": ParameterValue(
                            xtm60_use_sdk_timestamps, value_type=bool
                        ),
                    },
                ],
                remappings=[
                    ("/xtm60/points", "/xtm60/left/points"),
                    ("/xtm60/points_rejected", "/xtm60/left/points_rejected"),
                    ("/xtm60/quality", "/xtm60/left/quality"),
                    ("/xtm60/status", "/xtm60/left/status"),
                    ("/xtm60/timing", "/xtm60/left/timing"),
                    ("/xtm60/phase", "/xtm60/left/phase"),
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
                    {
                        "mode": mode,
                        "phase_realign_interval_sec": ParameterValue(
                            xtm60_phase_realign_interval_sec, value_type=float
                        ),
                        "quality_temporal_hard_reject": ParameterValue(
                            xtm60_quality_temporal_hard_reject, value_type=bool
                        ),
                        "timestamp_source": xtm60_timestamp_source,
                        "use_sdk_timestamps": ParameterValue(
                            xtm60_use_sdk_timestamps, value_type=bool
                        ),
                    },
                ],
                remappings=[
                    ("/xtm60/points", "/xtm60/right/points"),
                    ("/xtm60/points_rejected", "/xtm60/right/points_rejected"),
                    ("/xtm60/quality", "/xtm60/right/quality"),
                    ("/xtm60/status", "/xtm60/right/status"),
                    ("/xtm60/timing", "/xtm60/right/timing"),
                    ("/xtm60/phase", "/xtm60/right/phase"),
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
                    {
                        "mode": mode,
                        "use_device_timestamp": ParameterValue(
                            imu_use_device_timestamp, value_type=bool
                        ),
                        "require_device_timestamp": ParameterValue(
                            imu_require_device_timestamp, value_type=bool
                        ),
                    },
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
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'", enable_camera, "' == 'true' and '",
                            split_camera_processes, "' != 'true'",
                        ]
                    )
                ),
            ),
            *[
                Node(
                    package="wheelchair_sensors",
                    executable="camera_adapter_node",
                    namespace=f"camera_{role}_worker",
                    name="camera_adapter_node",
                    output="screen",
                    parameters=[
                        camera_config,
                        {"mode": mode, "enabled_cameras": [role]},
                    ],
                    condition=IfCondition(
                        PythonExpression(
                            [
                                "'", enable_camera, "' == 'true' and '",
                                split_camera_processes, "' == 'true'",
                            ]
                        )
                    ),
                )
                for role in ("front", "left", "right", "rear")
            ],
        ]
    )
