"""Fail-closed RTAB-Map entry point for a formal 3D product.

This launch file is intentionally separate from the experimental/general
``rtabmap_3d_mapping.launch.py`` entry point.  It does not manufacture a
calibration or time-synchronisation result.  Before any node is constructed it
requires an APPROVED calibration contract, both lidar inputs, no single-lidar
fallback, an explicit external odometry topic, and a real hardware profile.
The map-product exporter is configured to require the optimized backend cloud
and a matching intensity field.  ``hardware_validated`` remains false by
default; it can only be enabled together with an evidence file and the final
bundle must still pass ``validate_formal_3d_map``.

Sensors are not started by default.  Set ``bringup_sensors:=true`` only after
the separately authorized hardware preflight has passed.  This avoids turning
a launch-file test into an accidental radar or motor operation.
"""

import json
import math
import os
from pathlib import Path

from wheelchair_3d_mapping.formal_lio_contract import validate_hardware_identity

try:
    import yaml
except ImportError:  # pragma: no cover - launch hosts provide python3-yaml
    yaml = None


def _require_ros_launch_api():
    """Load ROS 2 launch dependencies only when a launch is actually built.

    The contract validator below is intentionally usable by offline CI and
    review tools on hosts that do not have a ROS installation.  Keeping the
    imports lazy preserves normal ``ros2 launch`` behaviour while making a
    missing ROS environment an explicit error only when the launch graph is
    requested.
    """

    try:
        from ament_index_python.packages import get_package_share_directory
        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        from launch.substitutions import LaunchConfiguration
        from launch_ros.actions import Node
        from launch_ros.substitutions import FindPackageShare
    except ImportError as exc:  # pragma: no cover - exercised on ROS hosts
        raise RuntimeError(
            "formal_3d_mapping requires ROS 2 launch and ament_index_python; "
            "source the ROS environment before building the launch graph"
        ) from exc
    return {
        "get_package_share_directory": get_package_share_directory,
        "LaunchDescription": LaunchDescription,
        "DeclareLaunchArgument": DeclareLaunchArgument,
        "IncludeLaunchDescription": IncludeLaunchDescription,
        "OpaqueFunction": OpaqueFunction,
        "PythonLaunchDescriptionSource": PythonLaunchDescriptionSource,
        "LaunchConfiguration": LaunchConfiguration,
        "Node": Node,
        "FindPackageShare": FindPackageShare,
    }


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in ("true", "1", "yes", "on")


def _finite_transform(contract: dict) -> bool:
    transform = contract.get("runtime_transform")
    if not isinstance(transform, dict):
        return False
    translation = transform.get("translation", transform.get("xyz"))
    rotation = transform.get("rotation", transform.get("rpy"))
    if not isinstance(translation, (list, tuple)) or not isinstance(rotation, (list, tuple)):
        return False
    values = list(translation) + list(rotation)
    return len(translation) == 3 and len(rotation) == 3 and all(
        isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))
        for value in values
    )


_APPROVED_TIME_SOURCES = {
    "device_timestamp",
    "device_time",
    "hardware_trigger",
    "ptp",
    "pps",
    "shared_hardware_clock",
}


def _validate_real_dual_profile(profile_path: Path) -> dict:
    """Reject mock, partial, and host-clock profiles before formal mapping."""

    if yaml is None:
        raise RuntimeError("FORMAL_MAP_BLOCKED: PyYAML is required to validate hardware_profile")
    try:
        profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise RuntimeError(f"FORMAL_MAP_BLOCKED: hardware profile is unreadable: {exc}") from exc
    if not isinstance(profile, dict):
        raise RuntimeError("FORMAL_MAP_BLOCKED: hardware profile must be a YAML mapping")
    root = profile.get("profile")
    if not isinstance(root, dict) or str(root.get("mode", "")).strip().lower() != "real":
        raise RuntimeError("FORMAL_MAP_BLOCKED: hardware profile mode must be real")
    if root.get("simulation_only") is True:
        raise RuntimeError("FORMAL_MAP_BLOCKED: hardware profile is simulation_only")
    installation_epoch = root.get("installation_epoch")
    if not isinstance(installation_epoch, str) or not installation_epoch.strip():
        raise RuntimeError(
            "FORMAL_MAP_BLOCKED: hardware profile installation_epoch is required"
        )
    installation_epoch = installation_epoch.strip()
    root["installation_epoch"] = installation_epoch
    try:
        hardware_identity = validate_hardware_identity(
            profile.get("hardware_identity"),
            installation_epoch=installation_epoch,
            label="hardware profile hardware_identity",
        )
    except ValueError as exc:
        raise RuntimeError(f"FORMAL_MAP_BLOCKED: {exc}") from exc

    ips = []
    for side in ("left", "right"):
        lidar = profile.get(f"lidar_{side}")
        if not isinstance(lidar, dict):
            raise RuntimeError(f"FORMAL_MAP_BLOCKED: hardware profile lacks lidar_{side}")
        if str(lidar.get("model", "")).strip().upper() != "XT-M60":
            raise RuntimeError(f"FORMAL_MAP_BLOCKED: lidar_{side}.model must be XT-M60")
        ip = str(lidar.get("ip_address", "")).strip()
        if not ip or ip.lower() in {"null", "none"}:
            raise RuntimeError(f"FORMAL_MAP_BLOCKED: lidar_{side}.ip_address is missing")
        ips.append(ip)
        if str(lidar.get("intensity_field", "")).strip() != "intensity":
            raise RuntimeError(f"FORMAL_MAP_BLOCKED: lidar_{side}.intensity_field must be intensity")
        if str(lidar.get("frame_id", "")).strip() != f"xtm60_{side}_link":
            raise RuntimeError(
                f"FORMAL_MAP_BLOCKED: lidar_{side}.frame_id must be xtm60_{side}_link"
            )
        if str(lidar.get("point_unit", "")).strip().lower() != "m":
            raise RuntimeError(f"FORMAL_MAP_BLOCKED: lidar_{side}.point_unit must be m")
        rate = lidar.get("scan_rate_hz")
        if (
            not isinstance(rate, (int, float))
            or isinstance(rate, bool)
            or not math.isfinite(float(rate))
            or float(rate) <= 0.0
        ):
            raise RuntimeError(f"FORMAL_MAP_BLOCKED: lidar_{side}.scan_rate_hz must be positive")
        source = str(lidar.get("timestamp_source", "")).strip().lower()
        if source not in _APPROVED_TIME_SOURCES:
            raise RuntimeError(
                f"FORMAL_MAP_BLOCKED: lidar_{side}.timestamp_source must be an approved device clock"
            )
        identity = hardware_identity["assets"][f"xtm60_{side}"]
        if identity["ip_address"] != ip or identity["frame_id"] != lidar["frame_id"]:
            raise RuntimeError(
                f"FORMAL_MAP_BLOCKED: lidar_{side} metadata does not match hardware_identity"
            )
    if ips[0] == ips[1]:
        raise RuntimeError("FORMAL_MAP_BLOCKED: left and right lidar IP addresses must differ")
    dual = profile.get("dual_lidar")
    if not isinstance(dual, dict) or str(dual.get("integration_mode", "")).strip().lower() != "map_only":
        raise RuntimeError("FORMAL_MAP_BLOCKED: dual_lidar.integration_mode must be map_only")
    pair_limit_ms = dual.get("max_pair_time_difference_ms")
    if (
        not isinstance(pair_limit_ms, (int, float))
        or isinstance(pair_limit_ms, bool)
        or not math.isfinite(float(pair_limit_ms))
        or float(pair_limit_ms) < 0.0
        or float(pair_limit_ms) > 20.0
    ):
        raise RuntimeError(
            "FORMAL_MAP_BLOCKED: dual_lidar.max_pair_time_difference_ms must be within 0..20"
        )
    imu = profile.get("imu")
    if not isinstance(imu, dict) or str(imu.get("model", "")).strip().upper() != "H30":
        raise RuntimeError("FORMAL_MAP_BLOCKED: hardware profile lacks H30 IMU")
    if str(imu.get("frame_id", "")).strip() != "imu_link":
        raise RuntimeError("FORMAL_MAP_BLOCKED: imu.frame_id must be imu_link")
    if str(imu.get("timestamp_source", "")).strip().lower() not in _APPROVED_TIME_SOURCES:
        raise RuntimeError("FORMAL_MAP_BLOCKED: imu.timestamp_source must be an approved device clock")
    imu_rate = imu.get("rate_hz")
    if (
        not isinstance(imu_rate, (int, float))
        or isinstance(imu_rate, bool)
        or not math.isfinite(float(imu_rate))
        or float(imu_rate) <= 0.0
    ):
        raise RuntimeError("FORMAL_MAP_BLOCKED: imu.rate_hz must be positive")
    imu_identity = hardware_identity["assets"]["imu"]
    if imu_identity["frame_id"] != imu["frame_id"]:
        raise RuntimeError(
            "FORMAL_MAP_BLOCKED: imu metadata does not match hardware_identity"
        )
    profile["hardware_identity"] = hardware_identity
    return profile


def _validate_hardware_evidence(evidence: dict, hardware_identity: dict) -> None:
    """Require a PASS gate that identifies each asset observed in this epoch."""

    gates = evidence.get("gates")
    hardware_gate = gates.get("hardware_validation") if isinstance(gates, dict) else None
    if not isinstance(hardware_gate, dict):
        raise RuntimeError(
            "FORMAL_MAP_BLOCKED: hardware_validation evidence must be a mapping"
        )
    hardware_status = hardware_gate.get("status")
    if hardware_status is not True and str(hardware_status).strip().upper() not in {
        "PASS",
        "PASSED",
        "APPROVED",
        "VALIDATED",
        "TRUE",
    }:
        raise RuntimeError(
            "FORMAL_MAP_BLOCKED: hardware_validated requires PASS hardware_validation evidence"
        )
    if hardware_gate.get("installation_epoch") != hardware_identity["installation_epoch"]:
        raise RuntimeError(
            "FORMAL_MAP_BLOCKED: hardware evidence installation_epoch does not match profile"
        )
    observed = hardware_gate.get("observed_assets")
    expected_assets = hardware_identity["assets"]
    if not isinstance(observed, dict) or set(observed) != set(expected_assets):
        raise RuntimeError(
            "FORMAL_MAP_BLOCKED: hardware evidence observed_assets must contain "
            "exactly xtm60_left, xtm60_right, and imu"
        )
    for role, expected in expected_assets.items():
        item = observed.get(role)
        if not isinstance(item, dict):
            raise RuntimeError(
                f"FORMAL_MAP_BLOCKED: observed hardware identity for {role} is missing"
            )
        source = item.get("identity_source")
        if not isinstance(source, str) or not source.strip():
            raise RuntimeError(
                f"FORMAL_MAP_BLOCKED: observed {role} identity_source is required"
            )
        if item.get("match") is not True:
            raise RuntimeError(
                f"FORMAL_MAP_BLOCKED: observed {role} identity is not confirmed"
            )
        if item.get("hardware_id") != expected["hardware_id"]:
            raise RuntimeError(
                f"FORMAL_MAP_BLOCKED: observed {role} hardware_id does not match profile"
            )
        for observed_key, expected_key in (
            ("observed_model", "model"),
            ("observed_identity_kind", "identity_kind"),
            ("observed_frame_id", "frame_id"),
        ):
            if item.get(observed_key) != expected[expected_key]:
                raise RuntimeError(
                    f"FORMAL_MAP_BLOCKED: observed {role} {observed_key} "
                    "does not match profile"
                )
        if "ip_address" in expected and item.get("observed_ip") != expected["ip_address"]:
            raise RuntimeError(
                f"FORMAL_MAP_BLOCKED: observed {role} IP does not match profile"
            )


def _validate_direct_sensor_clock_profile(profile: dict) -> None:
    """Ensure the built-in adapters can implement the declared clock source.

    External sensor stacks may legitimately supply PTP/PPS/hardware-trigger
    stamps.  The built-in XT-M60/H30 adapters have only their explicit device
    timestamp paths, so a self-contained formal bringup must not accept a
    profile it cannot actually enact.
    """

    for name in ("lidar_left", "lidar_right", "imu"):
        source = str(profile.get(name, {}).get("timestamp_source", "")).lower()
        if source not in {"device_timestamp", "device_time"}:
            raise RuntimeError(
                "FORMAL_MAP_BLOCKED: bringup_sensors=true requires direct "
                f"device timestamps for {name}; profile declares {source or '<missing>'}"
            )


def _validate_tf_edge_chain(
    global_edge: str, local_edge: str, body_bridge_edge: str
) -> dict:
    values = {
        "global_correction": global_edge,
        "local_odometry": local_edge,
        "body_bridge": body_bridge_edge,
    }
    parsed = {}
    normalized = {}
    for role, raw_value in values.items():
        value = str(raw_value).strip()
        if role == "body_bridge" and not value:
            continue
        parts = value.split("->")
        if (
            len(parts) != 2
            or any(not item or item.startswith("/") or item.strip() != item for item in parts)
            or parts[0] == parts[1]
        ):
            raise RuntimeError(f"FORMAL_MAP_BLOCKED: invalid {role} TF edge")
        normalized[role] = value
        parsed[role] = tuple(parts)
    if "global_correction" not in parsed or "local_odometry" not in parsed:
        raise RuntimeError("FORMAL_MAP_BLOCKED: global/local TF edges are required")
    if parsed["global_correction"][1] != parsed["local_odometry"][0]:
        raise RuntimeError("FORMAL_MAP_BLOCKED: global/local TF edges are disconnected")
    if "body_bridge" in parsed:
        if (
            parsed["local_odometry"][1] != parsed["body_bridge"][0]
            or parsed["body_bridge"][1] != "base_link"
        ):
            raise RuntimeError(
                "FORMAL_MAP_BLOCKED: local/body TF chain must end at base_link"
            )
    elif parsed["local_odometry"][1] != "base_link":
        raise RuntimeError(
            "FORMAL_MAP_BLOCKED: local TF must end at base_link without a bridge"
        )
    return normalized


def _validate_formal_output_paths(
    database_path: str,
    output_root: str,
    map_name: str,
    *,
    delete_db_on_start: bool,
) -> tuple[str, str, str]:
    """Require a new database and a writable, explicit formal output root."""

    if delete_db_on_start:
        raise RuntimeError(
            "FORMAL_MAP_BLOCKED: formal sessions never delete/replace an existing database"
        )
    raw_database = str(database_path).strip()
    raw_output = str(output_root).strip()
    normalized_name = str(map_name).strip()
    if not raw_database:
        raise RuntimeError(
            "FORMAL_MAP_BLOCKED: an explicit unique database_path is required"
        )
    if not raw_output:
        raise RuntimeError("FORMAL_MAP_BLOCKED: output_root is required")
    database = Path(raw_database).expanduser()
    output = Path(raw_output).expanduser()
    if not database.is_absolute() or not output.is_absolute():
        raise RuntimeError(
            "FORMAL_MAP_BLOCKED: database_path and output_root must be absolute"
        )
    database = database.resolve()
    output = output.resolve()
    if database.exists():
        raise RuntimeError(
            f"FORMAL_MAP_BLOCKED: database_path already exists: {database}"
        )
    if output.exists() and not output.is_dir():
        raise RuntimeError(
            f"FORMAL_MAP_BLOCKED: output_root is not a directory: {output}"
        )

    def writable_ancestor(path: Path) -> bool:
        candidate = path
        while not candidate.exists() and candidate != candidate.parent:
            candidate = candidate.parent
        return candidate.is_dir() and os.access(candidate, os.W_OK)

    if not writable_ancestor(database.parent):
        raise RuntimeError(
            f"FORMAL_MAP_BLOCKED: database parent is not writable: {database.parent}"
        )
    if not writable_ancestor(output):
        raise RuntimeError(
            f"FORMAL_MAP_BLOCKED: output_root has no writable ancestor: {output}"
        )
    if (
        not normalized_name
        or normalized_name in {".", ".."}
        or Path(normalized_name).name != normalized_name
        or "/" in normalized_name
        or "\\" in normalized_name
    ):
        raise RuntimeError("FORMAL_MAP_BLOCKED: map_name must be one safe path component")
    return str(database), str(output), normalized_name


def _validate_dual_runtime_transforms(contract: dict) -> dict:
    """Require independently recorded finite transforms for both lidars and IMU."""

    transforms = contract.get("runtime_transforms")
    if not isinstance(transforms, dict):
        raise RuntimeError("FORMAL_MAP_BLOCKED: approved contract lacks runtime_transforms")
    normalized = {}
    for name, child in (
        ("xtm60_left", "xtm60_left_link"),
        ("xtm60_right", "xtm60_right_link"),
        ("imu", "imu_link"),
    ):
        item = transforms.get(name)
        if not isinstance(item, dict) or not _finite_transform(item):
            raise RuntimeError(f"FORMAL_MAP_BLOCKED: runtime_transforms.{name} must be finite")
        frames = item.get("frames")
        if not isinstance(frames, dict) or frames.get("parent") != "base_link" or frames.get("child") != child:
            raise RuntimeError(
                f"FORMAL_MAP_BLOCKED: runtime_transforms.{name} must declare base_link->{child}"
            )
        transform = item["runtime_transform"]
        normalized[name] = {
            "parent": "base_link",
            "child": child,
            "translation": [float(value) for value in transform.get("translation", transform.get("xyz"))],
            "rotation": [float(value) for value in transform.get("rotation", transform.get("rpy"))],
        }
    return normalized


def _validate_formal_launch_contract(
    *,
    calibration_contract: str,
    hardware_profile: str,
    points_topic: str,
    left_points_topic: str,
    right_points_topic: str,
    fusion_status_topic: str,
    odom_topic: str,
    odom_mode: str,
    enable_left: bool,
    enable_right: bool,
    enable_imu: bool,
    allow_single_lidar_fallback: bool,
    enable_loop_closure: bool,
    max_pair_time_difference_sec: float,
    hardware_validated: bool,
    evidence_path: str,
    tf_global_edge: str,
    tf_local_edge: str,
    tf_body_bridge_edge: str,
) -> dict:
    """Validate launch inputs without touching ROS or hardware."""

    if not calibration_contract:
        raise RuntimeError("FORMAL_MAP_BLOCKED: calibration_contract is required")
    contract_path = Path(calibration_contract).expanduser().resolve()
    if not contract_path.is_file():
        raise RuntimeError(f"FORMAL_MAP_BLOCKED: calibration contract does not exist: {contract_path}")
    try:
        contract = json.loads(contract_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"FORMAL_MAP_BLOCKED: calibration contract is unreadable: {exc}") from exc
    if not isinstance(contract, dict) or str(contract.get("status", "")).upper() != "APPROVED":
        status = contract.get("status", "<missing>") if isinstance(contract, dict) else "<invalid>"
        raise RuntimeError(f"FORMAL_MAP_BLOCKED: calibration contract status is {status}")
    if contract.get("schema_version") != 1:
        raise RuntimeError("FORMAL_MAP_BLOCKED: calibration contract schema_version must be 1")
    if str(contract.get("scope", "")).strip().lower() != "dual_lidar_imu":
        raise RuntimeError("FORMAL_MAP_BLOCKED: calibration contract scope must be dual_lidar_imu")
    installation_epoch = contract.get("installation_epoch")
    if not isinstance(installation_epoch, str) or not installation_epoch.strip():
        raise RuntimeError("FORMAL_MAP_BLOCKED: calibration contract installation_epoch is required")
    installation_epoch = installation_epoch.strip()
    try:
        contract_hardware_identity = validate_hardware_identity(
            contract.get("hardware_identity"),
            installation_epoch=installation_epoch,
            label="calibration contract hardware_identity",
        )
    except ValueError as exc:
        raise RuntimeError(f"FORMAL_MAP_BLOCKED: {exc}") from exc
    runtime_transforms = _validate_dual_runtime_transforms(contract)
    if not hardware_profile:
        raise RuntimeError("FORMAL_MAP_BLOCKED: hardware_profile is required")
    profile_path = Path(hardware_profile).expanduser().resolve()
    if not profile_path.is_file():
        raise RuntimeError(f"FORMAL_MAP_BLOCKED: hardware profile does not exist: {profile_path}")
    profile = _validate_real_dual_profile(profile_path)
    if installation_epoch != profile["profile"]["installation_epoch"]:
        raise RuntimeError(
            "FORMAL_MAP_BLOCKED: calibration contract and hardware profile "
            "installation_epoch differ"
        )
    if contract_hardware_identity != profile["hardware_identity"]:
        raise RuntimeError(
            "FORMAL_MAP_BLOCKED: calibration contract and hardware profile "
            "hardware_identity differ"
        )
    for name, value in (
        ("points_topic", points_topic),
        ("left_points_topic", left_points_topic),
        ("right_points_topic", right_points_topic),
        ("fusion_status_topic", fusion_status_topic),
        ("odom_topic", odom_topic),
    ):
        if not value or not value.startswith("/"):
            raise RuntimeError(f"FORMAL_MAP_BLOCKED: {name} must be an absolute ROS topic")
    if odom_mode not in {"external", "contract_fastlio"}:
        raise RuntimeError(
            "FORMAL_MAP_BLOCKED: odom_mode must be external or contract_fastlio"
        )
    if odom_mode == "external" and odom_topic == "/rtabmap/odom":
        raise RuntimeError(
            "FORMAL_MAP_BLOCKED: formal mapping requires odom_mode=external and a non-default external odom_topic"
        )
    if odom_mode == "contract_fastlio" and odom_topic != "/Odometry":
        raise RuntimeError(
            "FORMAL_MAP_BLOCKED: contract_fastlio publishes the fixed /Odometry topic"
        )
    if not enable_left or not enable_right:
        raise RuntimeError("FORMAL_MAP_BLOCKED: both left and right lidar inputs are required")
    if not enable_imu:
        raise RuntimeError("FORMAL_MAP_BLOCKED: H30 IMU input is required")
    if allow_single_lidar_fallback:
        raise RuntimeError("FORMAL_MAP_BLOCKED: single-lidar fallback must be false")
    if not math.isfinite(max_pair_time_difference_sec) or not 0.0 <= max_pair_time_difference_sec <= 0.020:
        raise RuntimeError(
            "FORMAL_MAP_BLOCKED: max_pair_time_difference_sec must be within 0..0.020"
        )
    profile_pair_limit = float(profile["dual_lidar"]["max_pair_time_difference_ms"]) / 1000.0
    if max_pair_time_difference_sec > profile_pair_limit + 1.0e-12:
        raise RuntimeError(
            "FORMAL_MAP_BLOCKED: runtime pair limit cannot exceed the hardware profile limit"
        )
    if not enable_loop_closure:
        raise RuntimeError("FORMAL_MAP_BLOCKED: loop closure must be explicitly enabled for a formal product")
    if hardware_validated:
        if not evidence_path:
            raise RuntimeError("FORMAL_MAP_BLOCKED: hardware_validated=true requires evidence_path")
        evidence_file = Path(evidence_path).expanduser().resolve()
        if not evidence_file.is_file():
            raise RuntimeError("FORMAL_MAP_BLOCKED: hardware_validated evidence file does not exist")
        try:
            evidence = json.loads(evidence_file.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"FORMAL_MAP_BLOCKED: hardware_validated evidence is unreadable: {exc}") from exc
        if not isinstance(evidence, dict) or evidence.get("schema_version") != 1:
            raise RuntimeError(
                "FORMAL_MAP_BLOCKED: hardware_validated evidence must be a schema_version=1 JSON object"
            )
        _validate_hardware_evidence(evidence, contract_hardware_identity)
    tf_edges = _validate_tf_edge_chain(
        tf_global_edge, tf_local_edge, tf_body_bridge_edge
    )
    expected_tf_edges = {
        "global_correction": "map->camera_init",
        "local_odometry": "camera_init->body",
        "body_bridge": "body->base_link",
    }
    if tf_edges != expected_tf_edges:
        raise RuntimeError(
            "FORMAL_MAP_BLOCKED: formal mapping requires the fixed "
            "map->camera_init->body->base_link TF chain"
        )
    return {
        "contract_path": str(contract_path),
        "profile_path": str(profile_path),
        "points_topic": points_topic,
        "left_points_topic": left_points_topic,
        "right_points_topic": right_points_topic,
        "odom_topic": odom_topic,
        "odom_mode": odom_mode,
        "runtime_transforms": runtime_transforms,
        "hardware_identity": contract_hardware_identity,
        "hardware_profile_data": profile,
        "tf_edges": tf_edges,
    }


def _resolve_motion_fixed_frame(odom_mode: str, requested: str, local_edge: str) -> str:
    """Never compensate through a loop-corrected map or moving sensor frame."""
    local_parent = local_edge.split("->", 1)[0].strip()
    if odom_mode == "contract_fastlio":
        if requested and requested != "camera_init":
            raise RuntimeError("FORMAL_MAP_BLOCKED: contract_fastlio motion_fixed_frame must be camera_init")
        return "camera_init"
    if not requested or requested in {"map", "base_link", "body"} or requested != local_parent:
        raise RuntimeError(
            "FORMAL_MAP_BLOCKED: external odometry requires an explicit continuous motion_fixed_frame matching tf_local_edge parent"
        )
    return requested


def _setup(context, *args, **kwargs):
    ros = _require_ros_launch_api()
    get_package_share_directory = ros["get_package_share_directory"]
    IncludeLaunchDescription = ros["IncludeLaunchDescription"]
    PythonLaunchDescriptionSource = ros["PythonLaunchDescriptionSource"]
    LaunchConfiguration = ros["LaunchConfiguration"]
    Node = ros["Node"]
    mapping_share = get_package_share_directory("wheelchair_3d_mapping")
    bringup_share = get_package_share_directory("wheelchair_bringup")

    def value(name: str) -> str:
        return LaunchConfiguration(name).perform(context).strip()

    def flag(name: str) -> bool:
        return _as_bool(value(name))

    contract = value("calibration_contract")
    profile = value("hardware_profile")
    points = value("points_topic")
    odom = value("odom_topic")
    odom_mode = value("odom_mode").lower()
    motion_fixed_frame = _resolve_motion_fixed_frame(
        odom_mode, value("motion_fixed_frame"), value("tf_local_edge")
    )
    evidence = value("evidence_path")
    hardware_validated = flag("hardware_validated")
    validated = _validate_formal_launch_contract(
        calibration_contract=contract,
        hardware_profile=profile,
        points_topic=points,
        left_points_topic=value("left_points_topic"),
        right_points_topic=value("right_points_topic"),
        fusion_status_topic=value("fusion_status_topic"),
        odom_topic=odom,
        odom_mode=odom_mode,
        enable_left=flag("enable_xtm60_left"),
        enable_right=flag("enable_xtm60_right"),
        enable_imu=flag("enable_imu"),
        allow_single_lidar_fallback=flag("allow_single_lidar_fallback"),
        enable_loop_closure=flag("enable_loop_closure"),
        max_pair_time_difference_sec=float(value("max_pair_time_difference_sec")),
        hardware_validated=hardware_validated,
        evidence_path=evidence,
        tf_global_edge=value("tf_global_edge"),
        tf_local_edge=value("tf_local_edge"),
        tf_body_bridge_edge=value("tf_body_bridge_edge"),
    )
    database, output_root, map_name = _validate_formal_output_paths(
        value("database_path"),
        value("output_root"),
        value("map_name"),
        delete_db_on_start=flag("delete_db_on_start"),
    )
    actions = []

    # The APPROVED contract is the runtime TF authority for a formal session.
    # Publishing these edges here prevents provisional URDF values from being
    # silently used after the JSON gate passes.
    for name, transform in validated["runtime_transforms"].items():
        xyz = transform["translation"]
        rpy = transform["rotation"]
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"formal_contract_tf_{name}",
                output="screen",
                arguments=[
                    "--x", str(xyz[0]), "--y", str(xyz[1]), "--z", str(xyz[2]),
                    "--roll", str(rpy[0]), "--pitch", str(rpy[1]), "--yaw", str(rpy[2]),
                    "--frame-id", transform["parent"],
                    "--child-frame-id", transform["child"],
                ],
            )
        )

    if flag("bringup_sensors"):
        _validate_direct_sensor_clock_profile(validated["hardware_profile_data"])
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "sensors.launch.py")),
                launch_arguments={
                    "mode": "real",
                    # Formal contract TF publishers above own the three sensor
                    # edges; never start the provisional URDF publisher here.
                    "publish_description": "false",
                    "calibration_contract": contract,
                    "allow_approved_right_lidar": "true",
                    "enable_xtm60": "false",
                    "enable_xtm60_left": "true",
                    "enable_xtm60_right": "true",
                    "enable_imu": value("enable_imu"),
                    "enable_ultrasonic": "false",
                    "enable_camera": "false",
                    # These overrides enact the clock source already validated
                    # in the formal hardware profile. Neither adapter is
                    # allowed to fall back to host receive/publish time.
                    "xtm60_timestamp_source": "sdk_epoch",
                    "xtm60_use_sdk_timestamps": "false",
                    "imu_use_device_timestamp": "true",
                    "imu_require_device_timestamp": "true",
                }.items(),
            )
        )

    if odom_mode == "contract_fastlio":
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mapping_share, "launch", "formal_fast_lio.launch.py")
                ),
                launch_arguments={
                    "calibration_contract": contract,
                    "radar": value("formal_lio_radar"),
                    "input_topic": value("formal_lio_input_topic"),
                    "imu_topic": value("formal_lio_imu_topic"),
                }.items(),
            )
        )

    # Own a dedicated strict fusion publisher even when raw sensors were
    # started externally.  The formal backend therefore never has to trust a
    # generic /points_merged producer with unknown fallback/timestamp policy.
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mapping_share, "launch", "dual_lidar_fusion.launch.py")),
            launch_arguments={
                "use_sim_time": value("use_sim_time"),
                "left_points_topic": value("left_points_topic"),
                "right_points_topic": value("right_points_topic"),
                "output_topic": points,
                "status_topic": value("fusion_status_topic"),
                "target_frame": "base_link",
                "allow_single_lidar_fallback": "false",
                "enable_left_input": "true",
                "enable_right_input": "true",
                "require_synchronized_pair": "true",
                "max_pair_time_difference_sec": value("max_pair_time_difference_sec"),
                "require_intensity": "true",
                "require_nonzero_timestamps": "true",
                "motion_compensation": "true",
                "motion_fixed_frame": motion_fixed_frame,
            }.items(),
        )
    )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mapping_share, "launch", "rtabmap_3d_mapping.launch.py")),
            launch_arguments={
                "use_sim_time": value("use_sim_time"),
                "points_topic": points,
                "odom_topic": odom,
                "odom_mode": "external",
                "frame_id": "base_link",
                "enable_xtm60_left": "true",
                "enable_xtm60_right": "true",
                "allow_single_lidar_fallback": "false",
                "enable_loop_closure": "true",
                "bringup_sensors": "false",
                "subscribe_scan_cloud": "true",
                "subscribe_rgb": "false",
                "use_colorizer": "false",
                "database_path": database,
                "delete_db_on_start": value("delete_db_on_start"),
                "localization": "false",
                "rviz": value("rviz"),
            }.items(),
        )
    )
    actions.append(
        Node(
            package="smartwheel_global_mapping",
            executable="rtabmap_optimized_cloud_node",
            name="formal_rtabmap_optimized_cloud",
            output="screen",
            parameters=[
                {
                    "service_name": "/rtabmap/get_map_data",
                    "output_topic": "/rtabmap/optimized_cloud",
                    "path_output_topic": "/rtabmap/optimized_path",
                    "request_interval_sec": 2.0,
                    "voxel_size_m": 0.05,
                    "minimum_nodes": 2,
                }
            ],
        )
    )
    actions.append(
        Node(
            package="smartwheel_map_products",
            executable="map_products_node",
            name="formal_map_products",
            output="screen",
            parameters=[
                {
                    "map_name": map_name,
                    "output_root": output_root,
                    "hardware_profile_path": profile,
                    "calibration_contract_path": contract,
                    "mapping_backend": "rtabmap",
                    "state_mode": "lio_primary",
                    "lidar_mode": "dual_map_only",
                    "bag_path": value("bag_path"),
                    "cloud_topic": points,
                    "odom_topic": odom,
                    "cloud_frame_mode": "base_frame",
                    "expected_cloud_frame": "base_link",
                    "backend_cloud_topic": "/rtabmap/optimized_cloud",
                    "require_backend_cloud": True,
                    "backend_path_topic": "/rtabmap/optimized_path",
                    "require_backend_trajectory": True,
                    "require_fresh_backend_after_stop": True,
                    "require_intensity": True,
                    "require_session_stop": True,
                    "validation_stage": "H3_FORMAL_CANDIDATE",
                    "hardware_validated": hardware_validated,
                    # A formal product must never inherit map_products_node's
                    # simulation-safe default.  Keep this explicit even when
                    # hardware_validated is false (candidate runs still need
                    # a truthful provenance profile).
                    "mock_lio": False,
                    "formal_acceptance_evidence_path": evidence,
                    "tf_global_edge": validated["tf_edges"]["global_correction"],
                    "tf_local_edge": validated["tf_edges"]["local_odometry"],
                    "tf_body_bridge_edge": validated["tf_edges"].get(
                        "body_bridge", ""
                    ),
                    "enable_offline_colorization": False,
                }
            ],
        )
    )
    return actions


def generate_launch_description():
    ros = _require_ros_launch_api()
    FindPackageShare = ros["FindPackageShare"]
    get_package_share_directory = ros["get_package_share_directory"]
    LaunchDescription = ros["LaunchDescription"]
    DeclareLaunchArgument = ros["DeclareLaunchArgument"]
    OpaqueFunction = ros["OpaqueFunction"]
    mapping = FindPackageShare("wheelchair_3d_mapping")
    default_contract = Path(get_package_share_directory("wheelchair_bringup")) / "config" / "right_lidar_stage1_calibration_contract.json"
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("calibration_contract", default_value=str(default_contract)),
            DeclareLaunchArgument("hardware_profile", default_value=""),
            DeclareLaunchArgument("points_topic", default_value="/formal/points_merged"),
            DeclareLaunchArgument("left_points_topic", default_value="/xtm60/left/points"),
            DeclareLaunchArgument("right_points_topic", default_value="/xtm60/right/points"),
            DeclareLaunchArgument("fusion_status_topic", default_value="/formal/points_merged/status"),
            DeclareLaunchArgument("odom_topic", default_value="/Odometry"),
            DeclareLaunchArgument(
                "odom_mode",
                default_value="contract_fastlio",
                choices=["external", "contract_fastlio"],
            ),
            DeclareLaunchArgument(
                "formal_lio_radar", default_value="right", choices=["left", "right"]
            ),
            DeclareLaunchArgument("formal_lio_input_topic", default_value=""),
            DeclareLaunchArgument("formal_lio_imu_topic", default_value="/imu/data"),
            DeclareLaunchArgument(
                "motion_fixed_frame", default_value="",
                description="Continuous TF frame for two-time cloud compensation; camera_init for contract_fastlio, required explicitly for external odometry.",
            ),
            DeclareLaunchArgument("tf_global_edge", default_value="map->camera_init"),
            DeclareLaunchArgument("tf_local_edge", default_value="camera_init->body"),
            DeclareLaunchArgument("tf_body_bridge_edge", default_value="body->base_link"),
            DeclareLaunchArgument("enable_xtm60_left", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("enable_xtm60_right", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("enable_imu", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("allow_single_lidar_fallback", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("enable_loop_closure", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument(
                "max_pair_time_difference_sec",
                default_value="0.020",
                description="Maximum left/right acquisition-time separation for strict dual fusion.",
            ),
            DeclareLaunchArgument("hardware_validated", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("evidence_path", default_value=""),
            DeclareLaunchArgument("bag_path", default_value=""),
            DeclareLaunchArgument("map_name", default_value="formal_3d_map"),
            DeclareLaunchArgument("output_root", default_value=os.path.expanduser("~/maps/formal")),
            DeclareLaunchArgument("database_path", default_value=""),
            DeclareLaunchArgument("delete_db_on_start", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("rviz", default_value="false", choices=["true", "false"]),
            OpaqueFunction(function=_setup),
        ]
    )
