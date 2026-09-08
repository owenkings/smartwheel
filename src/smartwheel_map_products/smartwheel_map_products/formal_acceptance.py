"""Strict, fail-closed acceptance checks for a formal 3D map bundle.

The ordinary map-product quality gate answers a deliberately smaller question:
did the exporter write a coherent set of files?  That is useful for synthetic
replays and engineering experiments, but it must not be confused with a map
that is suitable for deployment.  This module adds the second, evidence-based
gate.  It never infers calibration, clock synchronisation, dynamic behaviour,
or real-time performance from point counts alone.

Every deployment gate must be backed by an explicit ``PASS``/``APPROVED``
entry in ``formal_acceptance.json`` (or in the ``formal_acceptance`` member of
``quality_report.json``).  Missing evidence is therefore a failure, rather
than an implicit pass.  The module has no ROS dependency so it can be used on
an exported bundle on a development machine or in CI.
"""

from __future__ import annotations

import csv
import hashlib
import ipaddress
import json
import math
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:  # pragma: no cover - the ROS image provides python3-yaml
    yaml = None


SCHEMA_VERSION = 1
DEFAULT_MAX_TIME_OFFSET_SEC = 0.020
DEFAULT_MAX_OUTPUT_GAP_SEC = 0.500
DEFAULT_MAX_DROP_RATE = 0.010
DEFAULT_MIN_REALTIME_DURATION_SEC = 60.0
DEFAULT_MAX_REJECTED_FRAME_RATE = 0.0

_PASS_STATUSES = {
    "APPROVED",
    "PASS",
    "PASSED",
    "VALIDATED",
    "TRUE",
}

_GATE_ALIASES = {
    "timestamp_sync": ("timestamp_sync", "time_sync", "timestamp"),
    "extrinsics": ("extrinsics", "extrinsic", "calibration"),
    "dynamic_validation": ("dynamic_validation", "dynamic"),
    "realtime": ("realtime", "real_time", "real-time"),
    "tf_ownership": ("tf_ownership", "tf", "transform_ownership"),
    "loop_closure": ("loop_closure", "loop"),
    "repeatability": ("repeatability", "repeated_runs"),
    "dual_lidar": ("dual_lidar", "dual_sensor", "lidar_pair"),
    "hardware_validation": ("hardware_validation", "hardware_validated"),
}

_REQUIRED_FILES = (
    "map_geometry.pcd",
    "map_geometry.ply",
    "trajectory.tum",
    "poses.csv",
    "quality_report.json",
    "algorithm_profile_used.yaml",
    "hardware_profile_used.yaml",
    "calibration_contract_used.json",
    "formal_acceptance.json",
    "manifest.json",
)

_FORBIDDEN_TIME_SOURCES = {
    "",
    "unknown",
    "host_receive",
    "host_receive_time",
    "host_interpolated",
    "host_interpolated_time",
    "ros_now",
    "wall_clock",
}
_APPROVED_TIME_SOURCES = {
    "device_timestamp",
    "device_time",
    "hardware_trigger",
    "ptp",
    "pps",
    "shared_hardware_clock",
}

_EXPECTED_HARDWARE_ASSETS = {
    "xtm60_left": {
        "model": "XT-M60",
        "identity_kind": "vendor_serial",
        "frame_id": "xtm60_left_link",
        "requires_ip": True,
    },
    "xtm60_right": {
        "model": "XT-M60",
        "identity_kind": "vendor_serial",
        "frame_id": "xtm60_right_link",
        "requires_ip": True,
    },
    "imu": {
        "model": "H30",
        "identity_kind": "installation_asset_id",
        "frame_id": "imu_link",
        "requires_ip": False,
    },
}


def _finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _load_json(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        return None, f"cannot read {path.name}: {exc}"
    if not isinstance(value, dict):
        return None, f"{path.name} must contain a JSON object"
    return value, None


def _load_yaml(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    """Load a YAML object without making the validator depend on ROS."""

    if yaml is None:
        return None, "PyYAML is unavailable; cannot validate profile provenance"
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError, yaml.YAMLError) as exc:
        return None, f"cannot read {path.name}: {exc}"
    if not isinstance(value, dict):
        return None, f"{path.name} must contain a YAML mapping"
    return value, None


def _required_identity_text(value: Any, *, label: str) -> str:
    if not isinstance(value, str) or value.strip().lower() in {"", "none", "null"}:
        raise ValueError(f"{label} is required")
    return value.strip()


def _normalize_hardware_identity(
    value: Any,
    *,
    installation_epoch: str,
    label: str,
) -> dict[str, Any]:
    expected_epoch = _required_identity_text(
        installation_epoch, label=f"{label} expected installation_epoch"
    )
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    if value.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"{label}.schema_version must be 1")
    identity_epoch = _required_identity_text(
        value.get("installation_epoch"), label=f"{label}.installation_epoch"
    )
    if identity_epoch != expected_epoch:
        raise ValueError(
            f"{label}.installation_epoch does not match the enclosing document"
        )
    assets = value.get("assets")
    if not isinstance(assets, dict) or set(assets) != set(_EXPECTED_HARDWARE_ASSETS):
        raise ValueError(
            f"{label}.assets must contain exactly xtm60_left, xtm60_right, and imu"
        )

    normalized: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "installation_epoch": identity_epoch,
        "assets": {},
    }
    identity_owners: dict[str, str] = {}
    lidar_ip_owners: dict[str, str] = {}
    for role, expected in _EXPECTED_HARDWARE_ASSETS.items():
        item = assets.get(role)
        if not isinstance(item, dict):
            raise ValueError(f"{label}.assets.{role} must be a mapping")
        if _required_identity_text(
            item.get("role"), label=f"{label}.assets.{role}.role"
        ) != role:
            raise ValueError(f"{label}.assets.{role}.role must be {role}")
        model = _required_identity_text(
            item.get("model"), label=f"{label}.assets.{role}.model"
        )
        if model.upper() != expected["model"].upper():
            raise ValueError(
                f"{label}.assets.{role}.model must be {expected['model']}"
            )
        identity_kind = _required_identity_text(
            item.get("identity_kind"),
            label=f"{label}.assets.{role}.identity_kind",
        )
        if identity_kind != expected["identity_kind"]:
            raise ValueError(
                f"{label}.assets.{role}.identity_kind must be "
                f"{expected['identity_kind']}"
            )
        hardware_id = _required_identity_text(
            item.get("hardware_id"), label=f"{label}.assets.{role}.hardware_id"
        )
        previous_role = identity_owners.get(hardware_id)
        if previous_role is not None:
            raise ValueError(
                f"{label} hardware_id is duplicated by {previous_role} and {role}"
            )
        identity_owners[hardware_id] = role
        frame_id = _required_identity_text(
            item.get("frame_id"), label=f"{label}.assets.{role}.frame_id"
        )
        if frame_id != expected["frame_id"]:
            raise ValueError(
                f"{label}.assets.{role}.frame_id must be {expected['frame_id']}"
            )
        normalized_item = {
            "role": role,
            "model": expected["model"],
            "hardware_id": hardware_id,
            "identity_kind": expected["identity_kind"],
            "frame_id": expected["frame_id"],
        }
        if expected["requires_ip"]:
            raw_ip = _required_identity_text(
                item.get("ip_address"), label=f"{label}.assets.{role}.ip_address"
            )
            try:
                parsed_ip = ipaddress.ip_address(raw_ip)
            except ValueError as exc:
                raise ValueError(
                    f"{label}.assets.{role}.ip_address is invalid"
                ) from exc
            if parsed_ip.version != 4:
                raise ValueError(f"{label}.assets.{role}.ip_address must be IPv4")
            canonical_ip = str(parsed_ip)
            previous_role = lidar_ip_owners.get(canonical_ip)
            if previous_role is not None:
                raise ValueError(
                    f"{label} LiDAR IP is duplicated by {previous_role} and {role}"
                )
            lidar_ip_owners[canonical_ip] = role
            normalized_item["ip_address"] = canonical_ip
        normalized["assets"][role] = normalized_item
    return normalized


def _hardware_identity_check(
    value: Any,
    *,
    installation_epoch: str,
    label: str,
) -> tuple[bool, str, dict[str, Any]]:
    try:
        normalized = _normalize_hardware_identity(
            value,
            installation_epoch=installation_epoch,
            label=label,
        )
    except ValueError as exc:
        return False, str(exc), {}
    return True, "canonical hardware identity binding is valid", normalized


def _path_from_evidence(value: Any, *, bundle: Path) -> Path | None:
    """Resolve evidence paths relative to the bundle when appropriate."""

    if not isinstance(value, str) or not value.strip():
        return None
    candidate = Path(value).expanduser()
    if not candidate.is_absolute():
        candidate = bundle / candidate
    return candidate.resolve()


def _profile_check(path: Path) -> tuple[bool, str, dict[str, Any]]:
    """Validate the minimum real dual-XT-M60 profile contract.

    A file merely existing is not evidence that it describes the hardware
    used for a map.  In particular, the repository's mock and left-only draft
    profiles must never be accepted as a formal product profile.
    """

    profile, error = _load_yaml(path)
    if profile is None:
        return False, error or "hardware profile is invalid", {}
    root = profile.get("profile")
    if not isinstance(root, dict):
        return False, "hardware profile has no profile mapping", profile
    if str(root.get("mode", "")).strip().lower() != "real":
        return False, "hardware profile mode is not real", profile
    if root.get("simulation_only") is True:
        return False, "hardware profile is marked simulation_only", profile
    installation_epoch = root.get("installation_epoch")
    if not isinstance(installation_epoch, str) or not installation_epoch.strip():
        return False, "hardware profile installation_epoch is missing", profile
    installation_epoch = installation_epoch.strip()
    root["installation_epoch"] = installation_epoch
    identity_ok, identity_reason, hardware_identity = _hardware_identity_check(
        profile.get("hardware_identity"),
        installation_epoch=installation_epoch,
        label="hardware profile hardware_identity",
    )
    if not identity_ok:
        return False, identity_reason, profile

    addresses: list[str] = []
    for side in ("left", "right"):
        item = profile.get(f"lidar_{side}")
        if not isinstance(item, dict):
            return False, f"hardware profile lacks lidar_{side}", profile
        if str(item.get("model", "")).strip().upper() != "XT-M60":
            return False, f"lidar_{side}.model is not XT-M60", profile
        ip = str(item.get("ip_address", "")).strip()
        if not ip or ip.lower() in {"null", "none"}:
            return False, f"lidar_{side}.ip_address is missing", profile
        addresses.append(ip)
        frame = str(item.get("frame_id", "")).strip()
        if not frame or frame.startswith("/"):
            return False, f"lidar_{side}.frame_id is invalid", profile
        bound_asset = hardware_identity["assets"][f"xtm60_{side}"]
        if bound_asset["ip_address"] != ip or bound_asset["frame_id"] != frame:
            return False, f"lidar_{side} metadata does not match hardware_identity", profile
        if str(item.get("point_unit", "")).strip().lower() != "m":
            return False, f"lidar_{side}.point_unit must be m", profile
        if str(item.get("intensity_field", "")).strip() != "intensity":
            return False, f"lidar_{side}.intensity_field must be intensity", profile
        timestamp = str(item.get("timestamp_source", "")).strip().lower()
        if timestamp in _FORBIDDEN_TIME_SOURCES or timestamp not in _APPROVED_TIME_SOURCES:
            return False, f"lidar_{side}.timestamp_source is not an approved device clock", profile
        rate = item.get("scan_rate_hz")
        if not _finite_number(rate) or float(rate) <= 0.0:
            return False, f"lidar_{side}.scan_rate_hz is invalid", profile

    if addresses[0] == addresses[1]:
        return False, "left and right lidar IP addresses are identical", profile
    dual = profile.get("dual_lidar")
    if not isinstance(dual, dict) or str(dual.get("integration_mode", "")).strip().lower() != "map_only":
        return False, "dual_lidar.integration_mode must be map_only", profile
    pair_limit = dual.get("max_pair_time_difference_ms")
    if not _finite_number(pair_limit) or float(pair_limit) < 0.0:
        return False, "dual_lidar.max_pair_time_difference_ms is invalid", profile

    imu = profile.get("imu")
    if not isinstance(imu, dict) or str(imu.get("model", "")).strip().upper() != "H30":
        return False, "hardware profile lacks a real H30 IMU", profile
    imu_timestamp = str(imu.get("timestamp_source", "")).strip().lower()
    if imu_timestamp in _FORBIDDEN_TIME_SOURCES or imu_timestamp not in _APPROVED_TIME_SOURCES:
        return False, "imu.timestamp_source is not an approved device clock", profile
    imu_rate = imu.get("rate_hz")
    if not _finite_number(imu_rate) or float(imu_rate) <= 0.0:
        return False, "imu.rate_hz is invalid", profile
    imu_frame = str(imu.get("frame_id", "")).strip()
    if not imu_frame or imu_frame.startswith("/"):
        return False, "imu.frame_id is invalid", profile
    if hardware_identity["assets"]["imu"]["frame_id"] != imu_frame:
        return False, "imu metadata does not match hardware_identity", profile
    profile["hardware_identity"] = hardware_identity
    return True, (
        "real dual-XT-M60/H30 profile, hardware identity, and approved "
        "device-clock metadata"
    ), profile


def _algorithm_profile_check(path: Path) -> tuple[bool, str, dict[str, Any]]:
    profile, error = _load_yaml(path)
    if profile is None:
        return False, error or "algorithm profile is invalid", {}
    backend = str(profile.get("mapping_backend", "")).strip().lower()
    if backend != "rtabmap":
        return False, "algorithm profile mapping_backend is not rtabmap", profile
    if profile.get("require_backend_cloud") is not True:
        return False, "algorithm profile does not require the optimized backend cloud", profile
    if profile.get("require_intensity") is not True:
        return False, "algorithm profile does not require intensity preservation", profile
    if profile.get("mock_lio") is True:
        return False, "algorithm profile is marked mock_lio", profile
    backend_topic = str(profile.get("backend_cloud_topic", "")).strip()
    if not backend_topic.startswith("/"):
        return False, "algorithm profile backend_cloud_topic is not absolute", profile
    if profile.get("require_backend_trajectory") is not True:
        return False, "algorithm profile does not require the optimized backend trajectory", profile
    backend_path_topic = str(profile.get("backend_path_topic", "")).strip()
    if not backend_path_topic.startswith("/"):
        return False, "algorithm profile backend_path_topic is not absolute", profile
    if profile.get("require_fresh_backend_after_stop") is not True:
        return False, "algorithm profile does not require a post-stop backend snapshot", profile
    backend_lag = profile.get("backend_max_trajectory_lag_sec")
    if (
        not _finite_number(backend_lag)
        or float(backend_lag) < 0.0
        or float(backend_lag) > 2.5
    ):
        return False, "algorithm profile backend trajectory lag limit is invalid", profile
    if str(profile.get("trajectory_source", "")) != f"backend:{backend_path_topic}":
        return False, "algorithm profile trajectory_source does not match backend_path_topic", profile
    if str(profile.get("trajectory_representation", "")) != "full_6dof_quaternion":
        return False, "algorithm profile trajectory is not full_6dof_quaternion", profile
    tf_edges = profile.get("tf_edges")
    if not isinstance(tf_edges, dict):
        return False, "algorithm profile lacks the explicit TF ownership chain", profile
    parsed = {}
    for role in ("global_correction", "local_odometry", "body_bridge"):
        value = str(tf_edges.get(role, "")).strip()
        if role == "body_bridge" and not value:
            continue
        parts = value.split("->")
        if (
            len(parts) != 2
            or any(not item or item.startswith("/") or item.strip() != item for item in parts)
            or parts[0] == parts[1]
        ):
            return False, f"algorithm profile has an invalid {role} TF edge", profile
        parsed[role] = tuple(parts)
    if "global_correction" not in parsed or "local_odometry" not in parsed:
        return False, "algorithm profile lacks global/local TF edges", profile
    if parsed["global_correction"][1] != parsed["local_odometry"][0]:
        return False, "algorithm profile TF chain is disconnected", profile
    if "body_bridge" in parsed:
        if (
            parsed["local_odometry"][1] != parsed["body_bridge"][0]
            or parsed["body_bridge"][1] != "base_link"
        ):
            return False, "algorithm profile TF bridge does not end at base_link", profile
    elif parsed["local_odometry"][1] != "base_link":
        return False, "algorithm profile local TF does not end at base_link", profile
    canonical_tf_edges = {
        "global_correction": "map->camera_init",
        "local_odometry": "camera_init->body",
        "body_bridge": "body->base_link",
    }
    if tf_edges != canonical_tf_edges:
        return False, "algorithm profile does not use the canonical formal TF chain", profile
    return True, "RTAB-Map backend geometry/trajectory/intensity requirements are recorded", profile


def _numeric_point_rows(
    path: Path,
    *,
    marker: str,
    columns: list[str],
    expected_count: int,
    require_intensity: bool,
) -> tuple[bool, str]:
    """Stream-validate ASCII point rows without loading a full map in RAM."""

    try:
        xyz_indices = [columns.index(name) for name in ("x", "y", "z")]
        intensity_index = columns.index("intensity") if require_intensity else None
    except ValueError as exc:
        return False, f"{path.name} point columns are incomplete: {exc}"
    rows = 0
    nonzero_xyz = False
    positive_intensity = False
    started = False
    try:
        with path.open("r", encoding="ascii", errors="strict") as stream:
            for line_number, line in enumerate(stream, start=1):
                stripped = line.strip()
                if not started:
                    if stripped.lower() == marker:
                        started = True
                    continue
                if not stripped or stripped.startswith("#"):
                    continue
                tokens = stripped.split()
                if len(tokens) != len(columns):
                    return False, (
                        f"{path.name}:{line_number} has {len(tokens)} values, "
                        f"expected {len(columns)}"
                    )
                try:
                    values = [float(token) for token in tokens]
                except ValueError:
                    return False, f"{path.name}:{line_number} contains a non-numeric value"
                if not all(math.isfinite(value) for value in values):
                    return False, f"{path.name}:{line_number} contains nan/inf"
                xyz = [values[index] for index in xyz_indices]
                nonzero_xyz = nonzero_xyz or any(value != 0.0 for value in xyz)
                if intensity_index is not None:
                    amplitude = values[intensity_index]
                    if amplitude < 0.0:
                        return False, f"{path.name}:{line_number} contains negative intensity"
                    positive_intensity = positive_intensity or amplitude > 0.0
                rows += 1
    except (OSError, UnicodeError) as exc:
        return False, f"cannot read {path.name} data: {exc}"
    if not started:
        return False, f"{path.name} has no complete data marker"
    if rows != expected_count:
        return False, f"{path.name} data rows ({rows}) do not match header count ({expected_count})"
    if not nonzero_xyz:
        return False, f"{path.name} geometry is entirely at the origin"
    if require_intensity and not positive_intensity:
        return False, f"{path.name} intensity is entirely zero"
    return True, "numeric finite point rows are complete"


def _point_file_header(path: Path, *, require_intensity: bool) -> tuple[bool, str, int | None]:
    """Check the structural header of an ASCII PCD/PLY product."""

    header_terminated = False
    try:
        with path.open("r", encoding="ascii", errors="strict") as stream:
            lines = []
            for _ in range(80):
                line = stream.readline()
                if not line:
                    break
                lines.append(line.strip())
                if line.strip().lower() == "end_header":
                    header_terminated = True
                    break
    except (OSError, UnicodeError) as exc:
        return False, f"cannot read {path.name} header: {exc}", None
    if path.suffix.lower() == ".pcd":
        fields_line = next((line for line in lines if line.upper().startswith("FIELDS ")), "")
        points_line = next((line for line in lines if line.upper().startswith("POINTS ")), "")
        data_line = next((line for line in lines if line.upper().startswith("DATA ")), "")
        fields = fields_line.split()[1:]
        count_line = next((line for line in lines if line.upper().startswith("COUNT ")), "")
        try:
            count = int(points_line.split()[1])
        except (IndexError, ValueError):
            return False, f"{path.name} has no valid PCD POINTS header", None
        if not {"x", "y", "z"}.issubset(fields):
            return False, f"{path.name} lacks x/y/z fields", None
        if require_intensity and "intensity" not in fields:
            return False, f"{path.name} lacks intensity field", None
        if data_line.upper() != "DATA ASCII":
            return False, f"{path.name} is not an ASCII PCD product", None
        if count <= 0:
            return False, f"{path.name} has no points", count
        try:
            counts = [int(value) for value in count_line.split()[1:]]
        except ValueError:
            return False, f"{path.name} has an invalid COUNT header", None
        if len(counts) != len(fields) or any(value != 1 for value in counts):
            return False, f"{path.name} requires one scalar per formal PCD field", None
        rows_ok, rows_reason = _numeric_point_rows(
            path,
            marker="data ascii",
            columns=fields,
            expected_count=count,
            require_intensity=require_intensity,
        )
        if not rows_ok:
            return False, rows_reason, count
        return True, "valid PCD header and numeric complete ASCII rows", count
    if not header_terminated:
        return False, f"{path.name} has no complete header terminator", None
    element = next((line for line in lines if line.lower().startswith("element vertex ")), "")
    property_names = [
        line.split()[-1]
        for line in lines
        if line.lower().startswith("property ") and len(line.split()) == 3
    ]
    properties = set(property_names)
    try:
        count = int(element.split()[2])
    except (IndexError, ValueError):
        return False, f"{path.name} has no valid PLY vertex header", None
    if not {"x", "y", "z"}.issubset(properties):
        return False, f"{path.name} lacks x/y/z properties", None
    if require_intensity and "intensity" not in properties:
        return False, f"{path.name} lacks intensity property", None
    if not any(line.lower() == "format ascii 1.0" for line in lines):
        return False, f"{path.name} is not an ASCII PLY product", None
    if count <= 0:
        return False, f"{path.name} has no vertices", count
    rows_ok, rows_reason = _numeric_point_rows(
        path,
        marker="end_header",
        columns=property_names,
        expected_count=count,
        require_intensity=require_intensity,
    )
    if not rows_ok:
        return False, rows_reason, count
    return True, "valid PLY header and numeric complete ASCII rows", count


def _status_pass(value: Any) -> bool:
    if value is True:
        return True
    if isinstance(value, str):
        return value.strip().upper() in _PASS_STATUSES
    return False


def _gate_value(evidence: dict[str, Any], name: str) -> Any:
    for alias in _GATE_ALIASES.get(name, (name,)):
        if alias in evidence:
            return evidence[alias]
    gates = evidence.get("gates")
    if isinstance(gates, dict):
        for alias in _GATE_ALIASES.get(name, (name,)):
            if alias in gates:
                return gates[alias]
    return None


def _gate_status(evidence: dict[str, Any], name: str) -> tuple[bool, dict[str, Any]]:
    value = _gate_value(evidence, name)
    if isinstance(value, dict):
        return _status_pass(value.get("status")), value
    return _status_pass(value), {}


def _observed_hardware_identity_check(
    gate: dict[str, Any],
    hardware_identity: Any,
) -> tuple[bool, str]:
    if not isinstance(hardware_identity, dict):
        return False, "hardware profile has no canonical hardware_identity"
    if gate.get("installation_epoch") != hardware_identity.get("installation_epoch"):
        return False, "hardware evidence installation_epoch does not match profile"
    expected_assets = hardware_identity.get("assets")
    observed_assets = gate.get("observed_assets")
    if (
        not isinstance(expected_assets, dict)
        or not isinstance(observed_assets, dict)
        or set(observed_assets) != set(expected_assets)
    ):
        return False, (
            "hardware evidence observed_assets must contain exactly "
            "xtm60_left, xtm60_right, and imu"
        )
    for role, expected in expected_assets.items():
        observed = observed_assets.get(role)
        if not isinstance(observed, dict):
            return False, f"observed hardware identity for {role} is missing"
        identity_source = observed.get("identity_source")
        if not isinstance(identity_source, str) or not identity_source.strip():
            return False, f"observed {role} identity_source is required"
        if observed.get("match") is not True:
            return False, f"observed {role} identity is not confirmed"
        if observed.get("hardware_id") != expected.get("hardware_id"):
            return False, f"observed {role} hardware_id does not match profile"
        for observed_key, expected_key in (
            ("observed_model", "model"),
            ("observed_identity_kind", "identity_kind"),
            ("observed_frame_id", "frame_id"),
        ):
            if observed.get(observed_key) != expected.get(expected_key):
                return False, (
                    f"observed {role} {observed_key} does not match profile"
                )
        if (
            "ip_address" in expected
            and observed.get("observed_ip") != expected.get("ip_address")
        ):
            return False, f"observed {role} IP does not match profile"
    return True, "all observed hardware identities match the bound profile"


def _check(passed: bool, reason: str, **details: Any) -> dict[str, Any]:
    result: dict[str, Any] = {"passed": bool(passed), "reason": reason}
    if details:
        result["details"] = details
    return result


def _manifest_check(output: Path, manifest: dict[str, Any]) -> tuple[bool, str]:
    if manifest.get("complete") is not True:
        return False, "manifest complete flag is not true"
    if (output / ".incomplete").exists():
        return False, "bundle still contains .incomplete marker"
    entries = manifest.get("files")
    if not isinstance(entries, list) or not entries:
        return False, "manifest.files is empty or invalid"
    listed: set[str] = set()
    for entry in entries:
        if not isinstance(entry, dict):
            return False, "manifest contains a non-object file entry"
        relative = entry.get("path")
        if not isinstance(relative, str):
            return False, "manifest file path is not a string"
        relative_path = Path(relative)
        if relative_path.is_absolute() or ".." in relative_path.parts:
            return False, f"manifest path escapes bundle: {relative}"
        normalized = relative_path.as_posix()
        if normalized in listed:
            return False, f"manifest contains duplicate file entry: {relative}"
        path = output / relative_path
        # Do not allow a manifest to hash a symlink that resolves outside the
        # bundle.  ``Path.is_file`` follows links, so a malicious or accidental
        # link could otherwise make an external file look like an immutable
        # product artifact.
        if path.is_symlink():
            return False, f"manifest path is a symlink: {relative}"
        try:
            path.resolve().relative_to(output.resolve())
        except ValueError:
            return False, f"manifest path escapes bundle: {relative}"
        try:
            expected_bytes = int(entry["bytes"])
            expected_sha = str(entry["sha256"])
        except (KeyError, TypeError, ValueError):
            return False, f"manifest entry for {relative} lacks bytes/sha256"
        if not path.is_file() or path.stat().st_size != expected_bytes:
            return False, f"manifest size check failed: {relative}"
        digest = hashlib.sha256()
        try:
            with path.open("rb") as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(chunk)
        except OSError as exc:
            return False, f"cannot hash {relative}: {exc}"
        if digest.hexdigest() != expected_sha:
            return False, f"manifest hash check failed: {relative}"
        listed.add(normalized)
    missing = [
        name
        for name in _REQUIRED_FILES
        if name != "manifest.json" and name not in listed
    ]
    if missing:
        return False, "required files are not covered by manifest: " + ", ".join(missing)
    # A formal bundle is immutable after export.  Permit only the database and
    # raw-bag payloads that the writer explicitly records as externally managed;
    # every other file must be hash-covered by the manifest.
    externally_managed = {
        str(item).replace("\\", "/")
        for item in manifest.get("externally_managed_files", [])
        if isinstance(item, str)
    }
    for path in output.rglob("*"):
        if not path.is_file():
            continue
        relative = path.relative_to(output).as_posix()
        if path.is_symlink():
            return False, f"formal bundle contains a symlink: {relative}"
        if relative in listed or relative == "manifest.json" or relative in externally_managed:
            continue
        if "raw_bag" in Path(relative).parts:
            continue
        return False, f"unlisted file is present in formal bundle: {relative}"
    return True, "all manifest hashes and required entries are valid"


def _contract_check(
    path: Path | None,
    *,
    hardware_profile: dict[str, Any],
) -> tuple[bool, str, dict[str, Any]]:
    if path is None:
        return False, "no calibration contract was supplied", {}
    contract, error = _load_json(path)
    if contract is None:
        return False, error or "invalid calibration contract", {}
    if str(contract.get("status", "")).upper() != "APPROVED":
        return False, f"calibration contract status is {contract.get('status', '<missing>')}", contract
    if contract.get("schema_version") != SCHEMA_VERSION:
        return False, "calibration contract schema_version must be 1", contract
    if str(contract.get("scope", "")).strip().lower() != "dual_lidar_imu":
        return False, "calibration contract scope must be dual_lidar_imu", contract
    epoch = contract.get("installation_epoch")
    if not isinstance(epoch, str) or not epoch.strip():
        return False, "calibration contract installation_epoch is missing", contract
    epoch = epoch.strip()
    profile_epoch = hardware_profile.get("profile", {}).get("installation_epoch")
    if epoch != profile_epoch:
        return False, "calibration contract installation_epoch does not match hardware profile", contract
    identity_ok, identity_reason, contract_identity = _hardware_identity_check(
        contract.get("hardware_identity"),
        installation_epoch=epoch,
        label="calibration contract hardware_identity",
    )
    if not identity_ok:
        return False, identity_reason, contract
    profile_identity = hardware_profile.get("hardware_identity")
    if contract_identity != profile_identity:
        return False, (
            "calibration contract hardware_identity does not match hardware profile"
        ), contract
    transforms = contract.get("runtime_transforms")
    if not isinstance(transforms, dict):
        return False, "approved contract lacks runtime_transforms", contract
    expected_children = {
        role: asset["frame_id"]
        for role, asset in contract_identity["assets"].items()
    }
    if any(not child for child in expected_children.values()):
        return False, "hardware profile frame IDs are incomplete", contract
    for name, child in expected_children.items():
        item = transforms.get(name)
        if not isinstance(item, dict):
            return False, f"runtime_transforms.{name} is missing", contract
        transform = item.get("runtime_transform")
        if not isinstance(transform, dict):
            return False, f"runtime_transforms.{name}.runtime_transform is missing", contract
        translation = transform.get("translation", transform.get("xyz"))
        rotation = transform.get("rotation", transform.get("rpy"))
        if not isinstance(translation, (list, tuple)) or not isinstance(rotation, (list, tuple)):
            return False, f"runtime_transforms.{name} lacks translation/rotation arrays", contract
        if len(translation) != 3 or len(rotation) != 3 or not all(
            _finite_number(value) for value in (*translation, *rotation)
        ):
            return False, f"runtime_transforms.{name} contains invalid values", contract
        frames = item.get("frames")
        if (
            not isinstance(frames, dict)
            or frames.get("parent") != "base_link"
            or frames.get("child") != child
            or frames.get("parent") == frames.get("child")
        ):
            return False, (
                f"runtime_transforms.{name} must bind base_link to profile frame {child}"
            ), contract
    return True, (
        "approved dual-LiDAR/H30 transforms and hardware identities match "
        "the hardware profile"
    ), contract


def validate_hardware_evidence_binding(
    evidence: object,
    *,
    hardware_profile_path: str | Path,
    calibration_contract_path: str | Path,
) -> tuple[bool, str]:
    """Validate the exact provenance required for a hardware-validated export.

    This is intentionally usable by both the ROS exporter and the bundle
    writer.  The exporter checks the live source files before capture/export;
    the writer checks the copies already placed in the bundle before it is
    allowed to publish a complete manifest.  Keeping the policy here prevents
    those two gates from drifting away from the final bundle validator.
    """

    if not isinstance(evidence, dict):
        return False, "formal hardware evidence is missing"
    if evidence.get("schema_version") != SCHEMA_VERSION:
        return False, "formal hardware evidence schema_version must be 1"

    profile_text = str(hardware_profile_path).strip()
    if not profile_text:
        return False, "hardware_profile_path is required"
    profile_path = Path(profile_text).expanduser().resolve()
    profile_ok, profile_reason, hardware_profile = _profile_check(profile_path)
    if not profile_ok:
        return False, f"hardware profile rejected: {profile_reason}"

    contract_text = str(calibration_contract_path).strip()
    if not contract_text:
        return False, "calibration_contract_path is required"
    contract_path = Path(contract_text).expanduser().resolve()
    contract_ok, contract_reason, _ = _contract_check(
        contract_path,
        hardware_profile=hardware_profile,
    )
    if not contract_ok:
        return False, f"calibration contract rejected: {contract_reason}"

    hardware_passed, hardware_gate = _gate_status(
        evidence, "hardware_validation"
    )
    if not hardware_passed:
        return False, "hardware_validation evidence is missing or not PASS"
    identity_ok, identity_reason = _observed_hardware_identity_check(
        hardware_gate,
        hardware_profile.get("hardware_identity"),
    )
    if not identity_ok:
        return False, identity_reason

    extrinsics_passed, extrinsics_gate = _gate_status(evidence, "extrinsics")
    if not extrinsics_passed:
        return False, "extrinsics evidence is missing or not PASS"
    if str(extrinsics_gate.get("contract_status", "")).strip().upper() != "APPROVED":
        return False, "extrinsics evidence contract_status is not APPROVED"
    try:
        contract_digest = hashlib.sha256(contract_path.read_bytes()).hexdigest()
    except OSError as exc:
        return False, f"cannot hash calibration contract: {exc}"
    if extrinsics_gate.get("contract_sha256") != contract_digest:
        return False, "extrinsics evidence contract_sha256 does not match contract"

    return True, (
        "hardware profile, calibration contract, observed identities, and "
        "contract digest are bound"
    )


def _trajectory_products_check(
    output: Path,
    expected_count: Any,
) -> tuple[bool, str, dict[str, Any]]:
    if not _finite_number(expected_count) or int(float(expected_count)) < 2:
        return False, "trajectory_pose_count is invalid", {}
    expected = int(float(expected_count))
    tum_path = output / "trajectory.tum"
    csv_path = output / "poses.csv"
    tum_rows = []
    try:
        with tum_path.open("r", encoding="ascii", errors="strict") as stream:
            for line_number, line in enumerate(stream, start=1):
                if not line.strip() or line.lstrip().startswith("#"):
                    continue
                tokens = line.split()
                if len(tokens) != 8:
                    return False, f"trajectory.tum:{line_number} must contain 8 values", {}
                values = [float(token) for token in tokens]
                if not all(math.isfinite(value) for value in values):
                    return False, f"trajectory.tum:{line_number} contains nan/inf", {}
                quaternion_norm = math.sqrt(sum(value * value for value in values[4:8]))
                if abs(quaternion_norm - 1.0) > 1.0e-5:
                    return False, f"trajectory.tum:{line_number} quaternion is not normalized", {}
                if tum_rows and values[0] <= tum_rows[-1][0]:
                    return False, "trajectory.tum timestamps are not strictly increasing", {}
                tum_rows.append(values)
    except (OSError, UnicodeError, ValueError) as exc:
        return False, f"trajectory.tum is invalid: {exc}", {}

    csv_rows = []
    try:
        with csv_path.open("r", encoding="ascii", errors="strict", newline="") as stream:
            reader = csv.DictReader(stream)
            required = (
                "timestamp", "x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad"
            )
            if reader.fieldnames != list(required):
                return False, "poses.csv header is invalid", {}
            for line_number, row in enumerate(reader, start=2):
                values = [float(row[name]) for name in required]
                if not all(math.isfinite(value) for value in values):
                    return False, f"poses.csv:{line_number} contains nan/inf", {}
                if csv_rows and values[0] <= csv_rows[-1][0]:
                    return False, "poses.csv timestamps are not strictly increasing", {}
                csv_rows.append(values)
    except (OSError, UnicodeError, TypeError, ValueError, KeyError) as exc:
        return False, f"poses.csv is invalid: {exc}", {}

    counts_ok = len(tum_rows) == expected and len(csv_rows) == expected
    if not counts_ok:
        return False, "trajectory file row counts do not match trajectory_pose_count", {
            "expected": expected,
            "tum_rows": len(tum_rows),
            "csv_rows": len(csv_rows),
        }
    for index, (tum, csv_values) in enumerate(zip(tum_rows, csv_rows), start=1):
        qx, qy, qz, qw = tum[4:8]
        roll = math.atan2(
            2.0 * (qw * qx + qy * qz),
            1.0 - 2.0 * (qx * qx + qy * qy),
        )
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        expected_csv = [tum[0], tum[1], tum[2], tum[3], roll, pitch, yaw]
        if any(abs(left - right) > 1.0e-6 for left, right in zip(expected_csv, csv_values)):
            return False, (
                "trajectory.tum and poses.csv disagree at pose "
                f"{index}"
            ), {}
    return True, "full finite 6-DoF trajectory files are consistent", {
        "poses": expected,
        "first_stamp": tum_rows[0][0],
        "last_stamp": tum_rows[-1][0],
    }


def _metric_number(item: dict[str, Any], *keys: str) -> float | None:
    for key in keys:
        value = item.get(key)
        if _finite_number(value):
            return float(value)
    return None


def _scenario_statuses(item: dict[str, Any]) -> dict[str, Any]:
    scenarios = item.get("scenarios")
    if isinstance(scenarios, dict):
        return scenarios
    if isinstance(scenarios, list):
        result = {}
        for entry in scenarios:
            if isinstance(entry, dict) and entry.get("name"):
                result[str(entry["name"])] = entry
        return result
    return {}


def template() -> dict[str, Any]:
    """Return a conservative evidence template for a future real run."""

    return {
        "schema_version": SCHEMA_VERSION,
        "evidence_id": "",
        "generated_at_utc": "",
        "source_artifacts": [],
        "operator_attestation": "UNVERIFIED",
        "gates": {
            "hardware_validation": {
                "status": "UNVERIFIED",
                "installation_epoch": "",
                "evidence": [],
                "observed_assets": {
                    "xtm60_left": {
                        "hardware_id": "",
                        "identity_source": "",
                        "observed_model": "",
                        "observed_identity_kind": "",
                        "observed_ip": "",
                        "observed_frame_id": "",
                        "match": False,
                    },
                    "xtm60_right": {
                        "hardware_id": "",
                        "identity_source": "",
                        "observed_model": "",
                        "observed_identity_kind": "",
                        "observed_ip": "",
                        "observed_frame_id": "",
                        "match": False,
                    },
                    "imu": {
                        "hardware_id": "",
                        "identity_source": "",
                        "observed_model": "",
                        "observed_identity_kind": "",
                        "observed_frame_id": "",
                        "match": False,
                    },
                },
            },
            "dual_lidar": {"status": "UNVERIFIED", "left": {}, "right": {}, "overlap": {}},
            "timestamp_sync": {
                "status": "UNVERIFIED",
                "source": "",
                "monotonic": False,
                "sensors": {
                    "xtm60_left": {"source": "", "monotonic": False},
                    "xtm60_right": {"source": "", "monotonic": False},
                    "imu": {"source": "", "monotonic": False},
                },
            },
            "extrinsics": {
                "status": "UNVERIFIED",
                "contract_status": "BLOCKED_CONFLICT",
                "contract_sha256": "",
            },
            "dynamic_validation": {"status": "UNVERIFIED", "scenarios": {}},
            "realtime": {"status": "UNVERIFIED", "max_output_gap_sec": None, "drop_rate": None},
            "tf_ownership": {
                "status": "UNVERIFIED",
                "edges": {
                    "global_correction": {"edge": "", "publishers": None},
                    "local_odometry": {"edge": "", "publishers": None},
                    "body_bridge": {"edge": "", "publishers": None},
                },
            },
            "loop_closure": {"status": "UNVERIFIED", "closures": 0},
            "repeatability": {"status": "UNVERIFIED", "runs": 0},
        },
    }


def validate_formal_3d_bundle(
    directory: str | Path,
    *,
    evidence_path: str | Path | None = None,
    calibration_contract_path: str | Path | None = None,
    require_dual_lidar: bool = True,
    require_loop_closure: bool = True,
    require_repeatability: bool = True,
    require_intensity: bool = True,
    require_occupancy: bool = False,
    max_time_offset_sec: float = DEFAULT_MAX_TIME_OFFSET_SEC,
    max_output_gap_sec: float = DEFAULT_MAX_OUTPUT_GAP_SEC,
    max_drop_rate: float = DEFAULT_MAX_DROP_RATE,
    min_realtime_duration_sec: float = DEFAULT_MIN_REALTIME_DURATION_SEC,
    max_rejected_frame_rate: float = DEFAULT_MAX_REJECTED_FRAME_RATE,
) -> dict[str, Any]:
    """Evaluate a bundle and return a JSON-serialisable acceptance report.

    The defaults intentionally describe a *formal* dual-LiDAR product.  The
    explicit ``require_*`` switches are useful for documented engineering
    diagnostics, but disabling any formal gate always keeps
    ``formal_ready=false`` so a reduced-scope report cannot be mistaken for a
    deployable product acceptance.
    """

    output = Path(directory).expanduser().resolve()
    if any(
        not _finite_number(value) or float(value) < 0.0
        for value in (
            max_time_offset_sec,
            max_output_gap_sec,
            max_drop_rate,
            min_realtime_duration_sec,
            max_rejected_frame_rate,
        )
    ):
        return {
            "schema_version": SCHEMA_VERSION,
            "formal_ready": False,
            "bundle": str(output),
            "checks": {"thresholds": _check(False, "acceptance thresholds must be finite and non-negative")},
            "blockers": ["acceptance thresholds must be finite and non-negative"],
        }
    checks: dict[str, dict[str, Any]] = {}
    blockers: list[str] = []

    degraded_modes = []
    if not require_dual_lidar:
        degraded_modes.append("single-lidar")
    if not require_loop_closure:
        degraded_modes.append("no-loop-closure")
    if not require_repeatability:
        degraded_modes.append("no-repeatability")
    if not require_intensity:
        degraded_modes.append("no-intensity")
    formal_scope_ok = not degraded_modes
    checks["formal_scope"] = _check(
        formal_scope_ok,
        (
            "all formal 3D product gates are enabled"
            if formal_scope_ok
            else "diagnostic invocation disabled formal gates: "
            + ", ".join(degraded_modes)
        ),
        degraded_modes=degraded_modes,
    )
    if not formal_scope_ok:
        blockers.append(checks["formal_scope"]["reason"])

    if not output.is_dir():
        return {
            "schema_version": SCHEMA_VERSION,
            "formal_ready": False,
            "bundle": str(output),
            "checks": {"bundle_directory": _check(False, "bundle directory does not exist")},
            "blockers": ["bundle directory does not exist"],
        }
    checks["bundle_directory"] = _check(True, "bundle directory exists")

    manifest, manifest_error = _load_json(output / "manifest.json")
    quality, quality_error = _load_json(output / "quality_report.json")
    if manifest is None:
        checks["bundle_integrity"] = _check(False, manifest_error or "manifest is unavailable")
        blockers.append(checks["bundle_integrity"]["reason"])
    else:
        ok, reason = _manifest_check(output, manifest)
        checks["bundle_integrity"] = _check(ok, reason)
        if not ok:
            blockers.append(reason)
    if quality is None:
        checks["quality_report"] = _check(False, quality_error or "quality report is unavailable")
        blockers.append(checks["quality_report"]["reason"])
        quality = {}
    else:
        checks["quality_report"] = _check(True, "quality report is valid JSON")

    missing = [name for name in _REQUIRED_FILES if not (output / name).is_file()]
    checks["required_products"] = _check(
        not missing,
        "all required geometry/provenance files exist" if not missing else "missing files: " + ", ".join(missing),
    )
    if missing:
        blockers.append(checks["required_products"]["reason"])

    profile_path = output / "hardware_profile_used.yaml"
    profile_ok, profile_reason, hardware_profile = _profile_check(profile_path)
    checks["hardware_profile"] = _check(profile_ok, profile_reason)
    if not profile_ok:
        blockers.append(profile_reason)
    algorithm_path = output / "algorithm_profile_used.yaml"
    algorithm_ok, algorithm_reason, algorithm_profile = _algorithm_profile_check(algorithm_path)
    checks["algorithm_profile"] = _check(algorithm_ok, algorithm_reason)
    if not algorithm_ok:
        blockers.append(algorithm_reason)

    point_count = quality.get("point_count")
    pose_count = quality.get("trajectory_pose_count")
    expected_trajectory_source = str(algorithm_profile.get("trajectory_source", ""))
    expected_trajectory_representation = str(
        algorithm_profile.get("trajectory_representation", "")
    )
    algorithm_provenance_ok = (
        quality.get("mock_lio") is False
        and quality.get("trajectory_source") == expected_trajectory_source
        and quality.get("trajectory_representation")
        == expected_trajectory_representation
        and quality.get("calibration_contract_bundled") is True
    )
    checks["algorithm_provenance"] = _check(
        algorithm_provenance_ok,
        (
            "quality report matches the non-mock backend trajectory and bundled contract"
            if algorithm_provenance_ok
            else "quality report does not match the required non-mock backend trajectory/contract"
        ),
        trajectory_source=quality.get("trajectory_source"),
        trajectory_representation=quality.get("trajectory_representation"),
        calibration_contract_bundled=quality.get("calibration_contract_bundled"),
    )
    if not algorithm_provenance_ok:
        blockers.append(checks["algorithm_provenance"]["reason"])

    trajectory_ok, trajectory_reason, trajectory_details = _trajectory_products_check(
        output, pose_count
    )
    checks["trajectory_products"] = _check(
        trajectory_ok, trajectory_reason, **trajectory_details
    )
    if not trajectory_ok:
        blockers.append(trajectory_reason)

    geometry_header_results = []
    for name in ("map_geometry.pcd", "map_geometry.ply"):
        path = output / name
        if path.is_file():
            ok, reason, count = _point_file_header(path, require_intensity=False)
            geometry_header_results.append((name, ok, reason, count))
        else:
            geometry_header_results.append((name, False, "file is missing", None))
    geometry_headers_ok = all(item[1] for item in geometry_header_results)
    geometry_header_counts = [item[3] for item in geometry_header_results if item[3] is not None]
    geometry_count_ok = (
        _finite_number(point_count)
        and len(geometry_header_counts) == 2
        and len(set(geometry_header_counts)) == 1
        and geometry_header_counts[0] == int(float(point_count))
    )
    checks["geometry_products"] = _check(
        geometry_headers_ok and geometry_count_ok,
        "geometry PCD/PLY headers are structurally valid" if geometry_headers_ok else "geometry PCD/PLY header validation failed",
        files={item[0]: {"passed": item[1], "reason": item[2], "points": item[3]} for item in geometry_header_results},
        point_count_matches_quality=geometry_count_ok,
    )
    if not checks["geometry_products"]["passed"]:
        blockers.append(checks["geometry_products"]["reason"])

    geometry_ok = (
        _finite_number(point_count)
        and float(point_count) > 0.0
        and _finite_number(pose_count)
        and float(pose_count) >= 2.0
    )
    checks["nonempty_geometry"] = _check(
        geometry_ok,
        "geometry and trajectory contain samples" if geometry_ok else "point_count/trajectory_pose_count are empty or invalid",
    )
    if not geometry_ok:
        blockers.append(checks["nonempty_geometry"]["reason"])

    geometry_source = str(quality.get("geometry_source", ""))
    expected_geometry_source = f"backend:{algorithm_profile.get('backend_cloud_topic', '')}"
    backend_geometry = (
        geometry_source == expected_geometry_source
        and str(quality.get("stage", "")).upper()
        not in {"A_SYNTHETIC", "SYNTHETIC"}
    )
    checks["backend_geometry"] = _check(
        backend_geometry,
        "geometry comes from the registered RTAB-Map/backend cloud" if backend_geometry else "geometry_source is not an accepted registered backend cloud",
        geometry_source=geometry_source,
        expected_geometry_source=expected_geometry_source,
    )
    if not backend_geometry:
        blockers.append(checks["backend_geometry"]["reason"])

    hardware_ok = quality.get("hardware_validated") is True
    checks["hardware_validation"] = _check(
        hardware_ok,
        "quality report explicitly marks hardware validation true" if hardware_ok else "quality report hardware_validated is not true",
    )
    if not hardware_ok:
        blockers.append(checks["hardware_validation"]["reason"])

    session = quality.get("session")
    session_ok = False
    session_details: dict[str, Any] = {}
    session_last_stamp: float | None = None
    if isinstance(session, dict):
        try:
            first = float(session["first_cloud_stamp"])
            last = float(session["last_cloud_stamp"])
            span = float(session["cloud_time_span_sec"])
            frame_count = int(session.get("frame_count", 0))
            raw_count = int(session.get("raw_point_count", 0))
            rejected = int(session.get("rejected_frame_count", 0))
            session_ok = (
                session.get("state") == "STOPPED"
                and session.get("complete") is True
                and not str(session.get("failure_reason", "")).strip()
                and frame_count > 0
                and raw_count > 0
                and _finite_number(first)
                and _finite_number(last)
                and _finite_number(span)
                and last >= first
                and abs(span - (last - first)) <= 1.0e-6
                and rejected >= 0
                and rejected <= max_rejected_frame_rate * max(frame_count, 1)
                and isinstance(session.get("frame_ids"), list)
                and len(session["frame_ids"]) == 1
            )
            session_details = {
                "frame_count": frame_count,
                "raw_point_count": raw_count,
                "rejected_frame_count": rejected,
                "cloud_time_span_sec": span,
                "max_rejected_frame_rate": max_rejected_frame_rate,
            }
            if session_ok:
                session_last_stamp = last
        except (KeyError, TypeError, ValueError, OverflowError):
            session_ok = False
    checks["session_integrity"] = _check(
        session_ok,
        "map session stopped cleanly with a single monotonic cloud frame" if session_ok else "map session is incomplete, failed, mixed-frame, or has rejected frames above the formal limit",
        **session_details,
    )
    if not session_ok:
        blockers.append(checks["session_integrity"]["reason"])

    cloud_snapshot_stamp = quality.get("backend_cloud_snapshot_stamp")
    path_snapshot_stamp = quality.get("backend_path_snapshot_stamp")
    trajectory_first_stamp = quality.get("trajectory_first_stamp")
    trajectory_last_stamp = quality.get("trajectory_last_stamp")
    backend_lag_limit = algorithm_profile.get("backend_max_trajectory_lag_sec")
    backend_snapshot_ok = (
        quality.get("backend_snapshot_after_session_stop") is True
        and _finite_number(cloud_snapshot_stamp)
        and float(cloud_snapshot_stamp) > 0.0
        and _finite_number(path_snapshot_stamp)
        and float(path_snapshot_stamp) > 0.0
        and abs(float(cloud_snapshot_stamp) - float(path_snapshot_stamp)) <= 1.0e-6
        and trajectory_ok
        and _finite_number(trajectory_first_stamp)
        and _finite_number(trajectory_last_stamp)
        and abs(float(trajectory_first_stamp) - trajectory_details.get("first_stamp", math.inf))
        <= 1.0e-9
        and abs(float(trajectory_last_stamp) - trajectory_details.get("last_stamp", math.inf))
        <= 1.0e-9
        and session_last_stamp is not None
        and _finite_number(backend_lag_limit)
        and float(trajectory_last_stamp) <= session_last_stamp + 0.15
        and session_last_stamp - float(trajectory_last_stamp)
        <= float(backend_lag_limit)
    )
    checks["backend_snapshot"] = _check(
        backend_snapshot_ok,
        (
            "optimized cloud and full trajectory share a fresh post-stop backend snapshot"
            if backend_snapshot_ok
            else "backend cloud/path snapshot is stale, mismatched, or inconsistent with the stopped session"
        ),
        cloud_snapshot_stamp=cloud_snapshot_stamp,
        path_snapshot_stamp=path_snapshot_stamp,
        trajectory_first_stamp=trajectory_first_stamp,
        trajectory_last_stamp=trajectory_last_stamp,
        after_session_stop=quality.get("backend_snapshot_after_session_stop"),
    )
    if not backend_snapshot_ok:
        blockers.append(checks["backend_snapshot"]["reason"])

    runtime_flags_ok = quality.get("tf_conflict_runtime_check") in {
        "PASS_EVIDENCE",
        "RUNTIME_GRAPH_CHECKED",
    }
    checks["runtime_provenance"] = _check(
        runtime_flags_ok,
        "runtime provenance flags are recorded" if runtime_flags_ok else "quality report still carries mock/unverified runtime provenance",
    )
    if not runtime_flags_ok:
        blockers.append(checks["runtime_provenance"]["reason"])

    amp_headers = []
    for name in ("map_pointcloud_amp.pcd", "map_pointcloud_amp.ply"):
        path = output / name
        if path.is_file():
            ok, reason, count = _point_file_header(path, require_intensity=True)
            amp_headers.append((name, ok, reason, count))
        else:
            amp_headers.append((name, False, "file is missing", None))
    amp_file_ok = all(item[1] for item in amp_headers)
    amp_metrics = [
        quality.get("pointcloud_amp_min"),
        quality.get("pointcloud_amp_median"),
        quality.get("pointcloud_amp_p95"),
        quality.get("pointcloud_amp_max"),
    ]
    amp_metrics_ok = (
        all(_finite_number(value) and float(value) >= 0.0 for value in amp_metrics)
        and all(
            float(left) <= float(right)
            for left, right in zip(amp_metrics, amp_metrics[1:])
        )
        and float(amp_metrics[-1]) > 0.0
    )
    amp_quality_ok = (
        quality.get("pointcloud_amp_preserved") is True
        and _finite_number(quality.get("pointcloud_amp_point_count"))
        and float(quality.get("pointcloud_amp_point_count")) > 0.0
        and _finite_number(point_count)
        and float(quality.get("pointcloud_amp_point_count")) == float(point_count)
        and amp_metrics_ok
    )
    amp_source_ok = (not backend_geometry) or quality.get("pointcloud_amp_source") == "backend_registered_cloud"
    amp_counts = [item[3] for item in amp_headers if item[3] is not None]
    amp_shape_ok = len(amp_counts) == 2 and len(set(amp_counts)) == 1 and (
        not _finite_number(point_count) or amp_counts[0] == int(float(point_count))
    )
    intensity_ok = (not require_intensity) or (amp_file_ok and amp_shape_ok and amp_quality_ok and amp_source_ok)
    checks["intensity_preservation"] = _check(
        intensity_ok,
        "PointCloud+Amp files and non-empty intensity metadata are present" if intensity_ok else "PointCloud+Amp is missing or was not marked preserved",
        files=amp_file_ok,
        headers={item[0]: {"passed": item[1], "reason": item[2], "points": item[3]} for item in amp_headers},
        point_counts_match=amp_shape_ok,
        metadata=amp_quality_ok,
        finite_ordered_metrics=amp_metrics_ok,
        source=amp_source_ok,
        required=require_intensity,
    )
    if not intensity_ok:
        blockers.append(checks["intensity_preservation"]["reason"])

    if require_occupancy:
        occupancy_files = all((output / name).is_file() for name in ("map_2d.pgm", "map_2d.yaml"))
        checks["occupancy_product"] = _check(occupancy_files, "2D occupancy side product exists" if occupancy_files else "2D occupancy side product is missing")
        if not occupancy_files:
            blockers.append(checks["occupancy_product"]["reason"])
    else:
        checks["occupancy_product"] = _check(True, "2D occupancy is not required for this 3D acceptance invocation", required=False)

    evidence: dict[str, Any] = {}
    sidecar_path = output / "formal_acceptance.json"
    sidecar, sidecar_error = _load_json(sidecar_path)
    sidecar_ok = sidecar is not None and sidecar.get("schema_version") == SCHEMA_VERSION
    if sidecar is None:
        checks["formal_evidence_sidecar"] = _check(False, sidecar_error or "formal_acceptance.json is missing")
        blockers.append(checks["formal_evidence_sidecar"]["reason"])
    else:
        checks["formal_evidence_sidecar"] = _check(
            sidecar_ok,
            "formal evidence sidecar schema version is supported" if sidecar_ok else "formal evidence sidecar schema version is unsupported",
            path=str(sidecar_path),
        )
        if not sidecar_ok:
            blockers.append(checks["formal_evidence_sidecar"]["reason"])

    if evidence_path:
        selected_evidence = Path(evidence_path).expanduser().resolve()
        loaded, error = _load_json(selected_evidence) if selected_evidence.is_file() else (None, "file does not exist")
        external_ok = loaded is not None and loaded.get("schema_version") == SCHEMA_VERSION
        evidence = loaded or {}
        checks["formal_evidence"] = _check(
            external_ok,
            "external formal evidence schema version is supported" if external_ok else f"external formal evidence is invalid: {error or 'unsupported schema'}",
            path=str(selected_evidence),
        )
        if not external_ok:
            blockers.append(checks["formal_evidence"]["reason"])
        elif sidecar_ok and loaded != sidecar:
            checks["formal_evidence_consistency"] = _check(False, "external evidence does not match manifest-covered formal_acceptance.json")
            blockers.append(checks["formal_evidence_consistency"]["reason"])
        else:
            checks["formal_evidence_consistency"] = _check(True, "external evidence matches the manifest-covered sidecar")
    elif sidecar_ok:
        evidence = sidecar or {}
        checks["formal_evidence"] = _check(True, "manifest-covered formal evidence sidecar is supported", path=str(sidecar_path))
    else:
        embedded = quality.get("formal_acceptance")
        if isinstance(embedded, dict):
            evidence = embedded
            checks["formal_evidence"] = _check(
                embedded.get("schema_version") == SCHEMA_VERSION,
                "embedded formal evidence schema version is supported" if embedded.get("schema_version") == SCHEMA_VERSION else "embedded formal evidence schema version is unsupported",
            )
        else:
            checks["formal_evidence"] = _check(False, "formal_acceptance.json/equivalent embedded evidence is missing")
        if not checks["formal_evidence"]["passed"]:
            blockers.append(checks["formal_evidence"]["reason"])

    if sidecar_ok and isinstance(quality.get("formal_acceptance"), dict) and quality["formal_acceptance"] != sidecar:
        checks["formal_evidence_consistency"] = _check(False, "quality_report formal_acceptance differs from sidecar evidence")
        blockers.append(checks["formal_evidence_consistency"]["reason"])

    source_artifacts = evidence.get("source_artifacts")
    evidence_provenance_ok = (
        isinstance(evidence.get("evidence_id"), str)
        and bool(evidence["evidence_id"].strip())
        and isinstance(evidence.get("generated_at_utc"), str)
        and bool(evidence["generated_at_utc"].strip())
        and isinstance(source_artifacts, list)
        and bool(source_artifacts)
        and all(
            (isinstance(value, str) and bool(value.strip()))
            or (isinstance(value, dict) and bool(value))
            for value in source_artifacts
        )
        and (
            _status_pass(evidence.get("operator_attestation"))
            or str(evidence.get("operator_attestation", "")).strip().upper()
            == "CONFIRMED"
        )
    )
    checks["evidence_provenance"] = _check(
        evidence_provenance_ok,
        (
            "formal evidence has an identity, timestamp, source artifacts, and operator attestation"
            if evidence_provenance_ok
            else "formal evidence provenance/attestation is incomplete"
        ),
    )
    if not evidence_provenance_ok:
        blockers.append(checks["evidence_provenance"]["reason"])

    # Evidence gates.  A status alone is not enough for the gates where a
    # metric or a scenario list is necessary to make the claim reproducible.
    hardware_evidence_passed, hardware_evidence_item = _gate_status(evidence, "hardware_validation")
    identity_evidence_ok, identity_evidence_reason = _observed_hardware_identity_check(
        hardware_evidence_item,
        hardware_profile.get("hardware_identity"),
    )
    hardware_evidence_ok = (
        hardware_evidence_passed
        and quality.get("hardware_validated") is True
        and identity_evidence_ok
    )
    checks["hardware_evidence"] = _check(
        hardware_evidence_ok,
        (
            "hardware evidence is PASS and all observed identities match"
            if hardware_evidence_ok
            else "hardware evidence must be PASS, hardware_validated=true, and "
            + identity_evidence_reason
        ),
    )
    if not hardware_evidence_ok:
        blockers.append(checks["hardware_evidence"]["reason"])
    for name, required in (("dual_lidar", require_dual_lidar), ("loop_closure", require_loop_closure), ("repeatability", require_repeatability)):
        passed, item = _gate_status(evidence, name)
        if not required:
            checks[name] = _check(True, f"{name} is not required for this invocation", required=False)
            continue
        checks[name] = _check(passed, f"{name} evidence is PASS" if passed else f"{name} evidence is missing or not PASS")
        if not passed:
            blockers.append(checks[name]["reason"])
        if name == "dual_lidar" and passed:
            has_sides = isinstance(item.get("left"), dict) and isinstance(item.get("right"), dict)
            side_metrics_ok = has_sides and all(
                _status_pass(item[side].get("status"))
                and (_metric_number(item[side], "valid_fraction", "valid_point_fraction") or -1.0) >= 0.90
                and (_metric_number(item[side], "rate_hz", "receive_rate_hz", "scan_rate_hz") or -1.0) > 0.0
                for side in ("left", "right")
            )
            checks[name] = _check(
                side_metrics_ok,
                "dual-lidar PASS includes passing left/right quality and rate evidence" if side_metrics_ok else "dual-lidar evidence lacks passing left/right status, valid fraction, or rate",
            )
            if not side_metrics_ok:
                blockers.append(checks[name]["reason"])
        if name == "loop_closure" and passed:
            closures = _metric_number(item, "closures", "count", "loop_count")
            checks[name] = _check(closures is not None and closures >= 1.0, "at least one measured loop closure is recorded" if closures is not None and closures >= 1.0 else "loop closure PASS lacks a positive measured closure count")
            if not checks[name]["passed"]:
                blockers.append(checks[name]["reason"])
        if name == "repeatability" and passed:
            runs = _metric_number(item, "runs", "repeat_count")
            checks[name] = _check(runs is not None and runs >= 3.0, "three or more repeat runs are recorded" if runs is not None and runs >= 3.0 else "repeatability PASS requires at least three runs")
            if not checks[name]["passed"]:
                blockers.append(checks[name]["reason"])

    passed, item = _gate_status(evidence, "timestamp_sync")
    source = str(item.get("source", "")).strip().lower()
    monotonic = item.get("monotonic") is True
    offset = _metric_number(item, "max_offset_sec", "maximum_offset_sec", "offset_sec")
    sensor_clock_evidence = item.get("sensors")
    sensor_clocks_ok = isinstance(sensor_clock_evidence, dict) and all(
        isinstance(sensor_clock_evidence.get(name), dict)
        and str(sensor_clock_evidence[name].get("source", "")).strip().lower()
        in _APPROVED_TIME_SOURCES
        and sensor_clock_evidence[name].get("monotonic") is True
        for name in ("xtm60_left", "xtm60_right", "imu")
    )
    timestamp_ok = (
        passed
        and source in _APPROVED_TIME_SOURCES
        and monotonic
        and sensor_clocks_ok
        and offset is not None
        and 0.0 <= offset <= max_time_offset_sec
    )
    checks["timestamp_sync"] = _check(
        timestamp_ok,
        "timestamp source is synchronised, monotonic, and within the configured offset limit" if timestamp_ok else "timestamp evidence must prove a non-host clock source, monotonic samples, and bounded offset",
        source=source,
        max_offset_sec=offset,
        limit_sec=max_time_offset_sec,
        sensor_clocks=sensor_clocks_ok,
    )
    if not timestamp_ok:
        blockers.append(checks["timestamp_sync"]["reason"])

    passed, item = _gate_status(evidence, "extrinsics")
    contract_status = str(item.get("contract_status", "")).upper()
    bundled_contract_path = output / "calibration_contract_used.json"
    contract_ok, contract_reason, bundled_contract = _contract_check(
        bundled_contract_path,
        hardware_profile=hardware_profile,
    )
    contract_digest = ""
    try:
        if bundled_contract_path.is_file():
            contract_digest = hashlib.sha256(
                bundled_contract_path.read_bytes()
            ).hexdigest()
    except OSError:
        contract_digest = ""
    contract_binding_ok = bool(contract_digest) and item.get("contract_sha256") == contract_digest
    external_contract_consistent = True
    if calibration_contract_path:
        external_path = Path(calibration_contract_path).expanduser().resolve()
        external_contract, _ = _load_json(external_path)
        external_contract_consistent = (
            external_contract is not None and external_contract == bundled_contract
        )
    checks["calibration_contract_bundle"] = _check(
        contract_ok and contract_binding_ok and external_contract_consistent,
        (
            "manifest-covered calibration contract is approved, profile-matched, and evidence-bound"
            if contract_ok and contract_binding_ok and external_contract_consistent
            else "calibration contract bundle is invalid, unbound, or differs from the supplied contract"
        ),
        contract=contract_reason,
        sha256=contract_digest,
        evidence_binding=contract_binding_ok,
        external_consistency=external_contract_consistent,
    )
    if not checks["calibration_contract_bundle"]["passed"]:
        blockers.append(checks["calibration_contract_bundle"]["reason"])
    extrinsics_ok = (
        passed
        and contract_status == "APPROVED"
        and contract_ok
        and contract_binding_ok
        and external_contract_consistent
    )
    checks["extrinsics"] = _check(
        extrinsics_ok,
        "extrinsics evidence and an APPROVED finite runtime contract are present" if extrinsics_ok else "extrinsics require PASS evidence plus an APPROVED calibration contract",
        contract=contract_reason,
    )
    if not extrinsics_ok:
        blockers.append(checks["extrinsics"]["reason"])

    passed, item = _gate_status(evidence, "dynamic_validation")
    scenarios = _scenario_statuses(item)
    required_scenarios = ("straight", "turn", "in_place", "stop_recovery")
    scenario_ok = all(
        _status_pass(value.get("status") if isinstance(value, dict) else value)
        for key in required_scenarios
        for value in [scenarios.get(key)]
    )
    dynamic_ok = passed and scenario_ok
    checks["dynamic_validation"] = _check(
        dynamic_ok,
        "straight/turn/in-place/stop-recovery scenarios all pass" if dynamic_ok else "dynamic evidence must include four passing motion scenarios",
        scenarios=sorted(scenarios),
    )
    if not dynamic_ok:
        blockers.append(checks["dynamic_validation"]["reason"])

    passed, item = _gate_status(evidence, "realtime")
    gap = _metric_number(item, "max_output_gap_sec", "max_gap_sec", "maximum_gap_sec")
    drop_rate = _metric_number(item, "drop_rate", "frame_drop_rate")
    duration = _metric_number(item, "duration_sec", "elapsed_sec", "test_duration_sec")
    samples = _metric_number(item, "odom_samples", "samples", "output_samples")
    realtime_ok = (
        passed
        and gap is not None
        and 0.0 <= gap <= max_output_gap_sec
        and drop_rate is not None
        and 0.0 <= drop_rate <= max_drop_rate
        and duration is not None
        and duration >= min_realtime_duration_sec
        and samples is not None
        and samples >= 100.0
    )
    checks["realtime"] = _check(
        realtime_ok,
        "output gap and drop rate are within configured limits" if realtime_ok else "realtime evidence must contain bounded max gap and drop rate",
        max_output_gap_sec=gap,
        max_gap_limit_sec=max_output_gap_sec,
        drop_rate=drop_rate,
        drop_rate_limit=max_drop_rate,
        duration_sec=duration,
        minimum_duration_sec=min_realtime_duration_sec,
        output_samples=samples,
    )
    if not realtime_ok:
        blockers.append(checks["realtime"]["reason"])

    passed, item = _gate_status(evidence, "tf_ownership")
    expected_tf_edges = algorithm_profile.get("tf_edges")
    recorded_tf_edges = item.get("edges")
    edge_results = {}
    if isinstance(expected_tf_edges, dict):
        for role, expected_edge in expected_tf_edges.items():
            recorded = (
                recorded_tf_edges.get(role)
                if isinstance(recorded_tf_edges, dict)
                else None
            )
            publishers = (
                _metric_number(recorded, "publishers")
                if isinstance(recorded, dict)
                else None
            )
            edge_results[role] = {
                "expected": expected_edge,
                "recorded": recorded.get("edge") if isinstance(recorded, dict) else None,
                "publishers": publishers,
                "passed": (
                    isinstance(recorded, dict)
                    and recorded.get("edge") == expected_edge
                    and publishers == 1.0
                ),
            }
    tf_ok = passed and bool(edge_results) and all(
        result["passed"] for result in edge_results.values()
    )
    checks["tf_ownership"] = _check(
        tf_ok,
        (
            "every named global/local/base TF edge has exactly one publisher"
            if tf_ok
            else "TF ownership evidence must name the profile's actual edges and prove one publisher each"
        ),
        edges=edge_results,
    )
    if not tf_ok:
        blockers.append(checks["tf_ownership"]["reason"])

    unique_blockers = list(dict.fromkeys(blockers))
    return {
        "schema_version": SCHEMA_VERSION,
        "formal_ready": not unique_blockers,
        "bundle": str(output),
        "checks": checks,
        "blockers": unique_blockers,
        "thresholds": {
            "max_time_offset_sec": max_time_offset_sec,
            "max_output_gap_sec": max_output_gap_sec,
            "max_drop_rate": max_drop_rate,
            "min_realtime_duration_sec": min_realtime_duration_sec,
            "max_rejected_frame_rate": max_rejected_frame_rate,
        },
    }
