"""Strict calibration-contract conversion for the formal FAST-LIO2 route.

The calibration contract records each sensor as ``base_link -> sensor`` in
XYZ/RPY form.  FAST-LIO2 expects the selected LiDAR pose in its IMU body
frame, while its hard-coded ``body`` TF frame must be bridged back to
``base_link``.  This module performs those two rigid-transform operations
without importing ROS so the conventions are unit-testable offline.
"""

from __future__ import annotations

import ipaddress
import json
import math
from pathlib import Path

import numpy as np


_EXPECTED_CHILDREN = {
    "xtm60_left": "xtm60_left_link",
    "xtm60_right": "xtm60_right_link",
    "imu": "imu_link",
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


def _required_text(value: object, *, label: str) -> str:
    if not isinstance(value, str) or value.strip().lower() in {"", "none", "null"}:
        raise ValueError(f"{label} is required")
    return value.strip()


def validate_hardware_identity(
    value: object,
    *,
    installation_epoch: str,
    label: str = "hardware_identity",
) -> dict:
    """Validate and normalize the canonical dual-LiDAR/H30 identity binding.

    IP addresses are checked as configuration metadata, but hardware identity
    remains the vendor serial for each XT-M60 and a project-assigned stable
    installation asset ID for the H30. Linux device paths are deliberately
    excluded because they can change after a USB reconnect.
    """

    expected_epoch = _required_text(
        installation_epoch, label=f"{label} expected installation_epoch"
    )
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    if value.get("schema_version") != 1:
        raise ValueError(f"{label}.schema_version must be 1")
    identity_epoch = _required_text(
        value.get("installation_epoch"),
        label=f"{label}.installation_epoch",
    )
    if identity_epoch != expected_epoch:
        raise ValueError(
            f"{label}.installation_epoch does not match the enclosing document"
        )

    assets = value.get("assets")
    expected_roles = set(_EXPECTED_HARDWARE_ASSETS)
    if not isinstance(assets, dict) or set(assets) != expected_roles:
        roles = ", ".join(_EXPECTED_HARDWARE_ASSETS)
        raise ValueError(f"{label}.assets must contain exactly: {roles}")

    normalized = {
        "schema_version": 1,
        "installation_epoch": identity_epoch,
        "assets": {},
    }
    identity_owners: dict[str, str] = {}
    lidar_ip_owners: dict[str, str] = {}
    for role, expected in _EXPECTED_HARDWARE_ASSETS.items():
        item = assets.get(role)
        if not isinstance(item, dict):
            raise ValueError(f"{label}.assets.{role} must be a mapping")
        if _required_text(
            item.get("role"), label=f"{label}.assets.{role}.role"
        ) != role:
            raise ValueError(f"{label}.assets.{role}.role must be {role}")

        raw_model = _required_text(
            item.get("model"), label=f"{label}.assets.{role}.model"
        )
        if raw_model.upper() != expected["model"].upper():
            raise ValueError(
                f"{label}.assets.{role}.model must be {expected['model']}"
            )
        identity_kind = _required_text(
            item.get("identity_kind"),
            label=f"{label}.assets.{role}.identity_kind",
        )
        if identity_kind != expected["identity_kind"]:
            raise ValueError(
                f"{label}.assets.{role}.identity_kind must be "
                f"{expected['identity_kind']}"
            )
        hardware_id = _required_text(
            item.get("hardware_id"),
            label=f"{label}.assets.{role}.hardware_id",
        )
        previous_role = identity_owners.get(hardware_id)
        if previous_role is not None:
            raise ValueError(
                f"{label} hardware_id is duplicated by {previous_role} and {role}"
            )
        identity_owners[hardware_id] = role

        frame_id = _required_text(
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
            ip_address = _required_text(
                item.get("ip_address"),
                label=f"{label}.assets.{role}.ip_address",
            )
            try:
                parsed_ip = ipaddress.ip_address(ip_address)
            except ValueError as exc:
                raise ValueError(
                    f"{label}.assets.{role}.ip_address is invalid"
                ) from exc
            if parsed_ip.version != 4:
                raise ValueError(
                    f"{label}.assets.{role}.ip_address must be IPv4"
                )
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


def _finite_vector(value, *, label: str) -> np.ndarray:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{label} must contain exactly three values")
    if any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value):
        raise ValueError(f"{label} must contain numeric values")
    vector = np.asarray(value, dtype=np.float64)
    if not np.isfinite(vector).all():
        raise ValueError(f"{label} must contain finite values")
    return vector


def _rotation_from_rpy(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation_x = np.asarray(
        [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64
    )
    rotation_y = np.asarray(
        [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64
    )
    rotation_z = np.asarray(
        [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64
    )
    return rotation_z @ rotation_y @ rotation_x


def _quaternion_from_rotation(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Return a normalized (x, y, z, w) quaternion."""

    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            qw = (matrix[2, 1] - matrix[1, 2]) / scale
            qx = 0.25 * scale
            qy = (matrix[0, 1] + matrix[1, 0]) / scale
            qz = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            qw = (matrix[0, 2] - matrix[2, 0]) / scale
            qx = (matrix[0, 1] + matrix[1, 0]) / scale
            qy = 0.25 * scale
            qz = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            qw = (matrix[1, 0] - matrix[0, 1]) / scale
            qx = (matrix[0, 2] + matrix[2, 0]) / scale
            qy = (matrix[1, 2] + matrix[2, 1]) / scale
            qz = 0.25 * scale
    quaternion = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm < 1.0e-12:
        raise ValueError("derived body-to-base quaternion is invalid")
    quaternion /= norm
    return tuple(float(value) for value in quaternion)


def _contract_transforms(contract: dict) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    transforms = contract.get("runtime_transforms")
    if not isinstance(transforms, dict):
        raise ValueError("approved contract lacks runtime_transforms")
    result = {}
    for name, child in _EXPECTED_CHILDREN.items():
        item = transforms.get(name)
        if not isinstance(item, dict):
            raise ValueError(f"runtime_transforms.{name} is missing")
        frames = item.get("frames")
        if (
            not isinstance(frames, dict)
            or frames.get("parent") != "base_link"
            or frames.get("child") != child
        ):
            raise ValueError(
                f"runtime_transforms.{name} must declare base_link->{child}"
            )
        transform = item.get("runtime_transform")
        if not isinstance(transform, dict):
            raise ValueError(f"runtime_transforms.{name}.runtime_transform is missing")
        translation = _finite_vector(
            transform.get("translation", transform.get("xyz")),
            label=f"runtime_transforms.{name}.translation",
        )
        rpy = _finite_vector(
            transform.get("rotation", transform.get("rpy")),
            label=f"runtime_transforms.{name}.rotation",
        )
        result[name] = (translation, _rotation_from_rpy(rpy))
    return result


def load_formal_lio_contract(path: str | Path, *, radar: str = "right") -> dict:
    """Load a canonical approved contract and derive FAST-LIO2 parameters."""

    radar_name = str(radar).strip().lower()
    if radar_name not in {"left", "right"}:
        raise ValueError("radar must be left or right")
    contract_path = Path(path).expanduser().resolve()
    try:
        contract = json.loads(contract_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"calibration contract is unreadable: {exc}") from exc
    if not isinstance(contract, dict):
        raise ValueError("calibration contract must be a JSON object")
    if contract.get("schema_version") != 1:
        raise ValueError("calibration contract schema_version must be 1")
    if str(contract.get("status", "")).strip().upper() != "APPROVED":
        raise ValueError("calibration contract status must be APPROVED")
    if str(contract.get("scope", "")).strip().lower() != "dual_lidar_imu":
        raise ValueError("calibration contract scope must be dual_lidar_imu")
    installation_epoch = contract.get("installation_epoch")
    if not isinstance(installation_epoch, str) or not installation_epoch.strip():
        raise ValueError("calibration contract installation_epoch is required")
    hardware_identity = validate_hardware_identity(
        contract.get("hardware_identity"),
        installation_epoch=installation_epoch,
        label="calibration contract hardware_identity",
    )

    transforms = _contract_transforms(contract)
    imu_translation, imu_rotation = transforms["imu"]
    lidar_key = f"xtm60_{radar_name}"
    lidar_translation, lidar_rotation = transforms[lidar_key]

    # T_imu_lidar = inv(T_base_imu) @ T_base_lidar.
    imu_rotation_inverse = imu_rotation.T
    lidar_in_imu_rotation = imu_rotation_inverse @ lidar_rotation
    lidar_in_imu_translation = imu_rotation_inverse @ (
        lidar_translation - imu_translation
    )

    # FAST-LIO's hard-coded body frame represents the IMU frame.
    body_to_base_rotation = imu_rotation_inverse
    body_to_base_translation = -(imu_rotation_inverse @ imu_translation)
    return {
        "contract_path": str(contract_path),
        "installation_epoch": installation_epoch.strip(),
        "hardware_identity": hardware_identity,
        "radar": radar_name,
        "radar_frame": _EXPECTED_CHILDREN[lidar_key],
        "input_topic": f"/xtm60/{radar_name}/points",
        "config_name": f"xtm60_{radar_name}_lio.yaml",
        "extrinsic_T": [float(value) for value in lidar_in_imu_translation],
        "extrinsic_R": [float(value) for value in lidar_in_imu_rotation.reshape(-1)],
        "body_to_base_translation": [
            float(value) for value in body_to_base_translation
        ],
        "body_to_base_quaternion": list(
            _quaternion_from_rotation(body_to_base_rotation)
        ),
    }
