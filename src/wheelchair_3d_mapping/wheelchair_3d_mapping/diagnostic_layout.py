"""One provisional layout drives diagnostic URDF, FAST extrinsics and bridges.

This is deliberately NOT an alternative approved formal calibration contract.
Only the diagnostic launch may opt in; formal_lio_contract remains unchanged.
"""

from pathlib import Path

import numpy as np
import yaml

from .formal_lio_contract import (
    _finite_vector, _quaternion_from_rotation, _rotation_from_rpy,
)


def load_diagnostic_layout(path, *, radar="right"):
    """Validate a provisional layout and derive all bridges from one source.

    Returns FAST extrinsic_R/T, body_to_base_translation/quaternion,
    map_to_camera_init_translation/quaternion, and sensor xyz/rpy mappings.
    """
    if radar not in {"left", "right"}:
        raise ValueError("radar must be left or right")
    source = Path(path).resolve()
    data = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise ValueError("diagnostic layout schema_version must be 1")
    if (data.get("status") != "DIAGNOSTIC_UNCALIBRATED"
            or data.get("formal_runtime_eligible") is not False):
        raise ValueError("diagnostic profile must remain explicitly uncalibrated")
    if data.get("parent_frame") != "base_link":
        raise ValueError("diagnostic layout parent must be base_link")
    sensors = data.get("sensors", {})
    normalized = {}
    rotations = {}
    for name, frame in (("left", "xtm60_left_link"),
                        ("right", "xtm60_right_link"), ("imu", "imu_link")):
        item = sensors.get(name, {})
        if item.get("frame") != frame:
            raise ValueError(f"unexpected {name} frame")
        xyz = _finite_vector(item.get("xyz"), label=f"{name}.xyz")
        rpy = _finite_vector(item.get("rpy"), label=f"{name}.rpy")
        if not 0.1 <= xyz[2] <= 2.0:
            raise ValueError(f"{name} height outside diagnostic sanity bounds")
        normalized[name] = {"frame": frame, "xyz": xyz.tolist(), "rpy": rpy.tolist()}
        rotations[name] = _rotation_from_rpy(rpy)
    left, right = (np.asarray(normalized[name]["xyz"]) for name in ("left", "right"))
    if not np.allclose(left[[0, 2]], right[[0, 2]], atol=1e-9, rtol=0):
        raise ValueError("diagnostic layout violates equal-height/equal-fore-aft constraint")
    if not left[1] > right[1]:
        raise ValueError("left LiDAR must lie left of right LiDAR")
    imu_xyz = np.asarray(normalized["imu"]["xyz"])
    inverse_imu = rotations["imu"].T
    return {
        "profile_path": str(source),
        "status": "DIAGNOSTIC_UNCALIBRATED",
        "formal_runtime_eligible": False,
        "sensors": normalized,
        "radar_frame": normalized[radar]["frame"],
        "extrinsic_T": (inverse_imu @ (np.asarray(normalized[radar]["xyz"]) - imu_xyz)).tolist(),
        "extrinsic_R": (inverse_imu @ rotations[radar]).reshape(-1).tolist(),
        "base_to_imu_R": rotations["imu"].reshape(-1).tolist(),
        "base_to_imu_T": imu_xyz.tolist(),
        "body_to_base_translation": (-inverse_imu @ imu_xyz).tolist(),
        "body_to_base_quaternion": list(_quaternion_from_rotation(inverse_imu)),
        "map_to_camera_init_translation": imu_xyz.tolist(),
        "map_to_camera_init_quaternion": list(_quaternion_from_rotation(rotations["imu"])),
    }
