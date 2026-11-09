"""Configuration of the live input chain, read from a frozen frontend cell plus explicit calibration files.

The frontend cell supplies what the offline chain already froze for this
hardware: the left lidar and IMU mounts (``T_base_axle``, ``R_base_imu``), the
host cloud filter, the planar motion-prior settings and the device identities.
As in the frontend command (``--hardware-setup``, ``--gyro-bias``), the wheel
parameters come from the ``wheel_odometry`` block of an explicit hardware setup
file and the gyro bias from an explicit bias file; recordings that froze older
wheel values are therefore processed with the same values as their frontend.
The IMU uses arrival timing: device-relative timing models are offline only.
Nothing here opens a device or starts ROS.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path

import numpy as np

LEFT_FILTERED_TOPIC = "/wc_mapping/lidar_left/source_frame_filtered"
# Read only by the 3D obstacle input (window C, perception/live_obstacles.py), which is off by default.
RIGHT_FILTERED_TOPIC = "/wc_mapping/lidar_right/source_frame_filtered"
IMU_TOPIC = "/wc_mapping/imu/source_frame"
WHEEL_TOPIC = "/wc_mapping/wheel/feedback_raw"
SOURCE_TOPICS = (LEFT_FILTERED_TOPIC, IMU_TOPIC, WHEEL_TOPIC)


def source_topics(obstacles=False):
    """Sensor topics a run reads: the three of the live chain, plus the right lidar with the 3D obstacle input."""
    return SOURCE_TOPICS + ((RIGHT_FILTERED_TOPIC,) if obstacles else ())
# Wheel fields the hardware setup may replace; signs and gap limits stay as frozen for the device.
WHEEL_OVERRIDE_KEYS = ("wheel_radius_m", "track_width_m", "register_to_wheel_rpm", "max_wheel_speed_m_s",
                       "left_sign", "right_sign", "max_gap_s", "provenance", "state", "schema",
                       "formal_odometry_eligible")


class LiveConfigError(ValueError):
    pass


def _read_json(path, label):
    path = Path(path).expanduser().resolve(strict=True)
    raw = path.read_bytes()
    if len(raw) > 10_000_000:
        raise LiveConfigError(label + " is too large")
    return json.loads(raw), {"path": str(path), "sha256": hashlib.sha256(raw).hexdigest()}


def merged_wheel(frozen, hardware_setup):
    """Frozen wheel candidate with the hardware setup's wheel_odometry values; returns (wheel, changed keys)."""
    block = hardware_setup.get("wheel_odometry")
    if not isinstance(block, dict):
        raise LiveConfigError("hardware setup has no wheel_odometry block")
    wheel = copy.deepcopy(frozen)
    changed = {}
    for key in WHEEL_OVERRIDE_KEYS:
        if key in block and block[key] != wheel.get(key):
            changed[key] = {"frozen": wheel.get(key), "applied": block[key]}
            wheel[key] = copy.deepcopy(block[key])
    for key in ("wheel_radius_m", "track_width_m"):
        value = wheel.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value < 2:
            raise LiveConfigError(key + " must be a positive length")
    return wheel, changed


def load_live_config(cell, hardware_setup, gyro_bias):
    """Return the live chain configuration and its provenance (hashes and replaced values)."""
    cell = Path(cell).expanduser().resolve(strict=True)
    runtime, runtime_ref = _read_json(cell / "runtime_config.json", "runtime config")
    prior, prior_ref = _read_json(cell / "prior_config.json", "prior config")
    setup, setup_ref = _read_json(hardware_setup, "hardware setup")
    bias, bias_ref = _read_json(gyro_bias, "gyro bias")
    if runtime.get("mode") != "left" or runtime.get("cloud_source") != "filtered":
        raise LiveConfigError("a single-left frontend on the filtered cloud is required")
    if prior.get("motion_model") != "planar_ekf" or prior.get("base_frame") != "lidar_left":
        raise LiveConfigError("planar motion prior with the left lidar base is required")
    if bias.get("sensor_id") != prior.get("imu_sensor_id"):
        raise LiveConfigError("gyro bias belongs to another IMU")
    wheel, changed = merged_wheel(prior["wheel_candidate"], setup)
    prior = copy.deepcopy(prior)
    prior["wheel_candidate"] = wheel
    prior["confirmed_gyro_bias"] = {k: copy.deepcopy(v) for k, v in bias.items()}
    timing = prior.get("offline_imu_time_mode", "arrival")
    prior["offline_imu_time_mode"] = "arrival"
    prior.pop("offline_imu_time_model", None)
    filter_config = {"cloud_filter": copy.deepcopy(runtime["cloud_filter"]), "mapping_enabled": True,
                     "wheel_candidate": copy.deepcopy(wheel),
                     "hardware_setup_provenance": copy.deepcopy(runtime.get("hardware_setup_provenance"))}
    base_axle = np.asarray(prior["T_base_axle"], dtype=float)
    if base_axle.shape != (4, 4) or not np.isfinite(base_axle).all():
        raise LiveConfigError("finite T_base_axle required")
    rotation = base_axle[:3, :3].T            # axle axes at the lidar origin (mapping_reference)
    lever = np.asarray(runtime["mounts"]["left"]["t_axle_lidar_m"][:2], dtype=float)
    if lever.shape != (2,) or not np.isfinite(lever).all() or np.hypot(*lever) > 2.0:
        raise LiveConfigError("finite planar lever arm required")
    initialization = cell / "prior_initialization.json"
    if initialization.is_file():
        frozen = np.asarray(json.loads(initialization.read_text(encoding="utf-8"))["R_reference_base"], dtype=float)
        if not np.allclose(frozen, rotation, atol=1e-9):
            raise LiveConfigError("mount rotation differs from the frozen prior initialization")
    return {
        "prior": prior, "filter": filter_config, "T_base_axle": base_axle.tolist(),
        "R_reference_base": rotation.tolist(), "lever_xy": lever.tolist(),
        "identities": {"session_id_frozen": runtime.get("session_id"), "lidar_left": runtime["sensor_ids"]["left"],
                       "imu": prior["imu_sensor_id"], "wheel": prior.get("wheel_device_id")},
        "provenance": {"cell": str(cell), "runtime_config": runtime_ref, "prior_config": prior_ref,
                       "hardware_setup": setup_ref, "gyro_bias": bias_ref, "wheel_values_replaced": changed,
                       "frozen_imu_time_mode": timing, "live_imu_time_mode": "arrival"},
    }
