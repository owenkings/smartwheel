#!/usr/bin/env python3
"""Derive a provisional XT-M60 sensor-to-base orientation from a floor fit."""

import argparse
import json
import math
from pathlib import Path

import numpy as np


def rotation_from_floor_normal(normal_xyz, yaw_deg=0.0):
    """Map XT axes (+z fwd, +x left, +y up) into a level ROS base frame."""
    normal = np.asarray(normal_xyz, dtype=np.float64)
    normal /= np.linalg.norm(normal)
    optical_forward = np.array([0.0, 0.0, 1.0])
    forward = optical_forward - normal * float(np.dot(optical_forward, normal))
    forward /= np.linalg.norm(forward)
    left = np.cross(normal, forward)
    left /= np.linalg.norm(left)
    level = np.vstack((forward, left, normal))

    yaw = math.radians(float(yaw_deg))
    rz = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    return rz @ level


def matrix_to_rpy(rotation):
    pitch = math.asin(-float(rotation[2, 0]))
    roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
    yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    return roll, pitch, yaw


def matrix_to_quaternion(rotation):
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            w = (rotation[2, 1] - rotation[1, 2]) / s
            x = 0.25 * s
            y = (rotation[0, 1] + rotation[1, 0]) / s
            z = (rotation[0, 2] + rotation[2, 0]) / s
        elif index == 1:
            s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            w = (rotation[0, 2] - rotation[2, 0]) / s
            x = (rotation[0, 1] + rotation[1, 0]) / s
            y = 0.25 * s
            z = (rotation[1, 2] + rotation[2, 1]) / s
        else:
            s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            w = (rotation[1, 0] - rotation[0, 1]) / s
            x = (rotation[0, 2] + rotation[2, 0]) / s
            y = (rotation[1, 2] + rotation[2, 1]) / s
            z = 0.25 * s
    return x, y, z, w


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ground-json", type=Path, required=True)
    parser.add_argument("--xyz", type=float, nargs=3, required=True)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    source = json.loads(args.ground_json.read_text(encoding="utf-8"))
    plane = source.get("plane")
    if plane is None:
        raise SystemExit("ground diagnostic contains no plane")
    if not source.get("checks", {}).get("floor_below_sensor_if_y_is_up", False):
        raise SystemExit("selected plane is not a floor below the sensor")

    normal = np.asarray(plane["normal_xyz"], dtype=np.float64)
    normal /= np.linalg.norm(normal)
    rotation = rotation_from_floor_normal(normal, args.yaw_deg)
    rpy = matrix_to_rpy(rotation)
    quaternion = matrix_to_quaternion(rotation)
    transformed_normal = rotation @ normal
    floor_z = float(args.xyz[2]) - float(plane["d"])
    orthogonality_error = float(
        np.max(np.abs(rotation @ rotation.T - np.eye(3)))
    )

    result = {
        "schema": "smartwheel.xtm60_orientation_diagnostic.v1",
        "read_only": True,
        "source_ground_json": str(args.ground_json),
        "sensor_axes": {"x": "left", "y": "up", "z": "forward"},
        "translation_xyz_m": [float(value) for value in args.xyz],
        "horizontal_yaw": {
            "value_deg": float(args.yaw_deg),
            "status": "constrained_not_measured",
        },
        "floor_normal_sensor_xyz": [float(value) for value in normal],
        "physical_adjustment_estimate_deg": {
            "nose_down": math.degrees(math.atan2(-normal[2], normal[1])),
            "roll_about_forward": math.degrees(math.atan2(normal[0], normal[1])),
        },
        "sensor_to_base_rotation_matrix": rotation.tolist(),
        "sensor_to_base_rpy_rad": [float(value) for value in rpy],
        "sensor_to_base_rpy_deg": [math.degrees(value) for value in rpy],
        "sensor_to_base_quaternion_xyzw": [float(value) for value in quaternion],
        "transformed_floor_normal_base_xyz": [
            float(value) for value in transformed_normal
        ],
        "transformed_floor_height_base_m": floor_z,
        "orthogonality_error": orthogonality_error,
        "determinant": float(np.linalg.det(rotation)),
    }
    result["checks"] = {
        "rotation_orthonormal": orthogonality_error < 1e-9,
        "right_handed": abs(result["determinant"] - 1.0) < 1e-9,
        "floor_normal_is_base_up": bool(
            np.linalg.norm(transformed_normal - np.array([0.0, 0.0, 1.0])) < 1e-9
        ),
        "floor_height_within_5cm_of_base_zero": abs(floor_z) <= 0.05,
        "horizontal_yaw_calibrated": False,
    }

    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
