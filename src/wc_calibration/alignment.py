"""Offline initial alignment candidates; these tools cannot validate live extrinsics."""

from dataclasses import asdict, dataclass
import hashlib
import json
import math

import numpy as np

from .core import CalibrationError, validate_transform


def _hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def _metadata(data):
    if data.get("schema_version") != 1 or data.get("source_mode") not in {"real", "synthetic"}:
        raise CalibrationError("schema_version=1 and explicit real/synthetic source_mode required")


def _vectors(value, minimum=3):
    array = np.asarray(value, dtype=float)
    if array.ndim != 2 or array.shape[1] != 3 or len(array) < minimum or not np.isfinite(array).all():
        raise CalibrationError("finite Nx3 observations with sufficient samples required")
    return array


def manual_initial(data):
    """Known matched XYZ pairs -> proper rigid right-to-left Kabsch, never scale fitting.

    Three noncollinear *identified* points suffice even if coplanar; this differs
    from unconstrained point-to-plane ICP on one untextured plane.
    """
    _metadata(data)
    ids = data.get("sensor_ids", {})
    if set(ids) != {"left", "right"} or not all(isinstance(x, str) and x for x in ids.values()) or ids["left"] == ids["right"]:
        raise CalibrationError("distinct explicit left/right sensor_ids required")
    conventions = data.get("coordinate_conventions", {})
    if set(conventions) != {"left", "right"} or not all(isinstance(x, str) and x for x in conventions.values()):
        raise CalibrationError("explicit coordinate_conventions for both input clouds required")
    if data.get("units") not in {"m", "mm"}:
        raise CalibrationError("explicit point units m or mm required; scale is never estimated")
    scale = 1.0 if data["units"] == "m" else 0.001
    left, right = (_vectors(data[side]) * scale for side in ("left", "right"))
    if left.shape != right.shape or len(left) > 10000:
        raise CalibrationError("equal paired point counts, at most 10000, required")
    policy = {"min_second_to_first_singular_ratio": 0.01, "max_pair_residual_m": 0.05}
    supplied = data.get("policy", {})
    if set(supplied) - set(policy):
        raise CalibrationError("unknown manual alignment policy")
    policy.update(supplied)
    if any(isinstance(x, bool) or not math.isfinite(x) or x <= 0 for x in policy.values()) or policy["min_second_to_first_singular_ratio"] >= 1:
        raise CalibrationError("manual alignment policy must be finite positive; singular ratio below one")
    centers = [points.mean(axis=0) for points in (left, right)]
    centered = [points - center for points, center in zip((left, right), centers)]
    shape_singular = [np.linalg.svd(points, compute_uv=False) for points in centered]
    for points, singular in zip((left, right), shape_singular):
        if len(np.unique(points, axis=0)) != len(points):
            raise CalibrationError("duplicate selected points do not provide independent constraints")
        if singular[0] <= 1e-12 or singular[1] / singular[0] < policy["min_second_to_first_singular_ratio"]:
            raise CalibrationError("collinear or nearly collinear selected points are degenerate")
    u, singular, vt = np.linalg.svd(centered[1].T @ centered[0])
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ correction @ u.T
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = centers[0] - rotation @ centers[1]
    validate_transform(transform)
    residuals = np.linalg.norm(right @ rotation.T + transform[:3, 3] - left, axis=1)
    reasons = ["PAIR_RESIDUAL_LIMIT_EXCEEDED"] if residuals.max() > policy["max_pair_residual_m"] else []
    return {"schema_version": 1, "kind": "manual_correspondence_initial", "source_mode": data["source_mode"],
            "status": "UNVALIDATED" if reasons else "CANDIDATE", "live_eligible": False,
            "sensor_ids": ids, "coordinate_conventions": conventions, "input_hash": _hash(data),
            "units": "m", "scale_fitted": False, "initial_T_left_right": transform.tolist(),
            "transform_definition": "p_left = R_left_right * p_right + t_left_right; metres",
            "initial_T_right_left": np.linalg.inv(transform).tolist(), "policy": policy,
            "rejection_reasons": reasons, "pair_count": len(left), "residuals_m": residuals.tolist(),
            "rmse_m": float(np.sqrt(np.mean(residuals**2))), "max_residual_m": float(residuals.max()),
            "shape_singular_values_m": {"left": shape_singular[0].tolist(), "right": shape_singular[1].tolist()},
            "cross_covariance_singular_values_m2": singular.tolist(),
            "independent_validation_performed": False,
            "limits": ["Manual matches are assumptions; low fit residual does not prove correct matching.",
                       "Candidate only: use as initial_T_left_right for multiscene ICP and independent holdout.",
                       "No sample synchronization, installation stability or sensor scale validation is inferred."]}


def attach_manual_initial(dataset, candidate):
    """Attach one explicit candidate while preserving all training and holdout observations."""
    if candidate.get("kind") != "manual_correspondence_initial" or candidate.get("status") != "CANDIDATE":
        raise CalibrationError("initial result must be an accepted manual correspondence CANDIDATE")
    if candidate.get("sensor_ids") != dataset.get("sensor_ids") or candidate.get("source_mode") != dataset.get("source_mode"):
        raise CalibrationError("manual candidate sensor identities/source_mode do not match calibration dataset")
    conventions = candidate.get("coordinate_conventions")
    checked = False
    if "coordinate_conventions" in dataset:
        if dataset["coordinate_conventions"] != conventions:
            raise CalibrationError("manual and calibration coordinate conventions differ")
        checked = True
    for split in ("training", "validation"):
        for scene in dataset.get(split, []):
            metadata = scene.get("input_files", scene.get("selected_raw_frames"))
            if metadata is not None:
                observed = {side: metadata.get(side, {}).get("coordinate_convention") for side in ("left", "right")}
                if observed != conventions:
                    raise CalibrationError("manual and scene coordinate conventions differ")
                checked = True
    if not checked:
        raise CalibrationError("calibration dataset must declare coordinate_conventions or retain prepared scene metadata")
    transform = validate_transform(candidate["initial_T_left_right"])
    result = dict(dataset)
    result["initial_T_left_right"] = transform.tolist()
    result["manual_initial_provenance"] = {"input_hash": candidate.get("input_hash"),
                                            "result_hash": _hash(candidate), "live_eligible": False}
    return result


@dataclass(frozen=True)
class GravityPolicy:
    min_samples: int = 100
    max_samples: int = 100000
    min_duration_s: float = 2.0
    gravity_m_s2: float = 9.80665
    max_norm_error_m_s2: float = 1.0
    max_accel_rms_m_s2: float = 0.25
    max_gyro_rad_s: float = 0.05
    max_direction_p95_deg: float = 3.0

    def __post_init__(self):
        if type(self.min_samples) is not int or type(self.max_samples) is not int or not 3 <= self.min_samples <= self.max_samples <= 1000000:
            raise CalibrationError("bounded integer sample counts required")
        if any(isinstance(x, bool) or not math.isfinite(x) or x <= 0 for key, x in asdict(self).items() if key not in {"min_samples", "max_samples"}):
            raise CalibrationError("gravity policy limits must be finite positive numbers")


def imu_gravity(data):
    """Estimate native-frame up and Euler roll/pitch from a static-like specific force.

    No quaternion/yaw, lever arm, IMU-to-lidar rotation, or right-lidar tilt is
    inferred. A constant translational acceleration can pass static-like checks.
    """
    _metadata(data)
    policy = GravityPolicy(**data.get("policy", {}))
    if not isinstance(data.get("sensor_id"), str) or not data["sensor_id"]:
        raise CalibrationError("explicit sensor_id required")
    if not isinstance(data.get("coordinate_convention"), str) or not data["coordinate_convention"]:
        raise CalibrationError("explicit native coordinate_convention required")
    if data.get("acceleration_includes_gravity") is not True or data.get("acceleration_sign") not in {"specific_force", "gravity"}:
        raise CalibrationError("explicit gravity-containing acceleration and specific_force/gravity sign required")
    if not isinstance(data.get("units_evidence"), str) or not data["units_evidence"].strip():
        raise CalibrationError("units_evidence must identify the reviewed conversion or exported units")
    accel_scale = {"m/s^2": 1., "g": policy.gravity_m_s2}.get(data.get("acceleration_units"))
    gyro_scale = {"rad/s": 1., "deg/s": math.pi / 180.}.get(data.get("angular_velocity_units"))
    if accel_scale is None or gyro_scale is None:
        raise CalibrationError("explicit supported acceleration/angular velocity units required")
    samples = data["samples"]
    if not policy.min_samples <= len(samples) <= policy.max_samples:
        raise CalibrationError("gravity sample count outside declared bounds")
    times = [sample["time_ns"] for sample in samples]
    if any(type(value) is not int or value < 0 for value in times) or any(a > b for a, b in zip(times, times[1:])):
        raise CalibrationError("integer nonnegative, nondecreasing observation timestamps required")
    duration = (times[-1] - times[0]) / 1e9
    if duration < policy.min_duration_s:
        raise CalibrationError("selected window is too short")
    acceleration = _vectors([sample["acceleration"] for sample in samples]) * accel_scale
    gyro = _vectors([sample["angular_velocity"] for sample in samples]) * gyro_scale
    magnitudes = np.linalg.norm(acceleration, axis=1)
    mean = acceleration.mean(axis=0)
    mean_norm = float(np.linalg.norm(mean))
    if mean_norm < 1e-9 or np.any(magnitudes < 1e-9):
        raise CalibrationError("zero/near-zero acceleration cannot define gravity direction")
    unit = mean / mean_norm
    up = unit if data["acceleration_sign"] == "specific_force" else -unit
    angles = np.degrees(np.arccos(np.clip(acceleration @ unit / magnitudes, -1., 1.)))
    norm_error = float(np.max(np.abs(magnitudes - policy.gravity_m_s2)))
    rms = float(np.sqrt(np.mean(np.sum((acceleration - mean)**2, axis=1))))
    gyro_max = float(np.max(np.linalg.norm(gyro, axis=1)))
    direction_p95 = float(np.quantile(angles, 0.95))
    reasons = []
    for observed, limit, reason in ((norm_error, policy.max_norm_error_m_s2, "ACCELERATION_NOT_NEAR_GRAVITY"),
                                   (rms, policy.max_accel_rms_m_s2, "ACCELERATION_DISPERSION_TOO_HIGH"),
                                   (gyro_max, policy.max_gyro_rad_s, "ANGULAR_MOTION_OR_GYRO_BIAS_TOO_HIGH"),
                                   (direction_p95, policy.max_direction_p95_deg, "GRAVITY_DIRECTION_UNSTABLE")):
        if observed > limit:
            reasons.append(reason)
    horizontal_yz = math.hypot(float(up[1]), float(up[2]))
    if horizontal_yz < 1e-6:
        reasons.append("ROLL_UNOBSERVABLE_AT_EULER_PITCH_SINGULARITY")
    return {"schema_version": 1, "kind": "imu_static_gravity_candidate", "source_mode": data["source_mode"],
            "status": "UNVALIDATED" if reasons else "CANDIDATE", "live_eligible": False,
            "sensor_id": data["sensor_id"], "coordinate_convention": data["coordinate_convention"],
            "input_hash": _hash(data), "units_evidence": data["units_evidence"], "policy": asdict(policy),
            "input_units": {"acceleration": data["acceleration_units"], "angular_velocity": data["angular_velocity_units"]},
            "declared_acceleration_sign": data["acceleration_sign"], "declared_acceleration_includes_gravity": True,
            "sample_count": len(samples), "duration_by_observation_timestamps_s": duration,
            "up_unit_in_imu": up.tolist(), "gravity_down_unit_in_imu": (-up).tolist(),
            "roll_rad": math.atan2(float(up[1]), float(up[2])) if horizontal_yz >= 1e-6 else None,
            "pitch_rad": math.atan2(-float(up[0]), horizontal_yz), "yaw_rad": None,
            "euler_definition": "R_level_imu = Rz(unknown_yaw) Ry(pitch) Rx(roll), level +Z up; right-handed native XYZ assumed",
            "T_left_imu": None, "T_left_right": None, "time_model_validated": False,
            "statistics": {"mean_acceleration_m_s2": mean.tolist(), "mean_norm_m_s2": mean_norm,
                           "max_norm_error_m_s2": norm_error, "acceleration_rms_about_mean_m_s2": rms,
                           "max_gyro_rad_s": gyro_max, "direction_p95_deg": direction_p95},
            "rejection_reasons": reasons, "sample_provenance": data.get("sample_provenance"),
            "limits": ["Static-like statistics do not prove standstill or absence of constant translational acceleration.",
                       "Gravity cannot observe yaw; this estimates native IMU tilt only, not absolute position.",
                       "X-axis alignment alone leaves rotation about X and translation unknown; no T_left_imu identity assumed.",
                       "Left IMU observations cannot determine right lidar tilt or full left-right extrinsics.",
                       "Thresholds are persisted experimental starting values, not an accuracy acceptance certificate."]}


def h30_bag_samples(selection, *, records=None):
    """Read a bounded whitelisted H30Frame bag window. No publishers or hardware I/O."""
    _metadata(selection)
    if selection.get("acceleration_units") != "m/s^2" or selection.get("angular_velocity_units") != "rad/s":
        raise CalibrationError("H30Frame ROS fields require declared SI m/s^2 and rad/s")
    start, end = selection.get("start_ns"), selection.get("end_ns")
    if type(start) is not int or type(end) is not int or not 0 <= start < end:
        raise CalibrationError("explicit bounded integer start_ns/end_ns bag window required")
    policy = GravityPolicy(**selection.get("policy", {}))
    if records is None:
        records = _iter_h30_bag(selection)
    samples, keys, epochs = [], set(), set()
    for stamp, message in records:
        if not start <= stamp <= end:
            continue
        if message.sensor_id != selection["sensor_id"] or message.coordinate_convention != selection["coordinate_convention"]:
            raise CalibrationError("H30 sensor identity or native coordinate convention mismatch")
        if not message.linear_acceleration_valid or not message.angular_velocity_valid:
            raise CalibrationError("selected H30 window contains unavailable acceleration/gyro")
        key = (message.sensor_id, message.stream_epoch, int(message.frame_sequence))
        if key in keys:
            raise CalibrationError("duplicate H30 source identity key")
        keys.add(key)
        epochs.add(message.stream_epoch)
        if len(epochs) != 1 or not message.stream_epoch:
            raise CalibrationError("gravity window must remain within one nonempty H30 stream epoch")
        a, w = message.imu.linear_acceleration, message.imu.angular_velocity
        samples.append({"time_ns": int(stamp), "acceleration": [a.x, a.y, a.z],
                        "angular_velocity": [w.x, w.y, w.z], "raw_key": list(key)})
        if len(samples) > policy.max_samples:
            raise CalibrationError("bounded H30 bag sample budget exceeded")
    output = dict(selection)
    output["samples"] = samples
    output["sample_provenance"] = {"kind": "rosbag2_H30Frame", "bag_uri": selection.get("bag_uri"),
                                   "topic": selection.get("topic"), "window_ns": [start, end],
                                   "stream_epochs": sorted(epochs), "timestamp_basis": "bag_record_time_not_validated_measurement_time"}
    return output


def _iter_h30_bag(selection):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except (ImportError, OSError) as error:
        raise CalibrationError("ROS bag support unavailable; source the verified target Humble environment") from error
    topic = selection.get("topic")
    if not isinstance(topic, str) or not topic.startswith("/"):
        raise CalibrationError("one explicit absolute H30Frame topic required")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(selection["bag_uri"]), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    available = {item.name: item for item in reader.get_all_topics_and_types()}
    if topic not in available or available[topic].type != "wc_interfaces/msg/H30Frame" or getattr(available[topic], "serialization_format", "cdr") != "cdr":
        raise CalibrationError("selected topic must be an authoritative CDR H30Frame")
    message_type = get_message("wc_interfaces/msg/H30Frame")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    reader.seek(selection["start_ns"])
    while reader.has_next():
        received_topic, serialized, stamp = reader.read_next()
        if received_topic != topic:
            raise CalibrationError("rosbag filter returned a nonwhitelisted topic")
        if stamp > selection["end_ns"]:
            break
        if stamp >= selection["start_ns"]:
            yield int(stamp), deserialize_message(serialized, message_type)
