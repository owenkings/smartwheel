"""Offline gravity leveling about the left lidar origin; no device or TF access.

Acceleration is SI specific force (upward while stationary), in native IMU
axes. R_left_imu maps native IMU vectors to left lidar vectors. A supplied
mounting rotation remains a candidate; stable samples do not validate it.
"""

from dataclasses import asdict, dataclass
import math
from numbers import Real

import numpy as np

from .core import CalibrationError


@dataclass(frozen=True)
class GravityLevelPolicy:
    """Explicit experimental quality gates, not an accuracy certificate."""

    min_samples: int = 100
    max_samples: int = 100000
    gravity_m_s2: float = 9.80665
    max_norm_error_m_s2: float = 1.0
    max_accel_rms_m_s2: float = 0.25
    max_gyro_rad_s: float = 0.05
    max_direction_p95_deg: float = 3.0
    min_forward_projection_norm: float = 1e-6

    def __post_init__(self):
        if type(self.min_samples) is not int or type(self.max_samples) is not int or \
                not 3 <= self.min_samples <= self.max_samples <= 1000000:
            raise CalibrationError("gravity leveling sample bounds must be integers in [3, 1000000]")
        for name, value in asdict(self).items():
            if name in {"min_samples", "max_samples"}:
                continue
            if isinstance(value, bool) or not isinstance(value, Real) or \
                    not math.isfinite(value) or value <= 0:
                raise CalibrationError("gravity leveling policy limits must be finite positive numbers")
        if self.min_forward_projection_norm >= 1 or self.max_direction_p95_deg > 180:
            raise CalibrationError("gravity leveling angular policy limits are out of range")


def _array(value, name):
    try:
        array = np.asarray(value)
        if array.dtype.kind not in "iuf":
            raise CalibrationError(name + " must contain real numeric values")
        array = array.astype(float, copy=True)
    except (TypeError, ValueError, OverflowError) as error:
        if isinstance(error, CalibrationError):
            raise
        raise CalibrationError(name + " must contain real numeric values") from error
    if not np.isfinite(array).all():
        raise CalibrationError(name + " must contain only finite values")
    return array


def _samples(value, name, policy):
    array = _array(value, name)
    if array.ndim != 2 or array.shape[1] != 3 or \
            not policy.min_samples <= len(array) <= policy.max_samples:
        raise CalibrationError(name + " must be Nx3 within the declared sample bounds")
    return array


def estimate_gravity_level(acceleration, R_left_imu, *, angular_velocity=None, policy=None):
    """Return a JSON-ready pure-rotation candidate or raise CalibrationError.

    Inputs are Nx3 m/s^2 specific force and a proper 3x3 mounting rotation.
    Optional angular_velocity is the same samples' Nx3 native-IMU rad/s data.
    The output frame has +Z upward and +X along the horizontal projection of
    the left lidar's +X. Its origin remains the left lidar origin, not ground.
    No absolute heading, lever arm, timing validation, or full extrinsic is
    inferred. A static scene must be established by the caller; in particular,
    these checks cannot distinguish gravity from constant linear acceleration.
    """
    if policy is None:
        policy = GravityLevelPolicy()
    elif isinstance(policy, dict):
        try:
            policy = GravityLevelPolicy(**policy)
        except TypeError as error:
            raise CalibrationError("invalid gravity leveling policy fields") from error
    elif not isinstance(policy, GravityLevelPolicy):
        raise CalibrationError("policy must be GravityLevelPolicy or a dictionary")

    acceleration = _samples(acceleration, "acceleration", policy)
    mount = _array(R_left_imu, "R_left_imu")
    if mount.shape != (3, 3) or not np.allclose(mount.T @ mount, np.eye(3), atol=1e-6, rtol=0) or \
            not math.isclose(float(np.linalg.det(mount)), 1.0, abs_tol=1e-6, rel_tol=0):
        raise CalibrationError("R_left_imu must be a proper 3x3 rotation; no reflection or scale")
    gyro = None
    if angular_velocity is not None:
        gyro = _samples(angular_velocity, "angular_velocity", policy)
        if len(gyro) != len(acceleration):
            raise CalibrationError("angular_velocity must correspond to every acceleration sample")

    with np.errstate(over="ignore", invalid="ignore"):
        magnitudes = np.linalg.norm(acceleration, axis=1)
        mean = acceleration.mean(axis=0)
        mean_norm = float(np.linalg.norm(mean))
        accel_rms = float(np.sqrt(np.mean(np.sum((acceleration - mean) ** 2, axis=1))))
        max_gyro = None if gyro is None else float(np.max(np.linalg.norm(gyro, axis=1)))
    finite_statistics = [mean_norm, accel_rms, *(magnitudes.tolist()), *([] if max_gyro is None else [max_gyro])]
    if not all(math.isfinite(value) for value in finite_statistics):
        raise CalibrationError("nonfinite gravity statistics; input magnitudes exceed numerical bounds")
    if mean_norm < 1e-6 or np.any(magnitudes < 1e-6):
        raise CalibrationError("zero/near-zero acceleration cannot define gravity")

    up_imu = mean / mean_norm
    angles = np.degrees(np.arccos(np.clip((acceleration / magnitudes[:, None]) @ up_imu, -1., 1.)))
    direction_p95 = float(np.quantile(angles, 0.95))
    norm_error = float(np.max(np.abs(magnitudes - policy.gravity_m_s2)))
    failures = []
    for observed, limit, reason in (
        (norm_error, policy.max_norm_error_m_s2, "ACCELERATION_NOT_NEAR_GRAVITY"),
        (accel_rms, policy.max_accel_rms_m_s2, "ACCELERATION_DISPERSION_TOO_HIGH"),
        (direction_p95, policy.max_direction_p95_deg, "GRAVITY_DIRECTION_UNSTABLE"),
    ):
        if observed > limit:
            failures.append(reason)
    if max_gyro is not None and max_gyro > policy.max_gyro_rad_s:
        failures.append("ANGULAR_MOTION_OR_GYRO_BIAS_TOO_HIGH")
    if failures:
        raise CalibrationError("gravity leveling rejected: " + ", ".join(failures))

    up_left = mount @ up_imu
    up_left /= np.linalg.norm(up_left)
    forward_left = np.array([1., 0., 0.])
    horizontal_forward = forward_left - np.dot(forward_left, up_left) * up_left
    forward_norm = float(np.linalg.norm(horizontal_forward))
    if forward_norm < policy.min_forward_projection_norm:
        raise CalibrationError("LEFT_FORWARD_HORIZONTAL_PROJECTION_UNOBSERVABLE")
    x_level_in_left = horizontal_forward / forward_norm
    y_level_in_left = np.cross(up_left, x_level_in_left)
    y_level_in_left /= np.linalg.norm(y_level_in_left)
    # Re-orthogonalize after finite-precision projection; rows are level axes in left.
    x_level_in_left = np.cross(y_level_in_left, up_left)
    level = np.stack((x_level_in_left, y_level_in_left, up_left))

    return {
        "schema_version": 1,
        "kind": "left_lidar_gravity_level_candidate",
        "status": "CANDIDATE",
        "live_eligible": False,
        "independent_validation_performed": False,
        "mounting_rotation_validated": False,
        "time_model_validated": False,
        "R_left_imu_candidate": mount.tolist(),
        "R_level_left": level.tolist(),
        "up_unit_in_imu": up_imu.tolist(),
        "up_unit_in_left": up_left.tolist(),
        "origin": "left_lidar_origin_unchanged",
        "transform_definition": "p_level = R_level_left * p_left; pure rotation about left lidar origin",
        "horizontal_heading_definition": "+X is the horizontal projection of left lidar +X; not absolute heading",
        "acceleration_sign": "specific_force",
        "acceleration_includes_gravity": True,
        "units": {"acceleration": "m/s^2", "angular_velocity": "rad/s"},
        "policy": asdict(policy),
        "sample_count": len(acceleration),
        "statistics": {
            "mean_acceleration_m_s2": mean.tolist(),
            "mean_norm_m_s2": mean_norm,
            "max_norm_error_m_s2": norm_error,
            "acceleration_rms_about_mean_m_s2": accel_rms,
            "direction_p95_deg": direction_p95,
            "angular_velocity_available": gyro is not None,
            "max_gyro_rad_s": max_gyro,
            "left_forward_horizontal_projection_norm": forward_norm,
        },
        "limits": [
            "Caller must establish a static recording; these statistics cannot rule out constant linear acceleration.",
            "Sample count does not establish observation duration or synchronization; no timestamp inference is made.",
            "Gyroscope motion is checked only when corresponding angular_velocity samples are supplied.",
            "Supplied IMU-to-left rotation is a candidate; its axes and mounting accuracy are not validated here.",
            "Gravity does not determine absolute yaw, ground height, IMU translation, or left-right extrinsics.",
            "Quality gates are explicit experimental starting values, not a sensor accuracy certificate.",
        ],
    }
