"""Shared physical measurement preparation; no device, ROS or guessed transform."""
import numpy as np


def vector(value, size=3):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError('finite measured vector required')
    return result


def rotation(value):
    if value is None:
        raise ValueError('MISSING_REQUIRED_IMU_ROTATION')
    result = np.asarray(value, dtype=float)
    if result.shape != (3, 3) or not np.isfinite(result).all() or not np.allclose(
            result.T @ result, np.eye(3), atol=1e-6, rtol=0) or not np.isclose(
            np.linalg.det(result), 1., atol=1e-6, rtol=0):
        raise ValueError('IMU installation must be an explicit proper rotation')
    return result


def prepare_gyro(gyro_native, bias_native, R_target_imu, covariance_native=None):
    """Subtract native-axis bias once, then rotate value and covariance.

    A declared rotation is required even when it is explicitly measured identity.
    None bias means an unestimated zero candidate, not a confirmed calibration.
    """
    R = rotation(R_target_imu)
    bias = np.zeros(3) if bias_native is None else vector(bias_native)
    corrected = vector(gyro_native)-bias
    result = {'native_raw_rad_s': vector(gyro_native).tolist(),
              'bias_native_rad_s': bias.tolist(), 'native_corrected_rad_s': corrected.tolist(),
              'target_rad_s': (R @ corrected).tolist(),
              'bias_status': 'UNESTIMATED_ZERO' if bias_native is None else 'EXPLICIT_NATIVE_BIAS',
              'operation': 'subtract_native_bias_then_rotate', 'R_target_imu': R.tolist()}
    if covariance_native is not None:
        C = np.asarray(covariance_native, dtype=float)
        if C.shape != (3, 3) or not np.isfinite(C).all() or not np.allclose(C, C.T) or np.linalg.eigvalsh(C).min() < 0:
            raise ValueError('finite symmetric PSD native covariance required')
        result['target_covariance'] = (R @ C @ R.T).tolist()
    return result


def gyro_z(gyro_native, bias_native, R_target_imu):
    return prepare_gyro(gyro_native, bias_native, R_target_imu)['target_rad_s'][2]
