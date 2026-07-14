import math

import numpy as np


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def evaluate_trajectory(
    estimated: list[tuple[float, float, float, float]],
    ground_truth: list[tuple[float, float, float, float]],
    max_time_delta_sec: float = 0.1,
) -> dict:
    if max_time_delta_sec <= 0.0:
        raise ValueError("max_time_delta_sec must be positive")
    unavailable = {
        "trajectory_matches": 0,
        "trajectory_alignment": "UNAVAILABLE",
        "trajectory_ground_truth_rmse_m": None,
        "trajectory_ground_truth_yaw_rmse_rad": None,
        "endpoint_position_error_m": None,
        "endpoint_yaw_error_rad": None,
    }
    if not estimated or not ground_truth:
        return unavailable
    matches = []
    for pose in estimated:
        truth = min(ground_truth, key=lambda candidate: abs(candidate[0] - pose[0]))
        if abs(truth[0] - pose[0]) <= max_time_delta_sec:
            matches.append((pose, truth))
    if not matches:
        return unavailable
    first_estimate, first_truth = matches[0]
    yaw_offset = _wrap(first_truth[3] - first_estimate[3])
    cosine = math.cos(yaw_offset)
    sine = math.sin(yaw_offset)
    aligned = []
    truth_values = []
    for estimate, truth in matches:
        dx = estimate[1] - first_estimate[1]
        dy = estimate[2] - first_estimate[2]
        x = first_truth[1] + cosine * dx - sine * dy
        y = first_truth[2] + sine * dx + cosine * dy
        yaw = _wrap(estimate[3] + yaw_offset)
        aligned.append((x, y, yaw))
        truth_values.append((truth[1], truth[2], truth[3]))
    aligned_array = np.asarray(aligned, dtype=np.float64)
    truth_array = np.asarray(truth_values, dtype=np.float64)
    position_errors = np.linalg.norm(aligned_array[:, :2] - truth_array[:, :2], axis=1)
    yaw_errors = np.asarray(
        [_wrap(value) for value in aligned_array[:, 2] - truth_array[:, 2]], dtype=np.float64
    )
    return {
        "trajectory_matches": len(matches),
        "trajectory_alignment": "SE2_FIRST_POSE",
        "trajectory_ground_truth_rmse_m": float(np.sqrt(np.mean(position_errors**2))),
        "trajectory_ground_truth_yaw_rmse_rad": float(np.sqrt(np.mean(yaw_errors**2))),
        "endpoint_position_error_m": float(position_errors[-1]),
        "endpoint_yaw_error_rad": float(abs(yaw_errors[-1])),
    }
