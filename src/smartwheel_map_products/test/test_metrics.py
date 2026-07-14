import math

import pytest

from smartwheel_map_products.metrics import evaluate_trajectory


def test_trajectory_metrics_remove_only_initial_se2_offset():
    truth = [
        (0.0, 10.0, 5.0, 0.5),
        (1.0, 10.0 + math.cos(0.5), 5.0 + math.sin(0.5), 0.5),
        (2.0, 10.0 + 2.0 * math.cos(0.5), 5.0 + 2.0 * math.sin(0.5), 0.5),
    ]
    estimate = [(0.0, 0.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0), (2.0, 2.0, 0.0, 0.0)]
    metrics = evaluate_trajectory(estimate, truth)
    assert metrics["trajectory_alignment"] == "SE2_FIRST_POSE"
    assert metrics["trajectory_ground_truth_rmse_m"] == pytest.approx(0.0)
    assert metrics["trajectory_ground_truth_yaw_rmse_rad"] == pytest.approx(0.0)


def test_trajectory_metrics_report_drift_and_reject_stale_matches():
    truth = [(0.0, 0.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0), (2.0, 2.0, 0.0, 0.0)]
    estimate = [(0.0, 0.0, 0.0, 0.0), (1.0, 1.2, 0.0, 0.1), (2.4, 2.8, 0.0, 0.2)]
    metrics = evaluate_trajectory(estimate, truth, max_time_delta_sec=0.15)
    assert metrics["trajectory_matches"] == 2
    assert metrics["trajectory_ground_truth_rmse_m"] > 0.0
    assert metrics["trajectory_ground_truth_yaw_rmse_rad"] > 0.0
    assert math.isfinite(metrics["endpoint_position_error_m"])
