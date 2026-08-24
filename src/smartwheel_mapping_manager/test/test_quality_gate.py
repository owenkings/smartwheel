import json

import pytest

from smartwheel_mapping_manager.quality_gate import REQUIRED_EXPORTS, validate_quality_bundle


def make_bundle(tmp_path, geometry_source="backend:/rtabmap/cloud_map", rmse=0.2):
    for name in REQUIRED_EXPORTS:
        (tmp_path / name).write_text("x\n", encoding="utf-8")
    (tmp_path / "manifest.json").write_text(
        json.dumps({"complete": True}), encoding="utf-8"
    )
    (tmp_path / "quality_report.json").write_text(
        json.dumps(
            {
                "point_count": 2000,
                "trajectory_pose_count": 100,
                "occupied_cells": 10,
                "free_cells": 20,
                "unknown_cells": 30,
                "trajectory_ground_truth_rmse_m": rmse,
                "geometry_source": geometry_source,
            }
        ),
        encoding="utf-8",
    )


def test_quality_gate_requires_measured_products(tmp_path):
    make_bundle(tmp_path)
    checks = validate_quality_bundle(tmp_path, "rtabmap", 1000, 50, 0.5)
    assert all(checks.values())


def test_quality_gate_accepts_zero_ground_truth_rmse(tmp_path):
    make_bundle(tmp_path, rmse=0.0)
    checks = validate_quality_bundle(tmp_path, "rtabmap", 1000, 50, 0.5)
    assert checks["trajectory_rmse_available"]
    assert checks["trajectory_rmse_within_limit"]


def test_quality_gate_rejects_local_geometry_for_rtabmap(tmp_path):
    make_bundle(tmp_path, geometry_source="local_odometry_accumulator")
    checks = validate_quality_bundle(tmp_path, "rtabmap", 1000, 50, 0.5)
    assert not checks["geometry_source_valid"]


@pytest.mark.parametrize(
    "rmse",
    (True, False, -0.01, float("inf"), float("-inf"), float("nan"), "0.2"),
)
def test_quality_gate_rejects_non_measured_ground_truth_rmse(tmp_path, rmse):
    make_bundle(tmp_path, rmse=rmse)
    checks = validate_quality_bundle(tmp_path, "rtabmap", 1000, 50, 0.5)
    assert not checks["trajectory_rmse_available"]
    assert not checks["trajectory_rmse_within_limit"]


def test_quality_gate_does_not_fallback_to_legacy_or_endpoint_metrics(tmp_path):
    make_bundle(tmp_path, rmse=None)
    quality_path = tmp_path / "quality_report.json"
    quality = json.loads(quality_path.read_text(encoding="utf-8"))
    quality.update(
        {
            "trajectory_rmse_m": 0.0,
            "endpoint_position_error_m": 0.0,
            "loop_closure_position_error_m": 0.0,
            "trajectory_evaluation": {
                "schema_version": 1,
                "endpoint_displacement_m": 0.0,
                "loop_closure": {
                    "status": "MEASURED",
                    "position_error_m": 0.0,
                    "source": "synthetic",
                },
            },
        }
    )
    quality_path.write_text(json.dumps(quality), encoding="utf-8")
    checks = validate_quality_bundle(tmp_path, "rtabmap", 1000, 50, 0.5)
    assert not checks["trajectory_rmse_available"]
    assert not checks["trajectory_rmse_within_limit"]
