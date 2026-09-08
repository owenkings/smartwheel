import hashlib
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


def test_quality_gate_rejects_incomplete_session_metadata(tmp_path):
    make_bundle(tmp_path)
    quality_path = tmp_path / "quality_report.json"
    quality = json.loads(quality_path.read_text(encoding="utf-8"))
    quality["session"] = {
        "state": "CAPTURING",
        "complete": False,
        "failure_reason": "",
        "frame_ids": ["camera_init"],
        "frame_count": 3,
        "raw_point_count": 30,
        "first_cloud_stamp": 1.0,
        "last_cloud_stamp": 2.0,
        "cloud_time_span_sec": 1.0,
        "rejected_frame_count": 0,
    }
    quality_path.write_text(json.dumps(quality), encoding="utf-8")
    checks = validate_quality_bundle(tmp_path, "rtabmap", 1000, 50, 0.5)
    assert checks["session_complete"] is False


def test_quality_gate_accepts_stopped_session_with_provenance(tmp_path):
    make_bundle(tmp_path)
    quality_path = tmp_path / "quality_report.json"
    quality = json.loads(quality_path.read_text(encoding="utf-8"))
    quality["session"] = {
        "state": "STOPPED",
        "complete": True,
        "failure_reason": "",
        "frame_ids": ["camera_init"],
        "frame_count": 3,
        "raw_point_count": 30,
        "first_cloud_stamp": 1.0,
        "last_cloud_stamp": 2.0,
        "cloud_time_span_sec": 1.0,
        "rejected_frame_count": 0,
    }
    quality_path.write_text(json.dumps(quality), encoding="utf-8")
    checks = validate_quality_bundle(tmp_path, "rtabmap", 1000, 50, 0.5)
    assert checks["session_complete"] is True


def test_quality_gate_rejects_in_progress_marker(tmp_path):
    make_bundle(tmp_path)
    (tmp_path / ".incomplete").write_text("in progress\n", encoding="ascii")
    checks = validate_quality_bundle(tmp_path, "rtabmap", 1000, 50, 0.5)
    assert checks["manifest_complete"] is False


def test_quality_gate_detects_manifest_file_tampering(tmp_path):
    make_bundle(tmp_path)
    path = tmp_path / "map_geometry.pcd"
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    (tmp_path / "manifest.json").write_text(
        json.dumps(
            {
                "complete": True,
                "files": [
                    {
                        "path": "map_geometry.pcd",
                        "bytes": path.stat().st_size,
                        "sha256": digest,
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    checks = validate_quality_bundle(tmp_path, "rtabmap", 1000, 50, 0.5)
    assert checks["manifest_integrity"] is True
    path.write_text("tampered\n", encoding="utf-8")
    checks = validate_quality_bundle(tmp_path, "rtabmap", 1000, 50, 0.5)
    assert checks["manifest_integrity"] is False
