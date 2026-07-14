import json

from smartwheel_mapping_manager.quality_gate import REQUIRED_EXPORTS, validate_quality_bundle


def make_bundle(tmp_path, geometry_source="backend:/rtabmap/cloud_map"):
    for name in REQUIRED_EXPORTS:
        (tmp_path / name).write_text("x\n", encoding="utf-8")
    (tmp_path / "manifest.json").write_text(json.dumps({"complete": True}), encoding="utf-8")
    (tmp_path / "quality_report.json").write_text(
        json.dumps(
            {
                "point_count": 2000,
                "trajectory_pose_count": 100,
                "occupied_cells": 10,
                "free_cells": 20,
                "unknown_cells": 30,
                "trajectory_ground_truth_rmse_m": 0.2,
                "geometry_source": geometry_source,
            }
        ),
        encoding="utf-8",
    )


def test_quality_gate_requires_measured_products(tmp_path):
    make_bundle(tmp_path)
    checks = validate_quality_bundle(tmp_path, "rtabmap", 1000, 50, 0.5)
    assert all(checks.values())


def test_quality_gate_rejects_local_geometry_for_rtabmap(tmp_path):
    make_bundle(tmp_path, geometry_source="local_odometry_accumulator")
    checks = validate_quality_bundle(tmp_path, "rtabmap", 1000, 50, 0.5)
    assert not checks["geometry_source_valid"]
