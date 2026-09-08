import json

import numpy as np
import pytest
import yaml

from smartwheel_map_products.occupancy import raycast_occupancy
from smartwheel_map_products.writers import export_map_bundle, publish_latest_path


def _inputs(tmp_path):
    points = np.array([[1.0, 0.0, 0.5], [0.0, 1.0, 0.7], [1.0, 1.0, 1.2]])
    origins = np.zeros_like(points)
    grid = raycast_occupancy(points, origins, resolution=0.1)
    profile = tmp_path / "profile.yaml"
    profile.write_text("profile:\n  mode: mock\n", encoding="utf-8")
    return points, grid, profile


def test_map_bundle_contains_nonempty_products_and_nested_quality(tmp_path):
    points, grid, profile = _inputs(tmp_path)
    output = tmp_path / "map"
    export_map_bundle(
        output,
        points,
        grid,
        [(1.0, 0.0, 0.0, 0.0), (2.0, 0.1, 0.0, 0.0)],
        np.full((3, 3), 127, dtype=np.uint8),
        str(profile),
        {"backend": "test"},
        "/tmp/smartwheel_experiment/rosbag",
        {
            "stage": "RIGHT_LIDAR_STAGE1_PRELIMINARY",
            "hardware_validated": False,
            "point_count": 3,
            "trajectory_evaluation": {
                "schema_version": 1,
                "endpoint_displacement_m": 0.1,
                "loop_closure": {
                    "status": "UNAVAILABLE",
                    "position_error_m": None,
                    "source": None,
                },
            },
        },
        intensity_points=points,
        intensity=np.array([100.0, 200.0, 300.0], dtype=np.float32),
    )
    expected = (
        "map_geometry.pcd",
        "map_geometry.ply",
        "map_colored.ply",
        "map_pointcloud_amp.pcd",
        "map_pointcloud_amp.ply",
        "map_2d.pgm",
        "map_2d.png",
        "map_2d.yaml",
        "trajectory.tum",
        "poses.csv",
        "hardware_profile_used.yaml",
        "algorithm_profile_used.yaml",
        "quality_report.json",
        "quality_report.md",
        "manifest.json",
        "bag_path.txt",
    )
    for name in expected:
        assert (output / name).stat().st_size > 0
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["format_version"] == 1
    assert manifest["complete"] is True
    assert manifest["stage"] == "RIGHT_LIDAR_STAGE1_PRELIMINARY"
    assert manifest["externally_managed_files"] == []
    assert (output / "bag_path.txt").read_text(encoding="utf-8").strip() == (
        "/tmp/smartwheel_experiment/rosbag"
    )
    bag_manifest = next(item for item in manifest["files"] if item["path"] == "bag_path.txt")
    assert bag_manifest["bytes"] == (output / "bag_path.txt").stat().st_size
    map_yaml = yaml.safe_load((output / "map_2d.yaml").read_text(encoding="utf-8"))
    assert map_yaml["resolution"] == 0.1
    assert "FIELDS x y z intensity" in (
        output / "map_pointcloud_amp.pcd"
    ).read_text(encoding="ascii")
    assert "property float intensity" in (
        output / "map_pointcloud_amp.ply"
    ).read_text(encoding="ascii")

    markdown = (output / "quality_report.md").read_text(encoding="utf-8")
    assert "`trajectory_evaluation.endpoint_displacement_m`: `0.1`" in markdown
    assert "`trajectory_evaluation.loop_closure.status`: `UNAVAILABLE`" in markdown
    assert (
        "`trajectory_evaluation.loop_closure.position_error_m`: `UNAVAILABLE`"
        in markdown
    )
    assert "None" not in markdown


def test_nonfinite_quality_is_rejected_before_manifest_is_created(tmp_path):
    points, grid, profile = _inputs(tmp_path)
    output = tmp_path / "invalid-map"
    with pytest.raises(ValueError, match="Out of range float values"):
        export_map_bundle(
            output,
            points,
            grid,
            [(1.0, 0.0, 0.0, 0.0), (2.0, 0.1, 0.0, 0.0)],
            None,
            str(profile),
            {"backend": "test"},
            "",
            {"stage": "TEST", "nonfinite": float("nan")},
        )
    assert not (output / "manifest.json").exists()
    assert not (output / "quality_report.json").exists()


def test_writer_failure_leaves_incomplete_marker_and_no_manifest(tmp_path, monkeypatch):
    points, grid, profile = _inputs(tmp_path)
    output = tmp_path / "failed-map"

    def fail_after_first_product(*_args, **_kwargs):
        raise OSError("synthetic disk failure")

    monkeypatch.setattr("smartwheel_map_products.writers.write_ply", fail_after_first_product)
    with pytest.raises(OSError, match="synthetic disk failure"):
        export_map_bundle(
            output,
            points,
            grid,
            [(1.0, 0.0, 0.0, 0.0)],
            None,
            str(profile),
            {"backend": "test"},
            "",
            {"stage": "TEST"},
        )
    assert (output / ".incomplete").is_file()
    assert not (output / "manifest.json").exists()


def test_successful_writer_removes_incomplete_marker(tmp_path):
    points, grid, profile = _inputs(tmp_path)
    output = tmp_path / "complete-map"
    export_map_bundle(
        output,
        points,
        grid,
        [(1.0, 0.0, 0.0, 0.0)],
        None,
        str(profile),
        {"backend": "test"},
        "",
        {"stage": "TEST"},
    )
    assert not (output / ".incomplete").exists()
    assert (output / "manifest.json").is_file()


def test_latest_path_is_published_atomically(tmp_path):
    output = tmp_path / "versions" / "map_1"
    pointer = publish_latest_path(tmp_path / "versions", output)
    assert pointer.read_text(encoding="utf-8") == str(output.resolve()) + "\n"
    assert not list(pointer.parent.glob(".latest_path.txt.*.tmp"))


def test_writer_preserves_preexisting_external_rtabmap_database(tmp_path):
    points, grid, profile = _inputs(tmp_path)
    output = tmp_path / "rtabmap-map"
    output.mkdir()
    database = output / "rtabmap.db"
    database.write_bytes(b"external database")
    export_map_bundle(
        output,
        points,
        grid,
        [(1.0, 0.0, 0.0, 0.0)],
        None,
        str(profile),
        {"backend": "test"},
        "",
        {"stage": "TEST"},
    )
    assert database.read_bytes() == b"external database"
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["externally_managed_files"] == ["rtabmap.db"]
