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
        "formal_acceptance.json",
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


def test_hardware_validated_writer_rechecks_copied_provenance(tmp_path, monkeypatch):
    points, grid, profile = _inputs(tmp_path)
    contract = tmp_path / "contract.json"
    contract.write_text('{"status":"APPROVED"}\n', encoding="utf-8")
    output = tmp_path / "rejected-formal-map"
    calls = []

    def reject_copied_snapshot(evidence, **paths):
        calls.append((evidence, paths))
        assert paths["hardware_profile_path"].is_file()
        assert paths["calibration_contract_path"].is_file()
        return False, "synthetic copied-snapshot mismatch"

    monkeypatch.setattr(
        "smartwheel_map_products.writers.validate_hardware_evidence_binding",
        reject_copied_snapshot,
    )
    with pytest.raises(ValueError, match="copied-snapshot mismatch"):
        export_map_bundle(
            output,
            points,
            grid,
            [(1.0, 0.0, 0.0, 0.0)],
            None,
            str(profile),
            {"backend": "test"},
            "",
            {
                "stage": "TEST",
                "hardware_validated": True,
                "formal_acceptance": {"schema_version": 1},
            },
            calibration_contract_path=str(contract),
        )
    assert len(calls) == 1
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


def test_writer_preserves_full_6dof_optimized_trajectory(tmp_path):
    points, grid, profile = _inputs(tmp_path)
    output = tmp_path / "full-pose-map"
    # 90 degrees around X retains non-planar orientation and non-zero height.
    half = np.sqrt(0.5)
    optimized = [
        (1.0, 1.0, 2.0, 0.4, 0.0, 0.0, 0.0, 1.0),
        (2.0, 1.5, 2.5, 0.6, half, 0.0, 0.0, half),
    ]
    export_map_bundle(
        output,
        points,
        grid,
        [(1.0, 1.0, 2.0, 0.0), (2.0, 1.5, 2.5, 0.0)],
        None,
        str(profile),
        {"backend": "test"},
        "",
        {"stage": "TEST"},
        trajectory_poses=optimized,
    )
    tum = (output / "trajectory.tum").read_text(encoding="ascii").splitlines()
    assert tum[1].split()[3] == "0.600000"
    assert float(tum[1].split()[4]) == pytest.approx(half, abs=1.0e-8)
    rows = (output / "poses.csv").read_text(encoding="ascii").splitlines()
    assert rows[2].split(",")[3] == "0.600000"
    assert float(rows[2].split(",")[4]) == pytest.approx(np.pi / 2.0, abs=1.0e-8)


def test_writer_manifest_covers_calibration_contract(tmp_path):
    points, grid, profile = _inputs(tmp_path)
    contract = tmp_path / "contract.json"
    contract.write_text('{"schema_version": 1, "status": "APPROVED"}\n', encoding="utf-8")
    output = tmp_path / "contract-map"
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
        calibration_contract_path=str(contract),
    )
    assert (output / "calibration_contract_used.json").read_bytes() == contract.read_bytes()
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert "calibration_contract_used.json" in {entry["path"] for entry in manifest["files"]}


def test_writer_rejects_nonfinite_geometry_and_nonmonotonic_trajectory(tmp_path):
    points, grid, profile = _inputs(tmp_path)
    invalid = points.copy()
    invalid[0, 0] = np.nan
    with pytest.raises(ValueError, match="points must be finite"):
        export_map_bundle(
            tmp_path / "nonfinite",
            invalid,
            grid,
            [(1.0, 0.0, 0.0, 0.0)],
            None,
            str(profile),
            {"backend": "test"},
            "",
            {"stage": "TEST"},
        )

    with pytest.raises(ValueError, match="strictly increasing"):
        export_map_bundle(
            tmp_path / "bad-trajectory",
            points,
            grid,
            [(2.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)],
            None,
            str(profile),
            {"backend": "test"},
            "",
            {"stage": "TEST"},
        )


def test_writer_rejects_preexisting_symlink(tmp_path):
    points, grid, profile = _inputs(tmp_path)
    output = tmp_path / "linked-map"
    output.mkdir()
    target = tmp_path / "outside.txt"
    target.write_text("outside", encoding="utf-8")
    try:
        (output / "linked.txt").symlink_to(target)
    except (OSError, NotImplementedError):
        pytest.skip("symlinks are unavailable on this platform")
    with pytest.raises(ValueError, match="symlink"):
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
