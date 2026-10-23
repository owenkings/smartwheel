"""Small saved-result fixtures: failed jobs must not appear as comparable maps."""
import json
from pathlib import Path

import pytest

from wc_panel.catalog import results


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False), encoding="utf-8")


def saved_cell(batch, run, cell):
    front = batch / run / cell
    export = batch / run / "native" / cell / "export"
    front.mkdir(parents=True)
    export.mkdir(parents=True)
    trajectory = front / "trajectory.csv"
    image = export / "map_3d.pgm"
    cloud = export / "map_3d_cloud.ply.xz"
    trajectory.write_text("stamp_ns,x_m,y_m\n1,0,0\n2,1,0\n", encoding="utf-8")
    image.write_bytes(b"P5\n1 1\n255\n\xff")
    # Discovery only identifies the retained cloud; decoding is tested elsewhere.
    cloud.write_bytes(b"compressed cloud discovery fixture")
    return front, trajectory, image, cloud


@pytest.mark.parametrize("status", ["FAILED", "CANCELLED", "INTERRUPTED", "SKIPPED"])
def test_empty_failed_batch_is_not_a_map(tmp_path, status):
    batch = tmp_path / "zoulang_failed_batch"
    marker = batch / "panel_result.json"
    write_json(marker, {"status": status, "error": "synthetic unsuccessful attempt"})

    assert results(tmp_path) == []
    assert results(batch) == []
    assert json.loads(marker.read_text(encoding="utf-8"))["status"] == status


@pytest.mark.parametrize("status", ["FAILED", "CANCELLED", "INTERRUPTED", "SKIPPED"])
def test_failed_diagnostic_images_and_routes_are_not_comparable_maps(tmp_path, status):
    batch = tmp_path / "zoulang_failed_diagnostics"
    write_json(batch / "panel_result.json", {"status": status, "dataset": "/recordings/zoulang"})
    route = batch / "diagnostics" / "prior_trajectory.csv"
    route.parent.mkdir()
    route.write_text("x_m,y_m\n0,0\n1,1\n", encoding="utf-8")
    image = route.parent / "diagnostic_grid.pgm"
    image.write_bytes(b"P5\n1 1\n255\n\xff")

    assert results(tmp_path) == []
    assert results(batch) == []
    # Failed evidence stays on disk for the task/log browser.
    assert route.read_text(encoding="utf-8") == "x_m,y_m\n0,0\n1,1\n"
    assert image.read_bytes() == b"P5\n1 1\n255\n\xff"


def test_unindexed_legacy_saved_map_remains_discoverable(tmp_path):
    saved = tmp_path / "legacy_corridor_map"
    saved.mkdir()
    image = saved / "map_3d.pgm"
    cloud = saved / "map_3d_cloud.ply"
    image.write_bytes(b"P5\n1 1\n255\n\xff")
    cloud.write_bytes(b"ply\nformat ascii 1.0\nelement vertex 0\nend_header\n")

    rows = results(tmp_path)

    assert len(rows) == 1
    assert rows[0]["path"] == str(saved)
    assert rows[0]["status"] == "AVAILABLE"
    assert rows[0]["images"] == [str(image)]
    assert rows[0]["clouds"] == [str(cloud)]


def test_failed_batch_keeps_explicitly_successful_cell_only(tmp_path):
    batch = tmp_path / "zoulang_partial_batch"
    successful = saved_cell(batch, "successful_run", "robot_localization_hz5_filter_on")
    unsuccessful = saved_cell(batch, "failed_run", "five_state_hz5_filter_on")
    dataset = "/media/nvidia/tiany-USB/wheelchair/data/experiments/zoulang"
    write_json(batch / "panel_result.json", {"status": "FAILED", "dataset": dataset})
    write_json(batch / "RESULT_SUMMARY.json", {
        "status": "FAILED", "dataset": dataset,
        "maps": [
            {"id": "official_baseline", "run": "successful_run",
             "cell": successful[0].name, "status": "COMPLETE", "title": "官方 EKF · legacy"},
            {"id": "five_state_baseline", "run": "failed_run",
             "cell": unsuccessful[0].name, "status": "FAILED", "title": "五状态 EKF · legacy"},
        ],
    })
    # Repeated lower indexes must not overwrite the summary's per-cell status.
    write_json(batch / "successful_run" / "result.json", {
        "status": "OFFLINE_COMPARISON_COMPLETE", "dataset": dataset,
        "cells": {successful[0].name: {}},
    })
    write_json(batch / "failed_run" / "result.json", {
        "status": "FAILED", "dataset": dataset, "cells": {unsuccessful[0].name: {}},
    })

    for rows in (results(tmp_path), results(batch)):
        assert len(rows) == 1
        row = rows[0]
        assert row["status"] == "COMPLETE"
        assert row["dataset"] == dataset
        assert row["group_id"] == "official_baseline"
        assert row["path"] == str(successful[0])
        assert row["images"] == [str(successful[2])]
        assert row["clouds"] == [str(successful[3])]
        assert row["trajectories"] == [str(successful[1])]
    assert all(path.exists() for path in unsuccessful)


@pytest.mark.parametrize("marker_name", ["panel_result.json", "result.json"])
def test_empty_complete_metadata_is_not_a_saved_map(tmp_path, marker_name):
    batch = tmp_path / "empty_complete_batch"
    write_json(batch / marker_name, {"status": "COMPLETE", "dataset": "/recordings/zoulang"})

    assert results(tmp_path) == []
    assert results(batch) == []
