"""An indexed native map must not expose its unexported intermediate as a map."""
import json

import pytest

from wc_panel.catalog import results


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def trajectory(run, cell):
    front = run / cell
    front.mkdir(parents=True, exist_ok=True)
    (front / "trajectory.csv").write_text("stamp_ns,x_m,y_m\n1,0,0\n2,1,1\n", encoding="utf-8")
    return front


def native_map(run, cell):
    front = trajectory(run, cell)
    export = run / "native" / cell / "export"
    export.mkdir(parents=True)
    (export / "map_3d.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    # Discovery recognizes this file; the point decoder has separate tests.
    (export / "map_3d_cloud.ply.xz").write_bytes(b"retained cloud fixture")
    return front


def indexed_geometry(batch):
    run = batch / "geometry_run"
    intermediate = trajectory(run, "calibrated_motion")
    final = native_map(run, "geometric_motion")
    write_json(run / "refinement_result.json", {
        "status": "OFFLINE_REFINEMENT_COMPLETE_REQUIRES_MAP_REVIEW",
        "dataset": "/recordings/zoulang",
        "cells": {"calibrated_motion": {}, "geometric_motion": {}},
    })
    write_json(batch / "RESULT_SUMMARY.json", {
        "status": "AVAILABLE", "dataset": "/recordings/zoulang",
        "maps": [{"id": "F", "run": run.name, "cell": "geometric_motion",
                  "status": "AVAILABLE", "title": "五状态 · 几何反馈"}],
    })
    return run, intermediate, final


@pytest.mark.parametrize("scan_batch_directly", [False, True])
def test_unindexed_intermediate_trajectory_is_hidden_beside_indexed_native_map(tmp_path, scan_batch_directly):
    batch = tmp_path / "six_maps"
    run, intermediate, final = indexed_geometry(batch)

    rows = results(batch if scan_batch_directly else tmp_path)

    assert len(rows) == 1
    assert rows[0]["group_id"] == "F"
    assert rows[0]["path"] == str(final)
    assert rows[0]["images"] and rows[0]["clouds"]
    assert (intermediate / "trajectory.csv").is_file()
    assert json.loads((run / "refinement_result.json").read_text())["cells"]["calibrated_motion"] == {}


def test_standalone_legacy_trajectory_remains_readable(tmp_path):
    run = tmp_path / "old_batch" / "legacy_motion"
    front = trajectory(run, "prior_only")
    write_json(run / "result.json", {"status": "COMPLETE", "cells": {"prior_only": {}}})

    rows = results(tmp_path)

    assert len(rows) == 1
    assert rows[0]["path"] == str(front)
    assert rows[0]["trajectories"] == [str(front / "trajectory.csv")]


@pytest.mark.parametrize("same_run", [False, True])
def test_unindexed_cell_with_saved_map_is_not_hidden(tmp_path, same_run):
    batch = tmp_path / "six_maps"
    run, _, final = indexed_geometry(batch)
    other_run = run if same_run else batch / "other_unindexed_run"
    other = native_map(other_run, "saved_map")
    marker = other_run / "result.json"
    write_json(marker, {"status": "COMPLETE", "cells": {"saved_map": {}}})

    rows = results(tmp_path)

    assert {row["path"] for row in rows} == {str(final), str(other)}
    assert next(row for row in rows if row["path"] == str(other))["clouds"]


def test_another_run_trajectory_is_not_mistaken_for_the_indexed_runs_intermediate(tmp_path):
    batch = tmp_path / "six_maps"
    _, _, final = indexed_geometry(batch)
    other_run = batch / "separate_motion_run"
    other = trajectory(other_run, "prior_only")
    write_json(other_run / "result.json", {"status": "COMPLETE", "cells": {"prior_only": {}}})

    rows = results(tmp_path)

    assert {row["path"] for row in rows} == {str(final), str(other)}


def test_explicitly_indexed_intermediate_trajectory_remains_visible(tmp_path):
    batch = tmp_path / "six_maps"
    run, intermediate, final = indexed_geometry(batch)
    summary_path = batch / "RESULT_SUMMARY.json"
    summary = json.loads(summary_path.read_text())
    summary["maps"].append({"id": "F0", "run": run.name, "cell": intermediate.name,
                            "status": "AVAILABLE", "title": "明确选择的中间轨迹"})
    write_json(summary_path, summary)

    rows = results(tmp_path)

    assert {row["path"] for row in rows} == {str(final), str(intermediate)}
    assert next(row for row in rows if row["path"] == str(intermediate))["group_id"] == "F0"


def test_trajectory_index_without_real_native_map_does_not_hide_other_trajectories(tmp_path):
    batch = tmp_path / "legacy_batch"
    run = batch / "motion_run"
    indexed = trajectory(run, "selected_prior")
    other = trajectory(run, "other_prior")
    write_json(batch / "RESULT_SUMMARY.json", {
        "status": "AVAILABLE", "maps": [
            {"id": "F0", "run": run.name, "cell": indexed.name, "title": "选择的轨迹"},
        ],
    })
    write_json(run / "result.json", {
        "status": "COMPLETE", "cells": {indexed.name: {}, other.name: {}},
    })

    assert {row["path"] for row in results(tmp_path)} == {str(indexed), str(other)}
