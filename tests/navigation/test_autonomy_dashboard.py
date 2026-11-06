"""Adding simulation runs to an offline progress snapshot."""
import base64
import json
from pathlib import Path

import pytest

from wc_navigation.autonomy.dashboard_update import run_row, updated_snapshot
from wc_navigation.dashboard import render_html

PNG = base64.b64decode("iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNkYPhfDwAChwGA60e6kgAAAABJRU5ErkJggg==")
TEMPLATE = Path(__file__).resolve().parents[2] / "src/wc_navigation/progress_dashboard.html"


def base_snapshot():
    return {"snapshot": {"mode": "OFFLINE_SNAPSHOT", "physical_navigation_validated": False,
                         "live_sensor_feed": False},
            "summary": {"completed": [{"work": "earlier"}], "tests": {"passed": 1}},
            **{key: {} for key in ["mapping", "near", "localization", "onsite"]}}


def run_dir(tmp_path, mode="explore", **edits):
    directory = tmp_path / ("run_" + mode)
    directory.mkdir()
    result = {"mode": mode, "status": "PASS", "simulation_only": True, "hardware_control": False,
              "config": {"world": {"kind": "synthetic", "name": "two_rooms", "obstacles": [[1, 1, 0.3]]}},
              "trajectory": {"localization_position_error_m": {"mean": 0.12, "max": 0.3}},
              "collisions": {"count": 0}, "distance_m": 12.5,
              "coverage": {"free_coverage_fraction": 0.9, "false_free_area_m2": 0.1},
              "exploration": {"status": "COMPLETE", "goals": [{"kind": "frontier", "succeeded": True}]},
              "goal_outcomes": [{"succeeded": True, "true_position_error_m": 0.2}]}
    result.update(edits)
    (directory / "result.json").write_text(json.dumps(result), encoding="utf-8")
    (directory / "preview.png").write_bytes(PNG)
    return directory


def test_rows_carry_hash_metrics_and_validated_png(tmp_path):
    row = run_row(run_dir(tmp_path))
    assert row["status_label"].startswith("完成") and row["collisions"] == 0
    assert "覆盖 90%" in row["outcome_label"] and "均值 0.12 m" in row["localization_label"]
    assert row["preview_data_url"].startswith("data:image/png;base64,")
    assert len(row["source"]["sha256"]) == 64
    navigate = run_row(run_dir(tmp_path, mode="navigate"))
    assert "目标 1/1" in navigate["outcome_label"] and "地图外障碍 1" in navigate["outcome_label"]


def test_comparison_rows_omit_the_preview(tmp_path):
    row = run_row(run_dir(tmp_path), preview=False)
    assert "preview_data_url" not in row and len(row["source"]["sha256"]) == 64


@pytest.mark.parametrize("edits", [{"simulation_only": False}, {"hardware_control": True}])
def test_non_simulation_results_are_refused(tmp_path, edits):
    with pytest.raises(ValueError, match="simulation"):
        run_row(run_dir(tmp_path, **edits))


def test_non_png_preview_is_refused(tmp_path):
    directory = run_dir(tmp_path)
    (directory / "preview.png").write_bytes(b"<svg onload=x>")
    with pytest.raises(ValueError, match="PNG"):
        run_row(directory)


def test_snapshot_keeps_offline_flags_and_base_is_unchanged(tmp_path):
    base = base_snapshot()
    before = json.dumps(base, sort_keys=True)
    snapshot = updated_snapshot(base, [run_row(run_dir(tmp_path))], tests={"passed": 500},
                                next_remote=["影子模式"], notes=["真实录包重放"])
    assert snapshot["autonomy"]["notes"][-1] == "真实录包重放"
    assert json.dumps(base, sort_keys=True) == before
    meta = snapshot["snapshot"]
    assert meta["mode"] == "OFFLINE_SNAPSHOT" and meta["physical_navigation_validated"] is False
    assert meta["live_sensor_feed"] is False and snapshot["autonomy"]["simulation_only"] is True
    assert snapshot["summary"]["completed"][0] == {"work": "earlier"} and len(snapshot["summary"]["completed"]) == 2
    assert snapshot["summary"]["tests_passed"] == 500 and snapshot["summary"]["next_remote"] == ["影子模式"]
    rendered = render_html(TEMPLATE.read_text(encoding="utf-8"), snapshot)
    assert 'id="autonomy-section"' in rendered and "__WC_DATA__" not in rendered


def shadow_dir(tmp_path, **edits):
    directory = tmp_path / "shadow_run"
    directory.mkdir()
    result = {"mode": "shadow", "status": "COMPLETE", "open_loop": True, "commands_sent": False,
              "hardware_control": False,
              "config": {"source_cell": "/data/analysis/cell_a/robot_localization", "window_s": [57.3, 180.3],
                         "map_yaml": "/reports/navigation_replay_mapping/run_x/odom_grid/map_nav.yaml"},
              "commands": {"drives_forward_fraction_while_recording_drives_forward": 0.62,
                           "outcome_fractions_while_recording_moves": {"STOPPED_BY_COLLISION_MONITOR": 0.015,
                                                                       "NAV2_NOT_COMMANDING": 0.3},
                           "slowed_by_collision_monitor_fraction_while_moving": 0.04,
                           "turn_direction_agreement": {"fraction": 0.97, "samples": 181}},
              "planner": {"plan_requests": 120, "plan_success_fraction": 0.6,
                          "failure_reasons": {"LIVE_COSTMAP_BLOCKS_THE_START": 20,
                                              "INFLATED_UNKNOWN_CUTS_THE_ROUTE": 28}}}
    result.update(edits)
    (directory / "result.json").write_text(json.dumps(result), encoding="utf-8")
    (directory / "preview.png").write_bytes(PNG)
    return directory


def test_shadow_rows_summarize_agreement_and_failure_reasons(tmp_path):
    from wc_navigation.autonomy.dashboard_update import shadow_row
    row = shadow_row(shadow_dir(tmp_path))
    assert row["drive_label"] == "62%" and row["plan_label"] == "72/120 成功"
    assert row["failure_label"].startswith("未知格膨胀后截断路线 28；实时扫描标记压住起点 20")
    assert row["turn_label"] == "97%（181 个样本）" and row["window_label"] == "57–180 s"
    assert row["map_label"] == "run_x/odom_grid/map_nav.yaml" and row["preview_data_url"].startswith("data:image/png")


@pytest.mark.parametrize("edits", [{"open_loop": False}, {"commands_sent": True}, {"hardware_control": True},
                                   {"mode": "explore"}])
def test_shadow_results_that_could_have_driven_are_refused(tmp_path, edits):
    from wc_navigation.autonomy.dashboard_update import shadow_row
    with pytest.raises(ValueError, match="open-loop"):
        shadow_row(shadow_dir(tmp_path, **edits))


def test_shadow_only_update_keeps_the_simulation_table_and_renders(tmp_path):
    from wc_navigation.autonomy.dashboard_update import shadow_row
    first = updated_snapshot(base_snapshot(), [run_row(run_dir(tmp_path))])
    second = updated_snapshot(first, [], shadow_rows=[shadow_row(shadow_dir(tmp_path))], shadow_notes=["补充"])
    assert second["autonomy"] == first["autonomy"]
    assert second["shadow"]["open_loop"] is True and second["shadow"]["commands_sent"] is False
    assert second["shadow"]["notes"][-1] == "补充"
    works = [c["work"] for c in second["summary"]["completed"]]
    assert len(works) == len(set(works)) == 3          # earlier, simulation, shadow; never duplicated
    (tmp_path / "later").mkdir()
    again = updated_snapshot(second, [], shadow_rows=[shadow_row(shadow_dir(tmp_path / "later"))])
    assert len(again["summary"]["completed"]) == 3
    assert any(key.startswith("shadow:") for key in second["provenance"]["files"])
    rendered = render_html(TEMPLATE.read_text(encoding="utf-8"), second)
    assert 'id="shadow-section"' in rendered and "shadow:s.shadow" in rendered


def test_shadow_previews_are_embedded_as_a_narrower_copy(tmp_path):
    from io import BytesIO
    from PIL import Image
    from wc_navigation.autonomy.dashboard_update import SHADOW_PREVIEW_WIDTH_PX, shadow_row
    directory = shadow_dir(tmp_path, config={"source_cell": "/a/b/c", "window_s": [0, 10], "map_yaml": "/m/r/k/map.yaml",
                                             "shadow_mode": "follow", "profile": {"radius_m": 0.45}},
                           shadow_mode="follow", planner={})
    wide = BytesIO()
    Image.new("RGB", (2400, 300), (200, 220, 240)).save(wide, format="PNG")
    (directory / "preview.png").write_bytes(wide.getvalue())
    row = shadow_row(directory)
    embedded = base64.b64decode(row["preview_data_url"].split(",", 1)[1])
    with Image.open(BytesIO(embedded)) as image:
        assert image.size == (SHADOW_PREVIEW_WIDTH_PX, round(300 * SHADOW_PREVIEW_WIDTH_PX / 2400))
    assert row["preview_sha256"] != row["embedded_preview_sha256"]
    assert row["label"].endswith("0.45 m") and row["plan_label"].startswith("不经规划")
    assert "preview_data_url" not in shadow_row(directory, preview=False)
