"""Display-only result labels retain recording identity and readable methods."""
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PyQt5")

from wc_panel.ui_common import result_label


@pytest.mark.parametrize("identifier,title", [
    ("robot_localization_filtered_filter_original_legacy_plain", "官方 EKF · filtered · 原始运动 · legacy"),
    ("five_state_filtered_filter_original_legacy_plain", "五状态 EKF · filtered · 原始运动 · legacy"),
])
def test_recording_name_and_chinese_method_take_priority_over_machine_id(identifier, title):
    result = {
        "name": "long_fusion_batch / " + identifier + " · obsolete internal label",
        "path": "/analysis/long_fusion_batch/run/cell",
        "dataset": "/media/nvidia/tiany-USB/wheelchair/data/experiments/zoulang",
        "group_id": identifier, "title": title,
    }

    label = result_label(result)

    assert label == "zoulang · " + title
    assert identifier not in label
    assert "obsolete internal label" not in label
    assert result["group_id"] == identifier


def test_numeric_gallery_id_remains_visible_with_recording_name():
    result = {
        "name": "long_fusion_batch / 04 · older title", "path": "/analysis/run/cell",
        "dataset": "/recordings/zoulang", "group_id": "04", "title": "官方 EKF · filtered",
    }

    assert result_label(result) == "zoulang · 04 · 官方 EKF · filtered"


@pytest.mark.parametrize("dataset", [None, ""])
def test_gallery_without_dataset_keeps_existing_compact_label(dataset):
    result = {
        "name": "a_very_long_batch_name / 04 · 官方 filtered",
        "group_id": "04", "path": "/analysis/run/cell", "dataset": dataset,
    }

    assert result_label(result) == "04 · 官方 filtered"


def test_unindexed_legacy_result_keeps_existing_name():
    result = {"name": "旧地图目录", "path": "/analysis/legacy_map"}

    assert result_label(result) == "旧地图目录"
