"""Offline input semantics and conservative suggestions; no hardware or ROS runtime needed."""

import copy
import json
from pathlib import Path
import struct
from types import SimpleNamespace as NS

import numpy as np
import pytest

from wc_calibration import CalibrationError
from wc_calibration.cli import main
from wc_calibration.importers import (iter_paired_records, pointcloud2_xyz, prepare_bag_dataset,
                                      prepare_file_dataset, read_ascii_cloud, source_frame_record,
                                      suggest_static_segments)


def _pcd(path, points):
    points = np.asarray(points)
    path.write_text("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n" +
                    f"WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n" +
                    "\n".join(" ".join(map(str, row)) for row in points) + "\n", encoding="ascii")


def test_pcd_field_order_count_units_nan_and_viewpoint(tmp_path):
    path = tmp_path / "ordered.pcd"
    path.write_text("# test only\nVERSION .7\nFIELDS descriptor z x y\nSIZE 4 4 4 4\nTYPE F F F F\n"
                    "COUNT 2 1 1 1\nWIDTH 2\nHEIGHT 2\nVIEWPOINT 4 5 6 1 0 0 0\nPOINTS 4\nDATA ascii\n"
                    "11 12 3000 1000 2000\n11 12 6000 4000 5000\n11 12 9000 7000 8000\n11 12 nan 10 20\n", encoding="ascii")
    result = read_ascii_cloud(path, units="mm", coordinate_convention="FLU")
    np.testing.assert_array_equal(result["points"], [[1, 2, 3], [4, 5, 6], [7, 8, 9]])
    assert result["metadata"]["discarded_nonfinite_count"] == 1
    assert result["metadata"]["viewpoint_applied"] is False
    assert len(result["metadata"]["sha256"]) == 64
    with pytest.raises(CalibrationError, match="units"):
        read_ascii_cloud(path, units="unknown", coordinate_convention="FLU")


def test_ply_ascii_reads_declared_elements_and_reordered_properties(tmp_path):
    path = tmp_path / "mesh.ply"
    path.write_text("ply\nformat ascii 1.0\ncomment test fixture\nelement face 1\n"
                    "property list uchar int vertex_indices\nelement vertex 3\n"
                    "property uchar red\nproperty float z\nproperty float y\nproperty float x\nend_header\n"
                    "3 0 1 2\n255 3 2 1\n128 6 5 4\n0 9 8 7\n", encoding="ascii")
    result = read_ascii_cloud(path, units="m", coordinate_convention="FLU")
    np.testing.assert_array_equal(result["points"], [[1, 2, 3], [4, 5, 6], [7, 8, 9]])


def test_binary_and_truncated_clouds_fail_explicitly(tmp_path):
    pcd = tmp_path / "bad.pcd"
    _pcd(pcd, [[1, 2, 3], [4, 5, 6], [7, 8, 9]])
    pcd.write_text(pcd.read_text().replace("DATA ascii", "DATA binary"))
    with pytest.raises(CalibrationError, match="ascii"):
        read_ascii_cloud(pcd, units="m", coordinate_convention="FLU")
    _pcd(pcd, [[1, 2, 3], [4, 5, 6], [7, 8, 9]])
    pcd.write_text(pcd.read_text().rsplit("7 8 9", 1)[0])
    with pytest.raises(CalibrationError, match="truncated"):
        read_ascii_cloud(pcd, units="m", coordinate_convention="FLU")
    ply = tmp_path / "bad.ply"
    ply.write_text("ply\nformat binary_little_endian 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nend_header\n")
    with pytest.raises(CalibrationError, match="ascii"):
        read_ascii_cloud(ply, units="m", coordinate_convention="FLU")


def _cloud(big=False):
    values = [[1., 2., 3.], [4., 5., 6.], [7., 8., 9.], [10., 11., 12.]]
    payload = bytearray()
    for row in range(2):
        for xyz in values[row*2:row*2+2]:
            payload.extend(struct.pack((">" if big else "<") + "ffff", *xyz, 321.))
        payload.extend(b"PAD!PAD!")
    header = NS(frame_id="lidar_left", stamp=NS(sec=5, nanosec=2))
    return NS(width=2, height=2, point_step=16, row_step=40, is_bigendian=big,
              data=payload, fields=[NS(name=name, offset=index*4, count=1, datatype=7) for index, name in enumerate("xyz")],
              header=header), values


@pytest.mark.parametrize("big", [False, True])
def test_pointcloud2_row_padding_and_endian(big):
    cloud, expected = _cloud(big)
    before = bytes(cloud.data)
    np.testing.assert_array_equal(pointcloud2_xyz(cloud), expected)
    assert bytes(cloud.data) == before
    cloud.row_step = 31
    with pytest.raises(CalibrationError, match="stride"):
        pointcloud2_xyz(cloud)


def test_source_frame_metadata_exact_and_nanoseconds_normalized():
    cloud, _ = _cloud()
    message = NS(side="left", session_id="S", sensor_id="L", stream_epoch="boot0", frame_sequence=42,
                 cloud=cloud, header=cloud.header, source_config_hash="cfg-hash", coordinate_convention="FLU", units="m",
                 device_timestamp_valid=True, device_timestamp_raw=12345, device_timestamp_unit="us",
                 device_time_type="relative", device_sync_state="unknown", host_receive_time=NS(sec=10, nanosec=20),
                 host_monotonic_ns=555, common_time_valid=False, common_time_ns=0, clock_model_id="",
                 time_source="arrival_only", uncertainty_valid=False, uncertainty_ns=0, raw_count=4, valid_count=4,
                 diagnostic_flags=["TIME_UNVALIDATED"])
    result = source_frame_record(message, 10_000_000_021, expected_side="left")
    assert result["raw_key"] == ["S", "L", "boot0", 42]
    assert result["host_receive_time_ns"] == 10_000_000_020
    assert result["uncertainty_ns"] is None and not result["common_time_valid"]
    assert result["device_timestamp_raw"] == 12345
    assert len(result["cloud_data_sha256"]) == 64
    message.header.stamp.nanosec = 1_000_000_000
    with pytest.raises(CalibrationError, match="normalized"):
        source_frame_record(message, 1)


def _record(side, sequence, stamp, points=None, epoch="boot0"):
    if points is None:
        points = np.random.default_rng(431).uniform([-2, -1, -1], [2, 1, 1], (400, 3))
    return {"raw_key": ["S", side, epoch, sequence], "side": side, "session_id": "S", "sensor_id": side,
            "stream_epoch": epoch, "frame_sequence": sequence, "points": np.asarray(points),
            "source_config_hash": side+"-config", "coordinate_convention": "FLU", "clock_model_id": side+"-clock",
            "common_time_valid": True, "common_time_ns": stamp, "host_receive_time_ns": stamp + 5_000_000,
            "bag_record_time_ns": stamp + 10_000_000, "uncertainty_valid": False, "uncertainty_ns": None}


def _pairs(count=7):
    records = []
    for index in range(count):
        records += [_record("left", index, index*250_000_000), _record("right", index, index*250_000_000+1_000_000)]
    return list(iter_paired_records(records))


def test_pairing_is_exactly_once_and_does_not_use_bag_record_clock():
    records = [_record("left", 0, 100), _record("right", 0, 120), _record("left", 1, 200), _record("right", 1, 210)]
    for record in records:
        record["bag_record_time_ns"] += 900_000_000_000
    pairs = list(iter_paired_records(records, max_pair_ns=25))
    assert [pair["t_ref_ns"] for pair in pairs] == [120, 210]
    assert len({tuple(pair["right"]["raw_key"]) for pair in pairs}) == 2
    with pytest.raises(CalibrationError, match="duplicate"):
        list(iter_paired_records(records + [records[0]], max_pair_ns=25))


def test_invalid_common_time_requires_explicit_arrival_exploration():
    records = [_record("left", 0, 100), _record("right", 0, 120)]
    for record in records:
        record["common_time_valid"] = False
        record["clock_model_id"] = ""
    assert list(iter_paired_records(records)) == []
    pair = list(iter_paired_records(records, time_basis="host_receive"))[0]
    assert pair["time_quality"] == "ARRIVAL_ONLY_NOT_SYNCHRONIZED"
    assert pair["static_validated"] is False


def test_epoch_change_flushes_cached_frames_and_old_epoch_late_is_dropped():
    records = [_record("left", 0, 100, epoch="old"), _record("left", 0, 110, epoch="new"),
               _record("right", 0, 110), _record("left", 1, 120, epoch="old"), _record("right", 1, 120)]
    from collections import Counter
    counts = Counter()
    pairs = list(iter_paired_records(records, diagnostics=counts))
    assert len(pairs) == 1 and pairs[0]["left"]["stream_epoch"] == "new"
    assert counts["stream_boundary_flush"] == 1 and counts["old_stream_late"] == 1


def test_static_suggestion_never_proves_motion_or_live_calibration():
    pairs = _pairs()
    report = suggest_static_segments(pairs)
    assert len(report["segments"]) == 1
    assert report["segments"][0]["status"] == "STATIC_CANDIDATE_REQUIRES_REVIEW"
    assert report["static_validated"] is False
    assert report["segments"][0]["confidence_probability"] is None
    moved = copy.deepcopy(pairs)
    for index, pair in enumerate(moved):
        for side in ("left", "right"):
            pair[side]["points"] = pair[side]["points"] + [index*.12, 0, 0]
    assert suggest_static_segments(moved)["segments"] == []
    # A small cloud cannot yield a geometry metric; unavailable is JSON null.
    tiny = copy.deepcopy(pairs[:2])
    for pair in tiny:
        for side in ("left", "right"):
            pair[side]["points"] = pair[side]["points"][:3]
    json.dumps(suggest_static_segments(tiny), allow_nan=False)


def test_prepare_file_dataset_and_cli_preserve_sources_without_static_promotion(tmp_path):
    manifest = {"schema_version": 1, "source_mode": "synthetic", "sensor_ids": {"left": "L", "right": "R"},
                "training": [], "validation": [], "provenance": {"static_scene_evidence": {"fake": "must-not-copy"}}}
    for index in range(3):
        scene = {"id": f"scene{index}"}
        for side in ("left", "right"):
            name = f"{side}-{index}.pcd"
            _pcd(tmp_path/name, np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]]) + index)
            scene[side] = {"path": name, "units": "m", "coordinate_convention": "FLU"}
        manifest["training" if index < 2 else "validation"].append(scene)
    result = prepare_file_dataset(manifest, tmp_path)
    assert len(result["training"]) == 2 and len(result["validation"]) == 1
    assert "provenance" not in result
    assert result["input_preparation"]["static_review_required"]
    source = tmp_path / "files.json"
    source.write_text(json.dumps(manifest), encoding="utf-8")
    assert main(["prepare-files", "--input", str(source), "--output-root", str(tmp_path / "prepared"), "--version", "v1"]) == 0
    frozen = json.loads((tmp_path / "prepared" / "v1" / "result.json").read_text())
    assert frozen["status"] == "PREPARED_NOT_VALIDATED"
    assert frozen["initial_T_left_right"] is None


def test_prepare_bag_explicit_windows_and_independence():
    records, training, validation = [], [], []
    for scene in range(3):
        start = scene*10_000_000_000
        entry = {"id": f"scene{scene}", "start_record_ns": start, "end_record_ns": start+2_000_000_000}
        (training if scene < 2 else validation).append(entry)
        for index in range(7):
            for side in ("left", "right"):
                records.append(_record(side, scene*10+index, start+index*250_000_000))
    selection = {"schema_version": 1, "source_mode": "synthetic", "sensor_ids": {"left": "left", "right": "right"},
                 "bag_uri": "synthetic-fixture", "topics": {"left": "/left", "right": "/right"},
                 "training": training, "validation": validation}
    result = prepare_bag_dataset(selection, record_iterator=iter(records))
    assert result["validation"][0]["id"] == "scene2"
    assert result["training"][0]["selected_raw_frames"]["left"]["raw_key"][1] == "left"
    assert result["training"][0]["static_suggestion"]["status"] == "SUGGESTION_ONLY"
    assert "provenance" not in result
    json.dumps(result, allow_nan=False)
    invalid = copy.deepcopy(selection)
    invalid["validation"][0]["start_record_ns"] = 0
    with pytest.raises(CalibrationError, match="independent"):
        prepare_bag_dataset(invalid, record_iterator=iter(records))
    limited = {**selection, "max_retained_points": 5}
    with pytest.raises(CalibrationError, match="budget"):
        prepare_bag_dataset(limited, record_iterator=iter(records))
