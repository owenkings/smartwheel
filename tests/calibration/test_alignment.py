"""Independent geometry truth, failure boundaries, CLI and bag adapter tests."""

import copy
import json
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from wc_calibration.alignment import (attach_manual_initial, h30_bag_samples,
                                       imu_gravity, manual_initial)
from wc_calibration.core import CalibrationError
from wc_calibration.cli import main


def matched_points(planar=False):
    right = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [1.2, .7, 0. if planar else .8],
                      [-.2, .3, 0. if planar else 1.1]])
    rotation = Rotation.from_euler("xyz", [0.13, -0.18, 0.07]).as_matrix()
    translation = np.array([0.08, -.6, 0.025])
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = rotation, translation
    left = right @ rotation.T + translation
    return {"schema_version": 1, "source_mode": "synthetic", "sensor_ids": {"left": "L", "right": "R"},
            "units": "m", "coordinate_conventions": {"left": "FLU", "right": "FLU"},
            "left": left.tolist(), "right": right.tolist()}, transform


@pytest.mark.parametrize("planar", [False, True])
def test_kabsch_recovers_proper_direction_without_scaling(planar):
    data, truth = matched_points(planar)
    candidate = manual_initial(data)
    assert candidate["status"] == "CANDIDATE"
    assert np.allclose(candidate["initial_T_left_right"], truth, atol=1e-12)
    assert np.allclose(np.asarray(candidate["initial_T_right_left"]) @ truth, np.eye(4))
    assert candidate["max_residual_m"] < 1e-12
    assert not candidate["scale_fitted"] and not candidate["live_eligible"]
    assert not candidate["independent_validation_performed"]


def test_three_noncollinear_mm_pairs_suffice_as_initial_only():
    data, truth = matched_points()
    data["units"] = "mm"
    for side in ("left", "right"):
        data[side] = (np.asarray(data[side][:3]) * 1000).tolist()
    assert np.allclose(manual_initial(data)["initial_T_left_right"], truth)


@pytest.mark.parametrize("points", [[[0, 0, 0], [1, 0, 0], [2, 0, 0]],
                                    [[0, 0, 0], [1, 0, 0], [2, .0001, 0]],
                                    [[0, 0, 0], [1, 0, 0], [0, 0, 0]]])
def test_line_and_duplicate_matches_rejected(points):
    data, _ = matched_points()
    data["left"] = data["right"] = points
    with pytest.raises(CalibrationError):
        manual_initial(data)


def test_wrong_match_scale_and_reflection_are_not_silently_corrected():
    for modification in ("bad_pair", "scale", "reflection"):
        data, _ = matched_points()
        if modification == "bad_pair":
            data["left"][0], data["left"][1] = data["left"][1], data["left"][0]
        elif modification == "scale":
            data["left"] = (np.asarray(data["left"]) * 1.4).tolist()
        else:
            data["left"] = (np.asarray(data["left"]) * [-1, 1, 1]).tolist()
        result = manual_initial(data)
        assert result["status"] == "UNVALIDATED"
        assert np.linalg.det(np.asarray(result["initial_T_left_right"])[:3, :3]) == pytest.approx(1)


def test_initial_attachment_preserves_holdout_and_rejects_identity_mismatch():
    data, _ = matched_points()
    candidate = manual_initial(data)
    dataset = {"sensor_ids": data["sensor_ids"], "source_mode": "synthetic",
               "coordinate_conventions": data["coordinate_conventions"],
               "training": [{"id": "train"}], "validation": [{"id": "holdout"}]}
    original = copy.deepcopy(dataset)
    result = attach_manual_initial(dataset, candidate)
    assert dataset == original
    assert result["validation"] == original["validation"]
    assert result["manual_initial_provenance"]["input_hash"] == candidate["input_hash"]
    for key, value in (("sensor_ids", {"left": "R", "right": "L"}), ("source_mode", "real"), ("status", "UNVALIDATED")):
        changed = dict(candidate, **{key: value})
        with pytest.raises(CalibrationError):
            attach_manual_initial(dataset, changed)
    wrong_axes = dict(dataset, coordinate_conventions={"left": "RDF", "right": "RDF"})
    with pytest.raises(CalibrationError, match="coordinate conventions"):
        attach_manual_initial(wrong_axes, candidate)


def gravity_data(roll=.12, pitch=-.2, yaw=.7):
    # Independent truth: rotate world up into IMU using full nonzero-yaw R.
    orientation = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
    acceleration = orientation.T @ np.array([0., 0., 9.80665])
    samples = [{"time_ns": 10**18 + i * 20_000_000, "acceleration": acceleration.tolist(),
                "angular_velocity": [0., 0., 0.]} for i in range(151)]
    return {"schema_version": 1, "source_mode": "synthetic", "sensor_id": "H30-test",
            "coordinate_convention": "H30_NATIVE_UNVALIDATED", "acceleration_units": "m/s^2",
            "angular_velocity_units": "rad/s", "units_evidence": "synthetic SI truth",
            "acceleration_includes_gravity": True, "acceleration_sign": "specific_force", "samples": samples}


def test_gravity_recovers_native_tilt_without_yaw_or_lidar_extrinsic():
    data = gravity_data()
    result = imu_gravity(data)
    assert result["status"] == "CANDIDATE"
    assert result["roll_rad"] == pytest.approx(.12)
    assert result["pitch_rad"] == pytest.approx(-.2)
    assert result["yaw_rad"] is None
    assert result["T_left_imu"] is None and result["T_left_right"] is None
    assert not result["live_eligible"] and not result["time_model_validated"]
    other_yaw = imu_gravity(gravity_data(yaw=-1.1))
    assert result["up_unit_in_imu"] == pytest.approx(other_yaw["up_unit_in_imu"])


def test_explicit_g_degrees_and_downward_gravity_sign():
    data = gravity_data()
    data.update(acceleration_units="g", angular_velocity_units="deg/s", acceleration_sign="gravity")
    for sample in data["samples"]:
        sample["acceleration"] = (-np.asarray(sample["acceleration"]) / 9.80665).tolist()
        sample["angular_velocity"] = [0.1, 0., 0.]
    result = imu_gravity(data)
    assert result["roll_rad"] == pytest.approx(.12)
    assert result["pitch_rad"] == pytest.approx(-.2)
    assert result["statistics"]["max_gyro_rad_s"] == pytest.approx(np.deg2rad(.1))


@pytest.mark.parametrize("change", ["units", "gravity_removed", "no_evidence", "backward", "short", "zero"])
def test_imu_unknown_semantics_or_inadequate_window_rejected(change):
    data = gravity_data()
    if change == "units": data["acceleration_units"] = "unknown"
    if change == "gravity_removed": data["acceleration_includes_gravity"] = False
    if change == "no_evidence": data["units_evidence"] = ""
    if change == "backward": data["samples"][9]["time_ns"] = 0
    if change == "short": data["samples"] = data["samples"][:50]
    if change == "zero":
        for sample in data["samples"]: sample["acceleration"] = [0., 0., 0.]
    with pytest.raises(CalibrationError):
        imu_gravity(data)


def test_near_gravity_dispersion_and_gyro_gates_are_independent():
    for change, reason in (("norm", "ACCELERATION_NOT_NEAR_GRAVITY"),
                           ("variation", "ACCELERATION_DISPERSION_TOO_HIGH"),
                           ("gyro", "ANGULAR_MOTION_OR_GYRO_BIAS_TOO_HIGH")):
        data = gravity_data(0., 0.)
        for i, sample in enumerate(data["samples"]):
            if change == "norm": sample["acceleration"] = [0., 0., 1.]
            if change == "variation": sample["acceleration"] = [(-1.)**i, 0., 9.80665]
            if change == "gyro": sample["angular_velocity"] = [0., 0., .2]
        result = imu_gravity(data)
        assert result["status"] == "UNVALIDATED"
        assert reason in result["rejection_reasons"]


def test_pitch_singularity_does_not_manufacture_roll():
    result = imu_gravity(gravity_data(0., np.pi / 2))
    assert result["roll_rad"] is None and result["status"] == "UNVALIDATED"


def bag_records(data):
    for i, sample in enumerate(data["samples"]):
        vector = lambda values: SimpleNamespace(**dict(zip(("x", "y", "z"), values)))
        yield sample["time_ns"], SimpleNamespace(sensor_id=data["sensor_id"], stream_epoch="epoch-one",
            frame_sequence=i, coordinate_convention=data["coordinate_convention"],
            linear_acceleration_valid=True, angular_velocity_valid=True,
            imu=SimpleNamespace(linear_acceleration=vector(sample["acceleration"]),
                                angular_velocity=vector(sample["angular_velocity"])))


def test_bag_adapter_preserves_identity_and_time_basis_and_rejects_bad_fields():
    data = gravity_data()
    selection = {key: value for key, value in data.items() if key != "samples"}
    selection.update(start_ns=data["samples"][0]["time_ns"], end_ns=data["samples"][-1]["time_ns"],
                     bag_uri="/offline/testbag", topic="/wc_mapping/imu/source_frame")
    prepared = h30_bag_samples(selection, records=bag_records(data))
    assert imu_gravity(prepared)["status"] == "CANDIDATE"
    assert prepared["sample_provenance"]["timestamp_basis"].startswith("bag_record_time")
    for field, value in (("sensor_id", "other"), ("linear_acceleration_valid", False),
                          ("stream_epoch", "other"), ("frame_sequence", 0)):
        records = list(bag_records(data))
        setattr(records[1][1], field, value)
        with pytest.raises(CalibrationError):
            h30_bag_samples(selection, records=records)


def test_manual_and_gravity_cli_write_immutable_explicit_candidates(tmp_path, capsys):
    for command, data in (("manual-initial", matched_points()[0]), ("imu-gravity", gravity_data())):
        input_path = tmp_path / (command + ".json")
        input_path.write_text(json.dumps(data), encoding="utf-8")
        args = [command, "--input", str(input_path), "--output-root", str(tmp_path / "versions"), "--version", command]
        assert main(args) == 0
        result = json.loads((tmp_path / "versions" / command / "result.json").read_text())
        assert result["status"] == "CANDIDATE" and result["live_eligible"] is False
        assert main(args) == 2  # No silent replacement of earlier evidence.


def test_cli_passes_manual_initial_into_existing_calibrate_without_touching_holdout(tmp_path, monkeypatch):
    data, truth = matched_points()
    candidate = manual_initial(data)
    dataset = {"sensor_ids": data["sensor_ids"], "source_mode": "synthetic",
               "coordinate_conventions": data["coordinate_conventions"], "validation": [{"id": "independent"}]}
    candidate_path, dataset_path = tmp_path / "initial.json", tmp_path / "dataset.json"
    candidate_path.write_text(json.dumps(candidate), encoding="utf-8")
    dataset_path.write_text(json.dumps(dataset), encoding="utf-8")
    captured = []
    def capture(value, profile):
        captured.append(value)
        return {"status": "CANDIDATE", "live_eligible": False}
    monkeypatch.setattr("wc_calibration.cli.calibrate", capture)
    assert main(["calibrate", "--input", str(dataset_path), "--initial-result", str(candidate_path),
                 "--output-root", str(tmp_path / "versions"), "--version", "icp"]) == 0
    assert np.allclose(captured[0]["initial_T_left_right"], truth)
    assert captured[0]["validation"] == dataset["validation"]
