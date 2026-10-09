"""Truth and failure-boundary tests. Execute on the verified remote target."""

import copy
import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from wc_calibration import (CalibrationError, QualityProfile, calibrate,
                            fit_clock_model, fit_ground_plane,
                            validate_calibration, validate_transform)
from wc_calibration.cli import main, save_version


@pytest.fixture(scope="module")
def dataset():
    rng = np.random.default_rng(6017)
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("xyz", [0.055, -0.065, 0.09]).as_matrix()
    transform[:3, 3] = [0.31, -0.16, 0.105]
    scenes = []
    for index in range(3):
        # Three independently sampled scenes. Surfaces differ with scene index,
        # and each sensor has independently perturbed observations.
        n = 480
        yz = rng.uniform([-1.4, -0.85], [1.45, 1.25], (n, 2))
        xz = rng.uniform([0.4, -0.85], [3.7, 1.3], (n, 2))
        xy = rng.uniform([0.4, -1.4], [3.7, 1.45], (n, 2))
        wall_x = np.column_stack((np.full(n, 4.2 + 0.27 * index), yz))
        wall_y = np.column_stack((xz[:, 0], np.full(n, 1.8 + 0.17 * index), xz[:, 1]))
        floor = np.column_stack((xy, np.full(n, -1.03 - 0.04 * index)))
        world = np.vstack((wall_x, wall_y, floor))
        left = world + rng.normal(0, 0.0010, world.shape)
        right = (world - transform[:3, 3]) @ transform[:3, :3]
        right += rng.normal(0, 0.0010, world.shape)
        # Nonmatching observations are genuinely different for the two sources.
        left = np.vstack((left, rng.uniform([2, -1, -0.5], [3, 0, 0.7], (25, 3))))
        right = np.vstack((right, rng.uniform([8, -2, 0], [9, 2, 1], (25, 3))))
        scenes.append({"id": f"static-scene-{index}", "left": left.tolist(), "right": right.tolist()})
    initial = np.eye(4)
    initial[:3, 3] = [0.25, -0.12, 0.09]
    return {"schema_version": 1, "source_mode": "synthetic", "sensor_ids": {"left": "SYN-L", "right": "SYN-R"},
            "training": scenes[:2], "validation": scenes[2:], "initial_T_left_right": initial.tolist()}, transform


@pytest.fixture(scope="module")
def solved(dataset):
    return calibrate(dataset[0])


def test_multiscene_recovers_fixed_se3_and_retains_holdout(dataset, solved):
    expected = dataset[1]
    actual = np.asarray(solved["T_left_right"])
    delta = actual @ np.linalg.inv(expected)
    assert solved["status"] == "CANDIDATE", solved["rejection_reasons"]
    assert not solved["live_eligible"]
    assert np.linalg.norm(delta[:3, 3]) < 0.015
    assert Rotation.from_matrix(delta[:3, :3]).magnitude() < 0.006
    assert solved["diagnostics"]["validation"]["scene_ids"] == ["static-scene-2"]
    assert solved["diagnostics"]["training"]["information"]["observable"]
    assert len(solved["diagnostics"]["local_repeat_solve_dispersion"]) == 2
    assert np.allclose(actual @ np.asarray(solved["T_right_left"]), np.eye(4), atol=1e-9)


def test_wrong_inverse_cannot_pass_heldout(dataset):
    result = validate_calibration(dataset[0], np.linalg.inv(dataset[1]))
    assert result["status"] == "UNVALIDATED"
    assert result["rejection_reasons"]


def test_repeated_training_validation_ids_rejected(dataset):
    value = copy.deepcopy(dataset[0])
    value["validation"][0]["id"] = value["training"][0]["id"]
    with pytest.raises(CalibrationError, match="repeated"):
        calibrate(value)


def test_reordered_same_scene_content_is_not_independent(dataset):
    value = copy.deepcopy(dataset[0])
    value["validation"][0] = copy.deepcopy(value["training"][0])
    value["validation"][0]["id"] = "different-name-same-capture"
    value["validation"][0]["left"].reverse()
    value["validation"][0]["right"].reverse()
    with pytest.raises(CalibrationError, match="repeated"):
        calibrate(value)


def test_single_plane_has_small_residual_but_is_degenerate():
    rng = np.random.default_rng(2008)
    scenes = []
    for index in range(3):
        xy = rng.uniform(-2, 2, (600, 2))
        cloud = np.column_stack((xy, np.full(len(xy), -1.0 - index * 0.1)))
        scenes.append({"id": str(index), "left": cloud.tolist(), "right": cloud.tolist()})
    value = {"schema_version": 1, "source_mode": "synthetic", "sensor_ids": {"left": "L", "right": "R"},
             "training": scenes[:2], "validation": scenes[2:], "initial_T_left_right": np.eye(4).tolist()}
    result = calibrate(value)
    assert result["status"] == "UNVALIDATED"
    assert "training:DEGENERATE_GEOMETRY" in result["rejection_reasons"]
    assert result["diagnostics"]["validation"]["forward"][0]["abs_residual_quantiles_m"]["p95"] < 1e-8


def test_low_overlap_rejected_even_with_known_transform(dataset):
    value = copy.deepcopy(dataset[0])
    value["validation"][0]["right"] = (np.asarray(value["validation"][0]["right"]) + [30, 20, 10]).tolist()
    result = validate_calibration(value, dataset[1])
    assert result["status"] == "UNVALIDATED"
    assert any("LOW_OVERLAP" in reason for reason in result["rejection_reasons"])


def test_axis_repeat_and_improper_transform_rejected(dataset):
    value = copy.deepcopy(dataset[0])
    value["axis_conversion_count"] = 2
    with pytest.raises(CalibrationError, match="exactly once"):
        calibrate(value)
    reflection = np.eye(4)
    reflection[0, 0] = -1
    with pytest.raises(CalibrationError, match="SO\\(3\\)"):
        validate_transform(reflection)


def test_real_provenance_is_required(dataset):
    value = copy.deepcopy(dataset[0])
    value["source_mode"] = "real"  # Numeric quality does not manufacture deployment provenance.
    result = validate_calibration(value, dataset[1])
    assert result["status"] == "CANDIDATE"
    assert not result["live_eligible"]
    assert "installation_id" in result["missing_live_provenance"]


def test_ground_requires_trusted_roi_and_preserves_reference():
    rng = np.random.default_rng(22)
    xy = rng.uniform([-2, -1.8], [3, 1.7], (800, 2))
    z = -1.10 + 0.06 * xy[:, 0] - 0.035 * xy[:, 1] + rng.normal(0, 0.002, len(xy))
    points = np.column_stack((xy, z))
    points = np.vstack((points, rng.uniform([-2, -1.8, -1.5], [3, 1.7, -0.6], (100, 3))))
    roi = {"min": [-2.1, -2, -1.6], "max": [3.1, 2, -0.5]}
    with pytest.raises(CalibrationError, match="trusted"):
        fit_ground_plane(points, roi)
    result = fit_ground_plane(points, roi, trusted_roi=True, expected_up=[0, 0, 1], expected_height_m=1.10)
    transform = validate_transform(result["T_ground_rig"])
    transformed = points[:800] @ transform[:3, :3].T + transform[:3, 3]
    assert np.quantile(np.abs(transformed[:, 2]), .95) < .008
    assert "yaw" in result["unobservable_dofs"]
    assert result["status"] == "CANDIDATE"
    assert not result["live_eligible"]
    with pytest.raises(CalibrationError, match="height"):
        fit_ground_plane(points, roi, trusted_roi=True, expected_up=[0, 0, 1], expected_height_m=2.0)


def test_clock_offset_drift_without_fake_transport_certainty():
    # Epoch near Unix time: origin subtraction must occur before float conversion.
    base = 1_789_000_000_000_000_000
    device = [base + index * 100_000_000 for index in range(101)]
    host = [base + 47_000_000 + round((stamp - base) * 1.000023) + ((index % 5) - 2) * 100
            for index, stamp in enumerate(device)]
    model = fit_clock_model(device, host, stream_epochs=["boot-7"] * len(device))
    assert abs(model["drift_ppm"] - 23.) < .02
    predicted_first = model["host_origin_ns"] + round(model["a"] * (device[0] - model["device_origin_ns"]) + model["offset_ns"])
    assert abs(predicted_first - (base + 47_000_000)) < 300
    assert model["fixed_transport_bias_ns"] is None
    assert model["uncertainty_bound_ns"] is None
    assert not model["uncertainty_valid"] and not model["time_valid"]
    bounded = fit_clock_model(device, host, measurement_semantics_verified=True, transport_bias_bound_ns=2_000_000)
    assert bounded["uncertainty_bound_ns"] >= 2_000_000
    assert bounded["time_valid"]


def test_clock_reset_wrap_epochs_and_float_ns_rejected():
    with pytest.raises(CalibrationError, match="nonmonotonic"):
        fit_clock_model([0, 1_000_000, 2_000_000, 0, 1_000_000], [0, 1_000_000, 2_000_000, 3_000_000, 4_000_000])
    with pytest.raises(CalibrationError, match="epoch"):
        fit_clock_model(list(range(0, 5_000_000, 1_000_000)), list(range(0, 5_000_000, 1_000_000)),
                        stream_epochs=[0, 0, 0, 1, 1])
    with pytest.raises(CalibrationError, match="integer"):
        fit_clock_model([float(v) for v in range(5)], list(range(5)))


def test_clock_extrapolation_requires_independent_drift_bound():
    d = [i * 1_000_000_000 for i in range(8)]
    h = [v + 10_000 for v in d]
    result = fit_clock_model(d, h, measurement_semantics_verified=True, transport_bias_bound_ns=1_000,
                             max_extrapolation_ns=1_000_000_000)
    assert not result["time_valid"]
    result = fit_clock_model(d, h, measurement_semantics_verified=True, transport_bias_bound_ns=1_000,
                             max_extrapolation_ns=1_000_000_000, oscillator_extrapolation_bound_ppm=10)
    assert result["uncertainty_bound_ns"] >= 11_000


def test_versioned_cli_writes_once_and_propagates_errors(tmp_path, dataset, solved):
    location = save_version(solved, tmp_path / "versions", "synthetic-v1")
    assert json.loads(location.read_text(encoding="utf-8"))["source_mode"] == "synthetic"
    with pytest.raises(CalibrationError, match="exists"):
        save_version(solved, tmp_path / "versions", "synthetic-v1")
    with pytest.raises(CalibrationError, match="version"):
        save_version(solved, tmp_path / "versions", "../escaped")
    assert main(["report", "--result", str(location)]) == 0
    assert main(["list", "--output-root", str(tmp_path / "versions")]) == 0
    assert main(["report", "--result", str(tmp_path / "missing.json")]) != 0
    source = tmp_path / "dataset.json"
    value = copy.deepcopy(dataset[0])
    value["source_mode"] = "real"
    source.write_text(json.dumps(value), encoding="utf-8")
    assert main(["validate", "--input", str(source), "--result", str(location),
                 "--output-root", str(tmp_path / "versions"), "--version", "must-not-promote"]) != 0
    assert not (tmp_path / "versions" / "must-not-promote").exists()


def test_invalid_quality_profile_rejected():
    with pytest.raises(CalibrationError):
        QualityProfile(voxel_sizes=(.1,), correspondence_distances=(.3, .2))
