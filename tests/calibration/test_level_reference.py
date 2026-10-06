"""Source-bound display references preserve original calibration geometry."""

import copy
import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from wc_calibration.alignment import manual_initial
from wc_calibration.core import CalibrationError, _hash
from wc_calibration.gravity_level import estimate_gravity_level
from wc_calibration.level_reference import build_level_reference, validate_level_reference
from wc_calibration.picker import PickerSession


def evidence():
    metadata = {side: {"sensor_id": sensor, "units": "m", "coordinate_convention": "FLU",
                       "raw_key": ["SYNTHETIC", sensor, "epoch", 7], "cloud_data_sha256": "a" * 64}
                for side, sensor in (("left", "SYN-L"), ("right", "SYN-R"))}
    rotation = Rotation.from_euler("xyz", [.23, -.31, 0.]).as_matrix()
    acceleration = np.tile(rotation.T @ [0., 0., 9.80665], (200, 1))
    gravity = estimate_gravity_level(acceleration, np.eye(3), angular_velocity=np.zeros((200, 3)))
    imu = {"source_mode": "synthetic", "sensor_id": "SYN-IMU", "sample_sha256": "b" * 64,
           "timestamp_basis": "synthetic_fixture_only"}
    mounting = {"status": "USER_REPORTED_AXES_PENDING_GEOMETRY_CHECK", "R_left_imu_candidate": np.eye(3).tolist()}
    return metadata, gravity, imu, mounting


def reference():
    metadata, gravity, imu, mounting = evidence()
    return build_level_reference("SYN-SCENE", metadata, gravity, imu, mounting), metadata


def resign(ref):
    ref["payload_sha256"] = _hash({key: value for key, value in ref.items() if key != "payload_sha256"})
    return ref


def test_builds_source_bound_owned_pure_rotation_and_validates_after_json_roundtrip():
    metadata, gravity, imu, mounting = evidence()
    ref = build_level_reference("SYN-SCENE", metadata, gravity, imu, mounting)
    restored = validate_level_reference(json.loads(json.dumps(ref, allow_nan=False)), "SYN-SCENE", metadata)
    assert ref == restored
    assert ref["source_metadata_sha256"] == _hash(metadata)
    assert ref["payload_sha256"] == _hash({key: value for key, value in ref.items() if key != "payload_sha256"})
    assert np.array_equal(np.asarray(ref["T_level_left"])[:3, 3], [0., 0., 0.])
    assert np.array_equal(np.asarray(ref["T_level_left"])[:3, :3], gravity["R_level_left"])
    gravity["R_level_left"][0][0] = 777
    imu["sensor_id"] = "changed"
    mounting["status"] = "changed"
    restored["gravity"]["status"] = "changed"
    assert ref["gravity"]["status"] == "CANDIDATE"
    assert ref["gravity"]["R_level_left"][0][0] != 777
    assert ref["imu_provenance"]["sensor_id"] == "SYN-IMU"
    assert ref["mounting_evidence"]["status"].startswith("USER_REPORTED")


@pytest.mark.parametrize("fault", ["scene", "left_id", "source_hash", "payload_hash", "metadata_changed",
                                  "raw_id_contradiction", "missing_sensor_identity", "not_flu", "missing_field",
                                  "null", "status", "live", "schema_bool", "kind", "gravity_status",
                                  "gravity_live", "gravity_kind", "gravity_validated", "translated",
                                  "tiny_translation", "reflection", "scale", "matrix_shape", "matrix_mismatch",
                                  "gravity_matrix_shape", "empty_imu", "empty_mount", "formal_extrinsic",
                                  "formal_claim"])
def test_bad_or_rebound_references_are_rejected_even_if_payload_is_rehashed(fault):
    ref, metadata = reference()
    if fault == "scene": ref["scene_id"] = "OTHER"
    elif fault == "left_id": ref["left_sensor_id"] = "OTHER"
    elif fault == "source_hash": ref["source_metadata_sha256"] = "c" * 64
    elif fault == "payload_hash": ref["payload_sha256"] = "c" * 64
    elif fault == "metadata_changed": metadata["right"]["raw_key"][3] += 1
    elif fault == "raw_id_contradiction": metadata["left"]["raw_key"][1] = "OTHER"
    elif fault == "missing_sensor_identity": metadata["left"].pop("sensor_id"); metadata["left"].pop("raw_key")
    elif fault == "not_flu": metadata["left"]["coordinate_convention"] = "RDF"
    elif fault == "missing_field": ref.pop("gravity")
    elif fault == "null": ref = None
    elif fault == "status": ref["status"] = "VALIDATED"
    elif fault == "live": ref["live_eligible"] = True
    elif fault == "schema_bool": ref["schema_version"] = True
    elif fault == "kind": ref["kind"] = "calibration"
    elif fault == "gravity_status": ref["gravity"]["status"] = "VALIDATED"
    elif fault == "gravity_live": ref["gravity"]["live_eligible"] = True
    elif fault == "gravity_kind": ref["gravity"]["kind"] = "imu_static_gravity_candidate"
    elif fault == "gravity_validated": ref["gravity"]["mounting_rotation_validated"] = True
    elif fault == "translated": ref["T_level_left"][2][3] = .7
    elif fault == "tiny_translation": ref["T_level_left"][0][3] = 1e-12
    elif fault == "reflection": ref["T_level_left"] = np.diag([1., -1., 1., 1.]).tolist()
    elif fault == "scale": ref["T_level_left"][0][0] *= 2
    elif fault == "matrix_shape": ref["T_level_left"] = np.eye(3).tolist()
    elif fault == "matrix_mismatch": ref["T_level_left"] = np.eye(4).tolist()
    elif fault == "gravity_matrix_shape": ref["gravity"]["R_level_left"] = np.eye(4).tolist()
    elif fault == "empty_imu": ref["imu_provenance"] = {}
    elif fault == "empty_mount": ref["mounting_evidence"] = {}
    elif fault == "formal_extrinsic": ref["T_left_imu"] = np.eye(4).tolist()
    elif fault == "formal_claim": ref["independent_validation_performed"] = True
    if ref is not None and fault != "payload_hash": resign(ref)
    with pytest.raises(CalibrationError):
        validate_level_reference(ref, "SYN-SCENE", metadata)


def test_unhashed_payload_change_and_nonfinite_evidence_are_rejected():
    ref, metadata = reference()
    ref["imu_provenance"]["sensor_id"] = "OTHER"
    with pytest.raises(CalibrationError, match="payload hash"):
        validate_level_reference(ref, "SYN-SCENE", metadata)
    ref, metadata = reference()
    ref["mounting_evidence"]["unexpected"] = float("nan")
    with pytest.raises(CalibrationError, match="finite JSON"):
        validate_level_reference(ref, "SYN-SCENE", metadata)


def test_raw_key_can_supply_missing_sensor_id_without_guessing():
    metadata, gravity, imu, mounting = evidence()
    metadata["left"].pop("sensor_id")
    ref = build_level_reference("SYN-SCENE", metadata, gravity, imu, mounting)
    assert ref["left_sensor_id"] == "SYN-L"


def prepared(with_reference):
    ref, metadata = reference()
    right = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [.3, .6, 1.1], [-.4, .8, .5]])
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("xyz", [.07, -.12, .09]).as_matrix()
    transform[:3, 3] = [.18, -.21, .04]
    left = right @ transform[:3, :3].T + transform[:3, 3]
    scene = {"id": "SYN-SCENE", "left": left.tolist(), "right": right.tolist(), "input_files": metadata}
    if with_reference: scene["level_reference"] = ref
    data = {"schema_version": 1, "status": "PREPARED_NOT_VALIDATED", "source_mode": "synthetic",
            "sensor_ids": {"left": "SYN-L", "right": "SYN-R"}, "units": "m",
            "coordinate_conventions": {"left": "FLU", "right": "FLU"}, "training": [scene]}
    return data, transform


@pytest.mark.parametrize("with_reference", [False, True])
def test_picker_keeps_original_coordinates_ids_hash_and_manual_solver(tmp_path, with_reference):
    data, expected = prepared(with_reference)
    original = copy.deepcopy(data)
    source = tmp_path / "prepared.json"
    source.write_text(json.dumps(data), encoding="utf-8")
    session = PickerSession(source, tmp_path / "out")
    assert session.input_hash == _hash(data)
    scene = session.scene()
    assert ("level_reference" in scene) is with_reference
    for side in ("left", "right"):
        assert scene["clouds"][side] == [{"id": i, "xyz": point} for i, point in enumerate(data["training"][0][side])]
    body = {"input_hash": session.input_hash, "scene_id": "SYN-SCENE",
            "pairs": [{"left_id": i, "right_id": i} for i in range(5)]}
    selected = session.selection(body, solve=True)
    assert selected["left"] == original["training"][0]["left"]
    assert selected["right"] == original["training"][0]["right"]
    assert ("level_reference" in selected["provenance"]) is with_reference
    result = manual_initial(selected)
    assert np.allclose(result["initial_T_left_right"], expected, atol=1e-12)
    assert data == original
    if with_reference:
        scene["level_reference"]["T_level_left"][0][0] = 888
        selected["provenance"]["level_reference"]["status"] = "changed"
        assert session.scene()["level_reference"] == original["training"][0]["level_reference"]
        assert session.selection(body)["provenance"]["level_reference"]["status"] == "CANDIDATE"


def test_picker_rejects_reference_before_creating_output_directory(tmp_path):
    data, _ = prepared(True)
    data["training"][0]["level_reference"]["payload_sha256"] = "0" * 64
    source = tmp_path / "prepared.json"
    source.write_text(json.dumps(data), encoding="utf-8")
    output = tmp_path / "out"
    with pytest.raises(CalibrationError):
        PickerSession(source, output)
    assert not output.exists()
