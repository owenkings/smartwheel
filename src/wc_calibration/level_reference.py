"""Validate source-bound, offline-only gravity display references.

References never replace original point coordinates or the right-to-left
calibration transform. Hashes bind content and source identity, not accuracy.
"""

import copy

import numpy as np

from .core import CalibrationError, _hash, validate_transform


_FIELDS = {
    "schema_version", "kind", "status", "live_eligible", "scene_id", "left_sensor_id",
    "source_metadata_sha256", "T_level_left", "gravity", "imu_provenance",
    "mounting_evidence", "payload_sha256",
}


def _digest(value, name):
    try:
        return _hash(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise CalibrationError(name + " must be finite JSON data") from error


def _source_identity(scene_id, metadata):
    if not isinstance(scene_id, str) or not scene_id.strip():
        raise CalibrationError("level reference requires an explicit scene id")
    if not isinstance(metadata, dict) or set(metadata) != {"left", "right"} or \
            any(not isinstance(row, dict) for row in metadata.values()):
        raise CalibrationError("level reference requires selected left/right source metadata")
    if any(row.get("units") != "m" or row.get("coordinate_convention") != "FLU" for row in metadata.values()):
        raise CalibrationError("level reference source metadata must use metre/FLU coordinates")
    left = metadata["left"]
    raw_key = left.get("raw_key")
    if raw_key is not None and (not isinstance(raw_key, list) or len(raw_key) != 4 or
                                not isinstance(raw_key[1], str) or not raw_key[1].strip()):
        raise CalibrationError("level reference left raw identity is malformed")
    sensor_id = left.get("sensor_id", raw_key[1] if raw_key is not None else None)
    if not isinstance(sensor_id, str) or not sensor_id.strip() or \
            (raw_key is not None and raw_key[1] != sensor_id):
        raise CalibrationError("level reference requires a consistent explicit left sensor identity")
    return sensor_id


def _matrix(value, shape, name):
    try:
        array = np.asarray(value)
        if array.dtype.kind not in "iuf" or array.shape != shape:
            raise CalibrationError(name + " must be a real numeric matrix of the declared shape")
        array = array.astype(float)
    except (TypeError, ValueError, OverflowError) as error:
        if isinstance(error, CalibrationError):
            raise
        raise CalibrationError(name + " must be a real numeric matrix") from error
    if not np.isfinite(array).all():
        raise CalibrationError(name + " must be finite")
    return array


def _gravity_rotation(gravity):
    if not isinstance(gravity, dict) or type(gravity.get("schema_version")) is not int or \
            gravity["schema_version"] != 1 or gravity.get("kind") != "left_lidar_gravity_level_candidate" or \
            gravity.get("status") != "CANDIDATE" or gravity.get("live_eligible") is not False:
        raise CalibrationError("level reference gravity must be an offline gravity-level candidate")
    for name in ("independent_validation_performed", "mounting_rotation_validated", "time_model_validated"):
        if name in gravity and gravity[name] is not False:
            raise CalibrationError("level reference cannot promote gravity validation status")
    rotation = _matrix(gravity.get("R_level_left"), (3, 3), "gravity.R_level_left")
    transform = np.eye(4)
    transform[:3, :3] = rotation
    validate_transform(transform)
    return rotation


def validate_level_reference(ref, scene_id, metadata):
    """Return an owned checked copy, or raise CalibrationError.

    ``metadata`` is the selected scene's complete ``selected_raw_frames`` or
    ``input_files`` dictionary. No timestamps, samples, or device settings are
    synthesized; external evidence remains candidate provenance.
    """
    sensor_id = _source_identity(scene_id, metadata)
    if not isinstance(ref, dict) or not _FIELDS.issubset(ref):
        raise CalibrationError("level reference is missing required fields")
    if type(ref.get("schema_version")) is not int or ref["schema_version"] != 1 or \
            ref.get("kind") != "imu_gravity_level_preview" or ref.get("status") != "CANDIDATE" or \
            ref.get("live_eligible") is not False:
        raise CalibrationError("level reference must remain an offline v1 candidate preview")
    if ref["scene_id"] != scene_id or ref["left_sensor_id"] != sensor_id:
        raise CalibrationError("level reference does not belong to the selected scene and left sensor")
    if ref["source_metadata_sha256"] != _digest(metadata, "source metadata"):
        raise CalibrationError("level reference source metadata hash mismatch")
    for name in ("imu_provenance", "mounting_evidence"):
        if not isinstance(ref[name], dict) or not ref[name]:
            raise CalibrationError("level reference requires nonempty " + name)
    payload = {name: value for name, value in ref.items() if name != "payload_sha256"}
    if ref["payload_sha256"] != _digest(payload, "level reference payload"):
        raise CalibrationError("level reference payload hash mismatch")
    for name in ("validated", "independent_validation_performed", "mounting_rotation_validated", "time_model_validated"):
        if name in ref and ref[name] is not False:
            raise CalibrationError("level reference cannot claim formal validation")
    if any(name in ref for name in ("calibration_id", "T_left_right", "T_left_imu", "T_rig_imu")):
        raise CalibrationError("level reference cannot contain formal calibration or full mounting extrinsics")
    transform = _matrix(ref["T_level_left"], (4, 4), "T_level_left")
    validate_transform(transform)
    if not np.array_equal(transform[:3, 3], np.zeros(3)) or \
            not np.array_equal(transform[3], [0., 0., 0., 1.]):
        raise CalibrationError("T_level_left must be pure rotation about the original left lidar origin")
    rotation = _gravity_rotation(ref["gravity"])
    if not np.allclose(transform[:3, :3], rotation, atol=1e-10, rtol=0):
        raise CalibrationError("T_level_left contradicts gravity.R_level_left")
    return copy.deepcopy(ref)


def build_level_reference(scene_id, metadata, gravity, imu_provenance, mounting_evidence):
    """Build and validate an offline-only reference from this scene's evidence."""
    sensor_id = _source_identity(scene_id, metadata)
    rotation = _gravity_rotation(gravity)
    transform = np.eye(4)
    transform[:3, :3] = rotation
    ref = {
        "schema_version": 1,
        "kind": "imu_gravity_level_preview",
        "status": "CANDIDATE",
        "live_eligible": False,
        "scene_id": scene_id,
        "left_sensor_id": sensor_id,
        "source_metadata_sha256": _digest(metadata, "source metadata"),
        "T_level_left": transform.tolist(),
        "gravity": copy.deepcopy(gravity),
        "imu_provenance": copy.deepcopy(imu_provenance),
        "mounting_evidence": copy.deepcopy(mounting_evidence),
    }
    ref["payload_sha256"] = _digest(ref, "level reference payload")
    return validate_level_reference(ref, scene_id, metadata)
