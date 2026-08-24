"""Fail-closed entry for right-lidar Stage 1 mapping.

No hardware action is constructed while the calibration contract is blocked.
"""

import json
import math
from pathlib import Path

from launch import LaunchDescription
from launch.actions import OpaqueFunction


_CONTRACT_FILE = "right_lidar_stage1_calibration_contract.json"
_CONTRACT_ID = "right_lidar_stage1_calibration"


def _reject_duplicate_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_nonstandard_constant(value):
    raise ValueError(f"non-standard JSON numeric constant: {value}")


def _contains_non_finite(value):
    if isinstance(value, float):
        return not math.isfinite(value)
    if isinstance(value, list):
        return any(_contains_non_finite(item) for item in value)
    if isinstance(value, dict):
        return any(_contains_non_finite(item) for item in value.values())
    return False


def _is_finite_number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    return not isinstance(value, float) or math.isfinite(value)


def _require_exact_keys(value, expected, label):
    if not isinstance(value, dict) or set(value) != set(expected):
        raise ValueError(f"{label} has an invalid schema")


def _matches_reviewed_value(value, expected):
    if isinstance(expected, bool):
        return value is expected
    if isinstance(expected, (int, float)):
        return _is_finite_number(value) and value == expected
    if isinstance(expected, str):
        return isinstance(value, str) and value == expected
    if isinstance(expected, list):
        return (
            isinstance(value, list)
            and len(value) == len(expected)
            and all(
                _matches_reviewed_value(item, expected_item)
                for item, expected_item in zip(value, expected)
            )
        )
    if isinstance(expected, dict):
        return (
            isinstance(value, dict)
            and set(value) == set(expected)
            and all(
                _matches_reviewed_value(value[key], expected_item)
                for key, expected_item in expected.items()
            )
        )
    return value is expected


def _validate_contract(contract):
    _require_exact_keys(
        contract,
        (
            "schema_version",
            "contract_id",
            "status",
            "runtime_transform",
            "frames",
            "deployment_identity",
            "evidence",
        ),
        "contract",
    )
    if type(contract["schema_version"]) is not int or contract["schema_version"] != 1:
        raise ValueError("unsupported schema_version")
    if contract["contract_id"] != _CONTRACT_ID:
        raise ValueError("unexpected contract_id")
    if contract["status"] != "BLOCKED_CONFLICT":
        raise ValueError("unexpected contract status")
    if contract["runtime_transform"] is not None:
        raise ValueError("runtime_transform must be null")

    frames = contract["frames"]
    _require_exact_keys(frames, ("parent", "child"), "frames")
    if frames != {"parent": "base_link", "child": "xtm60_right_link"}:
        raise ValueError("unexpected frame edge")

    identity = contract["deployment_identity"]
    _require_exact_keys(
        identity,
        (
            "device_ip",
            "host_bind_ip",
            "udp_port",
            "deployment_interface",
            "scope",
        ),
        "deployment_identity",
    )
    if identity["device_ip"] != "192.168.1.101":
        raise ValueError("unexpected device_ip")
    if identity["host_bind_ip"] != "192.168.1.100":
        raise ValueError("unexpected host_bind_ip")
    if type(identity["udp_port"]) is not int or identity["udp_port"] != 7687:
        raise ValueError("unexpected udp_port")
    if identity["deployment_interface"] != "eno1":
        raise ValueError("unexpected deployment_interface")
    if identity["scope"] != "DEPLOYMENT_IDENTITY_ONLY_NOT_CALIBRATION":
        raise ValueError("deployment identity scope is not fail-closed")

    evidence = contract["evidence"]
    if not isinstance(evidence, list) or len(evidence) != 4:
        raise ValueError("evidence must contain the four reviewed records")
    expected = {
        "ground_plane_summary_20260623": (
            "right_lidar_installation_20260623",
            "GROUND_PLANE_SUMMARY_PARTIAL_ONLY",
            "PARTIAL_ONLY",
            "auto_test/20260623_dual_radar_calib/calib_right.txt",
            {
                "mount_height_m": 0.510,
                "mount_pitch_deg": -1.04,
                "mount_roll_deg": -9.10,
                "plane_residual_std_m": 0.0043,
            },
        ),
        "initial_full_transform_candidate_20260623": (
            "right_lidar_installation_20260623",
            "FULL_TRANSFORM_CANDIDATE_SOURCE_REFERENCE_ONLY",
            "REJECTED_BAD",
            "auto_test/20260623_dual_radar_calib/right_extrinsic.txt",
            {
                "candidate_payload_in_contract": False,
                "rejection_evidence": (
                    "auto_test/20260623_dual_radar_calib/.verify_right.txt"
                ),
            },
        ),
        "ground_leveling_recompute_20260623": (
            "right_lidar_installation_20260623",
            "SAME_BAG_GROUND_LEVELING_ONLY",
            "GROUND_LEVELING_ONLY",
            "auto_test/20260623_dual_radar_calib/.recompute_right.txt",
            {
                "mount_height_m": 0.499,
                "plane_residual_std_m": 0.0082,
            },
        ),
        "provisional_orientation_20260724": (
            "right_lidar_installation_20260724",
            "PROVISIONAL_ORIENTATION_DIFFERENT_INSTALLATION_EPOCH",
            "PROVISIONAL_DIFFERENT_EPOCH",
            "docs/hardware/evidence/XT_M60_RIGHT_ORIENTATION_PROVISIONAL_20260724.json",
            {
                "translation_z_m": 0.735,
                "sensor_to_base_rpy_rad": [
                    1.707116522011628,
                    0.026526854616668576,
                    1.5744345976274012,
                ],
            },
        ),
    }
    seen = set()
    for record in evidence:
        _require_exact_keys(
            record,
            (
                "evidence_id",
                "installation_epoch",
                "claim_scope",
                "disposition",
                "runtime_eligible",
                "source",
                "observed",
            ),
            "evidence record",
        )
        evidence_id = record["evidence_id"]
        if evidence_id not in expected or evidence_id in seen:
            raise ValueError("unexpected or duplicate evidence_id")
        seen.add(evidence_id)
        epoch, scope, disposition, source, observed = expected[evidence_id]
        if (
            record["installation_epoch"] != epoch
            or record["claim_scope"] != scope
            or record["disposition"] != disposition
            or record["source"] != source
            or record["runtime_eligible"] is not False
            or not _matches_reviewed_value(record["observed"], observed)
        ):
            raise ValueError("evidence does not match the reviewed record")
    if seen != set(expected):
        raise ValueError("reviewed evidence set is incomplete")


def _load_fixed_contract():
    contract_path = Path(__file__).resolve().parents[1] / "config" / _CONTRACT_FILE
    try:
        text = contract_path.read_text(encoding="utf-8")
        contract = json.loads(
            text,
            object_pairs_hook=_reject_duplicate_object,
            parse_constant=_reject_nonstandard_constant,
        )
        if _contains_non_finite(contract):
            raise ValueError("contract contains a non-finite number")
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError, RecursionError) as error:
        raise RuntimeError(f"CONTRACT_INVALID: {error}") from None

    if isinstance(contract, dict) and (
        contract.get("status") == "APPROVED"
        or contract.get("runtime_transform") is not None
    ):
        raise RuntimeError("CONTRACT_TAMPERED: runtime calibration approval is forbidden")
    try:
        _validate_contract(contract)
    except (TypeError, ValueError, OverflowError) as error:
        raise RuntimeError(f"CONTRACT_INVALID: {error}") from None
    return contract


def _block_stage1(_context, *args, **kwargs):
    contract = _load_fixed_contract()
    raise RuntimeError(
        f"BLOCKED_CONFLICT: {contract['contract_id']} has no runtime-eligible transform"
    )


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_block_stage1)])
