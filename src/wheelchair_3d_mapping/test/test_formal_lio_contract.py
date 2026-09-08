import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

from wheelchair_3d_mapping.formal_lio_contract import (
    load_formal_lio_contract,
    validate_hardware_identity,
)


def _hardware_identity(epoch="test-installation"):
    return {
        "schema_version": 1,
        "installation_epoch": epoch,
        "assets": {
            "xtm60_left": {
                "role": "xtm60_left",
                "model": "XT-M60",
                "hardware_id": "XTM60B20250324000151",
                "identity_kind": "vendor_serial",
                "ip_address": "192.168.0.101",
                "frame_id": "xtm60_left_link",
            },
            "xtm60_right": {
                "role": "xtm60_right",
                "model": "XT-M60",
                "hardware_id": "XTM60B20250324000134",
                "identity_kind": "vendor_serial",
                "ip_address": "192.168.1.101",
                "frame_id": "xtm60_right_link",
            },
            "imu": {
                "role": "imu",
                "model": "H30",
                "hardware_id": "smartwheel-h30-primary",
                "identity_kind": "installation_asset_id",
                "frame_id": "imu_link",
            },
        },
    }


def _transform(child, xyz, rpy=(0.0, 0.0, 0.0)):
    return {
        "frames": {"parent": "base_link", "child": child},
        "runtime_transform": {
            "translation": list(xyz),
            "rotation": list(rpy),
        },
    }


def _contract(path, *, status="APPROVED", include_imu=True):
    transforms = {
        "xtm60_left": _transform("xtm60_left_link", (0.45, 0.30, 0.735)),
        "xtm60_right": _transform("xtm60_right_link", (0.45, -0.30, 0.735)),
    }
    if include_imu:
        transforms["imu"] = _transform("imu_link", (0.0, 0.0, 0.45))
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "status": status,
                "scope": "dual_lidar_imu",
                "installation_epoch": "test-installation",
                "hardware_identity": _hardware_identity(),
                "runtime_transforms": transforms,
            }
        ),
        encoding="utf-8",
    )


def test_contract_derives_right_lidar_in_imu_and_body_bridge(tmp_path):
    contract = tmp_path / "contract.json"
    _contract(contract)
    result = load_formal_lio_contract(contract)
    assert result["radar"] == "right"
    assert result["hardware_identity"] == _hardware_identity()
    assert result["radar_frame"] == "xtm60_right_link"
    np.testing.assert_allclose(result["extrinsic_T"], [0.45, -0.30, 0.285])
    np.testing.assert_allclose(result["extrinsic_R"], np.eye(3).reshape(-1))
    np.testing.assert_allclose(result["body_to_base_translation"], [0.0, 0.0, -0.45])
    np.testing.assert_allclose(result["body_to_base_quaternion"], [0.0, 0.0, 0.0, 1.0])


def test_rotated_imu_is_inverted_for_fast_lio_and_tf(tmp_path):
    contract = tmp_path / "contract.json"
    _contract(contract)
    value = json.loads(contract.read_text(encoding="utf-8"))
    value["runtime_transforms"]["imu"]["runtime_transform"]["rotation"] = [
        0.0,
        0.0,
        np.pi / 2.0,
    ]
    contract.write_text(json.dumps(value), encoding="utf-8")
    result = load_formal_lio_contract(contract, radar="right")
    np.testing.assert_allclose(result["extrinsic_T"], [-0.30, -0.45, 0.285], atol=1e-9)
    expected = np.asarray([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    np.testing.assert_allclose(np.asarray(result["extrinsic_R"]).reshape(3, 3), expected, atol=1e-9)


@pytest.mark.parametrize("failure", ["blocked", "missing_imu", "nan", "single"])
def test_contract_failures_are_closed(tmp_path, failure):
    contract = tmp_path / "contract.json"
    _contract(
        contract,
        status="BLOCKED_CONFLICT" if failure == "blocked" else "APPROVED",
        include_imu=failure != "missing_imu",
    )
    value = json.loads(contract.read_text(encoding="utf-8"))
    if failure == "nan":
        value["runtime_transforms"]["xtm60_right"]["runtime_transform"][
            "translation"
        ][0] = float("nan")
    if failure == "single":
        value["runtime_transforms"] = {
            "xtm60_right": value["runtime_transforms"]["xtm60_right"]
        }
    contract.write_text(json.dumps(value), encoding="utf-8")
    with pytest.raises(ValueError):
        load_formal_lio_contract(contract)


def test_formal_lio_launch_has_no_global_or_sensor_tf_competitor():
    launch_path = Path(__file__).parents[1] / "launch" / "formal_fast_lio.launch.py"
    source = launch_path.read_text(encoding="utf-8")
    assert '"mapping.extrinsic_T": spec["extrinsic_T"]' in source
    assert '"mapping.extrinsic_R": spec["extrinsic_R"]' in source
    assert '"body",' in source and '"base_link",' in source
    assert '"map", "camera_init"' not in source
    assert '"base_link", "xtm60_' not in source

    spec = importlib.util.spec_from_file_location("formal_fast_lio_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module._number(1.25) == "1.25"


def test_identity_accepts_h30_installation_asset_without_vendor_serial():
    identity = _hardware_identity()
    result = validate_hardware_identity(
        identity, installation_epoch="test-installation"
    )
    assert result["assets"]["imu"] == {
        "role": "imu",
        "model": "H30",
        "hardware_id": "smartwheel-h30-primary",
        "identity_kind": "installation_asset_id",
        "frame_id": "imu_link",
    }


@pytest.mark.parametrize(
    "mutation, expected",
    [
        ("duplicate_id", "duplicated"),
        ("missing_radar_ip", "ip_address"),
        ("wrong_frame", "frame_id"),
        ("wrong_kind", "identity_kind"),
        ("wrong_epoch", "installation_epoch"),
    ],
)
def test_identity_failures_are_closed(mutation, expected):
    identity = _hardware_identity()
    if mutation == "duplicate_id":
        identity["assets"]["xtm60_left"]["hardware_id"] = identity["assets"][
            "xtm60_right"
        ]["hardware_id"]
    elif mutation == "missing_radar_ip":
        identity["assets"]["xtm60_left"].pop("ip_address")
    elif mutation == "wrong_frame":
        identity["assets"]["xtm60_left"]["frame_id"] = "xtm60_right_link"
    elif mutation == "wrong_kind":
        identity["assets"]["imu"]["identity_kind"] = "vendor_serial"
    elif mutation == "wrong_epoch":
        identity["installation_epoch"] = "different-installation"
    with pytest.raises(ValueError, match=expected):
        validate_hardware_identity(
            identity, installation_epoch="test-installation"
        )
