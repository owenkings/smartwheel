"""Pure checks for the opt-in real right-lidar calibration gate."""

import importlib.util
import json
from pathlib import Path

import pytest


def _module():
    path = Path(__file__).parents[2] / "wheelchair_bringup" / "launch" / "sensors.launch.py"
    spec = importlib.util.spec_from_file_location("sensors_launch_for_test", path)
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except ImportError as exc:
        pytest.skip(f"ROS launch dependencies unavailable: {exc}")
    return module


def _transform(child):
    return {
        "frames": {"parent": "base_link", "child": child},
        "runtime_transform": {
            "translation": [0.45, -0.30, 0.735],
            "rotation": [1.7, 0.02, 1.57],
        },
    }


def _write_contract(path, status="APPROVED", *, canonical=False):
    contract = _transform("xtm60_right_link")
    contract["status"] = status
    if canonical:
        contract["runtime_transforms"] = {
            "xtm60_left": _transform("xtm60_left_link"),
            "xtm60_right": _transform("xtm60_right_link"),
            "imu": _transform("imu_link"),
        }
    path.write_text(json.dumps(contract), encoding="utf-8")


def test_approved_contract_gate_accepts_finite_transform(tmp_path):
    module = _module()
    contract = tmp_path / "contract.json"
    _write_contract(contract)
    passed, detail = module._approved_calibration_contract(str(contract))
    assert passed is True
    assert detail == str(contract.resolve())


@pytest.mark.parametrize("status", ["BLOCKED_CONFLICT", "", "PROVISIONAL"])
def test_contract_gate_rejects_non_approved_status(tmp_path, status):
    module = _module()
    contract = tmp_path / "contract.json"
    _write_contract(contract, status=status)
    passed, reason = module._approved_calibration_contract(str(contract))
    assert passed is False
    assert "status" in reason


def test_contract_gate_rejects_nonfinite_transform(tmp_path):
    module = _module()
    contract = tmp_path / "contract.json"
    _write_contract(contract)
    value = json.loads(contract.read_text(encoding="utf-8"))
    value["runtime_transform"]["translation"][0] = "nan"
    contract.write_text(json.dumps(value), encoding="utf-8")
    passed, reason = module._approved_calibration_contract(str(contract))
    assert passed is False
    assert "non-finite" in reason


def test_canonical_dual_contract_accepts_all_expected_transforms(tmp_path):
    module = _module()
    contract = tmp_path / "contract.json"
    _write_contract(contract, canonical=True)
    passed, detail = module._approved_calibration_contract(str(contract), require_dual=True)
    assert passed is True
    assert detail == str(contract.resolve())


def test_dual_gate_rejects_legacy_right_only_contract(tmp_path):
    module = _module()
    contract = tmp_path / "contract.json"
    _write_contract(contract)
    passed, reason = module._approved_calibration_contract(str(contract), require_dual=True)
    assert passed is False
    assert "dual-lidar" in reason


@pytest.mark.parametrize(
    "name, mutation, reason",
    [
        ("left_missing", lambda value: value["runtime_transforms"].pop("xtm60_left"), "xtm60_left is missing"),
        ("imu_wrong_frame", lambda value: value["runtime_transforms"]["imu"]["frames"].update(child="imu_bad"), "imu has the wrong frame pair"),
        ("right_nonfinite", lambda value: value["runtime_transforms"]["xtm60_right"]["runtime_transform"]["translation"].__setitem__(0, float("inf")), "xtm60_right contains non-finite"),
    ],
)
def test_canonical_dual_contract_fails_closed_for_missing_bad_or_nonfinite_members(tmp_path, name, mutation, reason):
    module = _module()
    contract = tmp_path / f"{name}.json"
    _write_contract(contract, canonical=True)
    value = json.loads(contract.read_text(encoding="utf-8"))
    mutation(value)
    contract.write_text(json.dumps(value), encoding="utf-8")
    passed, detail = module._approved_calibration_contract(str(contract), require_dual=True)
    assert passed is False
    assert reason in detail


def test_sensor_launch_exposes_strict_timestamp_overrides():
    source = (
        Path(__file__).parents[2]
        / "wheelchair_bringup"
        / "launch"
        / "sensors.launch.py"
    ).read_text(encoding="utf-8")
    assert '"xtm60_timestamp_source"' in source
    assert 'choices=["host_receive", "sdk_epoch"]' in source
    assert '"imu_require_device_timestamp"' in source
    assert '"require_device_timestamp": ParameterValue(' in source
