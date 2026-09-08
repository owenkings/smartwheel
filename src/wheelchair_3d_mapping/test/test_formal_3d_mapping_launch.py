import importlib.util
import json
from pathlib import Path

import pytest


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


def _observed_assets():
    return {
        "xtm60_left": {
            "hardware_id": "XTM60B20250324000151",
            "identity_source": "vendor_serial_query",
            "observed_model": "XT-M60",
            "observed_identity_kind": "vendor_serial",
            "match": True,
            "observed_ip": "192.168.0.101",
            "observed_frame_id": "xtm60_left_link",
        },
        "xtm60_right": {
            "hardware_id": "XTM60B20250324000134",
            "identity_source": "vendor_serial_query",
            "observed_model": "XT-M60",
            "observed_identity_kind": "vendor_serial",
            "match": True,
            "observed_ip": "192.168.1.101",
            "observed_frame_id": "xtm60_right_link",
        },
        "imu": {
            "hardware_id": "smartwheel-h30-primary",
            "identity_source": "operator_asset_tag",
            "observed_model": "H30",
            "observed_identity_kind": "installation_asset_id",
            "match": True,
            "observed_frame_id": "imu_link",
        },
    }


def _module():
    path = Path(__file__).parents[1] / "launch" / "formal_3d_mapping.launch.py"
    spec = importlib.util.spec_from_file_location("formal_3d_mapping_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _transform(child):
    return {
        "frames": {"parent": "base_link", "child": child},
        "runtime_transform": {
            "translation": [0.45, -0.30, 0.735],
            "rotation": [1.7, 0.02, 1.57],
        },
    }


def _contract(path, status="APPROVED", include_dual=True):
    contract = {
        "schema_version": 1,
        "status": status,
        "scope": "dual_lidar_imu",
        "installation_epoch": "test-installation",
        "hardware_identity": _hardware_identity(),
    }
    if include_dual:
        contract["runtime_transforms"] = {
            "xtm60_left": _transform("xtm60_left_link"),
            "xtm60_right": _transform("xtm60_right_link"),
            "imu": _transform("imu_link"),
        }
    path.write_text(json.dumps(contract), encoding="utf-8")


def _profile(path, *, timestamp_source="device_timestamp"):
    path.write_text(
        "\n".join(
            [
                "profile:", "  mode: real", "  installation_epoch: test-installation",
                "hardware_identity:", "  schema_version: 1",
                "  installation_epoch: test-installation", "  assets:",
                "    xtm60_left:", "      role: xtm60_left", "      model: XT-M60",
                "      hardware_id: XTM60B20250324000151",
                "      identity_kind: vendor_serial", "      ip_address: 192.168.0.101",
                "      frame_id: xtm60_left_link",
                "    xtm60_right:", "      role: xtm60_right", "      model: XT-M60",
                "      hardware_id: XTM60B20250324000134",
                "      identity_kind: vendor_serial", "      ip_address: 192.168.1.101",
                "      frame_id: xtm60_right_link",
                "    imu:", "      role: imu", "      model: H30",
                "      hardware_id: smartwheel-h30-primary",
                "      identity_kind: installation_asset_id", "      frame_id: imu_link",
                "lidar_left:", "  model: XT-M60", "  ip_address: 192.168.0.101",
                "  frame_id: xtm60_left_link", "  point_unit: m", "  scan_rate_hz: 10",
                "  intensity_field: intensity", f"  timestamp_source: {timestamp_source}",
                "lidar_right:", "  model: XT-M60", "  ip_address: 192.168.1.101",
                "  frame_id: xtm60_right_link", "  point_unit: m", "  scan_rate_hz: 10",
                "  intensity_field: intensity", f"  timestamp_source: {timestamp_source}",
                "dual_lidar:", "  integration_mode: map_only",
                "  max_pair_time_difference_ms: 20",
                "imu:", "  model: H30", "  frame_id: imu_link", "  rate_hz: 200",
                f"  timestamp_source: {timestamp_source}", "",
            ]
        ),
        encoding="utf-8",
    )


def _kwargs(contract, profile):
    return {
        "calibration_contract": str(contract),
        "hardware_profile": str(profile),
        "points_topic": "/points_merged",
        "left_points_topic": "/xtm60/left/points",
        "right_points_topic": "/xtm60/right/points",
        "fusion_status_topic": "/formal/points_merged/status",
        "odom_topic": "/Odometry",
        "odom_mode": "external",
        "enable_left": True,
        "enable_right": True,
        "enable_imu": True,
        "allow_single_lidar_fallback": False,
        "enable_loop_closure": True,
        "max_pair_time_difference_sec": 0.020,
        "hardware_validated": False,
        "evidence_path": "",
        "tf_global_edge": "map->camera_init",
        "tf_local_edge": "camera_init->body",
        "tf_body_bridge_edge": "body->base_link",
    }


def test_formal_launch_rejects_blocked_contract(tmp_path):
    module = _module()
    contract = tmp_path / "contract.json"
    profile = tmp_path / "hardware.yaml"
    _contract(contract, status="BLOCKED_CONFLICT")
    _profile(profile)
    with pytest.raises(RuntimeError, match="FORMAL_MAP_BLOCKED"):
        module._validate_formal_launch_contract(**_kwargs(contract, profile))


def test_formal_launch_rejects_single_lidar_fallback(tmp_path):
    module = _module()
    contract = tmp_path / "contract.json"
    profile = tmp_path / "hardware.yaml"
    _contract(contract)
    _profile(profile)
    kwargs = _kwargs(contract, profile)
    kwargs["allow_single_lidar_fallback"] = True
    with pytest.raises(RuntimeError, match="fallback"):
        module._validate_formal_launch_contract(**kwargs)


def test_formal_launch_accepts_explicit_external_odom(tmp_path):
    module = _module()
    contract = tmp_path / "contract.json"
    profile = tmp_path / "hardware.yaml"
    _contract(contract)
    _profile(profile)
    result = module._validate_formal_launch_contract(**_kwargs(contract, profile))
    assert result["points_topic"] == "/points_merged"
    assert result["odom_topic"] == "/Odometry"


def test_formal_launch_accepts_contract_driven_fast_lio(tmp_path):
    module = _module()
    contract = tmp_path / "contract.json"
    profile = tmp_path / "hardware.yaml"
    _contract(contract)
    _profile(profile)
    kwargs = _kwargs(contract, profile)
    kwargs["odom_mode"] = "contract_fastlio"
    result = module._validate_formal_launch_contract(**kwargs)
    assert result["odom_mode"] == "contract_fastlio"


def test_contract_fast_lio_rejects_nonfixed_odom_topic(tmp_path):
    module = _module()
    contract = tmp_path / "contract.json"
    profile = tmp_path / "hardware.yaml"
    _contract(contract)
    _profile(profile)
    kwargs = _kwargs(contract, profile)
    kwargs.update(odom_mode="contract_fastlio", odom_topic="/custom/odom")
    with pytest.raises(RuntimeError, match="fixed /Odometry"):
        module._validate_formal_launch_contract(**kwargs)


def test_contract_fast_lio_rejects_nonfixed_tf_chain(tmp_path):
    module = _module()
    contract = tmp_path / "contract.json"
    profile = tmp_path / "hardware.yaml"
    _contract(contract)
    _profile(profile)
    kwargs = _kwargs(contract, profile)
    kwargs.update(
        odom_mode="contract_fastlio",
        tf_global_edge="map->odom",
        tf_local_edge="odom->base_link",
        tf_body_bridge_edge="",
    )
    with pytest.raises(RuntimeError, match="fixed map->camera_init"):
        module._validate_formal_launch_contract(**kwargs)


def test_formal_output_paths_require_new_database(tmp_path):
    module = _module()
    database = tmp_path / "new.db"
    output = tmp_path / "maps"
    result = module._validate_formal_output_paths(
        str(database), str(output), "formal_map", delete_db_on_start=False
    )
    assert result == (str(database.resolve()), str(output.resolve()), "formal_map")

    database.write_bytes(b"existing")
    with pytest.raises(RuntimeError, match="already exists"):
        module._validate_formal_output_paths(
            str(database), str(output), "formal_map", delete_db_on_start=False
        )


def test_formal_output_paths_reject_delete_relative_and_unsafe_name(tmp_path):
    module = _module()
    database = tmp_path / "new.db"
    output = tmp_path / "maps"
    with pytest.raises(RuntimeError, match="never delete"):
        module._validate_formal_output_paths(
            str(database), str(output), "formal_map", delete_db_on_start=True
        )
    with pytest.raises(RuntimeError, match="must be absolute"):
        module._validate_formal_output_paths(
            "relative.db", str(output), "formal_map", delete_db_on_start=False
        )
    with pytest.raises(RuntimeError, match="safe path component"):
        module._validate_formal_output_paths(
            str(database), str(output), "../formal_map", delete_db_on_start=False
        )


def test_formal_launch_rejects_single_right_transform_contract(tmp_path):
    module = _module()
    contract = tmp_path / "contract.json"
    profile = tmp_path / "hardware.yaml"
    _contract(contract, include_dual=False)
    _profile(profile)
    with pytest.raises(RuntimeError, match="runtime_transforms"):
        module._validate_formal_launch_contract(**_kwargs(contract, profile))


def test_formal_launch_rejects_host_clock_profile(tmp_path):
    module = _module()
    contract = tmp_path / "contract.json"
    profile = tmp_path / "hardware.yaml"
    _contract(contract)
    _profile(profile, timestamp_source="host_receive_time")
    with pytest.raises(RuntimeError, match="approved device clock"):
        module._validate_formal_launch_contract(**_kwargs(contract, profile))


def test_hardware_validated_requires_existing_schema_v1_evidence(tmp_path):
    module = _module()
    contract = tmp_path / "contract.json"
    profile = tmp_path / "hardware.yaml"
    _contract(contract)
    _profile(profile)
    kwargs = _kwargs(contract, profile)
    kwargs.update(hardware_validated=True, evidence_path=str(tmp_path / "missing.json"))
    with pytest.raises(RuntimeError, match="evidence file"):
        module._validate_formal_launch_contract(**kwargs)
    evidence = tmp_path / "evidence.json"
    evidence.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "gates": {"hardware_validation": {"status": "PASS"}},
            }
        ),
        encoding="utf-8",
    )
    kwargs["evidence_path"] = str(evidence)
    with pytest.raises(RuntimeError, match="installation_epoch"):
        module._validate_formal_launch_contract(**kwargs)
    evidence.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "gates": {
                    "hardware_validation": {
                        "status": "PASS",
                        "installation_epoch": "test-installation",
                        "observed_assets": _observed_assets(),
                    }
                },
            }
        ),
        encoding="utf-8",
    )
    module._validate_formal_launch_contract(**kwargs)


def test_formal_launch_rejects_pair_limit_above_twenty_ms(tmp_path):
    module = _module()
    contract = tmp_path / "contract.json"
    profile = tmp_path / "hardware.yaml"
    _contract(contract)
    _profile(profile)
    kwargs = _kwargs(contract, profile)
    kwargs["max_pair_time_difference_sec"] = 0.021
    with pytest.raises(RuntimeError, match="0.020"):
        module._validate_formal_launch_contract(**kwargs)


def test_formal_launch_rejects_runtime_pair_limit_above_profile(tmp_path):
    module = _module()
    contract = tmp_path / "contract.json"
    profile = tmp_path / "hardware.yaml"
    _contract(contract)
    _profile(profile)
    text = profile.read_text(encoding="utf-8").replace(
        "max_pair_time_difference_ms: 20", "max_pair_time_difference_ms: 5"
    )
    profile.write_text(text, encoding="utf-8")
    kwargs = _kwargs(contract, profile)
    kwargs["max_pair_time_difference_sec"] = 0.010
    with pytest.raises(RuntimeError, match="profile limit"):
        module._validate_formal_launch_contract(**kwargs)


def test_formal_launch_rejects_installation_epoch_mismatch(tmp_path):
    module = _module()
    contract = tmp_path / "contract.json"
    profile = tmp_path / "hardware.yaml"
    _contract(contract)
    _profile(profile)
    profile.write_text(
        profile.read_text(encoding="utf-8").replace(
            "installation_epoch: test-installation",
            "installation_epoch: different-installation",
        ),
        encoding="utf-8",
    )
    with pytest.raises(RuntimeError, match="installation_epoch differ"):
        module._validate_formal_launch_contract(**_kwargs(contract, profile))


def test_formal_launch_rejects_swapped_lidar_identities(tmp_path):
    module = _module()
    contract = tmp_path / "contract.json"
    profile = tmp_path / "hardware.yaml"
    _contract(contract)
    _profile(profile)
    value = json.loads(contract.read_text(encoding="utf-8"))
    assets = value["hardware_identity"]["assets"]
    left_id = assets["xtm60_left"]["hardware_id"]
    assets["xtm60_left"]["hardware_id"] = assets["xtm60_right"]["hardware_id"]
    assets["xtm60_right"]["hardware_id"] = left_id
    contract.write_text(json.dumps(value), encoding="utf-8")
    with pytest.raises(RuntimeError, match="hardware_identity differ"):
        module._validate_formal_launch_contract(**_kwargs(contract, profile))


def test_formal_launch_rejects_observed_identity_mismatch(tmp_path):
    module = _module()
    contract = tmp_path / "contract.json"
    profile = tmp_path / "hardware.yaml"
    evidence = tmp_path / "evidence.json"
    _contract(contract)
    _profile(profile)
    observed = _observed_assets()
    observed["xtm60_left"]["hardware_id"] = "unexpected-left-serial"
    evidence.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "gates": {
                    "hardware_validation": {
                        "status": "PASS",
                        "installation_epoch": "test-installation",
                        "observed_assets": observed,
                    }
                },
            }
        ),
        encoding="utf-8",
    )
    kwargs = _kwargs(contract, profile)
    kwargs.update(hardware_validated=True, evidence_path=str(evidence))
    with pytest.raises(RuntimeError, match="observed xtm60_left hardware_id"):
        module._validate_formal_launch_contract(**kwargs)


@pytest.mark.parametrize(
    "role, field",
    [
        ("xtm60_left", "observed_ip"),
        ("xtm60_right", "observed_frame_id"),
        ("imu", "observed_frame_id"),
        ("imu", "observed_model"),
        ("imu", "observed_identity_kind"),
    ],
)
def test_hardware_evidence_requires_runtime_binding_fields(tmp_path, role, field):
    module = _module()
    contract = tmp_path / "contract.json"
    profile = tmp_path / "hardware.yaml"
    evidence = tmp_path / "evidence.json"
    _contract(contract)
    _profile(profile)
    observed = _observed_assets()
    observed[role].pop(field)
    evidence.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "gates": {
                    "hardware_validation": {
                        "status": "PASS",
                        "installation_epoch": "test-installation",
                        "observed_assets": observed,
                    }
                },
            }
        ),
        encoding="utf-8",
    )
    kwargs = _kwargs(contract, profile)
    kwargs.update(hardware_validated=True, evidence_path=str(evidence))
    with pytest.raises(RuntimeError, match="does not match profile"):
        module._validate_formal_launch_contract(**kwargs)


def test_builtin_sensor_bringup_rejects_unimplementable_clock_source(tmp_path):
    module = _module()
    profile = tmp_path / "hardware.yaml"
    _profile(profile)
    loaded = module._validate_real_dual_profile(profile)
    module._validate_direct_sensor_clock_profile(loaded)
    loaded["imu"]["timestamp_source"] = "ptp"
    with pytest.raises(RuntimeError, match="direct device timestamps for imu"):
        module._validate_direct_sensor_clock_profile(loaded)


def test_formal_tf_chain_must_be_connected_and_end_at_base(tmp_path):
    module = _module()
    contract = tmp_path / "contract.json"
    profile = tmp_path / "hardware.yaml"
    _contract(contract)
    _profile(profile)
    kwargs = _kwargs(contract, profile)
    kwargs["tf_local_edge"] = "odom->body"
    with pytest.raises(RuntimeError, match="disconnected"):
        module._validate_formal_launch_contract(**kwargs)
