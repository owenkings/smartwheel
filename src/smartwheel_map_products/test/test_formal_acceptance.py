import hashlib
import json

import pytest

from smartwheel_map_products.formal_acceptance import (
    _algorithm_profile_check,
    template,
    validate_formal_3d_bundle,
    validate_hardware_evidence_binding,
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


def _write_bundle(
    path,
    *,
    evidence=None,
    contract_path=None,
    hardware_validated=True,
    intensity_token="10.0",
    csv_second_x="0",
):
    path.mkdir()
    points = 10
    xyz_rows = "\n".join("0.0 0.0 1.0" for _ in range(points)) + "\n"
    xyzi_rows = "\n".join(
        f"0.0 0.0 1.0 {intensity_token}" for _ in range(points)
    ) + "\n"
    pcd_header = (
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
        "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {points}\nHEIGHT 1\nPOINTS {points}\nDATA ascii\n"
    )
    amp_pcd_header = pcd_header.replace("FIELDS x y z", "FIELDS x y z intensity").replace(
        "SIZE 4 4 4", "SIZE 4 4 4 4"
    ).replace("TYPE F F F", "TYPE F F F F").replace("COUNT 1 1 1", "COUNT 1 1 1 1")
    (path / "map_geometry.pcd").write_text(pcd_header + xyz_rows, encoding="ascii")
    (path / "map_geometry.ply").write_text(
        f"ply\nformat ascii 1.0\nelement vertex {points}\n"
        "property float x\nproperty float y\nproperty float z\nend_header\n" + xyz_rows,
        encoding="ascii",
    )
    (path / "map_pointcloud_amp.pcd").write_text(amp_pcd_header + xyzi_rows, encoding="ascii")
    (path / "map_pointcloud_amp.ply").write_text(
        f"ply\nformat ascii 1.0\nelement vertex {points}\n"
        "property float x\nproperty float y\nproperty float z\nproperty float intensity\nend_header\n" + xyzi_rows,
        encoding="ascii",
    )
    (path / "trajectory.tum").write_text("1 0 0 0 0 0 0 1\n2 0 0 0 0 0 0 1\n", encoding="ascii")
    (path / "poses.csv").write_text(
        "timestamp,x_m,y_m,z_m,roll_rad,pitch_rad,yaw_rad\n"
        f"1,0,0,0,0,0,0\n2,{csv_second_x},0,0,0,0,0\n",
        encoding="ascii",
    )
    (path / "hardware_profile_used.yaml").write_text(
        "profile:\n  mode: real\n  installation_epoch: test-installation\n"
        "hardware_identity:\n  schema_version: 1\n"
        "  installation_epoch: test-installation\n  assets:\n"
        "    xtm60_left:\n      role: xtm60_left\n      model: XT-M60\n"
        "      hardware_id: XTM60B20250324000151\n"
        "      identity_kind: vendor_serial\n      ip_address: 192.168.0.101\n"
        "      frame_id: xtm60_left_link\n"
        "    xtm60_right:\n      role: xtm60_right\n      model: XT-M60\n"
        "      hardware_id: XTM60B20250324000134\n"
        "      identity_kind: vendor_serial\n      ip_address: 192.168.1.101\n"
        "      frame_id: xtm60_right_link\n"
        "    imu:\n      role: imu\n      model: H30\n"
        "      hardware_id: smartwheel-h30-primary\n"
        "      identity_kind: installation_asset_id\n      frame_id: imu_link\n"
        "lidar_left:\n  model: XT-M60\n  ip_address: 192.168.0.101\n  frame_id: xtm60_left_link\n  point_unit: m\n  scan_rate_hz: 10\n  timestamp_source: device_timestamp\n  intensity_field: intensity\n"
        "lidar_right:\n  model: XT-M60\n  ip_address: 192.168.1.101\n  frame_id: xtm60_right_link\n  point_unit: m\n  scan_rate_hz: 10\n  timestamp_source: device_timestamp\n  intensity_field: intensity\n"
        "dual_lidar:\n  integration_mode: map_only\n  max_pair_time_difference_ms: 10\n"
        "imu:\n  model: H30\n  frame_id: imu_link\n  rate_hz: 200\n"
        "  timestamp_source: device_timestamp\n",
        encoding="utf-8",
    )
    (path / "algorithm_profile_used.yaml").write_text(
        "mapping_backend: rtabmap\nrequire_backend_cloud: true\nrequire_intensity: true\n"
        "mock_lio: false\nbackend_cloud_topic: /rtabmap/optimized_cloud\n"
        "require_backend_trajectory: true\nbackend_path_topic: /rtabmap/optimized_path\n"
        "require_fresh_backend_after_stop: true\nbackend_max_trajectory_lag_sec: 2.5\n"
        "trajectory_source: backend:/rtabmap/optimized_path\n"
        "trajectory_representation: full_6dof_quaternion\n"
        "tf_edges:\n"
        "  global_correction: map->camera_init\n"
        "  local_odometry: camera_init->body\n"
        "  body_bridge: body->base_link\n",
        encoding="utf-8",
    )
    quality = {
        "stage": "H2_REAL_DYNAMIC",
        "hardware_validated": hardware_validated,
        "point_count": points,
        "trajectory_pose_count": 2,
        "trajectory_source": "backend:/rtabmap/optimized_path",
        "trajectory_representation": "full_6dof_quaternion",
        "trajectory_first_stamp": 1.0,
        "trajectory_last_stamp": 2.0,
        "geometry_source": "backend:/rtabmap/optimized_cloud",
        "backend_cloud_snapshot_stamp": 10.0,
        "backend_path_snapshot_stamp": 10.0,
        "backend_snapshot_after_session_stop": True,
        "calibration_contract_bundled": True,
        "pointcloud_amp_preserved": True,
        "pointcloud_amp_point_count": points,
        "pointcloud_amp_min": 0.0 if intensity_token == "0.0" else 10.0,
        "pointcloud_amp_median": 0.0 if intensity_token == "0.0" else 10.0,
        "pointcloud_amp_p95": 0.0 if intensity_token == "0.0" else 10.0,
        "pointcloud_amp_max": 0.0 if intensity_token == "0.0" else 10.0,
        "pointcloud_amp_source": "backend_registered_cloud",
        "tf_conflict_runtime_check": "PASS_EVIDENCE",
        "rejected_pose_associations": 0,
        "mock_lio": False,
        "session": {
            "state": "STOPPED",
            "complete": True,
            "failure_reason": "",
            "frame_ids": ["base_link"],
            "frame_count": 2,
            "rejected_frame_count": 0,
            "raw_point_count": points,
            "first_cloud_stamp": 1.0,
            "last_cloud_stamp": 2.0,
            "cloud_time_span_sec": 1.0,
        },
    }
    (path / "quality_report.json").write_text(json.dumps(quality), encoding="utf-8")
    if contract_path is not None:
        (path / "calibration_contract_used.json").write_bytes(contract_path.read_bytes())
    else:
        fallback_contract = path / "calibration_contract_used.json"
        _approved_contract(fallback_contract, status="BLOCKED_CONFLICT")
    if evidence is not None:
        (path / "formal_acceptance.json").write_text(json.dumps(evidence), encoding="utf-8")
    manifest_entries = []
    for file in sorted(path.iterdir()):
        if file.name == "manifest.json":
            continue
        data = file.read_bytes()
        manifest_entries.append(
            {
                "path": file.name,
                "bytes": len(data),
                "sha256": hashlib.sha256(data).hexdigest(),
            }
        )
    (path / "manifest.json").write_text(
        json.dumps({"complete": True, "files": manifest_entries}), encoding="utf-8"
    )


def _approved_evidence(contract_path):
    return {
        "schema_version": 1,
        "evidence_id": "formal-test-evidence",
        "generated_at_utc": "2026-09-08T00:00:00Z",
        "source_artifacts": ["test://hardware-run"],
        "operator_attestation": "CONFIRMED",
        "gates": {
            "hardware_validation": {
                "status": "PASS",
                "installation_epoch": "test-installation",
                "observed_assets": _observed_assets(),
            },
            "dual_lidar": {
                "status": "PASS",
                "left": {"status": "PASS", "valid_fraction": 0.98, "rate_hz": 10.0},
                "right": {"status": "PASS", "valid_fraction": 0.99, "rate_hz": 10.0},
            },
            "timestamp_sync": {
                "status": "PASS",
                "source": "hardware_trigger",
                "monotonic": True,
                "max_offset_sec": 0.005,
                "sensors": {
                    "xtm60_left": {
                        "source": "hardware_trigger",
                        "monotonic": True,
                    },
                    "xtm60_right": {
                        "source": "hardware_trigger",
                        "monotonic": True,
                    },
                    "imu": {"source": "hardware_trigger", "monotonic": True},
                },
            },
            "extrinsics": {
                "status": "PASS",
                "contract_status": "APPROVED",
                "contract_path": str(contract_path),
                "contract_sha256": hashlib.sha256(contract_path.read_bytes()).hexdigest(),
            },
            "dynamic_validation": {
                "status": "PASS",
                "scenarios": {
                    "straight": {"status": "PASS"},
                    "turn": {"status": "PASS"},
                    "in_place": {"status": "PASS"},
                    "stop_recovery": {"status": "PASS"},
                },
            },
            "realtime": {
                "status": "PASS",
                "max_output_gap_sec": 0.2,
                "drop_rate": 0.001,
                "duration_sec": 120.0,
                "odom_samples": 1200,
            },
            "tf_ownership": {
                "status": "PASS",
                "edges": {
                    "global_correction": {
                        "edge": "map->camera_init",
                        "publishers": 1,
                    },
                    "local_odometry": {
                        "edge": "camera_init->body",
                        "publishers": 1,
                    },
                    "body_bridge": {
                        "edge": "body->base_link",
                        "publishers": 1,
                    },
                },
            },
            "loop_closure": {"status": "PASS", "closures": 2},
            "repeatability": {"status": "PASS", "runs": 3},
        },
    }


def _transform(child):
    return {
        "frames": {"parent": "base_link", "child": child},
        "runtime_transform": {
            "translation": [0.45, -0.30, 0.735],
            "rotation": [1.7, 0.02, 1.57],
        },
    }


def _approved_contract(path, *, status="APPROVED", include_all=True):
    runtime_transforms = {
        "xtm60_right": _transform("xtm60_right_link"),
    }
    if include_all:
        runtime_transforms.update(
            {
                "xtm60_left": _transform("xtm60_left_link"),
                "imu": _transform("imu_link"),
            }
        )
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "status": status,
                "scope": "dual_lidar_imu",
                "installation_epoch": "test-installation",
                "hardware_identity": _hardware_identity(),
                "runtime_transforms": runtime_transforms,
            }
        ),
        encoding="utf-8",
    )


def _binding_fixture(tmp_path):
    bundle = tmp_path / "bundle"
    contract = tmp_path / "contract.json"
    _approved_contract(contract)
    evidence = _approved_evidence(contract)
    _write_bundle(bundle, evidence=evidence, contract_path=contract)
    return bundle / "hardware_profile_used.yaml", contract, evidence


def test_export_hardware_binding_accepts_complete_three_way_provenance(tmp_path):
    profile, contract, evidence = _binding_fixture(tmp_path)
    passed, reason = validate_hardware_evidence_binding(
        evidence,
        hardware_profile_path=profile,
        calibration_contract_path=contract,
    )
    assert passed, reason


def test_export_hardware_binding_rejects_status_only_evidence(tmp_path):
    profile, contract, evidence = _binding_fixture(tmp_path)
    evidence["gates"]["hardware_validation"] = {"status": "PASS"}
    passed, reason = validate_hardware_evidence_binding(
        evidence,
        hardware_profile_path=profile,
        calibration_contract_path=contract,
    )
    assert not passed
    assert "hardware evidence" in reason


def test_export_hardware_binding_rejects_swapped_lidar_identity(tmp_path):
    profile, contract, evidence = _binding_fixture(tmp_path)
    observed = evidence["gates"]["hardware_validation"]["observed_assets"]
    observed["xtm60_left"]["hardware_id"], observed["xtm60_right"][
        "hardware_id"
    ] = (
        observed["xtm60_right"]["hardware_id"],
        observed["xtm60_left"]["hardware_id"],
    )
    passed, reason = validate_hardware_evidence_binding(
        evidence,
        hardware_profile_path=profile,
        calibration_contract_path=contract,
    )
    assert not passed
    assert "hardware_id does not match" in reason


def test_export_hardware_binding_rejects_wrong_contract_digest(tmp_path):
    profile, contract, evidence = _binding_fixture(tmp_path)
    evidence["gates"]["extrinsics"]["contract_sha256"] = "0" * 64
    passed, reason = validate_hardware_evidence_binding(
        evidence,
        hardware_profile_path=profile,
        calibration_contract_path=contract,
    )
    assert not passed
    assert "contract_sha256" in reason


def test_export_hardware_binding_rejects_missing_paths_and_epoch_conflict(tmp_path):
    profile, contract, evidence = _binding_fixture(tmp_path)
    passed, reason = validate_hardware_evidence_binding(
        evidence,
        hardware_profile_path="",
        calibration_contract_path=contract,
    )
    assert not passed
    assert "hardware_profile_path" in reason

    contract_data = json.loads(contract.read_text(encoding="utf-8"))
    contract_data["installation_epoch"] = "different-installation"
    contract.write_text(json.dumps(contract_data), encoding="utf-8")
    passed, reason = validate_hardware_evidence_binding(
        evidence,
        hardware_profile_path=profile,
        calibration_contract_path=contract,
    )
    assert not passed
    assert "installation_epoch" in reason


def test_missing_evidence_is_fail_closed(tmp_path):
    bundle = tmp_path / "bundle"
    _write_bundle(bundle)
    report = validate_formal_3d_bundle(bundle)
    assert report["formal_ready"] is False
    assert not report["checks"]["formal_evidence"]["passed"]
    assert not report["checks"]["timestamp_sync"]["passed"]


def test_complete_evidence_passes(tmp_path):
    bundle = tmp_path / "bundle"
    contract = tmp_path / "contract.json"
    _approved_contract(contract)
    _write_bundle(
        bundle, evidence=_approved_evidence(contract), contract_path=contract
    )
    report = validate_formal_3d_bundle(bundle)
    assert report["formal_ready"] is True
    assert not report["blockers"]


def test_hardware_status_without_observed_identities_is_rejected(tmp_path):
    bundle = tmp_path / "bundle"
    contract = tmp_path / "contract.json"
    _approved_contract(contract)
    evidence = _approved_evidence(contract)
    evidence["gates"]["hardware_validation"] = {"status": "PASS"}
    _write_bundle(bundle, evidence=evidence, contract_path=contract)
    report = validate_formal_3d_bundle(bundle)
    assert report["formal_ready"] is False
    assert not report["checks"]["hardware_evidence"]["passed"]


def test_observed_lidar_identity_swap_is_rejected(tmp_path):
    bundle = tmp_path / "bundle"
    contract = tmp_path / "contract.json"
    _approved_contract(contract)
    evidence = _approved_evidence(contract)
    observed = evidence["gates"]["hardware_validation"]["observed_assets"]
    left_id = observed["xtm60_left"]["hardware_id"]
    observed["xtm60_left"]["hardware_id"] = observed["xtm60_right"]["hardware_id"]
    observed["xtm60_right"]["hardware_id"] = left_id
    _write_bundle(bundle, evidence=evidence, contract_path=contract)
    report = validate_formal_3d_bundle(bundle)
    assert report["formal_ready"] is False
    assert not report["checks"]["hardware_evidence"]["passed"]


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
def test_observed_identity_requires_runtime_binding_fields(tmp_path, role, field):
    bundle = tmp_path / "bundle"
    contract = tmp_path / "contract.json"
    _approved_contract(contract)
    evidence = _approved_evidence(contract)
    evidence["gates"]["hardware_validation"]["observed_assets"][role].pop(field)
    _write_bundle(bundle, evidence=evidence, contract_path=contract)
    report = validate_formal_3d_bundle(bundle)
    assert report["formal_ready"] is False
    assert not report["checks"]["hardware_evidence"]["passed"]


def test_h30_asset_id_does_not_require_vendor_serial(tmp_path):
    bundle = tmp_path / "bundle"
    contract = tmp_path / "contract.json"
    _approved_contract(contract)
    identity = json.loads(contract.read_text(encoding="utf-8"))["hardware_identity"]
    assert "device_serial" not in identity["assets"]["imu"]
    _write_bundle(
        bundle, evidence=_approved_evidence(contract), contract_path=contract
    )
    assert validate_formal_3d_bundle(bundle)["formal_ready"] is True


def test_contract_and_profile_hardware_identity_must_match(tmp_path):
    bundle = tmp_path / "bundle"
    contract = tmp_path / "contract.json"
    _approved_contract(contract)
    value = json.loads(contract.read_text(encoding="utf-8"))
    assets = value["hardware_identity"]["assets"]
    left_id = assets["xtm60_left"]["hardware_id"]
    assets["xtm60_left"]["hardware_id"] = assets["xtm60_right"]["hardware_id"]
    assets["xtm60_right"]["hardware_id"] = left_id
    contract.write_text(json.dumps(value), encoding="utf-8")
    _write_bundle(
        bundle, evidence=_approved_evidence(contract), contract_path=contract
    )
    report = validate_formal_3d_bundle(bundle)
    assert report["formal_ready"] is False
    assert not report["checks"]["calibration_contract_bundle"]["passed"]


def test_host_interpolated_time_cannot_pass(tmp_path):
    bundle = tmp_path / "bundle"
    contract = tmp_path / "contract.json"
    _approved_contract(contract)
    evidence = _approved_evidence(contract)
    evidence["gates"]["timestamp_sync"]["source"] = "host_interpolated"
    _write_bundle(bundle, evidence=evidence, contract_path=contract)
    report = validate_formal_3d_bundle(bundle)
    assert report["formal_ready"] is False
    assert not report["checks"]["timestamp_sync"]["passed"]


def test_blocked_contract_cannot_pass(tmp_path):
    bundle = tmp_path / "bundle"
    contract = tmp_path / "contract.json"
    _approved_contract(contract, status="BLOCKED_CONFLICT")
    _write_bundle(
        bundle, evidence=_approved_evidence(contract), contract_path=contract
    )
    report = validate_formal_3d_bundle(bundle)
    assert not report["checks"]["extrinsics"]["passed"]


def test_manifest_tamper_is_detected(tmp_path):
    bundle = tmp_path / "bundle"
    contract = tmp_path / "contract.json"
    _approved_contract(contract)
    _write_bundle(
        bundle, evidence=_approved_evidence(contract), contract_path=contract
    )
    (bundle / "map_geometry.pcd").write_text("tampered\n", encoding="utf-8")
    report = validate_formal_3d_bundle(bundle)
    assert not report["checks"]["bundle_integrity"]["passed"]


@pytest.mark.parametrize(
    "disabled",
    [
        {"require_dual_lidar": False},
        {"require_loop_closure": False},
        {"require_repeatability": False},
        {"require_intensity": False},
    ],
)
def test_degraded_invocation_never_becomes_formal_ready(tmp_path, disabled):
    bundle = tmp_path / "bundle"
    contract = tmp_path / "contract.json"
    _approved_contract(contract)
    _write_bundle(
        bundle, evidence=_approved_evidence(contract), contract_path=contract
    )
    report = validate_formal_3d_bundle(bundle, **disabled)
    assert report["formal_ready"] is False
    assert not report["checks"]["formal_scope"]["passed"]


@pytest.mark.parametrize("intensity_token", ["0.0", "nan", "inf"])
def test_invalid_or_all_zero_intensity_is_rejected(tmp_path, intensity_token):
    bundle = tmp_path / "bundle"
    contract = tmp_path / "contract.json"
    _approved_contract(contract)
    _write_bundle(
        bundle,
        evidence=_approved_evidence(contract),
        contract_path=contract,
        intensity_token=intensity_token,
    )
    report = validate_formal_3d_bundle(bundle)
    assert report["formal_ready"] is False
    assert not report["checks"]["intensity_preservation"]["passed"]


def test_single_transform_contract_is_rejected(tmp_path):
    bundle = tmp_path / "bundle"
    contract = tmp_path / "contract.json"
    _approved_contract(contract, include_all=False)
    _write_bundle(
        bundle, evidence=_approved_evidence(contract), contract_path=contract
    )
    report = validate_formal_3d_bundle(bundle)
    assert report["formal_ready"] is False
    assert not report["checks"]["calibration_contract_bundle"]["passed"]


def test_trajectory_csv_must_match_tum_full_pose(tmp_path):
    bundle = tmp_path / "bundle"
    contract = tmp_path / "contract.json"
    _approved_contract(contract)
    _write_bundle(
        bundle,
        evidence=_approved_evidence(contract),
        contract_path=contract,
        csv_second_x="1",
    )
    report = validate_formal_3d_bundle(bundle)
    assert report["formal_ready"] is False
    assert not report["checks"]["trajectory_products"]["passed"]


def test_evidence_must_bind_manifest_covered_contract(tmp_path):
    bundle = tmp_path / "bundle"
    contract = tmp_path / "contract.json"
    _approved_contract(contract)
    evidence = _approved_evidence(contract)
    evidence["gates"]["extrinsics"]["contract_sha256"] = "0" * 64
    _write_bundle(bundle, evidence=evidence, contract_path=contract)
    report = validate_formal_3d_bundle(bundle)
    assert report["formal_ready"] is False
    assert not report["checks"]["calibration_contract_bundle"]["passed"]


def test_tf_evidence_must_name_actual_fast_lio_chain(tmp_path):
    bundle = tmp_path / "bundle"
    contract = tmp_path / "contract.json"
    _approved_contract(contract)
    evidence = _approved_evidence(contract)
    evidence["gates"]["tf_ownership"]["edges"]["local_odometry"][
        "edge"
    ] = "odom->base_link"
    _write_bundle(bundle, evidence=evidence, contract_path=contract)
    report = validate_formal_3d_bundle(bundle)
    assert report["formal_ready"] is False
    assert not report["checks"]["tf_ownership"]["passed"]


def test_self_consistent_noncanonical_tf_profile_is_rejected(tmp_path):
    profile = tmp_path / "algorithm_profile.yaml"
    profile.write_text(
        "mapping_backend: rtabmap\n"
        "require_backend_cloud: true\n"
        "require_intensity: true\n"
        "mock_lio: false\n"
        "backend_cloud_topic: /rtabmap/optimized_cloud\n"
        "require_backend_trajectory: true\n"
        "backend_path_topic: /rtabmap/optimized_path\n"
        "require_fresh_backend_after_stop: true\n"
        "backend_max_trajectory_lag_sec: 2.5\n"
        "trajectory_source: backend:/rtabmap/optimized_path\n"
        "trajectory_representation: full_6dof_quaternion\n"
        "tf_edges:\n"
        "  global_correction: map->odom\n"
        "  local_odometry: odom->base_link\n",
        encoding="utf-8",
    )
    passed, reason, _ = _algorithm_profile_check(profile)
    assert not passed
    assert "canonical formal TF chain" in reason


def test_template_is_conservative():
    evidence = template()
    assert evidence["gates"]["extrinsics"]["status"] == "UNVERIFIED"
    hardware = evidence["gates"]["hardware_validation"]
    assert hardware["installation_epoch"] == ""
    assert set(hardware["observed_assets"]) == {
        "xtm60_left",
        "xtm60_right",
        "imu",
    }
    assert all(
        item["match"] is False for item in hardware["observed_assets"].values()
    )
