import ast
import json
from pathlib import Path


ROOT = Path(__file__).parents[2]
BRINGUP = ROOT / "wheelchair_bringup"
DESCRIPTION = ROOT / "wheelchair_description"
CONTRACT_PATH = (
    BRINGUP / "config" / "right_lidar_stage1_calibration_contract.json"
)
LAUNCH_PATH = BRINGUP / "launch" / "right_lidar_stage1_mapping.launch.py"
MANUAL_LIO_RIGHT_PATH = (
    BRINGUP / "launch" / "manual_mapping_lio_right.launch.py"
)
SENSORS_LAUNCH_PATH = BRINGUP / "launch" / "sensors.launch.py"
URDF_PATH = DESCRIPTION / "urdf" / "wheelchair.urdf.xacro"
SCRIPT_PATH = (
    ROOT.parent
    / "scripts"
    / "hardware"
    / "run_right_lidar_stage1_static_acceptance.sh"
)


def _strict_json(path: Path):
    duplicates = []

    def reject_duplicates(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                duplicates.append(key)
            result[key] = value
        return result

    def reject_constant(value):
        raise AssertionError(f"non-standard JSON constant in contract: {value}")

    value = json.loads(
        path.read_text(encoding="utf-8"),
        object_pairs_hook=reject_duplicates,
        parse_constant=reject_constant,
    )
    assert duplicates == []
    return value


def test_calibration_contract_is_blocked_and_contains_no_runtime_transform():
    contract = _strict_json(CONTRACT_PATH)
    assert type(contract["schema_version"]) is int
    assert contract["schema_version"] == 1
    assert contract["contract_id"] == "right_lidar_stage1_calibration"
    assert contract["status"] == "BLOCKED_CONFLICT"
    assert contract["runtime_transform"] is None
    assert contract["frames"] == {
        "parent": "base_link",
        "child": "xtm60_right_link",
    }
    assert contract["deployment_identity"] == {
        "device_ip": "192.168.1.101",
        "host_bind_ip": "192.168.1.100",
        "udp_port": 7687,
        "deployment_interface": "eno1",
        "scope": "DEPLOYMENT_IDENTITY_ONLY_NOT_CALIBRATION",
    }


def test_contract_records_four_non_runtime_eligible_scoped_evidence_items():
    records = {
        record["evidence_id"]: record for record in _strict_json(CONTRACT_PATH)["evidence"]
    }
    assert set(records) == {
        "ground_plane_summary_20260623",
        "initial_full_transform_candidate_20260623",
        "ground_leveling_recompute_20260623",
        "provisional_orientation_20260724",
    }
    assert all(record["runtime_eligible"] is False for record in records.values())

    summary = records["ground_plane_summary_20260623"]
    assert summary["claim_scope"] == "GROUND_PLANE_SUMMARY_PARTIAL_ONLY"
    assert summary["disposition"] == "PARTIAL_ONLY"
    assert summary["observed"] == {
        "mount_height_m": 0.510,
        "mount_pitch_deg": -1.04,
        "mount_roll_deg": -9.10,
        "plane_residual_std_m": 0.0043,
    }

    rejected = records["initial_full_transform_candidate_20260623"]
    assert rejected["disposition"] == "REJECTED_BAD"
    assert rejected["claim_scope"] == "FULL_TRANSFORM_CANDIDATE_SOURCE_REFERENCE_ONLY"
    assert rejected["observed"]["candidate_payload_in_contract"] is False
    assert rejected["source"].endswith("right_extrinsic.txt")

    leveling = records["ground_leveling_recompute_20260623"]
    assert leveling["disposition"] == "GROUND_LEVELING_ONLY"
    assert leveling["observed"] == {
        "mount_height_m": 0.499,
        "plane_residual_std_m": 0.0082,
    }
    assert leveling["installation_epoch"] == summary["installation_epoch"]

    provisional = records["provisional_orientation_20260724"]
    assert provisional["disposition"] == "PROVISIONAL_DIFFERENT_EPOCH"
    assert provisional["observed"]["translation_z_m"] == 0.735
    assert provisional["observed"]["sensor_to_base_rpy_rad"] == [
        1.707116522011628,
        0.026526854616668576,
        1.5744345976274012,
    ]
    assert provisional["installation_epoch"] != summary["installation_epoch"]


def test_launch_has_zero_top_level_io_and_exactly_one_opaque_action():
    source = LAUNCH_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)

    imports = []
    for statement in tree.body:
        if isinstance(statement, ast.Import):
            imports.extend((item.name, item.asname) for item in statement.names)
        elif isinstance(statement, ast.ImportFrom):
            imports.append(
                (
                    statement.module,
                    tuple(item.name for item in statement.names),
                )
            )
        elif isinstance(statement, (ast.Assign, ast.AnnAssign)):
            value = statement.value
            assert value is None or not any(
                isinstance(node, ast.Call) for node in ast.walk(value)
            )
        elif isinstance(statement, ast.Expr):
            assert isinstance(statement.value, ast.Constant)
        else:
            assert isinstance(statement, ast.FunctionDef)

    assert imports == [
        ("json", None),
        ("math", None),
        ("pathlib", ("Path",)),
        ("launch", ("LaunchDescription",)),
        ("launch.actions", ("OpaqueFunction",)),
    ]

    generator = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "generate_launch_description"
    )
    assert len(generator.body) == 1
    returned = generator.body[0]
    assert isinstance(returned, ast.Return)
    assert isinstance(returned.value, ast.Call)
    assert isinstance(returned.value.func, ast.Name)
    assert returned.value.func.id == "LaunchDescription"
    actions = returned.value.args[0]
    assert isinstance(actions, ast.List)
    assert len(actions.elts) == 1
    action = actions.elts[0]
    assert isinstance(action, ast.Call)
    assert isinstance(action.func, ast.Name)
    assert action.func.id == "OpaqueFunction"

    forbidden = {
        "DeclareLaunchArgument",
        "Node",
        "IncludeLaunchDescription",
        "ExecuteProcess",
        "TimerAction",
        "IfCondition",
        "LaunchConfiguration",
    }
    referenced_names = {
        node.id for node in ast.walk(tree) if isinstance(node, ast.Name)
    }
    assert forbidden.isdisjoint(referenced_names)


def test_launch_statically_enforces_strict_fixed_contract_failure_classes():
    source = LAUNCH_PATH.read_text(encoding="utf-8")
    assert "right_lidar_stage1_calibration_contract.json" in source
    assert "object_pairs_hook=_reject_duplicate_object" in source
    assert "parse_constant=_reject_nonstandard_constant" in source
    assert "_contains_non_finite" in source
    assert "math.isfinite" in source
    assert "isinstance(value, bool)" in source
    assert "CONTRACT_INVALID" in source
    assert "CONTRACT_TAMPERED" in source
    assert 'contract.get("status") == "APPROVED"' in source
    assert 'contract.get("runtime_transform") is not None' in source
    assert "BLOCKED_CONFLICT" in source
    assert "Path(__file__).resolve().parents[1]" in source


def test_static_acceptance_script_is_exact_permanent_blocker():
    assert SCRIPT_PATH.read_bytes() == (
        b"#!/usr/bin/env bash\n"
        b"set -euo pipefail\n"
        b"printf '%s\\n' 'BLOCKED_CONFLICT' >&2\n"
        b"exit 78\n"
    )


def test_manual_right_lio_entry_delegates_only_to_strict_blocker():
    source = MANUAL_LIO_RIGHT_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)

    assert "right_lidar_stage1_mapping.launch.py" in source
    assert "manual_teleop.launch.py" not in source
    assert "fast_lio_mapping.launch.py" not in source

    forbidden = {
        "Node",
        "OpaqueFunction",
        "DeclareLaunchArgument",
        "ExecuteProcess",
        "TimerAction",
        "LaunchConfiguration",
    }
    referenced_names = {
        node.id for node in ast.walk(tree) if isinstance(node, ast.Name)
    }
    assert forbidden.isdisjoint(referenced_names)

    generator = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "generate_launch_description"
    )
    returns = [node for node in ast.walk(generator) if isinstance(node, ast.Return)]
    assert len(returns) == 1
    assert source.count("IncludeLaunchDescription(") == 1


def test_sensor_entry_blocks_all_real_right_lidar_routes_before_nodes():
    source = SENSORS_LAUNCH_PATH.read_text(encoding="utf-8")

    assert "def _block_unapproved_right_lidar" in source
    assert 'mode == "real" and right_requested' in source
    assert '_flag(context, "enable_xtm60")' in source
    assert '_flag(\n        context, "enable_xtm60_right"\n    )' in source
    assert "BLOCKED_CONFLICT" in source
    assert "right_lidar_stage1_calibration" in source
    assert "allow_blocked" not in source
    assert "bypass" not in source.lower()

    gate = "OpaqueFunction(function=_block_unapproved_right_lidar)"
    assert source.count(gate) == 1
    assert source.index(gate) < source.index("Node(")


def test_default_urdf_omits_blocked_right_tf_and_sensor_launch_only_enables_it_for_mock():
    urdf = URDF_PATH.read_text(encoding="utf-8")
    sensor_launch = SENSORS_LAUNCH_PATH.read_text(encoding="utf-8")

    assert (
        '<xacro:arg name="include_blocked_right_lidar_for_mock" default="false"/>'
        in urdf
    )
    assert '<xacro:if value="$(arg include_blocked_right_lidar_for_mock)">' in urdf
    assert "MOCK-ONLY historical candidate" in urdf

    assert '" include_blocked_right_lidar_for_mock:="' in sensor_launch
    assert "include_mock_right_tf = PythonExpression(" in sensor_launch
    assert '"\' == \'mock\' and (\'"' in sensor_launch
    assert "enable_xtm60_right" in sensor_launch
