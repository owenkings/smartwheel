"""Configuration regressions; no nodes or physical devices are started."""
import ast
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
LAUNCH = ROOT / 'src/wheelchair_bringup/launch/right_lidar_diag_mapping.launch.py'
CONFIG = ROOT / 'src/wheelchair_bringup/config/right_diag_wheel_imu_ekf.yaml'


def nodes():
    return [node for node in ast.walk(ast.parse(LAUNCH.read_text()))
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
            and node.func.id == 'Node']


def kwargs(node):
    return {kw.arg: kw.value for kw in node.keywords}


def named_node(name):
    return next(node for node in nodes()
                if isinstance(kwargs(node).get('name'), ast.Constant)
                and kwargs(node)['name'].value == name)


def test_ekf_file_key_matches_actual_node_name():
    ekf = kwargs(named_node('wheel_imu_ekf'))
    config = yaml.safe_load(CONFIG.read_text())
    assert list(config) == [ast.literal_eval(ekf['name'])]
    assert 'right_diag_wheel_imu_ekf.yaml' in ast.unparse(ekf['parameters'])


def test_ekf_uses_independent_wheel_vx_and_gyro_yaw_rate_only():
    cfg = yaml.safe_load(CONFIG.read_text())['wheel_imu_ekf']['ros__parameters']
    assert cfg['odom0'] == '/wheel/odom'
    assert cfg['imu0'] == '/imu/data'
    assert cfg['publish_tf'] is False
    assert cfg['two_d_mode'] is True
    assert len(cfg['odom0_config']) == len(cfg['imu0_config']) == 15
    assert [i for i, value in enumerate(cfg['odom0_config']) if value] == [6]
    assert [i for i, value in enumerate(cfg['imu0_config']) if value] == [11]


def test_fastlio_wheel_update_uses_same_layout_as_lidar_extrinsics():
    params = kwargs(named_node('fastlio_mapping'))['parameters']
    overrides = {ast.literal_eval(key): val
                 for key, val in zip(params.elts[1].keys, params.elts[1].values)}
    assert ast.unparse(overrides['wheel_update.enabled']) == 'False if wheel_primary else lio_wheel_aiding'
    assert ast.unparse(overrides['zupt.enabled']) == 'False if wheel_primary else lio_zupt'
    assert ast.unparse(overrides['publish.tf_en']) == 'not wheel_primary'
    for key, field in [('mapping.extrinsic_R', 'extrinsic_R'),
                       ('mapping.extrinsic_T', 'extrinsic_T'),
                       ('wheel_update.base_to_imu_R', 'base_to_imu_R'),
                       ('wheel_update.base_to_imu_T', 'base_to_imu_T')]:
        value = overrides[key]
        assert isinstance(value, ast.Subscript)
        assert value.value.id == 'layout'
        assert ast.literal_eval(value.slice) == field
    assert '/odometry/filtered' not in ast.unparse(params)


def test_wheel_and_zupt_can_be_disabled_independently_without_changing_tf_owner():
    tree = ast.parse(LAUNCH.read_text())
    declarations = {node.args[0].value: node for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        and node.func.id == 'DeclareLaunchArgument' and node.args
        and isinstance(node.args[0], ast.Constant)}
    for name in ('lio_wheel_aiding', 'lio_zupt'):
        assert ast.literal_eval(kwargs(declarations[name])['default_value']) == 'true'
    # No accidental activation of a second localization / TF source.
    executables = [ast.literal_eval(kwargs(node)['executable']) for node in nodes()]
    assert 'rtabmap' not in executables
    assert 'fastlio_mapping' in executables


def test_diagnostic_motion_still_defaults_off():
    tree = ast.parse(LAUNCH.read_text())
    declaration = next(node for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        and node.func.id == 'DeclareLaunchArgument' and node.args
        and isinstance(node.args[0], ast.Constant)
        and node.args[0].value == 'motion_control_enabled')
    assert ast.literal_eval(kwargs(declaration)['default_value']) == 'false'
