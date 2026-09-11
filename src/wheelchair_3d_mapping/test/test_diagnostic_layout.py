from copy import deepcopy
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from wheelchair_3d_mapping.diagnostic_layout import load_diagnostic_layout
from wheelchair_3d_mapping.formal_lio_contract import _rotation_from_rpy

ROOT = Path(__file__).resolve().parents[3]
PROFILE = ROOT / 'src/wheelchair_description/config/diagnostic_measured_layout.yaml'
XACRO = ROOT / 'src/wheelchair_description/urdf/wheelchair.urdf.xacro'


def test_derived_extrinsics_recompose_exact_layout():
    cfg = load_diagnostic_layout(PROFILE)
    ri = np.array(cfg['base_to_imu_R']).reshape(3,3)
    ti = np.array(cfg['base_to_imu_T'])
    np.testing.assert_allclose(ri @ cfg['extrinsic_T'] + ti, [.45,-.30,.735], atol=1e-10)
    np.testing.assert_allclose(ri @ np.array(cfg['extrinsic_R']).reshape(3,3),
                               _rotation_from_rpy(cfg['sensors']['right']['rpy']), atol=1e-10)
    np.testing.assert_allclose(ri @ cfg['body_to_base_translation'] + ti, [0,0,0], atol=1e-10)
    assert cfg['formal_runtime_eligible'] is False


@pytest.mark.parametrize('axis,value', [(0,.606), (2,.499)])
def test_old_mismatched_epoch_is_rejected(tmp_path, axis, value):
    data = yaml.safe_load(PROFILE.read_text())
    data['sensors']['right']['xyz'][axis] = value
    path = tmp_path/'bad.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='equal-height'):
        load_diagnostic_layout(path)


def test_diagnostic_cannot_claim_approved(tmp_path):
    data = yaml.safe_load(PROFILE.read_text())
    data['status'] = 'APPROVED'
    path = tmp_path/'bad.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='uncalibrated'):
        load_diagnostic_layout(path)


def xacro_joints(*args):
    process = subprocess.run(['xacro',str(XACRO), f'diagnostic_layout_profile:={PROFILE}',*args],
                             text=True, capture_output=True, check=True)
    robot = ET.fromstring(process.stdout)
    return {j.find('child').get('link'): j.find('origin') for j in robot.findall('joint')}


def test_default_gate_and_mock_fixture_remain_unchanged():
    assert 'xtm60_right_link' not in xacro_joints()
    mock = xacro_joints('include_blocked_right_lidar_for_mock:=true')
    assert mock['xtm60_right_link'].get('xyz') == '0.606 -0.24 0.499'


def test_diagnostic_urdf_matches_profile_even_if_mock_also_enabled():
    joints = xacro_joints('diagnostic_measured_layout:=true', 'include_blocked_right_lidar_for_mock:=true')
    layout = load_diagnostic_layout(PROFILE)
    for sensor in layout['sensors'].values():
        for field in ['xyz','rpy']:
            np.testing.assert_allclose([float(v) for v in joints[sensor['frame']].get(field).split()], sensor[field])
