"""Preview and mapping composition; no device or ROS graph."""
import json

import numpy as np
import pytest
import yaml

from wc_runtime import mapping_controller as controller
from wc_runtime.mapping_app import prepare_request
from wc_runtime.mapping_preview import preview_geometry
from test_mapping_controller import setup, tmp_path


@pytest.mark.parametrize('mode', ['left', 'right', 'all'])
@pytest.mark.parametrize('cameras', [False, True])
def test_preview_has_current_frames_without_any_mapping_or_recording(setup, mode, cameras):
    request = prepare_request(mode, project_root=setup['project_root'], config=setup['config_path'])
    assert request['mapping_enabled'] is False
    config = controller.configuration(request)
    config['cameras_enabled'] = cameras
    plan = controller.plan(request, config)
    commands = [' '.join(map(str, argv)) for argv in plan['commands']]
    # The health observer watches files/status only and is now a separate
    # owned process; it must not be counted as another sensor or estimator.
    assert len(commands) == (5 if cameras else 4)
    assert any('wc_runtime.runtime_health' in command and '--watch' in command for command in commands)
    assert plan['offline_experiment'] is True and plan['launch_allowed'] is False
    assert plan['bag_topics'] == []
    assert all(term not in '\n'.join(commands) for term in ('bag record', 'mapping_input', 'mapping_prior',
        'mapping_monitor', 'mapping_app.launch.py', 'wc_imu.ros_node', 'rtabmap'))
    assert any('wc_runtime.mapping_preview' in command for command in commands)
    assert any('wc_runtime.mapping_wheel' in command for command in commands)
    rendered = yaml.safe_load(controller.render_view(config))['Visualization Manager']
    assert rendered['Global Options']['Fixed Frame'] == 'mapping_reference'
    clouds = [d for d in rendered['Displays'] if d['Class'] == 'rviz_default_plugins/PointCloud2']
    assert len(clouds) == (4 if mode == 'all' else 2)
    assert all(d['Decay Time'] == 0 and d['Topic']['Depth'] == 1 and
               d['Topic']['Reliability Policy'] == 'Best Effort' for d in clouds)
    assert all(d['Enabled'] == d['Topic']['Value'].endswith('/points_filtered') for d in clouds)
    assert not any(d['Class'] in ('rviz_default_plugins/Map','rviz_default_plugins/Odometry') for d in rendered['Displays'])
    geometry = preview_geometry(config)
    sides = ['left','right'] if mode == 'all' else [mode]
    assert [t['child'] for t in geometry['transforms']] == ['lidar_'+side for side in sides]
    assert geometry['world_tracking'] is False
    assert geometry['scene']['native_maps_modified'] is False
    for side, transform in zip(sides, geometry['transforms']):
        assert np.allclose(transform['translation'], config['mounts'][side]['t_axle_lidar_m'])


def test_explicit_mapping_keeps_native_chain_and_temporary_capture(setup):
    config = controller.configuration(setup)
    assert config['mapping_enabled'] is True
    commands = '\n'.join(' '.join(map(str,c)) for c in controller.plan(setup,config)['commands'])
    assert 'bag record' in commands and 'mapping_app.launch.py' in commands
    assert 'mapping_preview' not in commands and 'wc_imu.ros_node' in commands


def test_preview_configuration_reports_no_active_icp_requirement(setup):
    request = dict(setup, mapping_enabled=False)
    path = controller.Path(request['config_path'])
    value = json.loads(path.read_text()); value['odometry_source'] = 'icp'
    path.write_text(json.dumps(value))
    result = controller.check_configuration(request)
    assert result['operation'] == 'live_preview' and result['icp_odometry_required'] is False
