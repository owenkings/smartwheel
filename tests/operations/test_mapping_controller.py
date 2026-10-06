"""Pure app configuration, process-plan and export tests; never open devices."""
import copy
import gc
import importlib.util
import json
from pathlib import Path
import shutil
import sqlite3
from types import SimpleNamespace
import uuid

import numpy as np
import pytest
import yaml

from wc_runtime import mapping_controller as controller
from wc_runtime.mapping_monitor import grid_arrays, native_guess_values


PROJECT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('policy', ['periodic', 'on_close'])
def test_checkpoint_policy_is_frozen_and_reported_by_configuration(setup, policy):
    path = Path(setup['config_path'])
    value = json.loads(path.read_text()); value['archive_checkpoint_policy'] = policy
    path.write_text(json.dumps(value))
    config = controller.configuration(setup)
    assert config['archive_checkpoint_policy'] == policy
    assert controller.check_configuration(setup)['archive_checkpoint_policy'] == policy
    assert controller.check_configuration(setup)['manual_controls']['arm_allowed'] is True


@pytest.mark.parametrize('defect', ['manifest', 'storage'])
def test_final_archive_rejects_mixed_checkpoint_policies(tmp_path, monkeypatch, defect):
    handle, status, checkpoint, save = closed_input_fixture(tmp_path, monkeypatch, 'right')
    status['archive_checkpoint_policy'] = 'on_close'
    checkpoint['checkpoint_policy'] = 'on_close'
    status['storage']['checkpoint_policy'] = 'on_close'
    status['storage']['checkpoint'] = copy.deepcopy(checkpoint)
    save()
    assert controller.input_archive_summary(handle)['archived_outputs'] > 0
    if defect == 'manifest':
        checkpoint['checkpoint_policy'] = 'periodic'
        status['storage']['checkpoint'] = copy.deepcopy(checkpoint)
    else:
        status['storage']['checkpoint_policy'] = 'periodic'
    save()
    with pytest.raises(RuntimeError, match='policy'):
        controller.input_archive_summary(handle)


def test_unknown_checkpoint_policy_never_silently_disables_sync(setup):
    path = Path(setup['config_path'])
    value = json.loads(path.read_text()); value['archive_checkpoint_policy'] = 'off'
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError, match='archive_checkpoint_policy'):
        controller.configuration(setup)


def test_main_entry_disables_all_continuous_mapping_shutdown_gates(setup):
    change_config(setup, lambda c: c.update(continuous_mapping=False))
    config = controller.configuration(setup)
    assert config['continuous_mapping'] is True
    assert config['prior_template']['continuous_mapping'] is True
    summary = controller.check_configuration(setup)
    assert summary['runtime_limits'] == {
        'duration_s': None, 'archive_outputs': None,
        'stationary_startup_required': False, 'data_freshness_shutdown': False}
    planned = controller.plan(setup, config)
    assert planned['duration_s'] == 0
    commands = planned['commands']
    assert any('--wait-config-seconds' in command and
               command[command.index('--wait-config-seconds')+1] == '0' for command in commands)
    assert any('--stale-timeout' in command and
               command[command.index('--stale-timeout')+1] == '0' for command in commands)
    assert any('source_stale_seconds:=0' in command for command in commands)
    config['manual_controls'] = None
    passive = controller.plan(setup, config)['commands'][6]
    assert 'wc_motion.feedback_transport' in passive and '--continuous-preview' in passive


@pytest.fixture
def tmp_path():
    # Ordinary workspace mkdir also works in the managed Windows test sandbox;
    # no hardware, user map directory or pytest global temp directory is used.
    parent=Path(__file__).absolute().parent
    directory=parent/('.mapping_controller_test_'+uuid.uuid4().hex)
    directory.mkdir()
    try:
        yield directory
    finally:
        # sqlite3 context managers finish transactions but connection cycles
        # may await collection; release fixture handles before Windows cleanup.
        gc.collect()
        resolved=directory.resolve()
        assert resolved.parent==parent and resolved.name.startswith('.mapping_controller_test_')
        shutil.rmtree(resolved)


@pytest.fixture
def setup(tmp_path, monkeypatch):
    """Historical schema1 process-plan fixture, explicitly offline and synthetic.

    These regression cases exercise archived matrix/plan contracts; the current
    V7 UNKNOWN default and live-start gate have separate test_v7_geometry tests.
    """
    config_dir = tmp_path / 'config'
    config_dir.mkdir()
    for name in ('mapping_live.json', 'live_unvalidated.json',
                 'wheel_feedback_current.json', 'wheel_history_calibration.json', 'hardware_setup.json','cameras.json'):
        source = (PROJECT/'config/calibration/legacy_hardware_setup_before_v7.json'
                  if name=='hardware_setup.json' else PROJECT/'config'/name)
        value = json.loads(source.read_text(encoding='utf-8'))
        if name=='mapping_live.json':
            value['offline_experiment']=True
        (config_dir / name).write_text(json.dumps(value), encoding='utf-8')
    (config_dir/'rviz').mkdir()
    shutil.copy2(PROJECT/'config/rviz/mapping_live.rviz',config_dir/'rviz/mapping_live.rviz')
    monkeypatch.setattr(controller, 'ROOT', tmp_path)
    monkeypatch.setattr(controller, 'RUN', tmp_path / '.phase1_runtime')
    monkeypatch.setattr(controller, 'ticks', lambda pid: 'SYNTHETIC_START_TICKS')

    def no_subprocess(*args, **kwargs):
        raise AssertionError('A pure controller test must not start any process')

    monkeypatch.setattr(controller.subprocess, 'Popen', no_subprocess)
    monkeypatch.setattr(controller.subprocess, 'run', no_subprocess)
    request = {'mode': 'all', 'session_id': 'mapping_contract',
               'project_root': str(tmp_path), 'output_dir': str(tmp_path / 'reports/maps/new'),
               'config_path': str(config_dir / 'mapping_live.json'),
               'duration_s': 80, 'owner_pid': 12345, 'mapping_enabled': True}
    return request


def change_config(request, modify):
    path = Path(request['config_path'])
    config = json.loads(path.read_text(encoding='utf-8'))
    modify(config)
    path.write_text(json.dumps(config), encoding='utf-8')


@pytest.mark.parametrize('source',['wheel_imu','icp'])
@pytest.mark.parametrize('model',['planar_ekf','se3_gyro'])
def test_explicit_odometry_source_reaches_launch_prior_archive_and_view(setup,source,model):
    change_config(setup,lambda c:c.update(odometry_source=source,motion_model=model,
                                       map_profile=c['map_profile'] if model=='planar_ekf' else None))
    config=controller.configuration(setup)
    plan=controller.plan(setup,config)
    assert config['prior_template']['odometry_source']==source
    assert 'odometry_source:='+source in ' '.join(map(str,plan['commands'][1]))
    assert ('/wc_mapping/app/odom_info' in plan['bag_topics'])==(source=='icp')
    assert controller.check_configuration(setup)['icp_odometry_required']==(source=='icp')
    view=yaml.safe_load(controller.render_view(config))
    displays={d.get('Topic',{}).get('Value'):d for d in view['Visualization Manager']['Displays']}
    label='平面 EKF：轮速 + IMU' if model=='planar_ekf' else '轮速 + IMU 里程计'
    assert displays['/wc_mapping/app/odom']['Name']==(label if source=='wheel_imu' else 'ICP 里程计')
    if source=='wheel_imu':
        assert config['prior_template']['wheel_imu_covariance']==config['wheel_imu_covariance']
        assert not displays['/wc_mapping/app/prior_odom']['Enabled']


@pytest.mark.parametrize('source',['auto','wheel',None,False,{},[]])
def test_invalid_odometry_source_never_falls_back_to_icp(setup,source):
    change_config(setup,lambda c:c.update(odometry_source=source))
    with pytest.raises(ValueError,match='odometry_source'):controller.configuration(setup)


@pytest.mark.parametrize('renderer', ['software','system'])
def test_renderer_override_only_affects_owned_gui_environment(setup,monkeypatch,renderer):
    change_config(setup,lambda config:config.update(rviz_renderer=renderer))
    config=controller.configuration(setup)
    directory=Path(setup['output_dir']);directory.mkdir(parents=True)
    (directory/'runtime_config.json').write_text(json.dumps(config),encoding='utf-8')
    desktop={'DISPLAY':':fixture','LIBGL_ALWAYS_SOFTWARE':'0','__GLX_VENDOR_LIBRARY_NAME':'fixture'}
    base={'ROS_DOMAIN_ID':'83'}
    monkeypatch.setattr(controller,'desktop_environment',lambda:dict(desktop))
    monkeypatch.setattr(controller,'environment',lambda:dict(base))
    before=dict(controller.os.environ)
    result=controller.rviz_environment({'directory':str(directory)})
    expected=dict(desktop,**base)
    if renderer=='software':expected.update(LIBGL_ALWAYS_SOFTWARE='1',__GLX_VENDOR_LIBRARY_NAME='mesa')
    assert result==expected and dict(controller.os.environ)==before
    assert desktop['LIBGL_ALWAYS_SOFTWARE']=='0' and base=={'ROS_DOMAIN_ID':'83'}
    assert controller.check_configuration(setup)['rviz_renderer']==renderer


@pytest.mark.parametrize('renderer', ['automatic','',None,True,{},[]])
def test_invalid_renderer_rejected_before_start(setup,renderer):
    change_config(setup,lambda config:config.update(rviz_renderer=renderer))
    with pytest.raises(ValueError,match='rviz_renderer'):controller.configuration(setup)


def change_hardware(request, modify):
    path=Path(request['project_root'])/'config/hardware_setup.json'
    value=json.loads(path.read_text(encoding='utf-8'))
    modify(value)
    path.write_text(json.dumps(value),encoding='utf-8')


def test_storage_reads_actual_session_filesystem_without_using_project_root(setup, monkeypatch):
    directory = Path(setup['output_dir']); directory.mkdir(parents=True)
    seen = []
    def usage(path):
        seen.append(path)
        return SimpleNamespace(free=2 * 1024**3)
    monkeypatch.setattr(controller.shutil, 'disk_usage', usage)
    status = controller.storage_status({'directory': directory})
    assert seen == [directory]
    assert status['path'] == str(directory)
    assert status['free_bytes'] == status['stop_free_bytes'] == 2 * 1024**3
    assert 'not guaranteed' in status['policy']


@pytest.mark.parametrize('failure', ['missing', 'read_error', 'invalid'])
def test_storage_read_failure_never_falls_back_to_root(setup, monkeypatch, failure):
    directory = Path(setup['output_dir'])
    if failure != 'missing': directory.mkdir(parents=True)
    seen = []
    def usage(path):
        seen.append(path)
        if failure == 'read_error': raise OSError('synthetic filesystem unavailable')
        return SimpleNamespace(free=-1)
    monkeypatch.setattr(controller.shutil, 'disk_usage', usage)
    with pytest.raises(RuntimeError, match='STORAGE_CHECK_FAILED'):
        controller.storage_status({'directory': directory})
    assert seen == ([] if failure == 'missing' else [directory])


def test_capacity_estimate_uses_nearest_existing_destination_parent_and_is_only_advisory(setup, monkeypatch):
    parent = Path(setup['output_dir']).parent; parent.mkdir(parents=True)
    seen = []
    monkeypatch.setattr(controller.shutil, 'disk_usage',
                        lambda path: (seen.append(path) or SimpleNamespace(free=5_850_000_000)))
    single = controller.storage_estimate({**setup, 'mode': 'left', 'duration_s': 600})
    dual = controller.storage_estimate({**setup, 'mode': 'all', 'duration_s': 600})
    assert seen == [parent, parent] and not Path(setup['output_dir']).exists()
    assert dual['estimated_recording_bytes'] == 2 * single['estimated_recording_bytes']
    assert single['estimated_recording_bytes'] == 3_960_000_000
    assert dual['warning'] and dual['estimate_only'] and '不保证' in dual['message']


@pytest.mark.parametrize('mode', ['left', 'right', 'all'])
def test_configuration_binds_identity_and_explicit_experiment_prior(setup, mode):
    request = {**setup, 'mode': mode}
    before = Path(request['config_path']).read_bytes()
    config = controller.configuration(request)
    assert config['mode'] == mode
    assert config['session_id'] == request['session_id']
    assert config['source_mode'] == 'real'
    assert config['formal_acceptance'] is False
    assert config['navigation_validated'] is False
    assert config['wheel_candidate']['formal_odometry_eligible'] is False
    assert config['wheel_device_id'] == 'ZLAC8030D-0000000014'
    prior = config['prior_template']
    assert prior['reference_frame'] == 'mapping_reference'
    assert prior['guess_frame_id'] == 'prior_odom'
    assert prior['private_tf_topic'] == '/wc_mapping/app/tf'
    assert prior['prior_odom_topic'] == '/wc_mapping/app/prior_odom'
    assert prior['input_cloud_topic'] != prior['output_cloud_topic']
    assert prior['imu_sensor_id'] == config['imu_sensor_id']
    assert Path(request['config_path']).read_bytes() == before
    assert not Path(request['output_dir']).exists()


@pytest.mark.parametrize('value', [np.eye(2), np.eye(3) * 2,
    np.diag([1., 1., -1.]), [[1, .2, 0], [0, 1, 0], [0, 0, 1]],
    [[float('nan'), 0, 0], [0, 1, 0], [0, 0, 1]],
    [[1, 0, 0], [0, float('inf'), 0], [0, 0, 1]]])
def test_rotation_rejects_scale_reflection_shear_and_nonfinite(value):
    with pytest.raises(ValueError):
        controller.rotation(value, 'test mounting')


def test_rotation_accepts_proper_nonidentity_measurement():
    rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    np.testing.assert_array_equal(controller.rotation(rotation, 'test mounting'), rotation)


@pytest.mark.parametrize('mode', ['left', 'right', 'all'])
def test_right_mount_is_required_independently_of_left_imu(setup, mode):
    change_hardware(setup, lambda config: config['lidars']['right'].pop('rpy_deg'))
    with pytest.raises(ValueError):
        controller.configuration({**setup, 'mode': mode})


def test_old_duplicate_installation_fields_cannot_silently_override_single_file(setup):
    change_config(setup, lambda config: config.update(mounts={}))
    with pytest.raises(ValueError, match='hardware_setup_config'):
        controller.configuration(setup)


def test_imu_moved_to_right_with_rotation_is_used_in_every_mode(setup):
    from wc_runtime.hardware_setup import rpy_rotation
    from wc_runtime.mapping_input import mount_transforms
    change_hardware(setup, lambda config: config['imu'].update(
        parent_frame='lidar_right', rpy_deg=[20., -7., 90.]))
    for mode in ('left','right','all'):
        config=controller.configuration({**setup,'mode':mode})
        expected=np.asarray(config['mounts']['right']['R_axle_lidar'])@rpy_rotation([20.,-7.,90.])
        np.testing.assert_allclose(config['imu_mount']['R_axle_imu'],expected,atol=1e-12)
        frames=mount_transforms(config,[0.,0.,9.81])
        np.testing.assert_allclose(frames['R_base_imu'],np.asarray(frames['T_base_axle'])[:3,:3]@expected,atol=1e-12)


def test_new_wheel_value_is_shared_by_prior_and_session_preview(setup):
    change_hardware(setup,lambda config:config['wheel_odometry'].update(wheel_radius_m=.21))
    config=controller.configuration(setup)
    assert config['wheel_candidate']['wheel_radius_m']==.21
    assert config['prior_template']['wheel_candidate']['wheel_radius_m']==.21
    task=controller.plan(setup,config)
    command=next(c for c in task['commands'] if 'wc_runtime.mapping_wheel' in c)
    assert str(command[command.index('--session-root')+1])==setup['output_dir']
    assert not any('wc_motion.feedback_transport' in c for c in task['commands'])


def test_check_configuration_performs_no_preflight_hardware_or_output_write(setup,monkeypatch):
    def forbidden(*args,**kwargs):raise AssertionError('hardware preflight called')
    monkeypatch.setattr(controller,'preflight',forbidden)
    monkeypatch.setattr(controller,'target',forbidden)
    monkeypatch.setattr(controller,'imu_preflight',forbidden)
    result=controller.check_configuration(setup)
    assert result['status']=='CONFIG_VALID_EXPERIMENT' and result['hardware_started'] is False
    assert not Path(setup['output_dir']).exists()


def test_session_freezes_exact_setup_and_wheel_parameters_before_future_edits(setup):
    import hashlib
    config=controller.configuration(setup)
    path=Path(setup['project_root'])/'config/hardware_setup.json'
    original=path.read_bytes()
    change_hardware(setup,lambda value:value['wheel_odometry'].update(wheel_radius_m=.22))
    directory=Path(setup['output_dir']);directory.mkdir(parents=True)
    frozen=controller.archive_configuration(directory,config)
    assert (directory/'hardware_setup.json').read_bytes()==original
    assert frozen['hardware_setup_provenance']['sha256']==hashlib.sha256(original).hexdigest()
    wheel=json.loads((directory/'wheel_candidate.json').read_text(encoding='utf-8'))
    assert wheel==frozen['wheel_candidate']==frozen['prior_template']['wheel_candidate']
    assert wheel['wheel_radius_m']==.165
    assert controller.configuration(setup)['wheel_candidate']['wheel_radius_m']==.22
    assert '_hardware_setup_raw_text' not in frozen
    assert json.loads((directory/'runtime_config.json').read_text(encoding='utf-8'))==frozen
    with pytest.raises(FileExistsError):controller.archive_configuration(directory,config)


@pytest.mark.parametrize('mode,source_mode,sides', [
    ('left', 'single_left', ('left',)), ('right', 'single_right', ('right',)),
    ('all', 'dual', ('left', 'right'))])
def test_plan_records_only_selected_sources_and_private_processing(setup, mode, source_mode, sides):
    request = {**setup, 'mode': mode}
    result = controller.plan(request, controller.configuration(request))
    topics = result['bag_topics']
    assert len(topics) == len(set(topics))
    source_topics = {topic for topic in topics if topic.startswith('/wc_mapping/lidar_')}
    assert source_topics == {'/wc_mapping/lidar_' + side + '/' + leaf
        for side in sides for leaf in ('source_frame', 'source_frame_filtered', 'diagnostics')}
    assert {'/wc_mapping/imu/source_frame', '/wc_mapping/wheel/feedback_raw',
            '/wc_mapping/app/prior_odom', '/wc_mapping/app/scan_cloud',
            '/wc_mapping/app/cloud_map', '/wc_mapping/app/grid_map',
            '/wc_mapping/app/tf', '/wc_mapping/app/tf_static'} <= set(topics)
    assert all(topic.startswith('/wc_mapping/') for topic in topics)
    assert '/tf' not in topics and '/tf_static' not in topics
    argv = [arg for command in result['commands'] for arg in command]
    assert 'source_mode:=' + source_mode in argv
    assert 'device_config_policy:=preserve_current' in argv
    assert 'max_runtime_seconds:=0' in argv
    assert 'wc_runtime.mapping_input' in argv
    assert 'wc_runtime.mapping_prior' in argv
    assert 'wc_runtime.mapping_monitor' in argv
    assert 'wc_imu.ros_node' in argv
    assert 'wc_runtime.mapping_wheel' in argv
    assert 'wc_motion.feedback_transport' not in argv
    assert controller.configuration(request)['manual_controls']['arm_allowed'] is True
    assert not any('manual_control' in arg or arg in ('-a', '--all') for arg in argv)
    assert set(topics) <= set(result['commands'][0])  # Recorder starts before publishers.
    assert 'record' in result['commands'][0]
    assert any(arg.endswith('mapping_app.launch.py') for arg in result['commands'][1])
    assert 'session_root:=' + request['output_dir'] in result['commands'][1]
    assert {'sensor_owner.lock', 'domain-83-source.lock', 'domain-83-processing.lock',
            'mapping-app.lock'} <= set(result['locks'])
    assert result['sigint_grace_s'] == 120
    assert result['allow_component_exit'] is False
    assert result['duration_s'] == 0
    assert result['owner_start_ticks'] == 'SYNTHETIC_START_TICKS'
    assert result['environment']['ROS_DOMAIN_ID'] == '83'
    assert not Path(request['output_dir']).exists()


@pytest.mark.parametrize('mode', ['left', 'right', 'all'])
@pytest.mark.parametrize('manual_adapter', [False, True])
@pytest.mark.parametrize('duration', [0, 1, 59, 2401, 86400])
def test_supervisor_owns_duration_without_finite_child_limits(setup, mode, manual_adapter, duration):
    from wc_runtime.mapping_app import prepare_request
    request = prepare_request(mode, project_root=setup['project_root'],
                              config=setup['config_path'], duration_s=duration, mapping_enabled=True)
    config = controller.configuration(request)
    if not manual_adapter:
        config.pop('manual_controls')
    result = controller.plan(request, config)
    assert result['duration_s'] == 0 and result['allow_component_exit'] is False
    assert result['sigint_grace_s'] == 120
    assert set(result['locks']) == set(controller.plan({**request, 'duration_s': 80}, config)['locks'])
    for command in result['commands']:
        if '--duration' in command:
            assert command[command.index('--duration')+1] == '0'
        if 'wc_motion.feedback_transport' in command:
            assert command[command.index('--samples')+1] == '0'
            assert command[command.index('--max-duration')+1] == '0'
            assert '--allow-read-queries' in command and '--async-journal' in command
        if 'wc_runtime.mapping_wheel' in command:
            assert config['manual_controls']['arm_allowed'] is True
    assert 'max_runtime_seconds:=0' in [word for command in result['commands'] for word in command]
    assert result['bag_topics'] == controller.plan({**request, 'duration_s': 80}, config)['bag_topics']


def test_until_close_capacity_estimate_is_unknown_not_zero(setup, monkeypatch):
    monkeypatch.setattr(controller.shutil, 'disk_usage', lambda path: SimpleNamespace(free=5*1024**3))
    result = controller.storage_estimate({**setup, 'duration_s': 0})
    assert result['estimated_recording_bytes'] is None and result['duration_s'] == 0
    assert result['stop_free_bytes'] == 2*1024**3


@pytest.mark.parametrize('owner_lost', [False, True])
def test_until_close_owner_watch_survives_old_sixty_second_deadline(setup, monkeypatch, owner_lost):
    clock, calls = [0.], []
    plan = {**setup, 'duration_s': 0, 'owner_start_ticks': 'SYNTHETIC_START_TICKS'}
    def read(root, path):
        if path.name == 'plan.json': return plan
        return {'state': 'STOPPED' if clock[0] >= 150000 and not owner_lost else 'RUNNING'}
    def identity(pid):
        calls.append(('identity', clock[0]))
        if owner_lost and clock[0] >= 100000: raise ProcessLookupError('synthetic owner exit')
        return 'SYNTHETIC_START_TICKS'
    monkeypatch.setattr(controller, 'read_json', read)
    monkeypatch.setattr(controller, 'ticks', identity)
    monkeypatch.setattr(controller, 'stop_registered', lambda path: calls.append(('stop', clock[0])))
    monkeypatch.setattr(controller.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(controller.time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0]+50000))
    assert controller.watch(Path(setup['project_root'])/'synthetic_runtime') == 0
    assert ('identity', 100000) in calls
    assert [event for event in calls if event[0] == 'stop'] == ([('stop', 100000)] if owner_lost else [])


@pytest.mark.parametrize('source',['wheel_imu','icp'])
def test_app_launch_and_rviz_use_the_same_private_frame_chain(setup,source):
    request = setup
    change_config(setup,lambda c:c.update(odometry_source=source))
    directory = Path(request['output_dir'])
    (directory / 'slam').mkdir(parents=True)
    launch_path = PROJECT / 'src/wc_bringup/launch/mapping_app.launch.py'
    spec = importlib.util.spec_from_file_location('_controller_launch_contract', launch_path)
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    native = launch.build_spec(directory, project_root=Path(request['project_root']),odometry_source=source)
    slam = native['nodes'][-1]['parameters'][0]
    prior = controller.configuration(request)['prior_template']
    assert slam['frame_id'] == prior['reference_frame']
    if source=='icp':
        odom=native['nodes'][0]['parameters'][0]
        assert odom['frame_id']==prior['reference_frame']
        assert odom['guess_frame_id']==prior['guess_frame_id']
        assert odom['odom_frame_id']=='mapping_odom'
    else:
        assert len(native['nodes'])==1 and not slam['subscribe_odom_info']
    assert slam['odom_frame_id'] == ''
    assert slam['odom_frame_id_init'] == 'mapping_odom'
    for node in native['nodes']:
        assert dict(node['remappings'])['/tf'] == prior['private_tf_topic']
    command = controller.rviz_command({'directory':str(directory)})
    assert '/tf:=/wc_mapping/app/tf' in command
    assert '/tf_static:=/wc_mapping/app/tf_static' in command
    assert str(directory/'view.rviz') in command
    rviz = yaml.safe_load((PROJECT / 'config/rviz/mapping_live.rviz').read_text(encoding='utf-8'))
    assert rviz['Visualization Manager']['Global Options']['Fixed Frame'] == slam['map_frame_id']


def test_camera_snapshot_labels_and_process_share_exact_config(setup):
    import hashlib
    source=Path(setup['project_root'])/'config/cameras.json'
    cameras=json.loads(source.read_text())
    cameras['cameras'][0]['port'],cameras['cameras'][1]['port']=4,1
    source.write_text(json.dumps(cameras),encoding='utf-8')
    raw=source.read_bytes()
    config=controller.configuration(setup)
    view=yaml.safe_load(controller.render_view(config))
    panel=next(p for p in view['Panels'] if p['Class']=='wc_camera_panel/CameraPanel')
    assert panel['left_front USB Port']==4 and panel['right_front USB Port']==1
    assert panel['Compact Layout'] is True
    # A subsequent user edit does not retarget the already resolved session.
    source.write_text('{}',encoding='utf-8')
    directory=Path(setup['output_dir']);directory.mkdir(parents=True)
    frozen=controller.archive_configuration(directory,config)
    assert (directory/'cameras_config.json').read_bytes()==raw
    assert frozen['cameras_provenance']['sha256']==hashlib.sha256(raw).hexdigest()
    assert '_cameras_raw_text' not in frozen
    task=controller.plan(setup,frozen)
    command=next(c for c in task['commands'] if 'wc_runtime.mapping_cameras' in c)
    assert command[command.index('--config')+1]==str(directory/'cameras_config.json')
    assert not any('/cameras/' in topic for topic in task['bag_topics'])


def test_camera_preview_can_be_disabled_without_removing_mapping(setup):
    change_config(setup,lambda c:c.update(cameras_enabled=False))
    config=controller.configuration(setup)
    view=yaml.safe_load(controller.render_view(config))
    assert not any(p['Class']=='wc_camera_panel/CameraPanel' for p in view['Panels'])
    task=controller.plan(setup,config)
    assert not any('wc_runtime.mapping_cameras' in c for c in task['commands'])
    assert any('wc_runtime.mapping_visuals' in c for c in task['commands'])
    assert '/wc_mapping/app/grid_map' in task['bag_topics']
    assert '/wc_mapping/app/view_grid' not in task['bag_topics']


def test_failed_session_reports_original_component_reason(setup):
    directory=Path(setup['output_dir']);(directory/'input').mkdir(parents=True)
    (directory/'input/status.json').write_text(json.dumps({'failure_reason':'ARCHIVE_QUEUE_FULL'}))
    (directory/'health').mkdir()
    (directory/'health/status.json').write_text(json.dumps({'failure':'odom stale'}))
    runtime=Path(setup['project_root'])/'runtime';runtime.mkdir()
    (runtime/'manifest.json').write_text(json.dumps({'state':'FAILED','exit_code':1,'stop_reason':'component_exit:123:1'}))
    result=controller.inspect({'runtime':str(runtime),'directory':str(directory)})
    assert result['component_failures']=={'input':'ARCHIVE_QUEUE_FULL','monitor':'odom stale'}


def grid_snapshot(tmp_path, data, **changes):
    metadata = {'width': data.shape[1], 'height': data.shape[0], 'resolution': .03,
                'origin': [-2.5, 1.25, -.52], 'frame_id': 'mapping_map'}
    metadata.update(changes)
    source = tmp_path / 'grid.npz'
    np.savez(source, data=data, metadata=json.dumps(metadata))
    output = tmp_path / 'export'
    output.mkdir()
    return source, output


def test_pgm_preserves_ros_bottom_left_origin_by_flipping_rows(tmp_path):
    # First array row is low Y; PGM first row is high Y. Include threshold edges.
    source, output = grid_snapshot(tmp_path, np.array([
        [-1, 0, 19, 20], [64, 65, 100, 0]], dtype=np.int16))
    result = controller.save_grid(source, output)
    header, dimensions, depth, payload = (output / 'map_2d.pgm').read_bytes().split(b'\n', 3)
    assert (header, dimensions, depth) == (b'P5', b'4 2', b'255')
    assert payload == bytes([205, 0, 0, 254, 205, 254, 254, 205])
    spec = yaml.safe_load((output / 'map_2d.yaml').read_text(encoding='utf-8'))
    assert spec['image'] == 'map_2d.pgm'
    assert spec['origin'] == [-2.5, 1.25, 0.0]  # Third field is yaw, not sensor Z.
    assert spec['resolution'] == .03 and spec['negate'] == 0
    assert spec['mode'] == 'trinary'
    assert spec['free_thresh'] == .196 and spec['occupied_thresh'] == .65
    assert result['occupied_cells'] == 2
    assert result['width'] == 4 and result['height'] == 2


def test_grid_shape_mismatch_is_rejected_before_writing(tmp_path):
    source, output = grid_snapshot(tmp_path, np.array([[0, 100]], dtype=np.int16), height=2)
    with pytest.raises(RuntimeError, match='shape'):
        controller.save_grid(source, output)
    assert not list(output.iterdir())


@pytest.mark.parametrize('resolution', [0, -1, float('nan'), float('inf')])
def test_invalid_grid_resolution_is_rejected_before_writing(tmp_path, resolution):
    source, output = grid_snapshot(tmp_path, np.array([[0, 100]], dtype=np.int16), resolution=resolution)
    with pytest.raises((RuntimeError, ValueError)):
        controller.save_grid(source, output)
    assert not list(output.iterdir())


def test_grid_export_never_overwrites_existing_image(tmp_path):
    source, output = grid_snapshot(tmp_path, np.array([[0, 100]], dtype=np.int16))
    (output / 'map_2d.pgm').write_bytes(b'original image')
    with pytest.raises(FileExistsError):
        controller.save_grid(source, output)
    assert (output / 'map_2d.pgm').read_bytes() == b'original image'


@pytest.mark.parametrize('state', [
    {'state': 'RUNNING', 'exit_code': None},
    {'state': 'FAILED', 'exit_code': 1},
    {'state': 'STOPPED', 'exit_code': 0, 'cleanup_errors': ['forced kill']}])
def test_export_refuses_unclosed_or_failed_session_before_io(setup, monkeypatch, state):
    monkeypatch.setattr(controller, 'inspect', lambda handle: copy.deepcopy(state))
    with pytest.raises(RuntimeError, match='异常退出'):
        controller.export({'directory': setup['output_dir']})
    assert not Path(setup['output_dir']).exists()


def test_stop_waits_for_final_manifest_not_only_signal_delivery(setup, monkeypatch):
    root = Path(setup['project_root'])
    runtime = root / '.phase1_runtime/sessions/mapping_contract/mapping_app'
    runtime.mkdir(parents=True)
    manifest = runtime / 'manifest.json'
    manifest.write_text(json.dumps({'state': 'RUNNING', 'role': 'mapping_app',
                                    'sigint_grace_s': 30}), encoding='utf-8')
    observed = iter([{'state': 'RUNNING'}, {'state': 'STOPPED', 'exit_code': 0}])
    signalled = []
    slept = []
    monkeypatch.setattr(controller, 'stop_registered', lambda path: signalled.append(path))
    monkeypatch.setattr(controller, 'inspect', lambda handle: next(observed))
    monkeypatch.setattr(controller.time, 'sleep', lambda seconds: slept.append(seconds))
    result = controller.stop({'runtime': str(runtime)})
    assert signalled == [manifest]
    assert slept == [.2]
    assert result == {'state': 'STOPPED', 'exit_code': 0}


def native_transform(quaternion=(0., 0., 0., 1.), position=(0., 0., 0.)):
    return SimpleNamespace(translation=SimpleNamespace(**dict(zip(('x','y','z'), position))),
                           rotation=SimpleNamespace(**dict(zip(('x','y','z','w'), quaternion))))


def test_native_null_guess_is_distinct_from_valid_identity_and_movement():
    assert native_guess_values(native_transform((0., 0., 0., 0.))) is None
    assert native_guess_values(native_transform()) == [0., 0., 0., 0., 0., 0., 1.]
    assert native_guess_values(native_transform(position=(.1, 0., 0.)))[:3] == [.1, 0., 0.]
    assert native_guess_values(native_transform((0., 0., 0., -1.)))[-1] == -1.


@pytest.mark.parametrize('quaternion', [(0., 0., 0., 2.),
                                     (0., float('nan'), 0., 1.),
                                     (float('inf'), 0., 0., 0.)])
def test_native_invalid_guess_is_never_counted_as_valid(quaternion):
    with pytest.raises(ValueError):
        native_guess_values(native_transform(quaternion))


def ros_grid():
    return SimpleNamespace(header=SimpleNamespace(frame_id='mapping_map'), data=[-1, 0, 100, 40],
        info=SimpleNamespace(width=2, height=2, resolution=.03,
            origin=SimpleNamespace(position=SimpleNamespace(x=-2., y=1., z=0.),
                orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))))


def test_monitor_grid_preserves_values_and_metric_origin():
    data, metadata = grid_arrays(ros_grid())
    np.testing.assert_array_equal(data, [[-1, 0], [100, 40]])
    assert metadata['origin'] == [-2., 1., 0.]
    assert metadata['resolution'] == .03


@pytest.mark.parametrize('part,field,value', [
    ('info', 'resolution', float('nan')), ('info', 'resolution', 0.),
    ('position', 'x', float('inf')), ('orientation', 'w', float('nan')),
    ('orientation', 'w', 0.), ('orientation', 'z', .1)])
def test_monitor_grid_rejects_invalid_units_or_origin(part, field, value):
    message = ros_grid()
    target = message.info if part == 'info' else getattr(message.info.origin, part)
    setattr(target, field, value)
    with pytest.raises(ValueError):
        grid_arrays(message)


def closed_input_fixture(tmp_path, monkeypatch, mode='all', discarded=0, accepted=2):
    """Small fabricated disk artifacts exercise validation, never ROS or devices."""
    directory=tmp_path/'session';root=directory/'input';root.mkdir(parents=True)
    monkeypatch.setattr(controller,'ROOT',tmp_path)
    handle={'directory':str(directory),'runtime':str(tmp_path/'runtime'),'session_id':'archive_fixture','mode':mode}
    sides=('left','right') if mode=='all' else (mode,)
    prefix={'archived_outputs':accepted,'sources':{},'artifacts':[]}
    source_status={}
    for side in sides:
        (root/side/'frames').mkdir(parents=True)
        rows=[]
        for number in range(1,accepted+1):
            relative='frames/%08d.cdr'%number
            payload=b'fabricated CDR fixture '+side.encode()+number.to_bytes(4,'little')
            (root/side/relative).write_bytes(payload)
            rows.append({'archive_index':number,'side':side,'raw_key':[handle['session_id'],side,'fixture_epoch',number],
                         'file':relative,'cdr_bytes':len(payload)})
        index=root/side/'frames.jsonl'
        index.write_bytes(b''.join(controller.json_bytes(row).replace(b'\n',b'')+b'\n' for row in rows))
        prefix['sources'][side]={'frames':accepted,'index_bytes':index.stat().st_size}
        source_status[side]={'state':'STOPPED','failure_reason':None,'archived_frames':accepted,
                             'published_frames':accepted-discarded}
    pairs=[{'archive_output_index':number,'session_id':handle['session_id'],'mode':mode,
            'sources':[{'side':side,'cdr_file':side+'/frames/%08d.cdr'%number} for side in sides]}
           for number in range(1,accepted+1)]
    index=root/'pairs.jsonl'
    index.write_bytes(b''.join(json.dumps(row).encode()+b'\n' for row in pairs))
    prefix['pairs_index_bytes']=index.stat().st_size
    for path in (root/'bootstrap.json',directory/'prior_config.json'):
        path.write_text('{}\n',encoding='utf-8');prefix['artifacts'].append(str(path))
    checkpoint={**copy.deepcopy(prefix),'schema_version':1,'kind':'FSYNCED_MAPPING_ARCHIVE_PREFIX',
                'generation':3,'final':True,'checkpoint_record_monotonic_ns':1000,
                'checkpoint_interval_s':1.0,'crash_loss_bound_s':None}
    status={'schema_version':1,'kind':'MAPPING_INPUT_EXPERIMENT','session_id':handle['session_id'],
            'mode':mode,'source_mode':'real','state':'STOPPED','failure_reason':None,'bootstrap_complete':True,
            'pending_outputs':0,'pending_reserved_bytes':0,'enqueued_outputs':accepted,'archived_outputs':accepted,
            'published_frames':accepted-discarded,'outputs_discarded_on_close':discarded,
            'archived_frames':accepted*len(sides),'sources':source_status,
            'storage':{'written_prefix':copy.deepcopy(prefix),'checkpoint':copy.deepcopy(checkpoint),
                       'accepted_outputs_not_checkpointed':0,'error':None}}
    def save():
        (root/'status.json').write_bytes(controller.json_bytes(status))
        (root/'checkpoint.json').write_bytes(controller.json_bytes(checkpoint))
    save()
    return handle,status,checkpoint,save


@pytest.mark.parametrize('mode', ['left','right','all'])
def test_final_archive_check_accepts_complete_selected_sources_without_rehashing_cdr(tmp_path,monkeypatch,mode):
    handle,status,checkpoint,save=closed_input_fixture(tmp_path,monkeypatch,mode,discarded=1)
    original=controller.hash_file;hashed=[]
    def audit_hash(path):
        assert path.suffix!='.cdr','CDR contents must not be reread during export'
        hashed.append(path);return original(path)
    monkeypatch.setattr(controller,'hash_file',audit_hash)
    before={path:path.read_bytes() for path in Path(handle['directory']).rglob('*') if path.is_file()}
    summary=controller.input_archive_summary(handle)
    assert summary['status']=='FINAL_CHECKPOINT_VERIFIED'
    assert summary['accepted_outputs']==summary['archived_outputs']==2
    assert summary['published_outputs']==summary['archived_but_not_published_on_close']==1
    assert summary['checkpoint_final'] is True and summary['pending_outputs']==0
    assert summary['cdr_contents_rehashed'] is False and summary['cdr_file_sizes_verified'] is True
    assert set(summary['sources'])==({'left','right'} if mode=='all' else {mode})
    assert hashed==[Path(handle['directory'])/'input/checkpoint.json']
    assert all(path.read_bytes()==data for path,data in before.items())


def test_final_archive_accepts_real_file_index_beyond_former_12000_limit(tmp_path, monkeypatch):
    handle, _, _, _ = closed_input_fixture(tmp_path, monkeypatch, 'right', accepted=12001)
    summary = controller.input_archive_summary(handle)
    assert summary['accepted_outputs'] == summary['archived_outputs'] == 12001
    assert summary['status'] == 'FINAL_CHECKPOINT_VERIFIED'


@pytest.mark.parametrize('defect', [
    'not_stopped','failure','pending','pending_bytes','accepted_mismatch','unexplained_unpublished',
    'not_final','checkpoint_count','embedded_manifest','written_prefix','storage_error','uncheckpointed',
    'wrong_source_count','wrong_source_set','wrong_session','missing_artifact','boolean_count'])
def test_export_rejects_incomplete_input_before_creating_output(tmp_path,monkeypatch,defect):
    handle,status,checkpoint,save=closed_input_fixture(tmp_path,monkeypatch)
    if defect=='not_stopped':status['state']='RUNNING'
    elif defect=='failure':status['failure_reason']='STORAGE_CHECKPOINT_FAILED'
    elif defect=='pending':status['pending_outputs']=1
    elif defect=='pending_bytes':status['pending_reserved_bytes']=100
    elif defect=='accepted_mismatch':status['enqueued_outputs']=3
    elif defect=='unexplained_unpublished':status['published_frames']=1
    elif defect=='not_final':checkpoint['final']=False
    elif defect=='checkpoint_count':checkpoint['archived_outputs']=1
    elif defect=='embedded_manifest':status['storage']['checkpoint']['generation']=2
    elif defect=='written_prefix':status['storage']['written_prefix']['pairs_index_bytes']+=1
    elif defect=='storage_error':status['storage']['error']='fsync failed'
    elif defect=='uncheckpointed':status['storage']['accepted_outputs_not_checkpointed']=1
    elif defect=='wrong_source_count':status['sources']['right']['archived_frames']=1
    elif defect=='wrong_source_set':status['sources'].pop('right')
    elif defect=='wrong_session':status['session_id']='other'
    elif defect=='missing_artifact':(Path(handle['directory'])/'prior_config.json').unlink()
    elif defect=='boolean_count':status['pending_outputs']=False
    save()
    monkeypatch.setattr(controller,'inspect',lambda handle:{'state':'STOPPED','exit_code':0})
    def forbidden(*args,**kwargs):raise AssertionError('Export process must not start with incomplete input')
    monkeypatch.setattr(controller.subprocess,'run',forbidden)
    before={path:path.read_bytes() for path in Path(handle['directory']).rglob('*') if path.is_file()}
    with pytest.raises(RuntimeError,match='输入归档最终同步验收失败'):
        controller.export(handle)
    assert not (Path(handle['directory'])/'export').exists()
    assert all(path.read_bytes()==data for path,data in before.items())


@pytest.mark.parametrize('defect', ['pair_truncated','pair_suffix','source_truncated','wrong_rows',
    'wrong_pair_member','missing_cdr','truncated_cdr','extra_cdr','wrong_source_session'])
def test_checkpoint_cannot_cover_changed_or_missing_archive_files(tmp_path,monkeypatch,defect):
    handle,status,checkpoint,save=closed_input_fixture(tmp_path,monkeypatch)
    root=Path(handle['directory'])/'input'
    if defect=='pair_truncated':
        path=root/'pairs.jsonl';path.write_bytes(path.read_bytes()[:-1])
    elif defect=='pair_suffix':
        with (root/'pairs.jsonl').open('ab') as stream:stream.write(b'{}\n')
    elif defect=='source_truncated':
        path=root/'right/frames.jsonl';path.write_bytes(path.read_bytes()[:-1])
    elif defect=='wrong_rows':
        path=root/'left/frames.jsonl';raw=path.read_bytes().replace(b'\n',b' ',1);path.write_bytes(raw)
    elif defect=='wrong_pair_member':
        path=root/'pairs.jsonl';path.write_bytes(path.read_bytes().replace(b'left/frames/00000001',b'left/frames/00000002'))
    elif defect=='missing_cdr':(root/'right/frames/00000002.cdr').unlink()
    elif defect=='truncated_cdr':(root/'left/frames/00000001.cdr').write_bytes(b'truncated')
    elif defect=='extra_cdr':(root/'left/frames/00000003.cdr').write_bytes(b'orphan')
    elif defect=='wrong_source_session':
        path=root/'right/frames.jsonl';path.write_bytes(path.read_bytes().replace(b'archive_fixture',b'another_fixture'))
    with pytest.raises((RuntimeError,ValueError),match='验收失败|Extra data'):
        controller.input_archive_summary(handle)


@pytest.mark.parametrize('capacity', ['enough', 'exact', 'low', 'read_error'])
@pytest.mark.parametrize('odometry_source', ['wheel_imu','icp'])
def test_export_report_includes_final_archive_evidence(tmp_path,monkeypatch,capacity,odometry_source):
    handle,status,checkpoint,save=closed_input_fixture(tmp_path,monkeypatch,mode='left')
    directory=Path(handle['directory']);runtime=Path(handle['runtime']);runtime.mkdir()
    monkeypatch.setattr(controller,'inspect',lambda handle:{'state':'STOPPED','exit_code':0})
    (directory/'health').mkdir()
    counts={name:1 for name in ('odom','odom_info','prior_odom','grid_map','cloud_map')}
    if odometry_source=='wheel_imu':counts.pop('odom_info')
    (directory/'runtime_config.json').write_text(json.dumps({'odometry_source':odometry_source}),encoding='utf-8')
    (directory/'health/status.json').write_text(json.dumps({'failure':None,'counts':counts,'odometry_source':odometry_source}),encoding='utf-8')
    (directory/'slam').mkdir();db=directory/'slam/rtabmap.db'
    with sqlite3.connect(db) as connection:
        connection.execute('CREATE TABLE Node(id INTEGER)');connection.execute('INSERT INTO Node VALUES(1)')
    (runtime/'process-1.log').write_text(str(db)+' Saving database/long-term memory...done!\n',encoding='utf-8')
    (directory/'bag').mkdir()
    with sqlite3.connect(directory/'bag/fixture.db3') as connection:connection.execute('CREATE TABLE fixture(id INTEGER)')
    topics=['/wc_mapping/imu/source_frame','/wc_mapping/wheel/feedback_raw',
            '/wc_mapping/lidar_left/source_frame','/wc_mapping/lidar_left/source_frame_filtered']
    topics+=['/wc_mapping/app/'+name for name in counts]
    metadata={'rosbag2_bagfile_information':{'relative_file_paths':['fixture.db3'],
              'topics_with_message_count':[{'topic_metadata':{'name':topic},'message_count':1} for topic in topics]}}
    (directory/'bag/metadata.yaml').write_text(yaml.safe_dump(metadata),encoding='utf-8')
    native_calls=[]
    def fake_native_export(*args,**kwargs):
        native_calls.append(args)
        output=directory/'export'
        (output/'map_3d.ply').write_bytes(b'ply\nformat ascii 1.0\nelement vertex 1\nend_header\n0 0 0\n')
        (output/'map.pgm').write_bytes(b'P5\n4 4\n255\n'+bytes([205])*16)
        (output/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':.03,'origin':[0.,0.,0.]}),encoding='utf-8')
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(controller.subprocess,'run',fake_native_export)
    before=db.read_bytes()
    required=db.stat().st_size + 512 * 1024**2
    checked=[]
    def usage(path):
        checked.append(path)
        if capacity=='read_error':raise OSError('synthetic export statvfs failure')
        return SimpleNamespace(free=required + {'enough':1,'exact':0,'low':-1}[capacity])
    monkeypatch.setattr(controller.shutil,'disk_usage',usage)
    if capacity in ('low','read_error'):
        reason='EXPORT_INSUFFICIENT_FREE_SPACE' if capacity=='low' else 'STORAGE_CHECK_FAILED'
        with pytest.raises(RuntimeError,match=reason):controller.export(handle)
        assert checked==[directory] and not native_calls
        assert not (directory/'export').exists() and db.read_bytes()==before
        return
    result=controller.export(handle)
    assert result['odometry_source']==odometry_source
    assert result['icp_odometry_required']==(odometry_source=='icp')
    assert checked==[directory] and len(native_calls)==1
    assert result['export_storage_check']['required_free_bytes']==required
    assert result['export_storage_check']['export_extra_free_bytes']==512 * 1024**2
    assert result['input_archive']['status']=='FINAL_CHECKPOINT_VERIFIED'
    assert result['input_archive']['checkpoint_generation']==checkpoint['generation']
    assert result['input_archive']['sources']['left']['frames']==2
    assert json.loads((directory/'export/result.json').read_text())['input_archive']==result['input_archive']
    assert db.read_bytes()==before
