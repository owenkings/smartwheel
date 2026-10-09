"""Selected-scope acquisition without ultrasound; no hardware is opened."""
import json
from pathlib import Path
import shutil
import sys
from types import SimpleNamespace

import pytest

from wc_runtime import capture, capture_audit, cli, runtime_health, source_recorder
from wc_runtime.source_archive import CameraArchive, atomic_json, digest
from wc_cameras.config import ROLES
from test_capture import fixture_capture

PROJECT = Path(__file__).resolve().parents[2]
CAMERAS = {'camera_'+role for role in ROLES}


def test_camera_scope_and_commands_exclude_ultrasound_without_changing_all_sensors(tmp_path):
    selected = capture.source_selection('mapping_cameras')
    assert set(selected['selected_sources']) == {'lidar_left', 'lidar_right', 'imu', 'wheel'} | CAMERAS
    assert selected['excluded_sources'] == ['ultrasonic']
    bindings = json.loads((PROJECT/'config/device_bindings.json').read_text(encoding='utf-8'))
    for directory in (tmp_path/'capture', tmp_path/'all'):
        (directory/'configuration').mkdir(parents=True)
        atomic_json(directory/'configuration/device_bindings.json', bindings)
    commands = capture.source_commands(PROJECT, tmp_path/'run', tmp_path/'capture', 'test', 'mapping_cameras')
    assert set(commands) == {'lidar', 'imu', 'wheel'} | CAMERAS
    assert 'ultrasonic' not in json.dumps(commands)
    assert 'mapping_wheel' not in json.dumps(commands) and 'mapping_prior' not in json.dumps(commands)
    assert capture.capacity('mapping_cameras', 30, 10**12) == capture.capacity('all_sensors', 30, 10**12)
    assert capture.capacity('mapping_cameras', 30, 10**12)['estimated_recording_bytes'] > capture.capacity('mapping_core', 30, 10**12)['estimated_recording_bytes']
    assert 'ultrasonic' in capture.source_commands(PROJECT, tmp_path/'run', tmp_path/'all', 'test', 'all_sensors')


def mock_preflight(monkeypatch, *, missing_camera=None):
    from wc_motion import feedback_transport
    from wc_cameras import capture as camera_capture
    from wc_runtime import ultrasonic_capture
    monkeypatch.setattr(cli, 'device_preflight', lambda: None)
    monkeypatch.setattr(cli, 'imu_preflight', lambda: {})
    monkeypatch.setattr(feedback_transport, 'verify_identity', lambda cfg: (Path('/mock/wheel'), 'identity'))
    monkeypatch.setattr(feedback_transport, 'require_unoccupied', lambda path: None)
    monkeypatch.setattr(camera_capture, 'require_unoccupied', lambda path: None)
    def camera_identity(camera):
        if camera['role'] == missing_camera: raise FileNotFoundError('selected camera absent')
        return {'resolved_node': '/mock/camera'}
    monkeypatch.setattr(camera_capture, 'verify_device', camera_identity)
    def forbidden(*args, **kwargs):
        raise AssertionError('excluded ultrasound must never be inspected')
    monkeypatch.setattr(ultrasonic_capture, 'identity', forbidden)
    monkeypatch.setattr(ultrasonic_capture, 'validate_config', forbidden)
    monkeypatch.setattr(capture.shutil, 'disk_usage', lambda path: SimpleNamespace(free=10**12))


@pytest.mark.parametrize('missing_camera', [None, 'left_front'])
def test_preflight_checks_all_requested_cameras_but_never_ultrasound(monkeypatch, missing_camera):
    mock_preflight(monkeypatch, missing_camera=missing_camera)
    result = capture.preflight(PROJECT, 'mapping_cameras', 30)
    assert 'ultrasonic' not in result
    assert CAMERAS <= set(result)
    assert result['camera_left_front']['status'] == ('BLOCKED' if missing_camera else 'AVAILABLE')
    expected=capture.capacity('mapping_cameras',30,10**12,initialization_budget_s=capture.ALL_SOURCE_READY_TIMEOUT_S)
    assert result['capacity']['estimated_recording_bytes']==expected['estimated_recording_bytes']
    assert result['capacity']['source_initialization_budget_s']==capture.ALL_SOURCE_READY_TIMEOUT_S


def test_snapshot_cameras_scope_does_not_require_ultrasound_configuration(tmp_path,monkeypatch):
    from wc_runtime import capture_support
    def software_snapshot(project,output):
        # This test isolates profile/config selection, without querying git or
        # dpkg in an intentionally synthetic project outside any repository.
        path=output/'software/synthetic_snapshot.json';path.parent.mkdir()
        atomic_json(path,{'synthetic':True})
        return {'software/synthetic_snapshot.json':digest(path)}
    monkeypatch.setattr(capture_support,'software_snapshot',software_snapshot)
    root = tmp_path/'project'; cfg = root/'config'; cfg.mkdir(parents=True)
    for filename in ('mapping_live.json', 'hardware_setup.json', 'cameras.json', 'wheel_feedback_current.json'):
        shutil.copyfile(PROJECT/'config'/filename, cfg/filename)
    (cfg/'calibration').mkdir()
    shutil.copytree(PROJECT/'src/wc_xt_driver/config', root/'src/wc_xt_driver/config')
    output = tmp_path/'capture'; output.mkdir()
    hashes = capture.snapshot(root, output, profile='mapping_cameras')
    assert not (output/'configuration/ultrasonic.json').exists()
    assert (output/'configuration/cameras.json').is_file()
    selection = json.loads((output/'configuration/source_selection.json').read_text())
    assert selection['profile'] == 'mapping_cameras' and selection['excluded_sources'] == ['ultrasonic']
    assert hashes['configuration/source_selection.json'] == digest(output/'configuration/source_selection.json')
    assert hashes['software/synthetic_snapshot.json']==digest(output/'software/synthetic_snapshot.json')


def test_recorder_subscriptions_are_profile_scoped():
    assert source_recorder.topics_for_profile('mapping_cameras') == source_recorder.TOPICS
    assert '/wc_mapping/ultrasonic/feedback_raw' not in source_recorder.topics_for_profile('mapping_cameras')
    assert '/wc_mapping/ultrasonic/feedback_raw' in source_recorder.topics_for_profile('all_sensors')
    with pytest.raises(ValueError): source_recorder.topics_for_profile('typo')


@pytest.mark.parametrize('fault', [None, 'camera_absent', 'camera_bytes', 'camera_exit'])
def test_integrity_requires_every_selected_camera_but_ignores_excluded_ultrasound(tmp_path, fault):
    fixture_capture(tmp_path)
    for role in ROLES:
        archive = CameraArchive(tmp_path/'sources/cameras'/role, role, 'epoch')
        archive.append(bytes(range(12)), width=2, height=2, sequence=1, preview_rotation_deg=0)
        archive.close()
    results = dict.fromkeys({'recorder', 'lidar', 'imu', 'wheel'} | CAMERAS, 0)
    # Even an unrelated failed old status is outside the explicit request scope.
    results['ultrasonic'] = 2
    (tmp_path/'sources/ultrasonic').mkdir()
    (tmp_path/'sources/ultrasonic/summary.json').write_text('{invalid ultrasound metadata')
    camera_dir = tmp_path/'sources/cameras/left_front'
    if fault == 'camera_absent': (camera_dir/'summary.json').unlink()
    elif fault == 'camera_bytes':
        summary = json.loads((camera_dir/'summary.json').read_text())
        (camera_dir/summary['chunks'][0]['path']).write_bytes(b'corrupt')
    elif fault == 'camera_exit': results['camera_left_front'] = 1
    result = capture_audit.audit_capture(tmp_path, 'mapping_cameras', results)
    assert result['recording_complete'] is (fault is None)
    assert 'ultrasonic' not in result['source_accounting']
    assert result['excluded_sources'] == ['ultrasonic']
    assert not any('ULTRASONIC' in issue['code'] for issue in result['issues'])
    if fault is None:
        assert CAMERAS <= set(result['source_accounting'])
        assert all(result['source_accounting'][camera]['retained'] == 1 for camera in CAMERAS)
        legacy = capture_audit.audit_capture(tmp_path, 'all_sensors', results)
        assert not legacy['recording_complete'] and any(issue['code'].startswith('ULTRASONIC') for issue in legacy['issues'])


def test_health_and_summary_never_claim_all_sensors_complete(tmp_path):
    manifest = dict(session_id='test', profile='mapping_cameras', status='COMPLETE', recording_complete=True,
        issues=[], source_accounting={name: dict(successfully_received=1, persisted=1, retained=1) for name in CAMERAS},
        manual_drive=False, control_transmissions=0, control_count_status='FINAL_SOURCE_COUNTER',
        **capture.source_selection('mapping_cameras'))
    atomic_json(tmp_path/'capture_manifest.json', manifest)
    health = runtime_health.collect_session_health(tmp_path)
    assert health['capabilities']['mapping_cameras_recording']['status'] == 'COMPLETE'
    assert health['capabilities']['all_sensors_recording']['status'] == 'NOT_REQUESTED'
    assert health['capabilities']['ultrasonic_recording']['status'] == 'NOT_REQUESTED'
    assert health['recording']['excluded_sources'] == ['ultrasonic']
    assert '不宣称全部传感器完整' in capture.capture_summary(manifest)
    manifest.update(recording_complete=False, status='PARTIAL', issues=[dict(code='CAMERA_ARCHIVE_INVALID', evidence='sources/cameras/left_front', detail='missing')])
    atomic_json(tmp_path/'capture_manifest.json', manifest)
    health = runtime_health.collect_session_health(tmp_path)
    camera_issue = next(row for row in health['issues'] if row['code'] == 'CAMERA_ARCHIVE_INVALID')
    assert camera_issue['capability_impact'] == ['mapping_cameras_recording']


def test_audit_cannot_change_the_frozen_profile_to_bypass_required_sources(tmp_path):
    fixture_capture(tmp_path)
    (tmp_path/'configuration').mkdir()
    atomic_json(tmp_path/'configuration/source_selection.json',
                dict(profile='all_sensors', **capture.source_selection('all_sensors')))
    result = capture_audit.audit_capture(tmp_path, 'mapping_core',
                                       dict.fromkeys(('recorder', 'lidar', 'imu', 'wheel'), 0))
    assert not result['recording_complete']
    assert any(row['code'] == 'CAPTURE_SOURCE_SELECTION_MISMATCH' for row in result['issues'])


def test_doctor_and_capture_accept_new_profile_without_starting_devices(tmp_path, monkeypatch, capsys):
    root = tmp_path/'project'
    config = root/'config'
    config.mkdir(parents=True)
    archive = tmp_path/'archive'
    (archive/'data/experiments').mkdir(parents=True)
    for name in ('live_unvalidated.json', 'hardware_setup.json'):
        shutil.copyfile(PROJECT/'config'/name, config/name)
    atomic_json(config/'storage.json', dict(schema_version=2, backend='directory',
        archive_root=str(archive), fallback_allowed=False))
    monkeypatch.setattr(cli, 'target', lambda: None)
    monkeypatch.setattr(cli, 'ROOT', root)
    monkeypatch.setattr(capture, 'preflight', lambda *args, **kwargs: dict(capacity={'sufficient': True}))
    monkeypatch.setattr(capture.shutil, 'disk_usage', lambda path: SimpleNamespace(free=10**12))
    assert cli.main(['doctor', '--profile', 'mapping_cameras']) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['capture_profile'] == 'mapping_cameras'
    assert report['capture_source_selection']['excluded_sources'] == ['ultrasonic']
    assert capture.main(['--profile', 'mapping_cameras', '--session', 'scope_test', '--duration', '30', '--staging', 'disk', '--dry-run']) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['excluded_sources'] == ['ultrasonic'] and report['control_transmissions'] == 0
