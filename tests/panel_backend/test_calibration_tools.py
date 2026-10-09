"""Offline tool ownership, source selection and write-boundary checks."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
from unittest import mock

import pytest

from wc_panel import calibration_tools as module
from wc_panel.calibration_tools import CalibrationTools, owns_loopback_listener


def prepared(root):
    source = root/'reports/example/prepared.json'
    source.parent.mkdir(parents=True)
    source.write_text(json.dumps(dict(schema_version=1, status='PREPARED_NOT_VALIDATED',
        training=[dict(id='fixture')], source_mode='synthetic')), encoding='utf-8')
    return source


def pointer(root, source, **overrides):
    path = root/'.phase1_runtime/state/point_picker_input.json'
    path.parent.mkdir(parents=True, exist_ok=True)
    value = dict(schema_version=1, status='READY_FOR_OFFLINE_PICKING',
                 prepared_input=source.relative_to(root).as_posix(),
                 prepared_file_sha256=hashlib.sha256(source.read_bytes()).hexdigest(), scene_id='fixture')
    value.update(overrides)
    path.write_text(json.dumps(value), encoding='utf-8')


def test_defaults_resolve_only_verified_pointer(tmp_path):
    source = prepared(tmp_path)
    pointer(tmp_path, source)
    service = CalibrationTools(tmp_path)
    result = service.defaults()
    assert result['input'] == str(source) and result['error'] == ''
    source.write_text(source.read_text() + ' ', encoding='utf-8')
    result = service.defaults()
    assert result['input'] == '' and '校验值' in result['error']


@pytest.mark.parametrize('relative', ['../prepared.json', '/etc/input.json', 'C:/input.json', 'config/prepared.json'])
def test_bad_pointer_does_not_fallback(tmp_path, relative):
    source = prepared(tmp_path)
    pointer(tmp_path, source, prepared_input=relative)
    result = CalibrationTools(tmp_path).defaults()
    assert not result['input'] and result['error']


def test_bad_scene_does_not_fallback(tmp_path):
    source = prepared(tmp_path)
    pointer(tmp_path, source, scene_id='other-scene')
    assert '场景' in CalibrationTools(tmp_path).defaults()['error']


def test_legacy_pointer_migrates_and_conflict_is_visible(tmp_path):
    source = prepared(tmp_path)
    pointer(tmp_path, source)
    current = tmp_path/'.phase1_runtime/state/point_picker_input.json'
    legacy = tmp_path/'state/point_picker_input.json'
    legacy.parent.mkdir()
    original = current.read_bytes()
    current.rename(legacy)
    service = CalibrationTools(tmp_path)
    assert service.defaults()['input'] == str(source)
    assert current.read_bytes() == original and not legacy.exists()
    legacy.write_bytes(b'{"conflicting_selection": true}')
    result = service.defaults()
    assert result['input'] == '' and '迁移失败' in result['error']
    assert current.read_bytes() == original
    assert legacy.read_bytes() == b'{"conflicting_selection": true}'


def test_explicit_input_remains_inside_policy(tmp_path):
    project = tmp_path/'project'; project.mkdir()
    source = prepared(tmp_path)
    with pytest.raises(ValueError, match='OUTSIDE_ALLOWED_ROOTS'):
        CalibrationTools(project)._input(source)


def test_configured_storage_failure_is_visible_without_fallback(tmp_path):
    service = CalibrationTools(tmp_path)
    service.policy = mock.Mock()
    service.policy.resolve.side_effect = ValueError('USB unavailable')
    result = service.defaults()
    assert result['error'] == 'USB unavailable' and result['input'] == ''
    assert not (tmp_path/'data').exists()


@pytest.mark.parametrize('url', ['http://192.168.5.10:8888/', 'https://127.0.0.1:8888/',
    'http://127.0.0.1:8888/other', 'http://user@127.0.0.1:8888/', 'http://127.0.0.1:8888/?x=1'])
def test_readiness_rejects_nonlocal_or_wrong_route(url):
    assert not owns_loopback_listener(1000, url, 'points')


def test_readiness_requires_process_socket_ownership(tmp_path):
    process = tmp_path/'123'; (process/'fd').mkdir(parents=True); (process/'net').mkdir()
    (process/'fd/3').touch()
    (process/'net/tcp').write_text('header\n 0: 0100007F:22B8 00000000:0000 0A 0 0 0 0 0 8877\n')
    real_path = Path
    with mock.patch.object(module, 'Path', side_effect=lambda path: tmp_path if path == '/proc' else real_path(path)):
        with mock.patch.object(module.os, 'readlink', return_value='socket:[8877]'):
            assert owns_loopback_listener(123, 'http://127.0.0.1:8888/', 'points')
        with mock.patch.object(module.os, 'readlink', return_value='socket:[1111]'):
            assert not owns_loopback_listener(123, 'http://127.0.0.1:8888/', 'points')


def test_status_keeps_failure_instead_of_successful_exit(tmp_path):
    service = CalibrationTools(tmp_path)
    process = mock.Mock(); process.poll.return_value = 0
    service._tasks['ours'] = dict(process=process, row=dict(id='ours', status='FAILED', error='USB lost', url=''))
    assert service.status('ours')['status'] == 'FAILED'


def test_stop_never_signals_unknown_task(tmp_path):
    service = CalibrationTools(tmp_path)
    with pytest.raises(ValueError, match='不属于'):
        service.stop('unrelated')


def test_stop_failure_does_not_report_complete(tmp_path):
    service = CalibrationTools(tmp_path)
    process = mock.Mock(); process.poll.return_value = None
    service._tasks['ours'] = dict(process=process, row=dict(id='ours', status='RUNNING', url='http://127.0.0.1:8888/',
        input=str(tmp_path/'prepared.json'), output_root=str(tmp_path/'candidates'), log_path=str(tmp_path/'log')))
    with mock.patch.object(service, '_terminate', side_effect=subprocess.TimeoutExpired('fixture', 2)):
        row = service.stop('ours')
    assert row['status'] == 'FAILED' and '停止失败' in row['error']
    assert row['process_alive'] and len(service.active_paths()) == 3
    process.poll.return_value = 0


def test_active_paths_cover_source_candidate_and_log(tmp_path):
    service = CalibrationTools(tmp_path)
    process = mock.Mock(); process.poll.return_value = None
    paths = dict(input=str(tmp_path/'prepared.json'), output_root=str(tmp_path/'candidates'), log_path=str(tmp_path/'log'))
    service._tasks['ours'] = dict(process=process, row=dict(id='ours', status='RUNNING', **paths))
    assert set(service.active_paths()) == {Path(value) for value in paths.values()}
    process.poll.return_value = 0
    assert service.active_paths() == []


@pytest.mark.parametrize('mode', ['points', 'alignment'])
def test_both_browser_pages_export_roots_are_protected(tmp_path, mode):
    service = CalibrationTools(tmp_path)
    process = mock.Mock(); process.poll.return_value = None
    roots = [str(tmp_path/'data/calibration'/name) for name in ('manual_points', 'alignment_candidates')]
    service._tasks['ours'] = dict(process=process, row=dict(id='ours', status='RUNNING', mode=mode,
        input=str(tmp_path/'prepared.json'), log_path=str(tmp_path/'log'),
        output_root=roots[0 if mode == 'points' else 1], candidate_roots=roots))
    assert all(Path(path) in service.active_paths() for path in roots)
    process.poll.return_value = 0


@pytest.mark.skipif(os.name != 'posix', reason='Existing runtime picker imports Linux fcntl; exercised on target')
def test_picker_exports_check_guard_each_time(tmp_path, monkeypatch):
    from wc_calibration import picker
    from wc_runtime import calibration_picker
    policy = mock.Mock()
    original = picker.PickerSession
    checked = []

    def runtime_main(arguments):
        assert arguments[-1] == '--alignment'
        session = object.__new__(picker.PickerSession)
        # A failing guard must run before inherited export touches any source.
        policy.check.side_effect = ValueError('USB lost')
        with pytest.raises(ValueError, match='USB lost'):
            session.export({})
        with pytest.raises(ValueError, match='USB lost'):
            session.alignment_workspace()
        checked.append(True)
        return 0

    monkeypatch.setattr(calibration_picker, 'main', runtime_main)
    assert module.guarded_picker(policy, os.getppid(), 'fixture.json', 'alignment') == 0
    assert checked and picker.PickerSession is original
