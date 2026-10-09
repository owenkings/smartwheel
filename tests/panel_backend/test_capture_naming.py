"""Human dataset names remain separate from authenticated runtime identities."""
from datetime import datetime
import copy
import hashlib
import json
import os
from pathlib import Path
import threading
import sqlite3
from types import SimpleNamespace as NS
from unittest import mock

import pytest

from wc_panel.backend import PanelBackend
from wc_runtime.capture_names import available_folder, recording_folder_name, validate_folder_name
from wc_runtime import capture


INSTANT = datetime(2026, 10, 8, 9, 7, 5)


def options(prefix='走廊_往返', **flags):
    return dict(prefix=prefix, year=flags.get('year', True), date=flags.get('date', True), time=flags.get('time', True))


def test_chinese_prefix_and_ordered_local_time_suffixes():
    assert recording_folder_name(options(), INSTANT) == '走廊_往返_2026_1008_090705'
    assert recording_folder_name(options(year=False), INSTANT) == '走廊_往返_1008_090705'
    assert recording_folder_name(options(date=False, time=False), INSTANT) == '走廊_往返_2026'
    assert recording_folder_name(options('', year=False, date=False), INSTANT) == '090705'
    assert recording_folder_name(options(year=False, date=False, time=False), INSTANT) == '走廊_往返'


@pytest.mark.parametrize('name', ['', ' ', '.', '..', '../escape', 'x/y', 'x\\y', 'C:foo',
    'foo\nbar', 'a\x00b', 'a\u202eb', 'NUL', 'con.txt', 'COM1', '.hidden', 'tail.', '前缀'*80])
def test_empty_escape_control_reserved_or_oversized_folder_rejected(name):
    with pytest.raises(ValueError):
        validate_folder_name(name)


@pytest.mark.parametrize('value', [options('', year=False, date=False, time=False),
    options('../escape'), dict(prefix='x', year=1, date=True, time=True), dict(prefix='x')])
def test_invalid_naming_options_rejected(value):
    with pytest.raises(ValueError):
        recording_folder_name(value, INSTANT)


def test_existing_files_and_queued_names_never_reused(tmp_path):
    (tmp_path/'场景').mkdir()
    (tmp_path/'场景_02').write_bytes(b'preserved original')
    queued = tmp_path/'场景_03'
    result = available_folder(tmp_path, '场景', [queued])
    assert result == tmp_path/'场景_04'
    assert not result.exists() and not queued.exists()
    assert (tmp_path/'场景_02').read_bytes() == b'preserved original'


def backend_fixture(tmp_path, monkeypatch):
    backend = PanelBackend.__new__(PanelBackend)
    backend.project_root = tmp_path
    backend._lock = threading.RLock()
    backend._recover_profiles = mock.Mock()
    backend.device_parameters = mock.Mock(return_value={'revision': 'frozen-current'})
    backend._cli = lambda operation, *arguments: [operation, *arguments]
    backend._manager = mock.Mock()
    protected = []
    submissions = []
    backend._manager.active_paths.side_effect = lambda: list(protected)
    def submit(*args, **kwargs):
        submissions.append((args, kwargs))
        protected.extend(kwargs['protected_paths'])
        return dict(id='job'+str(len(submissions)), status='QUEUED', output=str(kwargs['output']))
    backend._manager.submit.side_effect = submit
    monkeypatch.setattr('wc_panel.backend.resolve_user_destination',
        lambda root, path: (Path(path), NS(required_uuid=None, check=lambda: None)))
    return backend, submissions


def test_named_capture_queues_unique_output_with_independent_ascii_session(tmp_path, monkeypatch):
    backend, submissions = backend_fixture(tmp_path, monkeypatch)
    naming = options('实验甲', year=False, date=False, time=False)
    first = backend.start_capture(tmp_path, 'left', name_options=naming)
    second = backend.start_capture(tmp_path, 'right', name_options=naming)
    assert Path(first['output']).name == '实验甲'
    assert Path(second['output']).name == '实验甲_02'
    for (args, kwargs), side in zip(submissions, ('left', 'right')):
        task = args[2][0]
        command = task['commands'][0]
        session = command[command.index('--session')+1]
        folder = command[command.index('--folder-name')+1]
        assert session.isascii() and session.startswith('capture_') and session != folder
        assert command[command.index('--sides')+1] == side
        assert command[command.index('--staging')+1] == 'memory'
        assert '--manual-drive' in command
        assert task['session_id'] == session and task['folder_name'] == folder
        assert Path(task['progress_paths'][0]).parent.name == session
        assert Path(task['progress_paths'][1]).parent == Path(kwargs['output'])
        assert kwargs['protected_paths'] == [kwargs['output']]
        assert kwargs['snapshot']['revision'] == 'frozen-current'


def test_legacy_capture_retains_old_name_and_readonly_calibration_mode(tmp_path, monkeypatch):
    backend, submissions = backend_fixture(tmp_path, monkeypatch)
    backend.start_capture(tmp_path, manual_drive=False)
    args, kwargs = submissions[0]
    task = args[2][0]
    command = task['commands'][0]
    assert '--folder-name' not in command and '--manual-drive' not in command
    assert Path(kwargs['output']).name == command[command.index('--session')+1]


def test_invalid_name_never_submits_or_creates_output(tmp_path, monkeypatch):
    backend, submissions = backend_fixture(tmp_path, monkeypatch)
    with pytest.raises(ValueError):
        backend.start_capture(tmp_path, name_options=options('../escape'))
    assert submissions == [] and list(tmp_path.iterdir()) == []


def test_memory_archive_retains_session_and_raw_bytes_under_chinese_folder(tmp_path, monkeypatch):
    staged = tmp_path/'ram/capture_ascii'; staged.mkdir(parents=True)
    output = tmp_path/'disk/走廊_2026_1008_090705'; output.mkdir(parents=True)
    (staged/'bag').mkdir(); (staged/'bag/raw.db3').write_bytes(b'immutable recorded source')
    manifest = dict(session_id='capture_ascii', folder_name=output.name, status='RECORDING', recording_complete=False)
    (staged/'capture_manifest.json').write_text(json.dumps(manifest), encoding='utf-8')
    monkeypatch.setattr(capture.shutil, 'disk_usage', lambda path: NS(free=10**12))
    if os.name == 'nt':
        monkeypatch.setattr(capture, 'sync_capture_directory', lambda path: None)
    result = capture.transfer_capture(staged, output, manifest)
    assert result['status'] == 'PERSISTED_AWAITING_AUDIT'
    assert (output/'bag/raw.db3').read_bytes() == (staged/'bag/raw.db3').read_bytes()
    final = json.loads((output/'capture_manifest.json').read_text(encoding='utf-8'))
    assert final['session_id'] == 'capture_ascii' and final['folder_name'] == output.name
    assert final['status'] == 'PARTIAL' and final['recording_complete'] is False


def test_final_control_receipt_matches_manifest_session_not_display_folder(tmp_path):
    directory = tmp_path/'实验名称'; (directory/'sources').mkdir(parents=True)
    session = 'capture_20261008_090705_abcdef'
    (directory/'capture_manifest.json').write_text(json.dumps(dict(session_id=session)), encoding='utf-8')
    summary = dict(session_id=session, control_transmissions=0, ready_observed=True, recorder_discovered=True,
                   synchronized=True, closed_normally=True, journal=dict(final_fsync_complete=True))
    (directory/'sources/wheel_summary.json').write_text(json.dumps(summary), encoding='utf-8')
    result = capture.final_control_evidence(directory, manual_drive=True, session_id=session)
    assert result['control_count_status'] == 'SOURCE_OWNER_FINAL_SYNCHRONIZED_COUNTER'
    assert 'error' not in result
    assert capture.final_control_evidence(directory, manual_drive=True) == result


@pytest.mark.skipif(os.name == 'nt', reason='capture CLI imports the target POSIX process supervisor')
def test_runtime_name_routes_display_folder_but_keeps_ram_identity(tmp_path, monkeypatch, capsys):
    from wc_runtime import cli
    monkeypatch.setattr(cli, 'ROOT', tmp_path)
    monkeypatch.setattr(cli, 'target', lambda: None)
    monkeypatch.setattr(cli, 'require_device_bindings', lambda: {})
    monkeypatch.setattr(capture, 'preflight', lambda *args, **kwargs: dict(capacity=dict(sufficient=True)))
    monkeypatch.setattr(capture, 'memory_preflight', lambda *args, **kwargs: dict(sufficient=True))
    monkeypatch.setattr(capture, 'MEMORY_STAGING_ROOT', tmp_path/'ram')
    monkeypatch.setattr(capture.subprocess, 'Popen', lambda *a, **k: pytest.fail('dry-run may not start hardware'))
    args = NS(profile='mapping_core', session='capture_ascii', folder_name='走廊_2026_1008_090705',
              staging='memory', duration=5, dry_run=True, diagnostic=False)
    assert capture.run_capture(args) == 0
    result = json.loads(capsys.readouterr().out)
    assert Path(result['output']).name == args.folder_name
    assert Path(result['storage_staging']['live_directory']).name == args.session
    assert list(tmp_path.iterdir()) == []
    args.folder_name = ''
    with pytest.raises(ValueError):
        capture.run_capture(args)


def test_frozen_runtime_without_session_uses_manifest_plus_actual_runtime_identity(tmp_path):
    from wc_runtime.offline_refinement_inputs import recording_session
    source = tmp_path/'中文名称'; (source/'configuration').mkdir(parents=True)
    manifest = dict(session_id='capture_ascii', runtime_config_path='configuration/runtime_config.json')
    (source/'capture_manifest.json').write_text(json.dumps(manifest), encoding='utf-8')
    frozen = source/'configuration/runtime_config.json'
    # This is the actual mapping_core snapshot contract: identify_sources later
    # supplies session_id from SourceFrame; mapping_live itself has no session.
    frozen.write_text(json.dumps(dict(mode='all', imu_sensor_id='imu-id')), encoding='utf-8')
    assert recording_session(source, dict(session_id='capture_ascii')) == 'capture_ascii'
    with pytest.raises(ValueError, match='session'):
        recording_session(source, dict(session_id='other_capture'))
    frozen.write_text(json.dumps(dict(session_id='other_capture')), encoding='utf-8')
    with pytest.raises(ValueError, match='session'):
        recording_session(source, dict(session_id='capture_ascii'))


def test_motion_candidate_verifies_original_ascii_session_under_chinese_dataset(tmp_path):
    from wc_runtime import offline_motion_calibration as cal, offline_refinement_inputs as inputs
    source = tmp_path/'走廊_往返'; (source/'bag').mkdir(parents=True)
    with sqlite3.connect(source/'bag/data.db3') as db:
        db.executescript('CREATE TABLE topics(id INTEGER,name TEXT,type TEXT);'
                         'CREATE TABLE messages(id INTEGER,topic_id INTEGER,data BLOB);')
        db.executemany('INSERT INTO topics VALUES(?,?,?)', [(1, '/wc_mapping/imu/source_frame', 'wc_interfaces/msg/H30Frame'),
            (2, '/wc_mapping/wheel/feedback_raw', 'std_msgs/msg/String')])
        db.executemany('INSERT INTO messages VALUES(?,?,?)', [(1, 1, b'imu'), (2, 2, b'wheel')])
    (source/'capture_manifest.json').write_text(json.dumps(dict(session_id='capture_ascii')), encoding='utf-8')
    (source/'runtime_config.json').write_text(json.dumps(dict(mode='all')), encoding='utf-8')
    original = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in source.rglob('*') if path.is_file()}
    evidence, _, _ = inputs.selected_message_evidence(source)
    candidate = dict(schema=cal.SCHEMA, status=cal.VALIDATED, validation=dict(passed=True, reasons=[]),
        provenance=dict(session_id='capture_ascii', imu_device_id='imu-fixture', wheel_device_id='wheel-fixture',
            R_reference_imu=[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]], wheel_conversion=dict(fixture='synthetic'), sources=[evidence]),
        windows=dict(warmup=[0.,10.], bias_train=[10.,20.], bias_validation=[21.,31.],
                     wheel_train=[32.,72.], wheel_validation=[72.,112.]),
        policy=copy.deepcopy(cal.DEFAULT_POLICY), bias_native_rad_s=[0.,0.,.0005],
        wheel_yaw_scale=.9, wheel_yaw_speed_coefficient=-.01)
    candidate['candidate_id'] = cal._candidate_digest(candidate)
    path = tmp_path/'candidate.json'; path.write_text(json.dumps(candidate), encoding='utf-8')
    result = inputs.verify_candidate(candidate, path, source, {})
    assert result['status'] == 'VERIFIED' and result['session_id'] == 'capture_ascii'
    assert all(hashlib.sha256(path.read_bytes()).hexdigest() == digest for path, digest in original.items())
    value = json.loads((source/'capture_manifest.json').read_text()); value['session_id'] = 'other_capture'
    (source/'capture_manifest.json').write_text(json.dumps(value), encoding='utf-8')
    with pytest.raises(ValueError, match='different session or device'):
        inputs.verify_candidate(candidate, path, source, {})
