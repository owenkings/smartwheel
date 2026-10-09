"""Offline orchestration tests with synthetic audits; no device or SSH access."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

import pytest

from wc_runtime import prepare_picker_input as entry

PROJECT_SOURCE = Path(entry.__file__).resolve().parents[2]


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data), encoding='utf-8')


@pytest.fixture
def project(tmp_path):
    root = tmp_path/'project'
    study = root/'reports/lidar_stability/box_20260913T065406Z'
    for label, sides in entry.STAGES.items():
        directory = study/label
        rows = {}
        for side in sides:
            record = {'flags': {'device_config_policy': 'preserve_current'}}
            rows[side] = {'paired': 20, 'first': record, 'last': record}
            save(directory/(side+'_frames.json'), {'raw': [], 'filtered': []})
            (directory/(side+'.npz')).write_bytes(b'unit test placeholder; mocked preparation only')
        save(directory/'capture.json', {'status': 'CAPTURE_AUDIT_PASS', 'errors': [],
            'session': label+'-synthetic-session', 'sides': rows})
        save(study/(label+'_after.json'), {'registered_live_processes': []})
    save(study/'after.json', {'registered_live_processes': []})
    save(root/entry.POINTER, {'old': 'byte-exact backup expected'})
    return root, study


def prepared_fixture(root, study, output, log):
    """Only the existing preparation subprocess is replaced in orchestration tests."""
    data = {'schema_version': 1, 'status': 'PREPARED_NOT_VALIDATED', 'source_mode': 'real',
        'sensor_ids': {'left': 'SYN-L', 'right': 'SYN-R'}, 'units': 'm',
        'coordinate_conventions': {'left': 'FLU', 'right': 'FLU'},
        'input_preparation': {'study_path': str(study.resolve())},
        'training': [{'id': study.name+'_sequential_single_frames',
            'time_quality': 'NON_SIMULTANEOUS_STATIC_SCENE_ASSUMPTION',
            'left': [[1, 0, 0], [1, 1, 0], [1, 0, 1]],
            'right': [[2, 0, 0], [2, 1, 0], [2, 0, 1]],
            'input_files': {side: {'units': 'm', 'coordinate_convention': 'FLU'}
                            for side in ('left', 'right')}}]}
    entry.write_new(output, entry.json_bytes(data))
    entry.write_new(log, b'Synthetic preparer boundary only\n')


def test_success_validates_real_picker_backs_up_and_commits_manifest(project):
    root, study = project
    previous = (root/entry.POINTER).read_bytes()
    result = entry.prepare_input(root, study, preparer=prepared_fixture)
    pointer = json.loads((root/entry.POINTER).read_text())
    journal = json.loads(Path(result['result_json']).read_text())
    assert result['pointer_updated'] is True and result['status'] == 'READY_FOR_OFFLINE_PICKING'
    assert journal['status'] == 'PREPARED_VALIDATED'
    assert pointer['operation_id'] == journal['operation_id'] == result['operation_id']
    assert Path(result['previous_pointer_backup']).read_bytes() == previous
    assert result['prepared_input'] == str(root/'reports/calibration'/study.name/'prepared.json')
    assert result['point_counts']['left']['selectable_rows'] == 3
    assert set(result['stages']) == set(entry.STAGES)
    assert result['live_eligible'] is False and result['time_quality'].startswith('NON_SIMULTANEOUS')
    assert 'token' not in json.dumps(journal)
    assert result['picker_command'].startswith('bash scripts/pick_lidar_points.sh --input reports/')


@pytest.mark.parametrize('fault', ['preparer_failure', 'invalid_picker', 'wrong_study', 'atomic_replace', 'journal_write'])
def test_failures_never_replace_previous_pointer(project, monkeypatch, fault):
    root, study = project
    previous = (root/entry.POINTER).read_bytes()
    def preparer(*args):
        if fault == 'preparer_failure':
            raise RuntimeError('synthetic preparation failure')
        prepared_fixture(*args)
        if fault in ('invalid_picker', 'wrong_study'):
            output = args[2]
            data = json.loads(output.read_text())
            if fault == 'invalid_picker':
                data['units'] = 'mm'
            else:
                data['input_preparation']['study_path'] = str(study.parent/'another_study')
            output.write_bytes(entry.json_bytes(data))
    if fault == 'atomic_replace':
        monkeypatch.setattr(entry.os, 'replace', lambda *args: (_ for _ in ()).throw(OSError('replace denied')))
    if fault == 'journal_write':
        original = entry.write_new
        def fail_journal(path, data):
            if path.name == 'result.json':
                raise OSError('journal unavailable')
            original(path, data)
        monkeypatch.setattr(entry, 'write_new', fail_journal)
    with pytest.raises((OSError, RuntimeError, ValueError)):
        entry.prepare_input(root, study, preparer=preparer)
    assert (root/entry.POINTER).read_bytes() == previous
    assert list((root/'reports/calibration').rglob('failure.json'))


def test_existing_prepared_is_preserved_on_repeated_explicit_study(project):
    root, study = project
    first = entry.prepare_input(root, study, preparer=prepared_fixture)
    first_bytes = Path(first['prepared_input']).read_bytes()
    second = entry.prepare_input(root, study, preparer=prepared_fixture)
    assert first['prepared_input'] != second['prepared_input']
    assert Path(first['prepared_input']).read_bytes() == first_bytes
    assert Path(second['prepared_input']).parent.name == second['operation_id']


def test_explicit_study_ignores_misleading_latest_pointer(project):
    root, study = project
    latest = root/'reports/recapture/latest_static_comparison.txt'
    latest.parent.mkdir(parents=True)
    latest.write_text('reports/lidar_stability/DO_NOT_USE_20260913T070000Z\n')
    result = entry.prepare_input(root, study.relative_to(root), preparer=prepared_fixture)
    assert result['study'] == str(study)
    returned = entry.study_from_output(root, 'other log\nSTATIC_COMPARISON_COMPLETE '+study.relative_to(root).as_posix()+'\n', 'box')
    assert returned == study
    assert 'DO_NOT_USE' in latest.read_text()


@pytest.mark.parametrize('output', ['', 'STATIC_COMPARISON_COMPLETE reports/lidar_stability/other_20260913T065406Z\n',
    'STATIC_COMPARISON_COMPLETE /tmp/box_20260913T065406Z\n',
    'STATIC_COMPARISON_COMPLETE reports/lidar_stability/box_20260913T065406Z\n'*2])
def test_no_fallback_from_invalid_or_ambiguous_capture_completion(project, output):
    root, _ = project
    with pytest.raises(ValueError):
        entry.study_from_output(root, output, 'box')


@pytest.mark.parametrize('fault', ['failed_stage', 'apply_xtcfg', 'missing_stage', 'outside', 'parent_traversal'])
def test_invalid_study_is_rejected_before_preparer(project, fault):
    root, study = project
    previous = (root/entry.POINTER).read_bytes()
    audit_path = study/'dual_B/capture.json'
    if fault == 'failed_stage':
        data = json.loads(audit_path.read_text()); data['status'] = 'FAIL'; save(audit_path, data)
    elif fault == 'apply_xtcfg':
        data = json.loads(audit_path.read_text())
        data['sides']['right']['first']['flags']['device_config_policy'] = 'apply_xtcfg'
        save(audit_path, data)
    elif fault == 'missing_stage':
        audit_path.unlink()
    elif fault == 'outside':
        study = root.parent
    else:
        study = Path('reports/lidar_stability/../lidar_stability')/study.name
    with pytest.raises(ValueError):
        entry.prepare_input(root, study, preparer=lambda *args: pytest.fail('preparer should not run'))
    assert (root/entry.POINTER).read_bytes() == previous


def test_changed_default_during_commit_is_not_overwritten(project, monkeypatch):
    root, study = project
    original = entry.write_new
    concurrent = b'{"newer_external_pointer": true}\n'
    def intervening_write(path, data):
        original(path, data)
        if path.name == 'result.json':
            (root/entry.POINTER).write_bytes(concurrent)
    monkeypatch.setattr(entry, 'write_new', intervening_write)
    with pytest.raises(RuntimeError, match='concurrently'):
        entry.prepare_input(root, study, preparer=prepared_fixture)
    assert (root/entry.POINTER).read_bytes() == concurrent


def test_preparation_lock_rejects_second_writer_and_is_released(project):
    root, study = project
    old = (root/entry.POINTER).read_bytes()
    with entry.pointer_lock(root):
        with pytest.raises(OSError):
            entry.prepare_input(root, study, preparer=lambda *args: pytest.fail('second preparer ran'))
    assert (root/entry.POINTER).read_bytes() == old
    assert (root/'.phase1_runtime/locks/point_picker_input.lock').is_file()
    assert entry.prepare_input(root, study, preparer=prepared_fixture)['pointer_updated'] is True


@pytest.mark.parametrize('fault', ['during_spawn', 'wait_failure'])
def test_pending_spawn_interrupt_or_wait_error_cleans_only_owned_group(project, monkeypatch, fault):
    root, _ = project
    old = (root/entry.POINTER).read_bytes()
    handlers, signalled = {}, []
    monkeypatch.setattr(entry.signal, 'SIGHUP', getattr(signal, 'SIGHUP', 99), raising=False)
    def install(sig, handler):
        previous = handlers.get(sig)
        handlers[sig] = handler
        return previous
    monkeypatch.setattr(entry.signal, 'signal', install)
    class Child:
        pid, returncode = 123456, None
        def poll(self):
            return self.returncode
        def wait(self, timeout):
            raise RuntimeError('synthetic wait failure')
    child = Child()
    def spawn(*args, **kwargs):
        assert kwargs['start_new_session'] is True
        if fault == 'during_spawn':
            handlers[entry.signal.SIGHUP](entry.signal.SIGHUP, None)
            assert signalled == []  # The child's identity is not available until Popen returns.
        return child
    def kill_group(pid, sig):
        signalled.append((pid, sig))
        child.returncode = 130
    monkeypatch.setattr(entry.subprocess, 'Popen', spawn)
    monkeypatch.setattr(entry.os, 'killpg', kill_group, raising=False)
    with pytest.raises(RuntimeError):
        entry.capture_study(root, 'box', root/'capture.log')
    assert signalled == [(child.pid, signal.SIGINT)]
    assert child.poll() == 130
    assert all(value is None for value in handlers.values())
    assert (root/entry.POINTER).read_bytes() == old


def test_help_does_not_import_target_or_touch_capture(monkeypatch, capsys):
    monkeypatch.setitem(sys.modules, 'wc_runtime.cli', None)
    monkeypatch.setattr(entry, 'capture_study', lambda *a: pytest.fail('capture attempted'))
    with pytest.raises(SystemExit) as caught:
        entry.main(['--help'])
    assert caught.value.code == 0
    assert '--prepare-only' in capsys.readouterr().out


def test_prepare_only_cli_never_invokes_capture(project, monkeypatch):
    root, study = project
    monkeypatch.chdir(root)
    monkeypatch.setitem(sys.modules, 'wc_runtime.cli', SimpleNamespace(ROOT=root, target=lambda: None))
    monkeypatch.setattr(entry, 'capture_study', lambda *a: pytest.fail('capture attempted'))
    actual = entry.prepare_input
    monkeypatch.setattr(entry, 'prepare_input', lambda r, s: actual(r, s, preparer=prepared_fixture))
    assert entry.main(['--prepare-only', study.relative_to(root).as_posix()]) == 0


def test_real_preparation_api_with_original_synthetic_npz_fixture(project, tmp_path, monkeypatch):
    """Runs the actual preparation and PickerSession, with no mocked point generation."""
    root, study = project
    fixture_path = PROJECT_SOURCE/'tests/calibration/test_prepare_stability_picker.py'
    spec = importlib.util.spec_from_file_location('_existing_preparation_test_fixture', fixture_path)
    fixture_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fixture_module)
    fixture_root = tmp_path/'actual_npz'; fixture_root.mkdir()
    actual_study = fixture_module.study.__wrapped__(fixture_root)
    for side in ('left', 'right'):
        source = actual_study/(side+'_single')
        destination = study/(side+'_single')
        for filename in (side+'.npz', side+'_frames.json', 'capture.json'):
            (destination/filename).write_bytes((source/filename).read_bytes())
        audit_path = destination/'capture.json'
        data = json.loads(audit_path.read_text())
        for endpoint in ('first', 'last'):
            data['sides'][side][endpoint]['flags']['device_config_policy'] = 'preserve_current'
        save(audit_path, data)
        frames_path = destination/(side+'_frames.json')
        frames = json.loads(frames_path.read_text())
        for branch in ('raw', 'filtered'):
            for record in frames[branch]:
                record['flags']['device_config_policy'] = 'preserve_current'
        save(frames_path, frames)
    save(root/'config/live_unvalidated.json', {'source_mode': 'real', 'sensor_ids': fixture_module.IDS})
    preparation = fixture_module.preparation
    def actual_prepare(r, s, output, log):
        assert preparation.main(['--project-root', str(r), '--study', str(s), '--output', str(output)]) == 0
    result = entry.prepare_input(root, study, preparer=actual_prepare)
    assert result['point_counts']['left']['original_rows'] == 9600
    assert result['point_counts']['left']['selectable_rows'] == 9596


@pytest.mark.skipif(os.name == 'nt', reason='POSIX process-group behavior is tested on the Orin/Linux target')
def test_owned_recorder_interrupt_reaps_process_and_preserves_old_pointer(project):
    root, _ = project
    old = (root/entry.POINTER).read_bytes()
    script = root/'scripts/record_lidar_comparison.sh'
    script.parent.mkdir(parents=True)
    # Fake recorder only: never imports project capture or device code.
    fake = root/'fake_recorder.py'
    fake.write_text('import os,signal,time,pathlib\n'
        'def stop(signum, frame):\n'
        '    pathlib.Path("cleaned.txt").write_text(str(signum))\n'
        '    raise SystemExit(130)\n'
        'signal.signal(signal.SIGINT, stop)\n'
        'pathlib.Path("fake.pid").write_text(str(os.getpid()))\n'
        'while True: time.sleep(.1)\n')
    script.write_text('exec "'+sys.executable+'" -s "'+str(fake)+'"\n')
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'], start_new_session=True)
    ready = []
    def interrupt_after_ready():
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            if (root/'fake.pid').exists():
                ready.append(True)
                os.kill(os.getpid(), signal.SIGINT)
                return
            time.sleep(.02)
    helper = threading.Thread(target=interrupt_after_ready, daemon=True)
    helper.start()
    try:
        with pytest.raises(RuntimeError, match='interrupted or failed'):
            entry.capture_study(root, 'box', root/'capture.log')
        helper.join(timeout=9)
        assert ready and not helper.is_alive()
        assert (root/'cleaned.txt').read_text() == str(int(signal.SIGINT))
        pid = int((root/'fake.pid').read_text())
        with pytest.raises(ProcessLookupError):
            os.kill(pid, 0)
        assert unrelated.poll() is None
        assert (root/entry.POINTER).read_bytes() == old
    finally:
        unrelated.terminate()
        unrelated.wait(timeout=5)
