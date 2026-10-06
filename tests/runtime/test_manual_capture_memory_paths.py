"""RAM manual-owner path authorization only; no ROS, serial or hardware."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
from types import SimpleNamespace
import uuid

import pytest

from wc_motion.protocol import FeedbackError
from wc_runtime import mapping_wheel


@pytest.fixture
def memory_session(monkeypatch):
    parent = Path(__file__).resolve().parent
    fixture = parent/('.manual_memory_test_'+uuid.uuid4().hex)
    fixture.mkdir()
    memory_root = fixture/'wc_capture'
    memory_root.mkdir(mode=0o700)
    directory = memory_root/'synthetic_capture'
    directory.mkdir(mode=0o700)
    (directory/'configuration').mkdir()
    (directory/'sources').mkdir()
    monkeypatch.setattr(mapping_wheel, 'MEMORY_CAPTURE_ROOT', memory_root)
    actor = getattr(os, 'geteuid', lambda: 0)()
    monkeypatch.setattr(mapping_wheel.os, 'geteuid', lambda: actor, raising=False)
    if os.name == 'nt':
        # Windows lacks POSIX uid/mode semantics. Normalize only the synthetic
        # two checked directories; target tests exercise real POSIX metadata.
        actual_lstat = Path.lstat
        def posix_lstat(path, *args, **kwargs):
            info = actual_lstat(path, *args, **kwargs)
            if path in (memory_root, directory):
                values = list(info)
                values[0], values[4] = stat.S_IFMT(info.st_mode) | 0o700, actor
                return os.stat_result(values)
            return info
        monkeypatch.setattr(Path, 'lstat', posix_lstat)
    config = {'session_id': directory.name, 'source_mode': 'real', 'status': 'EXPERIMENT',
              'wheel_hardware_config': str(directory/'configuration/wheel_feedback.json')}
    hardware = {'schema': 'SYNTHETIC_PATH_AUTHORIZATION_ONLY', 'device_id': 'synthetic'}
    for name, value in (('manual_runtime.json', config), ('wheel_feedback.json', hardware)):
        (directory/'configuration'/name).write_text(json.dumps(value), encoding='utf-8')
    manifest = {'session_id': directory.name, 'status': 'RECORDING', 'manual_drive': True,
                'control_authorization': 'EXPLICIT_CAPTURE_MANUAL_DRIVE_ARGUMENT',
                'input_hashes': {str(path.relative_to(directory)).replace('\\', '/'):
                                hashlib.sha256(path.read_bytes()).hexdigest()
                                for path in (directory/'configuration').iterdir()}}
    (directory/'capture_manifest.json').write_text(json.dumps(manifest), encoding='utf-8')
    args = SimpleNamespace(session_root=directory, require_recorder=True,
                           config=directory/'configuration/manual_runtime.json',
                           raw_output=directory/'sources/wheel_feedback.jsonl',
                           summary_path=directory/'sources/wheel_summary.json')
    try:
        yield fixture, directory, args, manifest
    finally:
        assert fixture.resolve().parent == parent
        assert fixture.name.startswith('.manual_memory_test_')
        shutil.rmtree(fixture)


def write_manifest(directory, manifest):
    (directory/'capture_manifest.json').write_text(json.dumps(manifest), encoding='utf-8')


def update_config(directory, manifest, change):
    path = directory/'configuration/manual_runtime.json'
    config = json.loads(path.read_text(encoding='utf-8'))
    config.update(change)
    path.write_text(json.dumps(config), encoding='utf-8')
    manifest['input_hashes']['configuration/manual_runtime.json'] = hashlib.sha256(path.read_bytes()).hexdigest()
    write_manifest(directory, manifest)


def test_ram_scope_returns_same_byte_verified_snapshots_and_session_bound_paths(memory_session):
    fixture, directory, args, _ = memory_session
    actual, scope, snapshots = mapping_wheel.wheel_session_scope(args, fixture/'project')
    assert actual == scope == directory
    assert set(snapshots) == {directory/'configuration/manual_runtime.json',
                              directory/'configuration/wheel_feedback.json'}
    assert snapshots[args.config]['session_id'] == directory.name
    # A later change cannot silently change what the caller decoded/verified.
    (directory/'configuration/manual_runtime.json').write_text('{"session_id":"changed"}', encoding='utf-8')
    assert snapshots[args.config]['session_id'] == directory.name


@pytest.mark.parametrize('change', [dict(require_recorder=False), dict(config=None), dict(summary_path=None)])
def test_ram_scope_requires_explicit_capture_flags(memory_session, change):
    fixture, _, args, _ = memory_session
    for key, value in change.items():
        setattr(args, key, value)
    with pytest.raises(FeedbackError, match='outside-project'):
        mapping_wheel.wheel_session_scope(args, fixture/'project')


@pytest.mark.parametrize('change', [dict(status='COMPLETE'), dict(status='PARTIAL'), dict(manual_drive=False),
    dict(manual_drive=1), dict(session_id='other'), dict(control_authorization='IMPLICIT'), dict(control_authorization=None)])
def test_ram_manifest_must_declare_current_session_explicit_manual_authorization(memory_session, change):
    fixture, directory, args, manifest = memory_session
    manifest.update(change)
    write_manifest(directory, manifest)
    with pytest.raises(FeedbackError, match='manifest'):
        mapping_wheel.wheel_session_scope(args, fixture/'project')


@pytest.mark.parametrize('relative', ['configuration/manual_runtime.json', 'configuration/wheel_feedback.json'])
@pytest.mark.parametrize('change', ['absent_hash', 'wrong_hash', 'changed_bytes', 'missing_file'])
def test_both_configuration_files_require_matching_frozen_hash(memory_session, relative, change):
    fixture, directory, args, manifest = memory_session
    if change == 'absent_hash':
        manifest['input_hashes'].pop(relative)
    elif change == 'wrong_hash':
        manifest['input_hashes'][relative] = '0'*64
    elif change == 'changed_bytes':
        (directory/relative).write_bytes(b'{"changed":true}')
    else:
        (directory/relative).unlink()
    write_manifest(directory, manifest)
    with pytest.raises(FeedbackError, match='snapshot'):
        mapping_wheel.wheel_session_scope(args, fixture/'project')


@pytest.mark.parametrize('field', ['config', 'raw_output', 'summary_path'])
def test_ram_inputs_and_outputs_cannot_escape_the_same_session(memory_session, field):
    fixture, _, args, _ = memory_session
    setattr(args, field, fixture/'outside.json')
    with pytest.raises((FeedbackError, ValueError)):
        mapping_wheel.wheel_session_scope(args, fixture/'project')


@pytest.mark.parametrize('change', [dict(session_id='other'), dict(source_mode='replay'),
                                  dict(status='SYNTHETIC'), dict(wheel_hardware_config='/etc/hosts')])
def test_ram_verified_configuration_still_requires_live_identity_and_local_hardware(memory_session, change):
    fixture, directory, args, manifest = memory_session
    update_config(directory, manifest, change)
    with pytest.raises((FeedbackError, ValueError)):
        mapping_wheel.wheel_session_scope(args, fixture/'project')


@pytest.mark.parametrize('path_kind', ['root', 'session'])
@pytest.mark.parametrize('failure', ['different_owner', 'other_user_writable'])
def test_ram_root_and_session_directory_ownership_are_checked(memory_session, monkeypatch, path_kind, failure):
    fixture, directory, args, _ = memory_session
    checked = directory.parent if path_kind == 'root' else directory
    original_lstat = Path.lstat
    def lstat(path, *values, **kwargs):
        info = original_lstat(path, *values, **kwargs)
        if path == checked:
            fields = list(info)
            if failure == 'different_owner':
                fields[4] = os.geteuid()+1
            else:
                fields[0] = info.st_mode | 0o002
            return os.stat_result(fields)
        return info
    monkeypatch.setattr(Path, 'lstat', lstat)
    with pytest.raises(FeedbackError, match='directories'):
        mapping_wheel.wheel_session_scope(args, fixture/'project')


def test_nested_or_arbitrary_system_directory_is_not_admitted(memory_session):
    fixture, directory, args, _ = memory_session
    for path in (directory/'nested', directory.parent, Path('/tmp/arbitrary_session').absolute()):
        args.session_root = path
        with pytest.raises(FeedbackError, match='outside-project'):
            mapping_wheel.wheel_session_scope(args, fixture/'project')


def test_invalid_session_identifier_is_not_admitted(memory_session):
    fixture, directory, args, _ = memory_session
    args.session_root = directory.with_name('_invalid')
    with pytest.raises(FeedbackError, match='outside-project'):
        mapping_wheel.wheel_session_scope(args, fixture/'project')


def test_ram_configuration_symlink_is_rejected(memory_session):
    fixture, directory, args, _ = memory_session
    path = directory/'configuration/wheel_feedback.json'
    outside = fixture/'copied.json'
    outside.write_bytes(path.read_bytes())
    path.unlink()
    try:
        path.symlink_to(outside)
    except OSError:
        pytest.skip('This host cannot create symlinks; target tests check them')
    with pytest.raises(ValueError, match='symlinks'):
        mapping_wheel.wheel_session_scope(args, fixture/'project')


def test_normal_mapping_remains_project_bound_and_cannot_use_ram_without_flags(memory_session):
    fixture, directory, args, _ = memory_session
    project = fixture/'project'
    project.mkdir()
    normal = project/'session'
    normal.mkdir()
    args.session_root = normal
    args.require_recorder = False
    args.config = args.summary_path = None
    assert mapping_wheel.wheel_session_scope(args, project) == (normal, project, {})
    args.session_root = directory
    with pytest.raises(FeedbackError, match='outside-project'):
        mapping_wheel.wheel_session_scope(args, project)
