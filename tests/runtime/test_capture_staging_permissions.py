"""Producer/owner RAM permissions agreement; real POSIX files, no ROS or serial."""
import hashlib
import json
import os
from pathlib import Path
import stat
from types import SimpleNamespace

import pytest

from wc_motion.protocol import FeedbackError
from wc_runtime import capture, mapping_wheel


pytestmark = pytest.mark.skipif(os.name != 'posix', reason='actual POSIX uid, mode and dir_fd contract')


@pytest.fixture
def staging_root(tmp_path, monkeypatch):
    root = tmp_path/'wc_capture'
    monkeypatch.setattr(capture, 'MEMORY_STAGING_ROOT', root)
    monkeypatch.setattr(mapping_wheel, 'MEMORY_CAPTURE_ROOT', root)
    return root


def mode(path):
    return stat.S_IMODE(path.lstat().st_mode)


def authorize(directory):
    config = directory/'configuration'
    config.mkdir()
    (directory/'sources').mkdir()
    documents = {
        'manual_runtime.json': dict(session_id=directory.name, source_mode='real', status='EXPERIMENT',
            wheel_hardware_config=str(config/'wheel_feedback.json')),
        'wheel_feedback.json': dict(schema='SYNTHETIC_SCOPE_ONLY', device_id='synthetic'),
    }
    hashes = {}
    for name, value in documents.items():
        path = config/name
        payload = json.dumps(value).encode('utf-8')
        path.write_bytes(payload)
        hashes['configuration/'+name] = hashlib.sha256(payload).hexdigest()
    (directory/'capture_manifest.json').write_text(json.dumps(dict(session_id=directory.name,
        status='RECORDING', manual_drive=True,
        control_authorization='EXPLICIT_CAPTURE_MANUAL_DRIVE_ARGUMENT', input_hashes=hashes)), encoding='utf-8')
    return SimpleNamespace(session_root=directory, require_recorder=True,
        config=config/'manual_runtime.json', raw_output=directory/'sources/wheel_feedback.jsonl',
        summary_path=directory/'sources/wheel_summary.json')


def assert_owner_accepts(directory):
    args = authorize(directory)
    selected, scope, snapshots = mapping_wheel.wheel_session_scope(args, directory.parent.parent/'project')
    assert selected == scope == directory
    assert set(snapshots) == {directory/'configuration/manual_runtime.json',
                              directory/'configuration/wheel_feedback.json'}


def test_new_root_is_secure_under_group_writable_umask_and_real_owner_accepts(staging_root):
    original = os.umask(0o002)
    try:
        directory = capture.prepare_staging_directory('synthetic_new')
    finally:
        os.umask(original)
    assert directory == staging_root/'synthetic_new'
    assert mode(staging_root) == mode(directory) == 0o700
    assert staging_root.lstat().st_uid == directory.lstat().st_uid == os.geteuid()
    assert_owner_accepts(directory)


@pytest.mark.parametrize('previous_mode', [0o700, 0o755, 0o775, 0o777])
def test_existing_owned_root_only_loses_external_write_bits_and_preserves_old_sessions(staging_root, previous_mode):
    staging_root.mkdir()
    staging_root.chmod(previous_mode)
    old = staging_root/'old_session'
    old.mkdir()
    old.chmod(0o770)
    raw = old/'original.raw'
    raw.write_bytes(b'original retained sensor data')
    raw.chmod(0o664)
    old_before = old.lstat()
    raw_before = raw.lstat()
    directory = capture.prepare_staging_directory('synthetic_new')
    assert mode(staging_root) == previous_mode & ~0o022
    assert mode(directory) == 0o700
    assert old.lstat() == old_before and raw.lstat() == raw_before
    assert raw.read_bytes() == b'original retained sensor data'
    assert_owner_accepts(directory)


def test_old_leaf_only_creation_reproduces_parent_775_owner_rejection(staging_root):
    original = os.umask(0o002)
    try:
        directory = staging_root/'synthetic_legacy'
        directory.mkdir(mode=0o700, parents=True)
    finally:
        os.umask(original)
    assert mode(staging_root) == 0o775 and mode(directory) == 0o700
    args = authorize(directory)
    with pytest.raises(FeedbackError, match='directories must belong to this user'):
        mapping_wheel.wheel_session_scope(args, staging_root.parent/'project')


def test_foreign_owner_is_rejected_before_fchmod_or_session_creation(staging_root, monkeypatch):
    staging_root.mkdir()
    staging_root.chmod(0o775)
    identity = staging_root.lstat()
    real_fstat = os.fstat
    real_fchmod = os.fchmod
    chmod_calls = []
    def foreign_fstat(descriptor):
        info = real_fstat(descriptor)
        if (info.st_dev, info.st_ino) == (identity.st_dev, identity.st_ino):
            fields = list(info)
            fields[4] = os.geteuid()+1
            return os.stat_result(fields)
        return info
    def record_chmod(descriptor, permissions):
        chmod_calls.append((descriptor, permissions))
        return real_fchmod(descriptor, permissions)
    monkeypatch.setattr(capture.os, 'fstat', foreign_fstat)
    monkeypatch.setattr(capture.os, 'fchmod', record_chmod)
    with pytest.raises(ValueError, match='owned by this user'):
        capture.prepare_staging_directory('synthetic_new')
    assert chmod_calls == [] and mode(staging_root) == 0o775
    assert not (staging_root/'synthetic_new').exists()


@pytest.mark.parametrize('location', ['root', 'ancestor', 'session'])
def test_symlink_paths_never_chmod_the_target_or_create_session(staging_root, monkeypatch, location):
    target = staging_root.parent/'unrelated'
    target.mkdir()
    target.chmod(0o775)
    raw = target/'keep.raw'
    raw.write_bytes(b'keep unchanged')
    before = target.lstat()
    if location == 'root':
        staging_root.symlink_to(target, target_is_directory=True)
    elif location == 'ancestor':
        ancestor = staging_root.parent/'linked_parent'
        ancestor.symlink_to(target, target_is_directory=True)
        monkeypatch.setattr(capture, 'MEMORY_STAGING_ROOT', ancestor/'wc_capture')
    else:
        staging_root.mkdir(mode=0o700)
        (staging_root/'synthetic_new').symlink_to(target, target_is_directory=True)
    with pytest.raises(ValueError, match='unlinked|symlink'):
        capture.prepare_staging_directory('synthetic_new')
    assert target.lstat() == before and raw.read_bytes() == b'keep unchanged'
    assert not (target/'synthetic_new').exists() and not (target/'wc_capture').exists()


def test_regular_file_cannot_be_used_as_staging_root(staging_root):
    staging_root.write_bytes(b'not a directory')
    before = staging_root.lstat()
    with pytest.raises((FileExistsError, NotADirectoryError)):
        capture.prepare_staging_directory('synthetic_new')
    assert staging_root.lstat() == before and staging_root.read_bytes() == b'not a directory'


def test_existing_session_is_never_reused_or_modified(staging_root):
    directory = capture.prepare_staging_directory('synthetic_existing')
    payload = directory/'captured.raw'
    payload.write_bytes(b'existing capture must remain')
    before = directory.lstat()
    payload_before = payload.lstat()
    with pytest.raises(FileExistsError):
        capture.prepare_staging_directory(directory.name)
    assert directory.lstat() == before and payload.lstat() == payload_before
    assert payload.read_bytes() == b'existing capture must remain'


def test_root_replacement_during_descriptor_validation_is_rejected(staging_root, monkeypatch):
    staging_root.mkdir(mode=0o700)
    original_root = staging_root.lstat()
    moved = staging_root.with_name('original_root')
    real_fstat = os.fstat
    replaced = []
    def replace_after_open(descriptor):
        info = real_fstat(descriptor)
        if (info.st_dev, info.st_ino) == (original_root.st_dev, original_root.st_ino) and not replaced:
            staging_root.rename(moved)
            staging_root.mkdir(mode=0o700)
            replaced.append(True)
        return info
    monkeypatch.setattr(capture.os, 'fstat', replace_after_open)
    with pytest.raises(ValueError, match='root changed'):
        capture.prepare_staging_directory('synthetic_new')
    assert replaced and not (staging_root/'synthetic_new').exists()
    assert not (moved/'synthetic_new').exists()
