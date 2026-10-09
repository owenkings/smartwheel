"""Local index migration preserves bytes and refuses competing or conflicting state."""
import hashlib
from pathlib import Path

import pytest

from wc_runtime import picker_state as state


RAW = b'{"schema_version": 1, "prepared_input": "reports/example/prepared.json"}\n'


def legacy(root, raw=RAW):
    path = root/state.LEGACY_POINTER
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(raw)
    return path


def test_absent_selection_does_not_create_runtime_directories(tmp_path):
    assert state.migrate_pointer(tmp_path) == tmp_path/state.POINTER
    assert list(tmp_path.iterdir()) == []


def test_migration_is_byte_exact_idempotent_and_retains_original(tmp_path):
    original = legacy(tmp_path)
    target = state.migrate_pointer(tmp_path)
    digest = hashlib.sha256(RAW).hexdigest()
    backup = target.parent/'pointer_migrations'/(digest+'.json')
    assert target == tmp_path/'.phase1_runtime/state/point_picker_input.json'
    assert target.read_bytes() == backup.read_bytes() == RAW
    assert not original.exists()
    assert (tmp_path/'.phase1_runtime/locks/point_picker_input.lock').is_file()
    assert state.migrate_pointer(tmp_path) == target
    # A subsequent legitimate selection must not conflict with the retired old index.
    target.write_bytes(b'{"updated_selection": true}\n')
    assert state.migrate_pointer(tmp_path).read_bytes() == b'{"updated_selection": true}\n'
    assert backup.read_bytes() == RAW


def test_identical_existing_target_completes_interrupted_migration(tmp_path):
    original = legacy(tmp_path)
    target = tmp_path/state.POINTER
    target.parent.mkdir(parents=True)
    target.write_bytes(RAW)
    assert state.migrate_pointer(tmp_path) == target
    assert target.read_bytes() == RAW and not original.exists()


def test_conflicting_target_never_overwrites_or_retires_original(tmp_path):
    original = legacy(tmp_path)
    target = tmp_path/state.POINTER
    target.parent.mkdir(parents=True)
    target.write_bytes(b'{"another_selection": true}')
    with pytest.raises(RuntimeError, match='conflict'):
        state.migrate_pointer(tmp_path)
    assert original.read_bytes() == RAW
    assert target.read_bytes() == b'{"another_selection": true}'
    assert not (target.parent/'pointer_migrations').exists()


@pytest.mark.parametrize('location', ['legacy', 'target', 'runtime'])
def test_linked_state_is_rejected_before_migration(tmp_path, monkeypatch, location):
    original = legacy(tmp_path)
    paths = {'legacy': original, 'target': tmp_path/state.POINTER,
             'runtime': tmp_path/'.phase1_runtime'}
    real = Path.is_symlink
    monkeypatch.setattr(Path, 'is_symlink', lambda path: path == paths[location] or real(path))
    with pytest.raises(RuntimeError, match='symlinks'):
        state.migrate_pointer(tmp_path)
    assert original.read_bytes() == RAW
    assert not (tmp_path/state.POINTER).exists()


def test_changed_source_is_retained_alongside_verified_backup(tmp_path, monkeypatch):
    original = legacy(tmp_path)
    publish = state._publish_exact
    newer = b'{"newer_legacy_selection": true}\n'
    def update_after_backup(path, raw, digest):
        publish(path, raw, digest)
        if path.parent.name == 'pointer_migrations':
            original.write_bytes(newer)
    monkeypatch.setattr(state, '_publish_exact', update_after_backup)
    with pytest.raises(RuntimeError, match='changed'):
        state.migrate_pointer(tmp_path)
    assert original.read_bytes() == newer
    target = tmp_path/state.POINTER
    assert target.read_bytes() == RAW
    assert (target.parent/'pointer_migrations'/(hashlib.sha256(RAW).hexdigest()+'.json')).read_bytes() == RAW


@pytest.mark.parametrize('lock_kind', ['runtime', 'legacy'])
def test_active_writer_prevents_migration_and_keeps_selection(tmp_path, lock_kind):
    original = legacy(tmp_path)
    lock = (state.pointer_lock(tmp_path) if lock_kind == 'runtime'
            else state._file_lock(original.parent/'.point_picker_input.lock'))
    with lock:
        with pytest.raises(OSError):
            state.migrate_pointer(tmp_path)
    assert original.read_bytes() == RAW
    assert not (tmp_path/state.POINTER).exists()
    assert state.migrate_pointer(tmp_path).read_bytes() == RAW


@pytest.mark.parametrize('kind', ['directory', 'oversized'])
def test_nonregular_or_oversized_legacy_selection_is_preserved(tmp_path, kind):
    original = tmp_path/state.LEGACY_POINTER
    original.parent.mkdir()
    if kind == 'directory':
        original.mkdir()
    else:
        original.write_bytes(b'x'*(state.MAX_POINTER_BYTES+1))
    with pytest.raises(ValueError, match='bounded regular'):
        state.migrate_pointer(tmp_path)
    assert original.exists()
    assert not (tmp_path/state.POINTER).exists()
