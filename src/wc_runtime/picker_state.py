"""Local picker selection and verified migration from the former state folder."""
from contextlib import contextmanager
import hashlib
import os
from pathlib import Path
import stat
import uuid

from .project_paths import runtime_root


POINTER = Path('.phase1_runtime/state/point_picker_input.json')
LEGACY_POINTER = Path('state/point_picker_input.json')
MAX_POINTER_BYTES = 16384


def ordinary(path):
    path = Path(path).absolute()
    if '..' in path.parts or any(p.is_symlink() or getattr(p, 'is_junction', lambda: False)()
                                for p in (path, *path.parents)):
        raise RuntimeError('Picker state must not traverse symlinks or junctions')
    return path


def pointer_path(root):
    return ordinary(runtime_root(ordinary(root))/'state/point_picker_input.json')


@contextmanager
def _file_lock(path):
    path = ordinary(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a+b') as stream:
        if os.name == 'nt':
            import msvcrt
            if path.stat().st_size == 0:
                stream.write(b'0'); stream.flush()
            stream.seek(0)
            msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
            try:
                yield
            finally:
                stream.seek(0)
                msvcrt.locking(stream.fileno(), msvcrt.LK_UNLCK, 1)
        else:
            import fcntl
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            try:
                yield
            finally:
                fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


@contextmanager
def pointer_lock(root):
    """One local OS lock covers migration, preparation and pointer commit."""
    with _file_lock(runtime_root(ordinary(root))/'locks/point_picker_input.lock'):
        yield


def _identity(info):
    return (info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns)


def _read(path):
    path = ordinary(path)
    before = path.lstat()
    if not stat.S_ISREG(before.st_mode) or before.st_size > MAX_POINTER_BYTES:
        raise ValueError('Picker pointer must be a bounded regular file')
    with path.open('rb') as stream:
        if _identity(os.fstat(stream.fileno())) != _identity(before):
            raise RuntimeError('Picker pointer changed before read')
        raw = stream.read(MAX_POINTER_BYTES + 1)
    if len(raw) > MAX_POINTER_BYTES or _identity(path.lstat()) != _identity(before):
        raise RuntimeError('Picker pointer changed during read')
    return raw, hashlib.sha256(raw).hexdigest(), _identity(before)


def _sync_directory(path):
    if os.name != 'nt':
        descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def _publish_exact(path, raw, digest):
    """Publish complete bytes without replacing an existing, different file."""
    path = ordinary(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        temporary = ordinary(path.parent/('.picker_migration_'+uuid.uuid4().hex+'.tmp'))
        try:
            with temporary.open('xb') as output:
                output.write(raw)
                output.flush()
                os.fsync(output.fileno())
            try:
                # Unlike replace(), this atomic publication cannot overwrite a
                # destination created by another writer after the check above.
                os.link(temporary, path)
            except FileExistsError:
                pass
            _sync_directory(path.parent)
        finally:
            if temporary.exists():
                temporary.unlink()
    actual, actual_digest, _ = _read(path)
    if actual_digest != digest or actual != raw:
        raise RuntimeError('Picker pointer migration conflict; existing files were not overwritten')


def migrate_pointer_locked(root):
    """Caller holds pointer_lock; preserve original bytes before retiring legacy state."""
    root = ordinary(root)
    target = pointer_path(root)
    legacy = ordinary(root/LEGACY_POINTER)
    if not legacy.exists():
        return target
    # Coordinate with an old preparer during the one-time transition. The
    # ordinary runtime lock remains the lock used by all new operations.
    with _file_lock(legacy.parent/'.point_picker_input.lock'):
        if not legacy.exists():
            return target
        raw, digest, identity = _read(legacy)
        _publish_exact(target, raw, digest)
        backup = target.parent/'pointer_migrations'/(digest+'.json')
        _publish_exact(backup, raw, digest)
        current, current_digest, current_identity = _read(legacy)
        if current_identity != identity or current_digest != digest or current != raw:
            raise RuntimeError('Legacy picker pointer changed; original retained, migration not completed')
        if _read(target)[:2] != (raw, digest):
            raise RuntimeError('New picker pointer changed; legacy pointer retained')
        legacy.unlink()
        _sync_directory(legacy.parent)
    return target


def migrate_pointer(root):
    """Idempotent access: existing new state is unchanged; conflicts fail closed."""
    target = pointer_path(root)
    legacy = ordinary(ordinary(root)/LEGACY_POINTER)
    if not legacy.exists():
        return target
    with pointer_lock(root):
        return migrate_pointer_locked(root)
