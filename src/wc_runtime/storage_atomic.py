"""Cooperative publication for the configured removable archive.

exFAT/FUSE may support rename but reject renameat2(RENAME_NOREPLACE). All
application writers then share a local POSIX lock and verify absence under that
lock. The archive namespace must not be edited by other applications during
publication: external writers that ignore this lock cannot be made atomic by
this filesystem. There is deliberately no fallback for arbitrary destinations.
"""
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
from .project_paths import project_root, require_linux_runtime
import stat
import time

from .storage_policy import StoragePolicy


PROJECT_ROOT = project_root()
PROTOCOL = 'UUID_VERIFIED_COOPERATIVE_LOCKED_RENAME'


@contextmanager
def publication_lock(storage):
    import fcntl
    key = hashlib.sha256(str(storage.archive_root).encode('utf-8')).hexdigest()[:24]
    from .project_paths import shared_lock_root
    path = shared_lock_root()/('archive-publish-'+key+'.lock')
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(path, os.O_CREAT | os.O_RDWR | getattr(os, 'O_NOFOLLOW', 0), 0o600)
    with os.fdopen(descriptor, 'a+b') as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid():
            raise ValueError('Unsafe archive publication lock')
        fcntl.flock(stream, fcntl.LOCK_EX)
        try:
            yield stream
        finally:
            fcntl.flock(stream, fcntl.LOCK_UN)


def rename_on_configured_archive(source, destination, *, project_root=None):
    """Publish under the common app lock or fail; never replace an existing path."""
    storage = StoragePolicy(PROJECT_ROOT if project_root is None else project_root)
    source, destination = Path(source).absolute(), Path(destination).absolute()
    if (not storage.enabled or not source.is_relative_to(storage.archive_root)
            or not destination.is_relative_to(storage.archive_root)
            or source.parent != destination.parent or source == destination):
        raise ValueError('Cooperative publication requires sibling paths on configured archive')
    with publication_lock(storage) as receipt:
        storage.resolve(source)
        storage.resolve(destination)
        if not source.exists() or source.is_symlink():
            raise ValueError('Archive publication source is absent or linked')
        if os.path.lexists(destination):
            raise FileExistsError('Archive destination already exists: '+str(destination))
        # Exclusive ownership of each new session and the shared publication
        # lock protect all writers in this application. Same-user external
        # tools must not write into an in-progress session namespace.
        os.rename(source, destination)
        storage.check()
        event = {'protocol': PROTOCOL, 'destination': str(destination),
                 'host_unix_ns': time.time_ns(), 'external_writer_exclusion_required': True}
        receipt.seek(0); receipt.truncate()
        receipt.write((json.dumps(event, sort_keys=True)+'\n').encode('utf-8'))
        receipt.flush(); os.fsync(receipt.fileno())
        return event
