"""Host-wide (per UID) resource exclusion, independent of checkout paths."""
import os
import re
import stat

from .project_paths import shared_lock_root


def acquire_resource_lock(name):
    """Return an exclusively locked stream; closing it releases this owner."""
    import fcntl
    if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]{0,199}', name):
        raise ValueError('invalid shared resource lock name')
    path = shared_lock_root()/name
    descriptor = os.open(path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
    stream = os.fdopen(descriptor, 'a+', encoding='utf-8')
    try:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid() or stat.S_IMODE(info.st_mode) & 0o022:
            raise RuntimeError('shared resource lock must be an owned ordinary file without other-user writes')
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        return stream
    except BaseException:
        stream.close()
        raise
