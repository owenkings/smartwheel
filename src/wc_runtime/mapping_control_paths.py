"""Short, UID-owned Unix socket paths independent of archive or clone length."""
import hashlib
from pathlib import Path
import re

from .project_paths import socket_root
from .storage_policy import StoragePolicy


def manual_socket_path(project_root, directory, session_id):
    if not isinstance(session_id, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}', session_id):
        raise ValueError('Invalid manual control session identity')
    directory = Path(directory).absolute()
    if '..' in directory.parts:
        raise ValueError('Canonical manual control session directory required')
    for parent in (directory, *directory.parents):
        if parent.is_symlink():
            raise ValueError('Linked manual control session directory refused')
    storage = StoragePolicy(project_root)
    if storage.enabled and directory.is_relative_to(storage.archive_root):
        storage.resolve(directory)  # Recheck the selected archive before opening control.
    # Include the full session directory so a repeated session ID in another
    # archive/staging directory never attaches to an existing control server.
    identity = str(directory) + '\0' + session_id
    key = hashlib.sha256(identity.encode('utf-8')).hexdigest()[:24]
    return socket_root(project_root)/key/'manual.sock'
