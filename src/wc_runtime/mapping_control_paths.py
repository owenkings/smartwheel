"""Keep Unix control sockets local when mapping data resides on exFAT."""
import hashlib
from pathlib import Path
import re

from .storage_policy import StoragePolicy


def manual_socket_path(project_root, directory, session_id):
    if not isinstance(session_id, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}', session_id):
        raise ValueError('Invalid manual control session identity')
    storage = StoragePolicy(project_root)
    directory = Path(directory).absolute()
    if storage.enabled and directory.is_relative_to(storage.archive_root):
        storage.resolve(directory)  # Recheck volume identity before opening control.
        key = hashlib.sha256(session_id.encode('ascii')).hexdigest()[:24]
        return storage.resolve(Path('.phase1_runtime/control')/key/'manual.sock')
    return directory/'manual.sock'  # Existing local maps and authenticated RAM capture.
