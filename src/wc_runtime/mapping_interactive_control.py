"""Owner-bound local commands for one persistent mapping window.

Only internal runtime metadata is mutable. Session configuration and archives
are never control endpoints. No command can select an arbitrary session/path.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import uuid

from .mapping_app import checked_path, session_name

MAX_DOCUMENT_BYTES = 262144
IDENTITY_FIELDS = ('schema_version', 'window_id', 'owner_pid', 'owner_start_ticks')
COMMAND_FIELDS = {*IDENTITY_FIELDS, 'generation', 'active_session_id', 'command_token',
                  'sequence', 'nonce', 'action'}


def ordinary(path, *, directory=False):
    path = checked_path(path)
    info = path.lstat()
    if not (stat.S_ISDIR(info.st_mode) if directory else stat.S_ISREG(info.st_mode)):
        raise ValueError('Control path is not an ordinary '+('directory' if directory else 'file'))
    if os.name == 'posix' and (info.st_uid != os.getuid() or info.st_mode & 0o077):
        raise ValueError('Control metadata must be private to the current UID')
    return path


def read_document(path):
    path = ordinary(path)
    descriptor = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0))
    with os.fdopen(descriptor, 'rb') as stream:
        raw = stream.read(MAX_DOCUMENT_BYTES+1)
    if len(raw) > MAX_DOCUMENT_BYTES:
        raise ValueError('Control document exceeds its size limit')
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise ValueError('Control document must be an object')
    return value, raw


def atomic_document(path, value):
    path = Path(path)
    ordinary(path.parent, directory=True)
    if path.exists() or path.is_symlink():
        ordinary(path)
    temporary = path.with_name('.'+path.name+'.'+uuid.uuid4().hex)
    raw = (json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True)+'\n').encode('utf-8')
    if len(raw) > MAX_DOCUMENT_BYTES:
        raise ValueError('Control document exceeds its size limit')
    descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, 'O_NOFOLLOW', 0), 0o600)
    try:
        with os.fdopen(descriptor, 'wb') as stream:
            stream.write(raw)
            stream.flush()
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


class WindowControl:
    def __init__(self, root, window_id, *, owner_pid=None, owner_ticks=None):
        root = checked_path(root)
        session_name(window_id)
        base = checked_path(root/'.phase1_runtime/mapping_windows')
        base.mkdir(parents=True, exist_ok=True, mode=0o700)
        # A pre-existing shared runtime root is not silently chmod-ed.
        ordinary(base, directory=True)
        self.directory = base/window_id
        self.directory.mkdir(mode=0o700)
        pid = os.getpid() if owner_pid is None else owner_pid
        if owner_ticks is None:
            from .supervisor import ticks
            owner_ticks = ticks(pid)
        self.owner = dict(schema_version=1, project_root=str(root), window_id=window_id,
                          owner_pid=pid, owner_start_ticks=str(owner_ticks),
                          control_directory=str(self.directory))
        atomic_document(self.directory/'owner.json', self.owner)
        self.status = dict(self.owner, generation=0, view_generation=0, state='OPENING',
                           message='正在打开实时预览。', can_start=False, can_stop=False,
                           command_token=uuid.uuid4().hex, last_sequence=0, last_command=None,
                           active_session=None, save=dict(state='IDLE', session_id=None, result=None))
        self.last_document = None
        self.nonces = set()
        self.publish()

    def publish(self, *, transition=False, **values):
        if transition:
            self.status['generation'] += 1
        self.status.update(values)
        atomic_document(self.directory/'status.json', self.status)

    def bind(self, handle):
        self.status['view_generation'] += 1
        self.status['active_session'] = (None if handle is None else dict(
            session_id=handle['session_id'], directory=handle['directory'], runtime=handle['runtime'],
            view_path=str(Path(handle['directory'])/'view.rviz'), mapping_enabled=handle['mapping_enabled']))

    def receive(self):
        path = self.directory/'command.json'
        if not path.exists() and not path.is_symlink():
            return None
        command, raw = read_document(path)
        digest = hashlib.sha256(raw).hexdigest()
        if digest == self.last_document:
            return None
        self.last_document = digest
        reason = self.reject_reason(command)
        nonce, sequence = command.get('nonce'), command.get('sequence')
        self.status['last_command'] = dict(nonce=nonce, sequence=sequence,
                                           accepted=reason is None, reason=reason or 'ACCEPTED')
        if reason is None:
            self.status['last_sequence'] = sequence
            self.status['command_token'] = uuid.uuid4().hex
            self.nonces.add(nonce)
        self.publish()
        return command['action'] if reason is None else None

    def reject_reason(self, command):
        if set(command) != COMMAND_FIELDS:
            return 'INVALID_COMMAND_SCHEMA'
        if any(type(command.get(k)) is not type(self.owner[k]) or command[k] != self.owner[k]
               for k in IDENTITY_FIELDS):
            return 'OWNER_IDENTITY_MISMATCH'
        if type(command['generation']) is not int or command['generation'] != self.status['generation']:
            return 'STALE_GENERATION'
        active = self.status['active_session']
        if command['active_session_id'] != (active['session_id'] if active else None):
            return 'ACTIVE_SESSION_MISMATCH'
        if command['command_token'] != self.status['command_token']:
            return 'STALE_COMMAND_TOKEN'
        if (type(command['sequence']) is not int or
                not self.status['last_sequence'] < command['sequence'] < 2**53):
            return 'STALE_SEQUENCE'
        if not isinstance(command['nonce'], str) or re.fullmatch('[0-9a-f]{32}', command['nonce']) is None:
            return 'INVALID_NONCE'
        if command['nonce'] in self.nonces:
            return 'REUSED_NONCE'
        action = command['action']
        if action not in ('start', 'stop') or not self.status.get('can_'+action):
            return 'ACTION_NOT_AVAILABLE'
        return None
