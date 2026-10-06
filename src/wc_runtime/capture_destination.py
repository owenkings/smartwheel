"""Fail-closed identity checks for explicitly selected removable archives."""
import os
from pathlib import Path
import re
import stat


MOUNTINFO = Path('/proc/self/mountinfo')
UUID_ROOT = Path('/dev/disk/by-uuid')


def _unescape(value):
    return re.sub(r'\\([0-7]{3})', lambda match: chr(int(match[1], 8)), value)


def _mounts():
    rows = []
    for line in MOUNTINFO.read_text().splitlines():
        before, after = line.split(' - ', 1)
        fields, filesystem = before.split(), after.split()
        rows.append(dict(mount_id=fields[0], device=fields[2], root=_unescape(fields[3]),
            mount_point=_unescape(fields[4]), options=fields[5].split(','),
            filesystem=filesystem[0], source=_unescape(filesystem[1]),
            super_options=filesystem[2].split(',')))
    return rows


def _unlinked(path):
    for candidate in (path, *path.parents):
        try:
            mode = candidate.lstat().st_mode
        except FileNotFoundError:
            continue
        if stat.S_ISLNK(mode):
            raise ValueError('external capture path must not contain symlinks: ' + str(candidate))
        if not stat.S_ISDIR(mode):
            raise ValueError('external capture path requires directories: ' + str(candidate))


class CaptureDestination:
    """Bind an external output root to one mounted UUID for this session.

    The /dev/disk/by-uuid link is used only to identify its block device; archive
    paths themselves never use symlinks. Mount identity is rechecked on every
    call, including during acquisition, so detach/remount cannot change disks.
    """
    def __init__(self, output_root, required_uuid):
        self.output_root = Path(output_root)
        if not self.output_root.is_absolute() or '..' in self.output_root.parts:
            raise ValueError('external output-root must be absolute without parent traversal')
        if not isinstance(required_uuid, str) or not re.fullmatch(r'[A-Za-z0-9-]+', required_uuid):
            raise ValueError('required-output-uuid must be one filesystem UUID')
        self.required_uuid = required_uuid
        self._identity = None
        self.metadata = self.check()
        self.mount_root = Path(self.metadata['mount_point'])

    def check(self):
        try:
            _unlinked(self.output_root)
            rows = [row for row in _mounts()
                    if self.output_root.is_relative_to(Path(row['mount_point']))]
            if not rows:
                raise ValueError('output-root has no mounted filesystem')
            mount = max(rows, key=lambda row: len(Path(row['mount_point']).parts))
            if sum(row['mount_point'] == mount['mount_point'] for row in rows) != 1:
                raise ValueError('external archive mount is ambiguous: stacked mounts')
            mount_root = Path(mount['mount_point'])
            if mount_root == Path('/') or mount['root'] != '/':
                raise ValueError('external archive requires a separate whole filesystem mount')
            if 'rw' not in mount['options'] or 'rw' not in mount['super_options']:
                raise ValueError('external archive filesystem is not mounted read-write')
            source = Path(mount['source'])
            if not source.is_absolute():
                raise ValueError('external archive mount must identify its block device')
            actual, required = source.stat(), (UUID_ROOT/self.required_uuid).stat()
            if not stat.S_ISBLK(actual.st_mode) or not stat.S_ISBLK(required.st_mode):
                raise ValueError('external archive UUID and source must identify block devices')
            if actual.st_rdev != required.st_rdev:
                raise ValueError('external archive mounted UUID does not match required-output-uuid')
            mounted_device = mount_root.stat().st_dev
            root_device = Path('/').stat().st_dev
            if mounted_device == root_device or actual.st_rdev == root_device:
                raise ValueError('external archive must not use the root filesystem')
            if '{}:{}'.format(os.major(mounted_device), os.minor(mounted_device)) != mount['device']:
                raise ValueError('external archive mount metadata changed during verification')
            existing = self.output_root
            while not existing.exists():
                existing = existing.parent
            if existing.stat().st_dev != mounted_device:
                raise ValueError('external output-root crosses another filesystem')
            identity = (mount['mount_id'], mount['device'], str(mount_root), actual.st_rdev)
            if self._identity is not None and identity != self._identity:
                raise ValueError('external archive mount changed since session verification')
            self._identity = identity
            return dict(status='AVAILABLE', output_root=str(self.output_root),
                required_uuid=self.required_uuid, mount_point=str(mount_root),
                mount_id=mount['mount_id'], device=mount['device'], source=str(source),
                filesystem=mount['filesystem'], read_write=True, fallback_allowed=False)
        except (OSError, ValueError, IndexError) as error:
            raise ValueError('CAPTURE_DESTINATION_UNAVAILABLE: ' + str(error)) from error


def capture_destination(project_root, output_root=None, required_uuid=None):
    """Use configured storage by default; explicit external paths require UUID."""
    if (output_root is None) != (required_uuid is None):
        raise ValueError('--output-root and --required-output-uuid must be supplied together')
    if output_root is None:
        from .storage_policy import StoragePolicy
        policy = StoragePolicy(project_root)
        output = policy.resolve('data/experiments')
        return output, (CaptureDestination(output, policy.required_uuid) if policy.enabled else None)
    guard = CaptureDestination(output_root, required_uuid)
    return guard.output_root, guard
