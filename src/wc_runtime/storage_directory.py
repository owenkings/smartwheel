"""Session-bound ordinary-directory storage, independent of USB identifiers."""
import os
from pathlib import Path
from .capture_destination import _mounts, _unlinked


class DirectoryDestination:
    """Check a pre-existing directory without creating a fallback destination."""
    def __init__(self, output_root):
        self.output_root = Path(output_root)
        if not self.output_root.is_absolute() or '..' in self.output_root.parts:
            raise ValueError('directory output-root must be absolute without parent traversal')
        self.required_uuid = None
        self._identity = None
        self.metadata = self.check()
        self.mount_root = Path(self.metadata['mount_point'])

    def check(self):
        try:
            _unlinked(self.output_root)
            if not self.output_root.is_dir():
                raise ValueError('configured directory must exist; run storage setup first')
            rows = [row for row in _mounts()
                    if self.output_root.is_relative_to(Path(row['mount_point']))]
            if not rows:
                raise ValueError('configured directory has no mounted filesystem')
            mount = max(rows, key=lambda row: len(Path(row['mount_point']).parts))
            if sum(row['mount_point'] == mount['mount_point'] for row in rows) != 1:
                raise ValueError('configured directory mount is ambiguous')
            if 'rw' not in mount['options'] or 'rw' not in mount['super_options']:
                raise ValueError('configured directory filesystem is not writable')
            if not os.access(self.output_root, os.W_OK | os.X_OK):
                raise ValueError('configured directory is not writable by current user')
            info = self.output_root.stat()
            device = '{}:{}'.format(os.major(info.st_dev), os.minor(info.st_dev))
            if device != mount['device']:
                raise ValueError('configured directory mount identity mismatch')
            ancestors = tuple((str(path), path.stat().st_dev, path.stat().st_ino)
                              for path in (self.output_root, *self.output_root.parents))
            identity = (ancestors, mount['mount_id'], mount['device'], mount['mount_point'])
            if self._identity is not None and identity != self._identity:
                raise ValueError('configured directory or mount changed during the session')
            self._identity = identity
            return dict(status='AVAILABLE', backend='directory', output_root=str(self.output_root),
                required_uuid=None, mount_point=mount['mount_point'], mount_id=mount['mount_id'],
                device=mount['device'], source=mount['source'], filesystem=mount['filesystem'],
                read_write=True, fallback_allowed=False)
        except (OSError, ValueError, IndexError) as error:
            raise ValueError('CAPTURE_DESTINATION_UNAVAILABLE: ' + str(error)) from error
