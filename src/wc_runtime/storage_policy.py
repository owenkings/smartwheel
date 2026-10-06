"""Shared data location policy; executable code and device locks stay local.

Historical project-relative references are resolved, never rewritten in frozen
archives. A configured removable volume is mandatory; there is no local fallback.
"""
import json
from pathlib import Path


DATA_NAMES = ('data', 'reports', 'maps')
LOCAL_NAMES = ('src', 'scripts', 'config', 'install', 'build', '.phase1_runtime',
               'tests', 'docs', 'state', 'vendor_patches')
ORIN_PROJECT = Path('/home/nvidia/wheelchair')


def _ordinary(path):
    if '..' in path.parts:
        raise ValueError('STORAGE_PATH_INVALID: parent traversal is not allowed')
    for parent in (path, *path.parents):
        if parent.is_symlink() or getattr(parent, 'is_junction', lambda: False)():
            raise ValueError('STORAGE_PATH_INVALID: symlinks or junctions are not allowed: '+str(parent))
    return path


class StoragePolicy:
    def __init__(self, project_root):
        self.project_root = _ordinary(Path(project_root).absolute())
        self.enabled = False
        self.archive_root = None
        self.guard = None
        self.mount_point = None
        self.required_uuid = None
        self.original_project_root = self.project_root
        configuration = _ordinary(self.project_root/'config/storage.json')
        if not configuration.exists():
            if self.project_root == ORIN_PROJECT:
                raise ValueError('STORAGE_CONFIG_MISSING: configured Orin must not fall back to its internal disk')
            return  # Legacy checkouts and synthetic tests keep their old contract.
        if not configuration.is_file() or configuration.stat().st_size > 16384:
            raise ValueError('STORAGE_CONFIG_INVALID: expected a small ordinary JSON file')
        value = json.loads(configuration.read_text(encoding='utf-8'))
        if (not isinstance(value, dict) or type(value.get('schema_version')) is not int
                or value['schema_version'] != 1 or type(value.get('enabled')) is not bool):
            raise ValueError('STORAGE_CONFIG_INVALID: schema_version=1 and explicit enabled required')
        if not value['enabled']:
            return
        if value.get('fallback_allowed') is not False:
            raise ValueError('STORAGE_CONFIG_INVALID: removable storage must disable fallback')
        self.archive_root = _ordinary(Path(value.get('archive_root', '')))
        self.original_project_root = _ordinary(Path(value.get('original_project_root', str(self.project_root))))
        mount = _ordinary(Path(value.get('mount_point', '')))
        if not all(path.is_absolute() for path in (self.archive_root, self.original_project_root, mount)):
            raise ValueError('STORAGE_CONFIG_INVALID: absolute roots are required')
        if not self.archive_root.is_relative_to(mount) or self.archive_root == mount:
            raise ValueError('STORAGE_CONFIG_INVALID: archive root must be a child of the mount')
        if self.archive_root.is_relative_to(self.project_root) or self.project_root.is_relative_to(self.archive_root):
            raise ValueError('STORAGE_CONFIG_INVALID: archive and code roots must be separate')
        self.required_uuid = value.get('required_uuid')
        self.mount_point = mount
        self.enabled = True

    def check(self):
        if self.enabled and self.guard is None:
            from .capture_destination import CaptureDestination
            guard = CaptureDestination(self.archive_root, self.required_uuid)
            if guard.mount_root != self.mount_point:
                raise ValueError('STORAGE_CONFIG_INVALID: actual mount differs from configured mount')
            self.guard = guard
        return self.guard.check() if self.guard is not None else {'status': 'LEGACY_PROJECT_STORAGE'}

    def _external(self, path):
        self.check()
        path = _ordinary(path)
        existing = path
        while not existing.exists():
            existing = existing.parent
        if existing.stat().st_dev != self.guard.mount_root.stat().st_dev:
            raise ValueError('STORAGE_PATH_INVALID: data path crosses another mounted filesystem')
        return path

    def resolve(self, value):
        value = Path(value)
        if '..' in value.parts:
            raise ValueError('STORAGE_PATH_INVALID: parent traversal is not allowed')
        path = value if value.is_absolute() else self.project_root/value
        if self.enabled and path.is_relative_to(self.archive_root):
            return self._external(path)
        relative = None
        for root in (self.project_root, self.original_project_root):
            if path.is_relative_to(root):
                relative = path.relative_to(root)
                break
        if relative is None:
            raise ValueError('STORAGE_PATH_OUTSIDE_ALLOWED_ROOTS: '+str(path))
        if self.enabled and relative.parts and relative.parts[0] in DATA_NAMES:
            return self._external(self.archive_root/relative)
        # Keep config/source/install/locks attached to the actual executable tree.
        return _ordinary(self.project_root/relative)

    def data_roots(self):
        self.check()
        base = self.archive_root if self.enabled else self.project_root
        return tuple(base/name for name in DATA_NAMES)

    def logical(self, value):
        """Return a portable project-relative identity for a resolved path."""
        path = self.resolve(value)
        base = self.archive_root if self.enabled and path.is_relative_to(self.archive_root) else self.project_root
        return path.relative_to(base).as_posix()


def resolve_storage_path(project_root, value):
    # Stop/cleanup must work even if the storage configuration itself was lost
    # or damaged. This bypass is limited to known code/OS-runtime namespaces.
    root = _ordinary(Path(project_root).absolute())
    path = Path(value)
    if '..' in path.parts:
        raise ValueError('STORAGE_PATH_INVALID: parent traversal is not allowed')
    path = path if path.is_absolute() else root/path
    for candidate in (root, ORIN_PROJECT):
        if path.is_relative_to(candidate):
            relative = path.relative_to(candidate)
            if not relative.parts or relative.parts[0] in LOCAL_NAMES:
                return _ordinary(root/relative)
            break
    return StoragePolicy(project_root).resolve(value)


def allowed_data_roots(project_root):
    return StoragePolicy(project_root).data_roots()


def logical_path(project_root, value):
    return StoragePolicy(project_root).logical(value)


logical_storage_path = logical_path


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description='Print a verified data path without creating it.')
    parser.add_argument('--project-root', type=Path, default=ORIN_PROJECT)
    parser.add_argument('path')
    args = parser.parse_args(argv)
    print(resolve_storage_path(args.project_root, args.path))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
