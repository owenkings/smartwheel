"""Shared data location policy; executable code and device locks stay local.

Historical project-relative references are resolved, never rewritten in frozen
archives. A machine-local override selects the destination. Configured destinations never fall back.
"""
import hashlib
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
        self.backend = 'legacy'
        self.configuration_files = {}
        self.configuration_path = None
        self.configuration_value = None
        base = _ordinary(self.project_root/'config/storage.json')
        local = _ordinary(self.project_root/'config/storage.local.json')
        configuration = local if local.exists() else base
        # Read both original byte streams once. Only the selected file is parsed.
        for path in (base, local):
            if path.exists():
                if not path.is_file() or path.stat().st_size > 16384:
                    raise ValueError('STORAGE_CONFIG_INVALID: expected a small ordinary JSON file')
                self.configuration_files[path.name] = path.read_bytes()
        if not configuration.exists():
            if self.project_root == ORIN_PROJECT:
                raise ValueError('STORAGE_CONFIG_MISSING: explicit storage configuration is required')
            return  # Legacy checkouts and synthetic tests keep their old contract.
        self.configuration_path = configuration
        value = json.loads(self.configuration_files[configuration.name].decode('utf-8'))
        self.configuration_value = value
        if not isinstance(value, dict) or type(value.get('schema_version')) is not int:
            raise ValueError('STORAGE_CONFIG_INVALID: explicit schema_version required')
        if value['schema_version'] == 1:
            if type(value.get('enabled')) is not bool:
                raise ValueError('STORAGE_CONFIG_INVALID: schema 1 requires explicit enabled')
            if not value['enabled']:
                return
            self.backend = 'removable'
        elif value['schema_version'] == 2:
            if 'enabled' in value and value['enabled'] is not True:
                raise ValueError('STORAGE_CONFIG_INVALID: schema 2 requires enabled=true when supplied')
            self.backend = value.get('backend')
            if self.backend not in ('directory', 'removable'):
                raise ValueError('STORAGE_CONFIG_INVALID: directory or removable backend required')
        else:
            raise ValueError('STORAGE_CONFIG_INVALID: unsupported schema_version')
        if value.get('fallback_allowed') is not False:
            raise ValueError('STORAGE_CONFIG_INVALID: configured storage must disable fallback')
        def configured_path(field, default=None):
            raw = value.get(field, default)
            if not isinstance(raw, str) or not raw:
                raise ValueError('STORAGE_CONFIG_INVALID: nonempty '+field+' required')
            path = _ordinary(Path(raw).expanduser())
            if not path.is_absolute():
                raise ValueError('STORAGE_CONFIG_INVALID: absolute or home-relative roots required')
            return path
        self.archive_root = configured_path('archive_root')
        self.original_project_root = configured_path('original_project_root', str(self.project_root))
        if self.archive_root.is_relative_to(self.project_root) or self.project_root.is_relative_to(self.archive_root):
            raise ValueError('STORAGE_CONFIG_INVALID: archive and code roots must be separate')
        if self.backend == 'removable':
            mount = configured_path('mount_point')
            if not self.archive_root.is_relative_to(mount) or self.archive_root == mount:
                raise ValueError('STORAGE_CONFIG_INVALID: archive root must be a child of the mount')
            self.required_uuid = value.get('required_uuid')
            self.mount_point = mount
        elif 'required_uuid' in value or 'mount_point' in value:
            raise ValueError('STORAGE_CONFIG_INVALID: directory backend does not accept USB identity fields')
        self.enabled = True

    def destination_guard(self, output_root):
        if self.backend == 'removable':
            from .capture_destination import CaptureDestination
            guard = CaptureDestination(output_root, self.required_uuid)
            if guard.mount_root != self.mount_point:
                raise ValueError('STORAGE_CONFIG_INVALID: actual mount differs from configured mount')
        elif self.backend == 'directory':
            from .storage_directory import DirectoryDestination
            guard = DirectoryDestination(output_root)
        else:
            return None
        guard.storage_policy = self
        return guard

    def snapshot_configuration(self, directory):
        """Write the selected original bytes and frozen base/local provenance."""
        directory = Path(directory)
        hashes = {}
        if self.configuration_path is None:
            return hashes
        selected = self.configuration_files[self.configuration_path.name]
        files = {'storage.json': selected}
        for name, raw in self.configuration_files.items():
            files['storage_local.json' if name == 'storage.local.json' else 'storage_base.json'] = raw
        info = dict(schema_version=1, selected_file=self.configuration_path.name,
            selection_policy='LOCAL_OVERRIDE_ELSE_BASE', backend=self.backend,
            effective_sha256=hashlib.sha256(selected).hexdigest(),
            original_files={name: hashlib.sha256(raw).hexdigest()
                            for name, raw in self.configuration_files.items()})
        files['storage_selection.json'] = (json.dumps(info, ensure_ascii=False, indent=2)+'\n').encode('utf-8')
        for name, raw in files.items():
            target = directory/name
            with target.open('xb') as stream:
                stream.write(raw)
            hashes['configuration/'+name] = hashlib.sha256(raw).hexdigest()
        return hashes

    def check(self):
        if self.enabled and self.guard is None:
            self.guard = self.destination_guard(self.archive_root)
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
