"""Per-operation storage for explicitly selected input/output directories.

This does not broaden the project's default storage roots. CLI callers opt in
for one operation; every selected root retains its own mount identity guard.
"""
from pathlib import Path

from .storage_policy import StoragePolicy


class UserDataPaths:
    def __init__(self, project_root, dataset, output, auxiliaries=()):
        from wc_panel.storage import ordinary, resolve_user_destination
        self.project_root = Path(project_root)
        self.base = StoragePolicy(project_root)
        self.roots = []
        self.guards = []
        self.enabled = True

        def select(value, *, exists, file_allowed=False):
            candidate = Path(value).expanduser()
            path = ordinary(candidate if candidate.is_absolute() else self.project_root/candidate)
            # Keep historical project-relative aliases working.
            if path.is_relative_to(self.project_root):
                path = self.base.resolve(path)
            if file_allowed and path.is_relative_to(self.project_root/'config'):
                if not path.is_file():
                    raise ValueError('Configuration input does not exist: '+str(path))
                self.roots.append(path)
                self.guards.append(self.base)
                return path
            anchor = path.parent if file_allowed and path.is_file() else path
            _, guard = resolve_user_destination(project_root, anchor, must_exist=exists)
            self.roots.append(path)
            self.guards.append(guard)
            return path

        self.source = select(dataset, exists=True)
        self.output = select(output, exists=False)
        if self.source == self.output or self.source in self.output.parents or self.output in self.source.parents:
            raise ValueError('Output overlaps original dataset')
        self.archive_root = self.output.parent
        for value in auxiliaries:
            if value is not None:
                select(value, exists=True, file_allowed=True)
        self.check()

    def resolve(self, value):
        from wc_panel.storage import ordinary
        path = ordinary(Path(value) if Path(value).is_absolute() else self.project_root/Path(value))
        for root, guard in zip(self.roots, self.guards):
            if path == root or path.is_relative_to(root):
                guard.check()
                return path
        return self.base.resolve(path)

    def check(self):
        return {'status':'AVAILABLE','scope':'EXPLICIT_OPERATION_DATA_PATHS',
                'roots':[str(p) for p in self.roots],
                'destinations':[guard.check() for guard in self.guards],
                'fallback_allowed':False}
