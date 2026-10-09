"""Explicit user data roots and bounded, reviewable deletion.

The configured removable mount never becomes an ordinary directory after loss.
All guard instances bind the mount identity for the lifetime of an operation.
"""
import hashlib
import json
import os
from pathlib import Path
import secrets
import shutil
import stat
import time


def ordinary(value):
    path = Path(value).expanduser()
    if not path.is_absolute() or '..' in path.parts:
        raise ValueError('请选择不含 .. 的绝对目录路径')
    for item in (path, *path.parents):
        if item.is_symlink() or getattr(item, 'is_junction', lambda: False)():
            raise ValueError('数据路径不能包含符号链接或目录联接: ' + str(item))
    return path


def resolve_user_destination(project_root, value, *, must_exist=True):
    """Return (absolute Path, runtime guard) for an explicitly selected root.

    Runtime callers must retain the returned guard and call check periodically.
    A directory may be outside StoragePolicy's historical aliases. UUID-bound
    guards are used for the configured USB and other separately mounted block
    devices; the internal disk uses DirectoryDestination.
    """
    from wc_runtime.storage_policy import StoragePolicy
    from wc_runtime.capture_destination import CaptureDestination, _mounts, UUID_ROOT
    from wc_runtime.storage_directory import DirectoryDestination
    project = ordinary(Path(project_root).absolute())
    path = ordinary(value)
    if path == Path(path.anchor) or path == project or project.is_relative_to(path):
        raise ValueError('不能使用文件系统根目录或包含软件工程的目录')
    for name in ('src', 'scripts', 'config', 'install', 'build', 'tests', 'docs', '.git', '.phase1_runtime', 'state'):
        if path.is_relative_to(project/name):
            raise ValueError('不能把源代码、配置或运行目录作为数据目录')
    policy = StoragePolicy(project)
    if policy.backend == 'removable' and path.is_relative_to(policy.mount_point):
        # This check is required even when the mount has disappeared.
        guard = CaptureDestination(path, policy.required_uuid)
        if guard.mount_root != policy.mount_point:
            raise ValueError('USB 挂载位置与已配置位置不一致')
    else:
        if must_exist and not path.is_dir():
            raise ValueError('所选目录不存在，请先创建目录: ' + str(path))
        ancestor = path
        while not ancestor.exists():
            ancestor = ancestor.parent
        rows = [row for row in _mounts() if ancestor.is_relative_to(Path(row['mount_point']))]
        if not rows:
            raise ValueError('无法验证数据目录的挂载信息')
        mount = max(rows, key=lambda row: len(Path(row['mount_point']).parts))
        source = Path(mount['source'])
        is_block = source.is_absolute() and source.exists() and stat.S_ISBLK(source.stat().st_mode)
        if mount['mount_point'] != '/' and is_block:
            matches = [p.name for p in UUID_ROOT.iterdir() if p.stat().st_rdev == source.stat().st_rdev]
            if len(matches) != 1:
                raise ValueError('所选挂载盘没有唯一可验证的 UUID')
            guard = CaptureDestination(path, matches[0])
        else:
            guard = DirectoryDestination(ancestor)
    if must_exist and not path.is_dir():
        raise ValueError('所选目录不存在: ' + str(path))
    guard.check()
    return path, guard


def atomic_json(path, value):
    path = Path(path)
    ordinary(path.parent)
    path.parent.mkdir(parents=True, exist_ok=True)
    raw = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode('utf-8')
    temporary = path.with_name('.' + path.name + '.' + secrets.token_hex(8))
    try:
        with temporary.open('xb') as stream:
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def tree_entries(path):
    """lstat inventory, no link traversal and no nested mount traversal."""
    path = ordinary(path)
    device = path.stat().st_dev
    mounts = _mount_points()
    rows = []
    def walk(item):
        info = item.lstat()
        if (info.st_dev != device or stat.S_ISLNK(info.st_mode) or item in mounts
                or getattr(item, 'is_junction', lambda: False)()):
            raise ValueError('目录包含链接或嵌套挂载，不能自动处理: ' + str(item))
        if not stat.S_ISDIR(info.st_mode) and not stat.S_ISREG(info.st_mode):
            raise ValueError('目录包含非常规文件: ' + str(item))
        rows.append(dict(path=str(item), size=info.st_size if stat.S_ISREG(info.st_mode) else 0,
                         allocated_bytes=info.st_blocks*512 if hasattr(info,'st_blocks') else None,
                         mtime_ns=info.st_mtime_ns, ino=info.st_ino, dev=info.st_dev,
                         directory=stat.S_ISDIR(info.st_mode)))
        if stat.S_ISDIR(info.st_mode):
            for child in sorted(item.iterdir()):
                walk(child)
    walk(path)
    return rows


def tree_size(path):
    return sum(row['size'] for row in tree_entries(path))


def allocated_total(rows):
    return sum(row['allocated_bytes'] for row in rows) if all(row.get('allocated_bytes') is not None for row in rows) else None


def fingerprint(rows):
    return hashlib.sha256(json.dumps(rows, sort_keys=True).encode()).hexdigest()


def _mount_points():
    """Include bind mounts, whose device number may equal their parent."""
    if os.name=='nt':return set()
    from wc_runtime.capture_destination import _mounts
    return {Path(row['mount_point']) for row in _mounts()}


def scan_entry(path, root_device, mounts):
    """Read an item without following links; preserve partial accounting errors.

    This permissive reporting walk is never used as a deletion inventory.
    Unknown totals stay null instead of treating unreadable subtrees as empty.
    """
    problems=[]
    known_bytes=0
    known_allocated=0
    allocation_available=True
    entry_type='other'
    def walk(item):
        nonlocal known_bytes,known_allocated,allocation_available,entry_type
        try:
            info=item.lstat()
            kind=('symlink' if stat.S_ISLNK(info.st_mode) or getattr(item,'is_junction',lambda:False)()
                  else 'mount' if info.st_dev!=root_device or item in mounts
                  else 'directory' if stat.S_ISDIR(info.st_mode)
                  else 'file' if stat.S_ISREG(info.st_mode) else 'other')
            if item==path:entry_type=kind
            if kind in ('symlink','mount','other'):
                raise ValueError({'symlink':'链接未跟随','mount':'嵌套挂载未遍历','other':'非常规文件未统计'}[kind])
            if hasattr(info,'st_blocks'):known_allocated+=info.st_blocks*512
            else:allocation_available=False
            if kind=='file':known_bytes+=info.st_size
            else:
                for child in sorted(item.iterdir()):walk(child)
        except (OSError,ValueError) as error:
            problems.append(dict(path=str(item),error=str(error)))
    walk(path)
    complete=not problems
    return dict(entry_type=entry_type,status='AVAILABLE' if complete else 'UNAVAILABLE',
                error='；'.join(problem['error'] for problem in problems[:3]),problems=problems,
                bytes=known_bytes if complete else None,
                allocated_bytes=known_allocated if complete and allocation_available else None,
                known_bytes=known_bytes,known_allocated_bytes=known_allocated if allocation_available else None,
                total_complete=complete,allocation_available=allocation_available)


class DataStore:
    def __init__(self, project_root, active_paths=lambda: (), resolver=resolve_user_destination):
        self.project_root = Path(project_root)
        self.resolver = resolver
        self.active_paths = active_paths
        self.tokens = {}

    def active(self, path):
        path = Path(path)
        if any(path == p or path.is_relative_to(p) or p.is_relative_to(path)
               for p in map(Path, self.active_paths())): return True
        # Include independently launched project jobs and children surviving a
        # panel crash. No signals are sent from storage operations.
        proc=Path('/proc')
        if not proc.is_dir():return False
        for process in proc.iterdir():
            if not process.name.isdigit() or int(process.name)==os.getpid():continue
            try:
                if process.stat().st_uid!=os.getuid():continue
                arguments=(process/'cmdline').read_bytes().decode('utf-8',errors='replace').split('\0')
                for argument in arguments:
                    value=argument.split('=',1)[-1]
                    if value.startswith(str(path)+os.sep) or value==str(path):return True
                for descriptor in (process/'fd').iterdir():
                    try:value=os.readlink(descriptor).removesuffix(' (deleted)')
                    except OSError:continue
                    if value.startswith(str(path)+os.sep) or value==str(path):return True
            except (OSError,ValueError):continue
        return False

    def scan(self, root):
        root, guard = self.resolver(self.project_root, root)
        items = []
        root_info=root.stat()
        root_device=root_info.st_dev
        mounts=_mount_points()
        for child in sorted(root.iterdir()):
            details=scan_entry(child,root_device,mounts)
            directory=details['entry_type']=='directory'
            try:
                active=self.active(child) if details['entry_type'] in ('directory','file') else False
            except (OSError,ValueError) as error:
                active=None
                message='无法确认任务占用: '+str(error)
                details['error']='；'.join(filter(None,[details['error'],message]))
                details['problems'].append(dict(path=str(child),error=message))
                details['status']='UNAVAILABLE'
            kind=details['entry_type']
            if directory:
                try:kind='recording' if (child/'capture_manifest.json').is_file() or (child/'bag').is_dir() else 'result'
                except OSError:kind='directory'
            deletable=directory and details['total_complete'] and not child.name.startswith('.') and active is False
            items.append(dict(name=child.name,path=str(child),kind=kind,active=active,deletable=deletable,**details))
        guard.check()
        usage = shutil.disk_usage(root)
        complete=all(item['total_complete'] for item in items)
        known_bytes=sum(item['known_bytes'] for item in items)
        root_allocated=root_info.st_blocks*512 if hasattr(root_info,'st_blocks') else None
        known_allocated=(root_allocated+sum(item['known_allocated_bytes'] for item in items)
                         if root_allocated is not None and all(item['known_allocated_bytes'] is not None for item in items) else None)
        return dict(root=str(root),bytes=known_bytes if complete else None,free_bytes=usage.free,
                    allocated_bytes=known_allocated if complete else None,
                    known_bytes=known_bytes,known_allocated_bytes=known_allocated,
                    root_allocated_bytes=root_allocated,
                    total_complete=complete,unknown_items=sum(not item['total_complete'] for item in items),
                    scope='ALL_DIRECT_ENTRIES_INCLUDING_HIDDEN; NO_LINK_OR_NESTED_MOUNT_TRAVERSAL',
                    disk_total_bytes=usage.total, items=items)

    def prepare_delete(self, root, paths):
        root, guard = self.resolver(self.project_root, root)
        selected = sorted(set(str(ordinary(p)) for p in paths))
        if not selected:
            raise ValueError('请明确选择需要删除的数据子文件夹')
        rows = []
        for value in selected:
            path = Path(value)
            if path.parent != root or path.name.startswith('.') or not path.is_dir():
                raise ValueError('只允许删除当前数据根目录下明确选择的直接子文件夹')
            if path.stat().st_dev!=root.stat().st_dev or path in _mount_points():
                raise ValueError('所选子文件夹是另一个挂载，不能删除')
            if self.active(path):
                raise ValueError('活动任务正在使用该数据，不能删除: ' + str(path))
            rows.extend(tree_entries(path))
        guard.check()
        token = secrets.token_urlsafe(32)
        plan = dict(token=token, root=str(root), paths=selected, bytes=sum(r['size'] for r in rows),
                    allocated_bytes=allocated_total(rows),
                    file_count=sum(not r['directory'] for r in rows), expires_at=time.time()+300)
        self.tokens[token] = (plan, rows, guard)
        return dict(plan)

    def execute_delete(self, token):
        item = self.tokens.pop(token, None)
        if item is None:
            raise ValueError('删除确认已失效，请重新查看删除清单')
        plan, rows, guard = item
        if time.time() > plan['expires_at']:
            raise ValueError('删除确认已过期，请重新查看清单')
        guard.check()
        current = []
        for value in plan['paths']:
            if self.active(value):
                raise ValueError('数据已被任务占用，取消删除')
            current.extend(tree_entries(Path(value)))
        if fingerprint(current) != fingerprint(rows):
            raise ValueError('数据自预览后发生变化，请重新查看清单')
        deleted = []
        free_before=shutil.disk_usage(plan['root']).free
        # Never recursively discover new files while deleting. New files cause
        # rmdir to fail, rather than extending the user's approved selection.
        root_fd=None
        identity_by_path={r['path']:(r['dev'],r['ino']) for r in rows}
        try:
            if os.name!='nt':root_fd=os.open(plan['root'],os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW)
            for row in sorted(rows, key=lambda r: len(Path(r['path']).parts), reverse=True):
                guard.check()
                path = ordinary(row['path'])
                parent_fd=None
                try:
                    if root_fd is not None:
                        parent_fd=os.dup(root_fd);parent=Path(plan['root'])
                        for part in path.parent.relative_to(parent).parts:
                            child_fd=os.open(part,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW,dir_fd=parent_fd)
                            os.close(parent_fd);parent_fd=child_fd;parent=parent/part
                            identity=os.fstat(parent_fd)
                            if (identity.st_dev,identity.st_ino)!=identity_by_path[str(parent)]:
                                raise ValueError('删除过程中父目录身份改变，已停止')
                        info=os.stat(path.name,dir_fd=parent_fd,follow_symlinks=False)
                    else:info=path.lstat()
                    if (info.st_dev, info.st_ino) != (row['dev'], row['ino']):
                        raise ValueError('删除过程中目录身份改变，已停止')
                    if row['directory']:
                        if parent_fd is None:path.rmdir()
                        else:os.rmdir(path.name,dir_fd=parent_fd)
                    else:
                        if not stat.S_ISREG(info.st_mode) or (info.st_size, info.st_mtime_ns) != (row['size'], row['mtime_ns']):
                            raise ValueError('删除过程中文件变化，已停止')
                        if parent_fd is None:path.unlink()
                        else:os.unlink(path.name,dir_fd=parent_fd)
                    deleted.append(str(path))
                finally:
                    if parent_fd is not None:os.close(parent_fd)
        finally:
            if root_fd is not None:os.close(root_fd)
        guard.check()
        free_after=shutil.disk_usage(plan['root']).free
        return dict(deleted=plan['paths'], bytes=plan['bytes'],allocated_bytes=plan['allocated_bytes'],
                    removed_entries=len(deleted),free_before_bytes=free_before,free_after_bytes=free_after,
                    free_delta_bytes=free_after-free_before,
                    measurement_note='可用空间实测变化，可能包含同期其他进程写入或删除的影响')
