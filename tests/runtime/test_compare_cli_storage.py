"""CLI RAM output guards exercise real path/stat checks without ROS or devices."""
import os
from pathlib import Path
import shutil
from types import SimpleNamespace
import uuid

import pytest

from wc_runtime import mapping_compare


@pytest.fixture
def owned_paths(monkeypatch):
    parent = Path(__file__).absolute().parent
    directory = parent/('.compare_cli_storage_'+uuid.uuid4().hex)
    directory.mkdir()
    shm = directory/'shm'
    shm.mkdir(mode=0o700)
    base = shm/'wc_compare'
    mounts = directory/'mounts'
    mounts.write_text('tmpfs /dev/shm tmpfs rw,nosuid,nodev 0 0\n', encoding='utf-8')
    real_path = Path
    redirects = {str(Path('/dev/shm/wc_compare')): base,
                 str(Path('/dev/shm')): shm, str(Path('/proc/mounts')): mounts}
    def paths(value, *parts):
        result = redirects.get(str(real_path(value)), real_path(value))
        return result.joinpath(*parts)
    # Only constant system paths are redirected; output/parents/symlinks/stat
    # are actual filesystem operations. No third source tree or hardware.
    monkeypatch.setattr(mapping_compare, 'Path', paths)
    try:
        yield SimpleNamespace(root=directory, shm=shm, base=base, mounts=mounts)
    finally:
        assert directory.parent == parent and directory.name.startswith('.compare_cli_storage_')
        shutil.rmtree(directory)


def test_default_storage_scope_returns_original_root_without_filesystem_changes(monkeypatch):
    root, output = Path('/real/project'), Path('/some/output')
    def unexpected_path(*args, **kwargs):
        raise AssertionError('default mode must not probe or create RAM storage')
    monkeypatch.setattr(mapping_compare, 'Path', unexpected_path)
    assert mapping_compare.comparison_storage_root(root, output) is root
    assert mapping_compare.comparison_storage_root(root, output, False) is root


@pytest.mark.skipif(os.name != 'posix', reason='POSIX uid/mode and tmpfs device semantics')
def test_memory_storage_accepts_new_immediate_child_on_owned_same_filesystem(owned_paths):
    p = owned_paths
    output = p.base/'unique_session'
    assert mapping_compare.comparison_storage_root(p.root, output, True) == p.base
    assert p.base.is_dir() and not output.exists()
    assert p.base.stat().st_mode & 0o077 == 0
    assert p.base.stat().st_uid == os.getuid()
    assert p.base.stat().st_dev == p.shm.stat().st_dev


@pytest.mark.parametrize('location', ['base', 'outside', 'nested', 'parent_component'])
def test_memory_storage_rejects_scope_escape_and_nested_sessions(owned_paths, location):
    p = owned_paths
    output = {'base': p.base, 'outside': p.root/'session',
              'nested': p.base/'session'/'child',
              'parent_component': p.base/'session'/'..'/'escape'}[location]
    with pytest.raises(ValueError, match='new child'):
        mapping_compare.comparison_storage_root(p.root, output, True)
    assert not p.base.exists()


@pytest.mark.parametrize('table', [
    'rootfs / ext4 rw 0 0\n', 'none /dev/shm ext4 rw 0 0\n',
    'tmpfs /other tmpfs rw 0 0\n', 'tmpfs /dev/shm-other tmpfs rw 0 0\n',
    'malformed\n', '',
])
def test_memory_storage_requires_exact_tmpfs_mount_entry(owned_paths, table):
    p = owned_paths
    p.mounts.write_text(table, encoding='utf-8')
    with pytest.raises(ValueError, match='mounted tmpfs'):
        mapping_compare.comparison_storage_root(p.root, p.base/'session', True)
    assert not p.base.exists()


@pytest.mark.skipif(os.name != 'posix', reason='POSIX symlink creation without privilege')
@pytest.mark.parametrize('linked', ['base', 'output'])
def test_memory_storage_rejects_actual_symlink_in_scope(owned_paths, linked):
    p = owned_paths
    target = p.root/'other_directory'
    target.mkdir()
    output = p.base/'session'
    if linked == 'base':
        p.base.symlink_to(target, target_is_directory=True)
    else:
        p.base.mkdir(mode=0o700)
        output.symlink_to(target, target_is_directory=True)
    with pytest.raises(ValueError, match='not follow symlinks'):
        mapping_compare.comparison_storage_root(p.root, output, True)
    assert not (target/'session').exists()


@pytest.mark.skipif(os.name != 'posix', reason='POSIX uid/mode semantics')
def test_memory_storage_rejects_wrong_owner(owned_paths, monkeypatch):
    p = owned_paths
    p.base.mkdir(mode=0o700)
    monkeypatch.setattr(mapping_compare.os, 'getuid', lambda: p.base.stat().st_uid+1)
    with pytest.raises(ValueError, match='must be owned'):
        mapping_compare.comparison_storage_root(p.root, p.base/'session', True)


@pytest.mark.skipif(os.name != 'posix', reason='POSIX mode semantics')
@pytest.mark.parametrize('mode', [0o720, 0o702, 0o777])
def test_memory_storage_rejects_group_or_world_writable_base(owned_paths, mode):
    p = owned_paths
    p.base.mkdir(mode=0o700)
    p.base.chmod(mode)
    with pytest.raises(ValueError, match='must be owned'):
        mapping_compare.comparison_storage_root(p.root, p.base/'session', True)


@pytest.mark.skipif(os.name != 'posix', reason='POSIX filesystem device guard')
def test_memory_storage_rejects_different_device_despite_tmpfs_parent_entry(owned_paths, monkeypatch):
    p = owned_paths
    p.base.mkdir(mode=0o700)
    path_class = type(p.base)
    actual_stat = path_class.stat
    def changed_device(path, *args, **kwargs):
        result = actual_stat(path, *args, **kwargs)
        if path == p.base and kwargs.get('follow_symlinks', True):
            return SimpleNamespace(st_dev=result.st_dev+1, st_uid=result.st_uid, st_mode=result.st_mode)
        return result
    monkeypatch.setattr(path_class, 'stat', changed_device)
    with pytest.raises(ValueError, match='must remain on'):
        mapping_compare.comparison_storage_root(p.root, p.base/'session', True)


@pytest.mark.skipif(os.name != 'posix', reason='Target Linux CLI uses strict realpath; Windows sandbox denies resolving new fixtures')
@pytest.mark.parametrize('case', ['existing', 'inside_dataset', 'dataset_inside_output', 'parent_component'])
def test_cli_rejects_existing_or_nonseparate_output_before_loading_data(owned_paths, monkeypatch, case):
    p = owned_paths
    source = p.root/'dataset'
    source.mkdir()
    if case == 'existing':
        output = p.root/'existing'
        output.mkdir()
    elif case == 'inside_dataset':
        output = source/'derived'
    elif case == 'dataset_inside_output':
        output = p.root
    else:
        output = p.root/'new'/'..'/'derived'
    monkeypatch.setattr(mapping_compare, 'load_dataset', lambda *a, **k: pytest.fail('must reject before input loading'))
    with pytest.raises(ValueError, match='output must be new, unlinked and separate'):
        mapping_compare.main(['--dataset', str(source), '--project-root', str(p.root), '--output', str(output)])
