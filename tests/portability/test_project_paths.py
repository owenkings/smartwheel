"""Relocation must preserve execution identity and path/argument boundaries."""
import os
from pathlib import Path
import subprocess

import pytest

from wc_runtime import project_paths as paths


def checkout(path):
    (path/'scripts').mkdir(parents=True)
    (path/'scripts/wc_phase1').write_text('entry')
    (path/'src/wc_runtime').mkdir(parents=True)
    (path/'config').mkdir()
    return path


def test_source_and_installed_entry_discover_same_checkout(tmp_path, monkeypatch):
    monkeypatch.delenv('WHEELCHAIR_PROJECT_ROOT', raising=False)
    root = checkout(tmp_path/'another account'/'project with spaces')
    installed = root/'install/main/wc_bringup/lib/wc_bringup/wc_phase1'
    installed.parent.mkdir(parents=True); installed.write_text('entry')
    monkeypatch.chdir(tmp_path)
    assert paths.project_root(root/'scripts/wc_phase1') == root
    assert paths.project_root(installed) == root


def test_invalid_override_does_not_fall_back_to_other_checkout(tmp_path, monkeypatch):
    monkeypatch.setenv('WHEELCHAIR_PROJECT_ROOT', str(tmp_path/'missing'))
    with pytest.raises(RuntimeError, match='root not found'):
        paths.project_root()


def test_ros_shell_keeps_paths_and_arguments_literal(tmp_path, monkeypatch):
    from wc_runtime import cli
    root = checkout(tmp_path/'space $HOME; literal')
    setup = tmp_path/'ros setup.bash'; setup.write_text('export WC_ROS_TEST=ok\n')
    local = root/'install/main/setup.bash'; local.parent.mkdir(parents=True)
    local.write_text('export WC_PROJECT_TEST=ok\n')
    monkeypatch.setattr(cli, 'ROOT', root)
    monkeypatch.setenv('WHEELCHAIR_ROS_SETUP', str(setup))
    payload = '$(touch BAD_FILE); $HOME `literal`'
    command = cli.ros_command(['/usr/bin/python3', '-c',
        'import os,sys; print(os.environ["WHEELCHAIR_PROJECT_ROOT"]); print(sys.argv[1])', payload])
    result = subprocess.run(command, cwd=tmp_path, text=True, capture_output=True, check=True)
    assert result.stdout.splitlines() == [str(root), payload]
    assert not (tmp_path/'BAD_FILE').exists()


def test_host_lock_namespace_is_shared_across_clones(tmp_path, monkeypatch):
    first = checkout(tmp_path/'first'); second = checkout(tmp_path/'second')
    monkeypatch.setenv('WHEELCHAIR_PROJECT_ROOT', str(first)); a = paths.shared_lock_root()
    monkeypatch.setenv('WHEELCHAIR_PROJECT_ROOT', str(second)); b = paths.shared_lock_root()
    assert a == b
    assert a.stat().st_uid == os.geteuid()
    assert a.stat().st_mode & 0o777 == 0o700


def test_runtime_for_long_project_has_short_unique_socket_root(tmp_path):
    a = checkout(tmp_path/('a'*80)/('b'*80))
    b = checkout(tmp_path/('c'*80)/('d'*80))
    assert paths.socket_root(a) != paths.socket_root(b)
    # 24-character session digest, slash and manual.sock still fit sockaddr_un.
    assert len(os.fsencode(paths.socket_root(a)/('e'*24)/'manual.sock')) < 108


def test_existing_unsafe_runtime_directory_is_rejected(tmp_path):
    unsafe = tmp_path/'unsafe'; unsafe.mkdir(mode=0o777); unsafe.chmod(0o777)
    with pytest.raises(RuntimeError, match='0700'):
        paths._private_directory(unsafe)


def test_complete_checkout_missing_storage_does_not_fall_back(tmp_path):
    from wc_runtime.storage_policy import StoragePolicy
    root = checkout(tmp_path/'new installation')
    with pytest.raises(ValueError, match='STORAGE_CONFIG_MISSING'):
        StoragePolicy(root)


def test_runtime_accepts_linux_pc_without_account_or_hostname_gate(monkeypatch):
    monkeypatch.setattr(paths.platform, 'system', lambda: 'Linux')
    monkeypatch.setattr(paths.platform, 'machine', lambda: 'x86_64')
    paths.require_linux_runtime()
    monkeypatch.setattr(paths.platform, 'system', lambda: 'Windows')
    with pytest.raises(RuntimeError, match='Linux'):
        paths.require_linux_runtime()
