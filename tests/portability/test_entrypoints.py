"""Relocated entry points: no ROS, sensors, windows or control backend started."""
import ast
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
WRAPPERS = ('wc_phase1', 'map', 'view_map', 'save_map', 'confirm_gyro_bias', 'capture_usb')
ENTRY_MODULES = {'wc_phase1': 'cli', 'map': 'mapping_app', 'view_map': 'map_viewer',
                 'save_map': 'map_save_app', 'confirm_gyro_bias': 'mapping_bias_confirm',
                 'capture_usb': 'cli'}


def checkout(path):
    (path/'scripts').mkdir(parents=True)
    (path/'config').mkdir()
    (path/'config/storage.json').write_text('{}', encoding='utf-8')
    package = path/'src/wc_runtime'
    package.mkdir(parents=True)
    (package/'__init__.py').write_text('', encoding='utf-8')
    shutil.copyfile(ROOT/'src/wc_runtime/project_paths.py', package/'project_paths.py')
    for script in WRAPPERS:
        shutil.copyfile(ROOT/'scripts'/script, path/'scripts'/script)
    # These stubs report where imports/arguments came from. No real runtime code
    # is imported; a wrong path could never reach hardware through this test.
    for module in set(ENTRY_MODULES.values()):
        (package/(module+'.py')).write_text(
            'import json, os, sys\nfrom pathlib import Path\n'
            'def main(*args, **kwargs):\n'
            ' print(json.dumps({"module_root": str(Path(__file__).parents[2]), '
            '"explicit_root": str(kwargs.get("project_root", "")), '
            '"environment_root": os.environ.get("WHEELCHAIR_PROJECT_ROOT"), '
            '"argv": sys.argv[1:]}))\n return 0\n', encoding='utf-8')
    return path


@pytest.mark.parametrize('entry', WRAPPERS)
@pytest.mark.parametrize('installed', (False, True))
def test_entry_selects_own_checkout_from_source_and_install(tmp_path, entry, installed):
    selected = checkout(tmp_path/'different account'/'project with spaces')
    decoy = checkout(tmp_path/'wrong cwd project')
    script = selected/'scripts'/entry
    if installed:
        script = selected/'install/main/wc_bringup/lib/wc_bringup'/entry
        script.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(selected/'scripts'/entry, script)
    env = dict(os.environ, PYTHONPATH='', PYTHONDONTWRITEBYTECODE='1',
               WHEELCHAIR_PROJECT_ROOT=str(decoy))
    result = subprocess.run([sys.executable, '-s', str(script), '--sentinel', 'with space'],
                            cwd=decoy, env=env, capture_output=True, text=True, check=True)
    value = json.loads(result.stdout)
    assert Path(value['module_root']) == selected
    assert Path(value['environment_root']) == selected
    if entry in ('map', 'view_map', 'save_map'):
        assert Path(value['explicit_root']) == selected
    assert value['argv'] == (['capture'] if entry == 'capture_usb' else []) + ['--sentinel', 'with space']
    assert not list(selected.rglob('__pycache__'))
    assert not list(decoy.rglob('__pycache__'))


def test_unattached_wrapper_fails_instead_of_importing_cwd(tmp_path):
    selected = checkout(tmp_path/'cwd only')
    detached = tmp_path/'detached'/'wc_phase1'
    detached.parent.mkdir()
    shutil.copyfile(ROOT/'scripts/wc_phase1', detached)
    env = dict(os.environ, PYTHONPATH='', PYTHONDONTWRITEBYTECODE='1')
    env.pop('WHEELCHAIR_PROJECT_ROOT', None)
    result = subprocess.run([sys.executable, '-s', str(detached), '--help'], cwd=selected,
                            env=env, capture_output=True, text=True)
    assert result.returncode != 0
    assert 'Cannot locate the wheelchair checkout' in result.stderr


def test_long_archive_paths_and_repeated_ids_get_distinct_short_socket_names(tmp_path, monkeypatch):
    from wc_runtime import mapping_control_paths as control
    root = tmp_path/'project'
    data = tmp_path/('long archive '*15)/'session'
    private = tmp_path/'sockets'
    class Policy:
        enabled = False
    monkeypatch.setattr(control, 'StoragePolicy', lambda root: Policy())
    monkeypatch.setattr(control, 'socket_root', lambda root: private)
    first = control.manual_socket_path(root, data, 'same_id')
    second = control.manual_socket_path(root, data.parent/'second', 'same_id')
    assert first.parent.parent == private
    assert first.name == 'manual.sock'
    assert first.parent.name == hashlib.sha256((str(data.absolute())+'\0same_id').encode()).hexdigest()[:24]
    assert first != second
    with pytest.raises(ValueError):
        control.manual_socket_path(root, data/'..'/'other', 'same_id')


def test_socket_path_rejects_symlinked_session(tmp_path, monkeypatch):
    from wc_runtime import mapping_control_paths as control
    real = tmp_path/'real'; real.mkdir()
    linked = tmp_path/'linked'
    try:
        linked.symlink_to(real, target_is_directory=True)
    except OSError:
        pytest.skip('host does not allow symlink fixtures')
    with pytest.raises(ValueError, match='Linked'):
        control.manual_socket_path(tmp_path, linked, 'same_id')


def test_launch_passes_explicit_project_root_without_changing_mapping_parameters():
    for filename in ('mapping_app.launch.py', 'single_mapping.launch.py'):
        text = (ROOT/'src/wc_bringup/launch'/filename).read_text(encoding='utf-8')
        tree = ast.parse(text)
        function = next(x for x in tree.body if isinstance(x, ast.FunctionDef) and x.name == 'configure')
        calls = [x for x in ast.walk(function) if isinstance(x, ast.Call)
                 and isinstance(x.func, ast.Name) and x.func.id == 'build_spec']
        assert len(calls) == 1
        assert any(x.arg == 'project_root' for x in calls[0].keywords)
        assert "DeclareLaunchArgument('project_root', default_value=str(PROJECT_ROOT))" in text


def test_build_scripts_include_all_packages_and_do_not_pin_arm_multiarch():
    cmake = (ROOT/'src/wc_slam/CMakeLists.txt').read_text(encoding='utf-8')
    assert '${CMAKE_LIBRARY_ARCHITECTURE}' in cmake
    assert 'aarch64-linux-gnu' not in cmake
    assert 'OpenCV 4.5.4 EXACT' in cmake
    script = (ROOT/'scripts/build_verify.sh').read_text(encoding='utf-8')
    assert '--base-paths src --build-base' in script
    assert 'wc_camera_panel' in script


def test_maintained_scripts_do_not_select_old_account_or_host():
    files = list((ROOT/'scripts').iterdir()) + [ROOT/'tests/run_target_tests.sh']
    for path in files:
        if not path.is_file():
            continue
        text = path.read_text(encoding='utf-8')
        assert '/home/nvidia/wheelchair' not in text, path
        assert "getpass.getuser() != 'nvidia'" not in text, path
        assert "= nvidia" not in text, path
        assert "= aarch64" not in text, path
