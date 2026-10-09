"""Machine configuration selection and original-byte freezing, without devices."""
import hashlib
import json
from pathlib import Path
import sys

import pytest

from wc_runtime.project_config import (
    LIDAR_CONFIG_FILES, PROJECT_CONFIG_NAMES, lidar_config_directory, selected_config_path,
)


def write(path, raw=b'{"source":"base"}\n'):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(raw)
    return path


def link(path, target, *, directory=False):
    try:
        path.symlink_to(target, target_is_directory=directory)
    except OSError as error:
        pytest.skip('host does not permit symlink creation: ' + str(error))


@pytest.mark.parametrize('name', sorted(PROJECT_CONFIG_NAMES))
def test_local_replaces_complete_file_and_preserves_original_bytes(tmp_path, name):
    base = write(tmp_path/'config'/name, b'{"base_only":true,"source":"base"}\n')
    assert selected_config_path(tmp_path, name) == base
    original = b'{\n  "source" : "local"\n}\n'
    local = write(tmp_path/'config/local/project'/name, original)
    selected = selected_config_path(tmp_path, Path(name))
    assert selected == local and selected.read_bytes() == original
    assert 'base_only' not in json.loads(selected.read_bytes())
    assert base.read_bytes() == b'{"base_only":true,"source":"base"}\n'


@pytest.mark.parametrize('name', ('../cameras.json', 'config/cameras.json', 'unknown.json', '/cameras.json'))
def test_selector_refuses_names_outside_explicit_current_config_contract(tmp_path, name):
    with pytest.raises(ValueError, match='NAME_UNSUPPORTED'):
        selected_config_path(tmp_path, name)


@pytest.mark.parametrize('raw', (b'{broken', b'', b'\xff'))
def test_invalid_local_never_falls_back_to_valid_base(tmp_path, raw):
    write(tmp_path/'config/cameras.json')
    write(tmp_path/'config/local/project/cameras.json', raw)
    with pytest.raises((ValueError, UnicodeError)):
        selected_config_path(tmp_path, 'cameras.json')


def test_local_directory_cannot_masquerade_as_missing_file(tmp_path):
    write(tmp_path/'config/cameras.json')
    (tmp_path/'config/local/project/cameras.json').mkdir(parents=True)
    with pytest.raises(ValueError, match='FILE_REQUIRED'):
        selected_config_path(tmp_path, 'cameras.json')


def test_non_directory_local_parent_prevents_base_fallback(tmp_path):
    write(tmp_path/'config/cameras.json')
    write(tmp_path/'config/local', b'not a directory')
    with pytest.raises(ValueError, match='PARENT_NOT_DIRECTORY'):
        selected_config_path(tmp_path, 'cameras.json')


@pytest.mark.parametrize('broken', (False, True))
def test_linked_local_file_is_never_followed_or_ignored(tmp_path, broken):
    write(tmp_path/'config/cameras.json')
    local = tmp_path/'config/local/project/cameras.json'
    local.parent.mkdir(parents=True)
    target = tmp_path/'target.json'
    if not broken:
        write(target)
    link(local, target)
    with pytest.raises(ValueError, match='LINKED_PATH'):
        selected_config_path(tmp_path, 'cameras.json')


def test_linked_local_directory_is_rejected_even_without_selected_file(tmp_path):
    write(tmp_path/'config/cameras.json')
    destination = tmp_path/'elsewhere'
    destination.mkdir()
    link(tmp_path/'config/local', destination, directory=True)
    with pytest.raises(ValueError, match='LINKED_PATH'):
        selected_config_path(tmp_path, 'cameras.json')


def driver_files(directory, label):
    for name in LIDAR_CONFIG_FILES:
        write(directory/name, (label + ':' + name + '\n').encode())
    return directory


def test_lidar_uses_one_complete_local_directory(tmp_path):
    base = driver_files(tmp_path/'src/wc_xt_driver/config', 'base')
    assert lidar_config_directory(tmp_path) == base
    local = driver_files(tmp_path/'config/local/lidar', 'local')
    assert lidar_config_directory(tmp_path) == local
    assert (local/'left.yaml').read_bytes() != (base/'left.yaml').read_bytes()


@pytest.mark.parametrize('missing', LIDAR_CONFIG_FILES)
def test_incomplete_local_lidar_directory_never_borrows_base_files(tmp_path, missing):
    driver_files(tmp_path/'src/wc_xt_driver/config', 'base')
    local = driver_files(tmp_path/'config/local/lidar', 'local')
    (local/missing).unlink()
    with pytest.raises(FileNotFoundError):
        lidar_config_directory(tmp_path)


def test_linked_lidar_file_is_rejected(tmp_path):
    base = driver_files(tmp_path/'src/wc_xt_driver/config', 'base')
    local = driver_files(tmp_path/'config/local/lidar', 'local')
    (local/'left.yaml').unlink()
    link(local/'left.yaml', base/'left.yaml')
    with pytest.raises(ValueError, match='LINKED_PATH'):
        lidar_config_directory(tmp_path)


@pytest.mark.skipif(sys.platform != 'linux', reason='runtime integration requires Linux fcntl')
def test_capture_freezes_selected_raw_bytes_and_keeps_archive_independent(tmp_path, monkeypatch):
    from wc_runtime import capture, capture_support, cli, hardware_setup
    names = ('mapping_live.json', 'hardware_setup.json', 'wheel_feedback_current.json', 'cameras.json')
    selected = {}
    for name in names:
        write(tmp_path/'config'/name)
        raw = ('{\n "selected" : "' + name + '"\n}\n').encode()
        selected[name] = write(tmp_path/'config/local/project'/name, raw)
    write(tmp_path/'config/calibration/evidence.json', b'{"observed":true}\n')
    local_lidar = driver_files(tmp_path/'config/local/lidar', 'selected')
    write(local_lidar/'README.md', b'Not a runtime driver input.\n')
    monkeypatch.setattr(cli, 'require_device_bindings', lambda: {'imu': {'sensor_id': 'H30-TEST'}})
    monkeypatch.setattr(hardware_setup, 'resolve_hardware_setup', lambda value: {})
    monkeypatch.setattr(capture_support, 'software_snapshot', lambda root, output: {})
    class Storage:
        def snapshot_configuration(self, directory):
            return {}
    output = tmp_path/'recording'
    output.mkdir()
    hashes = capture.snapshot(tmp_path, output, profile='mapping_cameras', storage_policy=Storage())
    frozen = output/'configuration'
    destinations = {'mapping_live.json': 'runtime_config_source.json', 'hardware_setup.json': 'hardware_setup.json',
                    'wheel_feedback_current.json': 'wheel_feedback.json', 'cameras.json': 'cameras.json'}
    for name, destination in destinations.items():
        raw = selected[name].read_bytes()
        assert (frozen/destination).read_bytes() == raw
        assert hashes['configuration/'+destination] == hashlib.sha256(raw).hexdigest()
        selected[name].write_bytes(b'{"changed_after_recording":true}')
        assert (frozen/destination).read_bytes() == raw
    assert json.loads((frozen/'runtime_config.json').read_bytes())['imu_sensor_id'] == 'H30-TEST'
    for name in LIDAR_CONFIG_FILES:
        assert (frozen/'lidar'/name).read_bytes() == (local_lidar/name).read_bytes()
    assert not (frozen/'lidar/README.md').exists()
    assert (frozen/'calibration/evidence.json').read_bytes() == b'{"observed":true}\n'


@pytest.mark.skipif(sys.platform != 'linux', reason='runtime integration requires Linux fcntl')
def test_mapping_path_selection_never_redirects_explicit_archived_input(tmp_path, monkeypatch):
    from wc_runtime import mapping_controller
    monkeypatch.setattr(mapping_controller, 'ROOT', tmp_path)
    base = write(tmp_path/'config/mapping_live.json')
    local = write(tmp_path/'config/local/project/mapping_live.json', b'{"source":"local"}')
    archived = write(tmp_path/'data/recording/configuration/mapping_live.json', b'{"source":"archive"}')
    assert mapping_controller._current_configuration_path(base) == local
    assert mapping_controller._current_configuration_path(archived) == archived
    assert mapping_controller._current_configuration_path(base, select_local=False) == base
    assert not mapping_controller._is_current_configuration(archived)


@pytest.mark.skipif(sys.platform != 'linux', reason='runtime integration requires Linux fcntl')
def test_driver_command_selects_local_directory_without_opening_hardware(tmp_path, monkeypatch):
    from types import SimpleNamespace
    from wc_runtime import cli
    local = driver_files(tmp_path/'config/local/lidar', 'local')
    monkeypatch.setattr(cli, 'ROOT', tmp_path)
    monkeypatch.setattr(cli, 'ros_command', lambda args: args)
    args = SimpleNamespace(mode='dual', session='test', duration=30, device_config_policy='preserve_current')
    command = cli.driver_command(args, True)
    assert 'config_directory:='+str(local) in command
    assert 'read_only_probe:=true' in command
    assert 'device_config_policy:=preserve_current' in command
