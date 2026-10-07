"""Synthetic storage compatibility; no devices, ROS processes or GUI started."""
import errno
import hashlib
import importlib.util
import json
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest

from wc_runtime import capture_destination
from wc_runtime import storage_atomic
from wc_runtime import calibration_picker
from wc_runtime import mapping_app
from wc_runtime import mapping_control_paths
from wc_runtime.prepare_picker_input import project_path
from wc_runtime.single_mapping_input import FrameArchive, InputFailure, project_path as frame_path


@pytest.fixture
def storage(tmp_path, monkeypatch):
    code = tmp_path/'code'
    archive = tmp_path/'volume/wheelchair'
    (code/'config').mkdir(parents=True)
    archive.mkdir(parents=True)
    state = {'available': True}

    class Guard:
        def __init__(self, output_root, required_uuid):
            self.mount_root = archive.parent
            self.output_root = output_root
            self.required_uuid = required_uuid
            self.check()

        def check(self):
            if not state['available']:
                raise ValueError('CAPTURE_DESTINATION_UNAVAILABLE: synthetic unplug')
            return {'status': 'AVAILABLE', 'mount_point': str(self.mount_root), 'uuid': self.required_uuid}

    monkeypatch.setattr(capture_destination, 'CaptureDestination', Guard)
    monkeypatch.setattr(storage_atomic, 'PROJECT_ROOT', code)
    (code/'config/storage.json').write_text(json.dumps({
        'schema_version': 1, 'enabled': True, 'archive_root': str(archive),
        'mount_point': str(archive.parent), 'required_uuid': 'SYNTHETIC-USB',
        'original_project_root': '/home/nvidia/wheelchair', 'fallback_allowed': False}), encoding='utf-8')
    return SimpleNamespace(code=code, archive=archive, state=state)


def test_map_outputs_and_default_saved_map_go_to_usb(storage):
    request = mapping_app.prepare_request('all', name='usb_map', project_root=storage.code)
    assert Path(request['output_dir']) == storage.archive/'reports/maps/usb_map'
    assert Path(request['config_path']) == storage.code/'config/mapping_live.json'
    assert mapping_app.default_save_destination(request) == storage.archive/'maps/usb_map'
    assert not (storage.code/'reports').exists()


def test_non_data_map_override_cannot_silently_write_internal_disk(storage):
    with pytest.raises(ValueError):
        mapping_app.prepare_request('left', name='x', output='some_internal_folder', project_root=storage.code)


def test_legacy_absolute_data_and_source_paths_are_resolved_separately(storage):
    assert project_path(storage.code, '/home/nvidia/wheelchair/data/old.json') == storage.archive/'data/old.json'
    assert project_path(storage.code, 'src/module.py') == storage.code/'src/module.py'
    with pytest.raises(ValueError):
        project_path(storage.code, '/tmp/unauthorised.json')
    with pytest.raises(InputFailure):
        frame_path(storage.code, storage.archive)


def test_frozen_frame_archive_detects_usb_loss_without_internal_fallback(storage):
    archive = FrameArchive(storage.code, 'data/frames/synthetic')
    try:
        archive.append(b'SYNTHETIC_CDR', {'sequence': 1})
        storage.state['available'] = False
        with pytest.raises(ValueError, match='DESTINATION_UNAVAILABLE'):
            archive.storage.check()
        with pytest.raises(InputFailure):
            archive.append(b'AFTER_UNPLUG', {'sequence': 2})
    finally:
        archive.close()
    assert not (storage.code/'data').exists()
    assert len(list((storage.archive/'data/frames/synthetic/frames').glob('*.cdr'))) == 1


def test_existing_picker_pointer_resolves_without_rewriting_archive(storage, monkeypatch):
    monkeypatch.setattr(calibration_picker, 'ROOT', storage.code)
    prepared = storage.archive/'reports/calibration/old/prepared.json'
    prepared.parent.mkdir(parents=True)
    content = b'{"training": [{"id": "scene_1"}]}'
    prepared.write_bytes(content)
    (storage.code/'state').mkdir()
    pointer = {'schema_version': 1, 'status': 'READY_FOR_OFFLINE_PICKING',
               'prepared_input': 'reports/calibration/old/prepared.json',
               'prepared_file_sha256': hashlib.sha256(content).hexdigest(), 'scene_id': 'scene_1'}
    (storage.code/'state/point_picker_input.json').write_text(json.dumps(pointer))
    assert calibration_picker.resolve_input() == prepared
    assert calibration_picker.resolve_input('/home/nvidia/wheelchair/reports/calibration/old/prepared.json') == prepared
    assert prepared.read_bytes() == content


@pytest.mark.parametrize('module_name,relative,kwargs', [
    ('mapping_app.launch.py', 'reports/maps/map_1', {}),
    ('single_mapping.launch.py', 'reports/single_mapping/map_1', {'side': 'left'}),
])
def test_launch_database_is_on_usb_and_existing_database_is_rejected(storage, module_name, relative, kwargs):
    source = Path(__file__).resolve().parents[2]/'src/wc_bringup/launch'/module_name
    spec = importlib.util.spec_from_file_location('_usb_launch_'+module_name, source)
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    session = storage.archive/relative
    (session/'slam').mkdir(parents=True)
    result = launch.build_spec(session_root=storage.code/relative, project_root=storage.code, **kwargs)
    assert str(session/'slam/rtabmap.db') in str(result)
    (session/'slam/rtabmap.db').write_bytes(b'EXISTING')
    with pytest.raises(ValueError, match='existing database'):
        launch.build_spec(session_root=session, project_root=storage.code, **kwargs)


def test_manual_socket_remains_short_and_distinguishes_archive_and_ram(storage, monkeypatch, tmp_path):
    identity = 'usb_manual_session'
    local = tmp_path/'private_socket_root'
    monkeypatch.setattr(mapping_control_paths, 'socket_root', lambda root: local)
    archive = storage.archive/'reports/maps/x'
    ram = Path('/dev/shm/wc_capture')/identity
    def expected(directory):
        key = hashlib.sha256((str(directory.absolute()) + '\0' + identity).encode('utf-8')).hexdigest()[:24]
        return local/key/'manual.sock'
    assert mapping_control_paths.manual_socket_path(storage.code, archive, identity) == expected(archive)
    assert mapping_control_paths.manual_socket_path(storage.code, ram, identity) == expected(ram)
    assert expected(archive) != expected(ram)
    storage.state['available'] = False
    with pytest.raises(ValueError, match='DESTINATION_UNAVAILABLE'):
        mapping_control_paths.manual_socket_path(storage.code, archive, identity)


def test_read_only_view_maps_old_project_reference(storage, monkeypatch, capsys):
    from wc_runtime import map_viewer
    observed = []
    monkeypatch.setattr(map_viewer, 'map_source', lambda p: observed.append(p) or {'path': str(p)})
    monkeypatch.setattr(map_viewer, 'inspect_source', lambda p: {'status': 'SYNTHETIC'})
    assert map_viewer.main(['data/analysis/export', '--check'], project_root=storage.code) == 0
    assert observed == [storage.archive/'data/analysis/export']


@pytest.mark.skipif(__import__('os').name != 'posix', reason='target flock publication contract')
def test_cooperative_commit_rejects_existing_target_and_unconfigured_external(storage):
    parent = storage.archive/'data/atomic'; parent.mkdir(parents=True)
    source, target = parent/'.stage', parent/'complete'
    source.write_bytes(b'ORIGINAL')
    receipt = storage_atomic.rename_on_configured_archive(source, target)
    assert target.read_bytes() == b'ORIGINAL'
    assert receipt['external_writer_exclusion_required'] is True
    source.write_bytes(b'REPLACEMENT')
    with pytest.raises(FileExistsError):
        storage_atomic.rename_on_configured_archive(source, target)
    assert target.read_bytes() == b'ORIGINAL' and source.read_bytes() == b'REPLACEMENT'
    with pytest.raises(ValueError, match='configured archive'):
        storage_atomic.rename_on_configured_archive(storage.code/'x', storage.code/'y')


@pytest.mark.skipif(__import__('os').name != 'posix', reason='target flock publication contract')
def test_cooperative_commit_one_winner_under_writer_contention(storage):
    parent = storage.archive/'reports/contention'; parent.mkdir(parents=True)
    sources = [parent/f'.stage_{i}' for i in range(4)]
    for i, source in enumerate(sources):
        source.write_bytes(str(i).encode())
    target = parent/'complete'
    barrier = threading.Barrier(len(sources))
    outcomes = []
    def publish(source):
        barrier.wait()
        try:
            storage_atomic.rename_on_configured_archive(source, target)
            outcomes.append('OK')
        except FileExistsError:
            outcomes.append('EXISTS')
    workers = [threading.Thread(target=publish, args=(source,)) for source in sources]
    for worker in workers: worker.start()
    for worker in workers: worker.join(10)
    assert not any(worker.is_alive() for worker in workers)
    assert sorted(outcomes) == ['EXISTS', 'EXISTS', 'EXISTS', 'OK']
    assert sum(source.exists() for source in sources) == 3


@pytest.mark.skipif(__import__('os').name != 'posix', reason='target descriptor archive contract')
def test_raw_archive_falls_back_only_for_unsupported_hard_links(storage, monkeypatch):
    from wc_fusion import archive
    parent = storage.archive/'data/raw'; parent.mkdir(parents=True)
    def unsupported(*args, **kwargs):
        raise OSError(errno.EOPNOTSUPP, 'SYNTHETIC exFAT')
    monkeypatch.setattr(archive.os, 'link', unsupported)
    target = parent/'cloud.json'
    archive._commit_bytes(target, b'{"frame":1}', threading.Event())
    assert target.read_bytes() == b'{"frame":1}'
    with pytest.raises(FileExistsError):
        archive._commit_bytes(target, b'{"frame":2}', threading.Event())
    assert target.read_bytes() == b'{"frame":1}'
    assert not list(parent.glob('*.tmp'))


@pytest.mark.skipif(__import__('os').name != 'posix', reason='target file publication contract')
def test_mapping_bootstrap_publishes_on_hardlink_and_rename_flags_unsupported(storage, monkeypatch):
    from wc_runtime import mapping_input
    from wc_maps import store
    parent = storage.archive/'reports/bootstrap'; parent.mkdir(parents=True)
    def unsupported(*args, **kwargs):
        raise OSError(errno.EOPNOTSUPP, 'SYNTHETIC exFAT')
    monkeypatch.setattr(mapping_input.os, 'link', unsupported)
    monkeypatch.setattr(store, '_rename_no_replace', storage_atomic.rename_on_configured_archive)
    output = parent/'prior_config.json'
    mapping_input.write_exclusive_json(storage.code, output, {'status': 'READY'})
    assert json.loads(output.read_text()) == {'status': 'READY'}
    with pytest.raises(FileExistsError):
        mapping_input.write_exclusive_json(storage.code, output, {'status': 'REPLACED'})
    assert json.loads(output.read_text()) == {'status': 'READY'}
