"""Panel writes stay bound to the actual complete configuration selected at edit time."""
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil

import pytest

from wc_panel import config_profiles
from wc_panel.config_profiles import ConfigProfiles
from wc_panel.parameters import FILES, ParameterStore
from wc_panel.storage import atomic_json


PROJECT = Path(__file__).resolve().parents[2]


@pytest.fixture
def panel(tmp_path):
    project = tmp_path / 'project'
    (project / 'config').mkdir(parents=True)
    # Synthetic projects use one coherent public template, never host bindings.
    for name in (*FILES, 'device_bindings.json', 'wheel_feedback_current.json'):
        shutil.copyfile(PROJECT / 'config' / name, project / 'config' / name)
    setup = json.loads((project / 'config/hardware_setup.json').read_text(encoding='utf-8'))
    for source in setup.get('source_documents', []):
        destination = project / source['snapshot_path']
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(PROJECT / source['snapshot_path'], destination)
    atomic_json(project / 'config/storage.json', {'schema_version': 1, 'enabled': False})
    return ConfigProfiles(ParameterStore(project, tmp_path / 'backups'))


def originals(panel):
    return {name: (panel.project / 'config' / name).read_bytes() for name in FILES}


def localize(panel, names=FILES):
    local = panel.project / 'config/local/project'
    local.mkdir(parents=True, exist_ok=True)
    for name in names:
        shutil.copyfile(panel.project / 'config' / name, local / name)
        os.chmod(local / name, 0o600)
    return {name: local / name for name in names}


def change_wheels(panel):
    store = panel.store
    group = next(row for row in store.read()['groups'] if row['id'] == 'wheels')
    value = copy.deepcopy(group['value'])
    value['wheel_radius_m'] += .01
    permission = store.request_edit('wheels')
    return store.save('wheels', value, permission['token'], permission['revision'])


def saved(panel):
    return panel.save_current('device parameters', panel.store._read()[2])


def interrupt_apply(panel, monkeypatch):
    profile = saved(panel)
    change_wheels(panel)
    before = panel.store._read()[0]
    preview = panel.preview(profile['id'])
    paths = panel.store.selected_paths()
    original_replace = config_profiles._replace

    def interrupt(path, raw):
        if Path(path) == paths['mapping_live.json']:
            raise KeyboardInterrupt('synthetic interruption after the first config write')
        return original_replace(path, raw)

    with monkeypatch.context() as patch:
        patch.setattr(config_profiles, '_replace', interrupt)
        with pytest.raises(KeyboardInterrupt):
            panel.apply(preview['token'])
    assert (panel.root / '.transaction.json').is_file()
    pointer = json.loads((panel.root / '.transaction.json').read_text(encoding='utf-8'))
    return before, panel.backup_root / pointer['backup']


def test_parameters_write_selected_local_and_display_its_path(panel):
    public = originals(panel)
    paths = localize(panel)
    group = next(row for row in panel.store.read()['groups'] if row['id'] == 'wheels')
    assert group['path'] == str(paths['hardware_setup.json'])
    result = change_wheels(panel)
    assert originals(panel) == public
    assert paths['hardware_setup.json'].read_bytes() != public['hardware_setup.json']
    assert (Path(result['backup']) / 'hardware_setup.json').read_bytes() == public['hardware_setup.json']
    revision = json.loads((Path(result['backup']) / 'revision.json').read_text(encoding='utf-8'))
    assert revision['source_paths']['hardware_setup.json'] == 'config/local/project/hardware_setup.json'
    if os.name == 'posix':
        assert paths['hardware_setup.json'].stat().st_mode & 0o777 == 0o600


@pytest.mark.parametrize('direction', ['base_to_local', 'local_to_base'])
def test_same_bytes_different_selection_invalidates_open_edit(panel, direction):
    paths = localize(panel) if direction == 'local_to_base' else None
    permission = panel.store.request_edit('wheels')
    value = next(row['value'] for row in panel.store.read()['groups'] if row['id'] == 'wheels')
    if paths:
        for path in paths.values():
            path.unlink()
    else:
        localize(panel)
    assert panel.store._read()[2] != permission['revision']
    with pytest.raises(ValueError, match='改变'):
        panel.store.save('wheels', value, permission['token'], permission['revision'])


def test_wheel_identity_local_selection_changes_revision_even_with_same_bytes(panel):
    before = panel.store._read()[2]
    localize(panel, ('wheel_feedback_current.json',))
    assert panel.store._read()[2] != before


def test_invalid_local_never_falls_back_to_public(panel):
    public = originals(panel)
    paths = localize(panel)
    paths['mapping_live.json'].write_bytes(b'{invalid local')
    with pytest.raises(ValueError):
        panel.store.read()
    with pytest.raises(ValueError):
        panel.list()
    assert originals(panel) == public


def test_partial_parameter_failure_rolls_back_local_bytes_only(panel, monkeypatch):
    public = originals(panel)
    paths = localize(panel)
    before = {name: path.read_bytes() for name, path in paths.items()}
    snapshot = panel.store.read()
    value = copy.deepcopy(next(row['value'] for row in snapshot['groups'] if row['id'] == 'display'))
    value['rviz_renderer'] = 'system' if value['rviz_renderer'] == 'software' else 'software'
    writer, failed = panel.store._write_config, False

    def fail_once(path, raw):
        nonlocal failed
        if Path(path) == paths['mapping_live.json'] and not failed:
            failed = True
            raise OSError('synthetic second-file failure')
        return writer(path, raw)

    monkeypatch.setattr(panel.store, '_write_config', fail_once)
    with pytest.raises(OSError, match='synthetic'):
        panel.store.save('display', value, expected_revision=snapshot['revision'])
    assert {name: path.read_bytes() for name, path in paths.items()} == before
    assert originals(panel) == public


def test_profile_switch_and_backup_use_local_paths(panel):
    public = originals(panel)
    paths = localize(panel)
    profile = saved(panel)
    manifest = json.loads((Path(profile['path']) / 'profile.json').read_text(encoding='utf-8'))
    assert manifest['schema_version'] == 2
    assert manifest['source_paths'] == {name: 'config/local/project/' + name for name in FILES}
    change_wheels(panel)
    before = panel.store._read()[0]
    preview = panel.preview(profile['id'])
    assert preview['paths'] == [str(paths[name]) for name in FILES]
    result = panel.apply(preview['token'])
    assert originals(panel) == public
    assert panel.store._read()[1]['hardware_setup.json'] == json.loads(public['hardware_setup.json'])
    for name in FILES:
        assert (Path(result['backup']) / name).read_bytes() == before[name]
    assert panel.list()['config_paths'] == [str(paths[name]) for name in FILES]


def test_interrupted_profile_rolls_back_original_local_bytes(panel, monkeypatch):
    public = originals(panel)
    paths = localize(panel)
    before, backup = interrupt_apply(panel, monkeypatch)
    panel.list()
    assert {name: path.read_bytes() for name, path in paths.items()} == before
    assert originals(panel) == public
    assert json.loads((backup / 'transaction.json').read_text(encoding='utf-8'))['state'] == 'ROLLED_BACK'


def test_interrupted_profile_rejects_selection_change_without_touching_public(panel, monkeypatch):
    public = originals(panel)
    paths = localize(panel)
    _, backup = interrupt_apply(panel, monkeypatch)
    for path in paths.values():
        path.unlink()
    with pytest.raises(ValueError, match='来源路径'):
        panel.list()
    assert originals(panel) == public
    assert (panel.root / '.transaction.json').is_file()
    assert json.loads((backup / 'transaction.json').read_text(encoding='utf-8'))['state'] == 'PREPARED'


def test_legacy_profile_supported_only_on_verified_base_selection(panel):
    profile = saved(panel)
    path = Path(profile['path']) / 'profile.json'
    manifest = json.loads(path.read_text(encoding='utf-8'))
    manifest['schema_version'] = 1
    manifest.pop('source_paths')
    atomic_json(path, manifest)
    assert panel.preview(profile['id'])['changes'] == []
    localize(panel)
    with pytest.raises(ValueError, match='来源路径'):
        panel.preview(profile['id'])


def test_legacy_transaction_can_recover_verified_base_but_not_unknown_state(panel, monkeypatch):
    before, backup = interrupt_apply(panel, monkeypatch)
    path = backup / 'transaction.json'
    journal = json.loads(path.read_text(encoding='utf-8'))
    journal.pop('schema_version')
    journal.pop('source_paths')
    journal['state'] = 'UNSUPPORTED'
    atomic_json(path, journal)
    with pytest.raises(ValueError, match='恢复状态'):
        panel.list()
    assert (panel.root / '.transaction.json').exists()
    journal['state'] = 'PREPARED'
    atomic_json(path, journal)
    panel.list()
    assert panel.store._read()[0] == before


def test_live_calibration_resolves_only_current_project_config_paths(panel):
    from wc_panel.live_calibration import config_path, device_context
    from wc_runtime.device_bindings import load_device_bindings
    paths = localize(panel, (*FILES, 'wheel_feedback_current.json'))
    for name in (*FILES, 'wheel_feedback_current.json'):
        assert config_path(panel.project, 'config/' + name) == paths[name]
    frozen = panel.project / 'data/frozen/config/hardware_setup.json'
    frozen.parent.mkdir(parents=True)
    frozen.write_bytes(b'{"frozen": true}')
    assert config_path(panel.project, frozen) == frozen
    mapping = json.loads(paths['mapping_live.json'].read_text(encoding='utf-8'))
    mapping['imu_sensor_id'] = load_device_bindings(panel.project)['imu']['sensor_id']
    mapping['continuous_mapping'] = not mapping.get('continuous_mapping', False)
    atomic_json(paths['mapping_live.json'], mapping)
    assert device_context(panel.project)['continuous_mapping'] == mapping['continuous_mapping']
