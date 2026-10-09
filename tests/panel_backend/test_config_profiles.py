"""Profile switches retain device identity, source evidence and recovery bytes."""
import copy
import hashlib
import json
from pathlib import Path
import shutil
from unittest import mock

import pytest

from wc_panel.config_profiles import ConfigProfiles, FILES
from wc_panel.parameters import ParameterStore
from wc_panel.storage import atomic_json


PROJECT = Path(__file__).resolve().parents[2]


@pytest.fixture
def profiles(tmp_path):
    project = tmp_path / 'project'
    (project / 'config').mkdir(parents=True)
    for name in (*FILES, 'device_bindings.json', 'device_bindings.local.json', 'wheel_feedback_current.json'):
        source = PROJECT / 'config' / name
        if source.exists(): shutil.copyfile(source, project / 'config' / name)
    atomic_json(project / 'config/storage.json', dict(schema_version=1, enabled=False))
    setup = json.loads((project / 'config/hardware_setup.json').read_text(encoding='utf-8'))
    for source in setup['source_documents']:
        destination = project / source['snapshot_path']
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(PROJECT / source['snapshot_path'], destination)
    return ConfigProfiles(ParameterStore(project, tmp_path / 'backups'))


def save(profiles, name='日常配置'):
    return profiles.save_current(name, profiles.store._read()[2])


def change_wheels(profiles):
    snapshot = profiles.store.read()
    wheels = copy.deepcopy(next(row['value'] for row in snapshot['groups'] if row['id'] == 'wheels'))
    wheels['wheel_radius_m'] += .01
    permission = profiles.store.request_edit('wheels')
    return profiles.store.save('wheels', wheels, permission['token'], permission['revision'])


def alter_profile(profiles, identifier, filename, edit):
    directory = profiles.root / identifier
    document = json.loads((directory / filename).read_text(encoding='utf-8'))
    edit(document)
    atomic_json(directory / filename, document)
    manifest = json.loads((directory / 'profile.json').read_text(encoding='utf-8'))
    manifest['files'][filename] = hashlib.sha256((directory / filename).read_bytes()).hexdigest()
    atomic_json(directory / 'profile.json', manifest)


def test_switch_restores_supported_values_and_keeps_frozen_recording(profiles):
    original = profiles.store._read()[1]
    selected = save(profiles)
    assert profiles.list()['active_name'] == '日常配置'
    change_wheels(profiles)
    frozen = profiles.project / 'data/experiments/example/runtime_config.json'
    frozen.parent.mkdir(parents=True)
    frozen.write_bytes(b'{"immutable-recording":true}')
    assert profiles.list()['active_name'] == '当前配置（已调整）'
    before = profiles.store._read()[0]
    preview = profiles.preview(selected['id'])
    assert [row['group'] for row in preview['changes']] == ['wheels']
    assert profiles.store._read()[0] == before
    result = profiles.apply(preview['token'])
    after = profiles.store._read()[1]
    assert after['hardware_setup.json']['wheel_odometry'] == original['hardware_setup.json']['wheel_odometry']
    assert after['hardware_setup.json']['data_transforms'] == original['hardware_setup.json']['data_transforms']
    assert frozen.read_bytes() == b'{"immutable-recording":true}'
    for name in FILES:
        assert (Path(result['backup']) / name).read_bytes() == before[name]
    assert profiles.list()['active_name'] == '日常配置'


def test_preview_revision_and_token_are_required(profiles):
    selected = save(profiles)
    preview = profiles.preview(selected['id'])
    change_wheels(profiles)
    with pytest.raises(ValueError, match='改变'):
        profiles.apply(preview['token'])
    with pytest.raises(ValueError, match='预览'):
        profiles.apply('unknown')


def test_missing_or_changed_source_never_silently_restores(profiles):
    selected = save(profiles)
    setup = profiles.store._read()[1]['hardware_setup.json']
    source = profiles.project / setup['source_documents'][0]['snapshot_path']
    source.write_bytes(source.read_bytes() + b'changed')
    with pytest.raises(ValueError, match='改变'):
        profiles.preview(selected['id'])
    listed = profiles.list()['profiles'][0]
    assert not listed['available'] and '改变' in listed['error']


def test_profile_cannot_change_device_identity_or_assembly(profiles):
    selected = save(profiles)
    alter_profile(profiles, selected['id'], 'hardware_setup.json', lambda data: data['assembly'].update(revision='another'))
    with pytest.raises(ValueError, match='设备|装配'):
        profiles.preview(selected['id'])


def test_profile_cannot_promote_unknown_without_valid_source_binding(profiles):
    selected = save(profiles)
    def modify(data):
        transform = next(row for row in data['data_transforms'] if row['id'] == 'mounting_M_from_lidar_left')
        transform['translation'].update(value_m=[1, 2, 3], status='CALIBRATED')
    alter_profile(profiles, selected['id'], 'hardware_setup.json', modify)
    with pytest.raises(ValueError):
        profiles.preview(selected['id'])


def test_invalid_wheel_numbers_cannot_enter_through_snapshot(profiles):
    selected = save(profiles)
    alter_profile(profiles, selected['id'], 'hardware_setup.json', lambda data: data['wheel_odometry'].update(wheel_radius_m=-1))
    with pytest.raises(ValueError, match='大于零'):
        profiles.preview(selected['id'])


def test_failed_second_file_write_restores_exact_original_bytes(profiles):
    selected = save(profiles)
    change_wheels(profiles)
    preview = profiles.preview(selected['id'])
    before = profiles.store._read()[0]
    from wc_panel import config_profiles
    original_replace = config_profiles._replace
    injected = []
    def fail_once(path, raw):
        if Path(path) == profiles.project / 'config/mapping_live.json' and not injected:
            injected.append(True)
            raise OSError('synthetic write failure')
        return original_replace(path, raw)
    with mock.patch.object(config_profiles, '_replace', side_effect=fail_once):
        with pytest.raises(RuntimeError, match='已恢复原配置'):
            profiles.apply(preview['token'])
    assert profiles.store._read()[0] == before
    assert not (profiles.root / '.transaction.json').exists()


def test_interrupted_transaction_is_recovered_on_next_open(profiles):
    selected = save(profiles)
    change_wheels(profiles)
    preview = profiles.preview(selected['id'])
    before = profiles.store._read()[0]
    from wc_panel import config_profiles
    original_replace = config_profiles._replace
    def interrupt(path, raw):
        if Path(path) == profiles.project / 'config/mapping_live.json':
            raise KeyboardInterrupt('synthetic interrupted process')
        return original_replace(path, raw)
    with mock.patch.object(config_profiles, '_replace', side_effect=interrupt):
        with pytest.raises(KeyboardInterrupt): profiles.apply(preview['token'])
    assert (profiles.root / '.transaction.json').exists()
    reopened = ConfigProfiles(profiles.store)
    reopened.list()
    assert profiles.store._read()[0] == before
    assert not (profiles.root / '.transaction.json').exists()


def test_foreign_change_during_interruption_is_preserved_with_recovery_report(profiles):
    selected = save(profiles)
    change_wheels(profiles)
    preview = profiles.preview(selected['id'])
    from wc_panel import config_profiles
    original_replace = config_profiles._replace
    def interrupt(path, raw):
        if Path(path) == profiles.project / 'config/mapping_live.json':
            raise KeyboardInterrupt('synthetic interruption')
        return original_replace(path, raw)
    with mock.patch.object(config_profiles, '_replace', side_effect=interrupt):
        with pytest.raises(KeyboardInterrupt): profiles.apply(preview['token'])
    target = profiles.project / 'config/cameras.json'
    unrelated = target.read_bytes() + b'\n '
    target.write_bytes(unrelated)
    with pytest.raises(RuntimeError, match='其他修改'):
        profiles.list()
    assert target.read_bytes() == unrelated
    assert (profiles.root / '.transaction.json').exists()


def test_changed_device_binding_rejects_previously_saved_profile(profiles):
    selected = save(profiles)
    path = profiles.project / 'config/device_bindings.json'
    path.write_bytes(path.read_bytes() + b'\n')
    with pytest.raises(ValueError, match='设备身份'):
        profiles.preview(selected['id'])


def test_omitted_identity_digest_does_not_bypass_device_binding(profiles):
    selected = save(profiles)
    directory = profiles.root / selected['id']
    manifest = json.loads((directory / 'profile.json').read_text(encoding='utf-8'))
    manifest['dependencies'].pop('config/device_bindings.json')
    atomic_json(directory / 'profile.json', manifest)
    path = profiles.project / 'config/device_bindings.json'
    path.write_bytes(path.read_bytes() + b'\n')
    with pytest.raises(ValueError, match='清单不完整'):
        profiles.preview(selected['id'])


def test_profile_dependencies_cannot_escape_project(profiles):
    selected = save(profiles)
    outside = profiles.project.parent / 'outside.json'
    outside.write_text('{}')
    directory = profiles.root / selected['id']
    manifest = json.loads((directory / 'profile.json').read_text(encoding='utf-8'))
    manifest['dependencies'][str(outside)] = hashlib.sha256(outside.read_bytes()).hexdigest()
    atomic_json(directory / 'profile.json', manifest)
    with pytest.raises(ValueError, match='超出工程'):
        profiles.preview(selected['id'])


def test_profile_changed_after_preview_requires_new_confirmation(profiles):
    selected = save(profiles)
    preview = profiles.preview(selected['id'])
    alter_profile(profiles, selected['id'], 'hardware_setup.json', lambda data: data['wheel_odometry'].update(wheel_radius_m=.2))
    before = profiles.store._read()[0]
    with pytest.raises(ValueError, match='确认后改变'):
        profiles.apply(preview['token'])
    assert profiles.store._read()[0] == before


def test_rollback_failure_keeps_journal_and_original_backup(profiles):
    selected = save(profiles)
    change_wheels(profiles)
    preview = profiles.preview(selected['id'])
    before = profiles.store._read()[0]
    from wc_panel import config_profiles
    original_replace = config_profiles._replace
    def fail_write_and_rollback(path, raw):
        if Path(path) == profiles.project / 'config/mapping_live.json':
            raise OSError('synthetic device unavailable')
        return original_replace(path, raw)
    with mock.patch.object(config_profiles, '_replace', side_effect=fail_write_and_rollback):
        with pytest.raises(RuntimeError, match='回滚未完成.*恢复记录已保留'):
            profiles.apply(preview['token'])
    pointer = json.loads((profiles.root / '.transaction.json').read_text(encoding='utf-8'))
    backup = profiles.backup_root / pointer['backup']
    assert json.loads((backup / 'transaction.json').read_text(encoding='utf-8'))['state'] == 'ROLLING_BACK'
    for name in FILES:
        assert (backup / name).read_bytes() == before[name]
    profiles.list()
    assert profiles.store._read()[0] == before


def test_committed_pointer_cleanup_failure_does_not_rollback(profiles):
    selected = save(profiles)
    original_radius = profiles.store._read()[1]['hardware_setup.json']['wheel_odometry']['wheel_radius_m']
    change_wheels(profiles)
    preview = profiles.preview(selected['id'])
    pointer = profiles.root / '.transaction.json'
    original_unlink = Path.unlink
    failed = []
    def fail_once(path, *args, **kwargs):
        if path == pointer and not failed:
            failed.append(True)
            raise OSError('synthetic pointer cleanup failure')
        return original_unlink(path, *args, **kwargs)
    with mock.patch.object(Path, 'unlink', new=fail_once), mock.patch.object(profiles, '_rollback', wraps=profiles._rollback) as rollback:
        result = profiles.apply(preview['token'])
        rollback.assert_not_called()
    assert result['cleanup_warning'] and pointer.exists()
    assert profiles.store._read()[1]['hardware_setup.json']['wheel_odometry']['wheel_radius_m'] == original_radius
    journal = json.loads((Path(result['backup']) / 'transaction.json').read_text(encoding='utf-8'))
    assert journal['state'] == 'COMPLETE'
    profiles.list()
    assert not pointer.exists()
    assert profiles.store._read()[1]['hardware_setup.json']['wheel_odometry']['wheel_radius_m'] == original_radius
