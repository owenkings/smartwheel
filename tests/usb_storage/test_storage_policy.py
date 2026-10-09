"""Relocation contracts use ordinary real paths, with only block-device identity mocked."""
import json
from pathlib import Path

import pytest

from wc_runtime import capture_destination, storage_policy


@pytest.fixture
def configured(tmp_path, monkeypatch):
    project = tmp_path/'code'
    (project/'config').mkdir(parents=True)
    mount = tmp_path/'usb'
    archive = mount/'wheelchair'
    archive.mkdir(parents=True)
    state = {'available': True, 'checks': 0, 'created': []}
    class Guard:
        def __init__(self, output_root, required_uuid):
            assert required_uuid == 'ABCD-1234'
            self.output_root = Path(output_root)
            self.required_uuid = required_uuid
            self.mount_root = mount
            state['created'].append(self.output_root)
            self.metadata = self.check()
        def check(self):
            state['checks'] += 1
            if not state['available']:
                raise ValueError('CAPTURE_DESTINATION_UNAVAILABLE: test USB detached')
            return {'status': 'AVAILABLE', 'filesystem': 'exfat', 'mount_point': str(mount),
                    'output_root': str(self.output_root), 'required_uuid': 'ABCD-1234',
                    'fallback_allowed': False}
    monkeypatch.setattr(capture_destination, 'CaptureDestination', Guard)
    config = dict(schema_version=1, enabled=True, archive_root=str(archive), mount_point=str(mount),
                  required_uuid='ABCD-1234', original_project_root='/home/nvidia/wheelchair', fallback_allowed=False)
    (project/'config/storage.json').write_text(json.dumps(config), encoding='utf-8')
    return project, archive, state, config


@pytest.mark.parametrize('logical', ['data/experiments/session', 'reports/maps/run/export', 'maps/room'])
def test_relative_and_old_absolute_paths_resolve_identically(configured, logical):
    project, archive, _, _ = configured
    policy = storage_policy.StoragePolicy(project)
    expected = archive/logical
    assert policy.resolve(logical) == expected
    assert policy.resolve(project/logical) == expected
    assert policy.resolve(Path('/home/nvidia/wheelchair')/logical) == expected
    assert policy.resolve(expected) == expected
    assert policy.logical(expected) == logical
    assert storage_policy.logical_storage_path(project, expected) == logical
    assert not expected.exists()


def test_code_config_and_lock_paths_stay_on_project_filesystem(configured):
    project, _, _, _ = configured
    policy = storage_policy.StoragePolicy(project)
    for logical in ('src/wc_runtime/storage_policy.py', 'config/hardware_setup.json',
                    '.phase1_runtime/locks/sensor_owner.lock', 'install/main'):
        assert policy.resolve(logical) == project/logical


def test_allowed_data_roots_are_exactly_three_usb_children(configured):
    project, archive, _, _ = configured
    assert storage_policy.allowed_data_roots(project) == tuple(archive/name for name in ('data','reports','maps'))


def test_detach_blocks_even_after_success_and_does_not_make_local_fallback(configured):
    project, _, state, _ = configured
    policy = storage_policy.StoragePolicy(project)
    policy.resolve('data/experiments/a')
    state['available'] = False
    with pytest.raises(ValueError, match='detached'):
        policy.resolve('data/experiments/b')
    assert not (project/'data').exists()


def test_unmounted_usb_does_not_block_local_stop_state_or_code_reads(configured):
    project, _, state, _ = configured
    state['available'] = False
    policy = storage_policy.StoragePolicy(project)
    assert state['created'] == []
    for logical in ('.phase1_runtime/sessions/run/mapping_app/manifest.json',
                    '.phase1_runtime/locks/sensor_owner.lock', 'src/wc_runtime/component.py'):
        assert policy.resolve(logical) == project/logical
    assert state['checks'] == 0
    with pytest.raises(ValueError, match='detached'):
        policy.resolve('data/experiments/new')


@pytest.mark.parametrize('fault', ['missing', 'broken_json'])
def test_missing_or_broken_policy_does_not_block_local_stop(configured, monkeypatch, fault):
    project, _, _, _ = configured
    configuration = project/'config/storage.json'
    if fault == 'missing': configuration.unlink()
    else: configuration.write_text('{broken', encoding='utf-8')
    monkeypatch.setattr(storage_policy, 'ORIN_PROJECT', project)
    path = '.phase1_runtime/sessions/run/mapping_app/manifest.json'
    assert storage_policy.resolve_storage_path(project, path) == project/path
    with pytest.raises(ValueError):
        storage_policy.resolve_storage_path(project, 'data/experiments/new')


@pytest.mark.parametrize('fault', ['unmounted', 'missing', 'broken_json'])
def test_mapping_stop_reads_local_manifest_despite_storage_failure(configured, monkeypatch, fault):
    from wc_runtime import mapping_controller
    project, archive, state, _ = configured
    state['available'] = False
    if fault == 'missing': (project/'config/storage.json').unlink()
    if fault == 'broken_json': (project/'config/storage.json').write_text('{broken', encoding='utf-8')
    monkeypatch.setattr(storage_policy, 'ORIN_PROJECT', project)
    monkeypatch.setattr(mapping_controller, 'ROOT', project)
    runtime = project/'.phase1_runtime/sessions/owned/mapping_app'
    runtime.mkdir(parents=True)
    manifest = runtime/'manifest.json'
    manifest.write_text(json.dumps({'state':'RUNNING','exit_code':0}), encoding='utf-8')
    stopped = []
    def stop(path):
        stopped.append(path)
        path.write_text(json.dumps({'state':'STOPPED','exit_code':0}), encoding='utf-8')
    monkeypatch.setattr(mapping_controller, 'stop_registered', stop)
    monkeypatch.setattr(mapping_controller, 'shutdown_policy', lambda _: {'cli_stop_wait_s':.1})
    result=mapping_controller.stop({'runtime':str(runtime),'directory':str(archive/'reports/maps/owned')})
    assert result['state']=='STOPPED' and stopped==[manifest]
    assert state['checks']==0


def test_policy_instance_freezes_destination_when_config_changes(configured):
    project, archive, _, config = configured
    policy = storage_policy.StoragePolicy(project)
    config['archive_root'] = str(archive.parent/'other')
    (project/'config/storage.json').write_text(json.dumps(config), encoding='utf-8')
    assert policy.resolve('data/a') == archive/'data/a'


@pytest.mark.parametrize('value', ['data/../config/key', '../outside', '/tmp/unapproved',
                                   '/home/nvidia/wheelchair-other/data/a'])
def test_traversal_and_unapproved_external_paths_rejected(configured, value):
    project, _, _, _ = configured
    with pytest.raises(ValueError):
        storage_policy.resolve_storage_path(project, value)


@pytest.mark.skipif(__import__('os').name != 'posix', reason='ordinary POSIX symlink creation')
def test_usb_descendant_link_cannot_redirect_output(configured, tmp_path):
    project, archive, _, _ = configured
    (archive/'data').symlink_to(tmp_path/'other', target_is_directory=True)
    with pytest.raises(ValueError, match='symlinks'):
        storage_policy.resolve_storage_path(project, 'data/experiments/a')


def test_no_config_preserves_legacy_storage_without_probing_mount(tmp_path, monkeypatch):
    monkeypatch.setattr(capture_destination, 'CaptureDestination', lambda *_: pytest.fail('no external policy'))
    policy = storage_policy.StoragePolicy(tmp_path)
    assert not policy.enabled
    assert policy.resolve('data/experiments/a') == tmp_path/'data/experiments/a'
    assert capture_destination.capture_destination(tmp_path) == (tmp_path/'data/experiments', None)


def test_verified_production_root_requires_storage_configuration(tmp_path, monkeypatch):
    monkeypatch.setattr(storage_policy, 'ORIN_PROJECT', tmp_path)
    with pytest.raises(ValueError, match='STORAGE_CONFIG_MISSING'):
        storage_policy.StoragePolicy(tmp_path)


def test_default_capture_and_explicit_override_have_distinct_roots(configured):
    project, archive, _, _ = configured
    output, guard = capture_destination.capture_destination(project)
    assert output == archive/'data/experiments'
    assert guard.output_root == output
    explicit = archive/'explicit_capture'
    assert capture_destination.capture_destination(project, explicit, 'ABCD-1234')[0] == explicit


def test_doctor_scope_accepts_logical_and_usb_paths_and_keeps_subscope(configured, monkeypatch):
    from wc_runtime import cli
    project, archive, _, _ = configured
    monkeypatch.setattr(cli, 'ROOT', project)
    assert cli.scoped_path('data/experiments/run') == archive/'data/experiments/run'
    assert cli.scoped_path(archive/'data/experiments/run') == archive/'data/experiments/run'
    with pytest.raises(RuntimeError, match='outside'):
        cli.scoped_path('reports/a', project/'data/bags')


def test_compare_resolves_relocated_source_and_new_usb_output_before_loading(configured, monkeypatch):
    from wc_runtime import mapping_compare
    project, archive, _, _ = configured
    source = archive/'data/experiments/recorded'
    source.mkdir(parents=True)
    seen = []
    def load(path, **options):
        seen.append((path, options['project_root']))
        raise RuntimeError('STOP_BEFORE_REPLAY')
    monkeypatch.setattr(mapping_compare, 'load_dataset', load)
    with pytest.raises(RuntimeError, match='STOP_BEFORE_REPLAY'):
        mapping_compare.main(['--project-root',str(project),'--dataset','data/experiments/recorded',
                              '--output','reports/replays/new','--estimators','five_state'])
    assert seen == [(source, project)]
    assert not (project/'data').exists() and not (archive/'reports/replays/new').exists()


def test_default_capture_refuses_missing_usb_before_any_preflight(configured, monkeypatch):
    from types import SimpleNamespace
    from wc_runtime import capture, cli
    project, _, state, _ = configured
    state['available'] = False
    monkeypatch.setattr(cli, 'ROOT', project)
    monkeypatch.setattr(cli, 'target', lambda: None)
    monkeypatch.setattr(capture, 'preflight', lambda *_args, **_kwargs: pytest.fail('USB must be checked first'))
    args = SimpleNamespace(profile='mapping_core',session='no_usb',duration=1,staging='memory',
                           dry_run=False,diagnostic=False)
    with pytest.raises(ValueError, match='detached'):
        capture.run_capture(args)
    assert not (project/'data').exists()


def test_history_restore_uses_moved_receipt_and_archive_without_local_output(configured):
    import hashlib
    import lzma
    from wc_runtime import history_archive
    project, archive, _, _ = configured
    payload = b'immutable recorded sensor bytes\x00\xff' * 40
    relative = 'reports/maps/closed/input/frame.cdr'
    compressed = archive/('data/history_archive/'+relative+'.xz')
    compressed.parent.mkdir(parents=True)
    compressed.write_bytes(lzma.compress(payload))
    receipt = archive/'reports/storage/receipt.json'
    receipt.parent.mkdir(parents=True)
    receipt.write_text(json.dumps(dict(path=relative, archive_path=compressed.relative_to(archive).as_posix(),
        archive_sha256=hashlib.sha256(compressed.read_bytes()).hexdigest(),
        original_sha256=hashlib.sha256(payload).hexdigest(), original_bytes=len(payload),
        original_mode=0o600, original_mtime_ns=1700000000000000000)), encoding='utf-8')
    assert history_archive.main(['--root',str(project),'--restore-receipt','reports/storage/receipt.json']) == 0
    assert (archive/relative).read_bytes() == payload
    assert not (project/'reports').exists()


@pytest.mark.skipif(__import__('os').name != 'posix', reason='POSIX code rollback locks and atomic moves')
def test_rollback_reads_usb_backup_but_retains_displaced_code_locally(configured, monkeypatch):
    import hashlib
    import importlib.util
    import io
    import tarfile
    project, archive, _, _ = configured
    script = Path(__file__).resolve().parents[2]/'scripts/restore_deployment.py'
    spec = importlib.util.spec_from_file_location('usb_rollback_subject', script)
    rollback = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(rollback)
    monkeypatch.setattr(rollback, 'ROOT', project)
    current, original = b'current source', b'original source'
    code = project/'src/example.py'
    code.parent.mkdir(); code.write_bytes(current)
    report = archive/'reports/upgrade'
    (report/'baseline').mkdir(parents=True)
    (report/'deployed_state.json').write_text(json.dumps({'src/example.py':
        {'sha256':hashlib.sha256(current).hexdigest()}}), encoding='utf-8')
    (report/'baseline/source_manifest.json').write_text(json.dumps({'src/example.py':
        {'sha256':hashlib.sha256(original).hexdigest(),'mode':0o100644}}), encoding='utf-8')
    with tarfile.open(report/'baseline/source.tar.gz','w:gz') as bundle:
        member=tarfile.TarInfo('src/example.py'); member.size=len(original)
        bundle.addfile(member,io.BytesIO(original))
    assert rollback.main(['--report','reports/upgrade','--apply']) == 0
    assert code.read_bytes() == original
    retained=list((project/'.phase1_runtime/rollback_retained').glob('*/src/example.py'))
    assert len(retained) == 1 and retained[0].read_bytes() == current
    assert not (project/'reports').exists()
