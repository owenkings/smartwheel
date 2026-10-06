"""Rollback fault injection against synthetic files only; no hardware or deployment."""
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import tarfile
import uuid

import pytest


SCRIPT = Path(__file__).resolve().parents[2]/'scripts/v7_rollback.py'
spec = importlib.util.spec_from_file_location('v7_rollback_test_subject', SCRIPT)
rollback = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rollback)


@pytest.fixture
def tmp_path():
    parent = Path(__file__).absolute().parent
    directory = parent/('.v7_rollback_'+uuid.uuid4().hex)
    directory.mkdir()
    try:
        yield directory
    finally:
        assert directory.resolve().parent == parent and directory.name.startswith('.v7_rollback_')
        shutil.rmtree(directory)


def sha(value):
    return hashlib.sha256(value).hexdigest()


class FakeFlock:
    LOCK_EX, LOCK_NB = 2, 4

    def flock(self, descriptor, flags):
        assert descriptor >= 0 and flags == 6


@pytest.fixture
def project(tmp_path, monkeypatch):
    root = tmp_path/'project'
    root.mkdir()
    monkeypatch.setattr(rollback, 'ROOT', root)
    monkeypatch.setattr(rollback, 'fcntl', FakeFlock())
    files = {'src/existing.py':b'deployed existing\n', 'scripts/new_v7.py':b'deployed new\n'}
    for relative, payload in files.items():
        path = root/relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)
    unrelated = root/'data/experiments/keep/raw.bin'
    unrelated.parent.mkdir(parents=True)
    unrelated.write_bytes(b'acquisition evidence')
    (root/'src/user_file.py').write_bytes(b'unrelated user source')
    report = root/'reports/upgrade'
    (report/'baseline').mkdir(parents=True)
    state = {relative:{'sha256':sha(payload)} for relative, payload in files.items()}
    (report/'deployed_state.json').write_text(json.dumps(state), encoding='utf-8')
    original = b'original before upgrade\n'
    baseline = {'src/existing.py':{'sha256':sha(original), 'mode':0o100644}}
    (report/'baseline/source_manifest.json').write_text(json.dumps(baseline), encoding='utf-8')
    with tarfile.open(report/'baseline/source.tar.gz', 'w:gz') as archive:
        member = tarfile.TarInfo('src/existing.py')
        member.size = len(original)
        archive.addfile(member, io.BytesIO(original))
    return root, report, files, original


def snapshot(root):
    return {path.relative_to(root).as_posix():path.read_bytes() for path in root.rglob('*') if path.is_file()}


def run(report, apply=False):
    return rollback.main(['--report',str(report), *(['--apply'] if apply else [])])


def test_preview_is_readonly_and_dynamic_count(project, capsys):
    root, report, files, original = project
    before = snapshot(root)
    assert run(report) == 0
    result = json.loads(capsys.readouterr().out)
    assert result['files'] == len(files) and result['restore_existing'] == 1
    assert result['quarantine_new'] == 1 and not result['apply']
    assert snapshot(root) == before and not (root/'.phase1_runtime').exists()


def test_apply_restores_only_manifest_and_retains_new_file_and_evidence(project):
    root, report, files, original = project
    assert run(report, apply=True) == 0
    assert (root/'src/existing.py').read_bytes() == original
    assert not (root/'scripts/new_v7.py').exists()
    assert (root/'src/user_file.py').read_bytes() == b'unrelated user source'
    assert (root/'data/experiments/keep/raw.bin').read_bytes() == b'acquisition evidence'
    retained, = report.glob('rollback_retained_*')
    assert all((retained/relative).read_bytes() == payload for relative,payload in files.items())
    journal = json.loads((retained/'rollback_status.json').read_text())
    assert journal['status'] == 'SOURCE_RESTORED_REBUILD_REQUIRED'
    assert journal['completed'] == list(files)
    assert journal['retained_new_files'] == ['scripts/new_v7.py']
    assert set(rollback.FIXED_LOCKS) <= {path.name for path in (root/'.phase1_runtime/locks').iterdir()}


def test_independent_edit_refuses_before_any_source_move(project):
    root, report, files, original = project
    (root/'src/existing.py').write_bytes(b'user changed since deployment')
    before = snapshot(root)
    with pytest.raises(ValueError, match='changed since deployment'):
        run(report, apply=True)
    assert all((root/relative).read_bytes() == before[relative] for relative in files)
    assert not list(report.glob('rollback_retained_*'))


@pytest.mark.parametrize('relative', ['src/../data/raw.bin', 'src/../../outside', '/src/a',
    'src//a', 'src/./a', 'data/source.bin', 'src\\a', 'src/'])
def test_noncanonical_or_evidence_paths_refused(project, relative):
    root, report, files, original = project
    (report/'deployed_state.json').write_text(json.dumps({relative:{'sha256':'a'*64}}))
    with pytest.raises(ValueError, match='path'):
        run(report)
    assert not list(report.glob('rollback_retained_*'))


def test_duplicate_manifest_keys_refused(project):
    root, report, files, original = project
    (report/'deployed_state.json').write_text('{"src/a":{},"src/a":{}}')
    with pytest.raises(ValueError, match='duplicate manifest key'):
        run(report)


def test_bad_baseline_hash_refuses(project):
    root, report, files, original = project
    baseline = json.loads((report/'baseline/source_manifest.json').read_text())
    baseline['src/existing.py']['sha256'] = '0'*64
    (report/'baseline/source_manifest.json').write_text(json.dumps(baseline))
    with pytest.raises(ValueError, match='baseline hash mismatch'):
        run(report)
    assert all((root/relative).read_bytes() == payload for relative,payload in files.items())


def test_staging_write_failure_does_not_move_any_source(project, monkeypatch):
    root, report, files, original = project
    real_open = Path.open
    def fail_preparation(path, *args, **kwargs):
        if '.baseline_ready' in path.parts:
            raise OSError('synthetic disk full')
        return real_open(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', fail_preparation)
    with pytest.raises(OSError, match='synthetic disk full'):
        run(report, apply=True)
    assert all((root/relative).read_bytes() == payload for relative,payload in files.items())
    retained, = report.glob('rollback_retained_*')
    assert json.loads((retained/'rollback_status.json').read_text())['status'] == 'INTERRUPTED_DO_NOT_START_PROJECT'


def test_failed_atomic_replacement_puts_complete_deployed_file_back(project, monkeypatch):
    root, report, files, original = project
    real_replace = os.replace
    def fail_replacement(source, target):
        if '.baseline_ready' in Path(source).parts:
            raise OSError('synthetic replacement failure')
        return real_replace(source, target)
    monkeypatch.setattr(rollback.os, 'replace', fail_replacement)
    with pytest.raises(OSError, match='synthetic replacement failure'):
        run(report, apply=True)
    assert all((root/relative).read_bytes() == payload for relative,payload in files.items())
    retained, = report.glob('rollback_retained_*')
    assert json.loads((retained/'rollback_status.json').read_text())['status'] == 'INTERRUPTED_DO_NOT_START_PROJECT'


def test_existing_dynamic_lock_is_checked_and_busy_owner_refuses(project, monkeypatch):
    root, report, files, original = project
    lock_dir = root/'.phase1_runtime/locks'
    lock_dir.mkdir(parents=True)
    dynamic = lock_dir/'serial-188-0.lock'
    dynamic.write_bytes(b'')
    real_open = os.open
    last_path = []
    def remember_open(path, *args, **kwargs):
        last_path[:] = [Path(path)]
        return real_open(path, *args, **kwargs)
    class BusyFlock(FakeFlock):
        def flock(self, descriptor, flags):
            if last_path[0] == dynamic:
                raise BlockingIOError('synthetic held serial lock')
    monkeypatch.setattr(rollback.os, 'open', remember_open)
    monkeypatch.setattr(rollback, 'fcntl', BusyFlock())
    with pytest.raises(RuntimeError, match='serial-188-0.lock'):
        run(report, apply=True)
    assert not list(report.glob('rollback_retained_*'))
    assert all((root/relative).read_bytes() == payload for relative,payload in files.items())


@pytest.mark.skipif(os.name != 'posix', reason='target Linux symlink and real flock checks')
def test_report_symlink_is_rejected_before_resolve(project):
    root, report, files, original = project
    linked = report.parent/'linked_report'
    linked.symlink_to(report, target_is_directory=True)
    with pytest.raises(ValueError, match='linked path'):
        run(linked)


@pytest.mark.skipif(os.name != 'posix', reason='target Linux real flock check')
def test_real_standalone_camera_lock_blocks_apply(project, monkeypatch):
    import fcntl
    root, report, files, original = project
    monkeypatch.setattr(rollback, 'fcntl', fcntl)
    lock_dir = root/'.phase1_runtime/locks'
    lock_dir.mkdir(parents=True)
    with (lock_dir/'domain-83-cameras.lock').open('a+') as owner:
        fcntl.flock(owner.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(RuntimeError, match='domain-83-cameras.lock'):
            run(report, apply=True)
    assert not list(report.glob('rollback_retained_*'))
