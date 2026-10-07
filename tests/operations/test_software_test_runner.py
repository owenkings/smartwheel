"""Software-only runner acceptance; fake pytest never imports ROS or hardware."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import tarfile

import pytest


MODULE = Path(__file__).resolve().parents[2] / 'scripts/run_software_tests.py'
spec = importlib.util.spec_from_file_location('software_test_runner_under_test', MODULE)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


@pytest.fixture
def project(tmp_path):
    root = tmp_path / 'code'
    archive = tmp_path / 'usb'
    for folder in ('src/wc_runtime', 'tests', 'config'):
        (root / folder).mkdir(parents=True)
    (root / 'src/wc_runtime/__init__.py').write_text('')
    (root / 'src/wc_runtime/storage_policy.py').write_text(
        'import argparse\np=argparse.ArgumentParser()\n'
        'p.add_argument("--project-root",required=True)\np.add_argument("path")\n'
        'a=p.parse_args()\nassert a.project_root==' + repr(str(root)) + '\n'
        'assert a.path=="reports/test_runs"\nprint(' + repr(str(archive)) + ')\n')
    fake = root / 'fake_pytest.py'
    fake.write_text('''import json,os,pathlib,sys,socket
scratch=pathlib.Path(os.environ['TMPDIR'])
(scratch/'payload.bin').write_bytes(bytes(range(256))*100)
(scratch/'outside').symlink_to('/etc/hostname')
pathlib.Path(os.environ['MPLCONFIGDIR'],'font-cache.json').write_text('{}')
cache=[a.split('=',1)[1] for a in sys.argv if a.startswith('cache_dir=')][0]
pathlib.Path(cache,'seen.json').write_text('{}')
junit=[a.split('=',1)[1] for a in sys.argv if a.startswith('--junitxml=')][0]
pathlib.Path(junit).write_text('<testsuites/>')
print(json.dumps({'argv':sys.argv[1:],'scratch':str(scratch),'bytecode':os.environ['PYTHONDONTWRITEBYTECODE'],'domain':os.environ['ROS_DOMAIN_ID']}))
if 'socket' in sys.argv:
 s=socket.socket(socket.AF_UNIX);s.bind(str(scratch/'socket'));s.close()
if 'socket_roundtrip' in sys.argv:
 p=scratch/'pytest/test_capture_manual_drive_connection0/subdir/control.sock'
 p.parent.mkdir(parents=True)
 server=socket.socket(socket.AF_UNIX);server.bind(str(p));server.listen(1)
 client=socket.socket(socket.AF_UNIX);client.connect(str(p));peer,_=server.accept()
 client.sendall(b'fixture-ok');assert peer.recv(32)==b'fixture-ok'
 client.close();peer.close();server.close();p.unlink()
sys.exit(7 if 'failure' in sys.argv else 0)
''')
    return root, archive, fake


@pytest.mark.parametrize('selection, expected', [([], 0), (['failure', '-k', 'one'], 7), (['socket_roundtrip'], 0)])
def test_pass_and_failure_both_archive_before_owned_cleanup(project, selection, expected):
    root, archive, fake = project
    code, report = runner.run_tests(root, selection, pytest_prefix=[runner.sys.executable, str(fake)], check_target=False, storage_check=lambda: None)
    assert code == expected and report.parent == archive
    result = json.loads((report / 'run.json').read_text())
    assert result['status'] == ('PASS' if expected == 0 else 'FAIL')
    assert result['scratch_cleanup'] == 'REMOVED_AFTER_VERIFIED_ARCHIVE'
    assert not Path(result['scratch']).exists()
    assert result['storage_command_argv'][-3:] == ['--project-root', str(root), 'reports/test_runs']
    assert result['command_argv'][2:2 + len(selection or ['tests'])] == (selection or ['tests'])
    assert (report / 'junit.xml').is_file() and (report / 'pytest_cache/seen.json').is_file()
    assert (report / 'matplotlib/font-cache.json').is_file()
    with tarfile.open(report / 'scratch.tar.gz', 'r:gz') as saved:
        assert saved.extractfile('scratch/payload.bin').read() == bytes(range(256)) * 100
        assert saved.getmember('scratch/outside').issym()
        assert saved.getmember('scratch/outside').linkname == '/etc/hostname'
    assert not list(root.rglob('__pycache__'))
    assert not (root / '.pytest_cache').exists()
    assert '"bytecode": "1"' in (report / 'pytest.log').read_text()
    assert '"domain": "89"' in (report / 'pytest.log').read_text()
    assert result['storage_checks'] == ['INITIAL_BINDING', 'BEFORE_ARCHIVE', 'BEFORE_SCRATCH_REMOVAL', 'BEFORE_FINAL_REPORT']


def test_special_file_retains_owned_posix_scratch_and_does_not_claim_pass(project):
    root, archive, fake = project
    code, report = runner.run_tests(root, ['socket'], pytest_prefix=[runner.sys.executable, str(fake)], check_target=False, storage_check=lambda: None)
    result = json.loads((report / 'run.json').read_text())
    scratch = Path(result['scratch'])
    try:
        assert code == 74 and result['status'] == 'ARTIFACT_FAILURE'
        assert result['test_exit_code'] == 0 and result['scratch_cleanup'] == 'RETAINED'
        assert scratch.parent == Path('/tmp') and scratch.name.startswith('wct-')
        assert scratch.is_dir() and (scratch / 'payload.bin').is_file()
        assert 'unsupported scratch special file' in result['archive_or_cleanup_error']
    finally:
        # This test alone owns this scratch; production correctly retained it.
        if scratch.parent == Path('/tmp') and scratch.name.startswith('wct-'):
            shutil.rmtree(scratch)


def test_archive_mismatch_preserves_original(project, monkeypatch, tmp_path):
    scratch = tmp_path / 'scratch'
    scratch.mkdir()
    (scratch / 'payload').write_bytes(b'original')
    original = runner.tree_manifest
    calls = []
    def changed(path):
        calls.append(path)
        if len(calls) == 2:
            (scratch / 'payload').write_bytes(b'changed concurrently')
        return original(path)
    monkeypatch.setattr(runner, 'tree_manifest', changed)
    destination = tmp_path / 'result.tar.gz'
    with pytest.raises(ValueError, match='mismatch|changed'):
        runner.archive_scratch(scratch, destination)
    assert not destination.exists()
    assert (scratch / 'payload').read_bytes() == b'changed concurrently'


@pytest.mark.parametrize('fail_at', [2, 3])
def test_detach_before_archive_or_removal_retains_original_scratch(project, fail_at):
    root, archive, fake = project
    checks = []
    def guard():
        checks.append(len(checks) + 1)
        if len(checks) >= fail_at:
            raise RuntimeError('mounted device identity changed')
    code, report = runner.run_tests(root, [], pytest_prefix=[runner.sys.executable, str(fake)], check_target=False, storage_check=guard)
    initial = json.loads((report / 'run.json').read_text())
    scratch = Path(initial['scratch'])
    try:
        assert code == 74 and scratch.is_dir()
        retained = json.loads((scratch / 'runner_recovery.json').read_text())
        assert retained['status'] == 'ARTIFACT_FAILURE'
        assert retained['scratch_cleanup'] == 'RETAINED'
        assert (scratch / 'payload.bin').read_bytes() == bytes(range(256)) * 100
        assert 'identity changed' in retained['archive_or_cleanup_error']
        assert (report / 'scratch.tar.gz').exists() == (fail_at == 3)
        assert initial['status'] == 'RUNNING', 'must not write a completed report to the changed destination'
    finally:
        if scratch.parent == Path('/tmp') and scratch.name.startswith('wct-'):
            shutil.rmtree(scratch)
