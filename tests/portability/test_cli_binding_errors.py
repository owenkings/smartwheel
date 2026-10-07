"""Broken device bindings cannot disable help/status/stop; no hardware imports."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
pytestmark = pytest.mark.skipif(sys.platform != 'linux', reason='Real CLI uses Linux process supervision')


def checkout(directory, failure):
    (directory/'scripts').mkdir(parents=True)
    (directory/'config').mkdir()
    package = directory/'src/wc_runtime'; package.mkdir(parents=True)
    (package/'__init__.py').write_text('', encoding='utf-8')
    # The real CLI parser obtains PROFILES from capture. Copy its import-time
    # dependencies too, while excluding ROS, device modules and real configs.
    for name in ('cli', 'supervisor', 'component', 'project_paths', 'runtime_locks', 'device_bindings',
                 'capture', 'source_archive', 'mapping_control_paths', 'storage_policy'):
        shutil.copyfile(ROOT/'src/wc_runtime'/(name+'.py'), package/(name+'.py'))
    shutil.copyfile(ROOT/'scripts/wc_phase1', directory/'scripts/wc_phase1')
    (directory/'config/storage.json').write_text('{}', encoding='utf-8')
    # A valid base plus broken local file proves that fallback is forbidden.
    if failure != 'missing':
        shutil.copyfile(ROOT/'config/device_bindings.json', directory/'config/device_bindings.json')
        invalid = '{"imu":' if failure == 'malformed_local' else '{"schema_version":1,"imu":{},"network":{}}'
        (directory/'config/device_bindings.local.json').write_text(invalid, encoding='utf-8')
    return directory


def environment(directory):
    return dict(os.environ, WHEELCHAIR_PROJECT_ROOT=str(directory), PYTHONPATH=str(directory/'src'),
                PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1')


@pytest.mark.parametrize('failure', ['missing', 'malformed_local', 'incomplete_local'])
@pytest.mark.parametrize('command', ['--help', 'status', 'stop'])
def test_help_status_and_unknown_session_stop_keep_their_own_results(tmp_path, failure, command):
    directory = checkout(tmp_path/'different account'/'project with spaces', failure)
    before = {str(path.relative_to(directory)):path.read_bytes() for path in directory.rglob('*') if path.is_file()}
    arguments = [command] + (['--session','does-not-exist'] if command == 'stop' else [])
    result = subprocess.run([sys.executable,'-s',str(directory/'scripts/wc_phase1'),*arguments],
        cwd=tmp_path, env=environment(directory), capture_output=True, text=True, timeout=10)
    if command == '--help':
        assert result.returncode == 0 and 'usage:' in result.stdout
    elif command == 'status':
        assert result.returncode == 0 and json.loads(result.stdout) == []
    else:
        assert result.returncode == 2
        assert json.loads(result.stderr)['reason'] == 'No matching managed session'
    assert 'DEVICE_BINDINGS_INVALID' not in result.stderr and 'Traceback' not in result.stderr
    after = {str(path.relative_to(directory)):path.read_bytes() for path in directory.rglob('*') if path.is_file()}
    assert after == before  # No runtime, hardware, repair or fallback side effects.


@pytest.mark.parametrize('failure', ['missing', 'malformed_local', 'incomplete_local'])
def test_live_binding_entrypoints_reject_preserved_error_before_external_work(tmp_path, failure):
    directory = checkout(tmp_path/'project', failure)
    code = '''import json
from types import SimpleNamespace
from wc_runtime import cli
def forbidden(*args, **kwargs):
    raise AssertionError("external process boundary reached")
cli.subprocess.call = cli.subprocess.Popen = cli.subprocess.check_output = forbidden
cli.shutil.disk_usage = lambda path: SimpleNamespace(free=4*1024**3)
assert cli.DEVICE_BINDINGS is None and cli.DEVICE_BINDINGS_ERROR is not None
assert cli.H30_DEVICE is None and cli.H30_BY_ID is None and cli.H30_SERIAL is None
results = {}
for name in ("require_device_bindings", "device_preflight", "imu_preflight"):
    try:
        getattr(cli, name)()
    except RuntimeError as error:
        assert str(error).startswith("DEVICE_BINDINGS_INVALID: "), str(error)
        assert error.__cause__ is cli.DEVICE_BINDINGS_ERROR
        results[name] = "BLOCKED_BEFORE_DEVICE_ACCESS"
    else:
        raise AssertionError(name+" silently accepted invalid bindings")
print(json.dumps(results))
'''
    result = subprocess.run([sys.executable,'-s','-c',code],cwd=tmp_path,env=environment(directory),
                            capture_output=True,text=True,timeout=10)
    assert result.returncode == 0, result.stdout+result.stderr
    assert set(json.loads(result.stdout)) == {'require_device_bindings','device_preflight','imu_preflight'}
