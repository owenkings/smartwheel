"""Cross-checkout lock exclusion with temporary files; never opens devices."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import uuid

import pytest

from wc_runtime.runtime_locks import acquire_resource_lock
from wc_runtime.project_paths import shared_lock_root

pytestmark = pytest.mark.skipif(sys.platform != 'linux', reason='POSIX flock / C++ OS boundary')
ROOT = Path(__file__).resolve().parents[2]


def test_independent_checkout_processes_contend_for_same_named_resource(tmp_path):
    key = 'portability-'+uuid.uuid4().hex+'.lock'
    code = ('from wc_runtime.runtime_locks import acquire_resource_lock; '
            'import sys; lock=acquire_resource_lock(sys.argv[1]); print("acquired")')
    env = dict(os.environ, PYTHONPATH=str(ROOT/'src'), PYTHONDONTWRITEBYTECODE='1')
    try:
        with acquire_resource_lock(key):
            for clone in ('clone A', 'clone B'):
                cwd = tmp_path/clone; cwd.mkdir()
                env['WHEELCHAIR_PROJECT_ROOT'] = str(cwd)
                result = subprocess.run([sys.executable,'-c',code,key],cwd=cwd,env=env,
                                        capture_output=True,text=True,timeout=10)
                assert result.returncode != 0 and 'BlockingIOError' in result.stderr
        result = subprocess.run([sys.executable,'-c',code,key],env=env,capture_output=True,text=True,timeout=10)
        assert result.returncode == 0 and 'acquired' in result.stdout
    finally:
        (shared_lock_root()/key).unlink(missing_ok=True)


def test_python_lock_refuses_symlink_and_writable_file(tmp_path):
    key = 'portability-'+uuid.uuid4().hex+'.lock'
    path = shared_lock_root()/key; target = tmp_path/'untouched'; target.write_text('original')
    try:
        path.symlink_to(target)
        with pytest.raises(OSError): acquire_resource_lock(key)
        assert target.read_text() == 'original'
        path.unlink(); path.write_text('owned'); path.chmod(0o666)
        with pytest.raises(RuntimeError): acquire_resource_lock(key)
    finally:
        path.unlink(missing_ok=True)


def test_native_lidar_lock_uses_same_host_namespace_as_python(tmp_path):
    compiler = shutil.which('g++')
    if compiler is None: pytest.skip('C++ compiler unavailable')
    source = tmp_path/'host_lock.cpp'; binary = tmp_path/'host_lock'
    source.write_text('''#include "wc_xt_driver/host_lock.hpp"
#include <iostream>
int main(int argc, char **argv) {
  if(argc != 2) return 2;
  try { wc_xt_driver::HostResourceLock lock(argv[1]); return 0; }
  catch(const std::exception& error) { std::cerr << error.what(); return 3; }
}
''')
    subprocess.run([compiler,'-std=c++17','-Wall','-Wextra','-Werror',
                    '-I',str(ROOT/'src/wc_xt_driver/include'),str(source),'-o',str(binary)],
                   check=True,capture_output=True,text=True,timeout=30)
    key = 'xt-portability-'+uuid.uuid4().hex+'.lock'; path = shared_lock_root()/key
    try:
        with acquire_resource_lock(key):
            result = subprocess.run([str(binary),key],capture_output=True,text=True,timeout=10)
            assert result.returncode == 3 and 'busy' in result.stderr
        assert subprocess.run([str(binary),key],capture_output=True,timeout=10).returncode == 0
        path.unlink(); target = tmp_path/'untouched'; target.write_text('original'); path.symlink_to(target)
        assert subprocess.run([str(binary),key],capture_output=True,timeout=10).returncode == 3
        assert target.read_text() == 'original'
    finally:
        path.unlink(missing_ok=True)
