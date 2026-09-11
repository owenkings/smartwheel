"""Generate the canonical delivery patch and verify it on the pinned base.

Does not modify the active FAST-LIO checkout or its index. Used after review of
the current source changes; generated patch includes previous uncommitted fixes.
"""
import hashlib
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
FL = ROOT / 'src/third_party/FAST_LIO_ROS2'
BASE = '2fffc570a25d0df172720bac034fbdb6a13d2162'
FILES = ['src/laserMapping.cpp', 'src/IMU_Processing.hpp',
         'include/wheel_velocity_update.hpp', 'tests/test_wheel_velocity_update.cpp',
         'include/state_aiding_safety.hpp', 'tests/test_state_aiding_safety.cpp']

def run(*args, cwd=FL):
    return subprocess.check_output(args, cwd=cwd)

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    sections = []
    for relative in FILES:
        exists = subprocess.run(['git', 'cat-file', '-e', f'{BASE}:{relative}'],
                                cwd=FL, capture_output=True).returncode == 0
        if exists:
            sections.append(run('git', 'diff', '--binary', '--full-index', BASE, '--', relative))
        else:
            result = subprocess.run(['git', 'diff', '--binary', '--full-index', '--no-index',
                                     '/dev/null', relative], cwd=FL, capture_output=True)
            if result.returncode != 1:
                raise RuntimeError(result.stderr.decode())
            sections.append(result.stdout)
    patch = b''.join(sections)
    with tempfile.TemporaryDirectory(prefix='smartwheel-patch-check-') as folder:
        checkout = Path(folder)
        archive = run('git', 'archive', BASE)
        subprocess.run(['tar', '-xf', '-', '-C', folder], input=archive, check=True)
        subprocess.run(['git', 'apply', '--check', '-'], cwd=folder, input=patch, check=True)
        subprocess.run(['git', 'apply', '-'], cwd=folder, input=patch, check=True)
        for relative in FILES:
            assert sha(checkout / relative) == sha(FL / relative), relative
    output = ROOT / 'patches/fastlio_smartwheel_hardening.patch'
    output.write_bytes(patch)
    print('PINNED_BASE_RECONSTRUCTION=PASS')
    print('PATCH_SHA256=' + sha(output))
    for relative in FILES:
        print(relative + '=' + sha(FL / relative))

if __name__ == '__main__':
    main()
