"""Checkout-relative software paths, independent of account and host names.

Only explicit environment overrides or this module/entrypoint's ancestors are
used. The caller's working directory is never used to select another checkout.
"""
import hashlib
import os
from pathlib import Path
import platform
import stat


def project_root(start=None):
    override = os.environ.get('WHEELCHAIR_PROJECT_ROOT') if start is None else None
    if override:
        root = Path(override).expanduser()
        if not root.is_absolute():
            raise RuntimeError('WHEELCHAIR_PROJECT_ROOT must be absolute')
        candidates = [root]
    else:
        path = Path(__file__ if start is None else start).resolve()
        candidates = [path, *path.parents]
    for root in candidates:
        if (root/'scripts/wc_phase1').is_file() and (root/'src/wc_runtime').is_dir() and (root/'config').is_dir():
            return root.resolve()
    raise RuntimeError('Project root not found; keep src/, scripts/ and config/ together, or set WHEELCHAIR_PROJECT_ROOT')


def require_linux_runtime():
    if platform.system() != 'Linux' or platform.machine().lower() not in ('aarch64', 'arm64', 'x86_64', 'amd64'):
        raise RuntimeError('Runtime requires Linux aarch64 or x86_64; use Ubuntu 22.04 with ROS 2 Humble')


def ros_setup_path():
    value = Path(os.environ.get('WHEELCHAIR_ROS_SETUP', '/opt/ros/humble/setup.bash')).expanduser()
    if not value.is_absolute():
        raise RuntimeError('WHEELCHAIR_ROS_SETUP must be absolute')
    return value


def runtime_root(project=None):
    return (project_root() if project is None else Path(project).resolve())/'.phase1_runtime'


def _private_directory(path):
    path = Path(path)
    path.mkdir(mode=0o700, exist_ok=True)
    info = path.lstat()
    if (not stat.S_ISDIR(info.st_mode) or path.is_symlink() or info.st_uid != os.geteuid()
            or stat.S_IMODE(info.st_mode) != 0o700):
        raise RuntimeError('Runtime directory must be an ordinary 0700 directory owned by this UID: '+str(path))
    return path


def shared_lock_root():
    """Shared by every checkout for this UID; kernel device exclusivity remains."""
    return _private_directory(Path('/tmp')/('wc-locks-'+str(os.geteuid())))


def socket_root(project=None):
    root = project_root() if project is None else Path(project).resolve()
    key = hashlib.sha256(str(root).encode('utf-8')).hexdigest()[:12]
    return _private_directory(Path('/tmp')/('wc-sock-'+str(os.geteuid())+'-'+key))


def capture_staging_root():
    # Pure archive/configuration modules may be imported on a non-POSIX editor.
    # Acquisition still checks Linux and an actual tmpfs before using this path.
    uid = str(os.geteuid()) if hasattr(os, 'geteuid') else 'nonposix'
    return Path('/dev/shm')/('wc_capture_'+uid)


def compare_staging_root():
    uid = str(os.geteuid()) if hasattr(os, 'geteuid') else 'nonposix'
    return Path('/dev/shm')/('wc_compare_'+uid)
