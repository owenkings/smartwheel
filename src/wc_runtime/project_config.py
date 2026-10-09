"""Select complete machine-local configuration before creating a live session.

Frozen recordings and explicit external inputs must keep their own readers.
A present local file is authoritative, including when it is invalid.
"""
import json
from pathlib import Path
import stat


PROJECT_CONFIG_NAMES = frozenset((
    'hardware_setup.json', 'mapping_live.json', 'cameras.json',
    'wheel_feedback_current.json', 'live_unvalidated.json',
    'mapping_3d_diagnostic.json', 'comparison_20260917_handpush/mapping.json',
    'wheel_manual_stop_evidence.template.json',
))
LIDAR_CONFIG_FILES = (
    'left.yaml', 'right.yaml', 'left-2026-09-11.xtcfg',
    'right-2026-09-11.xtcfg', 'left.xtcfg', 'right.xtcfg',
)


def _ordinary(path):
    path = Path(path).absolute()
    if '..' in path.parts:
        raise ValueError('PROJECT_CONFIG_PATH_INVALID: parent traversal')
    if any(item.is_symlink() or getattr(item, 'is_junction', lambda: False)()
           for item in (path, *path.parents)):
        raise ValueError('PROJECT_CONFIG_LINKED_PATH: ' + str(path))
    if any(item.exists() and not item.is_dir() for item in path.parents):
        raise ValueError('PROJECT_CONFIG_PARENT_NOT_DIRECTORY: ' + str(path))
    return path


def _file(path):
    path = _ordinary(path)
    if not stat.S_ISREG(path.stat().st_mode):
        raise ValueError('PROJECT_CONFIG_FILE_REQUIRED: ' + str(path))
    return path


def _present(path):
    try:
        path.lstat()
    except FileNotFoundError:
        return False
    return True


def selected_config_path(root, relative_name):
    """Return a verified current-project path without merging or rewriting bytes."""
    relative = Path(relative_name)
    if relative.is_absolute() or '..' in relative.parts or relative.as_posix() not in PROJECT_CONFIG_NAMES:
        raise ValueError('PROJECT_CONFIG_NAME_UNSUPPORTED: ' + str(relative_name))
    root = _ordinary(root)
    local = _ordinary(root / 'config/local/project' / relative)
    chosen = local if _present(local) else root / 'config' / relative
    chosen = _file(chosen)
    # Leave each consumer's existing schema and identity validation authoritative.
    json.loads(chosen.read_bytes().decode('utf-8'))
    return chosen


def lidar_config_directory(root):
    """Select one complete driver configuration directory; never mix its files."""
    root = _ordinary(root)
    local = _ordinary(root / 'config/local/lidar')
    chosen = local if _present(local) else root / 'src/wc_xt_driver/config'
    chosen = _ordinary(chosen)
    if not chosen.is_dir():
        raise ValueError('LIDAR_CONFIG_DIRECTORY_REQUIRED: ' + str(chosen))
    for name in LIDAR_CONFIG_FILES:
        _file(chosen / name)
    for item in chosen.rglob('*'):
        item = _ordinary(item)
        if not item.is_dir():
            _file(item)
    return chosen
