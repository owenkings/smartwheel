"""Explicit machine bindings; no probing, device access or cached hot reload.

Call once at session creation and freeze the returned value. Offline estimators
use the identities in their input archives, never these current-machine values.
"""
import copy
import hashlib
import json
from pathlib import Path, PurePosixPath
import re


def identity_token(value, *, prefix=None):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}', value):
        raise ValueError('explicit bounded sensor identity required')
    if prefix is not None and (not value.startswith(prefix) or len(value) == len(prefix)):
        raise ValueError('sensor identity must start with '+prefix+' and identify one device')
    return value


def device_path(value, *, by_id=False, by_path=False):
    """Validate Linux device names without following symlinks or opening devices."""
    if not isinstance(value, str) or not value or len(value) > 512 or '\\' in value or any(ord(c) < 33 for c in value):
        raise ValueError('explicit absolute Linux device path required')
    path = PurePosixPath(value)
    if not path.is_absolute() or '..' in path.parts or str(path) != value or not path.is_relative_to('/dev'):
        raise ValueError('device path must be a normalized child of /dev')
    if path == PurePosixPath('/dev'):
        raise ValueError('a device path is required')
    if by_id and path.parent != PurePosixPath('/dev/serial/by-id'):
        raise ValueError('explicit /dev/serial/by-id identity required')
    if by_path and path.parent != PurePosixPath('/dev/v4l/by-path'):
        raise ValueError('explicit /dev/v4l/by-path camera identity required')
    if re.fullmatch(r'/dev/(?:tty(?:ACM|USB|S)\d+|video\d+)', value):
        raise ValueError('numbered device nodes are not stable identity bindings')
    return value


def topology(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.:+-]{0,255}', value):
        raise ValueError('explicit udev ID_PATH topology required')
    return value


def _unique(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError('duplicate device binding key: '+key)
        value[key] = item
    return value


def validate_device_bindings(value):
    if not isinstance(value, dict) or set(value) != {'schema_version', 'imu', 'network'} or type(value['schema_version']) is not int or value['schema_version'] != 1:
        raise ValueError('device bindings schema_version=1 with imu and network required')
    imu = value['imu']
    if not isinstance(imu, dict) or set(imu) != {'device', 'expected_by_id', 'hardware_serial', 'sensor_id'}:
        raise ValueError('complete IMU device, by-id, serial and sensor_id binding required')
    serial = identity_token(imu['hardware_serial'])
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}', serial):
        raise ValueError('invalid hardware serial')
    device_path(imu['device'])
    by_id = device_path(imu['expected_by_id'], by_id=True)
    if serial not in PurePosixPath(by_id).name or imu['sensor_id'] != 'H30-'+serial:
        raise ValueError('IMU serial, by-id and sensor_id must refer to the same device')
    network = value['network']
    if not isinstance(network, dict) or set(network) != {'interface'}:
        raise ValueError('network requires one explicit interface or null')
    interface = network['interface']
    if interface is not None and (not isinstance(interface, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.:-]{0,14}', interface)):
        raise ValueError('invalid Linux network interface name')
    return copy.deepcopy(value)


def load_device_bindings(project_root=None):
    if project_root is None:
        from .project_paths import project_root as locate_project
        project_root = locate_project()
    root = Path(project_root).absolute()
    base, local = root/'config/device_bindings.json', root/'config/device_bindings.local.json'
    # Full replacement, never a partial merge that could combine two devices.
    chosen = local if local.exists() or local.is_symlink() else base
    if any(p.is_symlink() or getattr(p, 'is_junction', lambda: False)() for p in (chosen, *chosen.parents)):
        raise ValueError('device binding file/parents must not be linked')
    if not chosen.is_file() or chosen.stat().st_size > 16384:
        raise ValueError('missing or oversized device binding configuration: '+str(chosen))
    raw = chosen.read_bytes()
    if len(raw) > 16384:
        raise ValueError('device binding configuration grew beyond its limit')
    result = validate_device_bindings(json.loads(raw.decode('utf-8'), object_pairs_hook=_unique))
    result['_provenance'] = {'selected_path': str(chosen),
                             'sha256': hashlib.sha256(raw).hexdigest(),
                             'original_text': raw.decode('utf-8')}
    return result
