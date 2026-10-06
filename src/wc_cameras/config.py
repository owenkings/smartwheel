"""Strict, hardware-free camera configuration validation."""
import hashlib
import json
import math
from pathlib import Path
import re

ROLES = ('left_front', 'right_front', 'left_side', 'right_side')


class CameraError(RuntimeError):
    pass


def number(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise CameraError(f'{name} must be finite in [{low}, {high}]')
    return value


def exact_keys(value, keys, name):
    if not isinstance(value, dict) or set(value) != set(keys):
        raise CameraError(f'{name}: expected exactly {sorted(keys)}')


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise CameraError('duplicate JSON key: '+key)
        result[key] = value
    return result


def validate_config(value):
    exact_keys(value, ('schema_version', 'mapping_status', 'mapping_note', 'rotation_note', 'rotation_status',
                       'default_profile', 'profiles', 'timeouts', 'cameras'), 'config')
    if (type(value['schema_version']) is not int or value['schema_version'] != 1
            or value['mapping_status'] not in ('UNCONFIRMED', 'USER_CONFIRMED') or value['rotation_status'] != 'CONFIG_ONLY'):
        raise CameraError('schema_version=1, mapping_status=UNCONFIRMED or USER_CONFIRMED, rotation_status=CONFIG_ONLY required')
    if not all(isinstance(value[k], str) and value[k].strip() for k in ('mapping_note', 'rotation_note')):
        raise CameraError('mapping and rotation evidence notes are required')
    profiles = value['profiles']
    if not isinstance(profiles, dict) or not 1 <= len(profiles) <= 8:
        raise CameraError('one to eight explicit profiles required')
    for name, profile in profiles.items():
        if not isinstance(name, str) or not re.fullmatch(r'[a-z][a-z0-9_]{0,39}', name):
            raise CameraError('invalid profile name')
        exact_keys(profile, ('width', 'height', 'fourcc', 'capture_fps', 'publish_hz'), 'profile')
        for key, low, high in (('width', 160, 1920), ('height', 120, 1080)):
            if type(profile[key]) is not int or not low <= profile[key] <= high or profile[key] % 2:
                raise CameraError(f'{key} must be an even integer in [{low}, {high}]')
        if profile['fourcc'] != 'MJPG':
            raise CameraError('only the reviewed MJPG capture format is supported; no automatic fallback')
        number(profile['capture_fps'], 'capture_fps', 1, 60)
        number(profile['publish_hz'], 'publish_hz', 1, 15)
        if profile['publish_hz'] > profile['capture_fps']:
            raise CameraError('publish_hz may not exceed capture_fps')
    if not isinstance(value['default_profile'], str) or value['default_profile'] not in profiles:
        raise CameraError('default_profile not found')
    timeouts = value['timeouts']
    exact_keys(timeouts, ('open_s', 'read_s', 'stale_s'), 'timeouts')
    number(timeouts['open_s'], 'open_s', 1, 30)
    number(timeouts['read_s'], 'read_s', .25, 10)
    number(timeouts['stale_s'], 'stale_s', .1, 10)
    if timeouts['stale_s'] >= timeouts['read_s']:
        raise CameraError('stale_s must precede read_s failure deadline')
    cameras = value['cameras']
    if not isinstance(cameras, list) or len(cameras) != 4:
        raise CameraError('exactly four cameras required')
    for camera in cameras:
        exact_keys(camera, ('role', 'port', 'rotate_deg', 'vid', 'pid', 'serial'), 'camera')
        if camera['role'] not in ROLES or type(camera['port']) is not int or camera['port'] not in range(1, 5):
            raise CameraError('invalid camera role or reviewed USB port')
        if type(camera['rotate_deg']) is not int or camera['rotate_deg'] not in (0, 180):
            raise CameraError('rotate_deg must be 0 or 180')
        if (camera['vid'], camera['pid'], camera['serial']) != ('0bda', '5858', '000000000011'):
            raise CameraError('camera identity differs from reviewed hardware')
    if {c['role'] for c in cameras} != set(ROLES) or {c['port'] for c in cameras} != set(range(1, 5)):
        raise CameraError('each role and physical USB port must occur exactly once')
    return value


def load_config(path):
    data = Path(path).read_bytes()
    if len(data) > 65536:
        raise CameraError('camera configuration exceeds 64 KiB')
    try:
        config = validate_config(json.loads(data, object_pairs_hook=unique_object))
    except (ValueError, TypeError) as error:
        raise CameraError('invalid camera configuration: '+str(error)) from error
    return config, hashlib.sha256(data).hexdigest()


def port_path(port):
    if type(port) is not int or port not in range(1, 5):
        raise CameraError('invalid USB camera port')
    return Path('/dev/v4l/by-path') / f'platform-3610000.usb-usb-0:3.{port}:1.0-video-index0'


def topic(role):
    if role not in ROLES:
        raise CameraError('invalid role')
    return '/wc_mapping/cameras/'+role+'/image_raw'
