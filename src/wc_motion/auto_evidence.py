"""Reviewed evidence for the automatic velocity channel (docs/architecture/velocity_channel.md, section 8).

The file records what was measured on this chair before automatic control may run at a stage, and the speed limits
the user approved for that stage. It is a human-reviewed record, not a manufacturer attestation: the hashes only
stop a different revision of the evidence being used by accident. ``load_evidence`` rejects anything incomplete;
the template in ``config/wheel_auto_drive_evidence.template.json`` must always be rejected.
"""
import hashlib
import json
import math
from pathlib import Path
import re
import struct

from .protocol import FeedbackError, checked_frame, read_request

EVIDENCE_SCHEMA = 'wc_auto_drive_evidence_v1'
STAGES = ('STAND_TEST', 'GROUND_TEST', 'SUPERVISED_CLOSED_LOOP')
# Section 5: no layer of the chain may exceed these, whatever a file says.
HARD_LIMITS = {'max_linear_m_s': 0.30, 'max_angular_rad_s': 0.50, 'max_wheel_rpm': 27.0,
               'max_linear_accel_m_s2': 0.30, 'max_linear_decel_m_s2': 0.60, 'max_angular_accel_rad_s2': 0.80}
# Wheels off the ground (level 2): the first speeds only.
STAGE_LIMITS = {'STAND_TEST': {'max_linear_m_s': 0.10, 'max_angular_rad_s': 0.30}}
STOP_TESTS = ('estop', 'dead_man', 'owner_killed', 'usb_unplugged', 'orin_power_cut', 'software_zero',
              'auto_stand_checks')
REQUIRED_STOP_TESTS = {
    # Level 2 items a-d are done with the existing manual channel before the automatic channel runs on the stand.
    'STAND_TEST': ('estop', 'dead_man', 'owner_killed', 'usb_unplugged', 'orin_power_cut'),
    # Level 2 items e-g with the automatic channel on the stand.
    'GROUND_TEST': STOP_TESTS,
    'SUPERVISED_CLOSED_LOOP': STOP_TESTS}
BRAKING_LIMITS_M = {'software_zero': 0.18, 'communication_loss': 0.30, 'estop': 0.30, 'dead_man': 0.30}
BRAKING_TRIALS, BRAKING_SPREAD, BRAKING_HEADING_DEG = 5, 0.20, 5.0
REQUIRED_READBACK = ('communication_timeout',)
_SHA256 = re.compile(r'[0-9a-f]{64}')


def _number(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) \
            or not low <= value <= high:
        raise FeedbackError('%s must be a finite number in [%s, %s]' % (name, low, high))
    return float(value)


def _integer(value, name, low, high):
    if type(value) is not int or not low <= value <= high:
        raise FeedbackError('%s must be an integer in [%s, %s]' % (name, low, high))
    return value


def _text(value, name):
    if not isinstance(value, str) or not value.strip() or len(value) > 200:
        raise FeedbackError(name + ' must be a short non-empty string')
    return value


def allowed_roots(project_root):
    """Evidence sources live in this checkout or in the configured data root."""
    roots = [Path(project_root).resolve()]
    try:
        from wc_runtime.storage_policy import StoragePolicy
        storage = StoragePolicy(project_root)
        if storage.enabled:
            roots.append(Path(storage.archive_root).resolve())
    except Exception:  # noqa: BLE001 - an unreadable storage policy only removes the data root option
        pass
    return roots


def check_source(item, project_root, roots):
    if not isinstance(item, dict) or set(item) - {'kind', 'path', 'sha256', 'note'} or \
            not isinstance(item.get('path'), str) or not _SHA256.fullmatch(str(item.get('sha256'))):
        raise FeedbackError('each source needs a path and a lowercase sha256')
    path = Path(item['path'])
    path = (Path(project_root) / path if not path.is_absolute() else path).resolve()
    if not any(path.is_relative_to(root) for root in roots) or not path.is_file() or path.stat().st_size > 200_000_000:
        raise FeedbackError('evidence source must be a bounded file in the checkout or the data root: ' + item['path'])
    if hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
        raise FeedbackError('evidence source content hash mismatch: ' + item['path'])
    return str(path)


def check_readback(item):
    """One read-only FC03 check of a driver parameter, written like manual_hardware's state checks."""
    if not isinstance(item, dict) or set(item) - {'purpose', 'request_hex', 'word_index', 'mask', 'equals', 'note'}:
        raise FeedbackError('readback check needs purpose, request_hex, word_index, mask and equals')
    purpose = _text(item.get('purpose'), 'readback purpose')
    try:
        request = checked_frame(bytes.fromhex(str(item.get('request_hex'))))
    except (ValueError, FeedbackError) as error:
        raise FeedbackError('readback request must be a valid RTU frame') from error
    if len(request) != 8 or request[:2] != b'\x01\x03':
        raise FeedbackError('readback permits only slave 1 FC03')
    _, _, address, count = struct.unpack('>BBHH', request[:-2])
    if not 1 <= count <= 8 or request != read_request(1, address, count):
        raise FeedbackError('readback range is invalid')
    index = _integer(item.get('word_index'), 'readback word_index', 0, count - 1)
    mask = _integer(item.get('mask'), 'readback mask', 1, 65535)
    equals = _integer(item.get('equals'), 'readback equals', 0, 65535)
    if equals & ~mask:
        raise FeedbackError('readback expected value lies outside its mask')
    return {'purpose': purpose, 'request': request, 'word_index': index, 'mask': mask, 'equals': equals}


def check_limits(value, stage):
    if not isinstance(value, dict) or set(value) != set(HARD_LIMITS):
        raise FeedbackError('approved_limits must give exactly ' + ', '.join(sorted(HARD_LIMITS)))
    caps = {**HARD_LIMITS, **STAGE_LIMITS.get(stage, {})}
    return {name: _number(value[name], 'approved_limits.' + name, 1e-6, caps[name]) for name in HARD_LIMITS}


def check_stop_test(name, value, project_root, roots):
    if not isinstance(value, dict) or value.get('verified') is not True:
        raise FeedbackError('stop test %s is not verified' % name)
    _integer(value.get('trials'), name + '.trials', 3, 1000)
    if value.get('max_stop_time_s') is not None:
        _number(value['max_stop_time_s'], name + '.max_stop_time_s', 0.0, 10.0)
    sources = value.get('sources')
    if not isinstance(sources, list) or not sources:
        raise FeedbackError('stop test %s needs its recordings or videos as sources' % name)
    return [check_source(item, project_root, roots) for item in sources]


def check_braking(rows, max_linear, project_root, roots):
    """Section 8 pass lines, at least at the approved top speed, for every stop path."""
    if not isinstance(rows, list):
        raise FeedbackError('braking must be a list of measured rows')
    covered = set()
    for row in rows:
        if not isinstance(row, dict) or row.get('path') not in BRAKING_LIMITS_M:
            raise FeedbackError('braking row needs a path in ' + ', '.join(BRAKING_LIMITS_M))
        speed = _number(row.get('speed_m_s'), 'braking.speed_m_s', 0.01, HARD_LIMITS['max_linear_m_s'])
        distances = row.get('distances_m')
        if not isinstance(distances, list) or len(distances) < BRAKING_TRIALS:
            raise FeedbackError('braking row needs at least %d distances' % BRAKING_TRIALS)
        values = [_number(d, 'braking distance', 0.0, 5.0) for d in distances]
        mean = sum(values) / len(values)
        if max(values) > BRAKING_LIMITS_M[row['path']]:
            raise FeedbackError('braking distance above the pass line for ' + row['path'])
        if mean > 0 and (max(values) - min(values)) / mean > BRAKING_SPREAD:
            raise FeedbackError('braking distances scatter more than 20% for ' + row['path'])
        if _number(row.get('max_heading_change_deg'), 'braking heading change', 0.0, 180.0) > BRAKING_HEADING_DEG:
            raise FeedbackError('heading changed more than 5 degrees while braking on ' + row['path'])
        for item in row.get('sources') or []:
            check_source(item, project_root, roots)
        if not row.get('sources'):
            raise FeedbackError('braking row needs its recordings as sources')
        if speed >= max_linear - 1e-9:
            covered.add(row['path'])
    missing = set(BRAKING_LIMITS_M) - covered
    if missing:
        raise FeedbackError('braking not measured at the approved top speed for ' + ', '.join(sorted(missing)))


def load_evidence(value, project_root, expected_device_id):
    """Validated content of an evidence document (already parsed JSON); raises FeedbackError otherwise."""
    from wc_runtime.device_bindings import identity_token
    if not isinstance(value, dict) or value.get('schema') != EVIDENCE_SCHEMA:
        raise FeedbackError('automatic drive evidence must use ' + EVIDENCE_SCHEMA)
    if value.get('state') != 'REVIEWED':
        raise FeedbackError('automatic drive evidence is not reviewed')
    stage = value.get('stage')
    if stage not in STAGES:
        raise FeedbackError('evidence stage must be one of ' + ', '.join(STAGES))
    try:
        device = identity_token(value.get('device_id'), prefix='ZLAC8030D-')
        expected = identity_token(expected_device_id, prefix='ZLAC8030D-')
    except ValueError as error:
        raise FeedbackError(str(error)) from error
    if device != expected:
        raise FeedbackError('evidence identifies another wheel controller')
    firmware = _text(value.get('firmware_id'), 'firmware_id')
    _text(value.get('reviewer'), 'reviewer')
    _text(value.get('reviewed_at'), 'reviewed_at')
    limits = check_limits(value.get('approved_limits'), stage)
    readback = value.get('driver_readback')
    if not isinstance(readback, list) or not readback:
        raise FeedbackError('driver parameter readback checks are required')
    checks = [check_readback(item) for item in readback]
    missing = set(REQUIRED_READBACK) - {item['purpose'] for item in checks}
    if missing:
        raise FeedbackError('driver readback must include ' + ', '.join(sorted(missing)))
    roots = allowed_roots(project_root)
    tests = value.get('stop_tests')
    if not isinstance(tests, dict) or set(tests) - set(STOP_TESTS):
        raise FeedbackError('stop_tests must use the names ' + ', '.join(STOP_TESTS))
    for name in REQUIRED_STOP_TESTS[stage]:
        check_stop_test(name, tests.get(name), project_root, roots)
    if stage == 'SUPERVISED_CLOSED_LOOP':
        check_braking(value.get('braking'), limits['max_linear_m_s'], project_root, roots)
    raw = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')
    return {'stage': stage, 'device_id': device, 'firmware_id': firmware, 'limits': limits, 'readback': checks,
            'evidence_sha256': hashlib.sha256(raw).hexdigest()}


def _reject_constant(value):
    raise FeedbackError('non-finite JSON constant ' + value)


def read_evidence_file(path, project_root, expected_device_id):
    path = Path(path)
    path = (Path(project_root) / path if not path.is_absolute() else path)
    if not path.is_file() or path.stat().st_size > 1_000_000:
        raise FeedbackError('automatic drive evidence file missing or oversized: ' + str(path))
    try:
        value = json.loads(path.read_text(encoding='utf-8'), parse_constant=_reject_constant)
    except ValueError as error:
        raise FeedbackError('automatic drive evidence is not valid JSON') from error
    return load_evidence(value, project_root, expected_device_id)
