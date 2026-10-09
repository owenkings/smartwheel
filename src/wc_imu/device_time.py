"""Verified device-relative H30 timing; never alter arrival stamps or configure hardware."""
import copy
import hashlib
import json
from pathlib import Path
import re


TIMING_KEYS = ('sample_timestamp_valid', 'sample_timestamp_raw',
               'dataready_timestamp_valid', 'dataready_timestamp_raw',
               'device_timestamp_unit', 'device_time_status', 'timing_profile_sha256')
PROFILE_FLAG = 'TIMING_PROFILE_SHA256:'
VALID_FLAG = 'DEVICE_RELATIVE_TIME_VALIDATED'


def _ordinary(path):
    if any(p.is_symlink() or getattr(p, 'is_junction', lambda: False)() for p in (path, *path.parents)):
        raise ValueError('IMU_TIMING_LINKED_PATH')
    return path


def load_timing_profile(path, sensor_id=None):
    """Load an immutable policy and verify the referenced local evidence bytes."""
    path = _ordinary(Path(path).absolute())
    raw = path.read_bytes()
    if len(raw) > 65536:
        raise ValueError('IMU_TIMING_PROFILE_OVERSIZED')
    value = json.loads(raw.decode('utf-8'))
    if (not isinstance(value, dict) or type(value.get('schema_version')) is not int or value['schema_version'] != 1
            or value.get('status') != 'VERIFIED_DEVICE_RELATIVE_TIME'):
        raise ValueError('IMU_TIMING_VERIFIED_PROFILE_REQUIRED')
    for key in ('sensor_id', 'hardware_serial', 'product_version'):
        if not isinstance(value.get(key), str) or not value[key].strip():
            raise ValueError('IMU_TIMING_IDENTITY_REQUIRED: '+key)
    if value['sensor_id'] != 'H30-'+value['hardware_serial'] or sensor_id is not None and value['sensor_id'] != sensor_id:
        raise ValueError('IMU_TIMING_SENSOR_IDENTITY_MISMATCH')
    if value.get('device_timestamp_unit') not in ('us', '100us') or type(value.get('tick_ns')) is not int:
        raise ValueError('IMU_TIMING_EXPLICIT_UNIT_REQUIRED')
    if value['tick_ns'] != {'us':1000, '100us':100000}[value['device_timestamp_unit']]:
        raise ValueError('IMU_TIMING_UNIT_SCALE_MISMATCH')
    fields = value.get('required_fields')
    if (not isinstance(fields, list) or not fields or len(set(fields)) != len(fields)
            or not set(fields) <= {'sample_timestamp_raw', 'dataready_timestamp_raw'}
            or 'sample_timestamp_raw' not in fields):
        raise ValueError('IMU_TIMING_REQUIRED_FIELDS_INVALID')
    if value.get('backward_policy') not in ('reject', 'guarded_uint32_wrap') or value.get('duplicate_policy') != 'reject':
        raise ValueError('IMU_TIMING_EXPLICIT_COUNTER_POLICY_REQUIRED')
    if value.get('read_mode') not in ('event', 'poll'):
        raise ValueError('IMU_TIMING_READ_MODE_INVALID')
    quarantine = value.get('startup_quarantine_ns', 250000000)
    if type(quarantine) is not int or not 0 < quarantine <= 2000000000:
        raise ValueError('IMU_TIMING_STARTUP_QUARANTINE_OUTSIDE_0_TO_2_SECONDS')
    if 'max_gap_ticks' in value and (type(value['max_gap_ticks']) is not int or value['max_gap_ticks'] <= 0):
        raise ValueError('IMU_TIMING_MAX_GAP_INVALID')
    if value['backward_policy'] == 'guarded_uint32_wrap':
        if (type(value.get('max_gap_ticks')) is not int or not 0 < value['max_gap_ticks'] < 2**31
                or type(value.get('wrap_host_tolerance_ns')) is not int or value['wrap_host_tolerance_ns'] < 0):
            raise ValueError('IMU_TIMING_WRAP_GUARDS_REQUIRED')
    evidence = value.get('evidence')
    if not isinstance(evidence, list) or not evidence:
        raise ValueError('IMU_TIMING_EVIDENCE_REQUIRED')
    for row in evidence:
        if not isinstance(row, dict) or not isinstance(row.get('path'), str):
            raise ValueError('IMU_TIMING_EVIDENCE_PATH_INVALID')
        relative = Path(row['path'])
        if relative.is_absolute() or '..' in relative.parts or not relative.parts:
            raise ValueError('IMU_TIMING_EVIDENCE_PATH_INVALID')
        target = _ordinary(path.parent/relative)
        expected = row.get('sha256')
        if not isinstance(expected, str) or re.fullmatch('[0-9a-f]{64}', expected) is None:
            raise ValueError('IMU_TIMING_EVIDENCE_HASH_INVALID')
        if hashlib.sha256(target.read_bytes()).hexdigest() != expected:
            raise ValueError('IMU_TIMING_EVIDENCE_HASH_MISMATCH: '+str(relative))
    return dict(value, startup_quarantine_ns=quarantine,
                _profile_sha256=hashlib.sha256(raw).hexdigest(), _profile_path=str(path))


def advance_device_counter(previous, current, host_delta_ns, profile):
    """Return (positive delta ticks, guarded wrap) without mutating source counters."""
    if any(type(value) is not int or not 0 <= value < 2**32 for value in (previous,current)):
        raise ValueError('IMU_DEVICE_TIME_RANGE')
    delta = current-previous
    if delta == 0:
        raise ValueError('IMU_DEVICE_TIME_DUPLICATE')
    if delta > 0:
        if 'max_gap_ticks' in profile and delta > profile['max_gap_ticks']:
            raise ValueError('IMU_DEVICE_TIME_GAP_EXCEEDED')
        return delta, False
    if profile['backward_policy'] != 'guarded_uint32_wrap':
        raise ValueError('IMU_DEVICE_TIME_BACKWARD_RESET_OR_WRAP')
    bound = profile['max_gap_ticks']
    if type(host_delta_ns) is not int or host_delta_ns < 0:
        raise ValueError('IMU_DEVICE_TIME_WRAP_HOST_DELTA_REQUIRED')
    delta += 2**32
    if (previous < 2**32-bound or current > bound or not 0 < delta <= bound
            or abs(delta*profile['tick_ns']-host_delta_ns) > profile['wrap_host_tolerance_ns']):
        raise ValueError('IMU_DEVICE_TIME_UNPROVEN_WRAP_OR_RESET')
    return delta, True


class DeviceTimeValidator:
    """A latched per-stream continuity gate. Backward/reset/wrap require a new reviewed epoch."""
    def __init__(self, profile):
        self.profile = copy.deepcopy(profile)
        self.previous = {}
        self.error = None
        self.accepted = 0
        self.first = {}
        self.last = {}
        self.previous_host = None
        self.wraps = {}
        self.elapsed_ticks = {}

    def observe(self, frame, host_monotonic_ns=None):
        if self.error is not None:
            raise ValueError('IMU_DEVICE_TIME_LATCHED: '+self.error)
        values = {}
        advances = {}
        try:
            if self.profile['backward_policy'] == 'guarded_uint32_wrap':
                if (type(host_monotonic_ns) is not int or host_monotonic_ns < 0
                        or self.previous_host is not None and host_monotonic_ns < self.previous_host):
                    raise ValueError('IMU_DEVICE_TIME_HOST_MONOTONIC_REQUIRED')
            for field in ('sample_timestamp_raw', 'dataready_timestamp_raw'):
                value = frame.fields.get(field)
                required = field in self.profile['required_fields']
                if value is None:
                    if required:
                        raise ValueError('IMU_DEVICE_TIME_MISSING: '+field)
                    continue
                if type(value) is not int or not 0 <= value < 2**32:
                    raise ValueError('IMU_DEVICE_TIME_RANGE: '+field)
                if field in self.previous:
                    host_delta = None if self.previous_host is None or host_monotonic_ns is None else host_monotonic_ns-self.previous_host
                    advances[field] = advance_device_counter(self.previous[field],value,host_delta,self.profile)
                values[field] = value
        except (ValueError, KeyError, TypeError) as error:
            self.error = str(error)
            raise
        self.previous.update(values)
        self.previous_host = host_monotonic_ns
        for field in values:
            delta, wrapped = advances.get(field,(0,False))
            self.elapsed_ticks[field] = self.elapsed_ticks.get(field,0)+delta
            self.wraps[field] = self.wraps.get(field,0)+int(wrapped)
        if not self.accepted:
            self.first = dict(values)
        self.last = dict(values)
        self.accepted += 1
        result = dict(device_timestamp_unit=self.profile['device_timestamp_unit'],
                      device_time_status='VALID_DEVICE_RELATIVE_TIME',
                      timing_profile_sha256=self.profile['_profile_sha256'])
        for field in ('sample_timestamp_raw', 'dataready_timestamp_raw'):
            result[field] = values.get(field, 0)
            result[field.replace('_raw', '_valid')] = field in values
        return result

    def report(self):
        return dict(accepted_frames=self.accepted, error=self.error, first_raw=self.first, last_raw=self.last,
                    timing_profile_sha256=self.profile['_profile_sha256'],
                    device_timestamp_unit=self.profile['device_timestamp_unit'],
                    counter_wrap_policy=self.profile['backward_policy'],
                    wrap_counts=dict(self.wraps), elapsed_ticks=dict(self.elapsed_ticks),
                    mathematical_counter_period_s=2**32*self.profile['tick_ns']/1e9,
                    common_measurement_time_validated=False)


def message_timing_metadata(message):
    """Extract additive evidence without upgrading legacy messages or changing their stamps."""
    flags = list(getattr(message, 'diagnostic_flags', []))
    hashes = [flag[len(PROFILE_FLAG):] for flag in flags if flag.startswith(PROFILE_FLAG)]
    if len(hashes) > 1 or hashes and re.fullmatch('[0-9a-f]{64}', hashes[0]) is None:
        raise ValueError('IMU_TIMING_MESSAGE_PROFILE_INVALID')
    if bool(hashes) != (VALID_FLAG in flags):
        raise ValueError('IMU_TIMING_MESSAGE_STATUS_MISMATCH')
    result = dict(device_timestamp_unit=getattr(message, 'device_timestamp_unit', 'UNKNOWN_us_or_100us'),
                  device_time_status='VALID_DEVICE_RELATIVE_TIME' if hashes else 'LEGACY_ARRIVAL_ONLY',
                  timing_profile_sha256=hashes[0] if hashes else None)
    for prefix in ('sample_timestamp', 'dataready_timestamp'):
        valid = getattr(message, prefix+'_valid', False)
        result[prefix+'_valid'] = valid
        result[prefix+'_raw'] = getattr(message, prefix+'_raw', 0)
    return result
