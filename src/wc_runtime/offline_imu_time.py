"""Frozen, offline-only device-relative IMU timing without changing source stamps.

The minimum arrival-minus-device offset anchors relative samples to the host
epoch. It removes read batching from sample intervals, but measures neither
physical sensor latency nor synchronization with wheel/lidar clocks.
"""
import copy
import hashlib
import json
from pathlib import Path


MODES = ('auto', 'arrival', 'device_relative')
FIELDS = ('sample_timestamp_raw', 'dataready_timestamp_raw')


def _require(condition, reason):
    if not condition:
        raise ValueError(reason)


def _sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False,
                                   separators=(',', ':')).encode()).hexdigest()


def timing_mode(config):
    mode = config.get('offline_imu_time_mode', 'arrival')
    _require(mode in MODES, 'offline IMU time mode must be auto, arrival or device_relative')
    return mode


def resolved_mode(source, config):
    mode = timing_mode(config)
    if mode == 'auto':
        # An existing but invalid declaration is an error, never an arrival fallback.
        profile_path = Path(source)/'configuration'/'imu_timing.json'
        return 'device_relative' if profile_path.exists() or profile_path.is_symlink() else 'arrival'
    return mode


def message_metadata(message):
    from wc_imu.device_time import message_timing_metadata
    result = message_timing_metadata(message)
    result.update(sensor_id=message.sensor_id, session_id=message.session_id,
                  stream_epoch=message.stream_epoch)
    return result


def build_model(events, source, config):
    """Attach derived stamps to event *copies*; never rewrite message metadata.

    Both raw counters are checked independently by the capture-side guard.
    A wrap needs the reviewed profile and host-elapsed evidence; there is no
    unconditional unwrap or sequence-ID interpolation.
    """
    from wc_imu.device_time import load_timing_profile, advance_device_counter
    _require(resolved_mode(source, config) == 'device_relative', 'device-relative mode required')
    sensor = config['prior_template']['imu_sensor_id']
    profile = load_timing_profile(Path(source)/'configuration'/'imu_timing.json', sensor_id=sensor)
    _require(set(profile['required_fields']) == set(FIELDS), 'offline device timing requires sample and dataready')
    _require(profile['backward_policy'] in ('reject', 'guarded_uint32_wrap')
             and profile['duplicate_policy'] == 'reject', 'unsupported device counter policy')
    rows = sorted((row for row in events if row['category'] == 'imu'),
                  key=lambda row: (row['monotonic_ns'], row['sequence'], row.get('bag', ''), row.get('message_id', 0)))
    _require(len(rows) >= 2, 'device timing requires at least two actual IMU samples')
    first, previous = {}, None
    accumulated = dict.fromkeys(FIELDS, 0)
    wraps = dict.fromkeys(FIELDS, 0)
    relative = []
    for row in rows:
        meta = row.get('imu_device_time', {})
        _require(meta.get('sensor_id') == sensor and meta.get('session_id') == config['session_id']
                 and isinstance(meta.get('stream_epoch'), str) and bool(meta['stream_epoch']),
                 'device timing identity/session/epoch mismatch')
        _require(meta.get('device_time_status') == 'VALID_DEVICE_RELATIVE_TIME'
                 and meta.get('timing_profile_sha256') == profile['_profile_sha256']
                 and meta.get('device_timestamp_unit') == profile['device_timestamp_unit'],
                 'device timing message must bind the frozen verified profile')
        if previous is not None:
            _require(row['monotonic_ns'] >= previous['monotonic_ns']
                     and row['sequence'] > previous['sequence']
                     and meta['stream_epoch'] == previous['imu_device_time']['stream_epoch'],
                     'device timing host/sequence/epoch discontinuity')
        for field in FIELDS:
            raw = meta.get(field)
            _require(meta.get(field.replace('_raw', '_valid')) is True
                     and type(raw) is int and 0 <= raw < 2**32, 'valid uint32 device timestamp required: '+field)
            if previous is not None:
                host_elapsed = row['monotonic_ns']-previous['monotonic_ns']
                delta, wrapped = advance_device_counter(previous['imu_device_time'][field], raw,
                                                        host_elapsed, profile)
                accumulated[field] += delta
                wraps[field] += int(wrapped)
                # A forward counter jump must not masquerade as elapsed motion.
                # This is a coarse discontinuity guard, not a clock calibration.
                _require(delta*profile['tick_ns'] <= host_elapsed+250_000_000,
                         'device timestamp advance inconsistent with host elapsed: '+field)
            first.setdefault(field, raw)
        relative.append(accumulated['sample_timestamp_raw']*profile['tick_ns'])
        previous = row
    host_span = rows[-1]['monotonic_ns']-rows[0]['monotonic_ns']
    for field in FIELDS:
        _require(abs(accumulated[field]*profile['tick_ns']-host_span) <= max(250_000_000, host_span//100),
                 'device/host elapsed span inconsistent with frozen unit: '+field)
    anchor = min(row['stamp_ns']-delta for row, delta in zip(rows, relative))
    _require(anchor >= 0, 'device-relative anchor predates supported ROS epoch')
    sequence_evidence = []
    for row, delta in zip(rows, relative):
        row['imu_measurement_stamp_ns'] = anchor+delta
        sequence_evidence.append([row['sequence'], row['monotonic_ns'], row['stamp_ns'],
                                 row['imu_measurement_stamp_ns'], row['cdr_sha256'],
                                 row['imu_device_time']['sample_timestamp_raw'],
                                 row['imu_device_time']['dataready_timestamp_raw']])
    report = dict(schema_version=1, mode='device_relative', scope='OFFLINE_ONLY',
        mapping='SAMPLE_COUNTER_RELATIVE_MINIMUM_HOST_ARRIVAL_OFFSET',
        timing_profile_sha256=profile['_profile_sha256'], sensor_id=sensor,
        session_id=config['session_id'], stream_epoch=rows[0]['imu_device_time']['stream_epoch'],
        device_timestamp_unit=profile['device_timestamp_unit'], tick_ns=profile['tick_ns'],
        first_sample_timestamp_raw=first['sample_timestamp_raw'],
        first_dataready_timestamp_raw=first['dataready_timestamp_raw'],
        anchor_ros_ns=anchor, sample_count=len(rows), derived_sequence_sha256=_sha(sequence_evidence),
        counter_wrap_policy=profile['backward_policy'], counter_wrap_counts=wraps,
        device_elapsed_ticks=accumulated,
        host_elapsed_guard=dict(adjacent_forward_allowance_ns=250_000_000,
                                whole_span_allowance='max(250 ms, 1 percent host span)'),
        original_message_stamps_modified=False, host_freshness_clock='ORIGINAL_HOST_MONOTONIC',
        event_dispatch_clock='HOST_ARRIVAL', future_data_used_for_offline_anchor=True,
        physical_measurement_time_validated=False, common_measurement_time_validated=False,
        intersensor_fixed_latency='UNKNOWN', sample_dataready_epoch_relationship='NOT_ASSUMED')
    report['model_sha256'] = _sha(report)
    for row in rows:
        row['imu_time_model_sha256'] = report['model_sha256']
    return report


def validate_model(model):
    _require(isinstance(model, dict), 'offline IMU timing model required')
    copy_model = copy.deepcopy(model)
    expected = copy_model.pop('model_sha256', None)
    _require(expected == _sha(copy_model) and copy_model.get('mode') == 'device_relative'
             and copy_model.get('scope') == 'OFFLINE_ONLY'
             and copy_model.get('physical_measurement_time_validated') is False
             and copy_model.get('common_measurement_time_validated') is False,
             'offline IMU timing model hash/scope invalid')
    return model


def configure_runtime(runtime, clock):
    """Freeze the same model into every replay cell and calibration candidate."""
    model = clock.report().get('imu_time_model')
    if model is not None:
        validate_model(model)
        runtime['offline_imu_time_model'] = copy.deepcopy(model)
        runtime['prior_template'].update(offline_experiment=True,
            offline_imu_time_mode='device_relative', offline_imu_time_model=copy.deepcopy(model))
    else:
        _require(runtime.get('offline_imu_time_model') is None
                 and runtime.get('prior_template', {}).get('offline_imu_time_model') is None,
                 'arrival replay cannot reuse a device timing model')


def prior_arguments(event):
    if 'imu_measurement_stamp_ns' not in event:
        return {}
    return dict(measurement_stamp_ns=event['imu_measurement_stamp_ns'],
                imu_time_model_sha256=event['imu_time_model_sha256'])
