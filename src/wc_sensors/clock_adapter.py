"""Pure application of an externally validated clock model to SourceFrame.

This module neither fits nor validates physical clocks. It checks the supplied
validation contract and applies its integer-rational mapping. Evidence references
are checked as declarations, never opened; their authenticity and sufficiency
remain the responsibility of the independent validation/loading workflow.
"""

from copy import deepcopy
from numbers import Integral
import re

from .metadata import ClockModel, timestamp_ns


UINT64_MAX = (1 << 64) - 1
INT64_MAX = (1 << 63) - 1
_BASIS = frozenset(('independent_measurement', 'hardware_synchronization'))
_CHANGED_FIELDS = ('common_time_valid', 'common_time_ns', 'clock_model_id',
                   'uncertainty_valid', 'uncertainty_ns', 'time_source')


class ClockAdapterError(ValueError):
    """A frame/model contract mismatch; the caller must reject this observation."""


def _value(obj, name):
    try:
        return obj[name] if isinstance(obj, dict) else getattr(obj, name)
    except (KeyError, AttributeError) as error:
        raise ClockAdapterError(f'missing required field: {name}') from error


def _text(value, name, *, identifier=False):
    if not isinstance(value, str) or not value.strip() or value != value.strip() or '\0' in value:
        raise ClockAdapterError(f'{name} must be an explicit nonempty string')
    if value.upper().startswith(('UNKNOWN', 'UNVERIFIED', 'TBD', 'TODO')):
        raise ClockAdapterError(f'{name} is unresolved')
    if identifier and (any(c.isspace() for c in value) or '/' in value or '\\' in value):
        raise ClockAdapterError(f'{name} must be an unambiguous identifier')
    return value


def _integer(value, name, *, minimum=0, maximum=None):
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ClockAdapterError(f'{name} must be an integer, not float/bool')
    value = int(value)
    if (minimum is not None and value < minimum) or (maximum is not None and value > maximum):
        raise ClockAdapterError(f'{name} outside valid range')
    return value


def _true(value, name):
    if value is not True:
        raise ClockAdapterError(f'{name} must explicitly be true')


def _evidence(value, name):
    if not isinstance(value, (list, tuple)) or not value:
        raise ClockAdapterError(f'{name} must contain nonempty evidence references')
    for reference in value:
        _text(reference, name)


def apply_clock_model(frame_msg, model_dict, expected_sensor_id, expected_model_id):
    """Return a deep-copied SourceFrame with only six common-time fields changed.

    Required model schema (all keys are explicit; no defaults are supplied):
      schema_version=1, source_mode='real', status='VALIDATED', validated=True;
      model_id, sensor_id, source_config_hash (64 hex SHA256), stream_epoch;
      input_timestamp='device_scalar'|'device_components_ns';
      device_timestamp_unit, scale_numerator, scale_denominator, offset_ns,
      uncertainty_ns, valid_raw_min, valid_raw_max;
      evidence_refs, unit_evidence_refs, measurement_evidence_refs (nonempty);
      unit_verified=True, measurement_semantics_verified=True,
      measurement_semantics (resolved description of the sample-time reference);
      validation_basis='independent_measurement'|'hardware_synchronization'.

    Bounds are inclusive and both mandatory. Scalar input uses the unchanged
    uint64 device_timestamp_raw and its exact unit label. Components input uses
    Python int seconds*1e9+nanoseconds, requires model unit 'ns', and never uses
    the optional scalar field or its unit. Components themselves are uint64
    seconds and canonical nanoseconds; their combined integer may exceed uint64.

    Only an unadapted native arrival_only envelope can be converted. A previous
    validated/synthetic frame is rejected to prevent accidental reapplication.
    The result must satisfy ClockModel's nonnegative-time rule and signed ROS
    int64 capacity. Invalid models/frames raise ClockAdapterError without mutation.
    No header stamp, device time, host time, cloud, diagnostic or identity changes.
    """
    if not isinstance(model_dict, dict):
        raise ClockAdapterError('model_dict must be a dictionary; no model loading is performed')
    expected_sensor_id = _text(expected_sensor_id, 'expected_sensor_id', identifier=True)
    expected_model_id = _text(expected_model_id, 'expected_model_id', identifier=True)
    if _integer(_value(model_dict, 'schema_version'), 'schema_version') != 1:
        raise ClockAdapterError('unsupported clock model schema')
    if _value(model_dict, 'source_mode') != 'real' or _value(model_dict, 'status') != 'VALIDATED':
        raise ClockAdapterError('only externally VALIDATED real clock models are accepted')
    _true(_value(model_dict, 'validated'), 'validated')
    for name in ('evidence_refs', 'unit_evidence_refs', 'measurement_evidence_refs'):
        _evidence(_value(model_dict, name), name)
    for name in ('unit_verified', 'measurement_semantics_verified'):
        _true(_value(model_dict, name), name)
    _text(_value(model_dict, 'measurement_semantics'), 'measurement_semantics')
    if _text(_value(model_dict, 'validation_basis'), 'validation_basis') not in _BASIS:
        raise ClockAdapterError('arrival fits or unspecified validation basis cannot establish measurement time')
    if _text(_value(model_dict, 'model_id'), 'model_id', identifier=True) != expected_model_id:
        raise ClockAdapterError('clock model ID mismatch')
    if _text(_value(model_dict, 'sensor_id'), 'sensor_id', identifier=True) != expected_sensor_id:
        raise ClockAdapterError('model sensor identity mismatch')
    if _value(frame_msg, 'sensor_id') != expected_sensor_id:
        raise ClockAdapterError('frame sensor identity mismatch')
    epoch = _text(_value(model_dict, 'stream_epoch'), 'stream_epoch', identifier=True)
    if _value(frame_msg, 'stream_epoch') != epoch:
        raise ClockAdapterError('stream epoch mismatch; reconnect/reset requires new validation')
    config_hash = _text(_value(model_dict, 'source_config_hash'), 'source_config_hash')
    if not re.fullmatch(r'[0-9a-fA-F]{64}', config_hash):
        raise ClockAdapterError('source_config_hash must be the native driver SHA256 hex value')
    if _value(frame_msg, 'source_config_hash') != config_hash:
        raise ClockAdapterError('source configuration hash mismatch')
    if _value(frame_msg, 'time_source') != 'arrival_only':
        raise ClockAdapterError('only original arrival_only native frames may be adapted')
    if _value(frame_msg, 'common_time_valid') is not False or _value(frame_msg, 'uncertainty_valid') is not False:
        raise ClockAdapterError('arrival_only frame must have unknown common time and uncertainty')
    if _value(frame_msg, 'clock_model_id') != '':
        raise ClockAdapterError('native arrival_only frame must not already name a clock model')
    # Explicit provenance, when present on a test/extended envelope, cannot
    # contradict the real source contract. Native SourceFrame has no such field.
    source_mode = frame_msg.get('source_mode') if isinstance(frame_msg, dict) else getattr(frame_msg, 'source_mode', None)
    if source_mode is not None and source_mode != 'real':
        raise ClockAdapterError('synthetic/unknown frame provenance cannot use a real clock model')

    input_timestamp = _value(model_dict, 'input_timestamp')
    unit = _text(_value(model_dict, 'device_timestamp_unit'), 'device_timestamp_unit')
    if input_timestamp == 'device_scalar':
        _true(_value(frame_msg, 'device_timestamp_valid'), 'device_timestamp_valid')
        if _value(frame_msg, 'device_timestamp_unit') != unit:
            raise ClockAdapterError('scalar timestamp unit mismatch')
        raw = _integer(_value(frame_msg, 'device_timestamp_raw'), 'device_timestamp_raw', maximum=UINT64_MAX)
    elif input_timestamp == 'device_components_ns':
        _true(_value(frame_msg, 'device_time_components_valid'), 'device_time_components_valid')
        if unit != 'ns':
            raise ClockAdapterError('components model unit must be ns with explicit unit evidence')
        seconds = _integer(_value(frame_msg, 'device_timestamp_seconds'), 'device_timestamp_seconds', maximum=UINT64_MAX)
        nanos = _integer(_value(frame_msg, 'device_timestamp_nanoseconds'), 'device_timestamp_nanoseconds', maximum=999_999_999)
        raw = timestamp_ns(seconds, nanos)
    else:
        raise ClockAdapterError('input_timestamp must explicitly select a device time representation')
    lower = _integer(_value(model_dict, 'valid_raw_min'), 'valid_raw_min')
    upper = _integer(_value(model_dict, 'valid_raw_max'), 'valid_raw_max', minimum=lower)
    uncertainty = _integer(_value(model_dict, 'uncertainty_ns'), 'uncertainty_ns', maximum=UINT64_MAX)
    try:
        model = ClockModel(model_id=expected_model_id, device_timestamp_unit=unit,
            scale_numerator=_integer(_value(model_dict, 'scale_numerator'), 'scale_numerator', minimum=1),
            scale_denominator=_integer(_value(model_dict, 'scale_denominator'), 'scale_denominator', minimum=1),
            offset_ns=_integer(_value(model_dict, 'offset_ns'), 'offset_ns', minimum=None),
            uncertainty_ns=uncertainty, validated=True, valid_raw_min=lower, valid_raw_max=upper)
        common = model.convert(raw, unit)
    except (TypeError, ValueError) as error:
        raise ClockAdapterError(str(error)) from error
    if not 0 <= common <= INT64_MAX:
        raise ClockAdapterError('common_time_ns exceeds nonnegative ROS int64 range')
    values = (True, common, expected_model_id, True, uncertainty, 'validated_model')
    result = deepcopy(frame_msg)
    for name, value in zip(_CHANGED_FIELDS, values):
        if isinstance(result, dict):
            result[name] = value
        else:
            setattr(result, name, value)
    return result
