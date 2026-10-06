"""Exact raw/filtered frame pairing; display filtering never replaces ray evidence."""
from collections import OrderedDict
import hashlib
from .core import FusionError, integer_ns


def frame_identity(message):
    return (message.session_id, message.side, message.sensor_id,
            message.stream_epoch, int(message.frame_sequence))


def acquisition_signature(message):
    return (message.header.frame_id, message.header.stamp.sec, message.header.stamp.nanosec,
            message.cloud.header.frame_id, message.cloud.header.stamp.sec, message.cloud.header.stamp.nanosec,
            message.device_timestamp_valid, message.device_timestamp_raw, message.device_timestamp_unit,
            message.device_time_components_valid, message.device_timestamp_seconds, message.device_timestamp_nanoseconds,
            message.device_time_type, message.device_sync_state,
            message.host_receive_time.sec, message.host_receive_time.nanosec, message.host_monotonic_ns,
            message.common_time_valid, message.common_time_ns, message.clock_model_id,
            message.time_source, message.uncertainty_valid, message.uncertainty_ns,
            message.coordinate_convention, message.units)


def content_signature(message):
    return (acquisition_signature(message), message.source_config_hash,
            hashlib.sha256(bytes(message.cloud.data)).hexdigest())


def verify_raw_reference(raw, filtered):
    flags = set(filtered.diagnostic_flags)
    expected = {'raw_source_config_hash=' + raw.source_config_hash,
                'before_host_filter_cloud_sha256=' + hashlib.sha256(bytes(raw.cloud.data)).hexdigest()}
    if not expected.issubset(flags):
        raise FusionError('RAW_FILTERED_CONTENT_REFERENCE_MISMATCH')


class RawFilteredJoin:
    def __init__(self, capacity=32, timeout_ns=300_000_000):
        if capacity < 1 or timeout_ns < 1:
            raise FusionError('INVALID_FILTER_JOIN_LIMIT')
        self.capacity, self.timeout_ns = capacity, timeout_ns
        self.pending = OrderedDict()
        self.completed = OrderedDict()

    def clear(self):
        self.pending.clear()

    def expire(self, now_ns):
        now_ns = integer_ns(now_ns)
        if any(not 0 <= now_ns-row['received'] <= self.timeout_ns for row in self.pending.values()):
            self.pending.clear()
            raise FusionError('RAW_FILTERED_FRAME_TIMEOUT')

    def push(self, kind, message, now_ns):
        if kind not in ('raw', 'filtered'):
            raise FusionError('UNKNOWN_FILTER_BRANCH')
        self.expire(now_ns)
        key = frame_identity(message)
        if key in self.completed:
            if self.completed[key][kind] != content_signature(message):
                raise FusionError('RAW_FILTERED_COMPLETED_CONTENT_CHANGED')
            return None
        if key not in self.pending:
            if len(self.pending) >= self.capacity:
                raise FusionError('RAW_FILTERED_QUEUE_FULL')
            self.pending[key] = {'received': now_ns}
        row = self.pending[key]
        if kind in row:
            raise FusionError('RAW_FILTERED_DUPLICATE_BRANCH')
        row[kind] = message
        if 'raw' not in row or 'filtered' not in row:
            return None
        del self.pending[key]
        if acquisition_signature(row['raw']) != acquisition_signature(row['filtered']):
            raise FusionError('RAW_FILTERED_ACQUISITION_MISMATCH')
        verify_raw_reference(row['raw'], row['filtered'])
        self.completed[key] = {kind: content_signature(row[kind]) for kind in ('raw', 'filtered')}
        while len(self.completed) > self.capacity * 4:
            self.completed.popitem(last=False)
        return row['raw'], row['filtered']
