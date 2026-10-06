"""Archive then relay one authoritative lidar stream for experimental mapping.

No sensor startup, TF, coordinate conversion, IMU or wheel subscriptions.
Saved .cdr files are complete wc_interfaces/msg/SourceFrame envelopes, including
the unmodified PointCloud2 XYZ/intensity storage and original timestamps.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import sys
import time

import numpy as np
from wc_sensors.pointcloud import decode_pointcloud2, validate_pointcloud2
from .storage_policy import StoragePolicy, resolve_storage_path

MAX_FRAMES = 12000
MIN_FREE_BYTES = 1_000_000_000
MAX_CDR_BYTES = 8_000_000
# Reviewed XT-M60 full sensor image and published float32 XYZ/amplitude layout.
MAX_CLOUD_POINTS = 320 * 240
MAX_CLOUD_BYTES = MAX_CLOUD_POINTS * 16
STALE_NS = 3_000_000_000
STARTUP_NS = 15_000_000_000


class InputFailure(RuntimeError):
    pass


def _require(condition, reason):
    if not condition:
        raise InputFailure(reason)


def _uint(value, reason):
    _require(type(value) is int and value >= 0, reason)
    return value


def _stamp(value):
    sec, nsec = _uint(value.sec, 'INVALID_HEADER_SECONDS'), _uint(value.nanosec, 'INVALID_HEADER_NANOSECONDS')
    _require(nsec < 1_000_000_000, 'INVALID_HEADER_NANOSECONDS')
    return sec * 1_000_000_000 + nsec


def project_path(root, value, *, storage=None):
    root, value = Path(root).absolute(), Path(value)
    _require('..' not in value.parts, 'PARENT_TRAVERSAL')
    try:
        storage = storage if storage is not None else StoragePolicy(root)
        path = storage.resolve(value)
    except (ValueError, OSError) as error:
        reason = 'REDIRECTED_PATH' if 'linked paths' in str(error) or 'symlink' in str(error) else 'PATH_OUTSIDE_PROJECT_OR_STORAGE'
        raise InputFailure(reason+': '+str(error)) from error
    _require(not any(p.is_symlink() or (hasattr(p, 'is_junction') and p.is_junction()) for p in (path, *path.parents)), 'REDIRECTED_PATH')
    _require(path != root, 'OUTPUT_MUST_BE_SESSION_DIRECTORY')
    _require(not storage.enabled or path != storage.archive_root, 'OUTPUT_MUST_BE_SESSION_DIRECTORY')
    return path


def _sync_dir(path):
    if os.name == 'nt':  # Pure local tests; production CLI verifies the Linux target.
        return
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n').encode('utf-8')


class SourceGate:
    """ROS-independent, fail-latched identity/time/schema gate and rate limiter."""
    def __init__(self, session_id, side, sensor_id, rate_hz=5., *, started_ns=None,
                 continuous_mapping=False, offline_experiment=False):
        _require(isinstance(session_id, str) and re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}', session_id), 'INVALID_SESSION_ID')
        _require(side in ('left', 'right') and isinstance(sensor_id, str) and bool(sensor_id.strip()), 'INVALID_SENSOR_IDENTITY')
        _require(type(offline_experiment) is bool, 'INVALID_OFFLINE_EXPERIMENT')
        _require(type(rate_hz) in (int, float) and math.isfinite(rate_hz) and
                 (0 < rate_hz <= 20 or (rate_hz == 0 and offline_experiment)), 'INVALID_RATE_HZ')
        _require(type(continuous_mapping) is bool, 'INVALID_CONTINUOUS_MAPPING_PROFILE')
        self.continuous_mapping = continuous_mapping
        self.session_id, self.side, self.sensor_id = session_id, side, sensor_id
        # Zero is an explicit offline-only policy: validate every source event
        # and let pairing/coverage decide eligibility, without synthetic timing.
        self.rate_hz = float(rate_hz)
        self.period_ns = int(math.ceil(1_000_000_000 / rate_hz)) if rate_hz else 0
        self.started_ns = time.monotonic_ns() if started_ns is None else started_ns
        self.failure_reason = None
        self.state = 'STARTING'
        self.stream_epoch = self.source_config_hash = None
        self.last_sequence = self.last_source_host_monotonic_ns = self.last_header_stamp_ns = None
        self.last_received_monotonic_ns = self.last_published_stamp_ns = self.last_forwarded_host_ns = None
        self.received_frames = self.validated_frames = self.throttled_frames = self.archived_frames = self.published_frames = 0
        self.sequence_gaps = self.invalid_xyz_points = 0

    def fail(self, reason):
        if self.failure_reason is None:
            self.failure_reason = str(reason)
        self.state = 'FAILED'

    def ensure_active(self):
        _require(self.failure_reason is None, self.failure_reason or 'INPUT_FAILED')
        _require(self.state != 'STOPPED', 'INPUT_ALREADY_STOPPED')

    def check_stale(self, now_ns):
        self.ensure_active()
        if self.continuous_mapping:
            return  # Missing/late data can resume; identity/order failures stay latched.
        if self.last_source_host_monotonic_ns is None:
            reason = 'SOURCE_STARTUP_TIMEOUT' if now_ns - self.started_ns > STARTUP_NS else None
        else:
            reason = 'SOURCE_STALE' if now_ns - self.last_source_host_monotonic_ns > STALE_NS else None
        if reason:
            self.fail(reason)
            raise InputFailure(reason)

    def inspect(self, message, now_ns):
        self.ensure_active()
        self.received_frames += 1
        self.last_received_monotonic_ns = now_ns
        try:
            _require(message.session_id == self.session_id and message.side == self.side and message.sensor_id == self.sensor_id, 'SOURCE_IDENTITY_MISMATCH')
            _require(message.units == 'm' and message.coordinate_convention == 'FLU', 'SOURCE_UNITS_OR_AXES_MISMATCH')
            _require(message.time_source == 'arrival_only' and message.common_time_valid is False and message.uncertainty_valid is False and message.clock_model_id == '', 'SOURCE_NOT_EXPLICIT_ARRIVAL_ONLY')
            _require(message.header.frame_id == 'lidar_' + self.side and message.cloud.header.frame_id == message.header.frame_id, 'SOURCE_FRAME_ID_MISMATCH')
            stamp = _stamp(message.header.stamp)
            _require(stamp > 0 and _stamp(message.cloud.header.stamp) == stamp and _stamp(message.host_receive_time) == stamp, 'SOURCE_HEADER_TIME_MISMATCH')
            _require(type(message.common_time_ns) is int and message.common_time_ns == stamp, 'ARRIVAL_TIME_METADATA_MISMATCH')
            host = _uint(message.host_monotonic_ns, 'INVALID_HOST_MONOTONIC')
            _require(0 <= now_ns-host and (self.continuous_mapping or now_ns-host <= STALE_NS),
                     'SOURCE_STALE_OR_FUTURE')
            if self.last_source_host_monotonic_ns is None and not self.continuous_mapping:
                _require(now_ns - self.started_ns <= STARTUP_NS, 'SOURCE_STARTUP_TIMEOUT')
            seq = _uint(message.frame_sequence, 'INVALID_SEQUENCE')
            epoch, config = message.stream_epoch, message.source_config_hash
            _require(isinstance(epoch, str) and 0 < len(epoch.strip()) <= 256, 'EMPTY_OR_INVALID_STREAM_EPOCH')
            _require(isinstance(config, str) and re.fullmatch(r'[a-fA-F0-9]{64}', config), 'INVALID_SOURCE_CONFIG_HASH')
            if self.stream_epoch is not None:
                _require(epoch == self.stream_epoch and config == self.source_config_hash, 'STREAM_EPOCH_OR_CONFIG_CHANGED')
                _require(seq > self.last_sequence, 'NONMONOTONIC_SOURCE_SEQUENCE')
                _require(host > self.last_source_host_monotonic_ns and stamp > self.last_header_stamp_ns, 'NONMONOTONIC_SOURCE_TIME')
            layout = validate_pointcloud2(message.cloud)
            _require(0 < layout['point_count'] <= MAX_CLOUD_POINTS and len(message.cloud.data) <= MAX_CLOUD_BYTES, 'CLOUD_SIZE_OUT_OF_BOUNDS')
            intensity = layout['fields'].get('intensity')
            _require(intensity is not None and intensity['count'] == 1 and intensity['datatype'] in (7, 8), 'INTENSITY_SCHEMA_REQUIRED')
            raw_count, valid_count = _uint(message.raw_count, 'INVALID_RAW_COUNT'), _uint(message.valid_count, 'INVALID_VALID_COUNT')
            _require(raw_count == layout['point_count'] and 0 < valid_count <= raw_count, 'SOURCE_POINT_COUNTS_MISMATCH')
            xyz = decode_pointcloud2(message.cloud)
            measured_valid = int(np.isfinite(xyz).all(axis=1).sum())
            _require(measured_valid == valid_count, 'FINITE_XYZ_COUNT_MISMATCH')
            _require(type(message.cloud.is_dense) is bool and (not message.cloud.is_dense or measured_valid == raw_count), 'CLOUD_DENSITY_FLAG_MISMATCH')
            if self.last_sequence is not None:
                self.sequence_gaps += seq - self.last_sequence - 1
            self.stream_epoch, self.source_config_hash = epoch, config
            self.last_sequence, self.last_source_host_monotonic_ns, self.last_header_stamp_ns = seq, host, stamp
            self.validated_frames += 1
            self.invalid_xyz_points += raw_count - valid_count
            self.state = 'RUNNING'
            if self.period_ns and self.last_forwarded_host_ns is not None and host - self.last_forwarded_host_ns < self.period_ns:
                self.throttled_frames += 1
                return None
            _require(self.continuous_mapping or self.archived_frames < MAX_FRAMES, 'FRAME_ARCHIVE_LIMIT_REACHED')
            return {'raw_key': [self.session_id, self.sensor_id, epoch, seq], 'frame_sequence': seq,
                    'side': self.side, 'sensor_id': self.sensor_id, 'stream_epoch': epoch, 'source_config_hash': config,
                    'units': 'm', 'coordinate_convention': 'FLU', 'frame_id': message.header.frame_id,
                    'header_stamp_ns': stamp, 'host_receive_time_ns': stamp, 'host_monotonic_ns': host,
                    'guard_receive_monotonic_ns': now_ns, 'time_source': 'arrival_only',
                    'common_time_valid': False, 'uncertainty_valid': False,
                    'raw_count': raw_count, 'valid_count': valid_count, 'invalid_xyz_count': raw_count - valid_count,
                    'cloud_data_sha256': hashlib.sha256(bytes(message.cloud.data)).hexdigest()}
        except Exception as error:
            self.fail(str(error))
            raise InputFailure(self.failure_reason) from error

    def status(self, now_ns):
        return {'schema_version': 1, 'kind': 'SINGLE_LIDAR_EXPERIMENT', 'source_mode': 'real',
                'sensor_mode': 'single_' + self.side, 'time_source': 'arrival_only', 'formal_acceptance': False,
                'state': self.state, 'failure_reason': self.failure_reason, 'session_id': self.session_id,
                'side': self.side, 'sensor_id': self.sensor_id, 'rate_hz': self.rate_hz,
                'rate_policy': 'NO_OFFLINE_THROTTLE' if self.period_ns == 0 else 'MINIMUM_SOURCE_INTERVAL',
                'input_topic': '/wc_mapping/lidar_' + self.side + '/source_frame_filtered',
                'output_topic': '/wc_mapping/single_' + self.side + '/scan_cloud',
                **{key: getattr(self, key) for key in ('received_frames', 'validated_frames', 'throttled_frames',
                   'archived_frames', 'published_frames', 'sequence_gaps', 'invalid_xyz_points', 'last_sequence',
                   'last_source_host_monotonic_ns', 'last_received_monotonic_ns', 'last_published_stamp_ns',
                   'stream_epoch', 'source_config_hash')},
                'updated_monotonic_ns': now_ns, 'updated_utc': datetime.now(timezone.utc).isoformat(),
                'continuous_mapping': self.continuous_mapping,
                'source_wait_policy': 'WAIT_WITHOUT_DEADLINE' if self.continuous_mapping else 'STRICT_TIMEOUT',
                'source_age_ns': now_ns-self.last_source_host_monotonic_ns
                    if self.last_source_host_monotonic_ns is not None else None,
                'max_archived_frames': None if self.continuous_mapping else MAX_FRAMES, 'min_free_bytes': MIN_FREE_BYTES,
                'coordinate_transform_applied': False, 'point_filter_applied': False, 'deskew_applied': False,
                'archive_format': 'wc_interfaces/msg/SourceFrame CDR; original filtered cloud bytes preserved',
                'calibration_status': 'EXPERIMENTAL_UNVALIDATED', 'navigation_validated': False}


class FrameArchive:
    def __init__(self, project_root, output_root, *, continuous_mapping=False):
        _require(type(continuous_mapping) is bool, 'INVALID_CONTINUOUS_MAPPING_PROFILE')
        self.continuous_mapping = continuous_mapping
        self.project_root = Path(project_root).absolute()
        self.storage = StoragePolicy(self.project_root)
        self.root = project_path(self.project_root, output_root, storage=self.storage)
        self.root.mkdir(parents=True, exist_ok=True)
        self.frames = project_path(self.project_root, self.root / 'frames', storage=self.storage)
        _require(not any((self.root / name).exists() for name in ('frames', 'frames.jsonl', 'status.json', 'status.json.partial')), 'ARCHIVE_ALREADY_EXISTS')
        _require(shutil.disk_usage(self.root).free >= MIN_FREE_BYTES, 'DISK_FREE_MARGIN_EXHAUSTED')
        self.frames.mkdir(exist_ok=False)
        self.index = (self.root / 'frames.jsonl').open('xb')
        self.count = 0
        _sync_dir(self.root)

    def append(self, cdr, metadata):
        _require(isinstance(cdr, bytes) and 0 < len(cdr) <= MAX_CDR_BYTES, 'INVALID_CDR_ENVELOPE')
        _require(self.continuous_mapping or self.count < MAX_FRAMES, 'FRAME_ARCHIVE_LIMIT_REACHED')
        _require(shutil.disk_usage(self.root).free >= MIN_FREE_BYTES + len(cdr) + 65536, 'DISK_FREE_MARGIN_EXHAUSTED')
        destination = project_path(self.project_root, self.frames / ('%08d.cdr' % (self.count + 1)), storage=self.storage)
        with destination.open('xb') as output:
            output.write(cdr); output.flush(); os.fsync(output.fileno())
        _sync_dir(self.frames)
        row = dict(metadata, archive_index=self.count + 1, file=destination.relative_to(self.root).as_posix(),
                   serialization_format='cdr', ros_type='wc_interfaces/msg/SourceFrame',
                   cdr_bytes=len(cdr), cdr_sha256=hashlib.sha256(cdr).hexdigest(), source_mode='real',
                   formal_acceptance=False, coordinate_transform_applied=False)
        self.index.write(_json_bytes(row)); self.index.flush(); os.fsync(self.index.fileno())
        self.count += 1
        return row

    def status(self, payload):
        destination = project_path(self.project_root, self.root / 'status.json', storage=self.storage)
        temporary = project_path(self.project_root, self.root / 'status.json.partial', storage=self.storage)
        with temporary.open('wb') as output:
            output.write(_json_bytes(payload)); output.flush(); os.fsync(output.fileno())
        os.replace(temporary, destination); _sync_dir(self.root)

    def close(self):
        self.index.close()


def forward_frame(gate, archive, message, serialize, publish, *, clock=time.monotonic_ns):
    """Publication is reachable only after both CDR and index have been fsynced."""
    try:
        metadata = gate.inspect(message, clock())
        if metadata is None:
            return False
        archive.append(serialize(message), metadata)
        gate.archived_frames += 1
        gate.check_stale(clock())
        publish(message.cloud)
        gate.published_frames += 1
        gate.last_forwarded_host_ns = message.host_monotonic_ns
        gate.last_published_stamp_ns = metadata['header_stamp_ns']
        return True
    except Exception as error:
        gate.fail(str(error))
        raise InputFailure(gate.failure_reason) from error


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-id', required=True)
    parser.add_argument('--side', choices=('right', 'left'), required=True)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--rate-hz', type=float, default=5.)
    args = parser.parse_args(argv)
    from .cli import ROOT, target
    target()  # Identity check only; no sensors or operating-system changes.
    config_path = project_path(ROOT, ROOT / 'config/live_unvalidated.json')
    config = json.loads(config_path.read_text(encoding='utf-8'))
    _require(config.get('source_mode') == 'real' and isinstance(config.get('sensor_ids'), dict)
             and set(config['sensor_ids']) == {'left', 'right'}, 'LIVE_SENSOR_CONFIG_REQUIRED')
    gate = SourceGate(args.session_id, args.side, config['sensor_ids'][args.side], args.rate_hz)
    archive = FrameArchive(ROOT, args.output_root)
    node = None
    try:
        archive.status(gate.status(time.monotonic_ns()))
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rclpy.serialization import serialize_message
        from rclpy.executors import ExternalShutdownException
        from sensor_msgs.msg import PointCloud2
        from wc_interfaces.msg import SourceFrame
        rclpy.init(args=[])
        node = Node('wc_single_' + args.side + '_input')
        output_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        source_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        pub = node.create_publisher(PointCloud2, '/wc_mapping/single_' + args.side + '/scan_cloud', output_qos)

        def receive(message):
            if gate.failure_reason:
                return
            try:
                forward_frame(gate, archive, message, serialize_message, pub.publish)
            except Exception as error:
                node.get_logger().error('Single lidar input failed: ' + str(error))

        node.create_subscription(SourceFrame, '/wc_mapping/lidar_' + args.side + '/source_frame_filtered', receive, source_qos)
        last_status = 0
        try:
            while rclpy.ok() and gate.failure_reason is None:
                rclpy.spin_once(node, timeout_sec=.2)
                now = time.monotonic_ns()
                if gate.failure_reason is None:
                    gate.check_stale(now)
                if now - last_status >= 1_000_000_000 or gate.failure_reason:
                    archive.status(gate.status(now)); last_status = now
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    except Exception as error:
        gate.fail(str(error))
        print('single_mapping_input: ' + str(error), file=sys.stderr, flush=True)
    finally:
        if gate.failure_reason is None:
            gate.state = 'STOPPED'
        try:
            archive.status(gate.status(time.monotonic_ns()))
        except Exception as error:
            gate.fail('STATUS_WRITE_FAILED: ' + str(error))
            print(gate.failure_reason, file=sys.stderr, flush=True)
        archive.close()
        if node is not None:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 1 if gate.failure_reason else 0


if __name__ == '__main__':
    raise SystemExit(main())
