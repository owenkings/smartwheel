#!/usr/bin/env python3
"""Read-only audit of closed project SQLite rosbag2 sensor envelopes.

Run on Orin after sourcing the project's ROS installation. This script creates
no ROS node, opens no device, does not play/reindex/repair bags, and writes only
the requested new JSON report. Arrival-time statistics never establish common
sample synchronization, static-map acceptance or dynamic-map acceptance.

DATA_REVIEW_OK covers the recorded envelopes and SQLite/metadata consistency.
It does not establish graceful exit of the recorder or any sensor process; a
managed STOPPED/0 summary cannot override a component log reporting exit -2.
Use the acquisition and component lifecycle logs for that separate acceptance.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from dataclasses import dataclass, field
import hashlib
import itertools
import json
import math
from pathlib import Path
import sqlite3
import sys
import time

import numpy as np
import yaml


LIDAR_TYPE = 'wc_interfaces/msg/SourceFrame'
IMU_TYPE = 'wc_interfaces/msg/H30Frame'
LEFT_TOPIC = '/wc_mapping/lidar_left/source_frame'
RIGHT_TOPIC = '/wc_mapping/lidar_right/source_frame'
IMU_TOPIC = '/wc_mapping/imu/source_frame'
KNOWN_TOPICS = {LEFT_TOPIC: LIDAR_TYPE, RIGHT_TOPIC: LIDAR_TYPE, IMU_TOPIC: IMU_TYPE}
NS = 1_000_000_000


def stamp_ns(value):
    sec, nano = int(value.sec), int(value.nanosec)
    if not 0 <= nano < NS:
        raise ValueError('noncanonical ROS nanoseconds')
    return sec * NS + nano


def quantiles(values):
    if not values:
        return {'count': 0}
    # Absolute clock values are never converted to float; only differences and
    # physical measurements enter these descriptive quantiles.
    data = np.asarray(values, dtype=np.float64)
    return {'count': len(values), 'min': float(np.min(data)), 'max': float(np.max(data)),
            **{f'p{p}': float(np.percentile(data, p)) for p in (1, 5, 50, 95, 99)}}


def clock_summary(values, ns_units=True):
    if not values:
        return {'count': 0}
    intervals = [b-a for a, b in zip(values, values[1:])]
    elapsed = values[-1]-values[0]
    return {'count': len(values), 'first_raw_integer': values[0], 'last_raw_integer': values[-1],
            'equal_intervals': sum(x == 0 for x in intervals),
            'backward_intervals': sum(x < 0 for x in intervals),
            'interval_' + ('ns' if ns_units else 'raw_units'): quantiles(intervals),
            'rate_hz_over_first_last': ((len(values)-1)*NS/elapsed if ns_units and elapsed > 0 else None)}


def sequence_summary(values):
    steps = [b-a for a, b in zip(values, values[1:])]
    return {'count': len(values), 'first': values[0] if values else None,
            'last': values[-1] if values else None, 'unique': len(set(values)),
            'duplicates': len(values)-len(set(values)),
            'backward_steps': sum(x < 0 for x in steps),
            'forward_gap_events': sum(x > 1 for x in steps),
            'missing_sequence_numbers_in_forward_steps': sum(max(0, x-1) for x in steps),
            'note': 'gaps describe recorded envelopes; they do not identify where loss occurred'}


def fingerprint(path):
    value = path.stat()
    return {'size': value.st_size, 'mtime_ns': value.st_mtime_ns, 'inode': value.st_ino}


def within(path, parent):
    path, parent = Path(path).absolute(), Path(parent).resolve(strict=True)
    resolved = path.resolve(strict=True)
    if not resolved.is_relative_to(parent):
        raise ValueError(f'path outside permitted project area: {path}')
    current = path
    while current != parent and current != current.parent:
        if current.is_symlink():
            raise ValueError(f'symlink not accepted for evidence path: {current}')
        current = current.parent
    return resolved


@dataclass
class Stream:
    kind: str
    side: str
    sensor_id: str
    epoch: str
    sessions: Counter = field(default_factory=Counter)
    topics: Counter = field(default_factory=Counter)
    sequence: list = field(default_factory=list)
    recorded: list = field(default_factory=list)
    receive: list = field(default_factory=list)
    monotonic: list = field(default_factory=list)
    device_components: list = field(default_factory=list)
    component_examples: list = field(default_factory=list)
    last_components: dict | None = None
    device_raw: dict = field(default_factory=lambda: defaultdict(list))
    common: list = field(default_factory=list)
    properties: dict = field(default_factory=lambda: defaultdict(Counter))
    flags: Counter = field(default_factory=Counter)
    counts: Counter = field(default_factory=Counter)
    ratios: list = field(default_factory=list)
    frame_ranges: list = field(default_factory=list)
    xyz_min: list = field(default_factory=lambda: [math.inf]*3)
    xyz_max: list = field(default_factory=lambda: [-math.inf]*3)
    imu_values: dict = field(default_factory=lambda: defaultdict(list))
    tids: list = field(default_factory=list)
    packet_hashes: set = field(default_factory=set)
    raw_digest: object = field(default_factory=hashlib.sha256)
    examples: list = field(default_factory=list)
    last_example: dict | None = None

    def finish(self):
        result = {'kind': self.kind, 'side': self.side, 'sensor_id': self.sensor_id,
                  'stream_epoch': self.epoch, 'sessions': dict(self.sessions), 'topics': dict(self.topics),
                  'sequence': sequence_summary(self.sequence),
                  'bag_record_time': clock_summary(self.recorded),
                  'host_receive_time': clock_summary(self.receive),
                  'host_monotonic_time': clock_summary(self.monotonic),
                  'bag_minus_host_receive_ns': quantiles([a-b for a, b in zip(self.recorded, self.receive)]),
                  'properties': {k: dict(v) for k, v in self.properties.items()},
                  'diagnostic_flags': dict(self.flags), 'counts': dict(self.counts),
                  'device_components_ns': clock_summary(self.device_components),
                  'device_components_examples': self.component_examples,
                  'device_components_last': self.last_components,
                  'device_raw_clocks': {k: clock_summary(v, False) for k, v in self.device_raw.items()},
                  'declared_common_time': clock_summary(self.common),
                  'sample_synchronization_validated_by_audit': False}
        if self.kind == 'lidar':
            result['cloud'] = {'finite_ratio_per_frame': quantiles(self.ratios),
                'point_range_m_per_frame': {'min': quantiles([r[0] for r in self.frame_ranges]),
                    'median': quantiles([r[1] for r in self.frame_ranges]),
                    'p95': quantiles([r[2] for r in self.frame_ranges]),
                    'max': quantiles([r[3] for r in self.frame_ranges])},
                'finite_xyz_min_m': [x if math.isfinite(x) else None for x in self.xyz_min],
                'finite_xyz_max_m': [x if math.isfinite(x) else None for x in self.xyz_max],
                'metric_scale_physically_validated': False,
                'note': 'observed coordinate ranges are descriptive, not a range calibration'}
        else:
            result['imu'] = {'field_values': {k: quantiles(v) for k, v in self.imu_values.items()},
                'tid': sequence_summary(self.tids),
                'tid_60000_to_1_transitions': sum(a == 60000 and b == 1 for a, b in zip(self.tids, self.tids[1:])),
                'raw_packet_unique_hashes': len(self.packet_hashes),
                'raw_packet_duplicates': self.counts['raw_packets']-len(self.packet_hashes),
                'ordered_length_prefixed_raw_packets_sha256': self.raw_digest.hexdigest(),
                'packet_examples': self.examples, 'last_packet': self.last_example,
                'tid_note': 'raw TID wrap/backward is reported, never silently unwrapped'}
        return result


class Audit:
    def __init__(self, expected, mode, require_h30):
        self.expected, self.mode, self.require_h30 = expected, mode, require_h30
        self.streams, self.raw_keys = {}, {}
        self.issues = Counter()
        self.issue_examples = defaultdict(list)
        self.observed = Counter()
        self.content_seen = {'left': set(), 'right': set()}
        self.identities = {'left': set(), 'right': set()}

    def issue(self, code, detail=''):
        self.issues[code] += 1
        if len(self.issue_examples[code]) < 5:
            self.issue_examples[code].append(str(detail))

    def consume(self, topic, message, recorded):
        from wc_sensors.pointcloud import decode_pointcloud2
        kind = 'lidar' if topic in (LEFT_TOPIC, RIGHT_TOPIC) else 'imu'
        side = message.side if kind == 'lidar' else 'imu'
        identity, epoch = message.sensor_id, message.stream_epoch
        if not identity or not epoch or not message.session_id:
            self.issue('empty_source_identity', topic)
        if kind == 'lidar':
            actual_side = 'left' if topic == LEFT_TOPIC else 'right'
            if side != actual_side:
                self.issue('topic_side_mismatch', f'{topic}: {side}')
            if identity != self.expected[actual_side]:
                self.issue('unexpected_sensor_id', f'{topic}: {identity}')
            self.identities[actual_side].add(identity)
        elif self.expected.get('imu') and identity != self.expected['imu']:
            self.issue('unexpected_imu_id', identity)
        key = (kind, side, identity, epoch)
        stream = self.streams.setdefault(key, Stream(kind, side, identity, epoch))
        sequence = int(message.frame_sequence)
        raw_key = identity + '/' + epoch + '/' + str(sequence)
        if raw_key in self.raw_keys:
            self.issue('duplicate_raw_key', raw_key)
            if self.raw_keys[raw_key] != topic:
                self.issue('cross_topic_raw_key_collision', raw_key)
        self.raw_keys[raw_key] = topic
        self.observed[topic] += 1
        stream.sessions[message.session_id] += 1
        stream.topics[topic] += 1
        stream.sequence.append(sequence)
        stream.recorded.append(int(recorded))
        receive = stamp_ns(message.host_receive_time)
        stream.receive.append(receive)
        stream.monotonic.append(int(message.host_monotonic_ns))
        for prop in ('time_source', 'coordinate_convention', 'device_timestamp_unit'):
            stream.properties[prop][getattr(message, prop)] += 1
        stream.flags.update(message.diagnostic_flags)
        if message.time_source == 'synthetic':
            self.issue('synthetic_source_in_requested_live_bag', raw_key)
        if message.common_time_valid:
            stream.common.append(int(message.common_time_ns))
            stream.counts['declared_common_time_valid'] += 1
        if message.uncertainty_valid:
            stream.counts['declared_uncertainty_valid'] += 1
        if message.time_source == 'arrival_only' and (message.common_time_valid or message.uncertainty_valid):
            self.issue('arrival_only_claims_valid_common_time_or_uncertainty', raw_key)
        if stamp_ns(message.header.stamp) != receive and message.time_source == 'arrival_only':
            self.issue('arrival_header_receive_mismatch', raw_key)
        if kind == 'imu':
            self.consume_imu(stream, message, raw_key)
            return
        for prop in ('units', 'source_config_hash', 'clock_model_id', 'device_time_type', 'device_sync_state'):
            stream.properties[prop][getattr(message, prop)] += 1
        if message.coordinate_convention != 'FLU' or message.units != 'm':
            self.issue('unexpected_lidar_coordinate_or_unit', raw_key)
        if not message.source_config_hash:
            self.issue('missing_source_config_hash', raw_key)
        if message.device_time_components_valid:
            ns = int(message.device_timestamp_nanoseconds)
            if not 0 <= ns < NS:
                self.issue('invalid_device_time_components', raw_key)
            else:
                stream.device_components.append(int(message.device_timestamp_seconds)*NS+ns)
                stream.counts['device_components_valid'] += 1
                components = {'raw_key': raw_key, 'seconds': int(message.device_timestamp_seconds), 'nanoseconds': ns}
                if len(stream.component_examples) < 3:
                    stream.component_examples.append(components)
                stream.last_components = components
        else:
            stream.counts['device_components_unavailable'] += 1
        if message.device_timestamp_valid:
            stream.device_raw['scalar:' + message.device_timestamp_unit].append(int(message.device_timestamp_raw))
        if message.cloud.header.frame_id != message.header.frame_id:
            self.issue('cloud_envelope_frame_id_mismatch', raw_key)
        if stamp_ns(message.cloud.header.stamp) != stamp_ns(message.header.stamp):
            self.issue('cloud_envelope_stamp_mismatch', raw_key)
        points = decode_pointcloud2(message.cloud)
        finite = np.isfinite(points).all(axis=1)
        count, valid = len(points), int(finite.sum())
        stream.counts.update(raw_points=count, finite_points=valid, nonfinite_points=count-valid)
        if int(message.raw_count) != count or int(message.valid_count) != valid:
            self.issue('declared_point_count_mismatch', raw_key)
        if count == 0:
            self.issue('empty_pointcloud', raw_key)
        stream.ratios.append(valid/count if count else 0.0)
        if valid:
            good = points[finite]
            ranges = np.linalg.norm(good, axis=1)
            stream.frame_ranges.append([float(np.min(ranges)), float(np.median(ranges)),
                                       float(np.percentile(ranges, 95)), float(np.max(ranges))])
            stream.xyz_min = np.minimum(stream.xyz_min, np.min(good, axis=0)).tolist()
            stream.xyz_max = np.maximum(stream.xyz_max, np.max(good, axis=0)).tolist()
            stream.counts['zero_range_points'] += int((ranges == 0).sum())
        digest = hashlib.sha256(bytes(message.cloud.data)).hexdigest()
        actual_side = 'left' if topic == LEFT_TOPIC else 'right'
        self.content_seen[actual_side].add(digest)

    def consume_imu(self, stream, message, raw_key):
        from wc_sensors.h30 import H30Parser
        packet = bytes(message.raw_packet)
        stream.counts['raw_packets'] += 1
        stream.counts['raw_packet_bytes'] += len(packet)
        digest = hashlib.sha256(packet).hexdigest()
        stream.packet_hashes.add(digest)
        stream.raw_digest.update(len(packet).to_bytes(4, 'little'))
        stream.raw_digest.update(packet)
        example = {'raw_key': raw_key, 'tid': int(message.tid), 'bytes': len(packet),
                   'sha256': digest, 'raw_hex': packet.hex() if len(packet) <= 262 else None}
        if len(stream.examples) < 3:
            stream.examples.append(example)
        stream.last_example = example
        stream.tids.append(int(message.tid))
        parser = H30Parser()
        decoded = parser.feed(packet)
        if len(decoded) != 1 or decoded[0].raw_packet != packet or parser.buffered_bytes:
            self.issue('imu_raw_packet_invalid', raw_key)
            stream.counts['raw_packet_invalid'] += 1
            return
        raw = decoded[0]
        stream.counts['raw_packet_valid'] += 1
        if raw.tid != int(message.tid):
            self.issue('imu_raw_tid_mismatch', raw_key)
        expected = raw.imu_dict(quaternion_norm_tolerance=0.001)
        if expected['orientation_valid']:
            expected['orientation'] = tuple(x/expected['orientation_norm'] for x in expected['orientation'])
        for name in ('orientation', 'angular_velocity', 'linear_acceleration'):
            valid = bool(getattr(message, name+'_valid'))
            want = expected['field_validity'][name]
            stream.counts[name+'_valid' if valid else name+'_absent'] += 1
            if valid != want:
                self.issue('imu_field_validity_mismatch', raw_key+': '+name)
            covariance = getattr(message.imu, name+'_covariance')
            if not valid and (not covariance or covariance[0] != -1):
                self.issue('imu_absent_field_covariance_not_minus_one', raw_key+': '+name)
            if valid:
                value = getattr(message.imu, name)
                values = [value.x, value.y, value.z] + ([value.w] if name == 'orientation' else [])
                if not all(math.isfinite(x) for x in values):
                    self.issue('imu_nonfinite_field', raw_key+': '+name)
                    continue
                if want and not np.allclose(values, expected[name], rtol=1e-6, atol=1e-7):
                    self.issue('imu_raw_decoded_field_mismatch', raw_key+': '+name)
                for axis, value in zip('xyzw', values):
                    stream.imu_values[name+'_'+axis].append(value)
                stream.imu_values[name+'_norm'].append(math.sqrt(sum(x*x for x in values)))
        for name in ('sample_timestamp', 'dataready_timestamp'):
            valid = bool(getattr(message, name+'_valid'))
            original = raw.fields.get(name+'_raw')
            if valid != (original is not None):
                self.issue('imu_timestamp_presence_mismatch', raw_key+': '+name)
            if valid:
                value = int(getattr(message, name+'_raw'))
                stream.device_raw[name+':'+message.device_timestamp_unit].append(value)
                if value != original:
                    self.issue('imu_raw_timestamp_mismatch', raw_key+': '+name)
        if message.imu.header.frame_id != message.header.frame_id or stamp_ns(message.imu.header.stamp) != stamp_ns(message.header.stamp):
            self.issue('imu_envelope_header_mismatch', raw_key)

    def finish(self):
        required = [LEFT_TOPIC, RIGHT_TOPIC] if self.mode == 'dual' else [LEFT_TOPIC if self.mode == 'single_left' else RIGHT_TOPIC]
        for topic in required + ([IMU_TOPIC] if self.require_h30 else []):
            if not self.observed[topic]:
                self.issue('required_source_absent', topic)
        overlap = self.identities['left'] & self.identities['right']
        if overlap:
            self.issue('left_right_sensor_identity_overlap', sorted(overlap))
        streams = [stream.finish() for stream in self.streams.values()]
        for result in streams:
            context = result['sensor_id']+'/'+result['stream_epoch']
            sequence = result['sequence']
            for key in ('duplicates', 'backward_steps', 'forward_gap_events'):
                if sequence[key]:
                    self.issue('sequence_'+key, context+': '+str(sequence[key]))
            for clock in ('bag_record_time', 'host_receive_time', 'host_monotonic_time', 'device_components_ns', 'declared_common_time'):
                if result[clock].get('backward_intervals'):
                    self.issue(clock+'_backward', context)
            for label, clock in result['device_raw_clocks'].items():
                if clock.get('backward_intervals'):
                    self.issue('raw_device_time_backward_or_wrap', context+': '+label)
        return {'observed_envelope_topics': dict(self.observed), 'streams': streams,
                'h30_recorded': bool(self.observed[IMU_TOPIC]),
                'warnings': (['H30 source topic has zero records; use --require-h30 to require it'] if not self.observed[IMU_TOPIC] and not self.require_h30 else []),
                'identity_independence': {'left_ids': sorted(self.identities['left']),
                    'right_ids': sorted(self.identities['right']), 'overlap': sorted(overlap),
                    'raw_key_collisions': self.issues['cross_topic_raw_key_collision'],
                    'equal_cloud_payload_hashes_across_sides': len(self.content_seen['left'] & self.content_seen['right']),
                    'note': 'equal cloud hashes are descriptive; equal sequence numbers across different sensors are permitted'},
                'issues': dict(self.issues), 'issue_examples': dict(self.issue_examples)}


def audit_bag(arguments):
    root = Path(arguments.project_root).resolve(strict=True)
    bag = within(arguments.bag, root/'data'/'bags')
    if not bag.is_dir():
        raise ValueError('bag must be a directory containing finalized metadata.yaml')
    metadata_path = within(bag/'metadata.yaml', bag)
    before = {str(metadata_path): fingerprint(metadata_path)}
    metadata_bytes = metadata_path.read_bytes()
    info = yaml.safe_load(metadata_bytes)['rosbag2_bagfile_information']
    if info.get('storage_identifier') != 'sqlite3':
        raise ValueError('only reviewed sqlite3 rosbag storage is supported; no conversion is attempted')
    if info.get('compression_format') or info.get('compression_mode') not in (None, '', 'none'):
        raise ValueError('compressed bag is not supported by this direct read-only auditor')
    relative = info.get('relative_file_paths', [])
    if not relative or len(set(relative)) != len(relative):
        raise ValueError('missing/duplicate finalized metadata file list')
    databases = []
    for value in relative:
        path = Path(value)
        if path.is_absolute() or '..' in path.parts:
            raise ValueError('unsafe database path in metadata')
        database = within(bag/path, bag)
        if database.suffix != '.db3':
            raise ValueError('unexpected SQLite filename suffix')
        databases.append(database)
        before[str(database)] = fingerprint(database)
        for suffix in ('-wal', '-shm', '-journal'):
            sidecar = Path(str(database)+suffix)
            if sidecar.exists() and sidecar.stat().st_size:
                raise ValueError(f'active/recovery SQLite sidecar present: {sidecar.name}; stop recorder first')
    if set(bag.glob('*.db3')) != set(databases):
        raise ValueError('database files do not match finalized metadata list')
    metadata_topics = {}
    for entry in info.get('topics_with_message_count', []):
        topic = entry['topic_metadata']
        if topic['name'] in metadata_topics:
            raise ValueError('duplicate topic in metadata')
        metadata_topics[topic['name']] = {'type': topic['type'], 'serialization_format': topic.get('serialization_format'),
                                        'count': int(entry['message_count'])}
    connections, iterators, totals, db_reports = [], [], Counter(), []
    topic_types = {}
    review = Audit({'left': arguments.expected_left, 'right': arguments.expected_right,
                    'imu': arguments.expected_imu}, arguments.mode, arguments.require_h30)
    try:
        for path in databases:
            connection = sqlite3.connect(path.as_uri()+'?mode=ro&immutable=1', uri=True)
            connection.execute('PRAGMA query_only=ON')
            connections.append(connection)
            integrity = [row[0] for row in connection.execute('PRAGMA quick_check')]
            if integrity != ['ok']:
                raise ValueError(f'SQLite quick_check failed: {path.name}: {integrity}')
            topics = {int(row[0]): row[1:] for row in connection.execute('SELECT id,name,type,serialization_format FROM topics')}
            for topic_id, (name, type_name, fmt) in topics.items():
                if name in topic_types and topic_types[name] != type_name:
                    raise ValueError('topic type changed between database splits')
                topic_types[name] = type_name
                if name in KNOWN_TOPICS and (KNOWN_TOPICS[name] != type_name or fmt != 'cdr'):
                    raise ValueError(f'authoritative topic type/serialization mismatch: {name}')
            count = 0
            for topic_id, number in connection.execute('SELECT topic_id,COUNT(*) FROM messages GROUP BY topic_id'):
                if topic_id not in topics:
                    raise ValueError('message references an absent topic row')
                totals[topics[topic_id][0]] += number
                count += number
            db_reports.append({'file': path.name, 'messages': count, 'quick_check': 'ok'})
            known_ids = [key for key, value in topics.items() if value[0] in KNOWN_TOPICS]
            if known_ids:
                placeholders = ','.join('?' for _ in known_ids)
                cursor = connection.execute('SELECT timestamp,id,topic_id,data FROM messages WHERE topic_id IN ('+
                                            placeholders+') ORDER BY id', known_ids)
                # Bind topics separately for each lazy iterator.
                def rows(cursor=cursor, topics=topics):
                    for stamp, row_id, topic_id, data in cursor:
                        yield int(stamp), int(row_id), topics[topic_id][0], data
                iterators.append(rows())
        if sum(totals.values()) != int(info.get('message_count', -1)):
            review.issue('metadata_total_count_mismatch', str(dict(totals)))
        for name in set(metadata_topics) | set(totals):
            meta = metadata_topics.get(name)
            if meta is None or meta['count'] != totals[name] or (name in topic_types and meta['type'] != topic_types[name]):
                review.issue('metadata_topic_count_or_type_mismatch', name)
        source_count = sum(totals[name] for name in KNOWN_TOPICS)
        if source_count > arguments.max_envelopes:
            raise ValueError(f'{source_count} envelopes exceeds explicit memory bound {arguments.max_envelopes}')
        from rclpy.serialization import deserialize_message
        from wc_interfaces.msg import H30Frame, SourceFrame
        # Preserve recorder insertion order within each finalized metadata split.
        # Sorting by timestamp would conceal clock regressions and sequence loss.
        for stamp, row_id, topic, data in itertools.chain.from_iterable(iterators):
            try:
                message = deserialize_message(data, H30Frame if topic == IMU_TOPIC else SourceFrame)
                review.consume(topic, message, stamp)
            except (ValueError, TypeError, AttributeError, RuntimeError, IndexError) as error:
                review.issue('envelope_deserialization_or_validation_error', f'{topic} row={row_id}: {error}')
    finally:
        for connection in connections:
            connection.close()
    sidecars_after = [str(path)+suffix for path in databases for suffix in ('-wal', '-shm', '-journal')
                      if Path(str(path)+suffix).exists() and Path(str(path)+suffix).stat().st_size]
    unchanged = all(fingerprint(Path(path)) == initial for path, initial in before.items())
    unchanged = unchanged and not sidecars_after and set(bag.glob('*.db3')) == set(databases)
    if not unchanged:
        review.issue('bag_changed_during_audit', 'result is incomplete; stop the recorder before another report')
    result = review.finish()
    manifest_path = Path(arguments.manifest) if arguments.manifest else root/'.phase1_runtime'/'sessions'/bag.name/'record'/'manifest.json'
    managed = None
    if manifest_path.exists():
        manifest_path = within(manifest_path, root)
        manifest = json.loads(manifest_path.read_text())
        managed = {key: manifest.get(key) for key in ('session_id', 'role', 'state', 'exit_code', 'stop_reason', 'ended_ns')}
        if managed.get('state') == 'RUNNING':
            review.issue('supervisor_still_running', str(manifest_path))
        elif managed.get('state') != 'STOPPED' or managed.get('exit_code') != 0:
            review.issue('supervisor_not_successfully_stopped', str(managed))
        result['issues'], result['issue_examples'] = dict(review.issues), dict(review.issue_examples)
    closure_ok = unchanged and not any(key.startswith('metadata_') or key.startswith('supervisor_') for key in review.issues)
    result.update(schema_version=1, audit_kind='RECORDED_SENSOR_DATA_ONLY', generated_ns=time.time_ns(),
        bag=str(bag), metadata_sha256=hashlib.sha256(metadata_bytes).hexdigest(),
        status='DATA_REVIEW_OK' if not review.issues else 'DATA_REVIEW_ISSUES',
        storage={'storage_id': 'sqlite3', 'opened_read_only_immutable': True, 'databases': db_reports,
            'metadata_message_count': int(info.get('message_count', -1)), 'actual_message_count': sum(totals.values()),
            'metadata_topics': metadata_topics, 'actual_topic_counts': dict(totals),
            'files_unchanged_during_read': unchanged, 'file_fingerprints': before},
        closure={'metadata_present': True, 'nonempty_wal_shm_journal_absent': not sidecars_after,
            'consistent_with_closed_bag': closure_ok, 'managed_session': managed,
            'graceful_recorder_exit_independently_proven': False,
            'note': 'metadata/count/SQLite consistency is checked; this is not proof of a particular recorder shutdown signal'},
        acceptance={'common_sample_synchronization': 'NOT_ESTABLISHED', 'static_hardware_map': 'NOT_TESTED',
            'dynamic_hardware_map': 'NOT_TESTED', 'loop_closure': 'NOT_TESTED', 'map_review': 'NOT_TESTED',
            'actual_physical_origin_of_bytes': 'REQUIRES_SESSION_ACQUISITION_EVIDENCE',
            'note': 'Arrival-only receive intervals, coordinate ranges and raw-message validity do not validate fusion or maps.'})
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--project-root', type=Path, default=Path('/home/nvidia/wheelchair'))
    parser.add_argument('--mode', choices=('dual', 'single_left', 'single_right'), default='dual')
    parser.add_argument('--require-h30', action='store_true')
    parser.add_argument('--expected-left', default='XTM60B00000000000012')
    parser.add_argument('--expected-right', default='XTM60B00000000000013')
    parser.add_argument('--expected-imu', default=None)
    parser.add_argument('--manifest', type=Path)
    parser.add_argument('--max-envelopes', type=int, default=500000)
    arguments = parser.parse_args(argv)
    if arguments.max_envelopes < 1:
        parser.error('--max-envelopes must be positive')
    output = arguments.output.absolute()
    root = arguments.project_root.resolve(strict=True)
    within(output.parent, root)
    if output.exists() or output.is_symlink():
        parser.error('output already exists; report overwrite is forbidden')
    if output.resolve().is_relative_to((root/'data'/'bags').resolve()):
        parser.error('output must be outside the input bag area')
    try:
        report = audit_bag(arguments)
    except Exception as error:
        report = {'schema_version': 1, 'audit_kind': 'RECORDED_SENSOR_DATA_ONLY', 'status': 'INCOMPLETE',
                  'bag': str(arguments.bag), 'generated_ns': time.time_ns(),
                  'error': f'{type(error).__name__}: {error}',
                  'acceptance': {'common_sample_synchronization': 'NOT_ESTABLISHED',
                                 'static_hardware_map': 'NOT_TESTED', 'dynamic_hardware_map': 'NOT_TESTED'}}
    # Exclusive creation is the only persistent write; JSON never emits NaN.
    with output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'status': report['status'], 'output': str(output),
                      'issues': report.get('issues', {}), 'error': report.get('error')}, ensure_ascii=False))
    return 0 if report['status'] == 'DATA_REVIEW_OK' else (2 if report['status'] == 'INCOMPLETE' else 1)


if __name__ == '__main__':
    raise SystemExit(main())
