#!/usr/bin/env python3
"""Actual Humble replay integrity; no SLAM, sensor/serial I/O, TF or navigation.

Creates one ros2 bag play child in an explicit independent domain. The player
starts paused; matched subscriptions and its private Resume service gate play.
Only a new summary JSON is persisted. Bag bytes must remain unchanged.

Reviewed Humble APIs:
https://github.com/ros2/rosbag2/blob/humble/ros2bag/ros2bag/verb/play.py
https://github.com/ros2/rosbag2/blob/humble/rosbag2_transport/src/rosbag2_transport/player.cpp
https://github.com/ros2/rosbag2/blob/humble/rosbag2_interfaces/srv/Resume.srv
"""

from __future__ import annotations

import argparse
import array
from collections import Counter, deque
import fcntl
import gc
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import stat
import struct
import subprocess
import sys
import threading
import time
import uuid


ROOT = Path('/home/nvidia/wheelchair')
LEFT = '/wc_mapping/lidar_left/source_frame'
RIGHT = '/wc_mapping/lidar_right/source_frame'
IMU = '/wc_mapping/imu/source_frame'
AUTHORITATIVE = {LEFT: 'wc_interfaces/msg/SourceFrame', RIGHT: 'wc_interfaces/msg/SourceFrame',
                 IMU: 'wc_interfaces/msg/H30Frame'}
FORBIDDEN = {'/tf', '/tf_static', '/cmd_vel', '/odom', '/wc_mapping/odom', '/clock'}
SCOPE = {'mapping': 'NOT_TESTED', 'loop_closure': 'NOT_TESTED', 'synchronization': 'NOT_ESTABLISHED',
         'dynamic_hardware': 'NOT_TESTED', 'navigation_validated': False,
         'physical_origin_of_recorded_bytes': 'REQUIRES_SEPARATE_ACQUISITION_EVIDENCE'}


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def check_deadline(deadline):
    if time.monotonic() > deadline:
        raise TimeoutError('bounded real-bag integrity test timed out')


def scoped(path, parent, *, existing=True):
    path = Path(path).absolute()
    parent = Path(parent).resolve(strict=True)
    resolved = path.resolve(strict=existing)
    require(resolved.is_relative_to(parent), 'path outside the allowed project area')
    current = path
    while current != current.parent:
        require(not current.is_symlink(), 'symlink in evidence/input path is not accepted')
        current = current.parent
    return resolved


def bag_hashes(bag, deadline, max_bytes):
    result, total = {}, 0
    paths = sorted(bag.rglob('*'))
    require(len(paths) <= 256, 'bag exceeds the explicit file-count bound')
    for path in paths:
        check_deadline(deadline)
        require(not path.is_symlink(), 'bag contains a symlink')
        if path.is_dir():
            continue
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
        with os.fdopen(fd, 'rb') as stream:
            before = os.fstat(stream.fileno())
            require(stat.S_ISREG(before.st_mode), 'bag contains a nonregular file')
            total += before.st_size
            require(total <= max_bytes, 'bag exceeds the explicit byte bound')
            digest = hashlib.sha256()
            while True:
                check_deadline(deadline)
                data = stream.read(1024*1024)
                if not data:
                    break
                digest.update(data)
            after = os.fstat(stream.fileno())
            require((before.st_ino, before.st_size, before.st_mtime_ns) ==
                    (after.st_ino, after.st_size, after.st_mtime_ns), 'bag changed while hashing')
        require(not (path.name.endswith(('-wal', '-shm', '-journal')) and before.st_size),
                'nonempty SQLite sidecar exists; recorder must close before replay')
        result[str(path.relative_to(bag))] = {'sha256': digest.hexdigest(), 'size': before.st_size}
    require('metadata.yaml' in result, 'closed metadata.yaml is required')
    return result


def semantic(value):
    """Deterministic field-level encoding, independent of CDR padding bytes."""
    if hasattr(value, 'get_fields_and_field_types'):
        return {name: semantic(getattr(value, name)) for name in value.get_fields_and_field_types()}
    if isinstance(value, (bytes, bytearray, memoryview)):
        return {'bytes': len(value), 'sha256': hashlib.sha256(value).hexdigest()}
    if isinstance(value, array.array):
        return {'array_type': value.typecode, 'length': len(value),
                'sha256': hashlib.sha256(value.tobytes()).hexdigest()}
    if hasattr(value, 'dtype') and hasattr(value, 'tobytes'):
        return {'dtype': str(value.dtype), 'shape': list(value.shape),
                'sha256': hashlib.sha256(value.tobytes()).hexdigest()}
    if isinstance(value, (list, tuple)):
        return [semantic(item) for item in value]
    if isinstance(value, float):
        return {'float64_bits': struct.pack('>d', value).hex()}
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    raise TypeError('unsupported ROS field type: '+type(value).__name__)


def signature(topic, message):
    digest = hashlib.sha256(json.dumps(semantic(message), sort_keys=True, separators=(',', ':'),
                                      ensure_ascii=True, allow_nan=False).encode()).hexdigest()
    if topic not in AUTHORITATIVE:
        return digest, None, None
    key = (message.session_id, message.sensor_id, message.stream_epoch, int(message.frame_sequence))
    require(all(key[:3]), 'authoritative frame has an empty identity')
    require('synthetic' not in str(message.time_source).lower(), 'requested real bag contains a declared synthetic source')
    if topic in (LEFT, RIGHT):
        require(message.side == ('left' if topic == LEFT else 'right'), 'SourceFrame side/topic mismatch')
        payload = bytes(message.cloud.data)
    else:
        payload = bytes(message.raw_packet)
    require(bool(payload), 'authoritative raw payload is empty')
    return digest, key, hashlib.sha256(payload).hexdigest()


def metadata_topics(bag):
    import yaml
    info = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    require(info.get('storage_identifier') == 'sqlite3', 'only finalized SQLite rosbag2 is supported')
    require(not info.get('compression_format') and info.get('compression_mode') in (None, '', 'none'),
            'compressed bags require a separately reviewed reader')
    files = info.get('relative_file_paths', [])
    require(bool(files) and len(files) == len(set(files)), 'missing or duplicate metadata database filenames')
    for value in files:
        require(not Path(value).is_absolute() and '..' not in Path(value).parts, 'unsafe database path')
        path = scoped(bag/value, bag)
        require(path.is_file() and path.suffix == '.db3', 'invalid bag database entry')
    require(set(bag.rglob('*.db3')) == {bag/value for value in files}, 'database files differ from metadata')
    result = {}
    for entry in info['topics_with_message_count']:
        topic = entry['topic_metadata']
        name = topic['name']
        require(name not in result, 'duplicate topic in metadata')
        result[name] = {'type': topic['type'], 'count': int(entry['message_count'])}
        require(topic.get('serialization_format') == 'cdr', 'only CDR serialization is supported')
    require(sum(item['count'] for item in result.values()) == int(info['message_count']), 'metadata count mismatch')
    return result


class Integrity:
    def __init__(self, topics, limit):
        self.topics, self.limit = topics, limit
        self.expected = {name: Counter() for name in topics}
        self.remaining = {}
        self.keys = {name: {} for name in AUTHORITATIVE}
        self.seen = {name: set() for name in AUTHORITATIVE}
        self.received, self.total_indexed = Counter(), 0
        self.received_total = 0

    def index(self, topic, message):
        self.total_indexed += 1
        require(self.total_indexed <= self.limit, 'input message bound exceeded')
        digest, key, payload_hash = signature(topic, message)
        self.expected[topic][digest] += 1
        if key is not None:
            require(key not in self.keys[topic], 'duplicate authoritative raw key in input bag')
            self.keys[topic][key] = (digest, payload_hash)

    def freeze(self):
        for topic, value in self.topics.items():
            require(sum(self.expected[topic].values()) == value['count'], 'rosbag2 enumeration/metadata count mismatch: '+topic)
        for topic in AUTHORITATIVE:
            require(bool(self.keys[topic]), 'required authoritative topic has no frames: '+topic)
        left_ids = {key[1] for key in self.keys[LEFT]}
        right_ids = {key[1] for key in self.keys[RIGHT]}
        require(not left_ids.intersection(right_ids), 'left and right share a sensor identity')
        self.remaining = {name: counts.copy() for name, counts in self.expected.items()}

    def consume(self, topic, message):
        self.received_total += 1
        require(self.received_total <= self.limit, 'received-message bound exceeded')
        self.received[topic] += 1
        require(self.received[topic] <= self.topics[topic]['count'], 'unexpected extra message: '+topic)
        digest, key, payload_hash = signature(topic, message)
        require(self.remaining[topic][digest] > 0, 'unrecorded/duplicate/changed semantic payload: '+topic)
        self.remaining[topic][digest] -= 1
        if key is not None:
            require(key not in self.seen[topic], 'duplicate received authoritative key: '+topic)
            require(self.keys[topic].get(key) == (digest, payload_hash), 'raw key/cloud/packet identity mismatch: '+topic)
            self.seen[topic].add(key)

    def complete(self):
        return all(self.received[name] == item['count'] for name, item in self.topics.items())

    def summary(self):
        return {name: {'type': item['type'], 'expected_count': item['count'], 'received_count': self.received[name],
            'missing_count': item['count']-self.received[name],
            'expected_unique_keys': len(self.keys[name]) if name in self.keys else None,
            'received_unique_keys': len(self.seen[name]) if name in self.seen else None,
            'identities': sorted({key[1] for key in self.keys[name]}) if name in self.keys else [],
            'semantic_multiset_remaining': sum(self.remaining.get(name, self.expected[name]).values())}
            for name, item in self.topics.items()}


def enumerate_bag(bag, integrity, deadline, max_message_bytes):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    types = {name: get_message(item['type']) for name, item in integrity.topics.items()}
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
        actual = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
        require(all(actual.get(name) == item['type'] for name, item in integrity.topics.items()), 'reader topic types differ from metadata')
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(integrity.topics)))
        while reader.has_next():
            check_deadline(deadline)
            topic, data, _recorded = reader.read_next()
            require(topic in integrity.topics and len(data) <= max_message_bytes, 'reader topic/payload-size bound violated')
            integrity.index(topic, deserialize_message(data, types[topic]))
    finally:
        del reader
        gc.collect()
    integrity.freeze()
    return types


def cleanup_child(child, pidfd):
    if child is None:
        return []
    sent = []
    for signum, seconds in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        if child.poll() is not None:
            break
        try:
            if pidfd is not None:
                signal.pidfd_send_signal(pidfd, signum)
            else:
                # Only possible if opening the pidfd failed immediately after
                # Popen. This is still our unreaped direct child, never a PID
                # discovered by a process name or an unrelated process group.
                child.send_signal(signum)
            sent.append(signal.Signals(signum).name)
        except ProcessLookupError:
            break
        try:
            child.wait(timeout=seconds)
        except subprocess.TimeoutExpired:
            continue
    require(child.poll() is not None, 'owned player failed to exit after bounded cleanup')
    return sent


def replay(arguments, bag, integrity, types, deadline, report):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from rosbag2_interfaces.srv import Resume
    import yaml

    token = uuid.uuid4().hex
    observer_name, player_name = 'wc_replay_check_'+token, 'wc_replay_player_'+token
    node = None
    child = None
    pidfd = None
    config_fd = None
    lock = None
    output_tail = deque(maxlen=8)
    reader_thread = None
    os.environ['ROS_DOMAIN_ID'] = str(arguments.ros_domain_id)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    rclpy.init(args=[])
    try:
        lock_path = ROOT/'.phase1_runtime'/'locks'/f'domain-{arguments.ros_domain_id}-source.lock'
        lock_path.parent.mkdir(parents=True, exist_ok=True)
        lock = lock_path.open('a+')
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        node = Node(observer_name)
        until = time.monotonic()+2
        while time.monotonic() < until:
            check_deadline(deadline)
            rclpy.spin_once(node, timeout_sec=.05)
        require(set(node.get_node_names_and_namespaces()) == {(observer_name, '/')}, 'requested test ROS domain is occupied')

        def depth(name):
            return 64 if name in (LEFT, RIGHT) else 2048

        subscriptions = []
        for topic, message_type in types.items():
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth(topic),
                             reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
            subscriptions.append(node.create_subscription(message_type, topic,
                lambda message, topic=topic: integrity.consume(topic, message), qos))
        # The fd is the only QoS file; it exists in RAM, not in the input bag or
        # as a persistent report sidecar. ros2bag opens the inherited fd path.
        config_fd = os.memfd_create('wc_real_replay_qos', os.MFD_CLOEXEC)
        overrides = {topic: {'history': 'keep_last', 'depth': depth(topic), 'reliability': 'reliable',
                             'durability': 'volatile'} for topic in types}
        os.write(config_fd, yaml.safe_dump(overrides).encode())
        os.lseek(config_fd, 0, os.SEEK_SET)
        command = ['ros2', 'bag', 'play', str(bag), '--storage', 'sqlite3', '--start-paused',
            '--disable-keyboard-controls', '--read-ahead-queue-size', '100', '--rate', str(arguments.rate),
            '--wait-for-all-acked', '5000', '--qos-profile-overrides-path', f'/proc/self/fd/{config_fd}',
            '--remap', '__node:='+player_name, '--topics', *report['replay_whitelist']]
        child = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 start_new_session=True, pass_fds=(config_fd, lock.fileno()), env=os.environ.copy())
        pidfd = os.pidfd_open(child.pid)
        report['player'] = {'pid': child.pid, 'node_name': player_name, 'argv': command, 'only_child_process': True}

        def drain():
            while True:
                data = child.stdout.read(4096)
                if not data:
                    break
                output_tail.append(data)

        reader_thread = threading.Thread(target=drain, daemon=True)
        reader_thread.start()
        client = node.create_client(Resume, '/'+player_name+'/resume')
        discovered_until = time.monotonic()+arguments.discovery_timeout
        stable_since = None
        while True:
            check_deadline(deadline)
            require(child.poll() is None, 'ros2 bag play exited before discovery/resume')
            require(time.monotonic() < discovered_until, 'player publisher/service discovery timed out')
            rclpy.spin_once(node, timeout_sec=.05)
            require(integrity.received_total == 0, 'player published data before explicit Resume')
            matched = client.service_is_ready()
            for topic in types:
                publishers = node.get_publishers_info_by_topic(topic)
                matched = matched and len(publishers) == 1 and publishers[0].node_name == player_name
                matched = matched and bool(publishers) and publishers[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
                own = node.get_subscriptions_info_by_topic(topic)
                matched = matched and any(item.node_name == observer_name for item in own)
            stable_since = (stable_since or time.monotonic()) if matched else None
            if stable_since is not None and time.monotonic()-stable_since >= 1:
                break
        # Check the exact player's advertised publishers before releasing data.
        advertised = []
        for topic, _ in node.get_topic_names_and_types():
            if any(item.node_name == player_name for item in node.get_publishers_info_by_topic(topic)):
                advertised.append(topic)
        internal = {'/rosout', '/parameter_events', '/events/read_split'}
        require(set(advertised).issubset(set(report['replay_whitelist']) | internal), 'player advertises a non-whitelisted historical/control topic')
        require(not set(advertised).intersection(FORBIDDEN), 'player advertises TF/control/pose/clock')
        report['player']['publishers_before_resume'] = sorted(advertised)
        request = client.call_async(Resume.Request())
        until = min(deadline, time.monotonic()+5)
        while not request.done():
            check_deadline(until)
            rclpy.spin_once(node, timeout_sec=.05)
        require(request.exception() is None, 'player Resume service failed')
        report['subscribers_discovered_before_resume'] = True
        started = time.monotonic()
        exit_seen = None
        while True:
            check_deadline(deadline)
            rclpy.spin_once(node, timeout_sec=.03)
            rc = child.poll()
            if rc is not None:
                exit_seen = exit_seen or time.monotonic()
                require(rc == 0, 'ros2 bag play returned nonzero: '+str(rc))
                if integrity.complete() and time.monotonic()-exit_seen >= .5:
                    break
                require(time.monotonic()-exit_seen <= 5, 'player exited with missing subscription messages')
        require(all(not any(counts.values()) for counts in integrity.remaining.values()), 'semantic multiset did not match')
        require(all(len(integrity.keys[name]) == len(integrity.seen[name]) for name in AUTHORITATIVE), 'authoritative raw key set incomplete')
        report['playback_elapsed_seconds'] = time.monotonic()-started
        report['all_whitelisted_topic_counts_and_semantic_hashes_match'] = True
        report['left_right_h30_unique_keys_and_payload_hashes_match'] = True
    finally:
        try:
            report.setdefault('player', {})['cleanup_signals'] = cleanup_child(child, pidfd)
            if child is not None:
                report['player']['exit_code'] = child.poll()
        finally:
            if reader_thread is not None:
                reader_thread.join(timeout=2)
            report['player_log_tail'] = b''.join(output_tail).decode('utf-8', errors='replace')[-16000:]
            if child is not None and child.stdout is not None:
                child.stdout.close()
            if pidfd is not None:
                os.close(pidfd)
            if config_fd is not None:
                os.close(config_fd)
            if lock is not None:
                lock.close()
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--ros-domain-id', required=True, type=int)
    parser.add_argument('--timeout', type=float, default=180)
    parser.add_argument('--discovery-timeout', type=float, default=20)
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--max-messages', type=int, default=50000)
    parser.add_argument('--max-message-bytes', type=int, default=16*1024*1024)
    parser.add_argument('--max-bag-bytes', type=int, default=8*1024**3)
    args = parser.parse_args(argv)
    require(sys.platform == 'linux', 'run only on the confirmed Orin target')
    from wc_runtime.cli import TOPICS, target
    target()
    require(1 <= args.ros_domain_id <= 232 and args.ros_domain_id != 83, 'explicit independent domain is required; domain 83 is forbidden')
    require(all(math.isfinite(value) and value > 0 for value in (args.timeout, args.discovery_timeout, args.rate)), 'timeouts/rate must be positive finite')
    require(.1 <= args.rate <= 2 and 15 <= args.timeout <= 900 and args.discovery_timeout <= 60, 'rate/timeouts exceed test bounds')
    require(1 <= args.max_messages <= 200000 and 1 <= args.max_message_bytes <= 64*1024*1024 and 1 <= args.max_bag_bytes <= 32*1024**3,
            'message/file bounds exceed supported test limits')
    require(TOPICS and len(TOPICS) == len(set(TOPICS)) and not set(TOPICS).intersection(FORBIDDEN), 'unsafe runtime replay whitelist')
    require(all(topic.startswith('/wc_mapping/') and '/_action/' not in topic for topic in TOPICS), 'unexpected replay whitelist namespace/action')
    bag = scoped(args.bag, ROOT/'data'/'bags')
    require(bag.is_dir(), 'bag input must be a finalized directory')
    output = scoped(args.output, ROOT, existing=False)
    require(output.parent.is_dir() and not output.is_relative_to(ROOT/'data'/'bags'), 'new report must have an existing project parent outside bags')
    require(not output.exists(), 'report already exists; do not overwrite evidence')
    deadline = time.monotonic()+args.timeout
    report = {'schema_version': 1, 'status': 'INCOMPLETE', 'level': 'REAL_BAG',
              'test_kind': 'REAL_BAG_TRANSPORT_INTEGRITY_ONLY', 'bag': str(bag), 'output': str(output),
              'ros_domain_id': args.ros_domain_id, 'replay_whitelist': list(TOPICS), 'rate': args.rate,
              'acceptance_scope': SCOPE, 'source_script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'started_ns': time.time_ns(), 'bag_unchanged': None,
              'transport_note': 'Replay QoS is explicitly reliable; this does not measure original live acquisition reliability',
              'bounds': {'timeout_seconds': args.timeout, 'discovery_seconds': args.discovery_timeout,
                         'max_messages': args.max_messages, 'max_message_bytes': args.max_message_bytes,
                         'max_bag_bytes': args.max_bag_bytes, 'player_read_ahead': 100,
                         'lidar_subscription_depth': 64, 'other_subscription_depth': 2048}}
    integrity = None
    before = None

    def interrupted(signum, frame):
        raise KeyboardInterrupt('replay test interrupted')

    signal.signal(signal.SIGTERM, interrupted)
    with output.open('x', encoding='utf-8') as stream:
        try:
            managed = ROOT/'.phase1_runtime'/'sessions'/bag.name/'record'/'manifest.json'
            if managed.exists():
                state = json.loads(scoped(managed, ROOT).read_text())
                report['recording_manifest'] = {key: state.get(key) for key in ('session_id', 'state', 'exit_code', 'stop_reason')}
                require(state.get('state') not in ('STARTING', 'RUNNING'), 'managed recorder is still active')
            before = bag_hashes(bag, deadline, args.max_bag_bytes)
            metadata = metadata_topics(bag)
            selected = {name: metadata[name] for name in TOPICS if name in metadata}
            require(all(name in selected and selected[name]['type'] == value for name, value in AUTHORITATIVE.items()),
                    'bag must contain both SourceFrame topics and H30Frame with exact types')
            require(sum(item['count'] for item in selected.values()) <= args.max_messages, 'metadata exceeds message bound')
            report['excluded_recorded_topics'] = sorted(set(metadata)-set(TOPICS))
            integrity = Integrity(selected, args.max_messages)
            types = enumerate_bag(bag, integrity, deadline, args.max_message_bytes)
            require(before == bag_hashes(bag, deadline, args.max_bag_bytes), 'bag changed during read-only rosbag2 enumeration')
            replay(args, bag, integrity, types, deadline, report)
            report['status'] = 'PASS'
        except (Exception, KeyboardInterrupt) as error:
            report['status'] = 'FAIL'
            report['error'] = type(error).__name__+': '+str(error)
        finally:
            if integrity is not None:
                report['topics'] = integrity.summary()
                report['indexed_messages'] = integrity.total_indexed
                report['received_messages'] = integrity.received_total
            if before is not None:
                report['bag_files_before'] = before
                try:
                    # Cleanup/hash verification has a separate bounded grace
                    # after the main timeout; a timeout can never become PASS.
                    after = bag_hashes(bag, time.monotonic()+60, args.max_bag_bytes)
                    report['bag_files_after'] = after
                    report['bag_unchanged'] = before == after
                except Exception as error:
                    report['bag_hash_error'] = type(error).__name__+': '+str(error)
                    report['bag_unchanged'] = None
                if report['bag_unchanged'] is not True:
                    report['status'] = 'FAIL'
            report['ended_ns'] = time.time_ns()
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
    print(json.dumps({'status': report['status'], 'test_kind': report['test_kind'], 'output': str(output),
                      'bag_unchanged': report['bag_unchanged'], 'topics': report.get('topics'), 'error': report.get('error')},
                     ensure_ascii=False))
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
