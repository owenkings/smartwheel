#!/usr/bin/env python3
"""Bounded SYNTHETIC String transport probe; no devices, images or control topics.

Source the normal ROS environment, then explicitly set ROS_DOMAIN_ID=88.
The default run starts three independent publishers, starts a subscriber process
5 seconds later, and observes it for 15 seconds without stopping on message gaps.
--camera-load adds four independent 8 Hz String publishers and subscriptions.
Only JSON metadata is written to stdout; payload padding is never archived.
PASS means the observation completed and each primary stream was received. It
does not mean all gaps/latencies meet any real sensor or motion requirement.
"""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time
import uuid


DOMAIN = '88'
SCHEMA = 'wc_synthetic_ros_transport_v1'
PRIMARY = {
    'fast': {'hz': 200., 'padding_bytes': 0, 'reliable': False, 'depth': 100},
    'medium': {'hz': 10., 'padding_bytes': 0, 'reliable': True, 'depth': 100},
    'cloud_load': {'hz': 5., 'padding_bytes': 150000, 'reliable': True, 'depth': 100},
}
SUBSCRIBER_DELAY_S = 5.
OBSERVATION_S = 15.
CONTROLLER_LIMIT_S = 25.
SAMPLE_LIMIT = 5000


def streams(camera_load):
    result = dict(PRIMARY)
    if camera_load:
        result.update({f'camera_load_{index}': {
            'hz': 8., 'padding_bytes': 230400, 'reliable': False, 'depth': 1,
        } for index in range(4)})
    return result


def emit(value):
    print(json.dumps(value, separators=(',', ':'), allow_nan=False), flush=True)


class StreamStats:
    """Preserve sender seq/time and local receive time; never replace a stamp."""
    def __init__(self, name, ready_ns):
        self.name, self.ready_ns = name, ready_ns
        self.messages = self.invalid = self.missing_sequences = self.nonincreasing_sequences = 0
        self.first_seq = self.last_seq = self.first_receive_ns = self.last_receive_ns = None
        self.first_published_ns = self.last_published_ns = None
        self.max_gap_s = self.max_callback_s = 0.
        self.gaps_over_300ms = 0
        self.recovered_gaps = []
        self.age_min_s = self.age_max_s = None
        self.age_sum_s = 0.
        self.message_bytes_min = self.message_bytes_max = None
        self.samples = []

    def accept(self, value, receive_ns, message_bytes, run_id):
        sequence, published = value.get('sequence'), value.get('published_monotonic_ns')
        if value.get('schema') != SCHEMA or value.get('run_id') != run_id or value.get('stream') != self.name or \
                type(sequence) is not int or sequence < 1 or type(published) is not int or \
                published <= 0 or published > receive_ns:
            self.invalid += 1
            return
        age = (receive_ns-published)/1e9
        if self.last_receive_ns is not None:
            gap = (receive_ns-self.last_receive_ns)/1e9
            self.max_gap_s = max(self.max_gap_s, gap)
            self.gaps_over_300ms += gap > .3
            if gap > .3 and len(self.recovered_gaps) < 64:
                self.recovered_gaps.append({'previous_sequence': self.last_seq,
                                           'resumed_sequence': sequence, 'gap_s': gap})
            self.missing_sequences += max(0, sequence-self.last_seq-1)
            self.nonincreasing_sequences += sequence <= self.last_seq
        else:
            gap = None
            self.first_seq, self.first_receive_ns, self.first_published_ns = sequence, receive_ns, published
        self.messages += 1
        self.last_seq, self.last_receive_ns, self.last_published_ns = sequence, receive_ns, published
        self.age_sum_s += age
        self.age_min_s = age if self.age_min_s is None else min(self.age_min_s, age)
        self.age_max_s = age if self.age_max_s is None else max(self.age_max_s, age)
        self.message_bytes_min = message_bytes if self.message_bytes_min is None else min(self.message_bytes_min, message_bytes)
        self.message_bytes_max = message_bytes if self.message_bytes_max is None else max(self.message_bytes_max, message_bytes)
        if len(self.samples) < SAMPLE_LIMIT:
            self.samples.append([sequence, published, receive_ns, age, gap])

    def report(self, end_ns):
        elapsed = 0 if self.first_receive_ns is None else (self.last_receive_ns-self.first_receive_ns)/1e9
        return {**{key: value for key, value in vars(self).items() if key not in ('ready_ns', 'age_sum_s')},
                'first_receive_delay_s': None if self.first_receive_ns is None else (self.first_receive_ns-self.ready_ns)/1e9,
                'tail_silence_s': (end_ns-(self.last_receive_ns or self.ready_ns))/1e9,
                'silent_over_300ms_at_end': (end_ns-(self.last_receive_ns or self.ready_ns)) > 300_000_000,
                'observed_hz_between_first_last': (self.messages-1)/elapsed if elapsed > 0 else None,
                'mean_age_s': self.age_sum_s/self.messages if self.messages else None,
                'samples_truncated': self.messages > len(self.samples),
                'sample_columns': ['sequence', 'published_monotonic_ns', 'received_monotonic_ns', 'age_s', 'gap_s']}


def child(args):
    # ROS is imported only in explicitly isolated child roles, never by --help
    # or by the controller. No application driver module is imported.
    stopped = [False]
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda *_: stopped.__setitem__(0, True))
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('synthetic_'+args.role+'_'+(args.stream or 'all')+'_'+args.run_id)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    selected = streams(args.camera_load)
    prefix = '/wc_synthetic_transport/'+args.run_id+'/'

    def qos(spec):
        return QoSProfile(depth=spec['depth'],
                          reliability=ReliabilityPolicy.RELIABLE if spec['reliable'] else ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE)

    ready_ns = time.monotonic_ns()
    stats = {name: StreamStats(name, ready_ns) for name in selected}
    sent = [0]
    publisher_time = {'first_published_ns': None, 'last_published_ns': None, 'max_publish_call_s': 0.}
    subscriptions = []
    if args.role == 'publisher':
        spec = selected[args.stream]
        publisher = node.create_publisher(String, prefix+args.stream, qos(spec))
        padding = 'x'*spec['padding_bytes']

        def publish():
            timestamp = time.monotonic_ns()
            sequence = sent[0]+1
            message = String(data=json.dumps({'schema': SCHEMA, 'run_id': args.run_id,
                'stream': args.stream, 'sequence': sequence, 'published_monotonic_ns': timestamp,
                'padding': padding}, separators=(',', ':')))
            before = time.monotonic_ns()
            publisher.publish(message)
            publisher_time['max_publish_call_s'] = max(publisher_time['max_publish_call_s'],
                                                       (time.monotonic_ns()-before)/1e9)
            sent[0] = sequence
            publisher_time['first_published_ns'] = publisher_time['first_published_ns'] or timestamp
            publisher_time['last_published_ns'] = timestamp

        timer = node.create_timer(1./spec['hz'], publish)
        deadline_ns = args.deadline_ns
    else:
        def receive(name, message):
            receive_ns = time.monotonic_ns()
            row = stats[name]
            try:
                value = json.loads(message.data)
                if not isinstance(value, dict):
                    raise ValueError('Non-object payload')
                row.accept(value, receive_ns, len(message.data), args.run_id)
            except (ValueError, TypeError):
                row.invalid += 1
            finally:
                row.max_callback_s = max(row.max_callback_s, (time.monotonic_ns()-receive_ns)/1e9)

        for name, spec in selected.items():
            subscriptions.append(node.create_subscription(String, prefix+name,
                lambda message, name=name: receive(name, message), qos(spec)))
        # Include discovery delay after creation in the full 15-second window.
        ready_ns = time.monotonic_ns()
        for row in stats.values():
            row.ready_ns = ready_ns
        deadline_ns = ready_ns+int(OBSERVATION_S*1e9)
    emit({'event': 'ready', 'role': args.role, 'stream': args.stream, 'ready_monotonic_ns': ready_ns})
    failure = None
    graph_samples, last_graph_ns = [], 0
    try:
        while not stopped[0] and rclpy.ok() and time.monotonic_ns() < deadline_ns:
            executor.spin_once(timeout_sec=.02)
            now_ns = time.monotonic_ns()
            if now_ns-last_graph_ns >= 500_000_000:
                counts = {args.stream: {'matched_subscriptions': publisher.get_subscription_count()}} \
                    if args.role == 'publisher' else {
                        name: {'graph_publishers': node.count_publishers(prefix+name)} for name in selected}
                graph_samples.append({'monotonic_ns': now_ns, 'streams': counts})
                last_graph_ns = now_ns
    except Exception as error:
        failure = type(error).__name__+': '+str(error)
    finally:
        ended_ns = time.monotonic_ns()
        executor.remove_node(node)
        executor.shutdown(timeout_sec=1.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    result = {'event': 'final', 'role': args.role, 'stream': args.stream,
              'ready_monotonic_ns': ready_ns, 'ended_monotonic_ns': ended_ns,
              'observed_duration_s': (ended_ns-ready_ns)/1e9, 'failure': failure,
              'stopped_by_signal': stopped[0], 'graph_samples_500ms': graph_samples}
    if args.role == 'publisher':
        result.update(published=sent[0], **publisher_time)
    else:
        result.update(streams={name: row.report(ended_ns) for name, row in stats.items()},
                      observation_completed=ended_ns >= deadline_ns and not stopped[0])
    emit(result)
    return 1 if failure else 0


class OwnedProcess:
    def __init__(self, command, name):
        self.name, self.events = name, []
        self.logs = deque(maxlen=8)
        self.process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, text=True, encoding='utf-8', errors='replace')
        self.readers = []
        for pipe, parse in ((self.process.stdout, True), (self.process.stderr, False)):
            reader = threading.Thread(target=self._drain, args=(pipe, parse), daemon=True)
            reader.start()
            self.readers.append(reader)

    def _drain(self, pipe, parse):
        try:
            for line in pipe:
                try:
                    value = json.loads(line) if parse else None
                    if isinstance(value, dict) and value.get('event') in ('ready', 'final') and len(self.events) < 4:
                        self.events.append(value)
                    else:
                        self.logs.append(line[-1000:].rstrip())
                except ValueError:
                    self.logs.append(line[-1000:].rstrip())
        finally:
            pipe.close()

    def report(self):
        return {'pid': self.process.pid, 'returncode': self.process.poll(),
                'events': list(self.events), 'diagnostic_tail': list(self.logs)}


def stop_owned(children):
    # Only Popen handles created by this invocation are signalled. No process
    # names, process groups, devices, network settings or external sessions.
    for number, allowance in ((signal.SIGINT, 2.), (signal.SIGTERM, 1.)):
        for item in children:
            if item.process.poll() is None:
                try:
                    item.process.send_signal(number)
                except ProcessLookupError:
                    pass
        end = time.monotonic()+allowance
        while any(item.process.poll() is None for item in children) and time.monotonic() < end:
            time.sleep(.02)
    for item in children:
        if item.process.poll() is None:
            item.process.kill()
    end = time.monotonic()+1.
    for item in children:
        try:
            item.process.wait(timeout=max(.001, end-time.monotonic()))
        except subprocess.TimeoutExpired:
            pass
    for item in children:
        for reader in item.readers:
            reader.join(timeout=.2)


def controller(args):
    stopped = [False]
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda *_: stopped.__setitem__(0, True))
    run_id = 'r'+uuid.uuid4().hex[:12]
    started_ns = time.monotonic_ns()
    children, subscriber, failure = [], None, None
    command = [sys.executable, '-u', str(Path(__file__).resolve()), '--run-id', run_id]
    if args.camera_load:
        command.append('--camera-load')
    try:
        for name in streams(args.camera_load):
            children.append(OwnedProcess(command+['--role', 'publisher', '--stream', name,
                '--deadline-ns', str(started_ns+int(CONTROLLER_LIMIT_S*1e9))], name))
        while not stopped[0] and (time.monotonic_ns()-started_ns)/1e9 < SUBSCRIBER_DELAY_S:
            time.sleep(.02)
        if not stopped[0]:
            subscriber = OwnedProcess(command+['--role', 'subscriber'], 'subscriber')
            children.append(subscriber)
        while subscriber is not None and subscriber.process.poll() is None and not stopped[0]:
            if (time.monotonic_ns()-started_ns)/1e9 >= CONTROLLER_LIMIT_S:
                failure = 'Controller hard deadline reached'
                break
            time.sleep(.02)
    except Exception as error:
        failure = type(error).__name__+': '+str(error)
    finally:
        stop_owned(children)
    details = {item.name: item.report() for item in children}
    subscriber_final = next((event for event in details.get('subscriber', {}).get('events', [])
                             if event['event'] == 'final'), None)
    good = not failure and not stopped[0] and subscriber_final is not None and \
        subscriber_final.get('observation_completed') is True and \
        all(row['returncode'] == 0 and any(event['event'] == 'final' for event in row['events'])
            for row in details.values()) and \
        all(subscriber_final['streams'][name]['messages'] > 0 and
            subscriber_final['streams'][name]['invalid'] == 0 for name in PRIMARY)
    result = {'validation_level': 'SYNTHETIC_ROS_TRANSPORT', 'status': 'PASS' if good else 'FAIL',
              'pass_meaning': 'Completed observation with all primary streams; inspect recorded gaps and ages separately',
              'domain_id': int(DOMAIN), 'run_id': run_id, 'topic_prefix': '/wc_synthetic_transport/'+run_id+'/',
              'requested_subscriber_delay_s': SUBSCRIBER_DELAY_S, 'requested_observation_s': OBSERVATION_S,
              'elapsed_s': (time.monotonic_ns()-started_ns)/1e9,
              'camera_load': args.camera_load, 'stream_configuration': streams(args.camera_load),
              'failure': failure, 'interrupted': stopped[0], 'processes': details,
              'devices_opened': False, 'images_saved': False, 'control_messages_sent': False,
              'environment': {key: os.environ.get(key) for key in
                              ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY', 'ROS_DISTRO')}}
    emit(result)
    return 0 if good else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera-load', action='store_true')
    parser.add_argument('--role', choices=('controller', 'publisher', 'subscriber'), default='controller', help=argparse.SUPPRESS)
    parser.add_argument('--run-id', help=argparse.SUPPRESS)
    parser.add_argument('--stream', help=argparse.SUPPRESS)
    parser.add_argument('--deadline-ns', type=int, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if os.environ.get('ROS_DOMAIN_ID') != DOMAIN:
        parser.error('Explicit ROS_DOMAIN_ID=88 is required; this script never changes the network configuration')
    if os.name != 'posix':
        parser.error('This owned-process ROS transport check requires the target Linux environment')
    if args.role != 'controller':
        if not args.run_id or not re.fullmatch(r'r[0-9a-f]{12}', args.run_id):
            parser.error('Internal child requires a bounded synthetic run identity')
        if args.role == 'publisher' and (args.stream not in streams(args.camera_load) or
                args.deadline_ns is None or not 0 < args.deadline_ns-time.monotonic_ns() <= 30_000_000_000):
            parser.error('Internal publisher requires an allowlisted stream and bounded deadline')
        return child(args)
    return controller(args)


if __name__ == '__main__':
    raise SystemExit(main())
