"""Ready-before-source recorder, with a bounded writer and per-message index."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import queue
import signal
import threading
import time

from .source_archive import atomic_json, digest


TOPICS = {
    '/wc_mapping/lidar_left/source_frame': 'wc_interfaces/msg/SourceFrame',
    '/wc_mapping/lidar_left/source_frame_filtered': 'wc_interfaces/msg/SourceFrame',
    '/wc_mapping/lidar_right/source_frame': 'wc_interfaces/msg/SourceFrame',
    '/wc_mapping/lidar_right/source_frame_filtered': 'wc_interfaces/msg/SourceFrame',
    '/wc_mapping/imu/source_frame': 'wc_interfaces/msg/H30Frame',
    '/wc_mapping/wheel/feedback_raw': 'std_msgs/msg/String',
}


def topics_for_profile(profile, sides='all'):
    from .capture import source_selection
    selected = source_selection(profile, sides)['selected_sources']
    topics = {topic:kind for topic,kind in TOPICS.items()
              if '/lidar_' not in topic or any('/'+s+'/' in topic for s in selected if s.startswith('lidar_'))}
    if profile == 'all_sensors':
        topics['/wc_mapping/ultrasonic/feedback_raw'] = 'std_msgs/msg/String'
    return topics


def message_index(topic, message):
    if '/lidar_' in topic:
        payload = bytes(message.cloud.data)
        result = dict(source_id=message.sensor_id, stream_epoch=message.stream_epoch,
                      session_id=message.session_id, side=message.side,
                      sequence=message.frame_sequence,
                      representation='filtered' if topic.endswith('_filtered') else 'raw',
                      payload_sha256=hashlib.sha256(payload).hexdigest(),
                      host_monotonic_ns=message.host_monotonic_ns,
                      point_count=message.raw_count, valid_point_count=message.valid_count)
    elif '/imu/' in topic:
        result = dict(source_id=message.sensor_id, stream_epoch=message.stream_epoch,
                      session_id=message.session_id,
                      sequence=message.frame_sequence, representation='packet',
                      payload_sha256=hashlib.sha256(bytes(message.raw_packet)).hexdigest(),
                      host_monotonic_ns=message.host_monotonic_ns)
        from wc_imu.device_time import message_timing_metadata
        result.update(message_timing_metadata(message))
    else:
        value = json.loads(message.data)
        response_sha256 = hashlib.sha256(bytes.fromhex(value['response_hex'])).hexdigest()
        if response_sha256 != value['response_sha256']:
            raise ValueError('response bytes differ from declared source hash')
        result = dict(source_id=value.get('device_id', value.get('source_id', 'ultrasonic')),
                      stream_epoch=value['stream_epoch'], sequence=value['sequence'],
                      representation='transaction', payload_sha256=response_sha256,
                      host_monotonic_ns=value.get('receive_monotonic_ns', value.get('host_monotonic_ns')))
        result.update({key:value[key] for key in ('session_id','status','address','register_words_u16') if key in value})
    return dict(result, topic=topic)


class SourceReadiness:
    """Receipt evidence only, emitted by the recorder without touching devices."""
    def __init__(self, directory, contract):
        self.directory, self.contract = Path(directory), contract
        self.values, self.lidar_representations, self.epochs = {}, {}, {}
        self.imu_timing = None
        timing_path = self.directory/'configuration/imu_timing.json'
        if timing_path.exists():
            from wc_imu.device_time import load_timing_profile
            self.imu_timing = load_timing_profile(timing_path, sensor_id=contract['sources']['imu']['source_id'])
        (self.directory/'ready').mkdir(exist_ok=True)

    def observe(self, row):
        topic = row['topic']
        if '/lidar_' in topic and row.get('valid_point_count',0) <= 0:
            return  # Preserve the raw record, but unusable geometry is not readiness.
        if '/lidar_left/' in topic:
            logicals = ['lidar_left']
        elif '/lidar_right/' in topic:
            logicals = ['lidar_right']
        elif '/imu/' in topic:
            logicals = ['imu']
            if self.imu_timing is not None:
                if (row.get('device_time_status') != 'VALID_DEVICE_RELATIVE_TIME'
                        or row.get('timing_profile_sha256') != self.imu_timing['_profile_sha256']
                        or row.get('device_timestamp_unit') != self.imu_timing['device_timestamp_unit']
                        or any(row.get(field.replace('_raw','_valid')) is not True for field in self.imu_timing['required_fields'])):
                    raise ValueError('IMU device timing differs from frozen capture policy')
        elif '/wheel/' in topic and row.get('status') == 'RESPONSE_VALID':
            if len(row.get('register_words_u16', [])) != 2:
                raise ValueError('wheel feedback needs both raw registers')
            logicals = ['wheel_left','wheel_right']
        elif '/ultrasonic/' in topic and row.get('status') == 'VALID_RESPONSE':
            logicals = ['ultrasonic_address_'+str(row['address'])]
        else:
            return
        stamp = row.get('host_monotonic_ns')
        if type(stamp) is not int or stamp <= 0:
            raise ValueError('valid source timestamp required')
        if row.get('session_id', self.contract['session_id']) != self.contract['session_id']:
            raise ValueError('source belongs to another session')
        for logical in logicals:
            spec = self.contract['sources'][logical]
            if row['source_id'] != spec['source_id'] or ('side' in row and row['side'] != spec.get('side')):
                raise ValueError('source identity differs from frozen capture contract')
            current = self.values.get(logical)
            if not row.get('stream_epoch') or self.epochs.get(logical,row['stream_epoch']) != row['stream_epoch']:
                raise ValueError('source epoch changed during capture')
            self.epochs[logical] = row['stream_epoch']
            if logical.startswith('lidar_'):
                streams = self.lidar_representations.setdefault(logical,{})
                streams[row['representation']] = stamp
                if set(streams) != {'raw','filtered'}:
                    continue
                stamp = min(streams.values())
            if current is None:
                current = dict(schema_version=1, session_id=self.contract['session_id'],
                    logical_source=logical, source_id=row['source_id'], stream_epoch=row['stream_epoch'],
                    first_valid_monotonic_ns=stamp,last_valid_monotonic_ns=stamp,valid_count=0,
                    ready=True,error=None)
                self.values[logical] = current
            current['last_valid_monotonic_ns'] = max(current['last_valid_monotonic_ns'],stamp)
            current['valid_count'] += 1

    def flush(self):
        for logical,value in self.values.items():
            atomic_json(self.directory/'ready'/(logical+'.json'),value)


def drain_received_callbacks(spin_once, received_count, *, quiet_s=.5, timeout_s=3., clock=time.monotonic):
    """Sources have stopped/ACKed; consume queued middleware callbacks first.

    Quiet time is a bounded transport drain, never a completeness certificate;
    source-vs-bag reconciliation remains the authority.
    """
    started=last_change=clock()
    previous=received_count()
    while clock()-started < timeout_s:
        spin_once()
        now=clock();current=received_count()
        if current!=previous:
            previous=current;last_change=now
        elif now-last_change>=quiet_s:
            return True
    return False


def main(argv=None):
    from .capture import PROFILES, source_selection
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', choices=PROFILES, required=True)
    parser.add_argument('--sides', choices=('left','right','all'), default='all')
    parser.add_argument('--queue-size', type=int, default=256)
    args = parser.parse_args(argv)
    if not 1 <= args.queue_size <= 8192:
        raise ValueError('queue-size must be 1..8192')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.serialization import serialize_message
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py
    args.output.mkdir(parents=True, exist_ok=True)
    topics = topics_for_profile(args.profile, args.sides)
    work = queue.Queue(maxsize=args.queue_size)
    closing = threading.Event()
    ready = threading.Event()
    stopped = threading.Event()
    counters = {'received': 0, 'persisted': 0, 'dropped': 0, 'topics': {}, 'error': None}
    contract_path = args.output/'configuration/capture_contract.json'
    readiness = SourceReadiness(args.output,json.loads(contract_path.read_text())) if contract_path.is_file() else None

    def writer():
        bag = None
        try:
            bag = rosbag2_py.SequentialWriter()
            bag.open(rosbag2_py.StorageOptions(uri=str(args.output/'bag'), storage_id='sqlite3'),
                     rosbag2_py.ConverterOptions('', ''))
            for topic, kind in topics.items():
                bag.create_topic(rosbag2_py.TopicMetadata(name=topic, type=kind, serialization_format='cdr'))
            with (args.output/'records.jsonl').open('x', encoding='utf-8') as index:
                ready.set()
                while True:
                    try:
                        topic, cdr, stamp, row = work.get(timeout=.05)
                    except queue.Empty:
                        if closing.is_set():
                            break
                        continue
                    bag.write(topic, cdr, stamp)
                    index.write(json.dumps(dict(row, storage_stamp_ns=stamp,
                                                cdr_sha256=hashlib.sha256(cdr).hexdigest()))+'\n')
                    counters['persisted'] += 1
                    counters['topics'][topic] = counters['topics'].get(topic, 0)+1
                index.flush()
                os.fsync(index.fileno())
            # Humble closes the storage and writes metadata in the destructor.
            del bag
            bag = None
            for file in (args.output/'bag').rglob('*'):
                if file.is_file():
                    with file.open('rb') as stream:
                        os.fsync(stream.fileno())
        except Exception as exc:
            counters['error'] = 'RECORDER_WRITE_FAILED: '+str(exc)
            stopped.set()
            ready.set()
        finally:
            if bag is not None:
                del bag

    def callback(topic, message):
        counters['received'] += 1
        try:
            row = message_index(topic, message)
            work.put_nowait((topic, serialize_message(message), time.time_ns(), row))
            if readiness:
                readiness.observe(row)
        except Exception as exc:
            counters['dropped'] += 1
            counters['error'] = 'RECORDER_QUEUE_OR_ENVELOPE_FAILED: '+str(exc)
            stopped.set()

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stopped.set())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('v7_capture_recorder')
    def ready_tick():
        try:
            if readiness:
                readiness.flush()
        except Exception as error:
            counters['error'] = 'RECORDER_READINESS_FAILED: '+str(error)
            stopped.set()
    timer = node.create_timer(.25,ready_tick)
    # Acquisition can deliver many IMU packets in one serial read. A preview's
    # depth=5 loses valid source packets during disk/camera scheduling bursts.
    # Capture sources explicitly offer reliable transport; preview keeps its
    # original low-latency profile. Source/bag reconciliation remains mandatory.
    subscriptions = [node.create_subscription(get_message(kind), topic,
                     lambda message, topic=topic: callback(topic, message),
                     QoSProfile(depth=2048 if '/imu/' in topic else 128,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE))
                     for topic, kind in topics.items()]
    thread = threading.Thread(target=writer, name='rosbag-writer', daemon=True)
    thread.start()
    ready.wait(15.)
    if not ready.is_set() or counters['error']:
        stopped.set()
        counters['error'] = counters['error'] or 'RECORDER_START_TIMEOUT'
    else:
        atomic_json(args.output/'recorder_ready.json', dict(ready=True, topics=topics, pid=os.getpid(),
                                                          host_monotonic_ns=time.monotonic_ns()))
    try:
        while not stopped.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        if not counters['error'] and rclpy.ok():
            try:
                if not drain_received_callbacks(lambda:rclpy.spin_once(node,timeout_sec=.05),
                                                lambda:counters['received']):
                    counters['error']='RECORDER_TRANSPORT_DRAIN_TIMEOUT'
            except Exception as error:
                counters['error']='RECORDER_TRANSPORT_DRAIN_FAILED: '+str(error)
        ready_tick()
        node.destroy_node()
        closing.set()
        thread.join(45.)
        if thread.is_alive():
            counters['error'] = 'RECORDER_DRAIN_TIMEOUT'
        if rclpy.ok():
            rclpy.shutdown()
        result = dict(counters, profile=args.profile, **source_selection(args.profile, args.sides), closed_normally=not counters['error'],
                      synchronized=not counters['error'] and counters['received'] == counters['persisted'],
                      queue_remaining=work.qsize())
        if (args.output/'records.jsonl').is_file() and not thread.is_alive():
            result['index_sha256'] = digest(args.output/'records.jsonl')
        atomic_json(args.output/'recorder_summary.json', result)
    return 0 if result['synchronized'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
