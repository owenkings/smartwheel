#!/usr/bin/env python3
"""Read-only STATIC bag audit; not valid for recordings containing wheel motion.

Requires raw wheel feedback, its UNVALIDATED preview and matched diagnostics.
Other sensor topics are counted without decoding. No ROS node or device opens.
"""

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))
from wc_motion.protocol import parse_exchange, read_request


RAW = '/wc_mapping/wheel/feedback_raw'
PREVIEW = '/wc_mapping/wheel/odom_preview'
DIAGNOSTICS = '/wc_mapping/wheel/preview_diagnostics'
FORMAL = '/wc_mapping/wheel/odom'
QUERY = read_request(1, 0x20AB, 2)
DEVICE = 'ZLAC8030D-0000000014'
TYPES = {RAW: 'std_msgs/msg/String', PREVIEW: 'nav_msgs/msg/Odometry',
         DIAGNOSTICS: 'std_msgs/msg/String', '/tf': 'tf2_msgs/msg/TFMessage',
         '/tf_static': 'tf2_msgs/msg/TFMessage'}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def positive_integer(value, name, minimum=1):
    require(type(value) is int and value >= minimum, name + ' must be an integer in range')
    return value


def zero(values, name):
    require(all(math.isfinite(float(value)) and abs(float(value)) <= 1e-10 for value in values),
            name + ' is not finite static zero')


def vector(value):
    return (value.x, value.y, value.z)


class StaticAudit:
    def __init__(self, minimum=10):
        self.minimum = minimum
        self.counts = Counter()
        self.raw = {}
        self.previews = set()
        self.diagnostics = {}
        self.previous = None
        self.errors = []
        self.tf_edges = Counter()
        self.terminal_diagnostics = Counter()

    def raw_record(self, record):
        require(record.get('schema') == 'wc_wheel_feedback_v1' and record.get('device_id') == DEVICE,
                'unexpected raw schema or controller identity')
        require(record.get('status') == 'RESPONSE_VALID', 'raw response not valid')
        require(record.get('time_valid') is False and record.get('time_source') == 'arrival_only'
                and record.get('uncertainty_ns') is None, 'raw time classification changed')
        require(record.get('formal_odometry_eligible') is False and record.get('protocol_units_state') == 'UNVALIDATED',
                'raw feedback incorrectly classified as validated')
        require(type(record.get('control_transmissions')) is int and record['control_transmissions'] == 0,
                'raw record does not explicitly report zero control transmissions')
        require(record.get('request_hex') == QUERY.hex() and record.get('transmitted_bytes') == len(QUERY),
                'unexpected request or transmitted length')
        reply = bytes.fromhex(record['response_hex'])
        _, _, words = parse_exchange(QUERY, reply)
        require(list(words) == record.get('register_words_u16') == [0, 0], 'STATIC audit requires two zero wheel words')
        require(record.get('response_sha256') == hashlib.sha256(reply).hexdigest(), 'raw response hash mismatch')
        stamp = positive_integer(record['stamp_ns'], 'arrival stamp')
        receive = positive_integer(record['receive_monotonic_ns'], 'arrival monotonic stamp')
        sequence = positive_integer(record['sequence'], 'sequence', 0)
        epoch = record.get('stream_epoch')
        require(isinstance(epoch, str) and bool(epoch), 'missing stream epoch')
        if self.previous is not None:
            require(epoch == self.previous['stream_epoch'] and sequence == self.previous['sequence'] + 1,
                    'raw stream epoch/sequence discontinuity')
            require(stamp > self.previous['stamp_ns'] and receive > self.previous['receive_monotonic_ns'],
                    'raw arrival time not increasing')
        require(stamp not in self.raw, 'duplicate raw stamp')
        self.raw[stamp] = record
        self.previous = record

    def preview(self, message):
        stamp = int(message.header.stamp.sec)*1_000_000_000 + int(message.header.stamp.nanosec)
        require(message.header.frame_id == 'wheel_odom_preview' and message.child_frame_id == 'axle_preview',
                'preview uses formal or unexpected frames')
        zero(vector(message.pose.pose.position), 'preview position')
        orientation = message.pose.pose.orientation
        zero((orientation.x, orientation.y, orientation.z, abs(orientation.w)-1), 'preview orientation')
        zero(vector(message.twist.twist.linear) + vector(message.twist.twist.angular), 'preview velocity')
        for covariance in (message.pose.covariance, message.twist.covariance):
            require(len(covariance) == 36 and all(math.isfinite(value) for value in covariance), 'invalid preview covariance')
            require(all(covariance[index] >= 1e6 for index in (0, 7, 14, 21, 28, 35)),
                    'preview covariance claims unsupported precision')
        require(stamp not in self.previews, 'duplicate preview stamp')
        self.previews.add(stamp)

    def diagnostic(self, record):
        require(record.get('formal_odometry_eligible') is False and record.get('publishes_tf') is False
                and record.get('time_valid') is False, 'preview diagnostic claims formal odometry, TF or common time')
        if record.get('state') == 'STOPPED':
            self.terminal_diagnostics['STOPPED'] += 1
            return
        require(record.get('schema') == 'wc_wheel_history_preview_sample_v1' and record.get('state') == 'UNVALIDATED',
                'preview diagnostic is blocked, malformed or validated')
        require(record.get('time_source') == 'arrival_only' and record.get('uncertainty_ns') is None
                and record.get('precision_evidence') == 'NONE', 'preview diagnostic changed timing/precision evidence')
        stamp = positive_integer(record['stamp_ns'], 'preview diagnostic stamp')
        require(stamp not in self.diagnostics, 'duplicate preview diagnostic stamp')
        zero([record[key] for key in ('left_raw_signed', 'right_raw_signed', 'linear_velocity_m_s',
            'angular_velocity_rad_s', 'x_m', 'y_m', 'yaw_rad', 'signed_distance_m', 'travel_distance_m')],
            'preview diagnostic motion')
        self.diagnostics[stamp] = record

    def message(self, topic, message):
        if topic == RAW:
            self.raw_record(json.loads(message.data))
        elif topic == PREVIEW:
            self.preview(message)
        elif topic == DIAGNOSTICS:
            self.diagnostic(json.loads(message.data))
        elif topic in ('/tf', '/tf_static'):
            for transform in message.transforms:
                parent, child = transform.header.frame_id.lstrip('/'), transform.child_frame_id.lstrip('/')
                self.tf_edges[parent + '->' + child] += 1
                require(not {parent, child}.intersection({'wheel_odom_preview', 'axle_preview', 'wheel_odom', 'axle_link'}),
                        'wheel preview/formal transform present in static feedback recording')

    def result(self):
        failures = list(self.errors)
        for condition, reason in (
            (len(self.raw) >= self.minimum, 'insufficient valid raw feedback'),
            (len(self.previews) >= self.minimum, 'insufficient valid preview odometry'),
            (self.previews.issubset(self.raw), 'preview timestamps missing matching raw feedback'),
            (self.previews.issubset(self.diagnostics), 'preview timestamps missing UNVALIDATED diagnostics'),
            (self.counts[FORMAL] == 0, 'formal wheel odometry was recorded')):
            if not condition:
                failures.append(reason)
        for stamp in self.previews.intersection(self.raw).intersection(self.diagnostics):
            raw, diagnostic = self.raw[stamp], self.diagnostics[stamp]
            if any(raw[key] != diagnostic.get(key) for key in ('stream_epoch', 'sequence', 'receive_monotonic_ns')):
                failures.append('preview/raw identity or monotonic timestamp mismatch at ' + str(stamp))
        sequences = [record['sequence'] for record in self.raw.values()]
        return {'schema': 'wc_static_recorded_feedback_audit_v1', 'status': 'PASS' if not failures else 'FAIL',
            'scope': 'STATIC_ZERO_WHEEL_FEEDBACK_ONLY_NOT_DYNAMIC_ODOMETRY_ACCEPTANCE',
            'minimum_samples': self.minimum, 'topic_counts': dict(self.counts),
            'valid_raw_count': len(self.raw), 'valid_preview_count': len(self.previews),
            'valid_preview_diagnostics_count': len(self.diagnostics),
            'sequence_first': sequences[0] if sequences else None, 'sequence_last': sequences[-1] if sequences else None,
            'subscription_may_start_after_sequence_zero': True, 'terminal_diagnostics': dict(self.terminal_diagnostics),
            'recorded_tf_edges': dict(self.tf_edges), 'formal_odometry_eligible': False,
            'control_transmissions_evidence': 'zero as reported in validated raw records; not an independent bus monitor',
            'other_sensor_verification': 'topic counts only; no lidar or IMU sample quality claim', 'failures': failures}


def audit_bag(bag, audit):
    require((bag / 'metadata.yaml').is_file(), 'bag metadata.yaml missing; incomplete recording')
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    metadata = {item.name: item.type for item in reader.get_all_topics_and_types()}
    classes = {}
    for topic, expected in TYPES.items():
        if topic in metadata:
            require(metadata[topic] == expected, 'incorrect ROS type for ' + topic)
            classes[topic] = get_message(expected)
    while reader.has_next():
        topic, data, _ = reader.read_next()
        audit.counts[topic] += 1
        if topic in classes:
            try:
                audit.message(topic, deserialize_message(data, classes[topic]))
            except Exception as error:
                if len(audit.errors) < 100:
                    audit.errors.append(topic + ': ' + str(error))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--minimum-samples', type=int, default=10)
    args = parser.parse_args(argv)
    require(args.minimum_samples >= 10, 'minimum-samples must be at least 10')
    audit = StaticAudit(args.minimum_samples)
    try:
        audit_bag(args.bag.resolve(), audit)
    except Exception as error:
        audit.errors.append(type(error).__name__ + ': ' + str(error))
    result = dict(audit.result(), bag=str(args.bag.resolve()))
    with args.output.open('x', encoding='utf-8') as output:
        json.dump(result, output, ensure_ascii=False, indent=2, allow_nan=False)
        output.write('\n')
    print(json.dumps({'status': result['status'], 'raw': result['valid_raw_count'],
        'preview': result['valid_preview_count'], 'output': str(args.output)}, ensure_ascii=False))
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
