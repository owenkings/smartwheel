"""Hardware-free H30 poll cadence and read-completion arrival semantics."""
import contextlib
import io
import json
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))
import wc_imu.ros_node as module
from wc_sensors.h30 import h30_checksum


def arguments(extra=()):
    return ['--device', '/dev/SYN', '--expected-by-id', '/dev/serial/by-id/SYN',
            '--hardware-serial', 'SYN', '--sensor-id', 'H30-SYN', '--session-id', 'SYN-poll',
            '--run-root', str(ROOT / '.phase1_runtime'), *extra]


class PollPeriodTests(unittest.TestCase):
    def test_explicit_periods_and_unchanged_default(self):
        self.assertEqual(module.parser().parse_args(arguments()).poll_period_ms, 10.0)
        for value, seconds in ((2.5, .0025), (5, .005), (10, .01)):
            self.assertEqual(module.poll_period_seconds(value), seconds)
            parsed = module.parser().parse_args(arguments(['--poll-period-ms', str(value)]))
            self.assertEqual(parsed.poll_period_ms, value)

    def test_invalid_periods_rejected_before_any_hardware(self):
        for value in (True, False, '2.5', None, 0, -1, 1, 2, 2.5001, 20, float('nan'), float('inf')):
            with self.subTest(value=value), self.assertRaises(module.ImuAcquisitionError):
                module.poll_period_seconds(value)
        for text in ('0', '3', 'nan', 'inf'):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                module.parser().parse_args(arguments(['--poll-period-ms', text]))
            self.assertEqual(error.exception.code, 2)

    def test_both_arrival_clocks_sample_only_after_read_returns(self):
        events = []
        serial = NS(read_available=lambda: events.append('read_completed') or b'bytes')
        with patch.object(module.time, 'time_ns', side_effect=lambda: events.append('wall') or 100), \
                patch.object(module.time, 'monotonic_ns', side_effect=lambda: events.append('mono') or 200):
            self.assertEqual(module.read_with_host_arrival(serial), (b'bytes', 100, 200))
        self.assertEqual(events, ['read_completed', 'wall', 'mono'])

    def test_empty_read_still_has_post_read_monotonic_for_stale_check(self):
        with patch.object(module.time, 'time_ns', return_value=100), patch.object(module.time, 'monotonic_ns', return_value=200):
            self.assertEqual(module.read_with_host_arrival(NS(read_available=lambda: b'')), (b'', 100, 200))

    def test_failed_read_does_not_create_arrival_timestamp(self):
        def failed():
            raise OSError('synthetic read failure')
        with patch.object(module.time, 'time_ns') as wall, patch.object(module.time, 'monotonic_ns') as mono:
            with self.assertRaises(OSError):
                module.read_with_host_arrival(NS(read_available=failed))
            wall.assert_not_called()
            mono.assert_not_called()

    def test_node_timer_diagnostics_and_same_batch_frames_use_requested_period(self):
        # Run the real acquisition callback with an in-memory serial fixture and
        # minimal ROS API doubles. No serial fd, ROS node, publisher or timer runs.
        for requested in (2.5, 5, 10):
            with self.subTest(requested=requested):
                state = {'ok': True, 'nodes': [], 'events': [], 'closed': False}
                class Publisher:
                    def __init__(self):
                        self.messages = []
                    def publish(self, message):
                        self.messages.append(message)
                class Node:
                    def __init__(self, name):
                        self.publishers, self.timers = {}, []
                        state['nodes'].append(self)
                    def get_parameter(self, name):
                        return NS(value=False)
                    def create_publisher(self, cls, name, qos):
                        self.publishers[name] = Publisher()
                        return self.publishers[name]
                    def create_timer(self, seconds, callback):
                        self.timers.append((seconds, callback))
                    def get_clock(self):
                        return NS(now=lambda: NS(to_msg=lambda: NS(sec=0, nanosec=0)))
                    def get_logger(self):
                        return NS(error=lambda message: None)
                    def destroy_node(self):
                        pass
                class Serial:
                    def __init__(self, *args):
                        pass
                    def open(self):
                        return self
                    def read_available(self):
                        state['events'].append('read_completed')
                        return packet()*2
                    def close(self):
                        state['closed'] = True
                def frame_message():
                    return NS(header=NS(stamp=NS(sec=0, nanosec=0), frame_id=''),
                        imu=NS(header=None, orientation=NS(x=0., y=0., z=0., w=0.),
                            angular_velocity=NS(x=0., y=0., z=0.), linear_acceleration=NS(x=0., y=0., z=0.)))
                def spin_once(node, **kwargs):
                    node.acquire()
                    node.diagnostics()
                # Acquisition stamps and a later diagnostic age observation are
                # independent; the latter must not restamp either batch frame.
                monotonic_values = iter((100, 110, 120))
                def mono():
                    state['events'].append('mono')
                    return next(monotonic_values)
                def wall():
                    state['events'].append('wall')
                    return 1_700_000_000_123_456_789
                fake_modules = {
                    'rclpy': NS(init=lambda **kwargs: None, ok=lambda: state['ok'], spin_once=spin_once,
                                shutdown=lambda: state.update(ok=False)),
                    'rclpy.node': NS(Node=Node), 'rclpy.qos': NS(qos_profile_sensor_data='SYN'),
                    'rclpy.duration': NS(Duration=NS),
                    'rclpy.signals': NS(SignalHandlerOptions=NS(NO='NO')),
                    'sensor_msgs.msg': NS(Imu=object),
                    'diagnostic_msgs.msg': NS(DiagnosticArray=lambda: NS(header=NS(stamp=None, frame_id='')),
                        DiagnosticStatus=type('Status', (NS,), {'ERROR': 2, 'WARN': 1}), KeyValue=NS),
                    'wc_interfaces.msg': NS(H30Frame=frame_message)}
                with patch.dict(sys.modules, fake_modules), patch.object(module, 'ReadOnlySerialLease', Serial), \
                        patch.object(module.time, 'time_ns', side_effect=wall), patch.object(module.time, 'monotonic_ns', side_effect=mono):
                    code = module.main(arguments(['--poll-period-ms', str(requested), '--duration', '0.000000005']))
                self.assertEqual(code, 0)
                self.assertTrue(state['closed'])
                self.assertEqual(state['events'], ['mono', 'read_completed', 'wall', 'mono', 'mono'])
                node = state['nodes'][0]
                self.assertEqual(node.timers[0][0], requested/1000)
                frames = node.publishers['/wc_mapping/imu/source_frame'].messages
                self.assertEqual(len(frames), 2)
                self.assertEqual([frame.frame_sequence for frame in frames], [1, 2])
                for frame in frames:
                    self.assertEqual(frame.host_monotonic_ns, 110)
                    self.assertEqual((frame.header.stamp.sec, frame.header.stamp.nanosec), (1_700_000_000, 123_456_789))
                    self.assertEqual(frame.time_source, 'arrival_only')
                    self.assertFalse(frame.common_time_valid)
                    self.assertEqual(frame.sample_timestamp_raw, 123456)  # Exact device bytes, never interpolated.
                diagnostic = node.publishers['/wc_mapping/imu/diagnostics'].messages[-1].status[0]
                values = {item.key: json.loads(item.value) for item in diagnostic.values}
                self.assertEqual(values['requested_poll_period_ms'], requested)


def packet():
    payload = b'\x10\x0c'+struct.pack('<iii', 0, 0, 9_810_000)
    payload += b'\x20\x0c'+struct.pack('<iii', 0, 0, 0)
    payload += b'\x51\x04'+struct.pack('<I', 123456)
    value = b'\x59\x53'+struct.pack('<H', 9)+bytes([len(payload)])+payload
    return value+h30_checksum(value[2:])


if __name__ == '__main__':
    unittest.main()
