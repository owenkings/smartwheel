"""SYNTHETIC H30Frame CDR roundtrip; no serial port is opened or node started."""

import argparse
import json
from pathlib import Path
import struct

from wc_imu.ros_node import assign_ros_frame, frame_record
from wc_sensors.h30 import H30Parser, h30_checksum


def run(output):
    from rclpy.serialization import serialize_message, deserialize_message
    from wc_interfaces.msg import H30Frame
    payload = b'\x10\x0c'+struct.pack('<iii', 1000000, 2000000, 9810000)
    payload += b'\x51\x04'+struct.pack('<I', 54321)
    packet = b'\x59\x53'+struct.pack('<H', 123)+bytes([len(payload)])+payload
    packet += h30_checksum(packet[2:])
    frames = H30Parser().feed(packet)
    assert len(frames) == 1
    records = [frame_record(frames[0], session_id='SYN-H30-CDR', sensor_id='SYN-IMU', stream_epoch='SYN-BOOT',
                           frame_sequence=sequence, host_receive_ns=1_789_000_000_123_456_789, host_monotonic_ns=654321)
               for sequence in (1, 2)]
    messages = [deserialize_message(serialize_message(assign_ros_frame(record, H30Frame())), H30Frame) for record in records]
    for message in messages:
        assert bytes(message.raw_packet) == packet
        assert message.header.frame_id == 'imu_h30_native'
        assert message.header.stamp.sec == 1_789_000_000 and message.header.stamp.nanosec == 123_456_789
        assert message.imu.header == message.header
        assert not message.orientation_valid and message.imu.orientation_covariance[0] == -1.
        assert not message.angular_velocity_valid and message.imu.angular_velocity_covariance[0] == -1.
        assert message.linear_acceleration_valid and abs(message.imu.linear_acceleration.z - 9.81) < 1e-9
        assert message.sample_timestamp_valid and message.sample_timestamp_raw == 54321
        assert not message.common_time_valid and not message.uncertainty_valid
        assert message.time_source == 'arrival_only'
    assert messages[0].header.stamp == messages[1].header.stamp
    assert messages[0].frame_sequence != messages[1].frame_sequence
    evidence = {'level': 'SYNTHETIC', 'status': 'PASS', 'test': 'H30Frame_real_ROS_CDR_roundtrip',
                'physical_hardware_used': False, 'frames': len(messages), 'raw_packet_sha256': records[0]['packet_sha256'],
                'same_stamp_frames_distinguished_by_sequence': True}
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8') as stream:
        json.dump(evidence, stream, indent=2)
        stream.write('\n')
    print(json.dumps(evidence))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', required=True)
    run(parser.parse_args().evidence)
