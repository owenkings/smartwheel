#!/usr/bin/env python3
"""Target ROS message ABI checks using real generated classes; no devices/nodes."""
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from sensor_msgs.msg import Image
from rclpy.serialization import deserialize_message, serialize_message
from wc_cameras.node import diagnostic_level_number, fill_image, set_diagnostic_level


def main():
    observed = {}
    for state, expected in (('ACTIVE', 1), ('ERROR', 2), ('STOPPING', 1)):
        status = DiagnosticStatus()
        set_diagnostic_level(status, state)
        status.name, status.message = 'SYNTHETIC_ABI_NO_DEVICE', state
        array = DiagnosticArray(status=[status])
        restored = deserialize_message(serialize_message(array), DiagnosticArray).status[0]
        actual = diagnostic_level_number(restored.level)
        if actual != expected:
            raise RuntimeError('Native ROS diagnostic level did not survive serialization')
        observed[state] = {'level': actual, 'python_type': type(restored.level).__name__}
    frame = {'host_arrival_ns': 1700000000000000123, 'width': 2, 'height': 1, 'data': b'abcdef'}
    image = fill_image(Image(), frame, 'camera_usb3_4')
    restored = deserialize_message(serialize_message(image), Image)
    if ((restored.width, restored.height, restored.step, restored.encoding, bytes(restored.data)) !=
            (2, 1, 6, 'bgr8', b'abcdef') or restored.header.frame_id != 'camera_usb3_4'):
        raise RuntimeError('Native ROS Image payload/layout/port did not survive serialization')
    if (restored.header.stamp.sec, restored.header.stamp.nanosec) != (1700000000, 123):
        raise RuntimeError('Native ROS Image host-arrival timestamp changed')
    print(json.dumps({'status': 'PASS', 'verification_level': 'TARGET_ROS_MESSAGE_ABI_NO_DEVICES',
                      'diagnostics': observed, 'image_roundtrip': 'BGR8_PASS',
                      'camera_capture_validated': False}, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
