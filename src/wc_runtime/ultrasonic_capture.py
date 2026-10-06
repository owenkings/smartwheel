"""Identity-checked FD07 distance reads; FC03/0x0001 only, no configuration writes."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import select
import signal
import stat
import struct
import subprocess
import time
import uuid

from .source_archive import SourceJournal


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xa001 if crc & 1 else crc >> 1
    return crc


def request(address):
    if type(address) is not int or address not in (1, 2, 3, 4):
        raise ValueError('only reviewed ultrasonic addresses 1..4 allowed')
    body = struct.pack('>BBHH', address, 3, 1, 1)
    return body + struct.pack('<H', crc16(body))


def decode(response, address):
    if not response:
        return {'status': 'RESPONSE_TIMEOUT', 'distance_m': None}
    if len(response) < 5:
        return {'status': 'SHORT_RESPONSE', 'distance_m': None}
    if crc16(response[:-2]) != int.from_bytes(response[-2:], 'little'):
        return {'status': 'CRC_ERROR', 'distance_m': None}
    if response[0] != address:
        return {'status': 'ADDRESS_MISMATCH', 'distance_m': None}
    if response[1] == 0x83:
        return {'status': 'MODBUS_EXCEPTION', 'exception': response[2], 'distance_m': None}
    if len(response) != 7 or response[1:3] != b'\x03\x02':
        return {'status': 'INVALID_RESPONSE', 'distance_m': None}
    value = int.from_bytes(response[3:5], 'big')
    return {'status': 'VALID_RESPONSE', 'response_value_u16': value, 'distance_m': None,
            'unit_status': 'UNVERIFIED_MODEL_AND_REGISTER_UNITS'}


def validate_config(value):
    if not isinstance(value, dict):
        raise ValueError('ultrasonic configuration must be an object')
    expected = {'device': '/dev/smartwheel_ultrasonic', 'baud': 9600,
                'usb_vid': '1a86', 'usb_pid': '7523', 'register': 1,
                'addresses': [1, 2, 3, 4], 'function_code': 3,
                'expected_by_id': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'}
    if any(value.get(k) != v for k, v in expected.items()):
        raise ValueError('ultrasonic configuration differs from reviewed read-only protocol')
    # This CH340 has no unique serial number. The explicit configuration must
    # pin its physical topology; never infer identity from ttyUSB numbering.
    path = value.get('expected_usb_path')
    if not isinstance(path, str) or not re.fullmatch(r'platform-3610000\.usb-usb-0:\d+(?:\.\d+)*:\d+\.\d+', path):
        raise ValueError('explicit ultrasonic USB topology required')
    if any(type(value.get(k)) not in (int, float) for k in ('timeout_s', 'inter_sensor_delay_s')) or \
            not .1 <= value['timeout_s'] <= .5 or not .1 <= value['inter_sensor_delay_s'] <= 1:
        raise ValueError('bounded response timeout and inter-sensor interval required')
    if value.get('physical_role_status') != 'UNVERIFIED':
        raise ValueError('V7 ultrasonic physical role binding remains unverified')
    return value


def identity(config):
    alias = Path(config['device'])
    by_id = Path(config['expected_by_id'])
    device = alias.resolve(strict=True)
    if device != by_id.resolve(strict=True):
        raise RuntimeError('ULTRASONIC_IDENTITY_MISMATCH')
    info = dict(row.split('=', 1) for row in subprocess.check_output(
        ['udevadm', 'info', '--query=property', '--name=' + str(device)], text=True, timeout=5).splitlines() if '=' in row)
    if any(info.get(k) != v for k, v in {'ID_VENDOR_ID': config['usb_vid'],
        'ID_MODEL_ID': config['usb_pid'], 'ID_PATH': config['expected_usb_path']}.items()):
        raise RuntimeError('ULTRASONIC_IDENTITY_MISMATCH')
    actual = device.stat()
    if not stat.S_ISCHR(actual.st_mode):
        raise RuntimeError('ultrasonic target is not a character device')
    return device, actual


def observed_identity(config, lease, session_id):
    """Freeze actual udev properties against the already-open unique descriptor."""
    if lease.fd is None:
        raise RuntimeError('ultrasonic identity evidence requires an open unique lease')
    device, checked = identity(config)
    opened = os.fstat(lease.fd)
    if any(getattr(opened,key)!=getattr(checked,key) for key in ('st_rdev','st_dev','st_ino')):
        raise RuntimeError('ultrasonic opened identity changed before evidence snapshot')
    properties = dict(row.split('=',1) for row in subprocess.check_output(
        ['udevadm','info','--query=property','--name='+str(device)],text=True,timeout=5).splitlines() if '=' in row)
    expected = {'ID_VENDOR_ID':config['usb_vid'],'ID_MODEL_ID':config['usb_pid'],'ID_PATH':config['expected_usb_path']}
    if any(properties.get(key)!=value for key,value in expected.items()):
        raise RuntimeError('ULTRASONIC_IDENTITY_MISMATCH_DURING_SNAPSHOT')
    observed = {key:properties[key] for key in expected}
    return dict(schema_version=1,source_id='ultrasonic',session_id=session_id,
        device=config['device'],expected_by_id=config['expected_by_id'],resolved_device=str(device),
        expected_usb_path=observed['ID_PATH'],usb_path=observed['ID_PATH'],
        usb_vid=observed['ID_VENDOR_ID'],usb_pid=observed['ID_MODEL_ID'],udev_properties=observed,
        opened_identity={key:getattr(opened,key) for key in ('st_rdev','st_dev','st_ino')},
        verified_monotonic_ns=time.monotonic_ns(),verified_wall_ns=time.time_ns(),
        evidence_basis='CURRENT_UDEV_AND_OPEN_DESCRIPTOR_IDENTITY',physical_role_status='UNVERIFIED')


class DistanceLease:
    def __init__(self, config, run_root):
        self.config, self.run_root = validate_config(config), Path(run_root)
        self.fd = self.lock = None

    def open(self):
        import fcntl
        import termios
        from wc_imu.ros_node import require_unoccupied
        device, checked = identity(self.config)
        locks = self.run_root / 'locks'
        locks.mkdir(parents=True, exist_ok=True)
        self.lock = os.open(str(locks / ('serial-%d-%d.lock' %
            (os.major(checked.st_rdev), os.minor(checked.st_rdev)))), os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            require_unoccupied(device)
            again, again_stat = identity(self.config)
            if again != device or again_stat.st_rdev != checked.st_rdev:
                raise RuntimeError('identity changed during ultrasonic preflight')
            self.fd = os.open(str(device), os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY | os.O_CLOEXEC | os.O_NOFOLLOW)
            if os.fstat(self.fd).st_rdev != checked.st_rdev:
                raise RuntimeError('opened ultrasonic identity mismatch')
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            from wc_motion.feedback_transport import require_only_self
            require_only_self(device)
            attrs = termios.tcgetattr(self.fd)
            attrs[0] = attrs[1] = attrs[3] = 0
            attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            attrs[4] = attrs[5] = termios.B9600
            attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            return self
        except BaseException:
            self.close()
            raise

    def exchange(self, address):
        payload = request(address)
        # Drain at most one bounded stale buffer; retain it as diagnostic evidence.
        stale = os.read(self.fd, 256) if select.select([self.fd], [], [], 0)[0] else b''
        before, wall = time.monotonic_ns(), time.time_ns()
        sent = os.write(self.fd, payload)
        if sent != len(payload):
            raise RuntimeError('incomplete ultrasonic read request')
        response = b''
        deadline = time.monotonic() + self.config['timeout_s']
        while time.monotonic() < deadline:
            if select.select([self.fd], [], [], max(0, deadline - time.monotonic()))[0]:
                response += os.read(self.fd, 64 - len(response))
                if len(response) >= 5 and (response[1] & 0x80 or len(response) >= 7):
                    break
        return {'address': address, 'request_hex': payload.hex(), 'response_hex': response.hex(),
                'stale_bytes_hex': stale.hex(), 'request_wall_ns': wall,
                'request_monotonic_ns': before, 'host_receive_ns': time.time_ns(),
                'host_monotonic_ns': time.monotonic_ns(), 'transmitted_bytes': sent,
                'response_sha256': hashlib.sha256(response).hexdigest(), **decode(response, address)}

    def close(self):
        errors = []
        if self.fd is not None:
            import fcntl
            import termios
            try:
                fcntl.ioctl(self.fd, termios.TIOCNXCL)
            except OSError as exc:
                errors.append(str(exc))
            finally:
                descriptor, self.fd = self.fd, None
                try:
                    os.close(descriptor)
                except OSError as exc:
                    errors.append(str(exc))
        if self.lock is not None:
            descriptor, self.lock = self.lock, None
            try:
                os.close(descriptor)
            except OSError as exc:
                errors.append(str(exc))
        if errors:
            raise RuntimeError('ultrasonic lease close failed: '+'; '.join(errors))


def wait_for_recording_ack(publisher, timeout_s=5.):
    """Complete RELIABLE publication before recording can stop its subscriber."""
    if type(timeout_s) not in (int, float) or not .05 <= timeout_s <= 10:
        raise ValueError('bounded publication ACK timeout required')
    if publisher is None:
        return {'status': 'NOT_REQUESTED', 'acknowledged': None, 'timeout_s': timeout_s}
    from rclpy.duration import Duration
    acknowledged = publisher.wait_for_all_acked(timeout=Duration(seconds=timeout_s))
    if acknowledged is not True:
        raise RuntimeError('ULTRASONIC_RECORDING_ACK_TIMEOUT')
    return {'status': 'PASS', 'acknowledged': True, 'timeout_s': timeout_s}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--run-root', type=Path, required=True)
    parser.add_argument('--session', required=True)
    parser.add_argument('--publish-ros', action='store_true')
    parser.add_argument('--allow-read-queries', action='store_true')
    args = parser.parse_args(argv)
    if not args.allow_read_queries:
        raise ValueError('explicit read-query selection required')
    config = validate_config(json.loads(args.config.read_text(encoding='utf-8')))
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    journal = SourceJournal(args.output, 'ultrasonic')
    lease = DistanceLease(config, args.run_root)
    ros = node = publisher = None
    error = None
    epoch = str(uuid.uuid4())
    sequence = 0
    published = 0
    try:
        if args.publish_ros:
            import rclpy as ros
            from rclpy.signals import SignalHandlerOptions
            from std_msgs.msg import String
            ros.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
            node = ros.create_node('wc_ultrasonic_distance_reader')
            publisher = node.create_publisher(String, '/wc_mapping/ultrasonic/feedback_raw', 32)
            deadline = time.monotonic()+10.
            while publisher.get_subscription_count() < 1:
                if stopped or time.monotonic() > deadline:
                    raise RuntimeError('recording subscriber discovery timeout')
                ros.spin_once(node, timeout_sec=.05)
        lease.open()
        from .source_archive import atomic_json
        atomic_json(args.output/'identity.json',observed_identity(config,lease,args.session))
        while not stopped:
            failures = []
            for address in config['addresses']:
                if stopped:
                    break
                row = dict(lease.exchange(address), event='transaction', sensor_id='ultrasonic',
                           stream_epoch=epoch, sequence=sequence, session_id=args.session,
                           physical_role_status='UNVERIFIED', time_source='arrival_only')
                journal.append(row)
                if publisher is not None:
                    publisher.publish(String(data=json.dumps(row)))
                    published += 1
                    ros.spin_once(node, timeout_sec=0)
                sequence += 1
                if row['status'] != 'VALID_RESPONSE':
                    failures.append('%d:%s' % (address, row['status']))
                time.sleep(config['inter_sensor_delay_s'])
            if failures:
                raise RuntimeError('ULTRASONIC_READ_FAILURE ' + ','.join(failures))
    except Exception as exc:
        error = str(exc)
    finally:
        cleanup_errors = []
        try:
            lease.close()
        except Exception as exc:
            cleanup_errors.append('lease close: '+str(exc))
        ack = {'status': 'FAIL', 'acknowledged': False}
        try:
            ack = wait_for_recording_ack(publisher)
        except Exception as exc:
            cleanup_errors.append('publication finalization: '+str(exc))
        try:
            journal.append({'event': 'source_finalization', 'session_id': args.session,
                'stream_epoch': epoch, 'completed_transactions': sequence,
                'published_transactions': published, 'publication_ack': ack,
                'lease_closed': not any(x.startswith('lease close:') for x in cleanup_errors),
                'cleanup_errors': cleanup_errors})
        except Exception as exc:
            cleanup_errors.append('finalization journal: '+str(exc))
        if cleanup_errors:
            error = '; '.join(([error] if error else [])+cleanup_errors)
        try:
            journal.close(source_error=error)
        finally:
            if node is not None:
                node.destroy_node()
            if ros is not None and ros.ok():
                ros.shutdown()
    if error:
        print(error, flush=True)
    return 2 if error else 0


if __name__ == '__main__':
    raise SystemExit(main())
