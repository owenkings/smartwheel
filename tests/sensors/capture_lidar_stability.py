#!/usr/bin/env python3
"""Read an already-owned ROS session; preserve same-pixel raw/filtered samples.

This observer starts no device or motion. Raw means before SDK host filters,
not unprocessed optical phase measurements.
"""
import argparse
import hashlib
import json
import time
from pathlib import Path

import numpy as np


def network_counters():
    lines = Path('/proc/net/snmp').read_text().splitlines()
    result = {}
    for a, b in zip(lines[::2], lines[1::2]):
        keys, values = a.split(), b.split()
        if keys[0] in ('Udp:', 'Ip:') and keys[0] == values[0]:
            result[keys[0][:-1]] = dict(zip(keys[1:], map(int, values[1:])))
    return result


def decode(message):
    cloud = message.cloud
    expected = [(key, 4*i, 7, 1) for i, key in enumerate(('x', 'y', 'z', 'intensity'))]
    if (cloud.is_bigendian or cloud.height != 1 or cloud.point_step != 16 or
            cloud.row_step != cloud.width*16 or len(cloud.data) != cloud.row_step or
            [(f.name, f.offset, f.datatype, f.count) for f in cloud.fields] != expected):
        raise ValueError('Unexpected point representation; pixel correspondence cannot be assumed')
    points = np.frombuffer(bytes(cloud.data), dtype='<f4').reshape(-1, 4).copy()
    stamp = int(message.host_receive_time.sec)*10**9+int(message.host_receive_time.nanosec)
    flags = dict(x.split('=', 1) for x in message.diagnostic_flags if '=' in x)
    metadata = {'session_id': message.session_id, 'side': message.side, 'sensor_id': message.sensor_id,
        'sequence': int(message.frame_sequence), 'epoch': message.stream_epoch, 'host_ns': stamp,
        'monotonic_ns': int(message.host_monotonic_ns), 'device_raw': int(message.device_timestamp_raw),
        'device_unit': message.device_timestamp_unit, 'time_source': message.time_source,
        'common_time_valid': message.common_time_valid, 'units': message.units,
        'coordinate_convention': message.coordinate_convention, 'config_hash': message.source_config_hash,
        'sdk_frame_id': int(flags.get('sdk_frame_id', '-1')), 'point_count': int(cloud.width),
        'payload_sha256': hashlib.sha256(bytes(cloud.data)).hexdigest(), 'flags': flags}
    return points, metadata


def main():
    from wc_runtime.cli import target, ROOT, name
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from wc_interfaces.msg import SourceFrame
    from diagnostic_msgs.msg import DiagnosticArray
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=name, required=True)
    parser.add_argument('--sides', nargs='+', choices=('left', 'right'), required=True)
    parser.add_argument('--duration', type=float, default=25)
    parser.add_argument('--warmup', type=float, default=5)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    target()
    if not 1 <= args.duration <= 40 or not 0 <= args.warmup <= 15:
        raise ValueError('Bounded observation requires 1..40s plus 0..15s warmup')
    from wc_runtime.storage_policy import StoragePolicy
    storage = StoragePolicy(ROOT)
    out = storage.resolve(args.output)
    if not out.is_relative_to(storage.resolve('reports')) or any(p.is_symlink() for p in (out, *out.parents)):
        raise ValueError('New evidence directory must be inside reports')
    out.mkdir(parents=True, exist_ok=False)
    ids = json.loads((ROOT/'config/live_unvalidated.json').read_text())['sensor_ids']
    streams = {(side, rep): {} for side in args.sides for rep in ('raw', 'filtered')}
    diag = {side: [] for side in args.sides}
    errors = []
    started = time.monotonic()
    before = network_counters()
    rclpy.init()
    node = Node('wc_static_stability_observer_'+args.session)
    subscriptions = []
    def receive(message, side, rep):
        if time.monotonic()-started < args.warmup:
            return
        try:
            if (message.session_id != args.session or message.sensor_id != ids[side] or
                    message.side != side or message.units != 'm' or message.coordinate_convention != 'FLU'):
                raise ValueError('Session/device/units/coordinates mismatch')
            points, metadata = decode(message)
            key = (message.stream_epoch, int(message.frame_sequence))
            if len(streams[side, rep]) >= 500:
                raise ValueError('Observation frame budget exceeded')
            if key in streams[side, rep]:
                raise ValueError('Duplicate source observation')
            streams[side, rep][key] = (points, metadata)
        except Exception as error:
            errors.append(str(error))
    def diagnostic(message, side):
        for status in message.status:
            level = status.level[0] if isinstance(status.level, (bytes, bytearray)) else int(status.level)
            diag[side].append({'level': level, 'message': status.message,
                'values': {item.key: item.value for item in status.values}})
    for side in args.sides:
        for rep, suffix in (('raw', 'source_frame'), ('filtered', 'source_frame_filtered')):
            subscriptions.append(node.create_subscription(SourceFrame, '/wc_mapping/lidar_'+side+'/'+suffix,
                lambda msg, side=side, rep=rep: receive(msg, side, rep), qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(DiagnosticArray, '/wc_mapping/lidar_'+side+'/diagnostics',
            lambda msg, side=side: diagnostic(msg, side), qos_profile_sensor_data))
    try:
        while time.monotonic()-started < args.duration+args.warmup:
            storage.check()
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    result = {'session': args.session, 'status': 'CAPTURE_AUDIT_ONLY', 'physical_static_independently_verified': False,
        'starts_hardware': False, 'starts_motion': False, 'network_before': before,
        'network_after': network_counters(), 'duration_s': args.duration, 'warmup_s': args.warmup,
        'errors': errors, 'sides': {}}
    for side in args.sides:
        storage.check()
        raw, filtered = streams[side, 'raw'], streams[side, 'filtered']
        common = sorted(set(raw) & set(filtered))
        row = {'raw_received': len(raw), 'filtered_received': len(filtered), 'paired': len(common),
               'raw_unmatched': len(set(raw)-set(filtered)), 'filtered_unmatched': len(set(filtered)-set(raw)),
               'diagnostics': diag[side]}
        result['sides'][side] = row
        if len(common) < 20:
            errors.append(side+': insufficient paired observations')
            continue
        rows, filtered_rows = [raw[k] for k in common], [filtered[k] for k in common]
        for (pa, ma), (pb, mb) in zip(rows, filtered_rows):
            if (pa.shape != pb.shape or ma['host_ns'] != mb['host_ns'] or
                    mb['flags'].get('before_host_filter_cloud_sha256') != ma['payload_sha256']):
                raise ValueError('Raw/filtered pairing hash or source timestamp mismatch')
        meta = [m for p, m in rows]
        np.savez_compressed(out/(side+'.npz'), raw_xyz=np.stack([p[:, :3] for p, m in rows]),
            filtered_xyz=np.stack([p[:, :3] for p, m in filtered_rows]),
            raw_intensity=np.stack([p[:, 3] for p, m in rows]),
            filtered_intensity=np.stack([p[:, 3] for p, m in filtered_rows]),
            frame_sequence=np.array([m['sequence'] for m in meta], dtype=np.int64),
            host_ns=np.array([m['host_ns'] for m in meta], dtype=np.int64),
            device_raw=np.array([m['device_raw'] for m in meta], dtype=np.int64),
            sdk_frame_id=np.array([m['sdk_frame_id'] for m in meta], dtype=np.int64))
        (out/(side+'_frames.json')).write_text(json.dumps({'raw': meta,
            'filtered': [m for p, m in filtered_rows]}, indent=2)+'\n')
        row.update(npz=str(out/(side+'.npz')), first=meta[0], last=meta[-1])
    result['status'] = 'CAPTURE_AUDIT_PASS' if not errors else 'FAIL'
    storage.check()
    (out/'capture.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'status': result['status'], 'output': str(out), 'counts':
        {s: v['paired'] for s, v in result['sides'].items()}, 'errors': errors}))
    return 0 if not errors else 1


if __name__ == '__main__':
    raise SystemExit(main())
