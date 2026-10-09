#!/usr/bin/env python3
"""REAL_BAG offline raw wheel/IMU regression with an injected missing interval.

Only deserialize recorded CDR and call the pure MotionPrior. No rclpy.init,
ROS node, publication, device, launch, subprocess or control operation occurs.
Recorded source timestamps, identities and payloads are never rewritten.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sqlite3
import traceback


def stamp(value):
    return int(value.sec)*1_000_000_000+int(value.nanosec)


def file_identity(path):
    info = path.stat()
    return [info.st_size, info.st_mtime_ns]


def read_recording(source, config, max_clouds):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    imu_topic, wheel_topic = config['imu_topic'], config['wheel_topic']
    base_side = config['base_frame'].removeprefix('lidar_')
    cloud_candidates = [config['input_cloud_topic'], config['output_cloud_topic'],
                        '/wc_mapping/lidar_'+base_side+'/source_frame_filtered',
                        '/wc_mapping/lidar_'+base_side+'/source_frame']
    bags = sorted((source/'bag').glob('*.db3'))
    if not bags:
        raise RuntimeError('No SQLite bag parts under the selected session')
    metadata, totals = {}, {}
    for path in bags:
        with sqlite3.connect(path.as_uri()+'?mode=ro', uri=True) as db:
            metadata[path] = list(db.execute('SELECT id,name,type FROM topics'))
            for topic_id, name, _ in metadata[path]:
                if name in cloud_candidates:
                    totals[name] = totals.get(name, 0)+db.execute(
                        'SELECT count(*) FROM messages WHERE topic_id=?', (topic_id,)).fetchone()[0]
    cloud_topic = next((topic for topic in cloud_candidates if totals.get(topic)), None)
    if cloud_topic is None:
        raise RuntimeError('No recorded input/output/raw lidar cloud stamps available')
    events, clouds, hashes, counts = [], [], {}, {}
    identities = {str(path): file_identity(path) for path in bags}
    for part, path in enumerate(bags):
        with sqlite3.connect(path.as_uri()+'?mode=ro', uri=True) as db:
            topics = {topic_id: (name, get_message(type_name)) for topic_id, name, type_name in metadata[path]
                      if name in (imu_topic, wheel_topic, cloud_topic)}
            placeholders = ','.join('?' for _ in topics)
            for row_id, topic_id, recorded_ns, raw in db.execute(
                    'SELECT id,topic_id,timestamp,data FROM messages WHERE topic_id IN ('+placeholders+') '
                    'ORDER BY timestamp,id', tuple(topics)):
                name, message_type = topics[topic_id]
                if name == cloud_topic and len(clouds) >= max_clouds:
                    continue
                message = deserialize_message(raw, message_type)
                hashes.setdefault(name, hashlib.sha256()).update(raw)
                counts[name] = counts.get(name, 0)+1
                if name == imu_topic:
                    if not message.angular_velocity_valid or not message.linear_acceleration_valid or message.uncertainty_valid:
                        raise RuntimeError('Recorded H30 validity/classification rejected')
                    if message.header.frame_id != 'imu_h30_native' or message.imu.header.frame_id != 'imu_h30_native':
                        raise RuntimeError('Recorded H30 native frame identity rejected')
                    a, w = message.imu.linear_acceleration, message.imu.angular_velocity
                    values = dict(stamp_ns=stamp(message.host_receive_time), monotonic_ns=int(message.host_monotonic_ns),
                                  acceleration=[a.x, a.y, a.z], angular_velocity=[w.x, w.y, w.z],
                                  sensor_id=message.sensor_id, session_id=message.session_id,
                                  stream_epoch=message.stream_epoch, sequence=int(message.frame_sequence),
                                  coordinate_convention=message.coordinate_convention, time_source=message.time_source,
                                  common_time_valid=message.common_time_valid)
                    events.append((recorded_ns, part, row_id, 'imu', values['stamp_ns'], values))
                elif name == wheel_topic:
                    values = json.loads(message.data)
                    events.append((recorded_ns, part, row_id, 'wheel', values['stamp_ns'], values))
                else:
                    # SourceFrame and PointCloud2 both retain the original header.
                    clouds.append(stamp(message.header.stamp))
    events.sort(key=lambda row: row[:3])
    return events, sorted(set(clouds)), {
        'bag_file_identities_before': identities, 'read_only_sqlite': True,
        'recorded_topic_counts_used': counts, 'cloud_stamp_topic': cloud_topic,
        'concatenated_original_cdr_sha256': {name: value.hexdigest() for name, value in hashes.items()}}


def run_estimator(events, clouds, config, session_id, gap, inject):
    import numpy as np
    from wc_runtime.mapping_prior import MotionPrior, NotReady, DroppedCloud
    now = [0.]
    prior = MotionPrior(config, session_id, clock=lambda: now[0])
    prior.enable_startup_recovery(0.)
    begin, end = gap
    last_before_gap = max(index for index, event in enumerate(events) if event[4] < begin)
    dropped = {'imu': 0, 'wheel': 0}
    cloud_index, cloud_drops = 0, 0
    poses = []
    before = resumed = None
    for index, (_, _, _, kind, source_stamp, values) in enumerate(events):
        if inject and begin <= source_stamp < end:
            dropped[kind] += 1
            continue
        now[0] = (values['monotonic_ns'] if kind == 'imu' else values['receive_monotonic_ns'])*1e-9
        if kind == 'imu': prior.add_imu(**values)
        else: prior.add_wheel(values)
        if prior.initialization is None:
            continue
        watermark = min(prior.imu[-1]['stamp'], prior.wheel[-1]['stamp'])
        if index == last_before_gap:
            before = {'stamp_ns': watermark, 'pose': prior.pose_at(watermark).tolist()}
        if before is not None and resumed is None and watermark >= end:
            resumed = {'stamp_ns': watermark, 'pose': prior.pose_at(watermark).tolist()}
        while cloud_index < len(clouds) and clouds[cloud_index] <= watermark:
            value = clouds[cloud_index]
            cloud_index += 1
            try:
                pose = prior.pose_at(value)
                prior.require_observed_cloud_time(value)
            except (NotReady, DroppedCloud):
                cloud_drops += 1
                continue
            prior.record_forwarded(value, pose)
            poses.append({'stamp_ns': value, 'pose': pose.tolist()})
    if prior.failure or before is None or resumed is None:
        raise RuntimeError('Recording did not provide valid before/resumed watermarks: '+str(prior.failure))
    unchanged = bool(np.allclose(before['pose'], resumed['pose'], atol=1e-10, rtol=0))
    if inject and (not all(dropped.values()) or not unchanged):
        raise RuntimeError('Injected gap did not preserve the pre-gap pose without extrapolation')
    if not poses:
        raise RuntimeError('No real cloud stamp received a covered pose estimate')
    return {'status': 'PASS', 'injected_missing_records': dropped,
            'gap_before': before, 'first_resumed_watermark': resumed,
            'pose_unchanged_across_injected_gap': unchanged if inject else None,
            'cloud_pose_count': len(poses), 'cloud_stamps_without_coverage': cloud_drops,
            'cloud_stamps_still_waiting': len(clouds)-cloud_index, 'cloud_poses': poses,
            'initialization': prior.initialization, 'coverage': prior.coverage_report(),
            'retained_samples': {'imu': len(prior.imu), 'wheel': len(prior.wheel)},
            'final_estimator_failure': prior.failure}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--gap-seconds', type=float, default=2.)
    parser.add_argument('--max-clouds', type=int, default=300)
    args = parser.parse_args(argv)
    if not 0.1 <= args.gap_seconds <= 30 or not 1 <= args.max_clouds <= 10000:
        parser.error('gap seconds must be in [0.1,30], cloud count in [1,10000]')
    source, output = args.session_root.resolve(), args.output.resolve()
    if output.exists() or source == output or source in output.parents:
        raise RuntimeError('Use a new report file outside the original recording session')
    report = {'status': 'FAIL', 'validation_level': 'REAL_BAG_RAW_REINTEGRATION_WITH_INJECTED_GAP',
              'source_session': str(source), 'ros_nodes_started': False, 'ros_messages_published': 0,
              'hardware_accessed': False, 'source_times_rewritten': False,
              'limitation': 'Checks source handling, configured frame algebra and missing-data behavior; not physical mapping accuracy.'}
    try:
        import numpy as np
        from wc_runtime.mapping_input import mount_transforms
        runtime = json.loads((source/'runtime_config.json').read_text())
        fixed_runtime = copy.deepcopy(runtime); fixed_runtime['continuous_mapping'] = True
        mounts = mount_transforms(fixed_runtime, None)
        archived_prior = source/'prior_config.json'
        config = json.loads(archived_prior.read_text()) if archived_prior.exists() else copy.deepcopy(runtime['prior_template'])
        if not archived_prior.exists():
            config.update({key: mounts[key] for key in ('base_frame', 'R_base_imu', 'T_base_axle')})
            config['reference_frame'] = 'mapping_reference'
        config['continuous_mapping'] = True
        for key in ('R_base_imu', 'T_base_axle'):
            if not np.allclose(config[key], mounts[key], atol=1e-10, rtol=0):
                raise RuntimeError('Archived prior '+key+' differs from fixed mounting bootstrap')
        report['archived_configuration_used'] = str(archived_prior) if archived_prior.exists() else 'runtime prior_template plus fixed mount_transforms'
        report['mount_geometry_consistent'] = True
        events, clouds, recording = read_recording(source, config, args.max_clouds)
        report.update(recording)
        stamps = {name: [event[4] for event in events if event[3] == name] for name in ('imu', 'wheel')}
        if not all(stamps.values()):
            raise RuntimeError('Recording must contain both raw IMU and wheel records')
        first, last = max(values[0] for values in stamps.values()), min(values[-1] for values in stamps.values())
        span = last-first
        width = min(int(args.gap_seconds*1e9), span//4)
        if width < 200_000_000:
            raise RuntimeError('Insufficient shared recording duration for a meaningful dropped wheel/IMU interval')
        begin = first+span*2//5
        gap = begin, begin+width
        report['injected_gap_source_stamps_ns'] = list(gap)
        report['baseline'] = run_estimator(events, clouds, config, runtime['session_id'], gap, False)
        report['injected_gap'] = run_estimator(events, clouds, config, runtime['session_id'], gap, True)
        report['source_bag_file_identities_unchanged'] = all(
            file_identity(Path(path)) == identity for path, identity in report['bag_file_identities_before'].items())
        if not report['source_bag_file_identities_unchanged']:
            raise RuntimeError('Original recording file identity changed during read-only regression')
        report['status'] = 'PASS'
    except Exception as error:
        report.update(error=str(error), traceback=traceback.format_exc())
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, ensure_ascii=False, allow_nan=False, indent=2)
    print(json.dumps({'status': report['status'], 'report': str(output), 'error': report.get('error')}, ensure_ascii=False))
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
