"""Real ROS messaging fault injection against wc_fusion.ros_node, synthetic data only.

This checks single-in-flight ICP admission, ID/time association, pause/recovery,
odometry jump rejection and source freshness. Injected OdomInfo/Odometry are test doubles, so PASS is not
RTAB-Map ICP or live mapping validation. No drivers, TF, navigation or motors.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid


def run(output_dir, domain_id):
    if not 0 <= domain_id <= 232:
        raise ValueError('explicit isolated ROS domain must be in 0..232')
    output = Path(output_dir).resolve()
    if not output.is_relative_to(Path('/home/nvidia/wheelchair').resolve()):
        raise ValueError('test output must be under the verified remote project parent')
    output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from rtabmap_msgs.msg import OdomInfo
    from wc_interfaces.msg import SourceFrame, FusionBundle, AcceptedBundle, MapStatus
    from wc_interfaces.srv import Resume
    from wc_fusion.ros_node import cloud_message

    session = 'SYN-FUSION-FAULTS-'+str(uuid.uuid4())
    calibration = np.eye(4)
    calibration[1, 3] = .32
    config = {'source_mode': 'synthetic', 'execution_mode': 'replay', 'session_id': session,
              'session_root': str(output/'session'), 'sensor_ids': {'left': 'SYN-L', 'right': 'SYN-R'},
              'calibration': {'status': 'CANDIDATE', 'source_mode': 'synthetic', 'calibration_id': 'SYN-CAL',
                              'sensor_ids': {'left': 'SYN-L', 'right': 'SYN-R'}, 'T_left_right': calibration.tolist()},
              'clock_model_ids': {'left': 'SYN-CLOCK-L', 'right': 'SYN-CLOCK-R'}, 'time_model_id': 'SYN-TIME',
              'fusion_policy': {'max_age_ns': 800_000_000}, 'icp_timeout_ns': 400_000_000,
              'min_icp_correspondences': 20}
    config_path = output/'session.json'
    config_path.write_text(json.dumps(config, indent=2), encoding='utf-8')
    log = (output/'fusion_process.log').open('w', encoding='utf-8')
    process = subprocess.Popen([sys.executable, '-m', 'wc_fusion.ros_node', '--session-config', str(config_path)],
                               env=os.environ.copy(), stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('synthetic_fusion_fault_injector')
    received, accepted, statuses, cloud_stamps, input_events = [], [], [], [], []

    def nanos(stamp):
        return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)

    def on_bundle(value):
        if value.session_id == session:
            received.append(value)
            input_events.append({'kind': 'bundle', 'bundle_id': int(value.bundle_id),
                                 'stamp_ns': nanos(value.header.stamp),
                                 'observed_monotonic_ns': time.monotonic_ns()})

    def on_cloud(value):
        # The explicit private ROS domain contains only this synthetic fixture.
        cloud_stamps.append(nanos(value.header.stamp))
        input_events.append({'kind': 'icp_cloud', 'stamp_ns': nanos(value.header.stamp),
                             'observed_monotonic_ns': time.monotonic_ns()})

    durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(FusionBundle, '/wc_mapping/fusion/bundle', on_bundle, 10)
    node.create_subscription(PointCloud2, '/wc_mapping/fusion/points', on_cloud,
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    node.create_subscription(AcceptedBundle, '/wc_mapping/accepted', lambda value: accepted.append(value) if value.bundle.session_id == session else None, 10)
    node.create_subscription(MapStatus, '/wc_mapping/frontend/status', lambda value: statuses.append(value) if value.session_id == session else None, durable)
    sources = {side: node.create_publisher(SourceFrame, '/wc_mapping/lidar_'+side+'/source_frame', qos_profile_sensor_data)
               for side in ('left', 'right')}
    odom_pub = node.create_publisher(Odometry, '/wc_mapping/odom', 8)
    info_pub = node.create_publisher(OdomInfo, '/wc_mapping/icp/odom_info', 8)
    resume_client = node.create_client(Resume, '/wc_mapping/resume_frontend')

    def pump_until(predicate, seconds=4):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            if process.poll() is not None:
                raise AssertionError('fusion process exited unexpectedly; inspect fusion_process.log')
            if predicate():
                return
            rclpy.spin_once(node, timeout_sec=.02)
        raise AssertionError('ROS integration condition timed out')

    def pump_for(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)

    def source(side, sequence):
        stamp_ns = 100_000_000_000 + sequence*100_000_000 + (1_000_000 if side == 'right' else 0)
        message = SourceFrame()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        message.header.frame_id = 'lidar_'+side
        message.session_id, message.side = session, side
        message.sensor_id, message.stream_epoch = config['sensor_ids'][side], 'SYN-BOOT-'+side
        message.frame_sequence = sequence
        points = np.array([[1.5 + .08*x, -.5+.08*y, .2+.05*(x % 3)] for x in range(8) for y in range(6)])
        if side == 'right':
            points = points + [.11, -.32, .03]  # Independent synthetic geometry, no copied sensor identity.
        message.cloud = cloud_message(points, message.header.stamp, 'lidar_'+side)
        message.source_config_hash, message.coordinate_convention, message.units = 'SYN-CONFIG-'+side, 'FLU', 'm'
        message.host_receive_time = message.header.stamp
        message.host_monotonic_ns = time.monotonic_ns()
        message.common_time_valid, message.common_time_ns = True, stamp_ns
        message.clock_model_id, message.time_source = config['clock_model_ids'][side], 'synthetic'
        message.uncertainty_valid, message.uncertainty_ns = True, 10_000
        message.raw_count = message.valid_count = len(points)
        sources[side].publish(message)

    def pair(sequence):
        source('left', sequence)
        source('right', sequence)

    def icp(bundle, *, x=.0, stamp_offset_ns=0, send_info=True, lost=False):
        stamp = bundle.header.stamp.sec*1_000_000_000+bundle.header.stamp.nanosec+stamp_offset_ns
        message = Odometry()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        message.header.frame_id, message.child_frame_id = 'odom', 'rig_link'
        message.pose.pose.position.x, message.pose.pose.orientation.w = float(x), 1.
        odom_pub.publish(message)
        if send_info:
            icp_info(bundle, lost=lost)

    def icp_info(bundle, *, stamp_offset_ns=0, lost=False):
        info = OdomInfo()
        # Build a distinct Header; changing a test-double stamp must not mutate
        # the observed FusionBundle that supplies the exact expected identity.
        info.header.frame_id = bundle.header.frame_id
        stamp = nanos(bundle.header.stamp)+stamp_offset_ns
        info.header.stamp.sec, info.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        info.lost = lost
        info.icp_correspondences, info.icp_inliers_ratio, info.icp_structural_complexity = 80, .9, .8
        info_pub.publish(info)

    def resume(epoch):
        request = Resume.Request()
        request.session_id, request.explicit_request, request.continuity_verified = session, True, True
        request.odom_epoch, request.calibration_id = epoch, 'SYN-CAL'
        request.left_clock_model_id, request.right_clock_model_id = 'SYN-CLOCK-L', 'SYN-CLOCK-R'
        future = resume_client.call_async(request)
        pump_until(future.done, 2)
        return future.result()

    checks = []
    admission_evidence = {}
    try:
        pump_until(lambda: all(pub.get_subscription_count() >= 1 for pub in sources.values()) and
                   odom_pub.get_subscription_count() >= 1 and info_pub.get_subscription_count() >= 1 and resume_client.service_is_ready(), 10)
        # Both pairs arrive before any ICP reply. A raw file appearing on disk
        # is not enough evidence of durability: require the worker's successful
        # directory-fsync acknowledgement to have been polled by the bridge.
        pair(1)
        pair(2)
        pump_until(lambda: len(received) >= 1 and len(cloud_stamps) >= 1)
        first = received[0]

        def both_archives_ready():
            try:
                state = json.loads((output/'session'/'frontend_status.json').read_text(encoding='utf-8'))
            except (FileNotFoundError, json.JSONDecodeError):
                return False
            assert state['session_id'] == session and state['source_mode'] == 'synthetic'
            entries = {int(item['bundle_id']): item for item in state.get('archive_timings', [])}
            if not {1, 2}.issubset(entries):
                return False
            for bundle_id in (1, 2):
                entry = entries[bundle_id]
                assert entry['error'] is None, entry
                assert 0 < entry['timing_ns']['directory_fsync_end'] <= entry['timing_ns']['polled'], entry
            admission_evidence['archive_acknowledgements'] = [entries[1], entries[2]]
            return True

        pump_until(both_archives_ready, 1)
        pump_for(.04)
        assert len(received) == len(cloud_stamps) == 1 and not accepted, \
            'two ready archives produced a second ICP input before any first-frame result'
        raw_records = [json.loads((output/'session'/'raw_observations'/f'{bundle_id}.json').read_text(encoding='utf-8'))
                       for bundle_id in (1, 2)]
        for bundle_id, raw in zip((1, 2), raw_records):
            assert raw['session_id'] == session and raw['source_mode'] == 'synthetic'
            assert int(raw['bundle_id']) == bundle_id
        assert first.bundle_id == 1 and nanos(first.header.stamp) == raw_records[0]['t_ref_ns']
        assert cloud_stamps == [nanos(first.header.stamp)]
        admission_evidence['first_odom_sent_monotonic_ns'] = time.monotonic_ns()
        icp(first, send_info=False)
        pump_for(.04)
        assert len(received) == len(cloud_stamps) == 1 and not accepted, \
            'Odometry alone released a second ICP input'
        icp_info(first, stamp_offset_ns=1)
        pump_for(.04)
        assert len(received) == len(cloud_stamps) == 1 and not accepted, \
            'mismatched OdomInfo released a second ICP input'
        admission_evidence['matching_info_sent_monotonic_ns'] = time.monotonic_ns()
        icp_info(first)
        pump_until(lambda: len(accepted) == 1 and len(received) == 2 and len(cloud_stamps) == 2)
        second = received[1]
        assert [int(value.bundle_id) for value in received] == [1, 2]
        assert [nanos(value.header.stamp) for value in received] == [raw['t_ref_ns'] for raw in raw_records]
        assert cloud_stamps == [raw['t_ref_ns'] for raw in raw_records]
        assert accepted[0].bundle.bundle_id == accepted[0].tracking.bundle_id == 1
        assert nanos(accepted[0].tracking.odometry.header.stamp) == raw_records[0]['t_ref_ns']
        second_events = [event for event in input_events if event['stamp_ns'] == raw_records[1]['t_ref_ns']]
        assert {event['kind'] for event in second_events} == {'bundle', 'icp_cloud'}
        assert all(event['observed_monotonic_ns'] >= admission_evidence['matching_info_sent_monotonic_ns']
                   for event in second_events), 'second input preceded matching dual ICP results'
        admission_evidence['inputs_after_matching_results'] = list(input_events)
        admission_evidence['raw_paths'] = [str(output/'session'/'raw_observations'/f'{bundle_id}.json')
                                         for bundle_id in (1, 2)]
        checks.append('two_durable_archives_wait_for_matching_odom_and_info_before_second_icp_input')
        epoch = accepted[-1].tracking.odom_epoch
        icp(second, x=.001, stamp_offset_ns=1)
        pump_for(.08)
        assert len(accepted) == 1, 'off-by-one-ns odometry must not match a bundle'
        icp(received[-1], x=.001, send_info=False)
        pump_until(lambda: len(accepted) == 2)
        checks.append('exact_stamp_join_rejects_wrong_ns_then_accepts_matching_pair')

        pair(3)
        pump_until(lambda: len(received) == 3)
        expired = received[-1]
        pump_until(lambda: statuses and statuses[-1].state == 'PAUSED_INVALID', 2)
        assert len(accepted) == 2
        checks.append('missing_icp_result_latches_pause')

        # Age old legitimate stability samples; repeated A/B packets alone must
        # not satisfy a five-independent-frame resume requirement.
        pump_for(4.2)
        for sequence in (20, 21, 20, 21, 20):
            pair(sequence)
            pump_for(.06)
        assert not resume(epoch).accepted, 'alternating duplicate observations falsely satisfied stable recovery'
        checks.append('resume_rejects_alternating_duplicate_frames')
        for sequence in range(30, 35):
            pair(sequence)
            pump_for(.06)
        response = resume(epoch)
        assert response.accepted, list(response.reasons)
        # Old odometry arriving after resume must not revive an expired join.
        icp(expired, x=.002)
        pump_for(.06)
        assert len(accepted) == 2
        pair(40)
        pump_until(lambda: len(received) == 4)
        icp(received[-1], x=100.)
        pump_until(lambda: statuses and any('ODOM_MOTION_JUMP' in reason for reason in statuses[-1].pause_reasons), 2)
        assert len(accepted) == 2
        checks.extend(['resume_clears_expired_join_and_old_timeout_origin', 'large_pose_jump_rejected'])

        for sequence in range(50, 55):
            pair(sequence)
            pump_for(.06)
        response = resume(epoch)
        assert response.accepted, list(response.reasons)
        for sequence in range(60, 69):
            source('left', sequence)
            pump_for(.13)
        pump_until(lambda: statuses and any('SOURCE_TIMEOUT:right' in reason for reason in statuses[-1].pause_reasons), 2)
        assert len(accepted) == 2
        checks.append('right_source_loss_latches_while_left_keeps_arriving')
        for sequence in range(80, 85):
            pair(sequence)
            pump_for(.06)
        response = resume(epoch)
        assert response.accepted, list(response.reasons)
        pair(90)
        pump_until(lambda: len(received) == 5)
        icp(received[-1], x=0., lost=True)
        pump_until(lambda: statuses and any('TRACKING_FAILED' in reason for reason in statuses[-1].pause_reasons), 2)
        assert len(accepted) == 2, 'lost ICP returning identity cannot be accepted as zero motion'
        checks.append('lost_tracker_identity_pose_is_not_zero_motion_success')
        bundle_ids = [int(value.bundle_id) for value in received]
        expected_cloud_stamps = [nanos(value.header.stamp) for value in received]
        assert len(bundle_ids) == len(set(bundle_ids)), 'ICP bundle was implicitly resent'
        assert cloud_stamps == expected_cloud_stamps and len(cloud_stamps) == len(set(cloud_stamps)), \
            'ICP cloud was lost, reordered, or implicitly resent'
        checks.append('each_icp_bundle_and_cloud_published_once_with_exact_original_stamp')
        evidence = {'level': 'SYNTHETIC', 'status': 'PASS', 'scope': 'ROS gate fault injection only',
                    'actual_icp_algorithm_validated': False, 'physical_hardware_used': False,
                    'session_id': session, 'checks': checks, 'accepted_bundle_ids': [value.bundle.bundle_id for value in accepted],
                    'single_inflight_admission': admission_evidence,
                    'icp_bundle_ids': bundle_ids, 'icp_cloud_stamps_ns': cloud_stamps,
                    'configured_max_age_ns': config['fusion_policy']['max_age_ns'],
                    'configured_icp_timeout_ns': config['icp_timeout_ns']}
        (output/'evidence.json').write_text(json.dumps(evidence, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(evidence))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)
        log.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--ros-domain-id', required=True, type=int)
    arguments = parser.parse_args()
    run(arguments.output_dir, arguments.ros_domain_id)
