"""Real ROS/CDR/SQLite integration using synthetic publishers only; domain 91."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from wc_runtime.source_archive import SourceJournal, atomic_json, digest
from wc_runtime.capture_audit import audit_capture
from wc_runtime.source_recorder import TOPICS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID='91', ROS_LOCALHOST_ONLY='1')
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from wc_interfaces.msg import SourceFrame, H30Frame
    from std_msgs.msg import String
    process = subprocess.Popen([sys.executable, '-m', 'wc_runtime.source_recorder', '--profile', 'mapping_core',
                                '--output', str(args.output)], env=os.environ.copy())
    node = None
    try:
        deadline = time.monotonic()+20
        while not (args.output/'recorder_ready.json').exists():
            if process.poll() is not None or time.monotonic()>deadline:
                raise RuntimeError('recorder failed before READY')
            time.sleep(.1)
        rclpy.init()
        node = rclpy.create_node('synthetic_capture_source')
        kinds = {'wc_interfaces/msg/SourceFrame':SourceFrame, 'wc_interfaces/msg/H30Frame':H30Frame,
                 'std_msgs/msg/String':String}
        capture_qos = QoSProfile(depth=128, reliability=ReliabilityPolicy.RELIABLE)
        publishers = {topic:node.create_publisher(kinds[kind],topic,capture_qos) for topic,kind in TOPICS.items()}
        deadline = time.monotonic()+10
        while any(p.get_subscription_count()<1 for p in publishers.values()):
            if time.monotonic()>deadline:raise RuntimeError('DDS subscriber discovery timeout')
            rclpy.spin_once(node,timeout_sec=.05)
        imu = SourceJournal(args.output/'sources/imu', 'synthetic_imu')
        wheels = []
        lidar_rows = {side:[] for side in ('left','right')}
        for sequence in range(10):
            for side in ('left','right'):
                hashes = {}
                for filtered in (False,True):
                    payload = bytes([sequence, int(filtered)])*32
                    message = SourceFrame(sensor_id=side, side=side, session_id='synthetic', stream_epoch='epoch',
                                          frame_sequence=sequence, host_monotonic_ns=time.monotonic_ns())
                    message.cloud.data = payload
                    hashes['filtered_sha256' if filtered else 'raw_sha256'] = hashlib.sha256(payload).hexdigest()
                    publishers['/wc_mapping/lidar_'+side+'/source_frame'+('_filtered' if filtered else '')].publish(message)
                lidar_rows[side].append(dict(sensor_id=side,stream_epoch='epoch',sequence=sequence,**hashes))
            payload = bytes([sequence])*8
            checksum = hashlib.sha256(payload).hexdigest()
            imu.append(dict(event='byte_batch',batch_id=sequence,bytes_hex=payload.hex(),sha256=checksum))
            imu.append(dict(event='frame',batch_id=sequence,sensor_id='imu',stream_epoch='epoch',sequence=sequence,packet_sha256=checksum))
            message = H30Frame(sensor_id='imu', stream_epoch='epoch', frame_sequence=sequence, raw_packet=payload)
            publishers['/wc_mapping/imu/source_frame'].publish(message)
            row = dict(event='transaction_complete',device_id='wheel',stream_epoch='epoch',sequence=sequence,
                       response_sha256=hashlib.sha256(bytes.fromhex('01030400000000fa33')).hexdigest(),
                       request_hex='010320ab0002be2b',response_hex='01030400000000fa33',register_words_u16=[0,0],
                       status='RESPONSE_VALID',receive_monotonic_ns=time.monotonic_ns())
            wheels.append(row)
            publishers['/wc_mapping/wheel/feedback_raw'].publish(String(data=json.dumps(row)))
            rclpy.spin_once(node,timeout_sec=.04)
        imu.close()
        wheels += [dict(event='capture_complete',completed=10),dict(event='lease_closed')]
        (args.output/'sources/wheel_feedback.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in wheels))
        atomic_json(args.output/'sources/wheel_summary.json',dict(synchronized=True,closed_normally=True,
            completed=10,journal={'final_fsync_complete':True},events_sha256=digest(args.output/'sources/wheel_feedback.jsonl')))
        for side,rows in lidar_rows.items():
            directory = args.output/'sources/lidar'/side/'epoch'
            directory.mkdir(parents=True)
            (directory/'source_frames.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
            atomic_json(directory/'source_summary.json',dict(synchronized=True,closed_normally=True,
                        published_frames=10,journal_frames=10))
        time.sleep(.5)
        process.send_signal(signal.SIGINT)
        rc = process.wait(timeout=55)
        report = audit_capture(args.output,'mapping_core',dict(recorder=rc,lidar=0,imu=0,wheel=0))
        report['hardware_started'] = False
        atomic_json(args.output/'integration_result.json',report)
        print(json.dumps(report,ensure_ascii=False))
        return 0 if report['recording_complete'] else 1
    finally:
        if process.poll() is None:
            process.terminate()
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        if node is not None:
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
