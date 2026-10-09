#!/usr/bin/env python3
"""Bounded real dual-preview entry test; never starts motion or other sensors.

Starts the operator entry and checks its own RViz DDS endpoints. The existing
display test opens additional owned windows with the same configs for capture.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from wc_runtime.cli import ROOT, target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    target()
    output = args.output_dir.absolute()
    if not output.resolve().is_relative_to(ROOT/'reports') or any(p.is_symlink() for p in (output, *output.parents)):
        raise ValueError('New evidence directory must be under project reports')
    output.mkdir(parents=True, exist_ok=False)
    if os.environ.get('ROS_DOMAIN_ID') != '83' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Requires project localhost domain 83')
    evidence = {'status': 'FAIL', 'starts_motion': False, 'source_mode': 'real',
                'scope': 'DUAL_SINGLE_FRAME_PREVIEW_NOT_MAP',
                'screenshot_scope': 'additional_owned_RViz_windows_using_same_configs'}
    child = descriptor = None
    started = time.monotonic()
    try:
        with (output/'entry.log').open('xb', buffering=0) as log:
            child = subprocess.Popen(['bash', 'scripts/view_lidars.sh', 'dual', '120'], cwd=ROOT,
                stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
            descriptor = os.pidfd_open(child.pid)
            deadline = time.monotonic()+25
            event = None
            while time.monotonic() < deadline and child.poll() is None:
                for line in (output/'entry.log').read_text().splitlines():
                    try:
                        row = json.loads(line)
                    except ValueError:
                        continue
                    if row.get('event') == 'LIDAR_VIEW_STARTED':
                        event = row
                if event:
                    break
                time.sleep(.2)
            if not event:
                raise RuntimeError('Viewer entry did not report startup; inspect entry.log')
            session = event['session_id']
            report_path = Path(event['report'])
            if report_path != ROOT/'reports/sensor_views'/session/'result.json':
                raise RuntimeError('Viewer reported an unexpected output path')
            evidence.update(session_id=session, entry_report=str(report_path))
            import rclpy
            from rclpy.node import Node
            rclpy.init()
            probe = Node('wc_lidar_entry_probe_'+str(os.getpid()))
            try:
                deadline = time.monotonic()+12
                found = {}
                while time.monotonic() < deadline and child.poll() is None:
                    rclpy.spin_once(probe, timeout_sec=.1)
                    for side in ('left', 'right'):
                        topic = '/wc_mapping/lidar_'+side+'/points_filtered'
                        expected = 'wc_lidar_view_'+session+'_'+side
                        found[side] = any(info.node_name == expected for info in probe.get_subscriptions_info_by_topic(topic))
                    if all(found.values()):
                        break
                evidence['entry_rviz_subscription_found'] = found
                if not all(found.values()):
                    raise RuntimeError('An original viewer RViz did not subscribe to its filtered stream')
            finally:
                probe.destroy_node()
                rclpy.shutdown()
            with (output/'display-test.log').open('xb') as log_display:
                checked = subprocess.run([sys.executable, '-s', 'tests/integration/check_sensor_rviz.py',
                    '--session', session, '--output-dir', str(output/'display')], cwd=ROOT,
                    stdout=log_display, stderr=subprocess.STDOUT, timeout=65)
            evidence['display_test_returncode'] = checked.returncode
            evidence['display_test'] = json.loads((output/'display/result.json').read_text())
            if checked.returncode:
                raise RuntimeError('Actual DDS/window capture did not pass')
            # Exercise the entry's window-exit path using one identity-owned
            # RViz process. This is a graceful signal, not an automated mouse click.
            from wc_runtime.component import process, belongs_to
            snapshot = json.loads(report_path.read_text())
            right = next(row for row in snapshot['rviz_processes'] if row['side'] == 'right')
            owner, record = process(child.pid), process(right['pid'])
            if owner is None or record is None or record.start_ticks != right['start_ticks'] or not belongs_to(record, owner):
                raise RuntimeError('Original right RViz is no longer owned by this test entry')
            window_fd = os.pidfd_open(record.pid)
            try:
                if not belongs_to(record, owner):
                    raise RuntimeError('Right RViz ownership changed before normal close signal')
                signal.pidfd_send_signal(window_fd, signal.SIGINT)
            finally:
                os.close(window_fd)
            evidence['stop_trigger'] = 'SIGINT_TO_OWNED_RIGHT_RVIZ_PROCESS_NOT_MOUSE_CLICK'
            child.wait(timeout=55)
            if json.loads(report_path.read_text()).get('stop_reason') != 'RVIZ_WINDOW_EXIT':
                raise RuntimeError('Entry did not detect the original right viewer process exit')
            evidence['functional_status'] = 'PASS'
    except Exception as error:
        evidence['error'] = type(error).__name__+': '+str(error)
    finally:
        if child is not None:
            if child.poll() is None:
                if descriptor is not None:
                    signal.pidfd_send_signal(descriptor, signal.SIGINT)
                else:
                    child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=55)
            except subprocess.TimeoutExpired:
                evidence['cleanup_timeout'] = True
            evidence['entry_exit_code'] = child.poll()
        if descriptor is not None:
            os.close(descriptor)
        report_path = Path(evidence['entry_report']) if evidence.get('entry_report') else None
        if report_path is not None and report_path.exists():
            evidence['entry_final'] = json.loads(report_path.read_text())
        clean = (evidence.get('entry_exit_code') == 0 and evidence.get('entry_final', {}).get('cleanup_status') == 'PASS')
        evidence['status'] = 'PASS' if clean and evidence.get('functional_status') == 'PASS' else 'FAIL'
        evidence['elapsed_s'] = time.monotonic()-started
        (output/'result.json').write_text(json.dumps(evidence, indent=2)+'\n')
    print(json.dumps({k: evidence.get(k) for k in ('status', 'session_id', 'entry_exit_code', 'elapsed_s', 'error')}))
    return 0 if evidence['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
