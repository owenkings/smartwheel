#!/usr/bin/env python3
"""Display-only second dashboard on the already owned live session; no sensors."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from wc_runtime.cli import ROOT, target, ros_command, environment, name
from wc_runtime.sensor_viewer import desktop_environment
from check_rviz_display import capture_owned_window
from check_mapping_app_live import close_owned_window


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-name', type=name, required=True)
    args = parser.parse_args()
    target(); os.chdir(ROOT)
    session = ROOT/'reports/maps'/args.session_name
    output = ROOT/'reports/map_six_issues_20260914'/('secondary_'+args.session_name)
    output.mkdir(exist_ok=False)
    view = output/'display_diagnostic.rviz'
    view.write_bytes((session/'view.rviz').read_bytes())
    report = {'scope': 'Same live topics and exact view copy; manual session identity deliberately unavailable.',
              'hardware_started': False, 'control_socket_connected': False, 'status': 'FAIL'}
    env = desktop_environment(); env.update(environment())
    binary = ROOT/'install/main/wc_bringup/lib/wc_bringup/mapping_rviz'
    argv = [sys.executable, '-s', '-m', 'wc_runtime.component', '--parent', str(os.getpid()),
            '--sigint-grace-s', '5', '--', *ros_command([binary, '-d', view, '-t',
             'LIVE DISPLAY DIAGNOSTIC - CONTROLS DISCONNECTED', '--ros-args', '-r', '__node:=live_secondary_display',
             '-r', '/tf:=/wc_mapping/app/tf', '-r', '/tf_static:=/wc_mapping/app/tf_static'])]
    child = None
    with (output/'rviz.log').open('xb') as log:
        try:
            report['source_health_before'] = json.loads((session/'health/status.json').read_text())
            if report['source_health_before']['status'] != 'RUNNING':
                raise RuntimeError('Original mapping session is no longer running; no display started')
            child = subprocess.Popen(argv, cwd=ROOT, env=env, stdin=subprocess.DEVNULL,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            time.sleep(6)
            report['window'] = capture_owned_window(child, output/'dashboard.png', env, activate_owned=True)
            report['source_health_at_capture'] = json.loads((session/'health/status.json').read_text())
            close_owned_window(child, report['window'], env)
            report['returncode'] = child.wait(timeout=15)
            if child.returncode != 0: raise RuntimeError('Display did not close normally')
            report['status'] = 'CAPTURED_REQUIRES_VISUAL_REVIEW'
        except Exception as error:
            report['error'] = str(error)
        finally:
            if child is not None and child.poll() is None:
                child.send_signal(signal.SIGINT); child.wait(timeout=15)
    (output/'result.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report))
    return 0 if report['status'] == 'CAPTURED_REQUIRES_VISUAL_REVIEW' else 1


if __name__ == '__main__': raise SystemExit(main())
