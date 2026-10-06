#!/usr/bin/env python3
"""Replay the recorded scene with its actual saved wheelchair display layer.

The display-only worker rebuilds the schematic and candidate grid height from
the original session config/IMU initialization. No hardware, fake TF, control or
new calibration data is used. Original source files remain unchanged.
"""
import json
from pathlib import Path
import signal
import subprocess
import sys


def main():
    import check_dashboard_recorded_scene as harness
    original_spawn, original_view = subprocess.Popen, harness.recorded_view
    children, logs = [], []
    output = Path('reports/map_six_issues_20260914/recorded_visuals_check_01')
    session = Path('/home/nvidia/wheelchair/reports/maps/map_dashboard_left_20260914_04')

    def view(original):
        result, changes = original_view(original)
        changed = []
        for display in result['Visualization Manager']['Displays']:
            topic = display.get('Topic', {}).get('Value')
            if topic == harness.PREFIX+'view_markers':
                display['Enabled'] = display['Value'] = True
                changed.append('wheelchair schematic rebuilt from original config and initialization')
        if len(changed) != 1:
            raise ValueError('Expected exactly one recorded wheelchair display change')
        changes.update(recorded_installation_display_layer=changed,
                       wheelchair_view_markers_rebuilt=True,
                       native_grid_used_without_candidate_ground_shift=True)
        return result, changes

    def spawn(argv, *args, **kwargs):
        command = list(argv)
        if any(str(value).endswith('/wc_bringup/mapping_rviz') for value in command):
            prefix = command[:command.index('--')+1]
            code = ('import rclpy; from rclpy.parameter import Parameter; '
                    'original=rclpy.create_node; '
                    'rclpy.create_node=lambda *a,**k: original(*a,parameter_overrides=[Parameter("use_sim_time",value=True)],**k); '
                    'from wc_runtime.mapping_visuals import main; import sys; raise SystemExit(main(sys.argv[1:]))')
            log = (output/'recorded_visuals.log').open('xb'); logs.append(log)
            options = dict(kwargs, stdout=log)
            children.append(original_spawn(prefix+[sys.executable, '-s', '-c', code,
                '--session-root', str(session)], *args, **options))
        return original_spawn(command, *args, **kwargs)

    subprocess.Popen, harness.recorded_view = spawn, view
    code = 1
    try:
        code = harness.main(['--output-root', str(output)])
    finally:
        subprocess.Popen, harness.recorded_view = original_spawn, original_view
        exits = []
        for child in children:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
            exits.append(child.wait(timeout=15))
        for log in logs:
            log.close()
        if output.is_dir():
            (output/'display_layer_scope.json').write_text(json.dumps({
                'source_session': str(session), 'hardware_started': False,
                'synthetic_tf_published': False, 'configuration_modified': False,
                'worker_exit_codes': exits, 'independent_visual_review_required': True,
                'display_only': 'Unmodified production schematic/grid display methods with simulated clock from the original bag.'}, indent=2))
    return code if len(children) == 1 and exits == [0] else 1


if __name__ == '__main__':
    raise SystemExit(main())
