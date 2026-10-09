#!/usr/bin/env python3
"""Isolate late map arrival, using the unchanged audited saved-map harness.

Only the owned saved-map publisher is delayed. No sensor, TF or control source
is started. A separate report records this diagnostic timing intervention.
"""
import json
from pathlib import Path
import subprocess
import sys


def main():
    import check_dashboard_saved_render as harness
    original = subprocess.Popen
    delayed = []

    def spawn(argv, *args, **kwargs):
        command = list(argv)
        if '--publish' in command and any(str(p).endswith('check_mapping_app_export.py') for p in command):
            index = command.index('--') + 1
            inner = command[index:]
            # Delay publication AFTER the unchanged export audit has written
            # its readiness file, so GUI initialization occurs during delay.
            # Delaying before that audit merely delays GUI startup too.
            command = command[:index]+[sys.executable, '-s', '-c',
                'import sys,time; from pathlib import Path; '
                'sys.path.insert(0,str(Path(sys.argv[1]).parent)); '
                'import check_mapping_app_export as h; original=h.publish_saved; '
                'h.publish_saved=lambda *a,**k: (time.sleep(12),original(*a,**k))[-1]; '
                'raise SystemExit(h.main(sys.argv[2:]))', *inner[2:]]
            delayed.append({'delay_s': 12, 'phase': 'after_audit_before_publish', 'original_command': inner})
        return original(command, *args, **kwargs)

    output = Path('reports/map_six_issues_20260914/saved_render_delayed_map_02')
    subprocess.Popen = spawn
    try:
        code = harness.main(['--synthetic-camera-images', '--output-root', str(output)])
    finally:
        subprocess.Popen = original
    if output.is_dir():
        (output/'timing_intervention.json').write_text(json.dumps({
            'validation_level': 'OFFLINE_SAVED_MAP_DELAYED_ARRIVAL_DIAGNOSTIC',
            'hardware_started': False, 'delayed_publishers': delayed,
            'pass_requires_visual_review': True}, indent=2))
    return code if len(delayed) == 1 else 1


if __name__ == '__main__':
    raise SystemExit(main())
