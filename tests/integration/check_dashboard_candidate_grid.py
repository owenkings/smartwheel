#!/usr/bin/env python3
"""Isolate only the configured candidate-ground display offset, without hardware."""
import json
from pathlib import Path
import subprocess
import sys


def publisher(argv):
    from rclpy.publisher import Publisher
    from nav_msgs.msg import OccupancyGrid
    from wc_runtime.mapping_visuals import build_scene, display_grid
    import check_mapping_app_export as exported
    root = Path('/home/nvidia/wheelchair/reports/maps/map_dashboard_left_20260914_04')
    scene = build_scene(json.loads((root/'runtime_config.json').read_text()),
                        json.loads((root/'prior/initialization.json').read_text()))
    original = Publisher.publish
    def publish(self, message):
        if isinstance(message, OccupancyGrid):
            message = display_grid(message, scene['ground_z_m'])
        return original(self, message)
    Publisher.publish = publish
    print(json.dumps({'diagnostic': 'ACTUAL_SAVED_INSTALLATION_CANDIDATE_GRID_HEIGHT',
                      'ground_z_m': scene['ground_z_m'], 'native_map_modified': False}), flush=True)
    return exported.main(argv)


def main():
    import check_dashboard_saved_render as harness
    original = subprocess.Popen
    replacements = []
    def spawn(argv, *args, **kwargs):
        command = list(argv)
        if '--publish' in command and any(str(p).endswith('check_mapping_app_export.py') for p in command):
            index = command.index('--')+1
            inner = command[index:]
            command = command[:index]+[sys.executable, '-s', str(Path(__file__).resolve()),
                                      '--publisher', *inner[3:]]
            replacements.append(inner)
        return original(command, *args, **kwargs)
    output = Path('reports/map_six_issues_20260914/saved_candidate_grid_check_01')
    subprocess.Popen = spawn
    try:
        code = harness.main(['--synthetic-camera-images', '--output-root', str(output)])
    finally:
        subprocess.Popen = original
    if output.is_dir():
        (output/'candidate_grid_scope.json').write_text(json.dumps({
            'only_geometry_change': 'Display grid Z uses actual04 build_scene ground_z_m; all saved originals retained.',
            'publisher_replacements': len(replacements), 'hardware_started': False,
            'independent_visual_review_required': True}, indent=2))
    return code if len(replacements) == 1 else 1


if __name__ == '__main__':
    raise SystemExit(publisher(sys.argv[2:]) if sys.argv[1:2] == ['--publisher'] else main())
