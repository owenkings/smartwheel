#!/usr/bin/env python3
"""Exercise real operator CLI against a new synthetic storage test map only."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def fingerprints(root):
    result = {}
    for path in sorted(root.rglob('*')):
        if path.is_symlink():
            raise ValueError('test input must not contain symlinks')
        if path.is_file():
            result[str(path.relative_to(root))] = [path.stat().st_mtime_ns, hashlib.sha256(path.read_bytes()).hexdigest()]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path('/home/nvidia/wheelchair')
    snapshot = args.snapshot.resolve(strict=True)
    if not snapshot.is_relative_to(root/'data/synthetic'):
        raise ValueError('only a synthetic snapshot under this project is permitted')
    metadata = json.loads((snapshot/'snapshot.json').read_text())
    if metadata['source_mode'] != 'synthetic':
        raise ValueError('test refuses real map data')
    output = args.output.resolve()
    if not output.is_relative_to(root/'reports'):
        raise ValueError('reports must remain in the project')
    output.mkdir(parents=True, exist_ok=False)
    before = fingerprints(snapshot)
    map_id = 'synthetic_cli_' + str(time.time_ns())
    cli = [sys.executable, '-s', str(root/'scripts/wc_phase1')]
    results = []
    def run(name, *arguments, expected=0):
        with (output/(name+'.stdout.json')).open('x') as stdout, (output/(name+'.stderr.log')).open('x') as stderr:
            code = subprocess.run([*cli,*arguments], stdout=stdout, stderr=stderr, timeout=90, cwd=root).returncode
        result = dict(name=name, argv=arguments, exit_code=code, expected_exit_code=expected)
        results.append(result)
        if code != expected:
            raise AssertionError(f'{name}: exit {code}, expected {expected}; inspect retained output')
    status, error = 'FAIL', None
    try:
        run('doctor', 'doctor')
        run('status', 'status')
        (output/'no_calibration_versions').mkdir()
        run('calibration_list', 'calibrate', 'list', '--output-root', str(output/'no_calibration_versions'))
        run('save', 'save', '--map-id', map_id, '--version', 'v1', '--snapshot', str(snapshot))
        run('collision_refused', 'save', '--map-id', map_id, '--version', 'v1', '--snapshot', str(snapshot), expected=2)
        for command in ('verify','inspect','load'):
            run(command, command, '--map-id', map_id, '--version', 'v1')
        run('list', 'list', '--map-id', map_id)
        run('goal_set', 'goals', '--map-id', map_id, '--version', 'v1', 'set', '--name', 'SYNTHETIC_CLI_ONLY', '--position', '0', '0', '0', '--orientation-xyzw', '0', '0', '0', '1')
        run('goal_list', 'goals', '--map-id', map_id, '--version', 'v1', 'list')
        run('reload_after_goal', 'load', '--map-id', map_id, '--version', 'v1')
        if fingerprints(snapshot) != before:
            raise AssertionError('source snapshot changed')
        status = 'PASS'
    except Exception as failure:
        error = f'{type(failure).__name__}: {failure}'
    report = dict(status=status, error=error, source_mode='synthetic', map_id=map_id,
                  results=results, input_unchanged=fingerprints(snapshot)==before,
                  navigation_validated=False, scope='OPERATOR_CLI_STORAGE_AND_READ_ONLY_COMMANDS')
    (output/'result.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(dict(status=status,error=error,map_id=map_id,output=str(output))))
    return 0 if status == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
