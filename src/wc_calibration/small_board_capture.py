"""Read a COMPLETE bag and export single-sensor board diagnostics only."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import secrets

from .small_board_diagnostic import diagnose_small_board


def _sources():
    here = Path(__file__).resolve().parent
    return {name: hashlib.sha256((here / name).read_bytes()).hexdigest()
            for name in ('small_board_diagnostic.py', 'small_board_capture.py', 'plane_capture.py')}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', required=True)
    parser.add_argument('--capture', required=True)
    parser.add_argument('--forward-x-min', type=float, default=0.6)
    parser.add_argument('--forward-x-max', type=float, default=1.6)
    parser.add_argument('--selection-note', default='Original diagnostic foreground range band')
    args = parser.parse_args(argv)
    from .plane_capture import load_capture, assert_sources
    from wc_runtime.storage_policy import StoragePolicy
    root = Path(args.project_root).absolute()
    policy = StoragePolicy(root)
    policy.check()
    config_before = dict(policy.configuration_files)
    software_before = _sources()
    capture = load_capture(root, args.capture)
    if capture.get('source_mode') != 'real' or capture.get('units') != 'm' or capture.get(
            'coordinate_conventions') != {'left': 'FLU', 'right': 'FLU'}:
        raise ValueError('Expected source-bound real FLU point clouds in metres')
    report = {
        'schema_version': 1, 'kind': 'single_sensor_small_board_observation_diagnostic',
        'verification_level': 'REAL_BAG', 'created_utc': datetime.now(timezone.utc).isoformat(),
        'capture_id': capture['capture_id'], 'sensor_ids': capture['sensor_ids'],
        'provenance': capture['provenance'], 'software_sha256': software_before,
        'width': capture['width'], 'height': capture['height'], 'clouds': capture['clouds'],
        'units': 'm', 'coordinate_convention': 'FLU', 'sides': {},
        'calibration_performed': False, 'transform_right_to_left': None, 'live_eligible': False,
        'limitations': [
            'Selected foreground identity requires inspection; no physical board identity is proven.',
            '50 cm board is a local observation diagnostic, not a calibrated geometric reference.',
            'All fixed interior points participate; fitted residuals are not absolute accuracy.',
            'Within-capture repeatability is not an independent calibration validation.',
            'Changing fitted plane distance can include normal estimation variation, not physical motion.',
        ],
    }
    for side in ('left', 'right'):
        report['sides'][side] = diagnose_small_board(capture['clouds'][side],
            frames=capture['frames'][side], width=capture['width'], height=capture['height'],
            selection_x_band_m=(args.forward_x_min, args.forward_x_max),
            selection_provenance={'basis': 'EXPLICIT_CLI_FOREGROUND_RANGE_BAND',
                                  'note': args.selection_note})
    assert_sources(capture)
    if software_before != _sources():
        raise ValueError('Diagnostic sources changed during analysis')
    current = StoragePolicy(root)
    if current.configuration_files != config_before:
        raise ValueError('Storage configuration changed during analysis')
    current.check()
    raw = json.dumps(report, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode('utf-8')
    session = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_') + secrets.token_hex(6)
    output = current.resolve('data/calibration/small_board_diagnostic') / session
    if any(p.is_symlink() for p in (output, *output.parents)):
        raise ValueError('Output path contains a symbolic link')
    output.mkdir(parents=True, exist_ok=False)
    current.check()
    with (output / 'diagnostic.json').open('xb') as stream:
        stream.write(raw)
    receipt = {'status': 'DIAGNOSTIC_EXPORT_COMPLETE', 'output_directory': str(output),
               'report_sha256': hashlib.sha256(raw).hexdigest(),
               'capture_id': capture['capture_id'], 'calibration_performed': False,
               'live_eligible': False,
               'side_status': {side: report['sides'][side].get('status') for side in ('left', 'right')}}
    current.check()
    with (output / 'complete.json').open('x', encoding='utf-8') as stream:
        json.dump(receipt, stream, ensure_ascii=False, indent=2)
    print(json.dumps(receipt, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
