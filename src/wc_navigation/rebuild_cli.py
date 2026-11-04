"""Rebuild declared single-lidar map candidates from an immutable frontend."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import time
import traceback
import uuid

from .rebuild import VARIANTS, derive_left_frontend


def _positive(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError('finite positive number required')
    return result


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, required=True)
    parser.add_argument('--source-cell', type=Path, required=True,
                        help='immutable frontend cell with runtime_config.json and frontend/index.jsonl')
    parser.add_argument('--variants', nargs='+', choices=VARIANTS, default=list(VARIANTS))
    parser.add_argument('--input-hz', type=_positive, default=5.0)
    parser.add_argument('--wall-interval', type=_positive, default=.20)
    parser.add_argument('--frame-timeout', type=_positive, default=30.0)
    args = parser.parse_args(argv)
    if len(args.variants) != len(set(args.variants)):
        parser.error('each declared variant may occur only once')
    if args.input_hz > 10:
        parser.error('--input-hz must be <= 10')
    if not .05 <= args.wall_interval <= 10:
        parser.error('--wall-interval must be in [0.05,10] seconds')
    if not 1 <= args.frame_timeout <= 120:
        parser.error('--frame-timeout must be in [1,120] seconds')
    return args


def validate_runtime_environment():
    if os.name != 'posix':
        raise RuntimeError('native rebuild requires the supported Linux target')
    if os.environ.get('ROS_DOMAIN_ID') != '94' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('offline rebuild requires ROS_DOMAIN_ID=94 and ROS_LOCALHOST_ONLY=1')
    if os.environ.get('ROS_DISCOVERY_SERVER'):
        raise RuntimeError('offline rebuild must not use an external ROS discovery server')


def validate_source_selection(config):
    """The batch workflow requires fresh own-stamp single-left observations."""
    if (not isinstance(config, dict) or config.get('mode') != 'left'
            or not isinstance(config.get('offline_source_selection'), dict)
            or config['offline_source_selection'].get('selected_sides') != ['left']):
        raise ValueError('rebuild workflow requires a frozen single-left frontend generated with compare --sides left')
    other = config.get('offline_lidar_selection')
    if isinstance(other, dict) and other.get('selected_sides') != ['left']:
        raise ValueError('frozen single-left source-selection declarations disagree')


def _sha(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def _stamp_hash(stamps):
    return hashlib.sha256(('\n'.join(str(x) for x in stamps) + '\n').encode()).hexdigest()


def validate_common_frames(manifests, rows_by_variant):
    """Require the exact same ordered observations across every candidate."""
    if not manifests or set(manifests) != set(rows_by_variant):
        raise ValueError('all declared variants need complete derived manifests and indexes')
    common = None
    source_id = None
    for variant, manifest in manifests.items():
        if manifest.get('status') != 'DERIVED_FRONTEND_COMPLETE' or manifest.get('sources_unchanged') is not True:
            raise ValueError('derived frontend is incomplete or source preservation is unverified: ' + variant)
        rows = rows_by_variant[variant]
        stamps = sorted(rows)
        if len(stamps) < 2 or any(type(x) is not int or x <= 0 for x in stamps):
            raise ValueError('at least two unique positive integer source stamps required')
        if manifest.get('retained_frames') != len(stamps) or manifest.get('eligible_stamp_sha256') != _stamp_hash(stamps):
            raise ValueError('derived index disagrees with its declared observation identity: ' + variant)
        identity = (manifest.get('source_cell'), manifest.get('source_runtime_sha256'), manifest.get('source_index_sha256'))
        if common is None:
            common, source_id = stamps, identity
        elif stamps != common or identity != source_id:
            raise ValueError('variants contain different original observations or source identities')
        for stamp in stamps:
            row = rows[stamp]
            if row.get('stamp_ns') != stamp:
                raise ValueError('index key and source stamp differ')
    return common


def _ordinary(path):
    path = Path(path).absolute()
    if '..' in path.parts or any(p.is_symlink() for p in (path, *path.parents)):
        raise ValueError('ordinary unlinked paths without parent traversal required')
    return path


@contextmanager
def _domain_lock(project):
    import fcntl
    directory = _ordinary(project / '.phase1_runtime/locks')
    directory.mkdir(parents=True, exist_ok=True)
    path = _ordinary(directory / 'navigation-rebuild-domain-94.lock')
    with path.open('a+b') as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('another owned rebuild is using offline ROS domain 94') from exc
        try:
            stream.seek(0)
            stream.truncate()
            stream.write(json.dumps({'pid': os.getpid(), 'mode': 'OFFLINE_MAP_REBUILD', 'ros_domain_id': 94}).encode())
            stream.flush()
            yield path
        finally:
            fcntl.flock(stream, fcntl.LOCK_UN)


def _write(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + '\n', encoding='utf-8')
    temporary.replace(path)


def main(argv=None):
    args = arguments(argv)
    validate_runtime_environment()
    from wc_runtime.storage_policy import StoragePolicy
    from wc_runtime import compare_native
    root = _ordinary(args.project_root)
    policy = StoragePolicy(root)
    if not policy.enabled:
        raise RuntimeError('native rebuild requires an explicitly configured external data root')
    policy.check()
    source = _ordinary(policy.resolve(args.source_cell))
    if not source.is_dir():
        raise ValueError('source frontend cell does not exist')
    validate_source_selection(json.loads((source / 'runtime_config.json').read_text(encoding='utf-8')))
    run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '_' + uuid.uuid4().hex[:8]
    output = policy.resolve('data/analysis/navigation_rebuild_' + run_id)
    if output.is_relative_to(source) or source.is_relative_to(output):
        raise ValueError('rebuild output must remain separate from its immutable source')
    result = {
        'schema_version': 1, 'run_id': run_id, 'status': 'STARTING',
        'source_cell': str(source), 'output': str(output), 'result_path': str(output / 'result.json'),
        'progress_path': str(output / 'progress.json'), 'ros_domain_id': 94, 'localhost_only': True,
        'hardware_started': False, 'control_topics_published': False, 'navigation_validated': False,
        'variant_selection_policy': 'RUN_DECLARED_VARIANTS_NO_AUTOMATIC_WINNER',
        'declared_variants': list(args.variants), 'derivations': {}, 'native_runs': {},
        'input_hz': args.input_hz, 'wall_interval_s': args.wall_interval,
        'frame_timeout_s': args.frame_timeout,
        'floor_model': 'CONFIGURED_AXLE_PLANE_AND_WHEEL_RADIUS_UNVALIDATED',
        'low_obstacle_qualification': False, 'physical_test_performed': False,
        'source_metadata_sha256': {name: _sha(source / name) for name in ['runtime_config.json', 'frontend/index.jsonl']},
        'software_sha256': {str(Path(p).relative_to(root)): _sha(p) for p in
                           [root / 'src/wc_navigation/rebuild.py', root / 'src/wc_navigation/rebuild_cli.py',
                            Path(compare_native.__file__)]},
    }
    rclpy = None
    initialized = False

    def checkpoint(stage):
        policy.check()
        result['stage'] = stage
        result['updated_utc'] = datetime.now(timezone.utc).isoformat()
        _write(output / 'progress.json', result)

    with _domain_lock(root) as lock:
        output.mkdir(parents=True, exist_ok=False)
        frontends, native = output / 'frontends', output / 'native'
        frontends.mkdir()
        native.mkdir()
        result['lock_file'] = str(lock)
        os.environ['ROS_LOG_DIR'] = str(output / 'ros_logs')
        result['ros_log_dir'] = os.environ['ROS_LOG_DIR']
        print(json.dumps({'stage': 'REBUILD_STARTED', 'output': str(output),
                          'progress_path': result['progress_path']}, ensure_ascii=True), flush=True)
        try:
            checkpoint('DERIVING_FRONTENDS')
            indexes = {}
            for variant in args.variants:
                manifest = derive_left_frontend(source, frontends / variant, variant)
                result['derivations'][variant] = manifest
                indexes[variant] = compare_native.load_index(frontends / variant)
                checkpoint('DERIVED_' + variant)
            common = validate_common_frames(result['derivations'], indexes)
            selected = compare_native.subsample(common, args.input_hz, 0)
            result['common_input_frames'] = len(common)
            result['common_input_stamp_sha256'] = _stamp_hash(common)
            result['selected_frames'] = len(selected)
            result['selected_stamp_sha256'] = _stamp_hash(selected)
            result['selected_stamps_ns'] = selected
            checkpoint('NATIVE_INITIALIZING')
            import rclpy as ros_client
            rclpy = ros_client
            rclpy.init(args=[])
            initialized = True
            from types import SimpleNamespace
            native_args = SimpleNamespace(input_hz=args.input_hz, wall_interval=args.wall_interval,
                                          frame_timeout=args.frame_timeout)
            for index, variant in enumerate(args.variants):
                checkpoint('NATIVE_RUNNING_' + variant)
                run_directory = native / variant
                try:
                    report = compare_native.run_cell(
                        root, frontends, native, variant, indexes[variant], selected, native_args, rclpy,
                        storage_root=policy.archive_root,
                        native_parameter_overrides=result['derivations'][variant]['profile']['native_parameter_overrides'])
                    result['native_runs'][variant] = {
                        'status': report['status'], 'result_path': str(run_directory / 'result.json'),
                        'published_pairs': report.get('published_pairs'),
                        'acknowledged_pairs': report.get('acknowledged_pairs'),
                        'native_exit_code': report.get('native_exit_code'),
                        'wall_elapsed_s': report.get('wall_elapsed_s'),
                        'native_parameter_overrides': result['derivations'][variant]['profile']['native_parameter_overrides'],
                    }
                except Exception as exc:
                    saved = {}
                    report_path = run_directory / 'result.json'
                    if report_path.is_file():
                        try:
                            saved = json.loads(report_path.read_text(encoding='utf-8'))
                        except (ValueError, OSError):
                            pass
                    result['native_runs'][variant] = {
                        'status': 'FAIL', 'error': type(exc).__name__ + ': ' + str(exc),
                        'result_path': str(report_path), 'unreaped_owner_pid': saved.get('unreaped_owner_pid'),
                    }
                    if saved.get('unreaped_owner_pid'):
                        raise RuntimeError('owned native writer has not reaped; refusing to start another variant') from exc
                checkpoint('NATIVE_FINISHED_' + variant)
                if index + 1 < len(args.variants):
                    # The prior native node has been closed/reaped by run_cell.
                    # Allow its DDS graph entry to retire before the next cell's
                    # own foreign-node guard performs its discovery window.
                    time.sleep(2.0)
            expected = 'NATIVE_REPLAY_AND_EXPORT_COMPLETE_REQUIRES_VISUAL_REVIEW'
            passed = all(item.get('status') == expected for item in result['native_runs'].values())
            result['status'] = 'COMPLETE_REQUIRES_MAP_AND_LOW_OBSTACLE_REVIEW' if passed else 'FAILED_VARIANTS_RETAINED'
            result['source_metadata_unchanged'] = all(_sha(source / name) == h for name, h in result['source_metadata_sha256'].items())
            if not result['source_metadata_unchanged']:
                raise ValueError('source metadata changed during native rebuild')
        except BaseException as exc:
            result.update(status='INTERRUPTED' if isinstance(exc, KeyboardInterrupt) else 'FAIL',
                          error=type(exc).__name__ + ': ' + str(exc))
            (output / 'failure_traceback.log').write_text(traceback.format_exc(), encoding='utf-8')
        finally:
            if initialized and rclpy is not None and rclpy.ok():
                rclpy.shutdown()
            checkpoint('FINISHED')
            _write(output / 'result.json', result)
            print(json.dumps({k: result.get(k) for k in
                              ['status', 'output', 'result_path', 'progress_path', 'common_input_frames',
                               'selected_frames', 'native_runs', 'error', 'hardware_started', 'navigation_validated']},
                             ensure_ascii=True, allow_nan=False), flush=True)
    return 0 if result['status'] == 'COMPLETE_REQUIRES_MAP_AND_LOW_OBSTACLE_REVIEW' else 1


if __name__ == '__main__':
    raise SystemExit(main())
