"""Orin operator entry for offline, source-bound lidar correspondence picking."""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import re

from .cli import ROOT, target
from .storage_policy import resolve_storage_path

LEGACY_INPUT = Path('reports/calibration/stability_single_20260913T011555Z/prepared.json')
INPUT_POINTER = Path('state/point_picker_input.json')


def _normal_project_file(path, root):
    try:
        source = resolve_storage_path(root, path)
    except (ValueError, OSError) as error:
        raise RuntimeError('Input must be a normal prepared file inside this project or configured archive: '+str(error)) from error
    if any(p.is_symlink() or (hasattr(p, 'is_junction') and p.is_junction()) for p in (source, *source.parents)) or \
            not source.is_file():
        raise RuntimeError('Input must be a normal prepared file inside this project')
    return source


def _unique_object(items):
    value = {}
    for key, item in items:
        if key in value:
            raise ValueError('duplicate JSON key')
        value[key] = item
    return value


def _reject_constant(value):
    raise ValueError('nonfinite JSON constants are not accepted')


def resolve_input(explicit_input=None, scene_index=0):
    """Resolve a read-only default pointer; a bad existing pointer never falls back."""
    root = ROOT.absolute()
    if explicit_input is not None:
        explicit = Path(explicit_input)
        return _normal_project_file(explicit if explicit.is_absolute() else root/explicit, root)
    pointer = root/INPUT_POINTER
    # Check links before exists(): a dangling pointer is not an absent pointer.
    if any(p.is_symlink() or (hasattr(p, 'is_junction') and p.is_junction()) for p in (pointer, *pointer.parents)):
        raise RuntimeError('Default picker input manifest must not traverse symlinks or junctions')
    if not pointer.exists():
        return _normal_project_file(root/LEGACY_INPUT, root)
    try:
        if not pointer.is_file() or pointer.stat().st_size > 16384:
            raise ValueError('manifest must be a regular JSON file of at most 16384 bytes')
        with pointer.open('rb') as stream:
            raw = stream.read(16385)
        if len(raw) > 16384:
            raise ValueError('manifest exceeds byte limit')
        manifest = json.loads(raw, object_pairs_hook=_unique_object, parse_constant=_reject_constant)
        if not isinstance(manifest, dict) or type(manifest.get('schema_version')) is not int or manifest['schema_version'] != 1 or \
                manifest.get('status') != 'READY_FOR_OFFLINE_PICKING':
            raise ValueError('schema_version=1 and READY_FOR_OFFLINE_PICKING required')
        relative = manifest.get('prepared_input')
        if not isinstance(relative, str) or not relative or '\\' in relative or \
                PurePosixPath(relative).is_absolute() or PureWindowsPath(relative).drive or \
                '..' in relative.split('/') or PurePosixPath(relative).parts[0] != 'reports' or \
                PurePosixPath(relative).suffix.lower() != '.json':
            raise ValueError('prepared_input must be a project-relative reports/... JSON path without parent traversal')
        source = _normal_project_file(root/relative, root)
        if not source.resolve().is_relative_to(resolve_storage_path(root, 'reports').resolve()):
            raise ValueError('prepared input must remain inside project reports')
        expected = manifest.get('prepared_file_sha256')
        if not isinstance(expected, str) or re.fullmatch('[0-9a-fA-F]{64}', expected) is None:
            raise ValueError('prepared_file_sha256 must be a SHA256 hex digest')
        if source.stat().st_size > 128_000_000:
            raise ValueError('prepared input exceeds the picker byte budget')
        with source.open('rb') as stream:
            content = stream.read(128_000_001)
        if len(content) > 128_000_000 or hashlib.sha256(content).hexdigest() != expected.lower():
            raise ValueError('prepared input file SHA256 does not match the manifest')
        if 'scene_id' in manifest:
            expected_scene = manifest['scene_id']
            if not isinstance(expected_scene, str) or not expected_scene.strip() or type(scene_index) is not int or scene_index < 0:
                raise ValueError('scene_id must identify the requested training scene')
            prepared = json.loads(content, object_pairs_hook=_unique_object, parse_constant=_reject_constant)
            training = prepared.get('training') if isinstance(prepared, dict) else None
            if not isinstance(training, list) or scene_index >= len(training) or not isinstance(training[scene_index], dict) or \
                    training[scene_index].get('id') != expected_scene:
                raise ValueError('prepared training scene does not match manifest scene_id')
        return source
    except (OSError, ValueError, TypeError, KeyError, IndexError, RuntimeError, RecursionError) as error:
        raise RuntimeError('Default picker input manifest rejected; no legacy fallback: '+str(error)) from error


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path,
        help='Explicit project input; otherwise use state/point_picker_input.json when present')
    parser.add_argument('--scene-index', type=int, default=0)
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--duration', type=int, default=3600)
    parser.add_argument('--no-browser', action='store_true')
    parser.add_argument('--alignment', action='store_true', help='Open whole-cloud adjustment and offline ICP')
    args = parser.parse_args(argv)
    target()
    if Path.cwd().resolve() != ROOT:
        raise RuntimeError('Run from the selected project root')
    source = resolve_input(args.input, args.scene_index)
    open_browser = not args.no_browser
    if open_browser:
        from .sensor_viewer import desktop_environment, confirm_display
        try:
            env = desktop_environment()
            confirm_display(env)
            os.environ.update({k: env[k] for k in ('DISPLAY', 'XAUTHORITY', 'DBUS_SESSION_BUS_ADDRESS') if k in env})
        except RuntimeError as error:
            print('图形会话不可用；仅启动本机选点服务。'+str(error), flush=True)
            open_browser = False
    from wc_calibration.picker import main as serve
    options = ['--input', str(source), '--output-root', str(resolve_storage_path(ROOT, 'data/calibration/manual_points')),
        '--scene-index', str(args.scene_index), '--port', str(args.port), '--duration', str(args.duration)]
    if open_browser:
        options.append('--open-browser')
    if args.alignment:
        options.append('--alignment')
    return serve(options)


if __name__ == '__main__':
    raise SystemExit(main())
