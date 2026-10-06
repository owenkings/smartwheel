"""Loopback-only source-bound point picking; never operates sensors or publishes TF."""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import secrets
import sys
import threading
import time
import uuid
import webbrowser

from .alignment import manual_initial
from .core import CalibrationError, _hash, validate_transform

MAX_INPUT_BYTES = 128_000_000
MAX_POINTS = 20_000
MAX_PAIRS = 10_000
MAX_BODY_BYTES = 1_048_576
ASSET_ROOT = Path(__file__).parent / 'picker_assets'
ASSETS = {'/': ('index.html', 'text/html; charset=utf-8'),
          '/index.html': ('index.html', 'text/html; charset=utf-8'),
          '/picker.js': ('picker.js', 'text/javascript; charset=utf-8'),
          '/picker.css': ('picker.css', 'text/css; charset=utf-8'),
          '/alignment': ('alignment.html', 'text/html; charset=utf-8'),
          '/alignment.js': ('alignment.js', 'text/javascript; charset=utf-8'),
          '/alignment.css': ('alignment.css', 'text/css; charset=utf-8')}


def _invalid_constant(value):
    raise CalibrationError('nonfinite JSON constants are not accepted')


def _unique_object(items):
    result = {}
    for key, value in items:
        if key in result:
            raise CalibrationError('duplicate JSON object keys are not accepted')
        result[key] = value
    return result


def _json_loads(value):
    return json.loads(value, parse_constant=_invalid_constant, object_pairs_hook=_unique_object)


def _json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode('utf-8')


def _no_links(path):
    if any(p.is_symlink() or (hasattr(p, 'is_junction') and p.is_junction())
           for p in (path, *path.parents)):
        raise CalibrationError('output and asset paths must not traverse symbolic links or junctions')


class PickerSession:
    """Freeze one prepared scene in memory; POSTs carry indices, never coordinates."""
    def __init__(self, input_path, output_root, scene_index=0):
        source = Path(input_path)
        if not source.is_file() or source.stat().st_size > MAX_INPUT_BYTES:
            raise CalibrationError('an existing prepared JSON of at most 128 MB is required')
        with source.open('rb') as stream:
            raw = stream.read(MAX_INPUT_BYTES + 1)
        if len(raw) > MAX_INPUT_BYTES:
            raise CalibrationError('input JSON exceeds the byte budget')
        data = _json_loads(raw.decode('utf-8'))
        if not isinstance(data, dict) or type(data.get('schema_version')) is not int or data['schema_version'] != 1 or data.get('status') != 'PREPARED_NOT_VALIDATED':
            raise CalibrationError('prepared v1 observations required')
        if data.get('source_mode') not in ('real', 'synthetic'):
            raise CalibrationError('explicit source_mode required')
        initial = data.get('initial_T_left_right')
        # Freeze only the optional input pose, alongside the already frozen scene.
        # A later input-file edit must not change this session's reset position.
        self._prepared_initial_T_left_right = (
            None if initial is None else validate_transform(initial).tolist())
        self._prepared_input_path = str(source.absolute())
        training = data.get('training')
        if type(scene_index) is not int or not isinstance(training, list) or not 0 <= scene_index < len(training):
            raise CalibrationError('scene-index must identify an existing training scene')
        scene = training[scene_index]
        ids = data.get('sensor_ids')
        if not isinstance(ids, dict) or set(ids) != {'left', 'right'} or any(not isinstance(v, str) or not v.strip() for v in ids.values()) or ids['left'] == ids['right']:
            raise CalibrationError('distinct explicit left/right sensor identities required')
        if not isinstance(scene, dict) or not isinstance(scene.get('id'), str) or not scene['id'].strip():
            raise CalibrationError('explicit scene id required')
        metadata = scene.get('selected_raw_frames', scene.get('input_files'))
        if not isinstance(metadata, dict) or set(metadata) != {'left', 'right'} or any(not isinstance(v, dict) for v in metadata.values()):
            raise CalibrationError('prepared scene source metadata required')
        units = data.get('units', 'm' if all(v.get('units') == 'm' for v in metadata.values()) else None)
        conventions = data.get('coordinate_conventions', {s: metadata[s].get('coordinate_convention') for s in ids})
        if units != 'm' or conventions != {'left': 'FLU', 'right': 'FLU'}:
            raise CalibrationError('explicit metre/FLU arrays required; no implicit scale or axis conversion')
        for side in ids:
            meta = metadata[side]
            if meta.get('coordinate_convention') != 'FLU' or meta.get('units') != 'm' or meta.get('sensor_id', ids[side]) != ids[side]:
                raise CalibrationError('scene source metadata contradicts metre/FLU sensor identity')
            if 'raw_key' in meta and (not isinstance(meta['raw_key'], list) or len(meta['raw_key']) != 4 or meta['raw_key'][1] != ids[side]):
                raise CalibrationError('raw frame identity contradicts selected sensor')
        self.input_hash = _hash(data)
        self._provenance = {'prepared_input_hash': self.input_hash,
            'prepared_file_sha256': hashlib.sha256(raw).hexdigest(), 'scene_index': scene_index,
            'scene_id': scene['id'], 'source_metadata': copy.deepcopy(metadata),
            'input_preparation': copy.deepcopy(data.get('input_preparation')),
            'time_quality': scene.get('time_quality', 'UNVALIDATED'),
            'pair_dt_ns': scene.get('pair_dt_ns'), 'static_suggestion': copy.deepcopy(scene.get('static_suggestion'))}
        self._points = {}
        clouds, excluded = {}, {}
        for side in ('left', 'right'):
            rows = scene.get(side)
            if not isinstance(rows, list) or not 1 <= len(rows) <= MAX_POINTS:
                raise CalibrationError('each original cloud must contain 1..20000 rows; no implicit decimation')
            finite = {}
            for index, row in enumerate(rows):
                if not isinstance(row, list) or len(row) != 3 or any(v is not None and type(v) not in (int, float) for v in row):
                    raise CalibrationError('XYZ rows must contain exactly three numeric or null coordinates')
                try:
                    finite_row = all(v is not None and math.isfinite(v) for v in row)
                except OverflowError as error:
                    raise CalibrationError('coordinate exceeds the finite numeric representation') from error
                if not finite_row:
                    continue
                finite[index] = tuple(float(v) for v in row)
            if not finite:
                raise CalibrationError('each side needs finite selectable observations')
            self._points[side] = finite
            clouds[side] = [{'id': index, 'xyz': list(xyz)} for index, xyz in finite.items()]
            excluded[side] = {'original_rows': len(rows), 'selectable_rows': len(finite),
                              'excluded_nonfinite_rows': len(rows)-len(finite)}
        self._scene = {'input_hash': self.input_hash, 'scene_id': scene['id'],
            'source_mode': data['source_mode'], 'sensor_ids': copy.deepcopy(ids), 'units': 'm',
            'coordinate_conventions': dict(conventions), 'time_quality': self._provenance['time_quality'],
            'clouds': clouds, 'point_counts': excluded, 'live_eligible': False}
        if 'level_reference' in scene:
            from .level_reference import validate_level_reference
            reference = validate_level_reference(scene['level_reference'], scene['id'], metadata)
            self._scene['level_reference'] = copy.deepcopy(reference)
            self._provenance['level_reference'] = copy.deepcopy(reference)
        requested = Path(output_root).absolute()
        _no_links(requested)
        requested.mkdir(parents=True, exist_ok=True)
        self.output_root = requested.resolve(strict=True)
        self.token = secrets.token_urlsafe(32)
        self._alignment_lock = threading.Lock()
        self._alignment = None

    def alignment_workspace(self):
        with self._alignment_lock:
            if self._alignment is None:
                from .alignment_workspace import AlignmentWorkspace
                self._alignment = AlignmentWorkspace(self)
            return self._alignment

    def scene(self):
        return dict(copy.deepcopy(self._scene), token=self.token)

    def selection(self, body, *, solve=False):
        if not isinstance(body, dict) or set(body) != {'input_hash', 'scene_id', 'pairs'}:
            raise CalibrationError('request accepts only input_hash, scene_id and index pairs')
        if body['input_hash'] != self.input_hash or body['scene_id'] != self._scene['scene_id']:
            raise CalibrationError('request does not belong to the frozen source scene')
        pairs = body['pairs']
        if not isinstance(pairs, list) or not (3 if solve else 1) <= len(pairs) <= MAX_PAIRS:
            raise CalibrationError('solve needs 3..10000 pairs; export needs 1..10000 pairs')
        seen = {'left': set(), 'right': set()}
        selected = {'left': [], 'right': []}
        for pair in pairs:
            if not isinstance(pair, dict) or set(pair) != {'left_id', 'right_id'}:
                raise CalibrationError('each pair accepts only left_id and right_id')
            for side in ('left', 'right'):
                index = pair[side+'_id']
                if type(index) is not int or index not in self._points[side] or index in seen[side]:
                    raise CalibrationError('duplicate, out-of-range or nonfinite point index')
                seen[side].add(index)
                selected[side].append(list(self._points[side][index]))
        if any(len({tuple(row) for row in selected[side]}) != len(pairs) for side in selected):
            raise CalibrationError('duplicate XYZ observations do not provide independent matches')
        return {'schema_version': 1, 'kind': 'manual_point_selection', 'status': 'SELECTED_NOT_VALIDATED',
            'source_mode': self._scene['source_mode'], 'sensor_ids': copy.deepcopy(self._scene['sensor_ids']),
            'units': 'm', 'coordinate_conventions': dict(self._scene['coordinate_conventions']),
            **selected, 'pairs': copy.deepcopy(pairs), 'provenance': copy.deepcopy(self._provenance),
            'live_eligible': False, 'independent_validation_performed': False}

    def export(self, body, *, solve=False):
        selection = self.selection(body, solve=solve)
        # Defaults and degeneracy thresholds are owned by the existing reviewed solver.
        result = manual_initial(selection) if solve else None
        _no_links(self.output_root)
        version = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ_') + uuid.uuid4().hex
        target = self.output_root / version
        target.mkdir()  # Exclusive reservation; never reuse an earlier export.
        files = {'selected_pairs.json': selection}
        if result is not None:
            files['result.json'] = result
        manifest = {'schema_version': 1, 'kind': 'manual_picker_export', 'status': 'SAVED_NOT_VALIDATED',
            'operation': 'solve' if solve else 'export', 'created_utc': datetime.now(timezone.utc).isoformat(),
            'prepared_input_hash': self.input_hash, 'scene_id': self._scene['scene_id'],
            'source_mode': self._scene['source_mode'], 'selection_hash': _hash(selection),
            'result_hash': _hash(result) if result is not None else None,
            'solver_status': result['status'] if result is not None else None,
            'live_eligible': False, 'independent_validation_performed': False, 'files': {}}
        # Each file becomes visible only after a complete flushed write. The
        # exclusive UUID directory keeps earlier exports outside this operation.
        for name, value in files.items():
            content = _json_bytes(value)
            temporary = target / ('.'+name+'.partial')
            with temporary.open('xb') as stream:
                stream.write(content); stream.flush(); os.fsync(stream.fileno())
            os.replace(temporary, target / name)
            manifest['files'][name] = {'sha256': hashlib.sha256(content).hexdigest(), 'bytes': len(content)}
        if result is not None:
            # Only solve has a result and completion manifest. Export preserves
            # just selected_pairs.json, as specified by the public API.
            temporary = target / '.manifest.json.partial'
            with temporary.open('xb') as stream:
                stream.write(_json_bytes(manifest)); stream.flush(); os.fsync(stream.fileno())
            os.replace(temporary, target / 'manifest.json')
        response = {'selection': selection, 'export_dir': str(target)}
        if result is not None:
            response['result'] = result
        return response


class PickerServer(ThreadingHTTPServer):
    allow_reuse_address = False
    daemon_threads = True
    block_on_close = False

    def __init__(self, session, port=8766):
        if type(port) is not int or not 0 <= port <= 65535:
            raise CalibrationError('port must be an integer in 0..65535')
        self.session = session
        self._slots = threading.BoundedSemaphore(4)
        super().__init__(('127.0.0.1', port), PickerHandler)
        self.timeout = .2
        self.origin = 'http://127.0.0.1:' + str(self.server_port)

    def get_request(self):
        connection, address = super().get_request()
        connection.settimeout(2.0)
        return connection, address

    def process_request(self, request, client_address):
        if not self._slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except BaseException:
            self._slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self._slots.release()


class PickerHandler(BaseHTTPRequestHandler):
    # HTTP/1.0 closes each connection. At most four daemon workers keep the CLI
    # duration bounded even when a local client trickles an incomplete request.
    server_version = 'WheelchairOfflinePicker'
    sys_version = ''

    def log_message(self, *args):
        pass  # Never log request bodies, headers or the ephemeral token.

    def _reply(self, status, content, content_type='application/json; charset=utf-8'):
        if not isinstance(content, bytes):
            content = _json_bytes(content)
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(content)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Referrer-Policy', 'no-referrer')
        self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; object-src 'none'; base-uri 'none'; frame-ancestors 'none'; form-action 'none'")
        self.end_headers()
        self.wfile.write(content)

    def _authorized(self, *, post=False):
        def one(name):
            values = self.headers.get_all(name, [])
            return values[0] if len(values) == 1 else None
        host = one('Host')
        origin = one('Origin')
        valid = host == self.server.origin.removeprefix('http://')
        if self.headers.get_all('Origin'):
            valid = valid and origin == self.server.origin
        valid = valid and self.headers.get('Sec-Fetch-Site') not in ('cross-site',)
        if post:
            token = one('X-Picker-Token')
            valid = valid and isinstance(token, str) and token.isascii() and secrets.compare_digest(token, self.server.session.token)
        if not valid:
            self._reply(403, {'error': 'request origin, host or token rejected'})
        return valid

    def do_GET(self):
        if not self._authorized():
            return
        if self.path == '/api/scene':
            self._reply(200, self.server.session.scene())
        elif self.path == '/api/alignment-bootstrap':
            try:
                self._reply(200, self.server.session.alignment_workspace().bootstrap())
            except (CalibrationError, OSError, ValueError, TypeError):
                self._reply(422, {'error': 'alignment source or previous export unavailable'})
        elif self.path in ASSETS:
            name, kind = ASSETS[self.path]
            path = ASSET_ROOT / name
            try:
                _no_links(path.absolute())
                if path.stat().st_size > 2_000_000:
                    raise CalibrationError('asset exceeds size budget')
                self._reply(200, path.read_bytes(), kind)
            except (OSError, CalibrationError):
                self._reply(503, {'error': 'fixed picker asset unavailable'})
        else:
            self._reply(404, {'error': 'route not found'})

    def do_POST(self):
        if not self._authorized(post=True):
            return
        if self.path not in ('/api/solve', '/api/export', '/api/alignment'):
            self._reply(404, {'error': 'route not found'})
            return
        lengths = self.headers.get_all('Content-Length', [])
        if self.headers.get_all('Transfer-Encoding') or len(lengths) != 1 or len(lengths[0]) > 8 or not lengths[0].isascii() or not lengths[0].isdigit():
            self._reply(400, {'error': 'one explicit Content-Length required; transfer encoding refused'})
            return
        length = int(lengths[0])
        if not 0 < length <= MAX_BODY_BYTES:
            self._reply(413, {'error': 'request body outside 1..1048576 byte budget'})
            return
        if self.headers.get('Content-Type', '').split(';')[0].strip().lower() != 'application/json':
            self._reply(415, {'error': 'application/json required'})
            return
        try:
            payload = self.rfile.read(length)
            if len(payload) != length:
                raise CalibrationError('incomplete request body')
            body = _json_loads(payload.decode('utf-8'))
        except (ValueError, UnicodeError, CalibrationError, TimeoutError, RecursionError):
            self._reply(400, {'error': 'invalid, incomplete or timed-out JSON body'})
            return
        try:
            if self.path == '/api/alignment':
                from .alignment_workspace import AlignmentBusy
                try:
                    response = self.server.session.alignment_workspace().export(body)
                except AlignmentBusy as error:
                    self._reply(409, {'error': str(error)})
                    return
            else:
                response = self.server.session.export(body, solve=self.path == '/api/solve')
        except (CalibrationError, ValueError, TypeError, OverflowError) as error:
            self._reply(422, {'error': str(error)})
        except OSError:
            self._reply(500, {'error': 'export failed; do not treat a partial directory as a completed operation'})
        else:
            self._reply(200, response)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--scene-index', type=int, default=0)
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--open-browser', action='store_true')
    parser.add_argument('--alignment', action='store_true', help='Open offline rigid cloud adjustment and ICP page')
    parser.add_argument('--duration', type=int, default=3600)
    args = parser.parse_args(argv)
    try:
        if not 1 <= args.duration <= 43200:
            raise CalibrationError('duration must be 1..43200 seconds')
        session = PickerSession(args.input, args.output_root, args.scene_index)
        with PickerServer(session, args.port) as server:
            url = server.origin + ('/alignment' if args.alignment else '/')
            print(json.dumps({'url': url, 'input_hash': session.input_hash,
                'status': 'OFFLINE_ALIGNMENT_ONLY' if args.alignment else 'OFFLINE_SELECTION_ONLY', 'live_eligible': False}), flush=True)
            if args.open_browser:
                webbrowser.open(url, new=2)
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                server.handle_request()
        return 0
    except KeyboardInterrupt:
        return 0
    except (CalibrationError, OSError, ValueError, TypeError, RecursionError) as error:
        print(json.dumps({'error': type(error).__name__, 'detail': str(error)}), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
