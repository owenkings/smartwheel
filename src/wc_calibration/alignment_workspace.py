"""Versioned offline rigid-cloud alignment; never installs calibration or TF."""
import copy
from datetime import datetime, timezone
import hashlib
import os
from pathlib import Path
import threading
import uuid

import numpy as np

from .core import CalibrationError, _hash, validate_transform
from .picker import _json_bytes, _json_loads, _no_links
from .preview_icp import evaluate_preview, refine_preview


class AlignmentBusy(RuntimeError):
    pass


class AlignmentWorkspace:
    def __init__(self, session):
        self.session = session
        self.output_root = session.output_root.parent / 'alignment_candidates'
        self._lock = threading.Lock()

    def _completed(self, root, kind):
        """Only load complete hash-bound records belonging to this frozen scene."""
        _no_links(root)
        if not root.exists():
            return None
        for directory in sorted(root.iterdir(), key=lambda p: p.name, reverse=True)[:256]:
            try:
                _no_links(directory)
                if not directory.is_dir():
                    continue
                manifest_path = directory/'manifest.json'
                _no_links(manifest_path)
                if manifest_path.stat().st_size > 100_000:
                    continue
                manifest = _json_loads(manifest_path.read_bytes())
                if not isinstance(manifest, dict) or manifest.get('kind') != kind or manifest.get('prepared_input_hash') != self.session.input_hash or \
                        manifest.get('scene_id') != self.session._scene['scene_id'] or manifest.get('live_eligible') is not False:
                    continue
                files = manifest.get('files', {})
                required = {'result.json', 'selected_pairs.json'} if kind == 'manual_picker_export' else {'result.json', 'correspondences.json'}
                if not isinstance(files, dict) or set(files) != required:
                    continue
                loaded = {}
                for name, metadata in files.items():
                    if not isinstance(metadata, dict):
                        raise ValueError('invalid file metadata')
                    path = directory/name
                    _no_links(path)
                    if path.stat().st_size > 10_000_000:
                        raise ValueError('record too large')
                    raw = path.read_bytes()
                    if hashlib.sha256(raw).hexdigest() != metadata['sha256'] or len(raw) != metadata['bytes']:
                        raise ValueError('record digest mismatch')
                    loaded[name] = _json_loads(raw)
                result = loaded['result.json']
                if not isinstance(result, dict):
                    continue
                if kind == 'manual_picker_export':
                    selection = loaded['selected_pairs.json']
                    if not isinstance(selection, dict) or not isinstance(selection.get('provenance'), dict) or \
                            selection['provenance'].get('prepared_input_hash') != self.session.input_hash or \
                            result.get('input_hash') != _hash(selection) or manifest.get('result_hash') != _hash(result):
                        continue
                elif result.get('prepared_input_hash') != self.session.input_hash or result.get('scene_id') != self.session._scene['scene_id']:
                    continue
                key = 'initial_T_left_right' if kind == 'manual_picker_export' else 'refined_T_left_right'
                validate_transform(result[key])
                if result.get('live_eligible') is not False:
                    continue
                return {'result': result, 'export_dir': str(directory)}
            except (OSError, ValueError, KeyError, TypeError, RecursionError):
                continue
        return None

    def bootstrap(self):
        manual = self._completed(self.session.output_root, 'manual_picker_export')
        if manual:
            initial = manual['result']['initial_T_left_right']
            source = {'kind': 'saved_manual_initial', 'path': manual['export_dir'],
                      'status': manual['result']['status']}
        elif self.session._prepared_initial_T_left_right is not None:
            initial = copy.deepcopy(self.session._prepared_initial_T_left_right)
            source = {'kind': 'PREPARED_INPUT_INITIAL',
                      'path': self.session._prepared_input_path,
                      'field': 'initial_T_left_right', 'status': 'PREPARED_NOT_VALIDATED'}
        else:
            initial = np.eye(4).tolist()
            source = {'kind': 'identity_for_manual_adjustment_only',
                      'path': None, 'status': 'UNKNOWN_INSTALLATION'}
        return {'scene': self.session.scene(), 'initial_T_left_right': initial,
                'initial_source': source,
                'saved_candidate': self._completed(self.output_root, 'offline_alignment_export')}

    def export(self, body):
        if not isinstance(body, dict) or set(body) != {'input_hash', 'scene_id', 'initial_T_left_right', 'operation'}:
            raise CalibrationError('alignment accepts only source identity, rigid transform and operation')
        if body['input_hash'] != self.session.input_hash or body['scene_id'] != self.session._scene['scene_id']:
            raise CalibrationError('request does not belong to the frozen source scene')
        if body['operation'] not in ('refine', 'save_manual'):
            raise CalibrationError('operation must be refine or save_manual')
        initial = validate_transform(body['initial_T_left_right'])
        if np.linalg.norm(initial[:3, 3]) > 10:
            raise CalibrationError('preview translation exceeds the 10 metre work budget')
        if not self._lock.acquire(blocking=False):
            raise AlignmentBusy('another alignment is running; wait for its result')
        try:
            clouds = self.session._scene['clouds']
            xyz = {s: [p['xyz'] for p in clouds[s]] for s in ('left', 'right')}
            if body['operation'] == 'refine':
                result = refine_preview(xyz['left'], xyz['right'], initial)
            else:
                metrics = evaluate_preview(xyz['left'], xyz['right'], initial)
                result = {'schema_version': 1, 'kind': 'manual_transform_candidate',
                    'status': 'MANUAL_UNVALIDATED', 'initial_T_left_right': initial.tolist(),
                    'refined_T_left_right': initial.tolist(), 'initial_metrics': metrics,
                    'final_metrics': copy.deepcopy(metrics), 'transform_change': {'translation_m': 0., 'rotation_deg': 0.},
                    'policy': {'scale_fitted': False}, 'history': [],
                    'reasons': ['MANUAL_PLACEMENT_NOT_INDEPENDENTLY_VALIDATED'], 'correspondences': []}
            result = copy.deepcopy(result)
            validate_transform(result['refined_T_left_right'])
            pairs = result.pop('correspondences', [])
            mapped = []
            for pair in pairs:
                li, ri = pair['left_index'], pair['right_index']
                if type(li) is not int or type(ri) is not int or not 0 <= li < len(clouds['left']) or not 0 <= ri < len(clouds['right']):
                    raise CalibrationError('ICP correspondence index outside frozen original clouds')
                mapped.append({'left_id': clouds['left'][li]['id'], 'right_id': clouds['right'][ri]['id'],
                               'distance_m': pair['distance_m'], 'verified': False})
            result.update({'live_eligible': False, 'independent_validation_performed': False,
                'prepared_input_hash': self.session.input_hash, 'scene_id': self.session._scene['scene_id'],
                'source_mode': self.session._scene['source_mode'], 'sensor_ids': self.session._scene['sensor_ids'],
                'units': 'm', 'transform_definition': 'p_left = R_left_right * p_right + t_left_right',
                'provenance': copy.deepcopy(self.session._provenance),
                'correspondence_count': len(mapped),
                'limitations': ['Single scene candidate only; no independent accuracy or extrinsic validation.',
                    'Nearest neighbours are geometric candidates, not verified identical physical points.',
                    'No sensor, clock, motor, default input or formal calibration configuration is modified.']})
            correspondence_file = {'schema_version': 1, 'kind': 'unverified_nearest_neighbours',
                'prepared_input_hash': self.session.input_hash, 'scene_id': self.session._scene['scene_id'],
                'pairs': mapped, 'live_eligible': False}
            _no_links(self.output_root)
            self.output_root.mkdir(parents=True, exist_ok=True)
            directory = self.output_root / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ_')+uuid.uuid4().hex)
            directory.mkdir()
            manifest = {'schema_version': 1, 'kind': 'offline_alignment_export',
                'operation': body['operation'], 'status': 'SAVED_NOT_VALIDATED',
                'prepared_input_hash': self.session.input_hash, 'scene_id': self.session._scene['scene_id'],
                'created_utc': datetime.now(timezone.utc).isoformat(),
                'live_eligible': False, 'independent_validation_performed': False, 'files': {}}
            for name, value in {'result.json': result, 'correspondences.json': correspondence_file}.items():
                raw = _json_bytes(value)
                self._write(directory, name, raw)
                manifest['files'][name] = {'sha256': hashlib.sha256(raw).hexdigest(), 'bytes': len(raw)}
            self._write(directory, 'manifest.json', _json_bytes(manifest))
            return {'result': result, 'export_dir': str(directory)}
        finally:
            self._lock.release()

    @staticmethod
    def _write(directory, name, raw):
        temporary = directory/('.'+name+'.partial')
        with temporary.open('xb') as stream:
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, directory/name)
