"""Isolated, loopback-only multi-pose plane calibration from completed captures.

This module neither starts acquisition nor imports the mapping job controller.
Every change is saved through StoragePolicy; candidate exports never install TF.
"""
import argparse
import copy
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import secrets
import threading
import time

import numpy as np

from .core import _hash
from .plane_calibration import fit_plane, solve_planes
from .plane_capture import load_capture, assert_sources
from wc_runtime.source_archive import atomic_json
from wc_runtime.storage_policy import StoragePolicy


MAX_POSES = 16
ASSETS = {'/': ('index.html', 'text/html; charset=utf-8'),
          '/plane.js': ('plane.js', 'text/javascript; charset=utf-8'),
          '/plane.css': ('plane.css', 'text/css; charset=utf-8')}
ASSET_ROOT = Path(__file__).parent / 'plane_assets'


def require(ok, message):
    if not ok:
        raise ValueError(message)


def no_links(path):
    require('..' not in path.parts and not any(
        p.is_symlink() or getattr(p, 'is_junction', lambda: False)()
        for p in (path, *path.parents)), '路径不可包含软链接或上级跳转')


def json_loads(raw):
    def unique(rows):
        obj = {}
        for k, v in rows:
            require(k not in obj, 'JSON 包含重复字段')
            obj[k] = v
        return obj
    def invalid(value):
        raise ValueError('JSON 不接受非有限数值：' + value)
    return json.loads(raw, object_pairs_hook=unique, parse_constant=invalid)


def roi_indices(roi, height, width):
    require(isinstance(roi, list) and len(roi) == 4 and all(type(v) is int for v in roi),
            '框选范围须为四个整数原始行列值')
    r0, c0, r1, c1 = roi
    require(0 <= r0 <= r1 < height and 0 <= c0 <= c1 < width, '框选范围超出原始图像')
    rr, cc = np.mgrid[r0:r1+1, c0:c1+1]
    return (rr*width+cc).ravel()


def finite(points):
    points = np.asarray(points, float)
    return points[np.isfinite(points).all(1) & (np.linalg.norm(points, axis=1) > 0)]


class PlaneSession:
    def __init__(self, project_root, *, resume=None, loader=load_capture, source_check=assert_sources):
        self.project_root = str(Path(project_root).absolute())
        self.policy = StoragePolicy(self.project_root)
        self.policy.check()
        self._loader, self._source_check = loader, source_check
        self.token = secrets.token_urlsafe(32)
        self.lock = threading.RLock()
        self.revision, self.poses, self.result, self.icp = 0, [], None, None
        self.captures = {}
        self._owner_lock = None
        parent = self.policy.resolve('data/calibration/plane_workbench')
        if resume is not None:
            path = self.policy.resolve(str(resume))
            no_links(path)
            require(path.name == 'workspace.json' and path.parent.parent == parent,
                    '只能恢复此平面工作台的 workspace.json')
            require(path.stat().st_size < 16_000_000, '工作区清单过大')
            saved = json_loads(path.read_text(encoding='utf-8'))
            require(saved['kind'] == 'offline_plane_workspace' and saved['schema_version'] == 1,
                    '不是平面工作台清单')
            self.output_root, self.session_id = path.parent, saved['session_id']
            require(self.output_root.name == self.session_id and len(saved['poses']) <= MAX_POSES,
                    '工作区身份或姿态数无效')
            self._claim()
            for record in saved['poses']:
                require(record['id'].isalnum() and record['role'] in ('train', 'holdout'), '姿态清单无效')
                capture = self._loader(self.project_root, record['dataset'])
                require(record['capture_id'] == capture['capture_id'] and
                        record['data_hash'] == _hash({'clouds': capture['clouds'],
                                                     'amplitude': capture['amplitude']}),
                        '恢复时原始来源或聚合数据发生改变')
                require(_hash(record['provenance']) == _hash(capture['provenance']) and
                        record['sensor_ids'] == capture['sensor_ids'] and
                        record['source_mode'] == capture['source_mode'],
                        '恢复时绑定的原始文件、存储配置或雷达身份发生改变')
                require(capture['capture_id'] not in self.captures, '恢复清单重复使用同一录制')
                pose = self._new_pose(capture, record['label'], record['role'], record['id'])
                for side in ('left', 'right'):
                    if record['rois'][side] is not None:
                        roi_indices(record['rois'][side], pose['height'], pose['width'])
                # Saved coefficients are a report, not trusted computation input.
                # Preserve the rectangles, and refit from the verified raw cloud.
                pose.update(rois=record['rois'], fit=None, same_face_confirmed=False)
                self.poses.append(pose)
                self.captures[capture['capture_id']] = capture
            self.revision = int(saved['revision']) + 1
            # A restored candidate must be recalculated in this process.
            self._persist()
        else:
            self.session_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_')+secrets.token_hex(6)
            self.output_root = parent / self.session_id
            no_links(self.output_root)
            self.output_root.mkdir(parents=True, exist_ok=False)
            self._claim()
            self._persist()

    def _claim(self):
        # Runtime ownership stays on the local filesystem, not the exFAT USB.
        folder = Path(self.project_root)/'.phase1_runtime/plane_workbench'
        no_links(folder)
        folder.mkdir(parents=True, exist_ok=True)
        path = folder/(self.session_id+'.lock')
        no_links(path)
        stream = path.open('a+b')
        if os.name == 'posix':
            import fcntl
            try:
                fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                stream.close()
                raise ValueError('该平面工作区已由另一个服务打开；请使用其原页面')
        self._owner_lock = stream

    def close(self):
        if self._owner_lock is not None:
            self._owner_lock.close()
            self._owner_lock = None

    def _new_pose(self, capture, label, role, identity=None):
        return {'id': identity or secrets.token_hex(8), 'label': label, 'role': role,
                'capture_id': capture['capture_id'], 'dataset': capture['provenance']['dataset'],
                'source_mode': capture['source_mode'], 'sensor_ids': capture['sensor_ids'],
                'width': capture['width'], 'height': capture['height'],
                'clouds': capture['clouds'], 'amplitude': capture['amplitude'],
                'provenance': capture['provenance'],
                'data_hash': _hash({'clouds': capture['clouds'], 'amplitude': capture['amplitude']}),
                'rois': {'left': None, 'right': None}, 'fit': None,
                'same_face_confirmed': False}

    def _manifest(self):
        excluded = ('clouds', 'amplitude')
        return {'schema_version': 1, 'kind': 'offline_plane_workspace',
                'session_id': self.session_id, 'revision': self.revision,
                'poses': [{k: v for k, v in p.items() if k not in excluded} for p in self.poses],
                'result': self.result, 'icp': self.icp,
                'offline_only': True, 'live_eligible': False}

    def _persist(self):
        self._check_storage()
        no_links(self.output_root)
        atomic_json(self.output_root/'workspace.json', self._manifest())
        self._check_storage()

    def _check_storage(self):
        current = StoragePolicy(self.project_root)
        require(current.configuration_files == self.policy.configuration_files,
                '存储配置在工作区打开后发生变化，请关闭并重新打开工作台')
        self.policy.check()

    def state(self):
        with self.lock:
            return copy.deepcopy({'token': self.token, 'revision': self.revision,
                'poses': self.poses, 'result': self.result, 'icp': self.icp,
                'session_id': self.session_id, 'output_root': str(self.output_root),
                'workspace_path': str(self.output_root/'workspace.json'),
                'offline_only': True, 'live_eligible': False})

    def _pose(self, identity):
        matches = [p for p in self.poses if p['id'] == identity]
        require(len(matches) == 1, '请选择当前工作区的姿态')
        return matches[0]

    def _observations(self):
        require(self.poses, '请先导入已完成的静止录制')
        observations = []
        for p in self.poses:
            require(p['fit'] is not None, '每个保留姿态都须先框选两侧并拟合平面')
            self._source_check(self.captures[p['capture_id']])
            observations.append({'id': p['id'], 'label': p['label'], 'capture_id': p['capture_id'],
                'role': p['role'], 'left': p['fit']['left'], 'right': p['fit']['right'],
                'same_face_confirmed': p['same_face_confirmed']})
        return observations

    def _fit(self, pose):
        capture = self.captures[pose['capture_id']]
        self._source_check(capture)
        fits = {}
        for side in ('left', 'right'):
            require(pose['rois'][side] is not None, '请先框选左右两侧的板面内部')
            ids = roi_indices(pose['rois'][side], pose['height'], pose['width'])
            values = np.asarray(pose['clouds'][side], float)[ids]
            valid = np.isfinite(values).all(1) & (np.linalg.norm(values, axis=1) > 0)
            ids, points = ids[valid], values[valid]
            fit = fit_plane(points)
            frames = np.asarray(capture['frames'][side], float)
            temporal = []
            for i, frame in enumerate(frames):
                item = fit_plane(finite(frame[ids]), seed=i)
                temporal.append({k: item.get(k) for k in
                    ('status', 'normal', 'd_m', 'rmse_m', 'all_p95_m', 'inlier_fraction', 'reasons')})
            good = [x for x in temporal if x['status'] == 'PASS']
            angles, offsets = [], []
            if fit.get('normal') is not None:
                normal = np.asarray(fit['normal'])
                for row in temporal:
                    if row.get('normal') is not None:
                        other = np.asarray(row['normal'])
                        dot = float(normal @ other)
                        sign = 1 if dot >= 0 else -1
                        angles.append(float(np.degrees(np.arccos(np.clip(abs(dot), -1, 1)))))
                        offsets.append(abs(float(row['d_m'])*sign-fit['d_m']))
            angle_p95 = float(np.percentile(angles, 95)) if angles else None
            offset_p95 = float(np.percentile(offsets, 95)) if offsets else None
            fit['temporal_quality'] = {'frames': len(temporal), 'pass_frames': len(good),
                'pass_fraction': len(good)/max(len(temporal), 1),
                'normal_angle_p95_deg': angle_p95, 'offset_p95_m': offset_p95,
                'per_frame': temporal, 'role': 'SAME_CAPTURE_REPEATABILITY_NOT_INDEPENDENT_VALIDATION'}
            if len(good) < .8*len(temporal) or not temporal:
                fit['reasons'].append('INSUFFICIENT_STABLE_FRAME_PLANES')
            if angle_p95 is None or angle_p95 > 3.0 or offset_p95 is None or offset_p95 > .03:
                fit['reasons'].append('TEMPORAL_PLANE_VARIATION_EXCEEDED')
            if fit['reasons']:
                fit['status'] = 'REJECTED'
            fit['roi_raw_row_col_inclusive'] = pose['rois'][side]
            fits[side] = fit
        self._source_check(capture)
        return fits

    def mutate(self, action, body):
        with self.lock:
            require(type(body.get('revision')) is int and body['revision'] == self.revision,
                    '页面版本已过期，请重新载入工作区；服务端已保存你的操作')
            self._check_storage()
            before = (copy.deepcopy(self.poses), copy.deepcopy(self.result),
                      copy.deepcopy(self.icp), self.revision, dict(self.captures))
            saved_path = None
            try:
                if action == 'import':
                    require(len(self.poses) < MAX_POSES, '单工作区最多 16 个姿态')
                    label = body.get('label', '').strip()
                    role = body.get('role')
                    require(1 <= len(label) <= 80 and role in ('train', 'holdout'), '请输入姿态名并选择用途')
                    require(isinstance(body.get('dataset'), str) and len(body['dataset']) <= 1024,
                            '请输入已完成录制的完整目录')
                    capture = self._loader(self.project_root, body['dataset'])
                    require(capture['capture_id'] not in self.captures,
                            '同一录制已经导入；连续帧不能冒充新的独立姿态')
                    if self.poses:
                        require(capture['sensor_ids'] == self.poses[0]['sensor_ids'], '左右雷达身份与已有姿态不一致')
                        require(capture['source_mode'] == self.poses[0]['source_mode'], '不能混合合成与实机数据')
                    self.poses.append(self._new_pose(capture, label, role))
                    self.captures[capture['capture_id']] = capture
                elif action in ('roi', 'role', 'remove', 'fit'):
                    pose = self._pose(body.get('pose_id'))
                    if action == 'roi':
                        side = body.get('side')
                        require(side in ('left', 'right'), '左右标识无效')
                        roi = body.get('roi')
                        if roi is not None:
                            roi_indices(roi, pose['height'], pose['width'])
                        pose['rois'][side] = None if roi is None else list(roi)
                        pose['fit'], pose['same_face_confirmed'] = None, False
                    elif action == 'role':
                        require(body.get('role') in ('train', 'holdout'), '姿态用途无效')
                        pose['role'] = body['role']
                    elif action == 'remove':
                        self.poses.remove(pose)
                        self.captures.pop(pose['capture_id'])
                    else:
                        require(type(body.get('same_face_confirmed')) is bool, '请明确确认左右是否为同一面')
                        pose['fit'] = self._fit(pose)
                        pose['same_face_confirmed'] = body['same_face_confirmed']
                elif action == 'solve':
                    self.result = solve_planes(self._observations())
                    self.icp = None
                elif action == 'icp':
                    require(self.result is not None and self.result.get('status') == 'CANDIDATE',
                            '须先获得通过训练门限的平面初值')
                    from .plane_icp import refine_planes_icp
                    observations = self._observations()
                    clouds = {}
                    for p in self.poses:
                        sides = {}
                        for side in ('left', 'right'):
                            ids = roi_indices(p['rois'][side], p['height'], p['width'])
                            values = finite(np.asarray(p['clouds'][side], float)[ids])
                            indices = p['fit'][side]['inlier_indices']
                            sides[side] = values[indices]
                        clouds[p['id']] = sides
                    self.icp = refine_planes_icp(observations, clouds, self.result['initial_T_left_right'])
                elif action == 'save':
                    require(self.result is not None, '请先计算平面标定结果')
                    self._observations()
                    export = self.output_root / ('result_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_')+
                                                  secrets.token_hex(4))
                    no_links(export)
                    export.mkdir(exist_ok=False)
                    atomic_json(export/'plane_calibration.json', {
                        'schema_version': 1, 'kind': 'offline_multi_pose_plane_candidate',
                        'source_mode': self.poses[0]['source_mode'], 'sensor_ids': self.poses[0]['sensor_ids'],
                        'units': 'm', 'coordinate_conventions': {'left': 'FLU', 'right': 'FLU'},
                        'transform_definition': 'p_left = R_left_right * p_right + t_left_right',
                        'session_id': self.session_id, 'workspace_revision': self.revision,
                        'observations': self._observations(), 'sources': [
                            {'pose_id': p['id'], 'label': p['label'], 'role': p['role'],
                             'rois': p['rois'], 'data_hash': p['data_hash'], 'provenance': p['provenance']}
                            for p in self.poses], 'plane_result': self.result, 'icp': self.icp,
                        'live_eligible': False, 'formal_configuration_modified': False})
                    self._check_storage()
                    self._observations()
                    atomic_json(export/'complete.json', {'status': 'COMPLETE_OFFLINE_EXPORT', 'live_eligible': False})
                    saved_path = str(export)
                else:
                    raise ValueError('未知操作')
                if action in ('import', 'roi', 'role', 'remove', 'fit'):
                    self.result, self.icp = None, None
                self.revision += 1
                self._persist()
            except BaseException:
                self.poses, self.result, self.icp, self.revision, self.captures = before
                raise
            return {'state': self.state(), 'saved_path': saved_path}


class PlaneServer(ThreadingHTTPServer):
    allow_reuse_address = False
    daemon_threads = True
    block_on_close = False

    def __init__(self, session, port=0):
        self.session = session
        self._slots = threading.BoundedSemaphore(8)
        super().__init__(('127.0.0.1', port), PlaneHandler)
        self.origin = 'http://127.0.0.1:'+str(self.server_port)
        self.timeout = .3

    def get_request(self):
        request, address = super().get_request()
        request.settimeout(3)
        return request, address

    def process_request(self, request, client_address):
        if not self._slots.acquire(False):
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


class PlaneHandler(BaseHTTPRequestHandler):
    server_version, sys_version = 'WheelchairOfflinePlanes', ''

    def log_message(self, *args):
        pass

    def reply(self, status, value, kind='application/json; charset=utf-8'):
        if not isinstance(value, bytes):
            value = json.dumps(value, ensure_ascii=False, allow_nan=False).encode('utf-8')
        self.send_response(status)
        for key, content in {'Content-Type': kind, 'Content-Length': str(len(value)),
            'Cache-Control': 'no-store', 'X-Content-Type-Options': 'nosniff',
            'Referrer-Policy': 'no-referrer', 'Content-Security-Policy':
            "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; "
            "object-src 'none'; base-uri 'none'; frame-ancestors 'none'; form-action 'none'"}.items():
            self.send_header(key, content)
        self.end_headers()
        self.wfile.write(value)

    def authorized(self, post=False):
        hosts, origins = self.headers.get_all('Host', []), self.headers.get_all('Origin', [])
        valid = hosts == [self.server.origin.removeprefix('http://')]
        valid = valid and (not origins or origins == [self.server.origin])
        valid = valid and self.headers.get('Sec-Fetch-Site') != 'cross-site'
        if post:
            tokens = self.headers.get_all('X-Picker-Token', [])
            valid = valid and len(tokens) == 1 and tokens[0].isascii() and secrets.compare_digest(
                tokens[0], self.server.session.token)
        if not valid:
            self.reply(403, {'error': '请求来源、Host 或令牌校验失败'})
        return valid

    def do_GET(self):
        if not self.authorized():
            return
        if self.path == '/api/state':
            self.reply(200, self.server.session.state())
        elif self.path in ASSETS:
            name, kind = ASSETS[self.path]
            path = ASSET_ROOT/name
            try:
                no_links(path.absolute())
                require(path.stat().st_size <= 2_000_000, '静态文件过大')
                self.reply(200, path.read_bytes(), kind)
            except (OSError, ValueError):
                self.reply(503, {'error': '工作台静态文件不可用'})
        else:
            self.reply(404, {'error': '未知页面'})

    def do_POST(self):
        if not self.authorized(post=True):
            return
        action = self.path.removeprefix('/api/')
        if self.path != '/api/'+action or action not in ('import', 'roi', 'role', 'remove', 'fit', 'solve', 'icp', 'save'):
            self.reply(404, {'error': '未知操作'})
            return
        lengths = self.headers.get_all('Content-Length', [])
        if self.headers.get_all('Transfer-Encoding') or len(lengths) != 1 or not lengths[0].isascii() or not lengths[0].isdigit() or len(lengths[0]) > 6:
            self.reply(400, {'error': '须提供唯一的 Content-Length'})
            return
        length = int(lengths[0])
        if not 0 < length <= 32768:
            self.reply(413, {'error': '请求体超出 32 KiB 上限'})
            return
        if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
            self.reply(415, {'error': '须使用 application/json'})
            return
        try:
            raw = self.rfile.read(length)
            require(len(raw) == length, '请求体不完整')
            body = json_loads(raw.decode('utf-8'))
            require(isinstance(body, dict), '请求须为 JSON 对象')
            require(body.get('token') == self.server.session.token, '请求体令牌不匹配')
            response = self.server.session.mutate(action, body)
            self.reply(200, response)
        except (OSError, ValueError, TypeError, KeyError, OverflowError, TimeoutError, RecursionError) as error:
            self.reply(422, {'error': str(error)})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', default='/home/nvidia/wheelchair')
    parser.add_argument('--resume', type=Path)
    parser.add_argument('--dataset', type=Path)
    parser.add_argument('--port', type=int, default=0)
    parser.add_argument('--duration', type=int, default=28800)
    args = parser.parse_args(argv)
    require(1 <= args.duration <= 43200 and 0 <= args.port <= 65535, '服务时长或端口无效')
    session = PlaneSession(args.project_root, resume=args.resume)
    try:
        if args.dataset:
            session.mutate('import', {'revision': session.revision, 'dataset': str(args.dataset),
                                     'label': '已有录制 · 质量检查', 'role': 'train'})
        with PlaneServer(session, args.port) as server:
            print(json.dumps({'url': server.origin+'/', 'remote_pid': os.getpid(),
                              'session_id': session.session_id, 'workspace_path': str(session.output_root/'workspace.json'),
                              'status': 'OFFLINE_PLANE_WORKBENCH', 'live_eligible': False}), flush=True)
            deadline = time.monotonic()+args.duration
            while time.monotonic() < deadline:
                server.handle_request()
    finally:
        session.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
