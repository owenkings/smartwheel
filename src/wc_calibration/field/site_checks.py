"""ZL1107 走廊现场补录的分析：录完直接读原始录包出结果，不用先跑前端。

  scripts/lidar_site_checks bias       --dataset 录包 --operator-statement "用户确认开头 35 s 车没人碰" [--dry-run]
  scripts/lidar_site_checks wall       --dataset 录包 --tape 4.0 2.0 1.0 0.5 0.3 0.1 4.0
  scripts/lidar_site_checks tape       --front-cm 8.9 --height-cm 3.9 [--lateral-cm 46.2] [--dataset 录包]
  scripts/lidar_site_checks push       --dataset 录包
  scripts/lidar_site_checks isolation  --both 录包 --left-only 录包 --right-only 录包
  scripts/lidar_site_checks nearby     --dataset 前后没人、中间有人的录包

- bias：开头 30 s 静止的 10+10+10 零偏确认。写用户的物理静止声明，再调 wc_runtime.mapping_bias_confirm。
- wall：车头正对平墙，按卷尺距离依次停车。量两台雷达到墙的距离、和卷尺的差、左右差、测距比例、墙面俯仰。
- tape：卷尺量的两台雷达外壳前后差、高度差，对照录包（或当前）冻结的外参。
- push：车头朝走廊尽头墙推近、拉远，三种速度。量每趟速度、左右墙面差，以及雷达相对轮速的时间偏移和比例
  （正前方尽头墙的距离对轮速积分的位移；比例取每趟前后的静止端点）。
- isolation：只开左、只开右、两台都开的静止录包，比较悬空点和地面上翘点的比例，判断是不是两台互相干扰。
- nearby：同一段静止录包里中间车前站人、前后没人（或两段录包），比较没被挡住的远处像素距离，判断近处亮物会不会把远处拉近。

点云默认用录包里的 filtered 源帧（建图用的），按录包冻结的 hardware_setup（或 --hardware-setup 覆盖）换到车轴系。
filtered 带 SDK 主机滤波（含 Kalman 时间滤波），移动时正对的面距离会滞后，所以 push 的时间偏移用 raw 帧（同一帧滤波前），
另外给出 filtered 的结果和两者之差；wall 也同时给出 raw 的距离。轮速和 IMU 时间都是主机到达时间。
结果写到存储策略下 reports/field_site_checks/<UTC 时间>_<子命令>_<录包名>/result.json。
只读录包，不打开设备，不改配置。判据是经验门限，不是精度证书。现场步骤见 docs/operations/field_site_checks.md。
"""
import argparse
import datetime
import json
import math
import re
import sqlite3
import subprocess
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from .capture import _load_imu, wheel_track
from .geometry import ransac_plane

POINTS, ROWS, COLS = 9600, 60, 160
CLOUD_TOPIC = '/wc_mapping/lidar_%s/source_frame_filtered'   # 建图用的帧（SDK 主机滤波后，含 Kalman 时间滤波）
RAW_TOPIC = '/wc_mapping/lidar_%s/source_frame'              # 同一帧主机滤波前
AXLE_HEIGHT = 0.185          # 车轴离地（名义轮半径）；高度带都按它换算，与以前的分析一致
FRONT_X = 0.81               # 实测外形 MEASURED_20261107：车轴到车头最前点（脚踏前沿）
SIDE_NAMES = {'left': '左', 'right': '右'}
CONFIGURED_OFFSET_MS = 75.0  # config/calibration/lidar_time_offset.json（2026-11-08 用户决定）
REFERENCE_BIAS_NAME = 'ZL1107capture_2026_1107_154018'           # 11-07 建图录包开头 30 s 的 10+10+10 结果
REFERENCE_BIAS_RAD_S = (-0.000982, -0.001537, 0.000580)
NORM_REASONS = {'BIAS_NORM', 'CALIBRATION_GYRO_MEAN', 'HOLDOUT_GYRO_MEAN'}  # --max-bias-norm-rad-s 只放宽这三项


# ------------------------------------------------------------------ 录包读取
class Recording:
    """一段录包：冻结外参、轮速反馈、IMU 角速度；雷达帧按需流式读取（不一次载入全部点云）。"""

    def __init__(self, directory, hardware_setup=None):
        from wc_runtime.hardware_setup import resolve_hardware_setup
        self.dir = Path(directory).expanduser().resolve(strict=True)
        self.manifest = json.loads((self.dir / 'capture_manifest.json').read_text(encoding='utf-8'))
        self.setup_path = (Path(hardware_setup).expanduser().resolve(strict=True) if hardware_setup
                           else self.dir / 'configuration' / 'hardware_setup.json')
        setup = json.loads(self.setup_path.read_text(encoding='utf-8'))
        resolved = resolve_hardware_setup(setup)
        self.mounts = {side: (np.asarray(m['R_axle_lidar'], float), np.asarray(m['t_axle_lidar_m'], float))
                       for side, m in resolved['mounts'].items()}
        self.wheel_cfg = setup['wheel_odometry']
        self.bags = sorted((self.dir / self.manifest.get('bag_path', 'bag')).glob('*.db3'))
        if not self.bags:
            raise ValueError('录包缺少 SQLite 数据：' + str(self.dir))
        window = self.manifest.get('acquisition_window', {})
        self.start_ns, self.end_ns = window.get('start_monotonic_ns'), window.get('end_monotonic_ns')
        if type(self.start_ns) is not int or type(self.end_ns) is not int or self.end_ns <= self.start_ns:
            raise ValueError('录包缺少正式采集时间窗')
        self.wheel = self._wheel()
        self.imu = _load_imu(self.bags)
        self.sides = [side for side in ('left', 'right') if side in self.mounts and self._has_topic(side)]
        self.raw_sides = [side for side in self.sides if self._has_topic(side, RAW_TOPIC)]

    def _has_topic(self, side, topic=CLOUD_TOPIC):
        for bag in self.bags:
            db = sqlite3.connect(bag.as_uri() + '?mode=ro', uri=True)
            try:
                if db.execute('SELECT COUNT(*) FROM topics WHERE name=?', (topic % side,)).fetchone()[0]:
                    return True
            finally:
                db.close()
        return False

    def _wheel(self):
        cfg, rows = self.wheel_cfg, []
        journal = self.dir / 'sources' / 'wheel_feedback.jsonl'
        if journal.is_file():
            with journal.open(encoding='utf-8') as stream:
                for line in stream:
                    record = json.loads(line)
                    if record.get('event') != 'transaction_complete' or record.get('status') != 'RESPONSE_VALID':
                        continue
                    words = [w - 65536 if w >= 32768 else w for w in record['register_words_u16']]
                    rpm = (words[0] * cfg['register_to_wheel_rpm'] * cfg['left_sign'],
                           words[1] * cfg['register_to_wheel_rpm'] * cfg['right_sign'])
                    rows.append((record['receive_monotonic_ns'],
                                 *(value * 2 * math.pi / 60 * cfg['wheel_radius_m'] for value in rpm)))
        return np.asarray(rows, dtype=np.float64).reshape(-1, 3)

    def capture(self):
        """capture.wheel_track 需要的字段。"""
        return dict(manifest=self.manifest, wheel=self.wheel, wheel_cfg=self.wheel_cfg, imu=self.imu)

    def frames(self, side, keep=None, stride=1, topic=CLOUD_TOPIC):
        """按录包顺序给出 (相对采集窗起点的秒, 9600×3 原生 FLU 点)；keep(秒) 为 False 的帧不解码点云。
        topic 默认 filtered 帧（建图用的），RAW_TOPIC 是同一帧主机滤波前。"""
        from rclpy.serialization import deserialize_message
        from wc_interfaces.msg import SourceFrame
        from wc_sensors.pointcloud import decode_pointcloud2
        kept = 0
        for bag in self.bags:
            db = sqlite3.connect(bag.as_uri() + '?mode=ro', uri=True)
            try:
                topics = {name: tid for tid, name in db.execute('SELECT id,name FROM topics')}
                tid = topics.get(topic % side)
                if tid is None:
                    continue
                for (data,) in db.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY id', (tid,)):
                    message = deserialize_message(data, SourceFrame)
                    rel = (int(message.host_monotonic_ns) - self.start_ns) * 1e-9
                    if keep is not None and not keep(rel):
                        continue
                    kept += 1
                    if (kept - 1) % stride:
                        continue
                    if message.units != 'm' or message.coordinate_convention != 'FLU':
                        raise ValueError('点云不是米制 FLU')
                    xyz = decode_pointcloud2(message.cloud).astype(np.float64)
                    fixed = np.full((POINTS, 3), np.nan)
                    fixed[:min(len(xyz), POINTS)] = xyz[:POINTS]
                    yield rel, fixed
            finally:
                db.close()

    def to_axle(self, side, xyz):
        R, t = self.mounts[side]
        return xyz @ R.T + t

    def complete(self):
        return self.manifest.get('status') == 'COMPLETE' and self.manifest.get('recording_complete') is True


def planar_track(track):
    """轮速前进速度 + 陀螺航向积分出的平面轨迹 (t, x, y, yaw)，车轴系起点为原点。"""
    t, v, yaw = track['t'], track['v'], track['yaw']
    dt = np.r_[0.0, np.diff(t)]
    dt[dt > 0.5] = 0.0
    vx, vy = v * np.cos(yaw), v * np.sin(yaw)
    x = np.cumsum(np.r_[0.0, 0.5 * (vx[1:] + vx[:-1]) * dt[1:]])
    y = np.cumsum(np.r_[0.0, 0.5 * (vy[1:] + vy[:-1]) * dt[1:]])
    return t, x, y, yaw


def pose_at(track, query):
    t, x, y, yaw = track
    return np.interp(query, t, x), np.interp(query, t, y), np.interp(query, t, yaw)


def edge_mask(xyz):
    """60×160 行优先图像里的深度跳变：8 邻域距离差 > 3% 或有效邻点 < 5（物体边界与混合像素）。"""
    r = np.linalg.norm(xyz, axis=1).reshape(ROWS, COLS)
    valid = np.isfinite(r)
    big = np.zeros((ROWS, COLS), dtype=bool)
    count = np.zeros((ROWS, COLS), dtype=np.int32)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == dy == 0:
                continue
            y0, y1 = max(0, -dy), min(ROWS, ROWS - dy)
            x0, x1 = max(0, -dx), min(COLS, COLS - dx)
            a, b = r[y0:y1, x0:x1], r[y0 + dy:y1 + dy, x0 + dx:x1 + dx]
            both = valid[y0:y1, x0:x1] & valid[y0 + dy:y1 + dy, x0 + dx:x1 + dx]
            with np.errstate(invalid='ignore'):
                big[y0:y1, x0:x1] |= both & (np.abs(a - b) > 0.03 * np.minimum(a, b))
            count[y0:y1, x0:x1] += both
    return (big | (count < 5) | ~valid).reshape(-1)


def voxel(points, size=0.02):
    if not len(points):
        return points
    _, index = np.unique(np.floor(points / size).astype(np.int64), axis=0, return_index=True)
    return points[np.sort(index)]


def check(name, status, value, limit, meaning):
    return {'name': name, 'status': status, 'value': value, 'limit': limit, 'meaning': meaning}


def r(value, digits=4):
    if value is None or not np.isfinite(value):
        return None
    return round(float(value), digits)


# ------------------------------------------------------------------ wall
def front_plane(points, mount_t, near_x, far_x, half_width=0.5):
    """车轴系里车前的竖直平墙：RANSAC 后返回 (n, d, 内点数, rms)，n·p + d 为点到墙距离（车这侧为正）。"""
    h = points[:, 2] + AXLE_HEIGHT
    sel = (np.isfinite(points).all(1) & (points[:, 0] > near_x) & (points[:, 0] < far_x)
           & (np.abs(points[:, 1]) <= half_width) & (h >= 0.30) & (h <= 1.40))
    candidate = points[sel]
    if len(candidate) > 40000:
        candidate = candidate[np.random.default_rng(0).choice(len(candidate), 40000, replace=False)]
    if len(candidate) < 500:
        return None
    plane = ransac_plane(candidate, 'v')
    if plane is None or abs(plane[0][0]) < 0.9:
        return None
    return plane[0], plane[1], int(plane[2].sum()), plane[3]


def analyze_wall(rec, tapes, segments=None, window_s=8.0):
    track = wheel_track(rec.capture(), min_still_s=window_s + 1.0)
    found = [(float(track['t'][a]), float(track['t'][b])) for a, b in track['segments']]
    info = {'static_segments_s': [[round(a, 1), round(b, 1)] for a, b in found], 'tape_m': list(tapes),
            'window_s': window_s, 'front_x_m': FRONT_X}
    if segments is not None:
        chosen = [found[i] for i in segments]
    elif len(found) == len(tapes):
        chosen = found
    else:
        info['error'] = ('找到 %d 个 ≥%.0f s 的静止段，卷尺距离有 %d 个；用 --segments 指定对应的静止段序号（从 0 开始）'
                         % (len(found), window_s + 1, len(tapes)))
        return info, [check('静止段数', 'FAIL', len(found), len(tapes), info['error'])]
    if len(chosen) != len(tapes):
        info['error'] = '--segments 和 --tape 个数不同'
        return info, [check('静止段数', 'FAIL', len(chosen), len(tapes), info['error'])]
    rows = []
    for (a, b), tape in zip(chosen, tapes):
        row = {'tape_m': tape, 'window_s': [round(b - window_s - 0.5, 1), round(b - 0.5, 1)]}
        keep = lambda s, b=b: b - window_s - 0.5 <= s <= b - 0.5  # noqa: E731
        for side in rec.sides:
            row[side] = wall_distance(rec, side, keep, tape, CLOUD_TOPIC)
            if row[side] and 'front_to_wall_m' in row[side] and side in rec.raw_sides:
                raw = wall_distance(rec, side, keep, tape, RAW_TOPIC)
                if raw and 'front_to_wall_m' in raw:
                    row[side]['raw'] = {key: raw[key] for key in ('front_to_wall_m', 'wall_pitch_deg', 'rms_m')}
                    row[side]['filtered_minus_raw_m'] = r(row[side]['front_to_wall_m'] - raw['front_to_wall_m'])
        L, R = row.get('left'), row.get('right')
        if L and R and 'front_to_wall_m' in L and 'front_to_wall_m' in R:
            row['left_minus_right_m'] = r(L['front_to_wall_m'] - R['front_to_wall_m'])
            row['pitch_left_minus_right_deg'] = r(L['wall_pitch_deg'] - R['wall_pitch_deg'], 2)
            if 'raw' in L and 'raw' in R:
                row['raw_left_minus_right_m'] = r(L['raw']['front_to_wall_m'] - R['raw']['front_to_wall_m'])
        rows.append(row)
    info['distances'] = rows
    return info, wall_checks(rows, rec.sides)


def wall_distance(rec, side, keep, tape, topic):
    """一段静止里一台雷达（一种帧）到车前平墙的距离、墙面姿态。"""
    pts = [rec.to_axle(side, xyz) for _, xyz in rec.frames(side, keep, topic=topic)]
    if not pts:
        return None
    mount_t = rec.mounts[side][1]
    plane = front_plane(np.concatenate(pts), mount_t, FRONT_X + 0.03, FRONT_X + tape + 1.0)
    if plane is None:
        return {'frames': len(pts), 'error': '没有找到车前的竖直平墙'}
    n, d, inliers, rms = plane
    front = float(n @ np.array([FRONT_X, 0.0, mount_t[2]]) + d)
    return {'frames': len(pts), 'wall_points': inliers, 'rms_m': r(rms),
            'front_to_wall_m': r(front), 'error_vs_tape_m': r(front - tape),
            'lidar_to_wall_m': r(float(n @ mount_t + d)),
            'wall_yaw_deg': r(math.degrees(math.atan2(n[1], -n[0])), 2),
            'wall_pitch_deg': r(math.degrees(math.asin(max(-1.0, min(1.0, n[2])))), 2)}


def wall_checks(rows, sides):
    """远处（卷尺 ≥1 m）判外参和测距；近处（≤0.5 m）单独看左雷达近距离偏差和墙面变形。值都带符号。"""
    out = []
    for row in rows:
        for side in sides:
            if not row.get(side) or row[side].get('error'):
                out.append(check('%s雷达在卷尺 %.2f m 处找墙' % (SIDE_NAMES[side], row['tape_m']), 'FAIL', None, None,
                                 (row.get(side) or {}).get('error', '这段静止没有雷达帧')
                                 + '：车头要正对一面空墙，墙前 1 m 内不要有人或东西'))
    ok = [row for row in rows if row.get('left_minus_right_m') is not None]
    far = [row for row in ok if row['tape_m'] >= 1.0]
    near = [row for row in ok if row['tape_m'] <= 0.5]
    if far:
        worst = max((row['left_minus_right_m'] for row in far), key=abs)
        out.append(check('远处左右差（卷尺 ≥1 m，负值=左雷达偏近）', 'PASS' if abs(worst) <= 0.015 else 'FAIL', worst, 0.015,
                         '两台到同一面墙的距离差；超过说明外参或远距离测距有问题'))
        pitch = float(np.median([row['pitch_left_minus_right_deg'] for row in far]))
        out.append(check('远处两台墙面俯仰差（中位）', 'PASS' if abs(pitch) <= 0.3 else 'WARN', r(pitch, 2), 0.3,
                         '两台相对俯仰外参的误差'))
    if near:
        values = {str(row['tape_m']): row['left_minus_right_m'] for row in near}
        worst = max(values.values(), key=abs)
        out.append(check('近处左右差（卷尺 ≤0.5 m，负值=左雷达偏近）', 'PASS' if abs(worst) <= 0.03 else 'WARN', values,
                         0.03, '超过 3 cm 说明左雷达近距离测距偏差确实存在，按各距离的差做距离修正表'))
        tilt = {str(row['tape_m']): row['pitch_left_minus_right_deg'] for row in near}
        out.append(check('近处两台墙面俯仰差（卷尺 ≤0.5 m）', 'INFO', tilt, None,
                         '近处墙面看起来倾斜，说明测距偏差随像素行变化（不是外参）；只记录'))
        raw = {str(row['tape_m']): row['raw_left_minus_right_m'] for row in near if 'raw_left_minus_right_m' in row}
        if raw:
            out.append(check('近处左右差（raw 帧，主机滤波前）', 'INFO', raw, None,
                             '和上面 filtered 的差不多，说明近距离偏差来自雷达本身；差很多说明来自主机滤波'))
    shift = {SIDE_NAMES[side]: {str(row['tape_m']): row[side]['filtered_minus_raw_m'] for row in rows
                                if row.get(side) and 'filtered_minus_raw_m' in row[side]} for side in sides}
    shift = {name: values for name, values in shift.items() if values}
    if shift:
        out.append(check('filtered − raw 距离（各卷尺距离）', 'INFO', shift, None,
                         '静止时主机滤波对墙面距离的影响；11-07 A→B 停车 2.8 m 处约 +4–5 mm'))
    for side in sides:
        name = SIDE_NAMES[side]
        good = [row for row in rows if row.get(side) and row[side].get('front_to_wall_m') is not None]
        far_side = [row for row in good if row['tape_m'] >= 1.0]
        if far_side:
            worst = max((row[side]['error_vs_tape_m'] for row in far_side), key=abs)
            out.append(check('%s雷达和卷尺的差（≥1 m，正值=比卷尺远）' % name, 'PASS' if abs(worst) <= 0.03 else 'WARN', worst,
                             0.03, '超过 3 cm 先确认卷尺量法和车是否摆正；各距离一样大的差是位置/常数偏差，随距离变大是比例'))
            pitch = float(np.median([row[side]['wall_pitch_deg'] for row in far_side]))
            out.append(check('%s雷达看到的远处墙面俯仰（中位）' % name, 'PASS' if abs(pitch) <= 0.5 else 'WARN', r(pitch, 2), 0.5,
                             '墙是竖直的；超过说明这台的俯仰外参偏了'))
        near_side = {str(row['tape_m']): row[side]['error_vs_tape_m'] for row in good if row['tape_m'] <= 0.5}
        if near_side:
            out.append(check('%s雷达和卷尺的差（≤0.5 m 各档，正值=比卷尺远）' % name, 'INFO', near_side, None,
                             '近处哪台偏、偏多少看这一行（含卷尺量法误差）；单雷达对照录包也看这一行'))
        scale_rows = [row for row in good if row[side]['lidar_to_wall_m'] >= 1.3]   # 卷尺 1.0 m 时右雷达到墙约 1.40 m
        if len(scale_rows) >= 3:
            x = np.array([row[side]['lidar_to_wall_m'] for row in scale_rows])
            y = np.array([row[side]['error_vs_tape_m'] for row in scale_rows])
            slope = float(np.polyfit(x, y, 1)[0])
            out.append(check('%s雷达测距比例（误差随距离的斜率）' % name, 'PASS' if abs(slope) <= 0.01 else 'WARN',
                             r(slope), 0.01, '1% 以内算没有比例误差；4% 就能解释 10-26 轮速比例 0.93 和卷尺 0.89 的差'))
        if good:
            yaw = float(np.median([row[side]['wall_yaw_deg'] for row in good]))
            out.append(check('车和墙的夹角（%s雷达，中位）' % name, 'PASS' if abs(yaw) <= 2.0 else 'WARN', r(yaw, 2), 2.0,
                             '车头没摆正；距离按垂直距离算，影响小，但卷尺读数会偏'))
        repeated = {}
        for row in good:                       # good 按停车先后排列
            repeated.setdefault(round(row['tape_m'], 3), []).append(row[side]['error_vs_tape_m'])
        for tape, values in sorted(repeated.items()):
            if len(values) >= 2:
                drift = values[-1] - values[0]
                out.append(check('%s雷达卷尺 %.2f m 首末两次停车之差（后 − 前）' % (name, tape),
                                 'PASS' if abs(drift) <= 0.01 else 'WARN', r(drift), 0.01,
                                 '这段录包里的测距漂移，含两次摆车和量卷尺的误差；超过 1 cm 时 A 修正绝对距离和测距比例，'
                                 '左右差不受影响'))
    return out


# ------------------------------------------------------------------ tape
def analyze_tape(mounts, front_cm, height_cm, lateral_cm=None):
    (_, tl), (_, tr) = mounts['left'], mounts['right']
    expected = {'right_ahead_of_left_m': r(tr[0] - tl[0]), 'left_above_right_m': r(tl[2] - tr[2]),
                'lateral_separation_m': r(tl[1] - tr[1])}
    measured = {'right_ahead_of_left_m': front_cm / 100.0, 'left_above_right_m': height_cm / 100.0}
    if lateral_cm is not None:
        measured['lateral_separation_m'] = lateral_cm / 100.0
    info = {'configured_from_frozen_extrinsic': expected, 'tape': measured,
            'difference_m': {k: r(measured[k] - expected[k]) for k in measured}}
    out = []
    for key, name, limit in (('right_ahead_of_left_m', '前后差', 0.02), ('left_above_right_m', '高度差', 0.02),
                             ('lateral_separation_m', '左右间距', 0.01)):
        if key in measured:
            diff = measured[key] - expected[key]
            out.append(check(name, 'PASS' if abs(diff) <= limit else 'WARN', r(diff), limit,
                             '卷尺减冻结外参；超过门限说明外参平移可能偏了（现场标定平移误差约 ±2–4 cm）'))
    return info, out


# ------------------------------------------------------------------ push
def passes_from(track, min_distance=1.0):
    t, s, v = track['t'], track['s'], track['v']
    segs = track['segments']
    out = []
    for (a0, b0), (a1, _) in zip(segs, segs[1:]):
        t0, t1 = float(t[b0]), float(t[a1])
        distance = float(s[a1] - s[b0])
        if abs(distance) < min_distance:
            continue
        moving = (t >= t0) & (t <= t1) & (np.abs(v) > 0.05)
        out.append({'start_s': round(t0, 2), 'end_s': round(t1, 2), 'distance_m': round(distance, 3),
                    'median_speed_m_s': round(float(np.median(np.abs(v[moving]))), 3) if moving.any() else None,
                    'yaw_change_deg': round(float(np.degrees(np.interp(t1, t, track['yaw']) - np.interp(t0, t, track['yaw']))), 1)})
    return out


def end_wall(points, mount_t, max_range):
    """正前方尽头墙：x = a + b·y + c·z 的最小二乘（两次剔除），返回 (雷达到墙垂直距离, 墙法向水平方向角, 点数, rms)。"""
    h = points[:, 2] + AXLE_HEIGHT
    sel = (np.isfinite(points).all(1) & (points[:, 0] > mount_t[0] + 0.8) & (points[:, 0] < mount_t[0] + max_range)
           & (np.abs(points[:, 1]) <= 0.4) & (h >= 0.30) & (h <= 1.40))
    P = points[sel]
    if len(P) < 150:
        return None
    P = P[np.abs(P[:, 0] - np.median(P[:, 0])) < 0.15]
    for _ in range(2):
        if len(P) < 120:
            return None
        A = np.column_stack([np.ones(len(P)), P[:, 1], P[:, 2]])
        coef, *_ = np.linalg.lstsq(A, P[:, 0], rcond=None)
        res = P[:, 0] - A @ coef
        keep = np.abs(res) < max(0.03, 2.5 * float(np.std(res)))
        P = P[keep]
    a, b, c = coef
    rms = float(np.sqrt(np.mean((P[:, 0] - np.column_stack([np.ones(len(P)), P[:, 1], P[:, 2]]) @ coef) ** 2)))
    if rms > 0.03 or len(P) < 120:
        return None
    norm = math.sqrt(1 + b * b + c * c)
    distance = (a + b * mount_t[1] + c * mount_t[2] - mount_t[0]) / norm
    return distance, math.atan2(-b, 1.0), len(P), rms


def local_planes(target, queries, k=12, radius=0.12):
    tree = cKDTree(target)
    dist, idx = tree.query(queries, k=k, distance_upper_bound=radius)
    full = np.isfinite(dist).all(axis=1)
    if not full.any():
        return None
    nb = target[idx[full]]
    c = nb.mean(axis=1)
    X = nb - c[:, None, :]
    vals, vecs = np.linalg.eigh(np.einsum('nki,nkj->nij', X, X) / k)
    flat = vals[:, 0] < 0.1 * vals[:, 1]
    return queries[full][flat], c[flat], vecs[flat, :, 0]


def band_points(rec, side, xyz):
    keep = ~edge_mask(xyz)
    p = rec.to_axle(side, xyz[keep])
    h = p[:, 2] + AXLE_HEIGHT
    rng = np.hypot(p[:, 0] - rec.mounts[side][1][0], p[:, 1] - rec.mounts[side][1][1])
    return voxel(p[(h >= 0.6) & (h <= 1.8) & (rng <= 6.0)]).astype(np.float32)


def lidar_projection(track, query, lever_xy, u):
    """雷达原点在轨迹系里沿墙法向 u 的投影，拆成车轴部分和杆臂部分。"""
    x, y, yaw = pose_at(track, query)
    lever = np.column_stack([np.cos(yaw) * lever_xy[0] - np.sin(yaw) * lever_xy[1],
                             np.sin(yaw) * lever_xy[0] + np.cos(yaw) * lever_xy[1]])
    return np.column_stack([x, y]) @ u, lever @ u


def pass_scale(series, track, lever_xy, u, pass_row, still_s=5.0):
    """趟前、趟后静止段里尽头墙距离的变化 / 轮速轨迹沿墙法向的位移（和时间偏移无关）。
    返回 (雷达位移 / 轮速位移, 是否朝墙走)；端点看不到墙或沿法向移动不到 1 m 时返回 None。"""
    t, d = series[:, 0], series[:, 1]
    a, b = pass_row['start_s'], pass_row['end_s']
    pre = (t >= a - still_s) & (t <= a - 0.3)
    post = (t >= b + 0.3) & (t <= b + still_s)
    if pre.sum() < 10 or post.sum() < 10:
        return None
    axle, lever = lidar_projection(track, np.array([a, b]), lever_xy, u)
    moved = axle[1] - axle[0]
    if abs(moved) < 1.0:
        return None
    return -(float(np.median(d[post]) - np.median(d[pre])) + lever[1] - lever[0]) / moved, bool(moved > 0)


def offset_curve(series, track, lever_xy, u, k, grid_ms):
    """k 固定：d + u·(R·lever)(t − τ) + k·u·p_axle(t − τ) = 常数；每个 τ 的残差平方和（常数取均值）。"""
    t, d = series[:, 0], series[:, 1]
    sse = np.empty(len(grid_ms))
    for n, tau_ms in enumerate(grid_ms):
        axle, lever = lidar_projection(track, t - tau_ms * 1e-3, lever_xy, u)
        y = d + lever + k * axle
        sse[n] = float(np.sum((y - y.mean()) ** 2))
    return sse


def offset_fit(series, track, lever_xy, passes, grid):
    """一台雷达、一种帧：每趟比例取静止端点；τ 网格上逐趟、合计、只朝墙、只离墙各取残差最小。
    朝墙和离墙两个方向合起来，测距随距离的非线性误差对 τ 的影响基本抵消。"""
    heading = np.interp(series[:, 0], track[0], track[3]) + series[:, 2]
    u = np.array([np.median(np.cos(heading)), np.median(np.sin(heading))])
    u /= np.linalg.norm(u)
    at_configured = int(np.argmin(np.abs(grid - CONFIGURED_OFFSET_MS)))
    per_pass, totals = [], {}
    for k, p in enumerate(passes):
        row = {'pass': k, 'median_speed_m_s': p['median_speed_m_s']}
        scale = pass_scale(series, track, lever_xy, u, p)
        m = (series[:, 0] >= p['start_s'] - 1.5) & (series[:, 0] <= p['end_s'] + 1.5)
        if scale is None or m.sum() < 20:
            row.update(usable=False, reason='趟前或趟后静止时看不到尽头墙，或沿墙法向移动不到 1 m')
            per_pass.append(row)
            continue
        sse = offset_curve(series[m], track, lever_xy, u, scale[0], grid)
        best, n = int(np.argmin(sse)), int(m.sum())
        row.update(usable=True, direction='toward' if scale[1] else 'away', lidar_over_wheel=r(scale[0]),
                   offset_ms=float(grid[best]), rms_m=r(math.sqrt(sse[best] / n)),
                   rms_at_configured_m=r(math.sqrt(sse[at_configured] / n)), samples=n)
        per_pass.append(row)
        for key in ('all', row['direction']):
            totals[key] = sse if key not in totals else totals[key] + sse
    usable = [p for p in per_pass if p['usable']]
    out = {'per_pass': per_pass, 'usable_passes': len(usable), 'wall_normal_in_track_frame': [r(v) for v in u]}
    for key, sse in totals.items():
        out['offset_ms_' + key] = float(grid[int(np.argmin(sse))])
    out['at_grid_edge'] = any(out['offset_ms_' + key] in (float(grid[0]), float(grid[-1])) for key in totals)
    if usable:
        out['lidar_over_wheel'] = r(float(np.mean([p['lidar_over_wheel'] for p in usable])))
        out['offset_spread_ms'] = float(max(p['offset_ms'] for p in usable) - min(p['offset_ms'] for p in usable))
    return out


def filter_lag(raw, filtered, wheel):
    """同一批帧（时间戳相同）：filtered 的尽头墙距离 ≈ raw 晚 L 秒。和轮速时间无关。
    只用两种帧在同一时刻都找到、且距离差 < 0.1 m 的帧（同一面墙），直行（|ω| < 0.1 rad/s）且在动。"""
    _, i_raw, i_filtered = np.intersect1d(raw[:, 0], filtered[:, 0], return_indices=True)
    same_wall = np.abs(filtered[i_filtered, 1] - raw[i_raw, 1]) < 0.10
    raw, filtered = raw[i_raw[same_wall]], filtered[i_filtered[same_wall]]
    tr, dr = raw[:, 0], raw[:, 1]
    tf, df = filtered[:, 0], filtered[:, 1]
    speed = np.abs(np.interp(tf, wheel['t'], wheel['v']))
    turning = np.abs(np.interp(tf, wheel['t'], wheel['omega'])) >= 0.1
    moving = (speed > 0.1) & ~turning
    curve = []
    for lag in np.arange(-0.1, 0.4001, 0.005):
        q = tf - lag
        idx = np.searchsorted(tr, q)
        ok = moving & (idx > 0) & (idx < len(tr))
        gap = np.zeros(len(tf), dtype=bool)
        gap[ok] = (tr[idx[ok]] - tr[idx[ok] - 1]) < 0.25      # 不跨过找不到墙的空档插值
        ok &= gap
        if ok.sum() >= 30:
            curve.append((float(lag), float(np.sqrt(np.mean((df[ok] - np.interp(q[ok], tr, dr)) ** 2))), int(ok.sum())))
    if not curve:
        return None
    best = min(curve, key=lambda c: c[1])
    zero = min(curve, key=lambda c: abs(c[0]))
    same = df - dr
    static = (speed < 0.01) & ~turning
    return {'lag_ms': round(1000 * best[0], 1), 'rms_m': r(best[1]), 'rms_at_0_m': r(zero[1]), 'moving_frames': best[2],
            'filtered_minus_raw_moving_m': r(float(np.median(same[moving]))) if moving.any() else None,
            'filtered_minus_raw_static_m': r(float(np.median(same[static]))) if static.any() else None,
            'moving_speed_median_m_s': r(float(np.median(speed[moving])), 3) if moving.any() else None}


def analyze_push(rec, max_range=7.5):
    wheel = wheel_track(rec.capture(), min_still_s=3.0)
    track = planar_track(wheel)
    passes = passes_from(wheel)
    info = {'passes': passes, 'yaw_source': wheel['yaw_source'], 'end_wall_max_range_m': max_range,
            'raw_frames': rec.raw_sides}
    if not passes:
        info['error'] = '没有找到"静止 → 推 ≥1 m → 静止"的趟次'
        return info, [check('趟次', 'FAIL', 0, 4, info['error'])]
    windows = [(p['start_s'] - 1.5, p['end_s'] + 1.5) for p in passes]

    def pass_of(s):
        for k, (a, b) in enumerate(windows):
            if a <= s <= b:
                return k
        return -1

    def near_pass(s):   # 时间偏移只用趟内和前后各 6 s 的帧
        return any(a - 4.5 <= s <= b + 4.5 for a, b in windows)

    series, bands, stamps = {}, {}, {}
    for side in rec.sides:
        mount_t = rec.mounts[side][1]
        rows, bands[side], stamps[side] = [], [], []
        for rel, xyz in rec.frames(side):
            w = end_wall(rec.to_axle(side, xyz), mount_t, max_range)
            if w is not None:
                rows.append((rel, w[0], w[1]))
            stamps[side].append(rel)
            bands[side].append(band_points(rec, side, xyz))
        series[side, 'filtered'] = np.array(rows, dtype=float).reshape(-1, 3)
        if side in rec.raw_sides:
            rows = []
            for rel, xyz in rec.frames(side, near_pass, topic=RAW_TOPIC):
                w = end_wall(rec.to_axle(side, xyz), mount_t, max_range)
                if w is not None:
                    rows.append((rel, w[0], w[1]))
            series[side, 'raw'] = np.array(rows, dtype=float).reshape(-1, 3)
    grid = np.arange(-100.0, 400.01, 1.0)
    primary = 'raw' if rec.sides and set(rec.sides) <= set(rec.raw_sides) else 'filtered'
    info['offset_from'] = primary
    out, fits = [], {}
    for side in rec.sides:
        name, lever, row = SIDE_NAMES[side], rec.mounts[side][1][:2], {}
        for kind in ('raw', 'filtered'):
            s = series.get((side, kind))
            if s is None:
                continue
            row[kind] = offset_fit(s, track, lever, passes, grid) if len(s) >= 40 else {}
            row[kind]['end_wall_frames'] = len(s)
        if row.get('raw', {}).get('end_wall_frames', 0) >= 40 and row['filtered']['end_wall_frames'] >= 40:
            row['filtered_lag_vs_raw'] = filter_lag(series[side, 'raw'], series[side, 'filtered'], wheel)
        info[side] = row
        if row[primary]['end_wall_frames'] < 40:
            out.append(check('%s雷达尽头墙' % name, 'FAIL', row[primary]['end_wall_frames'], 40,
                             '推的方向上要正对一面墙（走廊尽头），雷达到墙 %.1f m 以内；车头一直朝墙，往回拉时不要掉头' % max_range))
        else:
            fits[side] = row[primary]
    if len(fits) == len(rec.sides) and fits:
        usable = min(f['usable_passes'] for f in fits.values())
        out.append(check('能用的趟数（两台中较少的）', 'PASS' if usable >= 4 else 'WARN' if usable >= 2 else 'FAIL', usable, 4,
                         '每趟前后静止时都要看得到尽头墙、沿墙法向推 ≥1 m；三种速度各朝墙、离墙一趟共 6 趟'))
        both = all('offset_ms_toward' in f and 'offset_ms_away' in f for f in fits.values())
        if usable and not both:
            out.append(check('朝墙和离墙两个方向都有', 'WARN', False, True,
                             '只有一个方向时，测距随距离的误差会混进时间偏移；补几趟往回拉（车头朝墙）'))
    with_offset = {side: f for side, f in fits.items() if 'offset_ms_all' in f}
    if with_offset:
        values = {side: f['offset_ms_all'] for side, f in with_offset.items()}
        mean = float(np.mean(list(values.values())))
        edge = any(f['at_grid_edge'] for f in with_offset.values())
        out.append(check('雷达相对轮速的时间偏移（%s 帧，两台平均）' % primary,
                         'WARN' if edge else ('PASS' if abs(mean - CONFIGURED_OFFSET_MS) <= 20 else 'WARN'),
                         round(mean, 1), '75 ± 20 ms',
                         '正前方墙的距离变化对轮速积分；配置的 75 ms 来自转弯时 ICP 偏航对陀螺。超出不要直接改配置，交给 A 分析'
                         + ('；结果顶到了搜索边界（−100 或 400 ms），不可信' if edge else '')))
        if len(values) == 2:
            diff = values['left'] - values['right']
            out.append(check('两台时间偏移之差（左 − 右）', 'PASS' if abs(diff) <= 30 else 'WARN', round(diff, 1), 30,
                             '配置两台用同一个值；差超过 30 ms 说明需要分开'))
        spread = max(f.get('offset_spread_ms', 0.0) for f in with_offset.values())
        out.append(check('各趟时间偏移的离散（最大 − 最小）', 'PASS' if spread <= 40 else 'WARN', round(spread, 1), 40,
                         '各趟差太多说明这次数据定不准；慢的一趟定得最差，补推快一点的'))
        split = {side: round(f['offset_ms_toward'] - f['offset_ms_away'], 1) for side, f in with_offset.items()
                 if 'offset_ms_toward' in f and 'offset_ms_away' in f}
        if split:
            out.append(check('朝墙减离墙的时间偏移', 'INFO', split, None,
                             '两向差大说明测距随距离有非线性误差；合计值里两向基本抵消，只记录'))
        scale = float(np.mean([f['lidar_over_wheel'] for f in with_offset.values()]))
        out.append(check('雷达位移 / 轮速位移（静止端点，两台平均）', 'PASS' if abs(scale - 1.0) <= 0.01 else 'WARN', r(scale),
                         '1 ± 0.01', '大于 1 说明轮速少算了距离（或雷达测距偏大）；和墙面定距那一项的测距比例一起看'))
    if primary == 'raw':
        filtered = {side: info[side]['filtered'].get('offset_ms_all') for side in fits if 'filtered' in info[side]}
        if any(v is not None for v in filtered.values()):
            out.append(check('同样算法用 filtered 帧（建图用的）的时间偏移', 'INFO', filtered, None,
                             '比 raw 大出来的部分是 SDK 主机滤波（Kalman 等）对正对面的距离滞后'))
        lag = {side: info[side]['filtered_lag_vs_raw']['lag_ms'] for side in rec.sides
               if (info[side].get('filtered_lag_vs_raw') or {}).get('lag_ms') is not None}
        if lag:
            out.append(check('filtered 帧距离比 raw 晚（同一帧对比）', 'INFO', lag, None,
                             '和轮速无关；11-07 A→B 约 60–70 ms。移动时建图用的帧里正对的面会偏远约 速度 × 这个值'))
    if 'left' in bands and 'right' in bands:
        tl, tr = np.array(stamps['left']), np.array(stamps['right'])
        groups = {}
        for j in range(0, len(tr), 2):
            i = int(np.argmin(np.abs(tl - tr[j])))
            if abs(tl[i] - tr[j]) > 0.05 or len(bands['left'][i]) < 200 or len(bands['right'][j]) < 200:
                continue
            xi, yi, ai = pose_at(track, tl[i])
            xj, yj, aj = pose_at(track, tr[j])
            dyaw = aj - ai
            c, s = math.cos(-ai), math.sin(-ai)
            dx, dy = c * (xj - xi) - s * (yj - yi), s * (xj - xi) + c * (yj - yi)
            q = bands['right'][j].astype(float)
            cr, sr = math.cos(dyaw), math.sin(dyaw)
            q = np.column_stack([cr * q[:, 0] - sr * q[:, 1] + dx, sr * q[:, 0] + cr * q[:, 1] + dy, q[:, 2]])
            if len(q) > 2500:
                q = q[np.random.default_rng(j).choice(len(q), 2500, replace=False)]
            planes = local_planes(bands['left'][i].astype(float), q)
            if planes is None or len(planes[0]) < 50:
                continue
            res = np.abs(np.einsum('ij,ij->i', planes[2], planes[0] - planes[1]))
            speed = float(np.interp(tr[j], wheel['t'], np.abs(wheel['v'])))
            k = pass_of(tr[j])
            key = 'static' if speed < 0.02 else 'pass_%d' % k if k >= 0 else 'other_moving'
            groups.setdefault(key, []).append(float(np.median(res)))
        lr = {key: {'pairs': len(v), 'median_abs_mm': round(1000 * float(np.median(v)), 1)} for key, v in groups.items()}
        for k, p in enumerate(passes):
            if 'pass_%d' % k in lr:
                lr['pass_%d' % k]['median_speed_m_s'] = p['median_speed_m_s']
        info['left_right_wall_difference'] = lr
        if 'static' in lr:
            value = lr['static']['median_abs_mm'] / 1000
            out.append(check('左右墙面差（静止）', 'PASS' if value <= 0.015 else 'WARN', r(value), 0.015,
                             '两台 filtered 帧到同一片墙的点面距离中位；11-07 A→B 停车时 14.8 mm'))
        moving = {key: lr[key]['median_abs_mm'] / 1000 for key in lr if key.startswith('pass_')}
        if moving:
            worst = max(moving.values())
            out.append(check('左右墙面差（推着走，最差一趟）', 'PASS' if worst <= 0.05 else 'WARN', r(worst), 0.05,
                             '11-07 推着走 42–55 mm；看 result.json 里各趟是否随速度变大'))
    return info, out


# ------------------------------------------------------------------ isolation
def floating_mask(p, h, rng):
    """C 的悬空点定义：0.45–1.4 m 高、4 m 内，所在竖直连续段（缝 < 0.10 m、0.15 m 柱）下方空 ≥ 0.30 m。"""
    out = np.zeros(len(p), dtype=bool)
    cand = np.flatnonzero((h >= 0.45) & (h <= 1.4) & (rng <= 4.0))
    if not len(cand):
        return out
    tree = cKDTree(p[:, :2])
    for i in cand:
        nb = np.asarray(tree.query_ball_point(p[i, :2], 0.15), dtype=int)
        hn = h[nb]
        below = hn[hn < h[i] - 0.02]
        run = np.sort(hn[hn <= h[i]])[::-1]
        bottom = h[i]
        for value in run[1:]:
            if bottom - value < 0.10:
                bottom = value
            else:
                break
        rest = below[below < bottom - 1e-6]
        out[i] = bottom >= 0.45 and bottom - (rest.max() if len(rest) else 0.0) >= 0.30
    return out


def artifact_counts(mount_t, p):
    """一帧（车轴系有效点）：车前 0.5–4.5 m、左右 1.5 m 内的点数、其中的悬空点和地面上翘点（2 m 外、离地 0.2765–0.45 m）。"""
    p = p[(p[:, 0] > mount_t[0] + 0.5) & (p[:, 0] < mount_t[0] + 4.5) & (np.abs(p[:, 1]) <= 1.5)]
    h = p[:, 2] + AXLE_HEIGHT
    rng = np.hypot(p[:, 0] - mount_t[0], p[:, 1] - mount_t[1])
    floating = floating_mask(p, h, rng)
    curl = (rng >= 2.0) & (h >= 0.2765) & (h < 0.45)
    return len(p), int(floating.sum()), int(curl.sum())


def static_rates(rec, side, window=None, stride=2):
    """静止录包每帧：车前区域点数、悬空点、地面上翘点（2 m 外、离地 0.2765–0.45 m），和中位距离图。"""
    mount_t = rec.mounts[side][1]
    rows, ranges = [], []
    keep = None if window is None else (lambda s: window[0] <= s <= window[1])
    for _, xyz in rec.frames(side, keep, stride):
        ranges.append(np.linalg.norm(xyz, axis=1).astype(np.float32))
        rows.append(artifact_counts(mount_t, rec.to_axle(side, xyz[np.isfinite(xyz).all(1)])))
    if not rows:
        return None
    a = np.array(rows, dtype=float)
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)   # 一直无效的像素
        median_range = np.nanmedian(np.stack(ranges), axis=0)
    return {'frames': len(rows), 'roi_points_median': int(np.median(a[:, 0])),
            'floating_per_frame_median': float(np.median(a[:, 1])), 'curl_per_frame_median': float(np.median(a[:, 2])),
            'floating_share': float(a[:, 1].sum() / max(1.0, a[:, 0].sum())),
            'curl_share': float(a[:, 2].sum() / max(1.0, a[:, 0].sum())), '_median_range': median_range}


def analyze_isolation(recs, window=None, stride=2):
    info, out = {'window_s': window, 'frame_stride': stride}, []
    stats = {}
    for label, rec in recs.items():
        stats[label] = {side: static_rates(rec, side, window, stride) for side in rec.sides}
        info[label] = {'recording': str(rec.dir), 'sides': rec.sides, 'complete': rec.complete(),
                       **{side: {k: (r(v, 5) if isinstance(v, float) else v) for k, v in s.items() if not k.startswith('_')}
                          for side, s in stats[label].items() if s}}
    for side, alone in (('left', 'left_only'), ('right', 'right_only')):
        both = stats.get('both', {}).get(side)
        single = stats.get(alone, {}).get(side)
        if not both or not single:
            continue
        name = SIDE_NAMES[side]
        a, b = both['_median_range'], single['_median_range']
        valid = np.isfinite(a) & np.isfinite(b)
        moved = float(np.median(np.abs(a[valid] - b[valid]))) if valid.any() else None
        out.append(check('%s雷达两段录包是不是同一个摆放（中位距离差）' % name,
                         'PASS' if moved is not None and moved <= 0.03 else 'WARN', r(moved), 0.03,
                         '三段录包之间车不能动；超过说明摆放变了，比较不可靠'))
        for key, label in (('floating_share', '悬空点'), ('curl_share', '地面上翘点')):
            ratio = both[key] / single[key] if single[key] > 0 else None
            verdict = ('INTERFERENCE' if ratio is not None and ratio >= 3 else
                       'NOT_INTERFERENCE' if ratio is not None and ratio <= 1.5 else 'INCONCLUSIVE')
            info.setdefault('ratios', {})['%s_%s' % (side, key)] = {'both': r(both[key], 5), 'alone': r(single[key], 5),
                                                                     'both_over_alone': r(ratio, 2), 'verdict': verdict}
            out.append(check('%s雷达%s：两台都开 / 只开这台' % (name, label),
                             {'INTERFERENCE': 'WARN', 'NOT_INTERFERENCE': 'PASS'}.get(verdict, 'INFO'), r(ratio, 2),
                             '≥3 干扰；≤1.5 不是干扰',
                             '≥3 说明主要来自另一台的光（互相干扰）；≤1.5 说明和另一台无关'))
    return info, out


# ------------------------------------------------------------------ nearby
PERSON_BOX = (FRONT_X + 0.15, FRONT_X + 1.2, 0.5, 0.30, 1.80)  # 车轴系 x 范围、|y| 上限、离地高度：脚踏前 0.5 m 站的人
PERSON_HI, PERSON_LO = 150, 20   # 盒子里的点数：≥ PERSON_HI 算有人，≤ PERSON_LO 算没人，中间的帧不用


def frame_table(rec, side, keep=None, artifact_stride=4):
    """静止录包逐帧：时间、距离图、站人盒子里的点数；每 artifact_stride 帧另算悬空点和地面上翘点。"""
    x0, x1, half, h0, h1 = PERSON_BOX
    mount_t = rec.mounts[side][1]
    t, ranges, box, art = [], [], [], []
    for n, (rel, xyz) in enumerate(rec.frames(side, keep)):
        p = rec.to_axle(side, xyz[np.isfinite(xyz).all(1)])
        h = p[:, 2] + AXLE_HEIGHT
        t.append(rel)
        ranges.append(np.linalg.norm(xyz, axis=1).astype(np.float32))
        box.append(int(((p[:, 0] >= x0) & (p[:, 0] <= x1) & (np.abs(p[:, 1]) <= half) & (h >= h0) & (h <= h1)).sum()))
        art.append(artifact_counts(mount_t, p) if n % artifact_stride == 0 else (np.nan, np.nan, np.nan))
    if not t:
        return None
    return {'t': np.array(t), 'ranges': np.stack(ranges), 'box': np.array(box), 'artifacts': np.array(art, dtype=float)}


def presence_runs(t, flags, gap_s):
    """flags 为真的连续段 [(起始下标, 结束下标)]；中间断开不超过 gap_s 的算同一段。"""
    runs, start, last = [], None, None
    for i, flag in enumerate(flags):
        if not flag:
            continue
        if start is None:
            start = i
        elif t[i] - t[last] > gap_s:
            runs.append((start, last))
            start = i
        last = i
    if start is not None:
        runs.append((start, last))
    return runs


def split_presence(t, box, hi=PERSON_HI, lo=PERSON_LO, edge_s=2.0, gap_s=1.0):
    """按站人盒子的点数分出有人的最长一段，和紧挨着它前、后没人的两段（时间窗，交接处各让开 edge_s）。"""
    t, box = np.asarray(t, dtype=float), np.asarray(box)
    runs = presence_runs(t, box >= hi, gap_s)
    if not runs:
        return None
    k = max(range(len(runs)), key=lambda i: t[runs[i][1]] - t[runs[i][0]])
    a, b = runs[k]
    before = (t[0] if k == 0 else t[runs[k - 1][1]] + edge_s, t[a] - edge_s)
    after = (t[b] + edge_s, t[-1] if k + 1 == len(runs) else t[runs[k + 1][0]] - edge_s)
    windows = {'with': (t[a] + edge_s, t[b] - edge_s), 'before': before, 'after': after}
    return {key: (float(w[0]), float(w[1])) if w[1] > w[0] else None for key, w in windows.items()}


def subset_stats(table, mask):
    import warnings
    if not mask.any():
        return None
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)   # 一直无效的像素
        median = np.nanmedian(table['ranges'][mask], axis=0)
    art = table['artifacts'][mask]
    art = art[np.isfinite(art[:, 0])]
    stamps = table['t'][mask]
    return {'frames': int(mask.sum()), 'span_s': [r(float(stamps.min()), 1), r(float(stamps.max()), 1)],
            'roi_points_median': int(np.median(art[:, 0])) if len(art) else None,
            'floating_per_frame_median': float(np.median(art[:, 1])) if len(art) else None,
            'curl_per_frame_median': float(np.median(art[:, 2])) if len(art) else None, '_median_range': median}


def far_shift(reference, other):
    """reference 里 ≥2.4 m、两边差不到 30 cm 的像素：other − reference 的中位；另给变近 30 cm 以上（被挡住）的像素数。"""
    valid = np.isfinite(reference) & np.isfinite(other)
    diff = np.where(valid, other - reference, np.nan)
    blocked = valid & (diff < -0.30)
    far = valid & (reference >= 2.4) & (np.abs(diff) < 0.30)
    row = {'blocked_pixels': int(blocked.sum()), 'far_pixels': int(far.sum()),
           'far_shift_m': r(float(np.median(diff[far]))) if far.sum() >= 500 else None}
    if far.any():
        row['far_shift_p10_p90_m'] = [r(float(np.percentile(diff[far], q))) for q in (10, 90)]
    return row


def static_check(rec, spans):
    """spans 都落在同一个轮速静止段里（车没动）吗：返回 (True/False/None, 静止段)。没有轮速反馈时为 None。"""
    if not len(rec.wheel):
        return None, []
    track = wheel_track(rec.capture(), min_still_s=5.0)
    segments = [(float(track['t'][a]), float(track['t'][b])) for a, b in track['segments']]
    lo, hi = min(s[0] for s in spans), max(s[1] for s in spans)
    return any(a - 0.2 <= lo and hi <= b + 0.2 for a, b in segments), [[r(a, 1), r(b, 1)] for a, b in segments]


def stats_view(stats):
    return {k: (r(v, 5) if isinstance(v, float) else v) for k, v in stats.items() if not k.startswith('_')}


def analyze_nearby(with_rec, without_rec=None, with_window=None, without_window=None, window=None, stride=4):
    """车前站人和没人比：没被人挡住的远处像素距离变了多少，以及悬空点、地面上翘点。
    without_rec 为 None 时用同一段录包：按站人盒子的点数分出有人的一段和它前后没人的两段（C 的要求），
    前后两段一起当"没人"，线性漂移基本抵消；否则 with_rec 和 without_rec 两段录包（或同一段的两个时间窗）比。"""
    single = without_rec is None
    info = {'mode': 'same_recording_before_after' if single else 'two_windows', 'with': str(with_rec.dir),
            'without': str(with_rec.dir if single else without_rec.dir), 'artifact_stride': stride,
            'person_box_axle_m': {'x': list(PERSON_BOX[:2]), 'abs_y_max': PERSON_BOX[2], 'height': list(PERSON_BOX[3:])},
            'person_points': {'present_min': PERSON_HI, 'absent_max': PERSON_LO}}
    out = []
    span = lambda w: (lambda s: w is None or w[0] <= s <= w[1])  # noqa: E731
    if single:
        sides = list(with_rec.sides)
        tables = {side: frame_table(with_rec, side, span(window), stride) for side in sides}
        reference = next((side for side in sides if tables[side] is not None), None)
        split = split_presence(tables[reference]['t'], tables[reference]['box']) if reference else None
        info.update(window_s=window, segmented_by=reference, windows_s=split)
        length = lambda w: 0.0 if not w else w[1] - w[0]  # noqa: E731
        if not split or length(split['with']) < 15.0:
            out.append(check('有人的一段', 'FAIL', r(length(split['with']) if split else 0.0, 1), '≥ 15 s',
                             '车前 0.5 m 站人、站够 40 s 不动；没找到或太短就重录'))
            return info, out
        out.append(check('有人的一段', 'PASS', r(length(split['with']), 1), '≥ 15 s', '去掉进出各 2 s'))
        sides_ok = [key for key in ('before', 'after') if length(split[key]) >= 10.0]
        out.append(check('没人的前后两段（各 ≥ 10 s）', 'PASS' if len(sides_ok) == 2 else 'WARN' if sides_ok else 'FAIL',
                         {key: r(length(split[key]), 1) for key in ('before', 'after')}, '各 ≥ 10 s',
                         '前后都有才能抵消漂移；只有一侧时结果含漂移，都没有就重录'))
        if not sides_ok:
            return info, out
        still, segments = static_check(with_rec, [split[key] for key in ['with', *sides_ok]])
        info['static_segments_s'] = segments
        out.append(check('整段车没动', {True: 'PASS', False: 'FAIL', None: 'INFO'}[still], still, True,
                         '有人、没人的几段要在同一个轮速静止段里；没有轮速反馈时没法确认'))
        pairs = {}
        for side in sides:
            table = tables[side]
            if table is None:
                continue
            t, box = table['t'], table['box']
            inside = lambda w: (t >= w[0]) & (t <= w[1])  # noqa: E731
            parts = {'with': inside(split['with']) & (box >= PERSON_HI)}
            for key in sides_ok:
                parts[key] = inside(split[key]) & (box <= PERSON_LO)
            parts['without'] = np.logical_or.reduce([parts[key] for key in sides_ok])
            pairs[side] = {key: subset_stats(table, mask) for key, mask in parts.items()}
    else:
        same = with_rec.dir == without_rec.dir
        sides = [s for s in with_rec.sides if s in without_rec.sides]
        info.update(with_window_s=with_window, without_window_s=without_window, same_recording=same)
        pairs, checked = {}, False
        for side in sides:
            a = frame_table(without_rec, side, span(without_window), stride)
            b = frame_table(with_rec, side, span(with_window), stride)
            if a is None or b is None:
                continue
            if not checked:                    # 用到的帧都在一个轮速静止段里（车没动）
                checked = True
                for rec, table, label in ((with_rec, b, '有人'), (without_rec, a, '没人')):
                    still, _ = static_check(rec, [(float(table['t'].min()), float(table['t'].max()))])
                    out.append(check('%s那段车没动' % label, {True: 'PASS', False: 'FAIL', None: 'INFO'}[still], still,
                                     True, '用到的帧要在一个轮速静止段里；没有轮速反馈时没法确认'))
            pairs[side] = {'with': subset_stats(b, np.ones(len(b['t']), dtype=bool)),
                           'without': subset_stats(a, np.ones(len(a['t']), dtype=bool))}
    for side in sides:
        name = SIDE_NAMES[side]
        stats = pairs.get(side)
        if not stats or not stats.get('with') or not stats.get('without'):
            out.append(check('%s雷达帧' % name, 'FAIL', None, None, '有一段里没有这台雷达的帧'))
            continue
        a, b = stats['without'], stats['with']
        row = {key: stats_view(value) for key, value in stats.items() if value}
        row.update(far_shift(a['_median_range'], b['_median_range']))
        info[side] = row
        out.append(check('%s雷达看到车前的人（被挡住的像素）' % name, 'PASS' if row['blocked_pixels'] >= 200 else 'FAIL',
                         row['blocked_pixels'], 200, '人站在脚踏前 0.5 m、正对车头、不动；不够说明人不在视野里，重录'))
        shift = row['far_shift_m']
        status = ('INFO' if shift is None else 'PASS' if abs(shift) <= 0.01 else 'WARN' if shift <= -0.02 else 'INFO')
        out.append(check('%s雷达远处没被挡的像素：有人 − 没人（中位，负值=变近）' % name, status, shift,
                         '≤1 cm 不受影响；≤ −2 cm 受影响',
                         '近处有亮物时远处被拉近；' + ('没人取有人前后两段，线性漂移基本抵消'
                                                      if single else '两段不是同时录的，含约 1 cm 量级的温漂')))
        if single and stats.get('before') and stats.get('after'):
            drift = far_shift(stats['before']['_median_range'], stats['after']['_median_range'])
            row['after_minus_before'] = drift
            value = drift['far_shift_m']
            out.append(check('%s雷达没人的后一段 − 前一段（远处像素中位）' % name,
                             'INFO' if value is None else 'PASS' if abs(value) <= 0.01 else 'WARN', value, 0.01,
                             '这段录包里的漂移或场景变化；超过 1 cm 时上一行不太可靠'))
        for key, label in (('floating_per_frame_median', '悬空点'), ('curl_per_frame_median', '地面上翘点')):
            ratio = b[key] / a[key] if b[key] is not None and a[key] else None
            out.append(check('%s雷达%s每帧：有人 / 没人' % (name, label), 'INFO', r(ratio, 2), None,
                             '有人：%s，没人：%s；人挡住了身后的地面，比值只作参考' % (r(b[key], 1), r(a[key], 1))))
    return info, out


# ------------------------------------------------------------------ bias
def run_bias(project, dataset, statement, max_norm=None, dry_run=False):
    from wc_runtime.mapping_bias_confirm import capture_calibration_rows
    from wc_runtime.storage_policy import StoragePolicy
    policy = StoragePolicy(project)
    policy.check()
    dataset = Path(dataset).expanduser().resolve(strict=True)
    rows, provenance = capture_calibration_rows(dataset)
    if not rows:
        raise SystemExit('录包里没有原生 IMU/轮速标定事件')
    start = int(rows[0]['stamp_ns'])
    manifest = json.loads((dataset / 'capture_manifest.json').read_text(encoding='utf-8'))
    window_start = manifest['acquisition_window']['start_monotonic_ns']
    first = int(rows[0]['monotonic_ns'])
    today = datetime.date.today().strftime('%Y%m%d')
    declaration = {
        'physically_stationary': True, 'source': 'operator_visual_confirmation',
        'evidence_id': '%s:opening_still_30s:operator_confirmed_%s' % (dataset.name, today),
        'start_stamp_ns': start, 'end_stamp_ns': start + 30_000_000_000,
        'sensor_id': provenance['sensor_id'], 'dataset': str(dataset), 'operator_statement': statement,
        'interval_relative_to_acquisition_window_s': [round((first - window_start) * 1e-9, 3),
                                                      round((first - window_start) * 1e-9 + 30.0, 3)]}
    folder = Path(policy.resolve('data/calibration/gyro_bias'))
    declaration_path = folder / ('%s_stationary_declaration.json' % dataset.name)
    output = folder / ('%s_gyro_bias.json' % dataset.name)
    command = [sys.executable, '-s', '-m', 'wc_runtime.mapping_bias_confirm', '--capture-dataset', str(dataset),
               '--stationary-evidence', str(declaration_path), '--output', str(output)]
    if max_norm is not None:
        command += ['--max-bias-norm-rad-s', str(max_norm)]
    print(json.dumps({'declaration': declaration, 'declaration_path': str(declaration_path), 'output': str(output),
                      'command': command}, ensure_ascii=False, indent=1), flush=True)
    text = json.dumps(declaration, ensure_ascii=False, indent=2) + '\n'
    if dry_run:
        if declaration_path.exists():
            print('已有静止声明：' + ('内容相同，正式运行会沿用' if declaration_path.read_text(encoding='utf-8') == text
                                      else '内容不同，正式运行会拒绝（--operator-statement 要用同一句原话、同一天）'), flush=True)
        return 0
    folder.mkdir(parents=True, exist_ok=True)
    if write_declaration(declaration_path, text) == 'reused':
        print('沿用已有的静止声明（内容完全相同）：' + str(declaration_path), flush=True)
    done = subprocess.run(command, cwd=str(project), stdout=subprocess.PIPE, text=True)
    print(done.stdout, end='', flush=True)
    if done.returncode == 0 and output.is_file():
        bias = json.loads(output.read_text(encoding='utf-8'))
        print('零偏确认通过：', json.dumps(bias.get('bias_native_rad_s', bias), ensure_ascii=False), flush=True)
        value = bias.get('bias_native_rad_s')
        if isinstance(value, list) and len(value) == 3:
            delta = [float(a) - b for a, b in zip(value, REFERENCE_BIAS_RAD_S)]
            print('和 11-07 建图录包（%s）比：Δx %+.5f  Δy %+.5f  Δz %+.5f rad/s；%s' % (
                REFERENCE_BIAS_NAME, *delta,
                'z 在 ±0.0005 以内，和 11-07 一致' if abs(delta[2]) <= 0.0005 else 'z 差超过 0.0005，记下来，以这次为准'),
                flush=True)
        if max_norm is not None:
            print('注意：这次零偏模长和静止段均值门限放宽到 %g rad/s（默认 0.0015），留出段残差门限不变；结论里注明。' % max_norm,
                  flush=True)
        return 0
    reasons = rejection_reasons(done.stdout)
    if reasons and set(reasons) <= NORM_REASONS:
        if max_norm is None:
            print('没通过只是因为零偏模长或静止段均值超过默认 0.0015 rad/s（%s）。用户 2026-11-08 20:10 同意：'
                  '同一条命令加 --max-bias-norm-rad-s 0.003 再跑一次，留出段残差门限不变，结论里注明。' % ','.join(reasons),
                  flush=True)
        else:
            print('门限放宽到 %g rad/s 仍超限（%s），不要再放宽，找 A。' % (max_norm, ','.join(reasons)), flush=True)
    elif reasons:
        print('没通过的原因里有 %s，不只是模长超限；不要放宽门限，按原因重录或找 A。'
              % ','.join(sorted(set(reasons) - NORM_REASONS)), flush=True)
    print('零偏确认没有通过（退出码 %d），看上面的原因；不通过时不保存，不要当作零偏 0。' % done.returncode, flush=True)
    return done.returncode or 1


def write_declaration(path, text):
    """写静止声明；同名文件已存在且内容完全相同时沿用（放宽模长门限后重跑同一段），不同就拒绝。"""
    try:
        with path.open('x', encoding='utf-8') as stream:
            stream.write(text)
        return 'written'
    except FileExistsError:
        if path.read_text(encoding='utf-8') == text:
            return 'reused'
        raise SystemExit('静止声明已存在且内容不同：%s。重跑时 --operator-statement 要用同一句原话、同一天；否则找 A。' % path)


def rejection_reasons(stdout):
    """mapping_bias_confirm 最后一行 JSON 里 '10+10+10 calibration rejected: A,B' 的原因列表；没有就返回 []。"""
    for line in reversed(stdout.strip().splitlines()):
        try:
            message = json.loads(line)
        except ValueError:
            continue
        error = message.get('error', '') if isinstance(message, dict) else ''
        marker = 'calibration rejected: '
        return [x.strip() for x in error.split(marker, 1)[1].split(',') if x.strip()] if marker in error else []
    return []


# ------------------------------------------------------------------ 输出
def clean(value):
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items() if not str(k).startswith('_')}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, (np.floating, float)):
        return None if not np.isfinite(value) else float(value)
    if isinstance(value, np.integer):
        return int(value)
    return value


def write_result(project, kind, name, payload):
    from wc_runtime.storage_policy import StoragePolicy
    policy = StoragePolicy(project)
    policy.check()
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    out = Path(policy.resolve('reports/field_site_checks/%s_%s_%s' % (stamp, kind, re.sub(r'[^A-Za-z0-9_.-]', '_', name))))
    out.mkdir(parents=True, exist_ok=False)
    (out / 'result.json').write_text(json.dumps(clean(payload), indent=1, ensure_ascii=False, allow_nan=False) + '\n',
                                     encoding='utf-8')
    return out


def show(kind, out, checks, info):
    print('\n== %s：%s' % (kind, out), flush=True)
    for row in checks:
        print('  %-5s %s：%s（门限 %s）—— %s' % (row['status'], row['name'], json.dumps(row['value'], ensure_ascii=False),
                                             row['limit'], row['meaning']), flush=True)
    if info.get('error'):
        print('  错误：' + info['error'], flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--project-root', type=Path, required=True)
    sub = parser.add_subparsers(dest='command', required=True)
    bias = sub.add_parser('bias', help='开头 30 s 静止的 10+10+10 零偏确认')
    bias.add_argument('--dataset', type=Path, required=True)
    bias.add_argument('--operator-statement', required=True, help='用户确认开头 30 s 车没人碰的原话')
    bias.add_argument('--max-bias-norm-rad-s', type=float,
                      help='只因零偏模长超限没通过时加 0.003 重跑（用户 2026-11-08 20:10 同意）；留出段残差门限不变')
    bias.add_argument('--dry-run', action='store_true', help='只显示声明区间和命令，不写文件')
    for name in ('wall', 'push'):
        p = sub.add_parser(name)
        p.add_argument('--dataset', type=Path, required=True)
        p.add_argument('--hardware-setup', type=Path, help='覆盖录包冻结的 hardware_setup（例如旧录包的轮参数）')
    sub.choices['wall'].add_argument('--tape', type=float, nargs='+', default=[4.0, 2.0, 1.0, 0.5, 0.3, 0.1, 4.0],
                                     help='车头最前点到墙的卷尺距离（m），按停车先后顺序；同一距离停两次时给出两次之差')
    sub.choices['wall'].add_argument('--segments', type=int, nargs='+', help='静止段序号，和 --tape 一一对应')
    sub.choices['push'].add_argument('--max-range', type=float, default=7.5, help='尽头墙最远距离（m）')
    tape = sub.add_parser('tape')
    tape.add_argument('--front-cm', type=float, required=True, help='右雷达外壳前脸比左雷达靠前多少（cm，靠前为正）')
    tape.add_argument('--height-cm', type=float, required=True, help='左雷达外壳比右雷达高多少（cm，左高为正）')
    tape.add_argument('--lateral-cm', type=float, help='两台外壳中心的左右间距（cm）')
    tape.add_argument('--dataset', type=Path, help='取这段录包冻结的外参；不给就用当前设备配置')
    iso = sub.add_parser('isolation')
    iso.add_argument('--both', type=Path, required=True)
    iso.add_argument('--left-only', type=Path)
    iso.add_argument('--right-only', type=Path)
    iso.add_argument('--window', type=float, nargs=2, metavar=('START_S', 'END_S'),
                     help='只用每段录包这段时间（秒，从采集窗起点算）；默认整段')
    iso.add_argument('--stride', type=int, default=2, help='每隔几帧取一帧（默认 2，即 5 Hz）')
    near = sub.add_parser('nearby', help='车前站人和没人比：远处像素距离、悬空点')
    near.add_argument('--dataset', type=Path, help='同一段录包：前后没人、中间有人，自动分段')
    near.add_argument('--window', type=float, nargs=2, metavar=('START_S', 'END_S'), help='--dataset 只用这段时间（秒）')
    near.add_argument('--with', dest='with_dataset', type=Path, help='或者两段录包：车前站人的那段')
    near.add_argument('--without', dest='without_dataset', type=Path, help='和同一摆放、车前没人的那段')
    near.add_argument('--with-window', type=float, nargs=2, metavar=('START_S', 'END_S'), help='只用这段时间（秒）')
    near.add_argument('--without-window', type=float, nargs=2, metavar=('START_S', 'END_S'), help='只用这段时间（秒）')
    near.add_argument('--stride', type=int, default=4, help='悬空点每隔几帧算一次（默认 4）；距离图用每一帧')
    args = parser.parse_args(argv)
    project = args.project_root.resolve()
    if args.command == 'bias':
        return run_bias(project, args.dataset, args.operator_statement, args.max_bias_norm_rad_s, args.dry_run)
    if args.command == 'tape':
        if args.dataset:
            mounts, source = Recording(args.dataset).mounts, str(args.dataset)
        else:
            from wc_runtime.hardware_setup import resolve_hardware_setup
            from wc_runtime.project_config import selected_config_path
            path = Path(selected_config_path(project, 'hardware_setup.json'))
            mounts = {s: (np.asarray(m['R_axle_lidar']), np.asarray(m['t_axle_lidar_m']))
                      for s, m in resolve_hardware_setup(json.loads(path.read_text(encoding='utf-8')))['mounts'].items()}
            source = str(path)
        info, checks = analyze_tape(mounts, args.front_cm, args.height_cm, args.lateral_cm)
        info['extrinsic_source'] = source
        out = write_result(project, 'tape', Path(source).name, {'kind': 'tape', 'info': info, 'checks': checks})
        show('tape', out, checks, info)
        return 0
    if args.command == 'isolation':
        recs = {'both': Recording(args.both)}
        if args.left_only:
            recs['left_only'] = Recording(args.left_only)
        if args.right_only:
            recs['right_only'] = Recording(args.right_only)
        info, checks = analyze_isolation(recs, tuple(args.window) if args.window else None, max(1, args.stride))
        out = write_result(project, 'isolation', recs['both'].dir.name, {'kind': 'isolation', 'info': info, 'checks': checks})
        show('isolation', out, checks, info)
        return 0
    if args.command == 'nearby':
        if bool(args.dataset) == bool(args.with_dataset or args.without_dataset) or (
                not args.dataset and not (args.with_dataset and args.without_dataset)):
            parser.error('nearby：给 --dataset，或者 --with 和 --without 一起')
        stride = max(1, args.stride)
        if args.dataset:
            with_rec = Recording(args.dataset)
            info, checks = analyze_nearby(with_rec, window=tuple(args.window) if args.window else None, stride=stride)
        else:
            with_rec = Recording(args.with_dataset)
            without_rec = with_rec if args.without_dataset.resolve() == with_rec.dir else Recording(args.without_dataset)
            info, checks = analyze_nearby(with_rec, without_rec, tuple(args.with_window) if args.with_window else None,
                                          tuple(args.without_window) if args.without_window else None, stride=stride)
        out = write_result(project, 'nearby', with_rec.dir.name, {'kind': 'nearby', 'info': info, 'checks': checks})
        show('nearby', out, checks, info)
        return 0 if not any(row['status'] == 'FAIL' for row in checks) else 1
    rec = Recording(args.dataset, args.hardware_setup)
    if not rec.complete():
        print('注意：录包状态不是 COMPLETE，结果只作诊断', flush=True)
    if args.command == 'wall':
        info, checks = analyze_wall(rec, args.tape, args.segments)
    else:
        info, checks = analyze_push(rec, args.max_range)
    info.update(recording=str(rec.dir), hardware_setup=str(rec.setup_path), sides=rec.sides, complete=rec.complete())
    out = write_result(project, args.command, rec.dir.name, {'kind': args.command, 'info': info, 'checks': checks})
    show(args.command, out, checks, info)
    return 0 if not any(row['status'] == 'FAIL' for row in checks) else 1


if __name__ == '__main__':
    raise SystemExit(main())
