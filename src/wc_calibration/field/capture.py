"""只读加载完整录包中的左右雷达 filtered 点云与轮速反馈，供现场外参检查与离线求解。

轮速换算与 wc_motion.history_preview 一致：寄存器值 × register_to_wheel_rpm × 符号 → 轮转速，
再乘轮半径。读取的是录包冻结的 hardware_setup，不读当前配置。
"""
import json
import sqlite3
from pathlib import Path

import numpy as np

WIDTH, HEIGHT = 160, 60
POINTS = WIDTH * HEIGHT
CLOUD_TOPIC = '/wc_mapping/lidar_%s/source_frame_filtered'


def _decoder():
    from rclpy.serialization import deserialize_message
    from wc_interfaces.msg import SourceFrame
    return lambda raw: deserialize_message(raw, SourceFrame)


def load_capture(directory, decoder=None):
    """返回 dict(dir, manifest, lidar={side:{clouds[N,9600,3], mono[N]}}, wheel[M,3], wheel_cfg)。"""
    from wc_sensors.pointcloud import decode_pointcloud2
    directory = Path(directory)
    manifest = json.loads((directory / 'capture_manifest.json').read_text())
    decode = decoder or _decoder()
    bags = sorted((directory / manifest.get('bag_path', 'bag')).glob('*.db3'))
    if not bags:
        raise ValueError('录包缺少 SQLite 数据：' + str(directory))
    lidar = {side: dict(clouds=[], mono=[]) for side in ('left', 'right')}
    for bag in bags:
        db = sqlite3.connect(bag.as_uri() + '?mode=ro', uri=True)
        try:
            topics = {name: tid for tid, name in db.execute('SELECT id,name FROM topics')}
            for side in lidar:
                tid = topics.get(CLOUD_TOPIC % side)
                if tid is None:
                    continue
                for (data,) in db.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY id', (tid,)):
                    message = decode(data)
                    if message.units != 'm' or message.coordinate_convention != 'FLU':
                        raise ValueError('点云不是米制 FLU')
                    xyz = decode_pointcloud2(message.cloud).astype(np.float32)
                    fixed = np.full((POINTS, 3), np.nan, np.float32)
                    fixed[:min(len(xyz), POINTS)] = xyz[:POINTS]
                    lidar[side]['clouds'].append(fixed)
                    lidar[side]['mono'].append(int(message.host_monotonic_ns))
        finally:
            db.close()
    for side, row in lidar.items():
        order = np.argsort(row['mono'])
        row['clouds'] = np.stack(row['clouds'])[order] if row['clouds'] else np.zeros((0, POINTS, 3), np.float32)
        row['mono'] = np.asarray(row['mono'], np.int64)[order]
    setup = json.loads((directory / 'configuration' / 'hardware_setup.json').read_text())
    config = setup['wheel_odometry']
    rows = []
    journal = directory / 'sources' / 'wheel_feedback.jsonl'
    if journal.is_file():
        with journal.open(encoding='utf-8') as stream:
            for line in stream:
                record = json.loads(line)
                if record.get('event') != 'transaction_complete' or record.get('status') != 'RESPONSE_VALID':
                    continue
                words = [w - 65536 if w >= 32768 else w for w in record['register_words_u16']]
                rpm = (words[0] * config['register_to_wheel_rpm'] * config['left_sign'],
                       words[1] * config['register_to_wheel_rpm'] * config['right_sign'])
                rows.append((record['receive_monotonic_ns'],
                             *(value * 2 * np.pi / 60 * config['wheel_radius_m'] for value in rpm)))
    wheel = np.asarray(rows, dtype=np.float64).reshape(-1, 3)
    return dict(dir=str(directory), manifest=manifest, lidar=lidar, wheel=wheel, wheel_cfg=config,
                imu=_load_imu(bags))


IMU_TOPIC = '/wc_mapping/imu/source_frame'


def _load_imu(bags):
    """IMU 角速度（原生轴，当前装配与 M 系同向），按到达单调时间排列：[N,4] = t_ns, wx, wy, wz。"""
    try:
        from rclpy.serialization import deserialize_message
        from wc_interfaces.msg import H30Frame
    except ImportError:
        return np.zeros((0, 4))
    rows = []
    for bag in bags:
        db = sqlite3.connect(bag.as_uri() + '?mode=ro', uri=True)
        try:
            topics = {name: tid for tid, name in db.execute('SELECT id,name FROM topics')}
            tid = topics.get(IMU_TOPIC)
            if tid is None:
                continue
            for (data,) in db.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY id', (tid,)):
                m = deserialize_message(data, H30Frame)
                if not m.angular_velocity_valid:
                    continue
                w = m.imu.angular_velocity
                rows.append((int(m.host_monotonic_ns), w.x, w.y, w.z))
        finally:
            db.close()
    out = np.asarray(rows, dtype=np.float64).reshape(-1, 4)
    return out[np.argsort(out[:, 0])]


def window(capture):
    w = capture['manifest'].get('acquisition_window', {})
    start, end = w.get('start_monotonic_ns'), w.get('end_monotonic_ns')
    if type(start) is not int or type(end) is not int or end <= start:
        raise ValueError('录包缺少正式采集时间窗')
    return start, end


def wheel_track(capture, min_still_s=2.0, still_speed=0.01, yaw_source='auto', imu_delay_s=0.0):
    """录制窗口内轮速积分与静止段。静止 = 两轮速度都 < still_speed（停下时读数恰为 0）。

    yaw_source='imu' 用 IMU 陀螺 z（扣除静止段零偏）作车体角速度，'wheel' 用左右轮速差；
    'auto' 有 IMU 就用 IMU。轮距未标定，轮速差分的转向误差比陀螺大。
    imu_delay_s：IMU 到达时间相对轮速到达时间的滞后，正值表示 IMU 晚到。
    """
    start, end = window(capture)
    w = capture['wheel']
    w = w[(w[:, 0] >= start) & (w[:, 0] <= end)]
    t = (w[:, 0] - start) * 1e-9
    vl, vr = w[:, 1], w[:, 2]
    v = (vl + vr) / 2
    still = (np.abs(vl) < still_speed) & (np.abs(vr) < still_speed)
    omega = (vr - vl) / capture['wheel_cfg']['track_width_m']
    imu = capture.get('imu')
    source = 'wheel'
    if yaw_source in ('auto', 'imu') and imu is not None and len(imu) > 100:
        ti = (imu[:, 0] - start) * 1e-9 - imu_delay_s
        gz = imu[:, 3]
        still_imu = np.interp(ti, t, still.astype(float)) > 0.99
        bias = float(np.median(gz[still_imu])) if still_imu.sum() > 200 else float(np.median(gz))
        omega = np.interp(t, ti, gz - bias)
        source = 'imu'
    elif yaw_source == 'imu':
        raise ValueError('录包没有 IMU 角速度')
    dt = np.r_[0, np.diff(t)]
    dt[dt > 0.5] = 0  # 中断处不积分
    s = np.cumsum(np.r_[0, 0.5 * (v[1:] + v[:-1]) * dt[1:]])
    yaw = np.cumsum(np.r_[0, 0.5 * (omega[1:] + omega[:-1]) * dt[1:]])
    segments, first = [], None
    for k, flag in enumerate(np.r_[still, False]):
        if flag and first is None:
            first = k
        elif not flag and first is not None:
            if t[k - 1] - t[first] >= min_still_s:
                segments.append((first, k - 1))
            first = None
    return dict(t=t, s=s, yaw=yaw, v=v, omega=omega, segments=segments, yaw_source=source,
                duration=(end - start) * 1e-9, start_ns=start)


def frames_in(capture, side, t_from, t_to):
    """录制窗口内 [t_from, t_to] 秒的帧序号。"""
    start, _ = window(capture)
    rel = (capture['lidar'][side]['mono'] - start) * 1e-9
    return np.where((rel >= t_from) & (rel <= t_to))[0]


def still_pairs(capture, per_segment=3, max_dt_ns=30_000_000, margin_s=0.5):
    """每个轮速静止段取 per_segment 对同时刻左右帧序号 (i_left, j_right, t_rel)。"""
    track = wheel_track(capture)
    left, right = capture['lidar']['left']['mono'], capture['lidar']['right']['mono']
    out = []
    for segment, (a, b) in enumerate(track['segments']):
        idx = frames_in(capture, 'left', track['t'][a] + margin_s, track['t'][b] - margin_s)
        if not len(idx):
            continue
        for i in idx[np.linspace(0, len(idx) - 1, min(per_segment, len(idx))).astype(int)]:
            j = int(np.argmin(np.abs(right - left[i])))
            if abs(int(right[j]) - int(left[i])) <= max_dt_ns:
                out.append((int(i), j, segment))
    return out
