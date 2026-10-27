"""单雷达逐帧 ICP 里程计，以及雷达与轮速轨迹的平面手眼（求雷达相对车体的偏航）。

ICP 复用 wc_runtime.icp_shadow.point_to_plane。平面手眼先用各雷达自己的竖直方向找平，
再解 (R_b − I)·t_x − R(ψ)·t_l + λ·t_b = 0 的齐次最小二乘；λ 是轮速平移尺度，ψ 是安装偏航。
"""
import os
from multiprocessing import Pool

import numpy as np
from scipy.spatial.transform import Rotation

from wc_runtime.icp_shadow import ICPPolicy, point_to_plane

from .geometry import clean_cloud, level_rotation

ODOM_POLICY = dict(max_points=4000, max_iterations=25, max_correspondence_m=0.25,
                   max_correction_translation_m=0.2, max_correction_rotation_deg=8.0,
                   max_residual_m=0.06, min_inlier_ratio=0.3)
_CLOUDS = {}


def _odometry(side):
    policy = ICPPolicy(**ODOM_POLICY)
    clouds = _CLOUDS[side]
    transforms, accepted = [], []
    guess, previous = np.eye(4), clean_cloud(clouds[0])
    for k in range(1, len(clouds)):
        current = clean_cloud(clouds[k])
        report = point_to_plane(current, previous, guess, policy)
        T = np.asarray(report['T_target_source_shadow']) if report['accepted'] else np.eye(4)
        transforms.append(T)
        accepted.append(bool(report['accepted']))
        guess = T if report['accepted'] else np.eye(4)
        previous = current
    return (np.stack(transforms) if transforms else np.zeros((0, 4, 4))), np.asarray(accepted, bool)


def lidar_odometry(capture, processes=2):
    """返回 {side: dict(T[k]=第 k+1 帧到第 k 帧, ok[k], P[k]=第 k 帧在首帧坐标中的位姿)}。"""
    for side in ('left', 'right'):
        _CLOUDS[side] = capture['lidar'][side]['clouds']
    os.environ.setdefault('OMP_NUM_THREADS', '2')
    with Pool(processes) as pool:
        results = pool.map(_odometry, ['left', 'right'])
    out = {}
    for side, (T, ok) in zip(('left', 'right'), results):
        poses = [np.eye(4)]
        for M in T:
            poses.append(poses[-1] @ M)
        out[side] = dict(T=T, ok=ok, P=np.stack(poses))
    return out


def wheel_poses(track, times_s):
    """轮速积分的车体平面位姿 (x, y, yaw) 在指定时刻（秒，相对录制窗口）的插值。"""
    t, v, omega = track['t'], track['v'], track['omega']
    dt = np.r_[0, np.diff(t)]
    dt[dt > 0.5] = 0
    yaw = np.cumsum(np.r_[0, 0.5 * (omega[1:] + omega[:-1]) * dt[1:]])
    vx, vy = v * np.cos(yaw), v * np.sin(yaw)
    x = np.cumsum(np.r_[0, 0.5 * (vx[1:] + vx[:-1]) * dt[1:]])
    y = np.cumsum(np.r_[0, 0.5 * (vy[1:] + vy[:-1]) * dt[1:]])
    return np.column_stack([np.interp(times_s, t, x), np.interp(times_s, t, y), np.interp(times_s, t, yaw)])


def _se2(x, y, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def planar_hand_eye(odometry, frame_times_s, base_track, up, window=10, min_turn_deg=5.0, bootstrap=200, seed=0):
    """雷达相对车体的偏航 ψ（度）、雷达在车体中的 xy 和轮速尺度 λ，附自助法标准差。

    只用逐帧 ICP 全部接受的约 1 秒窗口；frame_times_s 是该雷达各帧相对录制窗口的秒数。
    """
    T, ok = odometry['T'], odometry['ok']
    R_level = level_rotation(up)
    rows, base_yaw, lidar_yaw = [], [], []
    for s in range(0, len(T) - window, window // 2):
        if not ok[s:s + window].all():
            continue
        M = np.eye(4)
        for k in range(s, s + window):
            M = M @ T[k]
        p0, p1 = wheel_poses(base_track, [frame_times_s[s], frame_times_s[s + window]])
        Tb = np.linalg.inv(_se2(*p0)) @ _se2(*p1)
        Rl = R_level @ M[:3, :3] @ R_level.T
        tl = R_level @ M[:3, 3]
        lidar_yaw.append(np.arctan2(Rl[1, 0], Rl[0, 0]))
        base_yaw.append(np.arctan2(Tb[1, 0], Tb[0, 0]))
        rows.append(np.column_stack([Tb[:2, :2] - np.eye(2),
                                     -np.array([[tl[0], -tl[1]], [tl[1], tl[0]]]), Tb[:2, 2]]))
    if len(rows) < 30:
        raise ValueError('可用运动窗口太少（%d），需要更多平稳行驶和转弯' % len(rows))
    A = np.vstack(rows)

    def solve(matrix):
        v = np.linalg.svd(matrix)[2][-1]
        v = v / np.hypot(v[2], v[3])
        return -v if v[4] < 0 else v

    v = solve(A)
    rng = np.random.default_rng(seed)
    n = len(rows)
    boot = []
    for _ in range(bootstrap):
        pick = rng.integers(0, n, n)
        w = solve(A[np.concatenate([[2 * i, 2 * i + 1] for i in pick])])
        boot.append([w[0], w[1], np.degrees(np.arctan2(w[3], w[2])), w[4]])
    boot = np.array(boot)
    base_yaw, lidar_yaw = np.array(base_yaw), np.array(lidar_yaw)
    turning = np.abs(base_yaw) > np.radians(min_turn_deg)
    return dict(windows=n, turning_windows=int(turning.sum()),
                yaw_deg=round(float(np.degrees(np.arctan2(v[3], v[2]))), 3),
                xy_in_base_m=[round(float(v[0]), 3), round(float(v[1]), 3)],
                wheel_translation_scale=round(float(v[4]), 4),
                std=dict(yaw_deg=round(float(boot[:, 2].std()), 3), x_m=round(float(boot[:, 0].std()), 3),
                         y_m=round(float(boot[:, 1].std()), 3), scale=round(float(boot[:, 3].std()), 4)),
                yaw_sign_agreement=round(float(np.mean(np.sign(base_yaw[turning]) == np.sign(lidar_yaw[turning]))), 3)
                if turning.any() else None)


def lidar_rotation_rate(odometry, frame_times_s):
    """逐帧旋转角速度（rad/s，绕雷达坐标 z 的分量带符号），用于与 IMU 互相关对时。"""
    T = odometry['T']
    rates, times = [], []
    for k in range(len(T)):
        if not odometry['ok'][k]:
            continue
        dt = frame_times_s[k + 1] - frame_times_s[k]
        if dt <= 0:
            continue
        rates.append(Rotation.from_matrix(T[k][:3, :3]).as_rotvec() / dt)
        times.append(0.5 * (frame_times_s[k] + frame_times_s[k + 1]))
    return np.asarray(times), np.asarray(rates)
