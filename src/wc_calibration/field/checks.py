"""三类现场录制的合格判断。每个函数返回 (fails: list[str], info: dict)；fails 为空即合格。

- wall：正对平墙静止，量墙面平整度和墙前地面的测距偏斜。
- approach：正对墙从约 4.5 m 推到约 1.5–2 m，雷达测墙距作真值，求轮速比例。
- room：房间里多处停顿，逐处做左右点到面配准；多次录制合并后看中位数是否稳定。
判据数值是现场经验门限，不是精度证书。
"""
import warnings

import numpy as np
from scipy.spatial.transform import Rotation

from wc_runtime.icp_shadow import ICPPolicy, point_to_plane

from .capture import frames_in, still_pairs, wheel_track
from .geometry import NAMES6, clean_cloud, fit_plane, ransac_plane

# 机械名义值：右雷达在左雷达左侧 −0.4575 m，平行
MECHANICAL_T_LEFT_RIGHT = np.eye(4)
MECHANICAL_T_LEFT_RIGHT[:3, 3] = [0.0, -0.4575, 0.0]
CROSS_POLICY = dict(max_points=5000, max_iterations=40, max_correspondence_m=0.3,
                    max_correction_translation_m=0.3, max_correction_rotation_deg=10.0,
                    max_residual_m=0.08, min_inlier_ratio=0.3)  # 视野只部分重叠，残差偏大
SIDE_NAMES = {'left': '左', 'right': '右'}


def basic_checks(capture, expect_s, fails, info):
    manifest = capture['manifest']
    if manifest.get('status') != 'COMPLETE' or manifest.get('recording_complete') is not True:
        fails.append('录制没有正常完成（状态 %s），需要重录' % manifest.get('status'))
    for side, name in SIDE_NAMES.items():
        n = len(capture['lidar'][side]['mono'])
        info['%s_frames' % side] = n
        if n < 8 * expect_s:
            fails.append('%s雷达帧数太少（%d 帧，预期约 %d）' % (name, n, 10 * expect_s))
    n = len(capture['wheel'])
    info['wheel_samples'] = n
    if n < 8 * expect_s:
        fails.append('轮速反馈太少（%d 条），检查轮速串口' % n)
    elif n > 1:
        gap = float(np.diff(capture['wheel'][:, 0]).max() * 1e-9)
        info['wheel_max_gap_s'] = round(gap, 3)
        if gap > 0.5:
            fails.append('轮速反馈中断 %.1f 秒' % gap)


def _median_image(clouds):
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        return np.nanmedian(clouds.reshape(len(clouds), 60, 160, 3), axis=0)


# ------------------------------------------------------------------ wall
def analyze_wall(capture, stated_distance_m):
    fails, info = [], dict(stated_distance_m=stated_distance_m)
    basic_checks(capture, 15, fails, info)
    if fails:
        return fails, info
    for side, name in SIDE_NAMES.items():
        c = capture['lidar'][side]['clouds'].reshape(-1, 60, 160, 3)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            q = max(3, len(c) // 4)
            r0 = np.nanmedian(np.linalg.norm(c[:q], axis=-1), axis=0)
            r1 = np.nanmedian(np.linalg.norm(c[-q:], axis=-1), axis=0)
        diff = np.abs(r1 - r0)[np.isfinite(r0) & np.isfinite(r1)]
        moved = float(np.median(diff)) if diff.size else np.inf
        changed = float(np.mean(diff > 0.05)) if diff.size else 1.0
        row = dict(range_change_median_m=round(moved, 4), pixels_changed_over_5cm=round(changed, 3))
        if moved > 0.03 or changed > 0.10:  # 静止录包实测噪声约 1 cm
            fails.append('%s雷达开头和结尾的画面不一致（测距中位变化 %.1f cm，%.0f%% 像素变化超过 5 cm）：'
                         '录制期间轮椅或人动了' % (name, 100 * moved, 100 * changed))
        med = _median_image(c[::2])
        wall_points = med[0:44].reshape(-1, 3)
        wall_points = wall_points[np.isfinite(wall_points).all(1) & (wall_points[:, 0] > 0.5)]
        floor_points = med[40:60].reshape(-1, 3)
        floor_points = floor_points[np.isfinite(floor_points).all(1) & (floor_points[:, 2] < -0.4)
                                    & (floor_points[:, 0] > 0.5)]
        wall = ransac_plane(wall_points, 'v') if len(wall_points) > 500 else None
        if wall is not None:  # 墙根附近的点不算地面
            floor_points = floor_points[np.abs(floor_points @ wall[0] + wall[1]) > 0.08]
        floor = ransac_plane(floor_points, 'h') if len(floor_points) > 300 else None
        if wall is None or wall[2].sum() < 1500 or wall[3] > 0.02:
            fails.append('%s雷达没有找到足够大的平整墙面（要求 ≥1500 点、RMS ≤2 cm）：正对一面空墙，别有人站在中间' % name)
        else:
            row.update(wall_points=int(wall[2].sum()), wall_rms_m=round(wall[3], 4),
                       wall_distance_m=round(wall[1], 3), wall_normal=np.round(wall[0], 4).tolist())
        if floor is None or floor[2].sum() < 300 or floor[3] > 0.02:
            fails.append('%s雷达看到的地面太少（要求 ≥300 点、RMS ≤2 cm）：雷达和墙之间的地面要空出来' % name)
        else:
            n, d = floor[0], floor[1]
            row.update(floor_points=int(floor[2].sum()), floor_rms_m=round(floor[3], 4),
                       floor_height_at_2p5m_m=round(float((d + n[0] * 2.5) / n[2]), 3),
                       floor_normal=np.round(n, 4).tolist())
        if wall is not None and floor is not None:
            angle = float(np.degrees(np.arccos(np.clip(abs(wall[0] @ floor[0]), 0, 1))))
            row.update(wall_floor_angle_deg=round(angle, 2), floor_bias_deg=round(90 - angle, 2))
            bands, fp = {}, floor_points[floor[2]]
            for lo, hi in ((0.5, 1.5), (1.5, 2.5), (2.5, 4.0)):
                q = fp[(fp[:, 0] >= lo) & (fp[:, 0] < hi)]
                if len(q) >= 80:
                    n, _ = fit_plane(q)
                    n = n if n[2] > 0 else -n
                    bands['%.1f-%.1fm' % (lo, hi)] = round(float(np.degrees(-np.arctan2(n[0], n[2]))), 2)
            row['floor_pitch_by_range_deg'] = bands
        info[side] = row
    return fails, info


# ------------------------------------------------------------------ approach
def front_wall_distance(capture, side, t_from, t_to):
    """静止段内雷达到正前方墙面的垂直距离 (d, 墙面偏离正前方角度°, 点数)；ToF 绝对测距作真值。"""
    idx = frames_in(capture, side, t_from, t_to)
    if len(idx) < 5:
        return None
    med = _median_image(capture['lidar'][side]['clouds'][idx])[4:44]
    points = med.reshape(-1, 3)
    points = points[np.isfinite(points).all(1) & (points[:, 0] > 0.5)]
    wall = ransac_plane(points, 'v') if len(points) > 500 else None
    if wall is None or wall[2].sum() < 800 or abs(wall[0][0]) < 0.9 or wall[3] > 0.02:
        return None
    return wall[1], float(np.degrees(np.arccos(min(1.0, abs(wall[0][0]))))), int(wall[2].sum())


def analyze_approach(capture):
    fails, info = [], {}
    track = wheel_track(capture)
    basic_checks(capture, track['duration'], fails, info)
    if fails:
        return fails, info
    t, s, yaw, segments = track['t'], track['s'], track['yaw'], track['segments']
    info['stops_s'] = [[round(float(t[a]), 1), round(float(t[b]), 1)] for a, b in segments]
    if len(segments) < 2 or t[segments[0][0]] > 1.0:
        fails.append('需要"开头静止 ≥2 秒 → 推向墙 → 停住 ≥2 秒"（轮速判断），当前静止段：%s' % info['stops_s'])
        return fails, info
    first, last = segments[0], segments[-1]
    distance = float(s[last[0]] - s[first[1]])
    turn = float(np.degrees(yaw[last[0]] - yaw[first[1]]))
    info.update(wheel_distance_m=round(distance, 3), wheel_yaw_deg=round(turn, 1))
    if abs(turn) > 10:
        fails.append('推的过程中转了 %.0f°（要求 ≤10°）：正对墙直线推' % turn)
    ratios = {}
    for side, name in SIDE_NAMES.items():
        a = front_wall_distance(capture, side, t[first[0]], t[first[1]])
        b = front_wall_distance(capture, side, t[last[0]], t[last[1]])
        if a is None or b is None:
            fails.append('%s雷达在%s没有看到足够大的正前方平墙：轮椅要正对一面空墙' % (name, '起点' if a is None else '终点'))
            continue
        moved = a[0] - b[0]
        info[side] = dict(wall_start_m=round(a[0], 3), wall_end_m=round(b[0], 3), lidar_distance_m=round(moved, 3),
                          wall_tilt_start_deg=round(a[1], 1), wall_tilt_end_deg=round(b[1], 1))
        if moved < 1.5:
            fails.append('%s雷达测得只靠近墙 %.2f m（要求 ≥1.5 m）：从离墙 4.5 m 左右推到 1.5–2 m' % (name, moved))
        elif abs(distance) > 0.1:
            ratios[side] = moved / abs(distance)
    if fails:
        return fails, info
    scale = float(np.mean(list(ratios.values())))
    info.update(lidar_left_right_disagreement_pct=round(100 * abs(ratios['left'] - ratios['right']) / scale, 1),
                wheel_scale_factor=round(scale, 4), wheel_scale_error_pct=round(100 * (1 / scale - 1), 1))
    if info['lidar_left_right_disagreement_pct'] > 3:
        fails.append('左右雷达算出的距离相差 %.1f%%（要求 ≤3%%）：推的时候保持车头正对墙'
                     % info['lidar_left_right_disagreement_pct'])
    return fails, info


def approach_agreement(scales, tolerance=0.03):
    """两次及以上推墙结果中，任意两次相差 ≤tolerance 才算确认；返回 (是否确认, 相互一致的一对)。"""
    for i, a in enumerate(scales):
        for b in scales[i + 1:]:
            if abs(a / b - 1) <= tolerance:
                return True, (a, b)
    return False, None


# ------------------------------------------------------------------ room
def per_stop_registration(capture, per_segment=5, min_ok=3):
    """每个轮速静止段做若干次左右点到面配准，取中位数。返回 [[tx,ty,tz,roll,pitch,yaw], ...]。"""
    policy = ICPPolicy(**CROSS_POLICY)
    L, R = capture['lidar']['left']['clouds'], capture['lidar']['right']['clouds']
    by_segment = {}
    for i, j, segment in still_pairs(capture, per_segment=per_segment):
        report = point_to_plane(clean_cloud(R[j]), clean_cloud(L[i]), MECHANICAL_T_LEFT_RIGHT, policy)
        if report['accepted']:
            T = np.asarray(report['T_target_source_shadow'])
            by_segment.setdefault(segment, []).append(
                [*T[:3, 3], *Rotation.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)])
    stops = [np.median(v, axis=0) for _, v in sorted(by_segment.items()) if len(v) >= min_ok]
    return [list(map(float, s)) for s in stops]


def robust_pool(stops, mad_k=3.0):
    """剔除任一分量偏离中位数超过 mad_k 倍 MAD 的停顿，返回 (保留数组, 剔除个数)。"""
    S = np.asarray(stops, dtype=float)
    med = np.median(S, axis=0)
    mad = 1.4826 * np.median(np.abs(S - med), axis=0)
    keep = (np.abs(S - med) / np.maximum(mad, 1e-6) < mad_k).all(1)
    return S[keep], int((~keep).sum())


def analyze_room(capture, prior_stops=()):
    fails, info = [], {}
    track = wheel_track(capture)
    basic_checks(capture, track['duration'], fails, info)
    if fails:
        return fails, info
    info['still_segments'] = len(track['segments'])
    stops = per_stop_registration(capture)
    info['stops_registered'] = len(stops)
    info['per_stop'] = [dict(zip(NAMES6, np.round(s, 4).tolist())) for s in stops]
    if len(stops) < 4:
        fails.append('只有 %d 处停顿配准成功（要求 ≥4）：多换几个位置，每处停稳 4 秒以上，'
                     '让两台雷达共同看到桌椅、柜子等立体物' % len(stops))
        return fails, info
    current, _ = robust_pool(stops)
    info['median_this_capture'] = dict(zip(NAMES6, np.round(np.median(current, axis=0), 4).tolist()))
    pooled = list(stops) + [list(p) for p in prior_stops]
    kept, dropped = robust_pool(pooled)
    med = np.median(kept, axis=0)
    se = 1.2533 * kept.std(axis=0, ddof=1) / np.sqrt(len(kept))
    info.update(pooled_stops=len(pooled), outliers_removed=dropped,
                median=dict(zip(NAMES6, np.round(med, 4).tolist())),
                median_std_error=dict(zip(NAMES6, np.round(se, 4).tolist())),
                se_translation_m=round(float(se[:3].max()), 4), se_rotation_deg=round(float(se[3:].max()), 3))
    if prior_stops:
        prior_kept, _ = robust_pool(prior_stops) if len(prior_stops) >= 4 else (np.asarray(prior_stops), 0)
        diff = np.abs(np.median(current, axis=0) - np.median(prior_kept, axis=0))
        info['diff_vs_prior'] = dict(zip(NAMES6, np.round(diff, 4).tolist()))
        if diff[:3].max() > 0.04 or diff[3:].max() > 1.5:
            fails.append('本次与之前房间录制的中位数相差 %.1f cm / %.2f°（要求 ≤4 cm/1.5°）'
                         % (100 * diff[:3].max(), diff[3:].max()))
    if len(kept) < 6 or info['se_translation_m'] > 0.015 or info['se_rotation_deg'] > 0.7:
        fails.append('多处平均后仍不够稳定（剔除离群后 %d 处，中位数标准误 平移 %.1f cm、角度 %.2f°，'
                     '要求 ≥6 处且 ≤1.5 cm/0.7°）：再录一次，换一个立体物更多的房间'
                     % (len(kept), 100 * info['se_translation_m'], info['se_rotation_deg']))
    return fails, info
