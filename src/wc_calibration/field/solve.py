"""离线联合求解现场外参候选，并用独立录包验证。只写候选文件，不改正式配置。

输入：
  --room      房间多处停顿录包（可重复）：左右相对外参 T_left_right
  --level     用于"墙面竖直"定各雷达倾角的录包（可重复，默认等于 --room，可加走廊）
  --motion    用于手眼求左雷达偏航的行驶录包（需有转弯，可重复，默认等于 --room）
  --validate  独立验证录包（不参与求解），比较机械名义值与候选的左右重叠残差
  --approach-results  推向墙检查结果 JSON（field_check 输出），给出轮速比例
输出：存储策略 data/calibration/field_solve/<时间>/candidate.json
"""
import argparse
import datetime
import json
import os
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .capture import load_capture, still_pairs, wheel_track
from .checks import MECHANICAL_T_LEFT_RIGHT, approach_agreement
from .geometry import (CloudPair, describe, level_rotation, overlap_residual_mm, roll_pitch_deg,
                       solve_left_right_4dof, up_from_wall_normals, vertical_plane_normals)
from .motion import lidar_odometry, planar_hand_eye


def wall_normals(clouds, step=4):
    normals = []
    images = clouds.reshape(-1, 60, 160, 3)
    for k in range(0, len(images), step):
        p = images[k, 4:44].reshape(-1, 3)
        p = p[np.isfinite(p).all(1)]
        r = np.linalg.norm(p, axis=1)
        normals += vertical_plane_normals(p[(r > 0.8) & (r < 6)], seed=k)
    return normals


def pairs_from(capture):
    L, R = capture['lidar']['left']['clouds'], capture['lidar']['right']['clouds']
    out = []
    for i, j, segment in still_pairs(capture):
        try:
            out.append(CloudPair(L[i], R[j], '%s#%d' % (Path(capture['dir']).name, segment)))
        except ValueError:
            pass
    return out


def solve(rooms, levels, motions, validations, approach_results, log=print):
    cache = {}

    def get(path):
        if path not in cache:
            log('读取 ' + path)
            cache[path] = load_capture(path)
        return cache[path]

    result = dict(schema='wc_field_extrinsic_candidate_v1', status='OFFLINE_CANDIDATE', live_eligible=False,
                  convention='p_left = R @ p_right + t；米/度；雷达原生 FLU；T_axle_lidar 为雷达点 -> 轮轴系',
                  inputs=dict(room=rooms, level=levels, motion=motions, validate=validations,
                              approach_results=approach_results))
    # 1) 各雷达竖直方向（墙面竖直）
    ups, per_capture = {}, {}
    for side in ('left', 'right'):
        normals = []
        for path in levels:
            n = wall_normals(get(path)['lidar'][side]['clouds'])
            u, _ = up_from_wall_normals(n)
            per_capture['%s/%s' % (Path(path).name, side)] = [round(v, 2) for v in roll_pitch_deg(u)]
            normals += n
        ups[side], uncertainty = up_from_wall_normals(normals)
        result.setdefault('level', {})[side] = dict(roll_pitch_deg=[round(v, 3) for v in roll_pitch_deg(ups[side])],
                                                    unc68_deg=round(uncertainty, 3), wall_planes=len(normals))
    result['level']['per_capture'] = per_capture
    # 2) 左右相对外参（固定倾角，4 自由度）
    room_pairs = {path: pairs_from(get(path)) for path in rooms}
    all_pairs = [p for v in room_pairs.values() for p in v]
    if len(all_pairs) < 6:
        raise ValueError('房间停顿配对太少（%d）' % len(all_pairs))
    T_lr = solve_left_right_4dof(all_pairs, ups['left'], ups['right'], np.radians(-4.5), [0.08, -0.46, 0.0])
    result['T_left_right'] = describe(T_lr)
    result['T_left_right_matrix'] = T_lr.tolist()
    if len(rooms) >= 2:
        singles = [solve_left_right_4dof(room_pairs[p], ups['left'], ups['right'], np.radians(-4.5),
                                         [0.08, -0.46, 0.0]) for p in rooms]
        d = np.linalg.inv(singles[0]) @ singles[1]
        result['leave_one_out_first_two'] = dict(
            translation_cm=round(100 * float(np.linalg.norm(d[:3, 3])), 2),
            rotation_deg=round(float(np.degrees(Rotation.from_matrix(d[:3, :3]).magnitude())), 3))
    # 3) 独立验证：重叠残差
    result['validation_overlap_residual_mm'] = {}
    for path in validations:
        pairs = pairs_from(get(path))
        result['validation_overlap_residual_mm'][Path(path).name] = dict(
            mechanical=overlap_residual_mm(pairs, MECHANICAL_T_LEFT_RIGHT), candidate=overlap_residual_mm(pairs, T_lr),
            pairs=len(pairs))
    # 4) 左雷达相对车体偏航（平面手眼，用各录包的轮速轨迹）
    yaws = {}
    for path in motions:
        capture = get(path)
        log('ICP 里程计 ' + Path(path).name)
        odometry = lidar_odometry(capture)
        track = wheel_track(capture)
        times = (capture['lidar']['left']['mono'] - track['start_ns']) * 1e-9
        try:
            yaws[Path(path).name] = planar_hand_eye(odometry['left'], times, track, ups['left'])
        except ValueError as error:
            yaws[Path(path).name] = dict(error=str(error))
    result['left_hand_eye'] = yaws
    good = [v for v in yaws.values() if 'yaw_deg' in v and v['std']['yaw_deg'] < 1.5]
    if not good:
        raise ValueError('没有可用的手眼结果来定左雷达偏航')
    weights = np.array([1 / max(v['std']['yaw_deg'], 0.05) ** 2 for v in good])
    yaw_left = float(np.sum(weights * [v['yaw_deg'] for v in good]) / weights.sum())
    # 5) 轮速比例
    scales = []
    for path in approach_results:
        row = json.loads(Path(path).read_text(encoding='utf-8'))
        value = row.get('info', {}).get('wheel_scale_factor', row.get('info', {}).get('wheel_scale_factor_suggested'))
        if row.get('status') == 'PASS' and value is not None:  # 后者为早期现场工具的字段名
            scales.append(value)
    confirmed, _ = approach_agreement(scales)
    result['wheel'] = dict(scale_factors=scales, confirmed=confirmed,
                           scale_mean=round(float(np.mean(scales)), 4) if scales else None)
    # 6) 组合成轮轴系外参：左雷达位置沿用机械值（数据不能定），姿态 = Rz(偏航)·找平
    R_left = Rotation.from_euler('z', np.radians(yaw_left)).as_matrix() @ level_rotation(ups['left'])
    result['T_M_lidar_left_rotation'] = R_left.tolist()
    result['left_yaw_in_base_deg'] = round(yaw_left, 3)
    right_yaw = float(np.degrees(np.arctan2(*(R_left @ T_lr[:3, :3])[[1, 0], 0])))
    result['right_yaw_in_base_deg'] = round(right_yaw, 3)
    result['notes'] = ['左雷达位置沿用 CAD 外壳前面中心；右雷达 = 左雷达 · T_left_right。',
                       '倾角受 ToF 测距畸变影响，录包之间散布约 1°；偏航最稳。']
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--project-root', default=os.environ.get('WHEELCHAIR_PROJECT_ROOT', '.'))
    parser.add_argument('--room', action='append', required=True)
    parser.add_argument('--level', action='append')
    parser.add_argument('--motion', action='append')
    parser.add_argument('--validate', action='append', default=[])
    parser.add_argument('--approach-results', action='append', default=[])
    parser.add_argument('--output')
    args = parser.parse_args(argv)
    from wc_runtime.storage_policy import StoragePolicy
    policy = StoragePolicy(Path(args.project_root).resolve())
    policy.check()
    result = solve(args.room, args.level or args.room, args.motion or args.room, args.validate, args.approach_results)
    out = Path(args.output) if args.output else Path(policy.resolve('data/calibration/field_solve')) / \
        datetime.datetime.now().strftime('%Y%m%dT%H%M%S')
    out.mkdir(parents=True, exist_ok=False)
    (out / 'candidate.json').write_text(json.dumps(result, indent=1, ensure_ascii=False), encoding='utf-8')
    print(json.dumps({k: result[k] for k in ('T_left_right', 'level', 'left_yaw_in_base_deg', 'right_yaw_in_base_deg',
                                              'wheel', 'validation_overlap_residual_mm')}, indent=1, ensure_ascii=False))
    print('候选已写入：' + str(out / 'candidate.json'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
