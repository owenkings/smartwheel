"""现场外参检查：录一项、马上分析、当场显示合格/不合格和原因。

  scripts/lidar_field_check check                 只检查设备，不录制
  scripts/lidar_field_check wall --dist 3.0       正对墙静止（也录 4.5）
  scripts/lidar_field_check approach              正对墙从约 4.5 m 推到约 1.5–2 m（录两次）
  scripts/lidar_field_check room                  房间里多处停顿（可多次，结果合并）
  scripts/lidar_field_check summary               当前进度
  scripts/lidar_field_check analyze KIND 录包目录  只分析已有录包

录制调用 wc_phase1 capture（mapping_core，只读，不驱动电机）；结果写入存储策略的
data/calibration/field_check/。不改正式配置。
"""
import argparse
import datetime
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

from . import checks
from .capture import load_capture

GREEN, RED, YELLOW, END = '\033[1;32m', '\033[1;31m', '\033[1;33m', '\033[0m'
NEEDED = {'wall_3.0': '正对墙 3.0 m', 'wall_4.5': '正对墙 4.5 m',
          'approach': '推向墙测距（两次一致）', 'room': '房间多处停顿'}
CAMERA_NEEDED = {'camera_intrinsics_left_front': '左前相机内参', 'camera_intrinsics_right_front': '右前相机内参',
                 'camera_board_left_front': '左前相机→左雷达外参', 'camera_board_right_front': '右前相机→右雷达外参'}


def say(message, color=''):
    print(color + message + (END if color else ''), flush=True)


class Context:
    def __init__(self, project_root):
        from wc_runtime.storage_policy import StoragePolicy
        self.project = Path(project_root).resolve()
        self.policy = StoragePolicy(self.project)
        self.policy.check()
        self.experiments = Path(self.policy.resolve('data/experiments'))
        self.out = Path(os.environ.get('FIELD_CHECK_OUT') or self.policy.resolve('data/calibration/field_check'))
        self.out.mkdir(parents=True, exist_ok=True)

    def records(self, key=None, passed=None):
        rows = []
        for path in sorted(self.out.glob('*.json')):
            row = json.loads(path.read_text(encoding='utf-8'))
            if (key is None or row['key'] == key) and (passed is None or (row['status'] == 'PASS') == passed):
                rows.append(row)
        return rows


# ------------------------------------------------------------------ 说明
def explain(kind, info):
    if kind == 'wall':
        for side, name in checks.SIDE_NAMES.items():
            r = info.get(side, {})
            if 'wall_floor_angle_deg' in r:
                say('  %s雷达：墙距 %.2f m，前方 2.5 m 处地面在下方 %.2f m，墙与地面夹角 %.2f°（地面偏斜 %+.2f°）；分段地面俯仰 %s'
                    % (name, r['wall_distance_m'], r['floor_height_at_2p5m_m'], r['wall_floor_angle_deg'],
                       r['floor_bias_deg'], r['floor_pitch_by_range_deg']))
    elif kind == 'approach':
        if 'wheel_distance_m' not in info:
            say('  检测到的静止段（秒）：%s' % info.get('stops_s'))
            return
        for side, name in checks.SIDE_NAMES.items():
            r = info.get(side)
            if r:
                say('  %s雷达：墙距 %.3f → %.3f m，实际前进 %.3f m' % (name, r['wall_start_m'], r['wall_end_m'],
                                                             r['lidar_distance_m']))
        say('  轮速积分 %.3f m（转角 %+.1f°）' % (abs(info['wheel_distance_m']), info['wheel_yaw_deg']))
        if 'wheel_scale_factor' in info:
            say('  轮速距离误差 %+.1f%% → 轮速比例应乘 %.4f' % (info['wheel_scale_error_pct'], info['wheel_scale_factor']),
                YELLOW)
    elif kind == 'camera_intrinsics' and 'rms_px' in info:
        say('  %s：合并 %d 次录制，%d 个不同视角，覆盖 %d/9 区域，最大倾角 %.0f°，重投影误差 %.2f 像素' % (
            info['role'], 1 + len(info.get('pooled_captures', [])), info['distinct_views'],
            round(9 * info['coverage_3x3']), info['max_tilt_deg'], info['rms_px']))
        if info.get('missing_cells'):
            say('  还没覆盖：' + '、'.join(info['missing_cells']), YELLOW)
        say('  fx %.1f  fy %.1f  cx %.1f  cy %.1f  畸变 %s' % (info['fx'], info['fy'], info['cx'], info['cy'], info['dist']))
    elif kind == 'camera_board':
        for row in info.get('poses', []):
            say('  姿态 %d：静止 %.1f s，距离 %.2f m，倾角 %.0f°，%s%s' % (
                row['pose'], row['duration_s'], row['distance_m'], row['tilt_deg'], row['status'],
                '（雷达板面 %d 点）' % row['lidar_points'] if 'lidar_points' in row else ''))
        if 'summary' in info:
            s = info['summary']
            say('  相机在%s雷达坐标中：x %.3f y %.3f z %.3f m，横滚 %.2f° 俯仰 %.2f° 偏航 %.2f°' % (
                '左' if info['lidar'] == 'left' else '右', s['tx_m'], s['ty_m'], s['tz_m'],
                s['roll_deg'], s['pitch_deg'], s['yaw_deg']))
            say('  用了 %d 个姿态（剔除离群 %d 个，合并此前 %d 个）；旋转不确定度 %.2f°' % (
                info['poses_used'], info['poses_dropped'], info.get('pooled_pairs', 0), info['rotation_std_deg']))
            say('  姿态间残差：法向中位 %.2f°、最大 %.2f°，距离 %.1f cm；离机械名义值 %.0f cm / %.1f°' % (
                info['normal_residual_deg_median'], info['normal_residual_deg_max'], 100 * info['distance_residual_m_max'],
                100 * info['translation_from_nominal_m'], info['rotation_from_nominal_deg']))
    elif kind == 'room' and 'median' in info:
        m = info['median']
        say('  合并 %d 处停顿（剔除离群 %d 处）：右雷达在左雷达坐标中 x %.3f m, y %.3f m, z %.3f m；'
            '横滚 %.2f° 俯仰 %.2f° 偏航 %.2f°' % (info['pooled_stops'], info['outliers_removed'], m['tx_m'],
                                             m['ty_m'], m['tz_m'], m['roll_deg'], m['pitch_deg'], m['yaw_deg']))
        say('  中位数标准误：平移 %.1f cm，角度 %.2f°' % (100 * info['se_translation_m'], info['se_rotation_deg']))


def summary(ctx):
    got = {}
    for row in ctx.records(passed=True):
        if row['key'] not in ('approach',):
            got[row['key']] = row
    scales = [r['info']['wheel_scale_factor'] for r in ctx.records('approach', True) if 'wheel_scale_factor' in r['info']]
    confirmed, pair = checks.approach_agreement(scales)
    if confirmed:
        got['approach'] = pair
    elif scales:
        say('\n  推向墙测距已有 %d 次单次合格（%s），需要再录一次，与其中某次相差 ≤3%% 才算确认。'
            % (len(scales), '、'.join('%.3f' % s for s in scales)), YELLOW)
    say('\n  ---- 现场进度 ----')
    for key, name in NEEDED.items():
        say('  %s %s' % ('✓' if key in got else '·', name), GREEN if key in got else '')
    missing = [name for key, name in NEEDED.items() if key not in got]
    say('  还缺：' + '、'.join(missing) if missing else '  全部合格，数据已录齐。', YELLOW if missing else GREEN)
    say('  ---- 相机 ----')
    for key, name in CAMERA_NEEDED.items():
        say('  %s %s' % ('✓' if key in got else '·', name), GREEN if key in got else '')


def report(ctx, kind, key, capture_dir, fails, info):
    status = 'PASS' if not fails else 'FAIL'
    row = dict(kind=kind, key=key, status=status, capture=str(capture_dir),
               at=datetime.datetime.now().isoformat(timespec='seconds'), reasons=fails, info=info)
    (ctx.out / ('%s_%s.json' % (key, Path(capture_dir).name))).write_text(
        json.dumps(row, indent=1, ensure_ascii=False), encoding='utf-8')
    print()
    if fails:
        say('  ✗ 不合格', RED)
        for reason in fails:
            say('    - ' + reason, RED)
        say('  请按上面原因调整后重新运行同一条命令。')
    else:
        say('  ✓ 合格', GREEN)
    explain(kind, info)
    summary(ctx)
    return status


def analyze(ctx, kind, capture_dir, dist=None, role=None):
    capture_dir = Path(capture_dir)
    say('\n  开始分析 %s …' % capture_dir.name)
    if kind == 'camera_intrinsics':
        from .camera import analyze_intrinsics
        # 只合并清晰的此前录制（重投影误差 ≤0.5 像素），模糊的录制会把畸变参数带偏
        earlier = [r['capture'] for r in ctx.records('camera_intrinsics_' + role)
                   if Path(r['capture']).name != capture_dir.name
                   and r['info'].get('detected_this_capture', r['info'].get('detected_frames', 0))
                   and (r['info'].get('rms_px') or 9) <= 0.5]
        say('  正在检测棋盘格并标定内参（合并此前 %d 次录制，约 1–3 分钟）…' % len(earlier))
        fails, info = analyze_intrinsics(capture_dir, role, earlier_captures=earlier)
        return report(ctx, kind, 'camera_intrinsics_' + role, capture_dir, fails, info)
    if kind == 'camera_board':
        from .camera import analyze_board_extrinsic
        intrinsics = [r for r in ctx.records('camera_intrinsics_' + role, True)]
        if not intrinsics:
            say('  还没有合格的 %s 内参，先做 camera-intrinsics --role %s' % (role, role), RED)
            return 'FAIL'
        say('  使用内参：%s' % Path(intrinsics[-1]['capture']).name)
        # 合并此前同一相机、同一内参、板够近（方格 ≥10 像素）的录制
        earlier = [p for r in ctx.records('camera_board_' + role)
                   if Path(r['capture']).name != capture_dir.name
                   and r['info'].get('intrinsics_capture') == intrinsics[-1]['capture']
                   and (r['info'].get('median_square_px') or 0) >= 10
                   for p in r['info'].get('plane_pairs', [])]
        if earlier:
            say('  合并此前录制的 %d 个合格姿态' % len(earlier))
        fails, info = analyze_board_extrinsic(load_capture(capture_dir), capture_dir, role, intrinsics[-1]['info'],
                                              earlier_pairs=earlier)
        info['intrinsics_capture'] = intrinsics[-1]['capture']
        return report(ctx, kind, 'camera_board_' + role, capture_dir, fails, info)
    capture = load_capture(capture_dir)
    if kind == 'wall':
        fails, info = checks.analyze_wall(capture, dist)
        return report(ctx, kind, 'wall_%.1f' % dist, capture_dir, fails, info)
    if kind == 'approach':
        fails, info = checks.analyze_approach(capture)
        return report(ctx, kind, 'approach', capture_dir, fails, info)
    prior = [[s[k] for k in checks.NAMES6] for r in ctx.records('room')
             if Path(r['capture']).name != capture_dir.name for s in r['info'].get('per_stop', [])]
    say('  正在逐处做左右配准（约 1–2 分钟）…')
    fails, info = checks.analyze_room(capture, prior)
    return report(ctx, kind, 'room', capture_dir, fails, info)


# ------------------------------------------------------------------ 录制
STEPS = {
    'wall': ['轮椅正对一面平整的空墙，车头垂直于墙',
             '雷达前脸到墙约 {dist} 米，雷达和墙之间的地面不要放东西',
             '人站到轮椅后面，录制期间所有人保持不动'],
    'approach': ['轮椅正对一面平整的空墙，雷达前脸离墙约 4.5 米，墙前不要有人或杂物',
                 '看到"开始录制"后先静止 5 秒',
                 '正对墙匀速直线推，推到离墙约 1.5–2 米，停稳到录制结束（至少 5 秒）'],
    'camera_intrinsics': ['轮椅不动。手拿 A4 棋盘格站在 {role} 相机前 0.3–0.8 米，整张纸都要在画面里',
                          '可以分几次录：每次结果会和之前同一相机的录制合并，提示缺哪些区域就重点补哪些',
                          '看到"开始录制"后慢慢移动：画面四角、中间各停 1–2 秒',
                          '再做前后倾、左右倾约 30–45°，远近都拍到；动作慢，避免拖影和反光'],
    'camera_board': ['棋盘格平整贴在 50 cm 木板上；轮椅不动',
                     '木板放在 {role} 相机正前方 0.8–1.0 米（太远相机看不清方格），高度与相机相近，整块板在画面里',
                     '看到"开始录制"后摆第 1 个姿态（正对），完全静止 6 秒，手不要挡住棋盘格',
                     '依次换姿态并各静止 6 秒：左转约 35°、右转约 35°、上沿后倾约 35°、上沿前倾约 35°、斜着（左上转）约 30°',
                     '可以分几次录，结果会自动合并'],
    'room': ['推到有桌椅、柜子等立体物的房间或大厅（不要在长走廊）',
             '看到"开始录制"后先静止 3 秒',
             '慢慢推到 5 个以上不同位置，每处换一个朝向，停稳 4 秒再走',
             '中间包含几次原地左转、右转；最后停住等录制结束'],
}


# 640×480、10 帧/秒：标定够用，四路相机同时归档时 CPU 和 USB 负担小
CAMERA_CAPTURE_ARGS = ['--profile', 'mapping_cameras', '--camera-profile', 'detail_640_10fps',
                       '--cameras', 'left_front,right_front']  # 只录前方两路，减轻 USB 负担


def capture_supports_camera_profile(ctx):
    out = subprocess.run([sys.executable, '-s', str(ctx.project / 'scripts' / 'wc_phase1'), 'capture', '--help'],
                         cwd=str(ctx.project), capture_output=True, text=True).stdout
    return '--camera-profile' in out


def record(ctx, label, duration, steps, profile_args=('--profile', 'mapping_core')):
    session = 'field_%s_%s' % (label, datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    say('\n================ %s（录制 %d 秒）================' % (label, duration), YELLOW)
    for k, step in enumerate(steps, 1):
        say('  %d. %s' % (k, step))
    say('  录制期间人不要站在轮椅前方的雷达视野里（站在轮椅后面推）。')
    try:
        input('\n  准备好后按回车开始（Ctrl+C 取消）… ')
    except KeyboardInterrupt:
        say('\n  已取消，未录制。')
        return None
    command = [sys.executable, '-u', '-s', str(ctx.project / 'scripts' / 'wc_phase1'), 'capture',
               *profile_args, '--sides', 'all', '--session', session,
               '--duration', str(duration), '--staging', 'memory']
    say('  正在启动传感器，等待"开始录制"提示后再动作…')
    process = subprocess.Popen(command, cwd=str(ctx.project), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, bufsize=1)
    lines, stop = [], threading.Event()
    progress = Path('/dev/shm/wc_capture_%d' % os.getuid()) / session / 'progress.json'

    def watch():
        started = None
        while not stop.is_set():
            try:
                stage = json.loads(progress.read_text()).get('stage')
            except (OSError, ValueError):
                stage = None
            if stage == 'RECORDING' and started is None:
                started = time.time()
                say('\n  ▶▶▶ 开始录制！现在按步骤动作（共 %d 秒）' % duration, GREEN)
            if started is not None:
                left = duration - (time.time() - started)
                if left <= 0:
                    say('  ■ 录制时间到，请保持不动，等待保存和校验…', GREEN)
                    return
                if int(left) % 10 == 0:
                    say('    还剩 %d 秒' % int(left))
                    time.sleep(1)
            time.sleep(0.2)

    threading.Thread(target=watch, daemon=True).start()
    try:
        for line in process.stdout:
            lines.append(line)
            if any(word in line for word in ('ERROR', 'Error', 'FAIL', 'PARTIAL', 'COMPLETE')):
                say('   │ ' + line.rstrip()[:240])
        process.wait()
    except KeyboardInterrupt:
        say('\n  收到 Ctrl+C，正在等待录制程序正常收尾保存，不要再按…', YELLOW)
        try:
            process.wait()
        except KeyboardInterrupt:
            process.send_signal(signal.SIGINT)
            process.wait()
    stop.set()
    logs = ctx.out / 'logs'
    logs.mkdir(exist_ok=True)
    (logs / (session + '.log')).write_text(''.join(lines), encoding='utf-8')
    found = next((p for p in [ctx.experiments / session, *sorted(ctx.experiments.glob(session + '*'))]
                  if (p / 'capture_manifest.json').is_file()), None)
    if process.returncode != 0 or found is None:
        say('  录制程序退出码 %s，%s。日志：%s' % (process.returncode, '录包 ' + str(found) if found else '没找到录包',
                                          logs / (session + '.log')), RED)
        if found is None:
            return None
    say('  录包：%s' % found)
    return found


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--project-root', default=os.environ.get('WHEELCHAIR_PROJECT_ROOT', '.'))
    sub = parser.add_subparsers(dest='cmd', required=True)
    sub.add_parser('check')
    sub.add_parser('summary')
    wall = sub.add_parser('wall')
    wall.add_argument('--dist', type=float, required=True, choices=[3.0, 4.5])
    wall.add_argument('--duration', type=int, default=20)
    sub.add_parser('approach').add_argument('--duration', type=int, default=40)
    sub.add_parser('room').add_argument('--duration', type=int, default=180)
    for name, seconds in (('camera-intrinsics', 60), ('camera-board', 60)):
        c = sub.add_parser(name)
        c.add_argument('--role', required=True, choices=['left_front', 'right_front'])
        c.add_argument('--duration', type=int, default=seconds)
    a = sub.add_parser('analyze')
    a.add_argument('kind', choices=['wall', 'approach', 'room', 'camera_intrinsics', 'camera_board'])
    a.add_argument('capture')
    a.add_argument('--dist', type=float, default=3.0)
    a.add_argument('--role', choices=['left_front', 'right_front'])
    args = parser.parse_args(argv)
    ctx = Context(args.project_root)
    if args.cmd == 'summary':
        summary(ctx)
        return 0
    if args.cmd == 'check':
        command = [sys.executable, '-u', '-s', str(ctx.project / 'scripts' / 'wc_phase1'), 'capture', '--profile',
                   'mapping_core', '--sides', 'all', '--session', 'field_check_dryrun', '--duration', '20', '--dry-run']
        code = subprocess.call(command, cwd=str(ctx.project))
        say('设备检查%s（退出码 %d）' % ('通过' if code == 0 else '未通过', code), GREEN if code == 0 else RED)
        return code
    if args.cmd == 'analyze':
        if args.kind.startswith('camera') and not args.role:
            parser.error('相机分析需要 --role')
        return 0 if analyze(ctx, args.kind, args.capture, args.dist, args.role) == 'PASS' else 1
    if args.cmd.startswith('camera-'):
        kind = args.cmd.replace('-', '_')
        if not capture_supports_camera_profile(ctx):
            say('当前采集程序不支持 --camera-profile（相机固定 320×240），先完成采集程序改动再录相机。', RED)
            return 2
        steps = [s.format(role=args.role) for s in STEPS[kind]]
        found = record(ctx, '%s_%s' % (kind, args.role), args.duration, steps, CAMERA_CAPTURE_ARGS)
        if found is None:
            return 1
        return 0 if analyze(ctx, kind, found, role=args.role) == 'PASS' else 1
    steps = [s.format(dist=getattr(args, 'dist', '')) for s in STEPS[args.cmd]]
    label = 'wall_%.1f' % args.dist if args.cmd == 'wall' else args.cmd
    found = record(ctx, label, args.duration, steps)
    if found is None:
        return 1
    return 0 if analyze(ctx, args.cmd, found, getattr(args, 'dist', None)) == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
