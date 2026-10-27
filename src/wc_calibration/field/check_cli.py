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


def analyze(ctx, kind, capture_dir, dist=None):
    capture_dir = Path(capture_dir)
    say('\n  开始分析 %s …' % capture_dir.name)
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
    'room': ['推到有桌椅、柜子等立体物的房间或大厅（不要在长走廊）',
             '看到"开始录制"后先静止 3 秒',
             '慢慢推到 5 个以上不同位置，每处换一个朝向，停稳 4 秒再走',
             '中间包含几次原地左转、右转；最后停住等录制结束'],
}


def record(ctx, label, duration, steps):
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
               '--profile', 'mapping_core', '--sides', 'all', '--session', session,
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
    a = sub.add_parser('analyze')
    a.add_argument('kind', choices=['wall', 'approach', 'room'])
    a.add_argument('capture')
    a.add_argument('--dist', type=float, default=3.0)
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
        return 0 if analyze(ctx, args.kind, args.capture, args.dist) == 'PASS' else 1
    steps = [s.format(dist=getattr(args, 'dist', '')) for s in STEPS[args.cmd]]
    label = 'wall_%.1f' % args.dist if args.cmd == 'wall' else args.cmd
    found = record(ctx, label, args.duration, steps)
    if found is None:
        return 1
    return 0 if analyze(ctx, args.cmd, found, getattr(args, 'dist', None)) == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
