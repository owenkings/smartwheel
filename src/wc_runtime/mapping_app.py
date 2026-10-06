"""Foreground preview or mapping: owned stop precedes every save decision.

The mapping_controller backend implements preflight(request), start(request),
inspect(handle), stop(handle), save(handle, destination), discard(handle),
export(handle), and rviz_command(handle). Request
paths are strings. inspect/stop return state, exit_code, cleanup_errors. start
must own an exclusive new output directory and clean up partial startup before
raising. Its supervisor must have a bounded lifetime / parent-death policy.
No fallback mode, sensor substitution, calibration guess, or motor command is
implemented here. Backend preflight must validate the actual requested chain.
"""
import argparse
import importlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
import uuid

from .mapping_cloud import CLOUD_SOURCES, validate_cloud_source, cloud_description
from .storage_policy import StoragePolicy, resolve_storage_path


BACKEND_MODULE = 'wc_runtime.mapping_controller'
MODES = ('left', 'right', 'all')
BACKEND_METHODS = ('preflight', 'start', 'inspect', 'stop', 'export', 'save', 'discard', 'rviz_command')


def session_name(value):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}', value):
        raise argparse.ArgumentTypeError('名称须为 1–64 个 ASCII 字母、数字、下划线、点或横线，并以字母或数字开头')
    return value


def checked_path(value):
    path = Path(value).absolute()
    if '..' in path.parts:
        raise ValueError('输出路径不能包含 ..')
    for current in (path, *path.parents):
        if current.is_symlink() or (hasattr(current, 'is_junction') and current.is_junction()):
            raise ValueError('项目和输出路径不能经过符号链接或目录联接: ' + str(current))
    return path


def prepare_request(mode, output=None, name=None, *, project_root, config=None, duration_s=0,
                    mapping_enabled=False, cloud_source=None, retention_profile='experiment', now=None):
    """Resolve a new session without creating files or starting any process."""
    if mode not in MODES:
        raise ValueError('模式必须是 left、right 或 all')
    if type(duration_s) is not int or duration_s < 0:
        raise ValueError('--duration 必须是非负整数秒，0 表示持续运行至停止')
    if type(mapping_enabled) is not bool:
        raise ValueError('mapping_enabled 必须为明确的布尔值')
    if cloud_source is not None:
        validate_cloud_source(cloud_source)
    if retention_profile not in ('experiment', 'map_only'):
        raise ValueError('retention_profile must be experiment or map_only')
    root = checked_path(project_root)
    if not root.is_dir():
        raise ValueError('工程目录不存在: ' + str(root))
    instant = now or datetime.now(timezone.utc)
    name = session_name(name or ('map_' + mode + '_' + instant.strftime('%Y%m%dT%H%M%SZ') + '_' + uuid.uuid4().hex[:6]))
    chosen = Path(output) if output is not None else Path('reports/maps') / name
    storage = StoragePolicy(root)
    original = checked_path(chosen if chosen.is_absolute() else root/chosen)
    try:
        directory = checked_path(storage.resolve(original))
    except ValueError as error:
        if 'linked' in str(error) or 'symlink' in str(error):
            raise ValueError('输出路径不能经过符号链接或目录联接: '+str(error)) from error
        raise ValueError('输出路径必须在工程目录内或已配置的 U 盘中: '+str(error)) from error
    location_root = storage.archive_root if storage.enabled else root
    if not directory.is_relative_to(location_root) or directory == location_root:
        raise ValueError('输出必须是工程目录内的新会话目录')
    relative = directory.relative_to(location_root)
    if relative.parts[0] in ('.git', '.codex', '.agents', '.phase1_runtime'):
        raise ValueError('不能把地图输出写入工程元数据或进程管理目录')
    if directory.exists():
        raise ValueError('输出目录已存在；使用新名称或新路径，原有数据不会覆盖: ' + str(directory))
    if any(parent.exists() and not parent.is_dir() for parent in directory.parents if parent.is_relative_to(location_root)):
        raise ValueError('输出路径的父级不是目录')
    chosen_config = Path(config) if config is not None else Path('config/mapping_live.json')
    config_path = checked_path(chosen_config if chosen_config.is_absolute() else root / chosen_config)
    if not config_path.is_relative_to(root):
        raise ValueError('建图配置必须在工程目录内')
    return {'session_id': name, 'mode': mode, 'output_dir': str(directory),
            'project_root': str(root), 'config_path': str(config_path), 'owner_pid': os.getpid(),
            'duration_s': 0, 'requested_duration_s': duration_s, 'mapping_enabled': mapping_enabled,
            'retention_profile': retention_profile,
            **({'cloud_source': cloud_source} if cloud_source is not None else {})}


def parse_request(argv=None, *, project_root):
    parser = argparse.ArgumentParser(description='默认纯实时预览；--mapping true 启用建图，停止后再选择是否保存。')
    parser.add_argument('positional_mode', nargs='?', choices=MODES, metavar='left|right|all')
    parser.add_argument('--mode', choices=MODES, help='选择左雷达、右雷达或双雷达试验建图')
    parser.add_argument('--output', type=Path, help='工程内新的临时工作会话目录；最终保存路径在停止后询问')
    parser.add_argument('--mapping', choices=('true', 'false'), default='false',
                        help='true 启用建图并在停止后询问保存；false 仅实时预览（默认）')
    parser.add_argument('--cloud', choices=CLOUD_SOURCES,
                        help='建图和实时预览使用的点云；默认配置值 filtered。raw 是主机滤波前 SDK XYZ，不是原始光学深度')
    parser.add_argument('--name', type=session_name, help='新的会话名称；省略时生成唯一名称')
    parser.add_argument('--retention-profile', choices=('experiment','map_only'), default='experiment',
                        help='experiment 保存原始档案；map_only 明确清理原始数据后不可重放')
    parser.add_argument('--config', type=Path, help='工程内建图配置，默认 config/mapping_live.json')
    parser.add_argument('--check-config', action='store_true', help='只检查安装与轮参数，不打开RViz或任何传感器')
    parser.add_argument('--duration', type=int, default=0, help='兼容旧参数；建图已取消总时长限制，此值不再触发计时结束')
    args = parser.parse_args(argv)
    if args.mode and args.positional_mode and args.mode != args.positional_mode:
        parser.error('位置参数和 --mode 指定了不同模式')
    mode = args.mode or args.positional_mode
    if mode is None:
        parser.error('请指定 --mode left|right|all，或使用位置参数，例如 python3 scripts/map right')
    request = prepare_request(mode, args.output, args.name, project_root=project_root, config=args.config,
                              duration_s=args.duration, mapping_enabled=args.mapping == 'true', cloud_source=args.cloud,
                              retention_profile=args.retention_profile)
    request['check_config_only'] = args.check_config
    return request


def load_backend():
    try:
        backend = importlib.import_module(BACKEND_MODULE)
    except ImportError as error:
        raise RuntimeError('建图后端未就绪，未启动采集: ' + str(error)) from error
    missing = [name for name in BACKEND_METHODS if not callable(getattr(backend, name, None))]
    if missing:
        raise RuntimeError('建图后端接口不完整，未启动采集: ' + ', '.join(missing))
    return backend


def launch_rviz(command, request):
    """Popen owns only this new GUI wrapper; parent death closes descendants."""
    if not isinstance(command, list) or not command or any(not isinstance(value, str) or '\0' in value for value in command):
        raise ValueError('RViz 启动命令必须是非空 argv 列表')
    output = checked_path(request['output_dir'])
    if not output.is_dir():
        raise RuntimeError('后端未创建当前会话目录')
    attempt = request.get('rviz_startup_attempt', 1)
    if type(attempt) is not int or attempt not in (1, 2):
        raise ValueError('RViz 启动次数只允许 1 或 2')
    wrapper = [sys.executable, '-m', 'wc_runtime.component', '--parent', str(os.getpid()),
               '--sigint-grace-s', '5', '--', *command]
    environment = dict(os.environ)
    environment.update(PYTHONNOUSERSITE='1', PYTHONPATH=str(Path(request['project_root'])/'src'),
                       ROS_DOMAIN_ID='83', ROS_LOCALHOST_ONLY='1')
    gui_environment = request.get('rviz_environment', {})
    if not isinstance(gui_environment, dict) or any(not isinstance(k, str) or not isinstance(v, str)
        or '\0' in k or '\0' in v or '=' in k for k, v in gui_environment.items()):
        raise ValueError('RViz 环境必须是有效的字符串映射')
    environment.update(gui_environment)
    log_name = 'rviz.log' if attempt == 1 else 'rviz.retry-1.log'
    with (output/log_name).open('xb') as log:
        return subprocess.Popen(wrapper, cwd=request['project_root'], env=environment,
                                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)


def close_rviz(child):
    """Signal only the directly owned Popen wrapper; never search process names."""
    if child is None or child.poll() is not None:
        return
    try:
        child.send_signal(signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        child.wait(timeout=15)
    except subprocess.TimeoutExpired as error:
        # Keep the ownership wrapper alive to reap descendants; killing the
        # wrapper itself would defeat its parent-death/descendant cleanup.
        raise RuntimeError('本会话 RViz 所有者未按时退出；保留日志，不标记正常保存') from error


def normal_close(state):
    return (isinstance(state, dict) and state.get('state') == 'STOPPED' and
            state.get('exit_code') == 0 and not state.get('cleanup_errors'))


def default_save_destination(request):
    storage = StoragePolicy(request.get('project_root', Path(__file__).absolute().parents[2]))
    if storage.enabled:
        return storage.resolve(Path('maps')/request['session_id'])
    return Path.home()/'maps'/request['session_id']


def terminal_save_choice(request, handle, *, input_stream=None, output_stream=None):
    """Ask only after owned devices have stopped; EOF never means discard."""
    incoming = sys.stdin if input_stream is None else input_stream
    outgoing = sys.stderr if output_stream is None else output_stream
    try:
        interactive = incoming is not None and incoming.isatty()
    except (OSError, ValueError):
        interactive = False
    if not interactive:
        return {'decision': 'pending', 'reason': 'NON_INTERACTIVE_TERMINAL'}
    previous = {}
    def interrupted(_signum, _frame):
        raise KeyboardInterrupt
    def ask(prompt):
        outgoing.write(prompt); outgoing.flush()
        line = incoming.readline()
        if line == '':
            raise EOFError
        return line.strip()
    try:
        # main's acquisition handlers only set a flag. Once all devices are
        # stopped, another Ctrl+C must interrupt the terminal read instead.
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, interrupted)
        while True:
            answer = ask('设备已停止。保存本次地图及实验原始数据？[y/n，n 将废弃本次地图和原始记录] ').lower()
            if answer in ('n', 'no'):
                return {'decision': 'discard', 'reason': 'USER_DECLINED_SAVE'}
            if answer in ('y', 'yes'):
                break
            outgoing.write('请输入 y 或 n。\n'); outgoing.flush()
        default = default_save_destination(request)
        while True:
            entered = ask('保存到新的目录 [%s]：' % default)
            try:
                destination = checked_path(Path(entered).expanduser() if entered else default)
                if destination.exists():
                    outgoing.write('该路径已存在，请选择新的目录，已有文件不会覆盖。\n'); outgoing.flush()
                    continue
            except (OSError, ValueError, RuntimeError) as error:
                outgoing.write('保存路径无效：%s\n' % error); outgoing.flush()
                continue
            return {'decision': 'save', 'destination': str(destination)}
    except (EOFError, KeyboardInterrupt, OSError, ValueError):
        return {'decision': 'pending', 'reason': 'SAVE_CHOICE_INTERRUPTED'}
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


def user_save_choice(request, handle):
    """Prefer the local desktop dialog; a closed/missing dialog never means No."""
    executable = Path(__file__).absolute().parents[2]/'install/main/wc_bringup/lib/wc_bringup/map_save_choice'
    if (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')) and executable.is_file():
        try:
            child = subprocess.run([str(executable), '--session-id', request['session_id'],
                '--default-destination', str(default_save_destination(request))],
                capture_output=True, text=True)
            if child.returncode == 0:
                value = json.loads(child.stdout)
                if isinstance(value, dict) and value.get('decision') in ('save', 'discard', 'pending'):
                    return value
        except (OSError, ValueError, KeyboardInterrupt):
            return {'decision': 'pending', 'reason': 'SAVE_DIALOG_INTERRUPTED'}
    return terminal_save_choice(request, handle)


def manual_control_notice(handle):
    controls=(handle.get('manual_controls') or {}) if isinstance(handle,dict) else {}
    if controls.get('arm_allowed') is not True:
        return '本会话未授权轮控制；当前窗口只显示传感器数据与状态。'
    if controls.get('interaction_policy')=='hybrid_manual':
        return ('当前混合手动配置允许在窗口前台直接手推或按住 WASD；正常松键归零并确认停稳后恢复手推。'
                '失焦、断联、故障单独停止处理；状态以控制面板为准。')
    return '当前配置允许直接按 WASD；手推采用显式模式切换，状态以控制面板为准。'


def run_session(request, backend, *, stop_requested=lambda: False, emit=lambda value: None,
                spawn=launch_rviz, close_gui=close_rviz, wait=time.sleep, clock=time.monotonic,
                choose_save=None):
    """Coordinate a concrete backend; dependency injection keeps tests device-free."""
    handle = gui = None
    started = False
    reason = None
    errors = []
    closed = exported = saved = discarded = save_decision = None
    mapping_enabled = request.get('mapping_enabled', False)
    if type(mapping_enabled) is not bool:
        raise ValueError('mapping_enabled 必须为明确的布尔值')
    choose_save = user_save_choice if choose_save is None else choose_save
    storage = None
    last_storage_check = None
    check_storage = getattr(backend, 'storage_status', None)
    retry_startup = getattr(backend, 'rviz_startup_retry', None)
    gui_attempts, startup_retries = [], []
    def announce(value):
        # A closed terminal/pipe must not interrupt the owned-session cleanup.
        try:
            emit(value)
        except (OSError, ValueError):
            pass
    try:
        preflight = backend.preflight(request)
        if isinstance(preflight, dict) and isinstance(preflight.get('storage'), dict):
            announce({'status': 'STORAGE_CAPACITY_ESTIMATE', **preflight['storage']})
        if stop_requested():
            return {'status': 'CANCELLED_BEFORE_START', 'session_id': request['session_id'],
                    'output': request['output_dir'], 'hardware_started': False}, 130
        handle = backend.start(request)
        started = True
        cloud_source = (handle.get('cloud_source', request.get('cloud_source', 'filtered'))
                        if isinstance(handle, dict) else request.get('cloud_source', 'filtered'))
        if not stop_requested():
            gui_request = dict(request)
            if callable(getattr(backend, 'rviz_environment', None)):
                gui_request['rviz_environment'] = backend.rviz_environment(handle)
            gui_started_at = clock()
            gui = spawn(backend.rviz_command(handle), gui_request)
            gui_attempts.append({'attempt': 1, 'pid': getattr(gui, 'pid', None),
                                 'started_monotonic': gui_started_at, 'log': 'rviz.log'})
        announce({'status': 'MAPPING_STARTED_NOT_YET_VERIFIED' if mapping_enabled else 'LIVE_PREVIEW_STARTED',
              'session_id': request['session_id'], 'mapping_enabled': mapping_enabled,
              'cloud_source': cloud_source, 'cloud_description': cloud_description(cloud_source),
              'mode': request['mode'], 'output': request['output_dir'],
              'odometry_source': handle.get('odometry_source') if isinstance(handle,dict) else None,
              'motion_model': handle.get('motion_model') if isinstance(handle,dict) else None,
              'duration_s': request['duration_s'],
              'message': ((('本次使用平面 EKF，融合轮速、轮差转向与 IMU 转向角速度。' if handle.get('motion_model')=='planar_ekf' else '本次使用轮速 + IMU 里程计，ICP 里程计未启动。')
                  if isinstance(handle,dict) and handle.get('odometry_source')=='wheel_imu' else '')+
                  '无需启动静止等待；采用配置安装坐标作为初始参考。' +
                  ('已加载用户确认的陀螺零偏文件。' if isinstance(handle,dict) and handle.get('confirmed_gyro_bias_applied') else '未加载已确认零偏；后台候选不会自动应用。')+
                  '无总时长和归档帧数上限；数据间断时等待恢复，不因数据年龄退出。关闭 RViz 或按 Ctrl+C 后先停止设备，再询问是否保存及保存路径。'
                  if mapping_enabled else '当前仅实时预览，不建图、不录包；关闭 RViz 或按 Ctrl+C 后停止设备并清理本次临时数据。') +
                  manual_control_notice(handle)})
        while True:
            if stop_requested():
                reason = 'USER_STOP_REQUEST'
                break
            state = backend.inspect(handle)
            if not isinstance(state, dict) or state.get('state') not in ('RUNNING', 'STOPPED', 'FAILED'):
                raise RuntimeError('受管会话状态缺失或未知')
            if state['state'] != 'RUNNING':
                if not normal_close(state):
                    raise RuntimeError('建图链路异常退出: ' + json.dumps(state, ensure_ascii=False))
                reason = 'SESSION_NORMAL_STOP'
                break
            if callable(check_storage):
                now = clock()
                if last_storage_check is None or now-last_storage_check >= 1.0:
                    try:
                        storage = check_storage(handle)
                        if (not isinstance(storage, dict) or type(storage.get('free_bytes')) is not int or
                                storage['free_bytes'] < 0 or type(storage.get('stop_free_bytes')) is not int or
                                storage['stop_free_bytes'] <= 0):
                            raise ValueError('磁盘空间检查返回值无效')
                    except Exception as error:
                        reason = 'STORAGE_CHECK_FAILED'
                        raise RuntimeError('STORAGE_CHECK_FAILED: 无法确认会话可用空间；停止采集并保留已有数据。' + str(error)) from error
                    last_storage_check = now
                    if storage['free_bytes'] <= storage['stop_free_bytes']:
                        reason = 'STORAGE_LIMIT_REACHED'
                        announce({'status': reason, 'storage': storage,
                                  'message': '可用空间已达到 2 GiB 收尾阈值，正在提前停止；停止后询问保存或废弃，此余量不保证导出成功。'})
                        break
            code = gui.poll()
            if code is not None:
                gui_attempts[-1]['exit_code'] = code
                gui_attempts[-1]['exit_observed_monotonic'] = clock()
                if code != 0:
                    evidence = (retry_startup(handle, code, clock()-gui_started_at)
                                if not startup_retries and callable(retry_startup) else None)
                    if isinstance(evidence, dict):
                        # The component exits only after reaping its descendants.
                        # Recheck the original session and stop request before
                        # consuming our one retry; no hardware/health restart.
                        current = backend.inspect(handle)
                        if stop_requested():
                            reason = 'USER_STOP_REQUEST'
                            raise RuntimeError('RViz 异常退出，退出码 ' + str(code))
                        if isinstance(current, dict) and current.get('state') == 'RUNNING':
                            record = {**evidence, 'previous_pid': getattr(gui, 'pid', None),
                                      'next_attempt': 2, 'next_log': 'rviz.retry-1.log', 'retry_started': False}
                            startup_retries.append(record)
                            announce({'status': 'RVIZ_STARTUP_RETRY', 'session_id': request['session_id'],
                                      **record, 'message': 'RViz 图形初始化失败，保留原日志并仅重开一次窗口；采集和建图会话不重启。'})
                            if stop_requested():
                                reason = 'USER_STOP_REQUEST'
                                raise RuntimeError('RViz 异常退出，退出码 ' + str(code))
                            current = backend.inspect(handle)
                            if stop_requested():
                                reason = 'USER_STOP_REQUEST'
                                raise RuntimeError('RViz 异常退出，退出码 ' + str(code))
                            if not isinstance(current, dict) or current.get('state') != 'RUNNING':
                                raise RuntimeError('RViz 重试前原建图会话已停止或异常')
                            gui_request = dict(gui_request, rviz_startup_attempt=2)
                            gui_started_at = clock()
                            gui = spawn(backend.rviz_command(handle), gui_request)
                            record['retry_started'] = True
                            gui_attempts.append({'attempt': 2, 'pid': getattr(gui, 'pid', None),
                                                 'started_monotonic': gui_started_at, 'log': 'rviz.retry-1.log'})
                            continue
                    raise RuntimeError('RViz 异常退出，退出码 ' + str(code))
                reason = 'RVIZ_WINDOW_CLOSED'
                break
            wait(.2)
    except KeyboardInterrupt:
        reason = 'USER_INTERRUPT'
    except Exception as error:
        errors.append(type(error).__name__ + ': ' + str(error))
        reason = reason or 'SESSION_ERROR'
    finally:
        if started:
            announce({'status': 'STOPPING_SESSION', 'session_id': request['session_id'],
                  'message': '正在停止设备并关闭本次会话。' +
                  ('完成后将询问是否保存地图；此时尚未选择保存。' if mapping_enabled else '完成后清理本次临时数据，保留轻量日志。')})
            try:
                close_gui(gui)
                if gui is not None and gui_attempts:
                    gui_attempts[-1]['exit_code'] = gui.poll()
                    gui_attempts[-1]['close_observed_monotonic'] = clock()
            except Exception as error:
                errors.append('RViz cleanup: ' + str(error))
            try:
                closed = backend.stop(handle)
            except Exception as error:
                errors.append('Session cleanup: ' + str(error))
    if started and not normal_close(closed):
        errors.append('会话未确认正常关闭；保留原始数据，不执行地图导出')
    status = 'FAILED'
    if started and not errors:
        try:
            if mapping_enabled:
                announce({'status': 'SESSION_STOPPED_AWAITING_SAVE', 'session_id': request['session_id'],
                          'output': request['output_dir'], 'message': '设备已停止，等待本次地图保存选择。'})
                try:
                    save_decision = choose_save(request, handle)
                except (EOFError, KeyboardInterrupt):
                    save_decision = {'decision': 'pending', 'reason': 'SAVE_CHOICE_INTERRUPTED'}
                if not isinstance(save_decision, dict) or save_decision.get('decision') not in ('save', 'discard', 'pending'):
                    raise RuntimeError('保存选择无效；保留待处理的会话数据')
            else:
                save_decision = {'decision': 'discard', 'reason': 'LIVE_PREVIEW_COMPLETE'}
            decision = save_decision['decision']
            if decision == 'pending':
                status = 'SAVE_PENDING'
            elif decision == 'discard':
                discarded = backend.discard(handle)
                if not isinstance(discarded, dict) or discarded.get('status') != 'SESSION_DATA_DISCARDED':
                    raise RuntimeError('临时数据清理未返回明确成功结果')
                status = 'DISCARDED' if mapping_enabled else 'PREVIEW_COMPLETE'
            else:
                destination = save_decision.get('destination')
                if not isinstance(destination, str) or not destination.strip():
                    raise RuntimeError('保存目录缺失；保留待处理的会话数据')
                saved = backend.save(handle, destination)
                if not isinstance(saved, dict) or saved.get('status') != 'SAVED_EXPERIMENTAL_MAP':
                    raise RuntimeError('保存未返回明确的成功结果')
                exported = saved.get('export')
                status = 'SAVED'
        except Exception as error:
            errors.append('Session finalization: ' + str(error))
    cleanup_errors = saved.get('cleanup_errors') if isinstance(saved, dict) else None
    messages = {'SAVED': '地图数据库、点云与二维地图已保存到所选目录。' +
                ('临时数据清理未完成；详情见 cleanup_errors。' if cleanup_errors else
                 '实验原始数据已保留，位置与哈希见保存清单。' if isinstance(saved, dict) and saved.get('raw_retention') == 'RETAINED_REFERENCED'
                 else '本次工作目录中的临时大数据已清理。'),
                'DISCARDED': '已丢弃本次地图与原始记录，保留轻量会话日志。',
                'PREVIEW_COMPLETE': '实时预览已结束，设备已停止，本次临时数据已清理。',
                'SAVE_PENDING': '设备已停止，保存选择尚未完成；本次数据保留在工作目录，未导出或删除。',
                'FAILED': '未完成所请求的收尾操作；请检查会话日志与现存数据，不代表地图保存成功。'}
    result = {'status': status, 'mapping_enabled': mapping_enabled,
              'cloud_source': handle.get('cloud_source', request.get('cloud_source', 'filtered'))
                  if isinstance(handle, dict) else request.get('cloud_source', 'filtered'),
              'session_id': request['session_id'], 'mode': request['mode'],
              'output': saved.get('output', save_decision['destination']) if status == 'SAVED' else request['output_dir'],
              'session_output': request['output_dir'],
              'stop_reason': reason, 'closed_session': closed, 'export': exported, 'errors': errors,
              'save': saved, 'discard': discarded, 'save_decision': save_decision,
              'cleanup_errors': cleanup_errors,
              'storage': storage,
              'rviz_attempts': gui_attempts, 'rviz_startup_retries': startup_retries,
              'message': messages[status]}
    if reason == 'STORAGE_LIMIT_REACHED':
        result['message'] = '因剩余空间达到收尾阈值提前结束。' + result['message']
    return result, (3 if status == 'SAVE_PENDING' else 2 if status == 'FAILED' else 0)


def main(argv=None, *, project_root=None, backend=None):
    root = project_root or Path(__file__).absolute().parents[2]
    requested = {'stop': False}
    previous = {}
    try:
        request = parse_request(argv, project_root=root)
        backend = backend or load_backend()
        if request.pop('check_config_only', False):
            check = getattr(backend, 'check_configuration', None)
            if not callable(check):
                raise RuntimeError('后端缺少不启动硬件的配置检查接口')
            print(json.dumps(check(request), ensure_ascii=False, allow_nan=False), flush=True)
            return 0
        def request_stop(_signum, _frame):
            requested['stop'] = True
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, request_stop)
        emit = lambda value: print(json.dumps(value, ensure_ascii=False, allow_nan=False), flush=True)
        result, code = run_session(request, backend, stop_requested=lambda: requested['stop'], emit=emit)
        emit(result)
        return code
    except (OSError, ValueError, RuntimeError, KeyError, TypeError) as error:
        print(json.dumps({'status': 'FAILED', 'reason': str(error)}, ensure_ascii=False), file=sys.stderr, flush=True)
        return 2
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


if __name__ == '__main__':
    sys.exit(main())
