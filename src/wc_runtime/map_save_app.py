"""Resume a stopped temporary session's retention decision without starting devices."""
import argparse
import json
from pathlib import Path
import sys

from .mapping_app import (MODES, checked_path, load_backend, normal_close,
                          session_name, user_save_choice)
from .prepare_picker_input import project_path, read_json


def session_handle(workdir, *, project_root):
    """Reconstruct only the handle recorded by this project's current map entry."""
    root = checked_path(project_root)
    directory = project_path(root, Path(workdir).expanduser())
    if not root.is_dir() or not directory.is_dir() or directory == root:
        raise ValueError('需要工程目录内已存在的临时会话目录')
    if directory.is_relative_to(root) and directory.relative_to(root).parts[0] in ('.git', '.codex', '.agents', '.phase1_runtime'):
        raise ValueError('不能把工程元数据或进程管理目录当作临时地图')
    session = read_json(root, directory/'session.json')
    if not isinstance(session, dict) or (
            session.get('kind') != 'MAPPING_APP_EXPERIMENT' or
            session.get('data_retention') not in ('TEMPORARY_UNTIL_USER_SAVE_CHOICE', 'experiment', 'map_only')):
        raise ValueError('仅允许处理明确标记的临时会话；不能处理历史地图')
    identifier = session.get('session_id')
    if not isinstance(identifier, str):
        raise ValueError('临时会话缺少有效 session_id')
    session_name(identifier)
    if (not isinstance(session.get('output_dir'), str) or
            project_path(root, session['output_dir']) != directory or session.get('project_root') != str(root)
            or session.get('mode') not in MODES or session.get('source_mode') != 'real'
            or type(session.get('mapping_enabled')) is not bool
            or session.get('odometry_source') not in ('wheel_imu', 'icp')):
        raise ValueError('临时会话路径、模式或身份不匹配')
    runtime = project_path(root, root/'.phase1_runtime/sessions'/identifier/'mapping_app')
    config = read_json(root, directory/'runtime_config.json')
    profile = session['data_retention']
    if profile in ('experiment', 'map_only') and config.get('retention_profile') != profile:
        raise ValueError('会话留存策略与不可变配置不一致')
    if (directory/'retention.json').exists():
        disposition=read_json(root,directory/'retention.json')
        if disposition.get('saved_map') or disposition.get('raw_retention') == 'DISCARDED':
            raise ValueError('会话已经保存或废弃；不能再次作为待决定会话清理')
    from .mapping_save import assert_archive_not_referenced
    assert_archive_not_referenced(root,directory)
    if (not isinstance(config, dict) or config.get('schema_version') != 1
            or config.get('status') != 'EXPERIMENT' or config.get('source_mode') != 'real'
            or any(config.get(key) != session[key] for key in ('session_id', 'mode', 'odometry_source'))
            or config.get('mapping_enabled') is not session['mapping_enabled']):
        raise ValueError('会话配置快照身份不匹配')
    for filename in ('plan.json', 'manifest.json'):
        document = read_json(root, runtime/filename)
        if (not isinstance(document, dict) or document.get('session_id') != identifier
                or document.get('role') != 'mapping_app'):
            raise ValueError('会话管理记录身份不匹配: ' + filename)
        if filename == 'plan.json' and document.get('mapping_enabled') is not session['mapping_enabled']:
            raise ValueError('会话管理计划模式不匹配')
        if filename == 'manifest.json' and not normal_close(document):
            raise RuntimeError('会话尚未确认正常停止；保留临时数据')
    handle = {'session_id': identifier, 'mode': session['mode'], 'directory': str(directory),
              'runtime': str(runtime), 'odometry_source': session['odometry_source'],
              'mapping_enabled': session['mapping_enabled']}
    if profile in ('experiment', 'map_only'):
        handle['retention_profile']=profile
    return session, handle


def recover_session(workdir, destination=None, *, project_root, backend, choose_save=None):
    """Validate first; a pending answer leaves every session file untouched."""
    request = handle = closed = saved = discarded = choice = None
    status = 'FAILED'
    errors = []
    try:
        request, handle = session_handle(workdir, project_root=project_root)
        closed = backend.inspect(handle)
        if not normal_close(closed):
            raise RuntimeError('会话尚未确认正常停止；保留临时数据')
        if destination is None:
            try:
                choice = (choose_save or user_save_choice)(request, handle)
            except (EOFError, KeyboardInterrupt):
                choice = {'decision': 'pending', 'reason': 'SAVE_CHOICE_INTERRUPTED'}
        else:
            choice = {'decision': 'save', 'destination': str(destination)}
        if not isinstance(choice, dict) or choice.get('decision') not in ('save', 'discard', 'pending'):
            raise ValueError('保存选择无效；保留待处理的会话数据')
        if choice['decision'] == 'pending':
            status = 'SAVE_PENDING'
        elif choice['decision'] == 'discard':
            discarded = backend.discard(handle)
            if not isinstance(discarded, dict) or discarded.get('status') != 'SESSION_DATA_DISCARDED':
                raise RuntimeError('临时数据清理未返回明确成功结果')
            status = 'DISCARDED'
        else:
            if not handle['mapping_enabled']:
                raise ValueError('纯预览会话没有地图可以保存')
            chosen = choice.get('destination')
            if not isinstance(chosen, str) or not chosen.strip():
                raise ValueError('保存目录缺失；保留待处理的会话数据')
            chosen = Path(chosen).expanduser()
            chosen = checked_path(chosen if chosen.is_absolute() else Path(project_root)/chosen)
            if chosen.exists():
                raise ValueError('保存路径已存在，请选择新的目录；原有文件不会覆盖')
            saved = backend.save(handle, str(chosen))
            if not isinstance(saved, dict) or saved.get('status') != 'SAVED_EXPERIMENTAL_MAP':
                raise RuntimeError('保存未返回明确的成功结果')
            status = 'SAVED'
    except KeyboardInterrupt:
        errors.append('收尾操作中断；请检查工作目录和所选保存目录中的现存数据')
    except Exception as error:
        errors.append(str(error))
    cleanup_errors = saved.get('cleanup_errors') if isinstance(saved, dict) else None
    messages = {
        'SAVED': '地图数据库、点云与二维地图已保存到所选目录。' +
            ('临时数据清理未完成；详情见 cleanup_errors。' if cleanup_errors else
             '实验原始数据已保留，位置与哈希见保存清单。' if isinstance(saved,dict) and saved.get('raw_retention')=='RETAINED_REFERENCED'
             else '本次工作目录中的临时大数据已清理。'),
        'DISCARDED': '已丢弃本次地图与原始记录，保留轻量会话日志。',
        'SAVE_PENDING': '设备已停止，保存选择尚未完成；本次数据保留在工作目录。',
        'FAILED': '未完成所请求的收尾操作；请检查会话日志与现存数据，不代表地图保存成功。',
    }
    directory = handle['directory'] if handle else str(workdir)
    result = {'status': status, 'session_id': handle['session_id'] if handle else None,
              'mode': handle['mode'] if handle else None,
              'mapping_enabled': handle['mapping_enabled'] if handle else None,
              'session_output': directory, 'output': saved.get('output', str(chosen)) if status == 'SAVED' else directory,
              'closed_session': closed, 'save_decision': choice, 'save': saved, 'discard': discarded,
              'cleanup_errors': cleanup_errors, 'errors': errors, 'message': messages[status]}
    return result, (3 if status == 'SAVE_PENDING' else 2 if status == 'FAILED' else 0)


def main(argv=None, *, project_root=None, backend=None):
    parser = argparse.ArgumentParser(description='恢复已停止的临时会话保存选择；不启动设备，不处理历史地图。')
    parser.add_argument('workdir', type=Path, metavar='WORKDIR', help='工程内的临时会话工作目录')
    parser.add_argument('destination', nargs='?', type=Path, metavar='DEST',
                        help='新的最终地图目录；省略时在终端询问是否保存和保存路径')
    args = parser.parse_args(argv)
    root = project_root or Path(__file__).absolute().parents[2]
    try:
        backend = backend or load_backend()
        result, code = recover_session(args.workdir, args.destination, project_root=root, backend=backend)
    except (OSError, ValueError, RuntimeError, ImportError) as error:
        result, code = {'status': 'FAILED', 'session_output': str(args.workdir), 'errors': [str(error)]}, 2
    print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
    return code


if __name__ == '__main__':
    sys.exit(main())
