"""Explicit map retention after owned processes stop; atomic copies and bounded deletion scope."""
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import uuid

from wc_maps.store import _rename_no_replace, _sync_directory
from .mapping_app import checked_path, normal_close
from .prepare_picker_input import project_path, read_json, write_new, json_bytes


METADATA = ('session.json', 'runtime_config.json', 'hardware_setup.json', 'wheel_candidate.json',
    'confirmed_gyro_bias.json', 'confirmed_gyro_bias.evidence.json',
    'cameras_config.json', 'prior_config.json', 'prior/initialization.json', 'prior/status.json',
    'health/status.json', 'wheel_status.json', 'input/status.json', 'input/checkpoint.json', 'input/bootstrap.json',
    'health.json', 'diagnosis_zh.md', 'capability_assessment.json', 'dataset/capture_manifest.json',
    'dataset/audit.json', 'cameras/status.json')
TEMPORARY_DATA = ('bag', 'input', 'slam', 'export', 'health/latest_cloud.npz', 'health/latest_grid.npz',
                  'health/tracking.jsonl', 'prior/guesses.jsonl', 'wheel_feedback.jsonl', 'dataset')


def retention_profile(directory, handle):
    """Omission always preserves evidence; only an explicit map_only may delete."""
    config = read_json(Path(directory).parent, Path(directory)/'runtime_config.json')
    profile = handle.get('retention_profile', config.get('retention_profile', 'experiment'))
    if profile not in ('experiment', 'map_only'):
        raise ValueError('retention_profile must be experiment or explicit map_only')
    if handle.get('retention_profile') and config.get('retention_profile') and profile != config['retention_profile']:
        raise ValueError('retention profile differs from the session configuration')
    session = read_json(Path(directory).parent, Path(directory)/'session.json')
    if session.get('data_retention') in ('experiment', 'map_only') and session['data_retention'] != profile:
        raise ValueError('retention profile differs from session metadata')
    return profile


def _raw_reference(directory):
    """Hash retained files in place. References require keeping the source folder."""
    files = {}
    for relative in ('bag', 'input', 'dataset', 'wheel_feedback.jsonl', 'prior/guesses.jsonl',
                     'health/tracking.jsonl'):
        path = directory/relative
        if not path.exists(): continue
        checked_path(path)
        candidates = sorted(path.rglob('*')) if path.is_dir() else [path]
        for candidate in candidates:
            checked_path(candidate)
            if candidate.is_symlink(): raise ValueError('raw archive must not contain symbolic links')
            if not candidate.is_file(): continue
            before = candidate.stat()
            checksum = _file_digest(candidate)
            after = candidate.stat()
            if (before.st_size, before.st_mtime_ns, before.st_ino) != (after.st_size, after.st_mtime_ns, after.st_ino):
                raise RuntimeError('raw archive changed while hashing')
            files[str(candidate.relative_to(directory))] = {'bytes': before.st_size, 'sha256': checksum}
    return {'schema_version': 1, 'status': 'RETAINED_REFERENCED', 'source_directory': str(directory),
            'files': files, 'independent_copy': False,
            'limitation': '引用要求保留源目录；哈希用于复核完整性，不证明采集没有丢帧。'}


def _replace_durable(path, value):
    temporary = path.with_name('.'+path.name+'.writing-'+uuid.uuid4().hex)
    try:
        write_new(temporary, json_bytes(value))
        with temporary.open('rb') as stream: os.fsync(stream.fileno())
        os.replace(temporary, checked_path(path))
        _sync_directory(path.parent)
    finally:
        if temporary.exists(): temporary.unlink()


@contextmanager
def stopped_session(root, handle, inspect):
    import fcntl
    root = Path(root).absolute()
    directory = project_path(root, handle['directory'])
    runtime = project_path(root, handle['runtime'])
    if runtime != root/'.phase1_runtime/sessions'/handle['session_id']/'mapping_app':
        raise ValueError('会话管理路径不匹配，拒绝保存或删除')
    session = read_json(root, directory/'session.json')
    if (session.get('session_id') != handle['session_id'] or not isinstance(session.get('output_dir'), str)
            or project_path(root, session['output_dir']) != directory
            or session.get('data_retention') not in ('TEMPORARY_UNTIL_USER_SAVE_CHOICE', 'experiment', 'map_only')
            or type(session.get('mapping_enabled')) is not bool
            or session['mapping_enabled'] != handle['mapping_enabled']):
        raise ValueError('仅允许处理本次明确标记的临时会话；不能清理历史地图')
    lock = project_path(root, runtime/'retention.lock')
    fd = os.open(lock, os.O_CREAT | os.O_RDWR | getattr(os, 'O_NOFOLLOW', 0), 0o600)
    with os.fdopen(fd, 'a') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if not normal_close(inspect(handle)):
            raise RuntimeError('会话尚未确认正常停止；保留临时数据')
        yield directory, runtime


def _file_digest(path):
    checksum = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''): checksum.update(block)
    return checksum.hexdigest()


def _copy_durable(source, destination):
    source = checked_path(source)
    before = source.stat()
    if not stat.S_ISREG(before.st_mode): raise ValueError('只能复制普通地图文件')
    destination.parent.mkdir(parents=True, exist_ok=True)
    checksum = hashlib.sha256(); count = 0
    with source.open('rb') as src, destination.open('xb') as dst:
        for block in iter(lambda: src.read(1024*1024), b''):
            dst.write(block); checksum.update(block); count += len(block)
        dst.flush(); os.fsync(dst.fileno())
    after = source.stat()
    if (before.st_size, before.st_mtime_ns, before.st_ino) != (after.st_size, after.st_mtime_ns, after.st_ino):
        raise RuntimeError('保存期间原始地图发生变化')
    if count != before.st_size or _file_digest(destination) != checksum.hexdigest():
        raise RuntimeError('地图副本校验失败')
    return {'bytes': count, 'sha256': checksum.hexdigest()}


def _discard_data(root, directory, runtime, *, profile):
    if profile != 'map_only': raise ValueError('raw cleanup requires explicit map_only profile')
    # Keep final diagnostic snapshots before removing the heavy input archive.
    metadata = directory/'retained_metadata'
    failed_exports = sorted(p.name for p in directory.iterdir()
                            if re.fullmatch(r'export_failed_[0-9a-f]{32}', p.name))
    diagnostic_files = METADATA + ('export/map_quality.json',) + tuple(name+'/export.log' for name in failed_exports)
    for relative in diagnostic_files:
        source = project_path(root, directory/relative)
        if (relative.startswith(('input/', 'dataset/')) or relative.endswith('/export.log') or relative=='export/map_quality.json') and source.is_file():
            if source.stat().st_size > 5_000_000: raise RuntimeError('会话诊断元数据过大')
            target = metadata/relative
            if not target.exists(): _copy_durable(source, target)
    paths = []
    for relative in (*TEMPORARY_DATA, *failed_exports):
        path = project_path(root, directory/relative)
        if path.exists():
            if path.is_dir():
                for parent, dirs, files in os.walk(path, followlinks=False):
                    if Path(parent).stat().st_dev != directory.stat().st_dev:
                        raise ValueError('临时数据包含其他文件系统，拒绝递归删除')
                    for name in dirs+files:
                        child = Path(parent)/name
                        if child.is_symlink(): raise ValueError('临时数据包含符号链接，拒绝递归删除')
            paths.append(path)
    removed = []
    for path in paths:
        if path.is_dir(): shutil.rmtree(path)
        else: path.unlink()
        removed.append(str(path.relative_to(directory)))
    result = {'status': 'SESSION_DATA_DISCARDED', 'raw_retention': 'DISCARDED', 'retention_profile': profile,
              'output': str(directory), 'removed': removed,
              'diagnostics_retained': True, 'historical_sessions_touched': False,
              'map_quality_evidence': ('retained_metadata/export/map_quality.json'
                if (metadata/'export/map_quality.json').is_file() else 'NOT_RUN_OR_NOT_AVAILABLE_BEFORE_DISCARD')}
    _replace_durable(directory/'retention.json', result)
    write_new(runtime/('discard-'+uuid.uuid4().hex+'.json'), json_bytes(result))
    return result


def assert_archive_not_referenced(root, directory):
    """A durable pre-commit intent protects references even if final metadata fails."""
    if (directory/'retention.json').exists():
        disposition=read_json(root,directory/'retention.json')
        if disposition.get('saved_map'):
            raise ValueError('本会话原始档案已被已保存地图引用，拒绝作为未保存结果废弃')
    intents=project_path(root,directory/'save_commit_intents')
    if intents.exists():
        for path in intents.iterdir():
            intent=read_json(root,project_path(root,path))
            if intent.get('status')!='ABORTED_BEFORE_COMMIT':
                raise ValueError('存在保存提交意图或已保存引用；核对目标地图后再处理，不删除原始档案')


def discard_session(root, handle, *, inspect, user_confirmed=False):
    root = Path(root).absolute()
    with stopped_session(root, handle, inspect) as (directory, runtime):
        profile = retention_profile(directory, handle)
        if user_confirmed or profile=='map_only':
            assert_archive_not_referenced(root,directory)
        if profile == 'experiment' and not user_confirmed:
            result = {'status': 'SESSION_DATA_RETAINED', 'raw_retention': 'RETAINED_IN_PLACE',
                      'retention_profile': profile, 'output': str(directory), 'removed': [],
                      'diagnostics_retained': True, 'historical_sessions_touched': False}
            _replace_durable(directory/'retention.json', result)
            write_new(runtime/('retain-'+uuid.uuid4().hex+'.json'), json_bytes(result))
            return result
        result = _discard_data(root, directory, runtime, profile='map_only' if user_confirmed else profile)
        if user_confirmed:
            result.update(disposition_reason='USER_DECLINED_SAVE', original_retention_profile=profile,
                          replayable=False, user_confirmed_discard=True)
            _replace_durable(directory/'retention.json', result)
        return result


def save_session(root, handle, destination, *, inspect, export):
    root = Path(root).absolute()
    chosen = Path(destination).expanduser()
    chosen = checked_path(chosen if chosen.is_absolute() else root/chosen)
    from .storage_policy import StoragePolicy
    storage = StoragePolicy(root)
    if storage.enabled:
        chosen = storage.resolve(chosen)
        if not any(chosen.is_relative_to(base) and chosen != base for base in storage.data_roots()):
            raise ValueError('Saved maps must use a new directory on configured USB storage')
    directory = checked_path(handle['directory'])
    if chosen.exists(): raise ValueError('保存路径已存在，请选择一个新的地图目录')
    if chosen == directory or directory in chosen.parents or chosen in directory.parents:
        raise ValueError('最终保存路径不能与临时会话目录重叠')
    if chosen.is_relative_to(root) and chosen.relative_to(root).parts[0] in (
            '.git', '.codex', '.agents', '.phase1_runtime', 'src', 'config', 'SDKs', 'install', 'build'):
        raise ValueError('最终地图不能保存到代码、配置或安装目录')
    with stopped_session(root, handle, inspect) as (directory, runtime):
        profile = retention_profile(directory, handle)
        if not handle['mapping_enabled']: raise ValueError('纯预览会话没有地图可以保存')
        exported = export(handle)
        if exported.get('status') != 'EXPORTED_EXPERIMENTAL_MAP':
            raise RuntimeError('地图导出未通过完整性检查')
        files = [(directory/'slam/rtabmap.db', Path('slam/rtabmap.db'))]
        files += [(path, Path('export')/path.name) for path in (directory/'export').iterdir() if path.is_file()]
        files += [(directory/relative, Path(relative)) for relative in METADATA if (directory/relative).is_file()]
        required = sum(source.stat().st_size for source, _ in files)
        parent = chosen.parent
        while not parent.exists(): parent = parent.parent
        if shutil.disk_usage(parent).free < required+64*1024**2:
            raise RuntimeError('保存目标空间不足；临时地图已保留')
        chosen.parent.mkdir(parents=True, exist_ok=True)
        checked_path(chosen.parent)
        # Persist every newly created ancestor before raw data can be removed.
        ancestor = chosen.parent
        while True:
            _sync_directory(ancestor)
            if ancestor == parent: break
            ancestor = ancestor.parent
        stage = chosen.with_name('.'+chosen.name+'.saving-'+uuid.uuid4().hex)
        stage.mkdir(exist_ok=False)
        committed = False
        intent_path = None
        intent = None
        try:
            hashes = {str(relative): _copy_durable(source, stage/relative) for source, relative in files}
            reference = _raw_reference(directory) if profile == 'experiment' else None
            if reference is not None:
                write_new(stage/'raw_archive.reference.json', json_bytes(reference))
                with (stage/'raw_archive.reference.json').open('rb') as stream: os.fsync(stream.fileno())
            result = {'status': 'SAVED_EXPERIMENTAL_MAP', 'output': str(chosen), 'session_id': handle['session_id'],
                'mode': handle['mode'], 'source_work_directory': str(directory), 'files': hashes,
                'raw_recordings_saved': False, 'formal_acceptance': False, 'navigation_validated': False,
                'retention_profile': profile,
                'raw_retention': 'RETAINED_REFERENCED' if reference is not None else 'CLEANUP_PENDING',
                'raw_archive_reference': str(chosen/'raw_archive.reference.json') if reference is not None else None,
                'database': str(chosen/'slam/rtabmap.db'), 'export_directory': str(chosen/'export'),
                'map_quality':exported.get('map_quality',{'status':'UNKNOWN_LEGACY_EXPORT'})}
            write_new(stage/'saved_map.json', json_bytes(result))
            with (stage/'saved_map.json').open('rb') as stream: os.fsync(stream.fileno())
            for folder in sorted((p for p in stage.rglob('*') if p.is_dir()), key=lambda p:len(p.parts), reverse=True):
                _sync_directory(folder)
            _sync_directory(stage)
            intent_directory=project_path(root,directory/'save_commit_intents')
            intent_directory.mkdir(exist_ok=True)
            _sync_directory(directory)
            intent_path=intent_directory/(uuid.uuid4().hex+'.json')
            intent={'schema_version':1,'status':'PREPARED_BEFORE_COMMIT','session_id':handle['session_id'],
                    'destination':str(chosen),'stage':str(stage),'retention_profile':profile,
                    'raw_archive_reference':result['raw_archive_reference']}
            _replace_durable(intent_path,intent)
            _rename_no_replace(stage, chosen)
            committed = True
            _sync_directory(chosen.parent)
        except BaseException:
            if not committed and stage.exists():
                # A surviving stage proves this atomic rename did not publish it.
                # If this marker cannot be synchronized, PREPARED remains and
                # later discard fails closed until the destination is reviewed.
                if intent_path is not None and intent_path.exists():
                    _replace_durable(intent_path,{**intent,'status':'ABORTED_BEFORE_COMMIT'})
                shutil.rmtree(stage)
            raise
        result['cleanup_errors'] = []
        try:
            result['cleanup'] = (_discard_data(root, directory, runtime, profile=profile) if profile == 'map_only'
                                 else {'status': 'SESSION_DATA_RETAINED', 'raw_retention': 'RETAINED_REFERENCED', 'removed': []})
        except Exception as error: result['cleanup_errors'].append(str(error))
        result['raw_retention'] = ('DISCARDED' if profile == 'map_only' and not result['cleanup_errors'] else
                                   ('PARTIALLY_RETAINED' if profile == 'map_only' else 'RETAINED_REFERENCED'))
        try:
            _replace_durable(directory/'retention.json', {'retention_profile':profile, 'raw_retention':result['raw_retention'],
                              'saved_map':str(chosen), 'raw_archive_reference':result['raw_archive_reference'],
                              'cleanup_errors':result['cleanup_errors']})
            _replace_durable(chosen/'saved_map.json', result)
            _replace_durable(intent_path,{**intent,'status':'COMMITTED'})
        except Exception as error:
            # The map was already durably committed; report the late metadata
            # error without falsely treating a completed save as an absent map.
            result['cleanup_errors'].append('RETENTION_METADATA_UPDATE_FAILED: '+str(error))
        write_new(runtime/('save-'+uuid.uuid4().hex+'.json'), json_bytes(result))
        return result
