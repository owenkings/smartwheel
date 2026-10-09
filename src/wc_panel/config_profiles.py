"""Named, device-bound snapshots of the panel's supported configuration fields.

Profile validation uses the normal ParameterStore editors in an isolated copy.
No profile imports new identities, frames, assembly definitions or evidence.
"""
import copy
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import secrets
import stat
import tempfile
import time

from .parameters import ParameterStore, FILES, WHEEL_NUMBERS, ALGORITHM_FIELDS, algorithm_values
from .storage import atomic_json, ordinary

IDENTITY_FILES = ('device_bindings.json', 'device_bindings.local.json', 'wheel_feedback_current.json')
LIVE_FIELDS = ('gyro_bias_config', 'live_motion_calibration_config')
LABELS = {'extrinsics': '设备外参', 'wheels': '轮参数', 'display': '画面与显示',
          'algorithms': '算法默认值', 'live_motion': '在线运动校正'}


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _encoded(value):
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode('utf-8')


def _replace(path, raw):
    path = ordinary(path)
    temporary = path.with_name('.profile_' + secrets.token_hex(10))
    try:
        with temporary.open('xb') as stream:
            os.chmod(temporary, stat.S_IMODE(path.stat().st_mode) if path.exists() else 0o600)
            stream.write(raw); stream.flush(); os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _groups(configs):
    setup, mapping, cameras = (configs[name] for name in FILES)
    return dict(extrinsics=setup.get('data_transforms', []),
        wheels={key: setup['wheel_odometry'][key] for key in WHEEL_NUMBERS},
        display=dict(visualization=setup.get('visualization', {}),
            camera_rotations={row['role']: row['rotate_deg'] for row in cameras['cameras']},
            camera_profile=cameras['default_profile'], rviz_renderer=mapping.get('rviz_renderer', 'software')),
        algorithms=algorithm_values(mapping),
        live_motion={key: mapping.get(key) for key in LIVE_FIELDS})


def _immutable(configs):
    values = copy.deepcopy(configs)
    setup, mapping, cameras = (values[name] for name in FILES)
    for key in ('data_transforms', 'visualization', 'source_documents'):
        setup.pop(key, None)
    for key in WHEEL_NUMBERS:
        setup['wheel_odometry'].pop(key, None)
    for key in (*ALGORITHM_FIELDS, 'rviz_renderer', *LIVE_FIELDS):
        mapping.pop(key, None)
    cameras.pop('default_profile', None)
    for camera in cameras['cameras']:
        camera.pop('rotate_deg', None)
    return values


class ConfigProfiles:
    def __init__(self, parameter_store):
        self.store = parameter_store
        self.project = ordinary(parameter_store.project.absolute())
        self.root = ordinary(self.project / 'config/panel_profiles')
        self.backup_root = ordinary(parameter_store.backup_root.absolute())
        self.tokens = {}

    def _dependency_path(self, value):
        selected = Path(value)
        if not selected.is_absolute():
            selected = self.project / selected
        path = ordinary(selected)
        if not path.is_relative_to(self.project):
            raise ValueError('配置集只引用本工程内的证据文件：' + str(path))
        if not path.is_file() or path.stat().st_size > 16 * 1024 * 1024:
            raise ValueError('配置依据缺失或超过 16 MiB：' + str(path))
        return path

    def _dependencies(self, configs):
        result = {}
        setup, mapping = configs[FILES[0]], configs[FILES[1]]
        for source in setup.get('source_documents', []):
            path = self._dependency_path(source['snapshot_path'])
            digest = _sha(path.read_bytes())
            if digest != source['sha256']:
                raise ValueError('测量来源文件已改变：' + source['snapshot_path'])
            result[path.relative_to(self.project).as_posix()] = digest
        for field in LIVE_FIELDS:
            if not mapping.get(field):
                continue
            path = self._dependency_path(mapping[field])
            for selected in (path, path.with_name(path.stem + '.evidence.json')):
                selected = self._dependency_path(selected)
                result[selected.relative_to(self.project).as_posix()] = _sha(selected.read_bytes())
        for name, path in self.store.identity_paths().items():
            result[path.relative_to(self.project).as_posix()] = _sha(ordinary(path).read_bytes()) if path.exists() else None
        return result

    def _verify_dependencies(self, dependencies):
        for relative, expected in dependencies.items():
            path = ordinary(self.project / relative)
            if not path.is_relative_to(self.project):
                raise ValueError('配置依据路径超出工程')
            if expected is None:
                if path.exists():
                    raise ValueError('设备身份文件已改变：' + relative)
                continue
            if _sha(self._dependency_path(relative).read_bytes()) != expected:
                raise ValueError('配置依据或设备身份已改变：' + relative)

    def _bound_paths(self, record):
        version = record.get('schema_version', 1)
        if type(version) is not int or version not in (1, 2):
            raise ValueError('不支持的配置集／恢复记录格式')
        if version == 1 and 'source_paths' in record:
            raise ValueError('旧配置记录不支持来源路径字段')
        paths = self.store.selected_paths()
        expected = ({name: 'config/' + name for name in FILES} if version == 1 else record.get('source_paths'))
        if not isinstance(expected, dict) or expected != self.store.path_identity(paths):
            raise ValueError('配置来源路径已改变或旧记录缺少本机路径绑定；保留记录，请人工核对')
        return paths

    def _journal_paths(self, journal):
        paths = self._bound_paths(journal)
        if journal.get('state') not in ('PREPARED', 'ROLLING_BACK', 'ROLLED_BACK', 'COMPLETE'):
            raise ValueError('不支持的配置切换恢复状态')
        for field in ('before', 'after'):
            values = journal.get(field)
            if not isinstance(values, dict) or set(values) != set(FILES) or any(
                    not isinstance(value, str) or not re.fullmatch(r'[a-f0-9]{64}', value) for value in values.values()):
                raise ValueError('配置切换恢复哈希清单不完整')
        if 'active_before' not in journal:
            raise ValueError('配置切换原始状态记录缺失')
        active = journal.get('active_before')
        if active is not None and (not isinstance(active, str) or len(active) % 2 or not re.fullmatch(r'[a-fA-F0-9]*', active)):
            raise ValueError('配置切换原始状态记录无效')
        return paths

    def _load(self, identifier):
        if not isinstance(identifier, str) or not re.fullmatch(r'[0-9]{8}_[0-9]{6}_[a-f0-9]{12}', identifier):
            raise ValueError('配置集标识无效')
        directory = ordinary(self.root / identifier)
        manifest = json.loads(ordinary(directory / 'profile.json').read_text(encoding='utf-8'))
        if manifest.get('schema_version') not in (1, 2) or manifest.get('id') != identifier:
            raise ValueError('配置集格式无效')
        self._bound_paths(manifest)
        if not isinstance(manifest.get('files'), dict) or set(manifest['files']) != set(FILES):
            raise ValueError('配置集文件清单不完整')
        configs = {}
        for name in FILES:
            path = ordinary(directory / name)
            if path.stat().st_size > 2 * 1024 * 1024:
                raise ValueError('配置集文件过大')
            raw = path.read_bytes()
            if _sha(raw) != manifest['files'][name]:
                raise ValueError('配置集文件已改变：' + name)
            configs[name] = json.loads(raw.decode('utf-8'))
        dependencies = manifest.get('dependencies')
        if not isinstance(dependencies, dict) or any(
                not isinstance(key, str) or (value is not None and
                (not isinstance(value, str) or not re.fullmatch(r'[a-f0-9]{64}', value)))
                for key, value in dependencies.items()):
            raise ValueError('配置集依据清单格式无效')
        self._verify_dependencies(dependencies)
        # A manifest may not make a binding or calibration disappear simply by
        # omitting its digest. Require the complete dependency set, including
        # explicit absence of optional identity files at snapshot time.
        if dependencies != self._dependencies(configs):
            raise ValueError('配置集依据或设备身份清单不完整／不一致')
        return manifest, configs

    def _recover(self):
        pointer = self.root / '.transaction.json'
        if not pointer.exists():
            return
        pending = json.loads(ordinary(pointer).read_text(encoding='utf-8'))
        name = pending.get('backup', '')
        if not re.fullmatch(r'profile_[0-9]{8}_[0-9]{6}_[a-f0-9]{12}', name):
            raise ValueError('配置切换恢复记录无效，请检查备份目录')
        backup = ordinary(self.backup_root / name)
        journal = json.loads(ordinary(backup / 'transaction.json').read_text(encoding='utf-8'))
        paths = self._journal_paths(journal)
        if journal.get('state') in ('COMPLETE', 'ROLLED_BACK'):
            expected = journal['after'] if journal['state'] == 'COMPLETE' else journal['before']
            if any(_sha(ordinary(paths[name]).read_bytes()) != expected[name] for name in FILES):
                raise ValueError('已结束事务的配置文件已改变，保留恢复记录')
            pointer.unlink()
            return
        self._rollback(backup, journal)

    def _rollback(self, backup, journal):
        paths = self._journal_paths(journal)
        originals = {}
        for name in FILES:
            original = ordinary(backup / name).read_bytes()
            if _sha(original) != journal['before'][name]:
                raise RuntimeError('配置回滚备份不完整，保留恢复记录：' + str(backup))
            current = ordinary(paths[name]).read_bytes()
            if _sha(current) not in (journal['before'][name], journal['after'][name]):
                raise RuntimeError('配置切换中检测到其他修改，未覆盖；请使用备份核查：' + str(backup))
            originals[name] = original
        try:
            # Persist intent before the first restored file. Recovery must not
            # confuse an interrupted rollback with a committed transaction.
            journal['state'] = 'ROLLING_BACK'
            atomic_json(backup / 'transaction.json', journal)
            for name, raw in originals.items():
                self.store.require_paths(paths)
                if _sha(ordinary(paths[name]).read_bytes()) not in (journal['before'][name], journal['after'][name]):
                    raise RuntimeError('配置回滚期间出现其他修改，保留原始备份')
                _replace(paths[name], raw)
            marker = self.root / 'active.json'
            if journal['active_before'] is None:
                if marker.exists(): marker.unlink()
            else:
                _replace(marker, bytes.fromhex(journal['active_before']))
            journal['state'] = 'ROLLED_BACK'
            atomic_json(backup / 'transaction.json', journal)
            (self.root / '.transaction.json').unlink()
        except Exception as error:
            raise RuntimeError('配置回滚未完成；原始配置和恢复记录已保留在 ' + str(backup) + '：' + str(error)) from error

    def list(self):
        with self.store.lock:
            self._recover()
            revision = self.store._read()[2]
            active = None
            marker = self.root / 'active.json'
            if marker.exists():
                active = json.loads(ordinary(marker).read_text(encoding='utf-8'))
            rows = []
            if self.root.exists():
                for directory in sorted(self.root.iterdir(), reverse=True):
                    if not directory.is_dir() or directory.name.startswith('.'):
                        continue
                    try:
                        manifest, _ = self._load(directory.name)
                        rows.append(dict(id=manifest['id'], name=manifest['name'], created_at=manifest['created_at'],
                                         path=str(directory), available=True, error=''))
                    except (OSError, ValueError, KeyError, TypeError) as error:
                        rows.append(dict(id=directory.name, name=directory.name, path=str(directory),
                                         available=False, error=str(error)))
            return dict(revision=revision, active_name=(active['name'] if active and active.get('revision') == revision
                        else '当前配置（已调整）' if active else '当前工作配置'), profiles=rows,
                        config_paths=[str(path) for path in self.store.selected_paths().values()], profile_root=str(self.root),
                        scope='当前设备、装配和坐标系；外参、轮参数、显示与已支持算法默认值。录包冻结配置不变。')

    def _validate(self, current, proposed):
        if _immutable(current) != _immutable(proposed):
            raise ValueError('配置集属于不同设备、装配或基础配置；仅支持切换当前设备的参数')
        current_sources = {row['id']: row for row in current[FILES[0]].get('source_documents', [])}
        for source in proposed[FILES[0]].get('source_documents', []):
            if current_sources.get(source['id']) != source:
                raise ValueError('配置集测量依据不属于当前工程，不能导入或替换依据')
        proposed = copy.deepcopy(proposed)
        proposed[FILES[0]]['source_documents'] = copy.deepcopy(current[FILES[0]].get('source_documents', []))
        dependencies = self._dependencies(current)
        dependencies.update(self._dependencies(proposed))
        # Temporary validation is under the configured report/backup root, never
        # written into the active config or a recording's frozen configuration.
        self.backup_root.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='profile_validate_', dir=self.backup_root) as temporary:
            workspace = Path(temporary) / 'project'
            (workspace / 'config').mkdir(parents=True)
            for name in FILES:
                atomic_json(workspace / 'config' / name, current[name])
            atomic_json(workspace / 'config/storage.json', dict(schema_version=1, enabled=False))
            for relative, digest in dependencies.items():
                if digest is None: continue
                destination = workspace / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                raw = ordinary(self.project / relative).read_bytes()
                if _sha(raw) != digest: raise ValueError('配置依据在校验过程中改变')
                destination.write_bytes(raw)
            validator = ParameterStore(workspace, Path(temporary) / 'validation_backups')
            for group, value in _groups(proposed).items():
                if group == 'live_motion': continue
                permission = validator.request_edit(group)
                validator.save(group, value, permission['token'], permission['revision'])
            result = validator._read()[1]
        mapping = proposed[FILES[1]]
        if mapping.get('live_motion_calibration_config'):
            from .live_calibration import current_value, verify_value, device_context
            context = device_context(self.project, mapping)
            verify_value(self.project, current_value(self.project, mapping), context=context)
        for field in LIVE_FIELDS:
            if field in mapping: result[FILES[1]][field] = copy.deepcopy(mapping[field])
            else: result[FILES[1]].pop(field, None)
        return result

    def save_current(self, name, expected_revision):
        if not isinstance(name, str) or not 1 <= len(name.strip()) <= 64 or any(ord(char) < 32 for char in name):
            raise ValueError('请填写 1–64 字的配置名称，不含控制字符')
        with self.store.lock:
            self._recover()
            raw, configs, revision, paths = self.store._snapshot()
            if revision != expected_revision: raise ValueError('配置已改变，请刷新后保存配置集')
            self._validate(configs, configs)
            dependencies = self._dependencies(configs)
            identifier = datetime.now().strftime('%Y%m%d_%H%M%S_') + secrets.token_hex(6)
            directory = ordinary(self.root / identifier)
            directory.mkdir(parents=True, exist_ok=False)
            for filename in FILES:
                _replace(directory / filename, raw[filename])
            manifest = dict(schema_version=2, source_paths=self.store.path_identity(paths), id=identifier, name=name.strip(), created_at=datetime.now().isoformat(timespec='seconds'),
                            revision=revision, files={key: _sha(value) for key, value in raw.items()}, dependencies=dependencies)
            if self.store._read()[2] != revision:
                raise ValueError('保存期间配置已改变；该未完成配置集不会用于切换，请刷新后重试')
            atomic_json(directory / 'profile.json', manifest)
            atomic_json(self.root / 'active.json', dict(id=identifier, name=name.strip(), revision=revision))
            return dict(id=identifier, name=name.strip(), path=str(directory), revision=revision)

    def preview(self, identifier):
        with self.store.lock:
            self._recover()
            _, current, revision = self.store._read()
            manifest, proposed = self._load(identifier)
            candidate = self._validate(current, proposed)
            before, after = _groups(current), _groups(candidate)
            changes = [dict(group=key, label=LABELS[key], before=before[key], after=after[key])
                       for key in before if before[key] != after[key]]
            token = secrets.token_urlsafe(32)
            self.tokens = {key: value for key, value in self.tokens.items() if value['expires'] >= time.time()}
            self.tokens[token] = dict(id=identifier, revision=revision, expires=time.time()+900,
                fingerprint=_sha(_encoded(proposed)), dependencies=manifest['dependencies'])
            return dict(token=token, revision=revision, name=manifest['name'], changes=changes,
                message='将切换当前设备的参数默认值。错误外参、轮尺寸或校正会影响地图方向、尺度和融合。原配置会备份；录包内冻结配置不变。',
                effective='next_task', paths=[str(path) for path in self.store.selected_paths().values()])

    def apply(self, token):
        with self.store.lock:
            self._recover()
            proof = self.tokens.get(token)
            raw, current, revision, paths = self.store._snapshot()
            if not proof or proof['expires'] < time.time() or proof['revision'] != revision:
                raise ValueError('配置或预览已改变／过期，请重新预览并确认')
            manifest, proposed = self._load(proof['id'])
            if _sha(_encoded(proposed)) != proof['fingerprint'] or manifest['dependencies'] != proof['dependencies']:
                raise ValueError('配置集在确认后改变，请重新预览')
            candidate = self._validate(current, proposed)
            encoded = {name: _encoded(candidate[name]) for name in FILES}
            if self.store._read()[2] != revision: raise ValueError('配置在校验过程中改变，请重新预览')
            backup = self.backup_root / ('profile_' + datetime.now().strftime('%Y%m%d_%H%M%S_') + secrets.token_hex(6))
            backup.mkdir(parents=True, exist_ok=False)
            for name in FILES:
                _replace(backup / name, raw[name])
            marker = self.root / 'active.json'
            journal = dict(schema_version=2, source_paths=self.store.path_identity(paths), state='PREPARED', before={name: _sha(raw[name]) for name in FILES},
                after={name: _sha(encoded[name]) for name in FILES}, profile=proof['id'],
                active_before=ordinary(marker).read_bytes().hex() if marker.exists() else None)
            atomic_json(backup / 'transaction.json', journal)
            atomic_json(self.root / '.transaction.json', dict(backup=backup.name))
            try:
                expected = dict(raw)
                for name in FILES:
                    self.store.require_bytes(paths, expected)
                    _replace(paths[name], encoded[name])
                    expected[name] = encoded[name]
                self.store.require_bytes(paths, expected)
                new_revision = self.store._read()[2]
                atomic_json(marker, dict(id=proof['id'], name=manifest['name'], revision=new_revision))
                journal.update(state='COMPLETE', revision=new_revision)
                atomic_json(backup / 'transaction.json', journal)
            except Exception as error:
                self._rollback(backup, journal)
                raise RuntimeError('配置切换失败，已恢复原配置；备份：' + str(backup) + '；原因：' + str(error)) from error
            # COMPLETE is the commit point. A failed cleanup must never start a
            # rollback of already committed files; a later open removes this
            # pointer after observing the durable COMPLETE journal.
            cleanup_warning = ''
            try:
                (self.root / '.transaction.json').unlink()
            except OSError as error:
                cleanup_warning = '参数已提交；恢复指针尚未清理，下次打开将重试：' + str(error)
            self.tokens.pop(token, None)
            return dict(revision=new_revision, backup=str(backup), name=manifest['name'], effective='next_task',
                        cleanup_warning=cleanup_warning)
