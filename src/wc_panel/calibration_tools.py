"""Owned, loopback-only access to the existing frozen-cloud calibration tools.

This service does not acquire sensors or apply calibration candidates. The child
uses the same runtime entry as pick_lidar_points.sh / align_lidar_clouds.sh,
with guarded exports and an ephemeral port instead of their interactive defaults.
"""
import atexit
import copy
import hashlib
import json
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import secrets
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import urlsplit

from .storage import ordinary
from wc_runtime.storage_policy import StoragePolicy


MAX_INPUT_BYTES = 128_000_000
ACTIVE = {'STARTING', 'RUNNING', 'STOPPING'}


def owns_loopback_listener(pid, url, mode):
    """A readiness message alone must not point the panel at another process."""
    try:
        parsed = urlsplit(url)
        if (parsed.scheme != 'http' or parsed.hostname != '127.0.0.1'
                or parsed.username or parsed.password or parsed.query or parsed.fragment
                or parsed.path != ('/alignment' if mode == 'alignment' else '/')
                or not parsed.port or not 0 < parsed.port <= 65535):
            return False
        base = Path('/proc') / str(pid)
        owned = set()
        for fd in (base/'fd').iterdir():
            try:
                target = os.readlink(fd)
                if target.startswith('socket:['):
                    owned.add(target[8:-1])
            except OSError:
                continue
        expected = '0100007F:{:04X}'.format(parsed.port)
        for row in (base/'net/tcp').read_text().splitlines()[1:]:
            fields = row.split()
            if len(fields) > 9 and fields[1] == expected and fields[3] == '0A' and fields[9] in owned:
                return True
    except (OSError, ValueError, TypeError):
        pass
    return False


class CalibrationTools:
    def __init__(self, project_root):
        self.project_root = ordinary(Path(project_root).absolute())
        self.policy = StoragePolicy(self.project_root)
        self._lock = threading.RLock()
        self._tasks = {}
        self._latest = None
        atexit.register(self.close)

    def _input(self, value=''):
        expected = None
        expected_scene = None
        if value:
            path = self.policy.resolve(value)
        else:
            pointer = ordinary(self.project_root/'state/point_picker_input.json')
            if pointer.exists():
                if not pointer.is_file() or pointer.stat().st_size > 16384:
                    raise ValueError('默认点云清单无效；请选择有效的 prepared.json')
                manifest = json.loads(pointer.read_text(encoding='utf-8'))
                if manifest.get('schema_version') != 1 or manifest.get('status') != 'READY_FOR_OFFLINE_PICKING':
                    raise ValueError('默认点云尚未准备完成')
                relative = manifest.get('prepared_input')
                if (not isinstance(relative, str) or not relative or '\\' in relative
                        or PurePosixPath(relative).is_absolute() or PureWindowsPath(relative).drive
                        or '..' in relative.split('/') or PurePosixPath(relative).parts[0] != 'reports'
                        or PurePosixPath(relative).suffix.lower() != '.json'):
                    raise ValueError('默认点云清单中的文件位置无效')
                path = self.policy.resolve(relative)
                expected = manifest.get('prepared_file_sha256')
                if not isinstance(expected, str) or len(expected) != 64:
                    raise ValueError('默认点云清单缺少完整校验值')
                expected_scene = manifest.get('scene_id')
            else:
                path = self.policy.resolve('reports/calibration/stability_single_20260913T011555Z/prepared.json')
        path = ordinary(path)
        if not path.is_file() or not 0 < path.stat().st_size <= MAX_INPUT_BYTES:
            raise ValueError('请选择已准备好的双雷达 prepared.json（不超过128 MB）')
        with path.open('rb') as stream:
            raw = stream.read(MAX_INPUT_BYTES + 1)
        if len(raw) > MAX_INPUT_BYTES:
            raise ValueError('点云准备文件超过读取上限')
        if expected and hashlib.sha256(raw).hexdigest() != expected.lower():
            raise ValueError('默认点云文件已改变，与准备清单的校验值不一致')
        data = json.loads(raw)
        if (not isinstance(data, dict) or data.get('schema_version') != 1
                or data.get('status') != 'PREPARED_NOT_VALIDATED'
                or not isinstance(data.get('training'), list) or not data['training']):
            raise ValueError('文件不是可用于配对的 prepared.json')
        if expected_scene is not None and data['training'][0].get('id') != expected_scene:
            raise ValueError('默认点云场景与准备清单不一致')
        self.policy.check()
        return path

    def defaults(self):
        result = dict(input='', error='', output_root='', alignment_output_root='')
        try:
            result['output_root'] = str(self.policy.resolve('data/calibration/manual_points'))
            result['alignment_output_root'] = str(self.policy.resolve('data/calibration/alignment_candidates'))
            result['input'] = str(self._input())
        except (OSError, ValueError, TypeError, KeyError) as error:
            result['error'] = str(error)
        return result

    def active_paths(self):
        with self._lock:
            protected = []
            for identifier in self._tasks:
                row = self.status(identifier)
                if row['status'] not in ACTIVE and not row['process_alive']:
                    continue
                protected.extend(Path(row[key]) for key in ('input', 'output_root', 'log_path'))
                # Both pages are available on each picker server. Its browser
                # links can open the other page without changing launch mode.
                protected.extend(Path(path) for path in row.get('candidate_roots', []))
            return list(dict.fromkeys(protected))

    def _copy(self, task):
        return copy.deepcopy(task['row'])

    def status(self, identifier=None):
        with self._lock:
            task = self._tasks.get(identifier or self._latest)
            if task is None:
                if identifier:
                    raise ValueError('该点云配对任务不属于当前面板')
                return None
            process = task.get('process')
            code = process.poll() if process else None
            row = task['row']
            row['process_alive'] = bool(process and code is None)
            if process and code is not None and row['status'] in ACTIVE:
                row.update(status='COMPLETE' if code == 0 else 'FAILED', exit_code=code, url='')
                if code and not row.get('error'):
                    row['error'] = '工具异常退出（{}），请查看运行日志'.format(code)
            return self._copy(task)

    def start(self, input_path='', mode='points'):
        if mode not in ('points', 'alignment'):
            raise ValueError('请选择对应点配对或拖动 / ICP')
        if os.name != 'posix' or not Path('/proc/self/fd').is_dir():
            raise RuntimeError('点云配对工具请在 Orin Linux 面板中启动')
        with self._lock:
            for identifier in self._tasks:
                existing = self.status(identifier)
                if existing['status'] in ACTIVE or existing['process_alive']:
                    raise ValueError('已有点云配对工具在运行，请先停止或重新打开当前工具')
            source = self._input(input_path)
            public_entry = self.project_root/'scripts'/('align_lidar_clouds.sh' if mode == 'alignment' else 'pick_lidar_points.sh')
            if not ordinary(public_entry).is_file():
                raise ValueError('缺少既有点云工具入口：' + str(public_entry))
            directory = self.policy.resolve('reports/panel/calibration_tools') / (time.strftime('%Y%m%d_%H%M%S') + '_' + secrets.token_hex(4))
            self.policy.check()
            directory.mkdir(parents=True, exist_ok=False)
            log_path = directory/'runtime.log'
            identifier = directory.name
            task = dict(row=dict(id=identifier, status='STARTING', mode=mode, input=str(source),
                                 url='', log_path=str(log_path), error='', entry=str(public_entry),
                                 candidate_roots=[str(self.policy.resolve('data/calibration/'+name))
                                                  for name in ('manual_points', 'alignment_candidates')],
                                 output_root=str(self.policy.resolve('data/calibration/alignment_candidates' if mode == 'alignment' else 'data/calibration/manual_points'))),
                        ready=threading.Event(), process=None)
            self._tasks[identifier] = task
            self._latest = identifier
            env = os.environ.copy()
            env.update(WHEELCHAIR_PROJECT_ROOT=str(self.project_root), PYTHONNOUSERSITE='1',
                       PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1',
                       PYTHONPATH=str(self.project_root/'src') + os.pathsep + env.get('PYTHONPATH', ''))
            command = [sys.executable, '-B', '-s', '-m', 'wc_panel.calibration_tools', '--serve',
                       '--parent-pid', str(os.getpid()), '--input', str(source), '--mode', mode]
            try:
                self.policy.check()
                task['process'] = subprocess.Popen(command, cwd=str(self.project_root), env=env,
                    stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, encoding='utf-8', errors='replace', start_new_session=True)
                task['row']['pid'] = task['process'].pid
            except (OSError, ValueError) as error:
                task['row'].update(status='FAILED', error=str(error))
                self.policy.check()
                log_path.write_text('START_FAILED: ' + str(error) + '\n', encoding='utf-8')
                return self._copy(task)
            threading.Thread(target=self._read_output, args=(task,), daemon=True).start()
        if not task['ready'].wait(25):
            self._fail(task, '工具在25秒内未就绪；请查看日志')
        return self.status(identifier)

    def _fail(self, task, message):
        with self._lock:
            task['row'].update(status='FAILED', error=str(message), url='')
            task['ready'].set()
        self._terminate(task)

    def _read_output(self, task):
        process, row = task['process'], task['row']
        try:
            self.policy.check()
            with Path(row['log_path']).open('a', encoding='utf-8') as log:
                for line in process.stdout:
                    self.policy.check()
                    log.write(line); log.flush()
                    try:
                        record = json.loads(line)
                    except ValueError:
                        continue
                    if not isinstance(record, dict):
                        continue
                    if record.get('error'):
                        with self._lock:
                            row.update(status='FAILED', url='', error=str(record.get('detail') or record['error']))
                            task['ready'].set()
                    if record.get('url'):
                        expected = 'OFFLINE_ALIGNMENT_ONLY' if row['mode'] == 'alignment' else 'OFFLINE_SELECTION_ONLY'
                        if (record.get('status') != expected or process.poll() is not None
                                or not owns_loopback_listener(process.pid, record['url'], row['mode'])):
                            raise ValueError('工具报告的本机端口不属于本次启动进程，拒绝打开')
                        with self._lock:
                            if row['status'] == 'STARTING':
                                row.update(status='RUNNING', url=record['url'])
                            task['ready'].set()
            process.wait()
            self.status(row['id'])
        except (OSError, ValueError) as error:
            self._fail(task, str(error))
        finally:
            task['ready'].set()
            process.stdout.close()

    @staticmethod
    def _terminate(task):
        process = task.get('process')
        if process is None or process.poll() is not None:
            return
        try:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
        except ProcessLookupError:
            pass

    def stop(self, identifier):
        with self._lock:
            current = self.status(identifier)
            task = self._tasks[identifier]
            if current['status'] not in ACTIVE and not current['process_alive']:
                return current
            task['row']['status'] = 'STOPPING'
        try:
            self._terminate(task)
        except (OSError, subprocess.TimeoutExpired) as error:
            with self._lock:
                task['row'].update(status='FAILED', error='停止失败：' + str(error), url='')
        else:
            with self._lock:
                task['row'].update(status='COMPLETE', url='', exit_code=task['process'].poll())
        return self.status(identifier)

    def close(self):
        for identifier in list(self._tasks):
            try:
                self.stop(identifier)
            except (OSError, ValueError, subprocess.TimeoutExpired):
                pass


def guarded_picker(policy, parent_pid, input_path, mode):
    """Run the original picker with export guards for this owned child only."""
    from wc_calibration import picker
    from wc_runtime import calibration_picker
    original = picker.PickerSession

    class GuardedSession(original):
        def __init__(self, *args, **kwargs):
            policy.check()
            super().__init__(*args, **kwargs)

        def export(self, *args, **kwargs):
            policy.check()
            result = super().export(*args, **kwargs)
            policy.check()
            return result

        def alignment_workspace(self):
            policy.check()
            workspace = super().alignment_workspace()
            if not getattr(workspace, '_panel_storage_guarded', False):
                export = workspace.export

                def guarded_export(*args, **kwargs):
                    policy.check()
                    result = export(*args, **kwargs)
                    policy.check()
                    return result
                workspace.export = guarded_export
                workspace._panel_storage_guarded = True
            return workspace

    picker.PickerSession = GuardedSession
    finished = threading.Event()

    def supervise():
        while not finished.wait(.5):
            try:
                if os.getppid() != parent_pid:
                    raise ValueError('所属面板已退出，停止本次点云工具')
                policy.check()
            except (OSError, ValueError) as error:
                print(json.dumps(dict(error='SESSION_GUARD', detail=str(error))), file=sys.stderr, flush=True)
                os.kill(os.getpid(), signal.SIGINT)
                return

    policy.check()
    threading.Thread(target=supervise, daemon=True).start()
    try:
        args = ['--input', str(input_path), '--port', '0', '--no-browser', '--duration', '3600']
        if mode == 'alignment':
            args.append('--alignment')
        return calibration_picker.main(args)
    finally:
        finished.set()
        picker.PickerSession = original


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serve', action='store_true', required=True)
    parser.add_argument('--parent-pid', type=int, required=True)
    parser.add_argument('--input', required=True)
    parser.add_argument('--mode', choices=('points', 'alignment'), required=True)
    arguments = parser.parse_args()
    raise SystemExit(guarded_picker(StoragePolicy(os.environ['WHEELCHAIR_PROJECT_ROOT']),
                                   arguments.parent_pid, arguments.input, arguments.mode))
