"""Persistent preview window with separate, normally closed mapping sessions."""
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

from .mapping_app import prepare_request, normal_close, launch_rviz, close_rviz
from .mapping_interactive_control import WindowControl, atomic_document, read_document


class OwnedSave:
    """The existing descendant-owning wrapper also owns the save dialog/export."""
    def __init__(self, control, handle, job, *, gui_environment=None):
        self.control, self.handle, self.job = control, handle, str(job)
        self.cancelled = False
        self.result_path = control.directory/('save-'+self.job+'.result.json')
        atomic_document(control.directory/('save-'+self.job+'.json'), dict(
            control.owner, job=self.job, session_id=handle['session_id'], directory=handle['directory']))
        environment = dict(os.environ, PYTHONNOUSERSITE='1',
                           PYTHONPATH=str(Path(control.owner['project_root'])/'src'),
                           ROS_DOMAIN_ID='83', ROS_LOCALHOST_ONLY='1')
        environment.update(gui_environment or {})
        command = [sys.executable, '-s', '-m', 'wc_runtime.component', '--parent', str(os.getpid()),
                   '--sigint-grace-s', '120', '--', sys.executable, '-s', '-m',
                   'wc_runtime.mapping_interactive_worker', '--control-directory', str(control.directory),
                   '--job', self.job]
        with (control.directory/('save-'+self.job+'.log')).open('xb') as stream:
            self.process = subprocess.Popen(command, cwd=control.owner['project_root'], env=environment,
                                            stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT,
                                            start_new_session=True)

    def cancel(self):
        if not self.cancelled and self.process.poll() is None:
            self.cancelled = True
            try:
                self.process.send_signal(signal.SIGTERM)
            except ProcessLookupError:
                pass

    def is_running(self):
        return self.process.poll() is None

    def poll(self):
        code = self.process.poll()
        if code is None:
            return None
        if self.result_path.exists():
            value, _ = read_document(self.result_path)
            if (value.get('window_id') != self.control.owner['window_id'] or value.get('job') != self.job
                    or value.get('session_id') != self.handle['session_id']):
                raise ValueError('Save result identity mismatch')
            result = value.get('result')
            if not isinstance(result, dict) or result.get('session_id') != self.handle['session_id']:
                raise ValueError('Save result session mismatch')
            expected = {'SAVED': 0, 'DISCARDED': 0, 'SAVE_PENDING': 3, 'FAILED': 2}
            if result.get('status') not in expected:
                raise ValueError('Unknown save result')
            if not self.cancelled and code != expected[result['status']]:
                raise RuntimeError('Save exit status disagrees with its result')
            return result
        return dict(status='SAVE_PENDING', session_id=self.handle['session_id'],
                    session_output=self.handle['directory'], errors=[],
                    message='保存操作已取消或异常结束；保留现有数据，未声明保存成功。', worker_exit_code=code)


class InteractiveCoordinator:
    def __init__(self, request, backend, control, *, stop_requested=lambda: False,
                 emit=lambda value: None, spawn=launch_rviz, close_gui=close_rviz,
                 save_factory=OwnedSave, executor=None, clock=time.monotonic, wait=time.sleep):
        self.request, self.backend, self.control = request, backend, control
        self.stop_requested, self.emit, self.spawn, self.close_gui = stop_requested, emit, spawn, close_gui
        self.save_factory, self.clock, self.wait = save_factory, clock, wait
        self.executor = executor or ThreadPoolExecutor(max_workers=1, thread_name_prefix='mapping-lifecycle')
        self.own_executor = executor is None
        self.future = self.operation = self.active = self.gui = self.saver = None
        self.gui_started = None
        self.gui_retry_used = False
        self.pending_map = None
        self.closing = self.cancel_start = False
        self.errors, self.results = [], []
        self.sequence = self.save_sequence = 0
        self.last_storage_check = None
        self.desired_after_stop = None
        self.stop_attempts = {}
        self.unconfirmed = []
        self.failed_operation = None
        self.save_poll_error = None

    def state(self, state, message, *, start=False, stop=False, **values):
        self.control.publish(transition=True, state=state, message=message, can_start=start, can_stop=stop, **values)
        try:
            self.emit(dict(status=state, window_id=self.control.owner['window_id'], message=message))
        except (OSError, ValueError):
            pass

    def session_request(self, mapping):
        self.sequence += 1
        # Preserve the window's random suffix, including for long user names.
        prefix = self.control.owner['window_id'][:52]
        identifier = prefix+('_m' if mapping else '_p')+str(self.sequence).zfill(4)
        requested = self.request
        result = prepare_request(requested['mode'], output=Path(requested['output_dir']).parent/identifier,
                                 name=identifier, project_root=requested['project_root'],
                                 config=requested['config_path'], mapping_enabled=mapping,
                                 cloud_source=requested.get('cloud_source'),
                                 retention_profile=requested.get('retention_profile', 'experiment'))
        for key in ('panel_layout', 'estimator', 'motion_correction', 'process_noise', 'geometry'):
            if key in requested:
                result[key] = requested[key]
        if not mapping:
            # These options remain untouched in the requested mapping profile.
            # Pure preview performs none of the correction/estimator pipelines.
            result.update(motion_correction=False, geometry=False, process_noise='legacy')
        return result

    def begin_start(self, mapping):
        request = self.session_request(mapping)
        self.next_request = request
        self.operation = 'start_map' if mapping else 'start_preview'
        def work():
            self.backend.preflight(request)
            if self.closing or self.stop_requested() or (mapping and self.cancel_start):
                return {'cancelled_before_start': True}
            return self.backend.start(request)
        self.future = self.executor.submit(work)

    def begin_stop(self, after):
        if self.active is None:
            raise RuntimeError('No owned session to stop')
        self.desired_after_stop = after
        identity = self.active['session_id']
        if self.stop_attempts.get(identity, 0) >= 2:
            raise RuntimeError('Owned stop attempt budget exhausted')
        self.stop_attempts[identity] = self.stop_attempts.get(identity, 0)+1
        self.operation = 'stop_map' if self.active['mapping_enabled'] else 'stop_preview'
        self.future = self.executor.submit(self.backend.stop, self.active)

    def open_gui(self):
        gui_request = dict(self.request, output_dir=self.active['directory'])
        if self.gui_retry_used:
            gui_request['rviz_startup_attempt'] = 2
        environment = (self.backend.rviz_environment(self.active)
                       if callable(getattr(self.backend, 'rviz_environment', None)) else {})
        gui_request['rviz_environment'] = dict(environment, WC_MAPPING_CONTROL_DIR=str(self.control.directory))
        self.gui = self.spawn(self.backend.rviz_command(self.active), gui_request)
        self.gui_started = self.clock()

    def complete_operation(self):
        if self.future is None or not self.future.done():
            return
        future, operation = self.future, self.operation
        self.future = self.operation = None
        self.failed_operation = operation
        result = future.result()
        if operation.startswith('start_'):
            if result == {'cancelled_before_start': True}:
                self.failed_operation = None
                if self.stop_requested():
                    self.request_close()
                if not self.closing:
                    self.cancel_start = False
                    self.begin_start(False)
                    self.state('STOPPING', '已取消开始，正在恢复当前帧预览。')
                return
            if (not isinstance(result, dict) or result.get('session_id') != self.next_request['session_id']
                    or result.get('mapping_enabled') is not self.next_request['mapping_enabled']):
                raise RuntimeError('Started session identity mismatch')
            self.active = result
            self.failed_operation = None
            self.control.bind(result)
            if self.closing:
                self.begin_stop('close')
                return
            if operation == 'start_map' and self.cancel_start:
                self.state('STOPPING', '已取消开始，正在正常关闭刚启动的会话。')
                self.begin_stop('preview')
                return
            if self.gui is None:
                self.open_gui()
            if operation == 'start_map':
                self.state('MAPPING', '正在建图；再次点击停止并保存。', stop=True)
            elif self.pending_map is not None:
                self.start_save()
            else:
                self.state('PREVIEW', '当前帧实时预览；点击开始建立一份新地图。', start=True)
            return
        if not normal_close(result):
            raise RuntimeError('Owned session did not close normally: '+str(result))
        self.failed_operation = None
        stopped = self.active
        self.active = None
        self.control.bind(None)
        if operation == 'stop_map':
            self.pending_map = stopped
        else:
            discarded = self.backend.discard(stopped)
            if discarded.get('status') != 'SESSION_DATA_DISCARDED':
                raise RuntimeError('Stopped preview cleanup was not confirmed')
        if self.closing:
            self.control.publish(transition=True)
            return
        if self.desired_after_stop == 'map' and not self.cancel_start:
            self.begin_start(True)
            self.state('STARTING', '正在切换：启动新的建图会话。', stop=True)
        else:
            self.cancel_start = False
            self.begin_start(False)
            self.state('STOPPING', '正在切换：恢复当前帧实时预览。')

    def start_save(self):
        self.save_sequence += 1
        handle, self.pending_map = self.pending_map, None
        self.save_poll_error = None
        try:
            gui_environment = (self.backend.rviz_environment(handle)
                               if callable(getattr(self.backend, 'rviz_environment', None)) else {})
            self.saver = self.save_factory(self.control, handle, self.save_sequence,
                                           gui_environment=gui_environment)
        except BaseException:
            self.pending_map = handle
            raise
        self.state('SAVING', '已恢复当前帧预览，正在处理上一份地图的保存选择。',
                   save=dict(state='RUNNING', session_id=handle['session_id'], result=None))

    def poll_save(self):
        if self.saver is None:
            return
        if self.closing:
            self.saver.cancel()
        try:
            result = self.saver.poll()
        except Exception as error:
            if self.save_poll_error is None:
                self.save_poll_error = str(error)
                self.request_close('保存进程结果未通过核验: '+str(error))
            # Real OwnedSave only reads the result after wrapper exit. If a
            # transport implementation fails while alive, retain/cancel it but
            # let the main loop stop the preview instead of repeating a throw.
            if self.saver.is_running():
                self.saver.cancel()
                return
            result = dict(status='FAILED', session_id=self.saver.handle['session_id'],
                          session_output=self.saver.handle['directory'], errors=[self.save_poll_error],
                          message='保存进程结果无法核验；保留原件及现有保存目录，不重试删除。')
        if result is None:
            return
        self.results.append(result)
        self.saver = None
        self.control.publish(save=dict(state=result['status'], session_id=result['session_id'], result=result))
        if not self.closing:
            message = str(result.get('message', result['status']))
            if result['status'] == 'SAVED' and result.get('cleanup_errors'):
                message = '地图已保存；收尾警告：'+str(result['cleanup_errors'])
            self.state('PREVIEW', message+' 当前为实时预览。', start=True)

    def command(self, action):
        if action == 'start':
            self.cancel_start = False
            self.state('STARTING', '正在切换：停止预览并准备新地图。', stop=True)
            self.begin_stop('map')
        elif action == 'stop':
            if self.control.status['state'] == 'STARTING':
                self.cancel_start = True
                self.state('STARTING', '正在取消开始，等待当前设备切换正常收尾。')
            else:
                self.state('STOPPING', '正在停止建图并关闭数据库，随后恢复实时预览。')
                self.begin_stop('preview')

    def request_close(self, error=None):
        if error:
            self.errors.append(str(error))
        if not self.closing:
            self.closing = True
            self.state('CLOSING', '正在停止本窗口的设备与保存进程；未完成地图将保留。')
        if self.saver is not None:
            self.saver.cancel()

    def operation_failed(self, error):
        self.request_close(error)
        if (self.failed_operation or '').startswith('stop_') and self.active is not None:
            if self.stop_attempts.get(self.active['session_id'], 0) >= 2:
                # Never report a successful stop, discard, or start a replacement
                # after an unconfirmed close. Preserve exact ownership evidence;
                # the existing owner-watch also observes coordinator death.
                self.unconfirmed.append(dict(self.active, stop_confirmed=False))
                if self.active['mapping_enabled']:
                    self.pending_map = self.active
                self.active = None
                self.control.bind(None)
        self.failed_operation = None

    def inspect_active(self):
        if self.active is None or self.future is not None or self.closing:
            return
        observed = self.backend.inspect(self.active)
        if observed.get('state') != 'RUNNING':
            raise RuntimeError('Owned session exited unexpectedly: '+str(observed))
        check = getattr(self.backend, 'storage_status', None)
        now = self.clock()
        if callable(check) and (self.last_storage_check is None or now-self.last_storage_check >= 1):
            value = check(self.active)
            self.last_storage_check = now
            if (type(value.get('free_bytes')) is not int or type(value.get('stop_free_bytes')) is not int
                    or value['stop_free_bytes'] <= 0 or value['free_bytes'] <= value['stop_free_bytes']):
                raise RuntimeError('Storage unavailable or below finalization threshold; data retained')

    def run(self):
        if self.stop_requested():
            result = dict(status='CANCELLED_BEFORE_START', window_id=self.control.owner['window_id'],
                          hardware_started=False, errors=[])
            self.control.publish(transition=True,state='CLOSED',message='已在设备启动前取消。',result=result)
            if self.own_executor:
                self.executor.shutdown(wait=True)
            return result, 130
        try:
            self.begin_start(False)
            while True:
                try:
                    if self.stop_requested():
                        self.request_close()
                    if self.gui is not None and self.gui.poll() is not None and not self.closing:
                        code = self.gui.poll()
                        retry = getattr(self.backend, 'rviz_startup_retry', None)
                        evidence = (retry(self.active, code, self.clock()-self.gui_started)
                                    if code and self.active is not None and not self.gui_retry_used
                                    and self.future is None and callable(retry) else None)
                        if evidence:
                            self.gui_retry_used = True
                            self.open_gui()
                        else:
                            self.request_close(None if code == 0 else 'RViz exited with code '+str(code))
                    # Consume an already-written command against the published
                    # generation before a finished startup advances that state.
                    # Otherwise a valid cancel click can be lost at completion.
                    if not self.closing:
                        action = self.control.receive()
                        if action:
                            self.command(action)
                    self.complete_operation()
                    self.poll_save()
                    if self.closing:
                        if self.future is None and self.active is not None:
                            self.begin_stop('close')
                        if self.future is None and self.active is None and self.saver is None:
                            break
                    else:
                        self.inspect_active()
                except Exception as error:
                    self.operation_failed(error)
                self.wait(.1)
        except KeyboardInterrupt:
            self.request_close()
        finally:
            self.request_close()
            # Bounded backend start/stop must finish before ownership exits.
            # Owner-watch also stops sessions if this process dies unexpectedly.
            while self.future is not None or self.active is not None:
                try:
                    if self.future is None:
                        self.begin_stop('close')
                    # Complete exactly one lifecycle operation at a time. A
                    # start completion can enqueue stop; settle that next.
                    self.future.result()
                    self.complete_operation()
                except BaseException as error:
                    # result() may raise before complete_operation consumes it.
                    if self.future is not None:
                        self.failed_operation = self.operation
                        self.future = self.operation = None
                    self.operation_failed(error)
            if self.saver is not None:
                self.saver.cancel()
                # The wrapper's own descendant cleanup has a bounded budget;
                # do not kill the owner wrapper and orphan a dialog/export.
                while self.saver is not None:
                    try:
                        self.poll_save()
                    except Exception as error:
                        self.errors.append(str(error))
                        self.saver = None
                    if self.saver is not None:
                        self.wait(.1)
            try:
                self.close_gui(self.gui)
            except BaseException as error:
                self.errors.append(str(error))
            if self.own_executor:
                self.executor.shutdown(wait=True)
        pending_sessions = {}
        for saved in self.results:
            if saved['status'] in ('SAVE_PENDING', 'FAILED'):
                pending_sessions[saved['session_id']] = dict(
                    session_id=saved['session_id'], directory=saved.get('session_output', saved.get('output')),
                    status=saved['status'])
            if saved['status'] == 'FAILED':
                self.errors.append('地图收尾失败 '+saved['session_id']+': '+str(saved.get('errors', [])))
        pending = self.pending_map
        if pending is not None:
            pending_sessions[pending['session_id']] = dict(session_id=pending['session_id'],
                                                          directory=pending['directory'], status='SAVE_PENDING')
        status = 'FAILED' if self.errors else 'SAVE_PENDING' if pending_sessions else 'INTERACTIVE_COMPLETE'
        result = dict(status=status,
                      window_id=self.control.owner['window_id'], errors=self.errors,
                      unconfirmed_owned_sessions=self.unconfirmed,
                      saves=self.results, pending_sessions=list(pending_sessions.values()),
                      cleanup_errors=[dict(session_id=row['session_id'], errors=row['cleanup_errors'])
                                      for row in self.results if row.get('cleanup_errors')],
                      message='本窗口已关闭；未完成的地图保留，未自动丢弃。')
        self.control.bind(None)
        self.control.publish(transition=True, state='FAILED' if self.errors else 'CLOSED',
                             can_start=False, can_stop=False, message=result['message'], result=result)
        return result, 2 if self.errors else 3 if pending_sessions else 0


def run_interactive(request, backend, **options):
    if request.get('mapping_enabled') is not True:
        raise ValueError('--interactive requires --mapping true')
    # Validate requested mapping capabilities before opening even a preview.
    backend.preflight(request)
    if options.get('stop_requested', lambda: False)():
        return dict(status='CANCELLED_BEFORE_START', session_id=request['session_id'],
                    hardware_started=False, errors=[]), 130
    identifier = request['session_id'][:42]+'_w'+uuid.uuid4().hex[:8]
    control = WindowControl(request['project_root'], identifier)
    return InteractiveCoordinator(request, backend, control, **options).run()
