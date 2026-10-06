"""Hardware-free camera contract and owned-process failure regression tests."""
import copy
import json
import multiprocessing as mp
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, Mock, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from camera_fixture import blocked_worker, frame_worker
from wc_cameras.capture import CaptureProcess, SharedFrame, capture_worker, configure_capture, fourcc_name, process_observation, verify_properties
from wc_cameras.config import CameraError, ROLES, load_config, port_path, topic, unique_object, validate_config
from wc_cameras.node import FrameTracker, camera_description, check_capture_events, diagnostic_level_number, exit_if_capture_unreaped, fill_image, finish_capture
from check_ros_camera_images import diagnostic_identity_matches


class FakeCv:
    CAP_PROP_FOURCC, CAP_PROP_FRAME_WIDTH, CAP_PROP_FRAME_HEIGHT, CAP_PROP_FPS = range(4)

    @staticmethod
    def VideoWriter_fourcc(*letters):
        return sum(ord(letter) << (8*i) for i, letter in enumerate(letters))


class FakeCapture:
    def __init__(self, *, override=None, setters=True):
        self.values = {}
        self.override, self.setters = override or {}, setters
        self.requests = []

    def set(self, key, value):
        self.requests.append((key, value))
        self.values[key] = value
        return self.setters

    def get(self, key):
        return self.override.get(key, self.values[key])

    def getBackendName(self):
        return 'SYNTHETIC_NO_DEVICE'


class CameraContractTests(unittest.TestCase):
    def setUp(self):
        self.config, self.config_hash = load_config(ROOT / 'config/cameras.json')
        self.profile = self.config['profiles']['monitor_320']

    def test_four_ports_match_user_confirmed_directions_without_upgrading_rotation(self):
        self.assertEqual(self.config['mapping_status'], 'USER_CONFIRMED')
        self.assertEqual(self.config['rotation_status'], 'CONFIG_ONLY')
        self.assertEqual(len(self.config_hash), 64)
        self.assertEqual([c['role'] for c in self.config['cameras']], list(ROLES))
        self.assertEqual([c['port'] for c in self.config['cameras']], [1, 4, 2, 3])
        # 2026-10-05 saved-frame review corrected the inherited upside-down
        # preview to 0 degrees; this remains CONFIG_ONLY, not calibration.
        self.assertEqual([c['rotate_deg'] for c in self.config['cameras']], [0]*4)
        self.assertIn('2026-09-12 user message', self.config['mapping_note'])
        self.assertIn('Left-front remains USB1', self.config['mapping_note'])

    def test_direction_confirmation_only_changes_port_identity_and_description(self):
        expected_ports = {'left_front': 1, 'right_front': 4, 'left_side': 2, 'right_side': 3}
        for camera in self.config['cameras']:
            role = camera['role']
            with self.subTest(role=role):
                description = camera_description(self.config, self.config_hash, camera, 'monitor_320')
                self.assertEqual(description['topic'], '/wc_mapping/cameras/'+role+'/image_raw')
                self.assertEqual(description['frame_id'], 'camera_usb3_'+str(expected_ports[role]))
                self.assertEqual(description['slot_label'], 'USB3.'+str(expected_ports[role]))
                self.assertEqual(description['physical_direction'], role)
                self.assertEqual(description['mapping_status'], 'USER_CONFIRMED')
                self.assertEqual(description['mapping_note'], self.config['mapping_note'])
                self.assertEqual(description['rotation_status'], 'CONFIG_ONLY')
                self.assertEqual(description['camera_info'], 'UNAVAILABLE')
                self.assertEqual(description['time_source'], 'arrival_only')
                self.assertFalse(description['common_time_valid'])
                self.assertFalse(description['tf_published'])

    def test_unconfirmed_config_remains_supported_without_claiming_direction(self):
        value = copy.deepcopy(self.config)
        value['mapping_status'] = 'UNCONFIRMED'
        value['mapping_note'] = 'Synthetic preview assignment without user confirmation.'
        validate_config(value)
        for camera in value['cameras']:
            description = camera_description(value, 'synthetic-config-hash', camera, 'monitor_320')
            self.assertEqual(description['physical_direction'], 'UNKNOWN')
        for note in ('', '  ', None):
            value['mapping_note'] = note
            with self.assertRaises(CameraError):
                validate_config(value)

    def test_live_observer_checks_new_port_identity_and_preserves_calibration_limits(self):
        camera = next(row for row in self.config['cameras'] if row['role'] == 'right_front')
        description = camera_description(self.config, self.config_hash, camera, 'monitor_320')
        self.assertTrue(diagnostic_identity_matches(description, self.config, self.config_hash, camera))
        cases = {'frame_id': 'camera_usb3_2', 'physical_direction': 'left_side',
                 'mapping_status': 'UNCONFIRMED', 'mapping_note': 'different provenance',
                 'common_time_valid': True, 'tf_published': True, 'rotation_status': 'VERIFIED'}
        for key, value in cases.items():
            with self.subTest(key=key):
                self.assertFalse(diagnostic_identity_matches({**description, key: value}, self.config, self.config_hash, camera))

    def test_rejects_duplicate_port_or_role(self):
        for key in ('port', 'role'):
            config = copy.deepcopy(self.config)
            config['cameras'][1][key] = config['cameras'][0][key]
            with self.assertRaises(CameraError):
                validate_config(config)

    def test_rejects_unsafe_or_misleading_configuration(self):
        cases = [('width', 1922), ('height', 239), ('capture_fps', float('nan')),
                 ('publish_hz', 31), ('width', True), ('fourcc', 'YUYV')]
        for field, value in cases:
            with self.subTest(field=field, value=value):
                config = copy.deepcopy(self.config)
                config['profiles']['monitor_320'][field] = value
                with self.assertRaises(CameraError):
                    validate_config(config)
        config = copy.deepcopy(self.config)
        config['mapping_status'] = 'VERIFIED'
        with self.assertRaises(CameraError):
            validate_config(config)
        config = copy.deepcopy(self.config)
        config['cameras'][0]['serial'] = 'different-camera'
        with self.assertRaises(CameraError):
            validate_config(config)

    def test_unknown_json_keys_and_duplicates_rejected(self):
        with self.assertRaises(CameraError):
            json.loads('{"width":320,"width":640}', object_pairs_hook=unique_object)
        self.config['silent_fallback'] = True
        with self.assertRaises(CameraError):
            validate_config(self.config)

    def test_stale_threshold_precedes_kill_deadline(self):
        self.config['timeouts']['stale_s'] = self.config['timeouts']['read_s']
        with self.assertRaises(CameraError):
            validate_config(self.config)

    def test_topic_and_port_do_not_use_ambiguous_video_indices(self):
        self.assertEqual(topic('left_front'), '/wc_mapping/cameras/left_front/image_raw')
        self.assertEqual(str(port_path(4)).replace('\\', '/'), '/dev/v4l/by-path/platform-3610000.usb-usb-0:3.4:1.0-video-index0')
        with self.assertRaises(CameraError):
            port_path(0)
        with self.assertRaises(CameraError):
            topic('front')

    def test_identical_serial_does_not_hide_wrong_physical_port(self):
        camera = self.config['cameras'][0]
        props = {'ID_VENDOR_ID': '0bda', 'ID_MODEL_ID': '5858', 'ID_SERIAL_SHORT': '000000000011',
                 'ID_PATH': 'platform-3610000.usb-usb-0:3.1:1.0', 'ID_V4L_CAPABILITIES': ':capture:'}
        self.assertEqual(verify_properties(camera, props)['ID_PATH'], props['ID_PATH'])
        for key, value in (('ID_PATH', 'platform-3610000.usb-usb-0:3.2:1.0'),
                           ('ID_MODEL_ID', '0000'), ('ID_SERIAL_SHORT', 'another'), ('ID_V4L_CAPABILITIES', ':metadata:')):
            with self.subTest(key=key):
                wrong = {**props, key: value}
                with self.assertRaises(CameraError):
                    verify_properties(camera, wrong)

    def test_capture_request_sequence_and_actual_readback(self):
        capture = FakeCapture()
        actual = configure_capture(capture, FakeCv, self.profile)
        self.assertEqual([key for key, _ in capture.requests], [0, 1, 2, 3])
        self.assertEqual(actual['fourcc'], 'MJPG')
        self.assertTrue(actual['fps_matches_request'])

    def test_no_format_or_size_fallback(self):
        for override in ({1: 640.0}, {2: 480.0}, {0: float(FakeCv.VideoWriter_fourcc(*'YUYV'))}, {3: float('nan')}):
            capture = FakeCapture(override=override)
            with self.assertRaises(CameraError):
                configure_capture(capture, FakeCv, self.profile)
            self.assertEqual(len(capture.requests), 4)  # One requested profile, never retries smaller settings.

    def test_negotiated_fps_separate_from_requested(self):
        actual = configure_capture(FakeCapture(override={3: 15.0}), FakeCv, self.profile)
        self.assertEqual(actual['capture_fps'], 15)
        self.assertFalse(actual['fps_matches_request'])
        self.assertEqual(self.profile['capture_fps'], 30)

    def test_false_setter_with_matching_readback_is_reported(self):
        actual = configure_capture(FakeCapture(setters=False), FakeCv, self.profile)
        self.assertFalse(any(actual['setters_returned'].values()))
        self.assertEqual(actual['width'], 320)

    def test_fourcc_validation(self):
        self.assertEqual(fourcc_name(FakeCv.VideoWriter_fourcc(*'MJPG')), 'MJPG')
        with self.assertRaises(CameraError):
            fourcc_name(float('nan'))

    def test_startup_stale_read_timeout_and_no_fake_refresh(self):
        tracker = FrameTracker(1_000_000_000, self.config['timeouts'])
        self.assertEqual(tracker.health(2_000_000_000)[0], 'STARTING')
        self.assertEqual(tracker.health(17_000_000_000)[0], 'ERROR')
        tracker.ready_ns = 2_000_000_000
        self.assertEqual(tracker.health(3_100_000_000)[0], 'STALE')
        self.assertEqual(tracker.health(5_100_000_000)[0], 'ERROR')
        frame = sample_frame(sequence=1, mono=6_000_000_000)
        tracker.accept(frame)
        self.assertEqual(tracker.health(6_100_000_000)[0], 'ACTIVE')
        self.assertEqual(tracker.health(9_100_000_000)[0], 'ERROR')
        with self.assertRaises(CameraError):
            tracker.accept(frame)

    def test_latest_frame_skip_accounting_not_device_loss(self):
        tracker = FrameTracker(1, self.config['timeouts'])
        first = tracker.published(sample_frame(sequence=3, mono=10), 20)
        second = tracker.published(sample_frame(sequence=8, mono=30), 50)
        self.assertEqual(first['skipped_capture_frames_since_previous_image'], 2)
        self.assertEqual(second['skipped_capture_frames_total'], 6)
        self.assertEqual(second['host_arrival_to_publish_elapsed_ns'], 20)
        self.assertEqual(second['device_dropped_frames'], 'UNKNOWN')
        self.assertEqual(second['measurement_latency_ns'], 'UNKNOWN')
        with self.assertRaises(CameraError):
            tracker.published(sample_frame(sequence=8, mono=30), 60)

    def test_image_preserves_arrival_stamp_and_port_frame(self):
        frame = sample_frame()
        message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace()))
        fill_image(message, frame, 'camera_usb3_2')
        self.assertEqual((message.header.stamp.sec, message.header.stamp.nanosec), (1700000000, 123))
        self.assertEqual(message.header.frame_id, 'camera_usb3_2')
        self.assertEqual((message.encoding, message.width, message.height, message.step), ('bgr8', 2, 1, 6))
        self.assertEqual(message.data, b'abcdef')
        frame['data'] = b'bad'
        with self.assertRaises(CameraError):
            fill_image(message, frame, 'camera_usb3_2')

    def test_check_config_cli_without_ros_or_camera_access(self):
        result = subprocess.run([sys.executable, '-s', '-c',
            "import sys;sys.path.insert(0,sys.argv.pop(1));from wc_cameras.node import main;raise SystemExit(main())",
            str(ROOT / 'src'), '--config', str(ROOT / 'config/cameras.json'), '--role', 'right_side',
            '--session-id', 'fixture-no-hardware', '--run-root', str(ROOT / '.phase1_runtime'), '--check-config'],
            text=True, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        row = json.loads(result.stdout)
        self.assertEqual(row['frame_id'], 'camera_usb3_3')
        self.assertEqual(row['mapping_status'], 'USER_CONFIRMED')
        self.assertEqual(row['physical_direction'], 'right_side')
        self.assertFalse(row['common_time_valid'])

    def test_final_pending_child_error_is_never_accepted_as_session_end(self):
        for alive, code in ((True, None), (False, 0), (False, 7)):
            with self.assertRaisesRegex(CameraError, 'late read failure'):
                check_capture_events([{'event': 'ERROR', 'error': 'late read failure'}], alive, code)
        with self.assertRaisesRegex(CameraError, 'exited unexpectedly'):
            check_capture_events([], False, 0)
        check_capture_events([{'event': 'READY'}], True, None)

    def test_cleanup_result_latches_failure_before_main_return_value(self):
        report = {'status': 'FAIL', 'termination': 'terminate', 'final_exit_code': -15,
                  'errors': ['forced_terminate']}
        node = SimpleNamespace(capture=SimpleNamespace(close=lambda: report), failed=False, latest_error='')
        self.assertEqual(finish_capture(node), report)
        self.assertTrue(node.failed)
        self.assertIn('forced_terminate', node.latest_error)
        node = SimpleNamespace(capture=SimpleNamespace(close=lambda: {'status': 'PASS', 'final_exit_code': 0}),
                               failed=True, latest_error='earlier capture failure')
        finish_capture(node)
        self.assertTrue(node.failed)  # Normal cleanup cannot clear an earlier failure.

    def test_diagnostic_ros_byte_converts_to_json_integer(self):
        for value, expected in ((b'\x01', 1), (b'\x02', 2), (bytearray(b'\x03'), 3), (1, 1)):
            self.assertEqual(json.loads(json.dumps({'level': diagnostic_level_number(value)})), {'level': expected})
        for value in (b'', b'\x01\x02', -1, 256, True, 'WARN'):
            with self.assertRaises(CameraError):
                diagnostic_level_number(value)

    def test_immediate_failure_exit_only_for_explicitly_unreaped_child(self):
        force_exit = Mock()
        for result in (None, {'status': 'PASS', 'alive_after_cleanup': False},
                       {'status': 'FAIL', 'termination': 'kill', 'alive_after_cleanup': False}):
            node = SimpleNamespace(capture=SimpleNamespace(cleanup_result=result))
            exit_if_capture_unreaped(node, immediate_exit=force_exit)
        exit_if_capture_unreaped(None, immediate_exit=force_exit)
        force_exit.assert_not_called()
        result = {'status': 'FAIL', 'termination': 'kill', 'final_exit_code': None,
                  'alive_after_cleanup': True, 'owned_pid': 2345}
        node = SimpleNamespace(capture=SimpleNamespace(cleanup_result=result), failed=False, latest_error='')
        node.capture.close = Mock(side_effect=CameraError('owned camera child cleanup did not complete'))
        self.assertEqual(finish_capture(node)['status'], 'FAIL')
        self.assertTrue(node.failed)
        self.assertIn('2345', node.latest_error)
        exit_if_capture_unreaped(node, immediate_exit=force_exit)
        force_exit.assert_called_once_with(1)

    def test_unreaped_failure_bypasses_unbounded_interpreter_exit_hook(self):
        # A synthetic blocking exit hook exercises the actual os._exit path,
        # without creating an orphan process or touching a camera/ROS device.
        program = '''import atexit, json, sys, time
from types import SimpleNamespace
sys.path.insert(0, sys.argv[1])
from wc_cameras.node import exit_if_capture_unreaped
def blocked_finalizer():
    print("UNEXPECTED_EXIT_HOOK", flush=True)
    time.sleep(30)
atexit.register(blocked_finalizer)
report = {"status": "FAIL", "alive_after_cleanup": True, "owned_pid": 2345}
print(json.dumps(report), flush=True)
exit_if_capture_unreaped(SimpleNamespace(capture=SimpleNamespace(cleanup_result=report)))
'''
        result = subprocess.run([sys.executable, '-s', '-c', program, str(ROOT/'src')],
                                capture_output=True, text=True, timeout=8)
        self.assertEqual(result.returncode, 1)
        self.assertNotIn('UNEXPECTED_EXIT_HOOK', result.stdout)
        self.assertEqual(json.loads(result.stdout)['owned_pid'], 2345)


def sample_frame(sequence=1, mono=100):
    return {'capture_sequence': sequence, 'host_arrival_ns': 1700000000000000123,
            'host_monotonic_ns': mono, 'host_read_elapsed_ns': 7,
            'width': 2, 'height': 1, 'data': b'abcdef'}


def late_error_worker(camera, profile, run_root, session_id, shared, stop, status, parent_pid):
    status.send({'event': 'READY', 'fixture': 'LATE_ERROR_NO_DEVICE'})
    stop.wait(10)
    status.send({'event': 'ERROR', 'error': 'release failed after last ROS tick'})
    status.close()  # Even exit zero must not erase the ERROR event.


def nonzero_worker(camera, profile, run_root, session_id, shared, stop, status, parent_pid):
    status.send({'event': 'READY', 'fixture': 'NONZERO_NO_DEVICE'})
    stop.wait(10)
    status.close()
    raise SystemExit(7)


def slow_release_worker(camera, profile, run_root, session_id, shared, stop, status, parent_pid):
    status.send({'event': 'READY', 'fixture': 'SLOW_RELEASE_NO_DEVICE'})
    stop.wait(10)
    begin = time.monotonic_ns()
    time.sleep(.8)  # Longer than the former .5 s shutdown allowance.
    complete = time.monotonic_ns()
    status.send({'event': 'RELEASE_COMPLETE', 'host_release_start_monotonic_ns': begin,
                 'host_release_complete_monotonic_ns': complete, 'host_release_elapsed_ns': complete-begin})
    status.close()


class SharedCaptureTests(unittest.TestCase):
    def test_shared_frame_is_bounded_and_preserves_metadata(self):
        shared = SharedFrame(mp.get_context('spawn'), 2, 1)
        shared.put(b'abcdef', 9, 10, 11, 12, 2, 1)
        row = shared.snapshot(8)
        self.assertEqual(row['data'], b'abcdef')
        self.assertEqual(row['capture_sequence'], 9)
        self.assertEqual(row['host_monotonic_ns'], 11)
        self.assertIsNone(shared.snapshot(9))
        with self.assertRaises(CameraError):
            shared.put(b'abcdefghijkl', 10, 10, 11, 12, 4, 1)

    def test_abandoned_frame_mutex_does_not_hang_parent(self):
        shared = SharedFrame(mp.get_context('spawn'), 2, 1)
        shared.lock.acquire()
        begin = time.monotonic()
        try:
            self.assertIsNone(shared.snapshot(0))
            self.assertLess(time.monotonic()-begin, .5)
        finally:
            shared.lock.release()

    def process_fixture(self, worker):
        return CaptureProcess({'port': 1}, {'width': 2, 'height': 1}, ROOT, 'fixture-no-device', worker=worker)

    def wait_for_event(self, process):
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            events = process.events()
            if events:
                return events
            if not process.process.is_alive():
                self.fail('synthetic child exited unexpectedly')
            time.sleep(.01)
        self.fail('synthetic child startup timed out')

    def test_blocked_owned_capture_child_is_terminated(self):
        process = self.process_fixture(blocked_worker)
        process.start()
        try:
            self.assertEqual(self.wait_for_event(process)[0]['fixture'], 'BLOCKED_SYNTHETIC_NO_DEVICE')
        finally:
            begin = time.monotonic()
            cleanup = process.close()
        self.assertGreaterEqual(time.monotonic()-begin, 5)
        self.assertLess(time.monotonic()-begin, 8)  # Explicit 5+1+1 s budget plus scheduling margin.
        self.assertFalse(process.started)
        self.assertEqual(cleanup['status'], 'FAIL')
        self.assertEqual(cleanup['termination'], 'terminate')
        self.assertNotEqual(cleanup['final_exit_code'], 0)
        self.assertFalse(cleanup['alive_after_cleanup'])
        self.assertGreaterEqual(cleanup['stop_to_reap_observed_elapsed_ns'], 5_000_000_000)

    def test_synthetic_frame_crosses_process_and_graceful_cleanup(self):
        process = self.process_fixture(frame_worker)
        process.start()
        try:
            self.wait_for_event(process)
            deadline = time.monotonic()+3
            frame = None
            while frame is None and time.monotonic() < deadline:
                frame = process.shared.snapshot(0)
                time.sleep(.01)
            self.assertIsNotNone(frame)
            self.assertEqual(frame['data'], bytes([19, 31, 47])*2)
            self.assertEqual(frame['capture_sequence'], 1)
        finally:
            cleanup = process.close()
        self.assertEqual(cleanup['status'], 'PASS')
        self.assertEqual(cleanup['termination'], 'normal')
        self.assertEqual(cleanup['final_exit_code'], 0)
        self.assertEqual(process.close(), cleanup)  # Idempotent after a completed close.

    def test_release_longer_than_former_half_second_budget_can_finish_normally(self):
        process = self.process_fixture(slow_release_worker)
        process.start()
        try:
            self.wait_for_event(process)
        finally:
            cleanup = process.close()
        self.assertEqual(cleanup['status'], 'PASS')
        self.assertEqual(cleanup['termination'], 'normal')
        self.assertEqual(cleanup['final_exit_code'], 0)
        self.assertEqual(cleanup['normal_join_budget_s'], 5)
        self.assertEqual(cleanup['process_observations'], [])
        self.assertGreater(cleanup['stop_to_reap_observed_elapsed_ns'], 500_000_000)
        self.assertEqual(cleanup['stop_to_reap_observed_elapsed_ns'],
                         cleanup['reap_observed_host_monotonic_ns']-cleanup['stop_requested_host_monotonic_ns'])
        released = [row for row in cleanup['late_events'] if row.get('event') == 'RELEASE_COMPLETE']
        self.assertEqual(len(released), 1)
        self.assertGreater(released[0]['host_release_elapsed_ns'], 500_000_000)

    def test_late_error_during_close_is_recorded_even_with_zero_exit(self):
        process = self.process_fixture(late_error_worker)
        process.start()
        try:
            self.wait_for_event(process)
        finally:
            result = process.close()
        self.assertEqual(result['final_exit_code'], 0)
        self.assertEqual(result['termination'], 'normal')
        self.assertEqual(result['status'], 'FAIL')
        self.assertIn('release failed after last ROS tick', result['errors'])

    def test_nonzero_exit_during_close_is_not_a_pass_without_error_event(self):
        process = self.process_fixture(nonzero_worker)
        process.start()
        try:
            self.wait_for_event(process)
        finally:
            result = process.close()
        self.assertEqual(result['final_exit_code'], 7)
        self.assertEqual(result['status'], 'FAIL')
        self.assertIn('capture_child_exit_code=7', result['errors'])

    def test_kill_escalation_remains_failure_and_records_final_exit(self):
        capture = CaptureProcess.__new__(CaptureProcess)
        capture.normal_close_budget_s = 5
        capture.started, capture.cleanup_result = True, None
        capture.stop_event, capture.sender, capture.receiver = Mock(), Mock(), Mock()
        capture.process = Mock()
        capture.process.pid = 2345
        capture.process.is_alive.side_effect = [True, True, False]
        capture.process.exitcode = -9
        capture.events = Mock(return_value=[])
        result = capture.close()
        self.assertEqual(result['termination'], 'kill')
        self.assertEqual(result['final_exit_code'], -9)
        self.assertEqual(result['status'], 'FAIL')
        capture.process.terminate.assert_called_once()
        capture.process.kill.assert_called_once()

    def test_unreaped_child_preserves_failure_and_skips_potential_blocking_pipe_drain(self):
        capture = CaptureProcess.__new__(CaptureProcess)
        capture.normal_close_budget_s = 5
        capture.started, capture.cleanup_result = True, None
        capture.stop_event, capture.sender, capture.receiver = Mock(), Mock(), Mock()
        capture.process = Mock()
        capture.process.pid = 2345
        capture.process.is_alive.return_value = True
        capture.process.exitcode = None
        capture.events = Mock(side_effect=AssertionError('must not recv from an unreaped sender'))
        with patch('wc_cameras.capture.process_observation', side_effect=lambda pid, stage: {'pid': pid, 'stage': stage}) as observe:
            with self.assertRaisesRegex(CameraError, 'cleanup did not complete'):
                capture.close()
        report = capture.cleanup_result
        self.assertEqual(report['status'], 'FAIL')
        self.assertEqual(report['termination'], 'kill')
        self.assertEqual(report['owned_pid'], 2345)
        self.assertIsNone(report['final_exit_code'])
        self.assertTrue(report['alive_after_cleanup'])
        self.assertEqual(report['late_event_drain'], 'SKIPPED_UNREAPED_CHILD')
        self.assertEqual([row['stage'] for row in report['process_observations']],
                         ['before_terminate', 'before_kill', 'after_kill_still_alive'])
        self.assertEqual(observe.call_count, 3)
        self.assertEqual([call.args for call in capture.process.join.call_args_list], [(5,), (1,), (1,)])
        self.assertIsNone(report['reap_observed_host_monotonic_ns'])
        self.assertIsNone(report['stop_to_reap_observed_elapsed_ns'])
        self.assertGreaterEqual(report['stop_to_cleanup_observed_elapsed_ns'], 0)
        capture.events.assert_not_called()
        capture.process.close.assert_not_called()  # Keep ownership evidence; never invent exit success.

    def test_proc_observation_records_wait_state_without_cmdline_or_unbounded_files(self):
        with tempfile.TemporaryDirectory(prefix='proc-fixture-', dir=ROOT/'tests/cameras') as temporary:
            proc_root = Path(temporary)
            folder = proc_root/'2345'
            folder.mkdir()
            (folder/'status').write_text('Name:\tcamera-test\nState:\tD (disk sleep)\nPid:\t2345\nPPid:\t1234\nSigPnd:\t0000000000000100\nIgnoredUnrelated:\tprivate\n', encoding='utf-8')
            (folder/'wchan').write_text('synthetic_v4l_wait\n', encoding='utf-8')
            row = process_observation(2345, 'after_kill_still_alive', proc_root=proc_root)
            self.assertEqual(row['status_fields']['State'], 'D (disk sleep)')
            self.assertEqual(row['status_fields']['SigPnd'], '0000000000000100')
            self.assertNotIn('IgnoredUnrelated', row['status_fields'])
            self.assertEqual(row['wchan'], 'synthetic_v4l_wait')
            self.assertEqual(row['read_errors'], [])
            self.assertGreater(row['host_monotonic_ns'], 0)
            self.assertGreater(row['host_wall_ns'], 0)
            (folder/'wchan').write_text('x'*1025, encoding='utf-8')
            (folder/'status').unlink()
            limited = process_observation(2345, 'after_kill_still_alive', proc_root=proc_root)
            self.assertEqual(len(limited['read_errors']), 2)
            self.assertIsNone(limited['wchan'])
            self.assertEqual(limited['status_fields'], {})


class WorkerReleaseTests(unittest.TestCase):
    def run_worker(self, *, fail_read=False, fail_release=False):
        camera = {'rotate_deg': 0}
        profile = {'width': 2, 'height': 1}
        identity = {'resolved_node': '/SYNTHETIC_NO_DEVICE'}
        lease = MagicMock()
        lease.__enter__.return_value = identity
        lease.__exit__.return_value = False
        cap, status, stop = Mock(), Mock(), Mock()
        cap.isOpened.return_value = True
        cap.read.return_value = (False, None)
        stop.is_set.return_value = not fail_read

        def release():
            lease.__exit__.assert_not_called()  # The physical-port lease still covers release.
            if fail_release:
                raise RuntimeError('synthetic release failure')
        cap.release.side_effect = release
        cv = SimpleNamespace(CAP_V4L2=200, VideoCapture=Mock(return_value=cap),
                             setNumThreads=Mock(), __version__='SYNTHETIC')
        with patch.dict(sys.modules, {'cv2': cv}), \
             patch('wc_cameras.capture.arm_parent_death'), \
             patch('wc_cameras.capture.CameraLease', return_value=lease), \
             patch('wc_cameras.capture.verify_device', return_value=identity), \
             patch('wc_cameras.capture.configure_capture', return_value={'fixture': True}):
            if fail_read or fail_release:
                with self.assertRaises(SystemExit) as raised:
                    capture_worker(camera, profile, ROOT, 'fixture-no-device', Mock(), stop, status, 1)
                self.assertEqual(raised.exception.code, 1)
            else:
                capture_worker(camera, profile, ROOT, 'fixture-no-device', Mock(), stop, status, 1)
        cap.release.assert_called_once()
        lease.__exit__.assert_called_once()
        status.close.assert_called_once()
        return [call.args[0] for call in status.send.call_args_list]

    def test_worker_reports_release_complete_only_after_successful_release(self):
        events = self.run_worker()
        self.assertEqual([row['event'] for row in events], ['READY', 'RELEASE_COMPLETE'])
        timing = events[1]
        self.assertEqual(timing['host_release_elapsed_ns'],
                         timing['host_release_complete_monotonic_ns']-timing['host_release_start_monotonic_ns'])
        self.assertGreaterEqual(timing['host_release_elapsed_ns'], 0)

    def test_completed_release_does_not_erase_an_earlier_read_error(self):
        events = self.run_worker(fail_read=True)
        self.assertEqual([row['event'] for row in events], ['READY', 'RELEASE_COMPLETE', 'ERROR'])
        self.assertIn('V4L2 read failed', events[-1]['error'])

    def test_failed_release_emits_error_without_release_complete(self):
        events = self.run_worker(fail_release=True)
        self.assertEqual([row['event'] for row in events], ['READY', 'ERROR'])
        self.assertIn('synthetic release failure', events[-1]['error'])


if __name__ == '__main__':
    unittest.main()
