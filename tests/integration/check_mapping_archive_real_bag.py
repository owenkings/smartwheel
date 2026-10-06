#!/usr/bin/env python3
"""ARCHIVE_REPLAY of real saved SourceFrame CDR; no ROS node or publication.

Uses MappingInput, ArchiveWorker and CheckpointJournal with on_close. An
explicit synthetic stop-after-accept hook suppresses publication on every row;
this does not test live IMU/wheel freshness, motion integration or SLAM.
Run from the sourced project environment. The owned worker has a hard timeout.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time
import traceback

SOURCE_SESSION = 'map_right_20260914T125443Z_100a36'
TOTAL_TIMEOUT_S = 240


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def simple_name(value):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}', value):
        raise argparse.ArgumentTypeError('Expected a simple session/run name')
    return value


def digest(data):
    return hashlib.sha256(data).hexdigest()


def encoding_difference(original, encoded):
    """Describe serialized-layout differences without treating them as data loss."""
    shared = min(len(original), len(encoded))
    count = abs(len(original)-len(encoded))
    offsets = []
    for index in range(shared):
        if original[index] != encoded[index]:
            count += 1
            if len(offsets) < 32:
                offsets.append(index)
    offsets.extend(range(shared, min(max(len(original), len(encoded)), shared+32-len(offsets))))
    return {'original_bytes': len(original), 'reencoded_bytes': len(encoded),
            'different_byte_count': count, 'different_offset_sample': offsets,
            'reencoded_sha256': digest(encoded)}


def file_hash(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def identity(path):
    value = path.stat()
    return (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns)


def save_new(path, value):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())


def locations(args):
    # All ROS/project imports are delayed, including for --help on a PC.
    from wc_runtime.cli import ROOT, target
    from wc_runtime.single_mapping_input import project_path
    target()  # Read-only platform/identity check; no device preflight.
    source = project_path(ROOT, ROOT/'reports/maps'/args.source_session)
    output = project_path(ROOT, ROOT/'reports/map_wasd_storage_20260914'/args.run_name)
    require(source.is_dir() and source != output, 'Missing or overlapping source')
    require(not output.exists(), 'Output already exists; choose a new run name')
    return ROOT, source, output


class SyncObserver:
    """Count real os.fsync calls without suppressing/changing their effects."""
    def __init__(self):
        self.original = os.fsync
        self.phase = 'initialization'
        self.lock = threading.Lock()
        self.rows = {}

    def __call__(self, fd):
        with self.lock:
            phase = self.phase
        started = time.monotonic()
        success = False
        try:
            result = self.original(fd)
            success = True
            return result
        finally:
            elapsed = time.monotonic()-started
            with self.lock:
                row = self.rows.setdefault(phase, {'calls': 0, 'failures': 0, 'total_s': 0., 'max_s': 0.})
                row['calls'] += 1
                row['failures'] += int(not success)
                row['total_s'] += elapsed
                row['max_s'] = max(row['max_s'], elapsed)

    def set_phase(self, value):
        with self.lock:
            self.phase = value

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.rows)


def worker(args):
    from rclpy.serialization import deserialize_message, serialize_message
    from wc_interfaces.msg import SourceFrame
    from wc_runtime import mapping_input as m
    from wc_runtime.mapping_controller import input_archive_summary
    root, source, output = locations(args)
    source_input = source/'input'
    rows_path = m.project_path(root, source_input/'right/frames.jsonl')
    originals = [source/'runtime_config.json', source_input/'bootstrap.json', rows_path,
                 source_input/'status.json', source_input/'checkpoint.json']
    originals = [m.project_path(root, p) for p in originals]
    before = {str(p.relative_to(source)): {'sha256': file_hash(p), 'identity': identity(p)} for p in originals}
    config = json.loads(originals[0].read_text(encoding='utf-8'))
    bootstrap = json.loads(originals[1].read_text(encoding='utf-8'))
    rows = [json.loads(line) for line in rows_path.read_text(encoding='utf-8').splitlines()]
    prior_status = json.loads(originals[3].read_text(encoding='utf-8'))
    prior_checkpoint = json.loads(originals[4].read_text(encoding='utf-8'))
    require(config['session_id'] == bootstrap['session_id'] == args.source_session and config['mode'] == 'right',
            'Only the named real right-side recording is accepted')
    require(prior_status['state'] in ('STOPPED', 'FAILED') and prior_checkpoint.get('final') is True,
            'Source recording has no final closed checkpoint')
    require(300 <= len(rows) <= 500, 'Require 300..500 complete recorded frames; no duplication/subsampling')
    offsets = [0.]
    sources = []
    for number, row in enumerate(rows, 1):
        require(row['archive_index'] == number and row['file'] == 'frames/%08d.cdr' % number,
                'Unexpected recorded archive filename/order')
        require(row['raw_key'][0] == args.source_session and row['side'] == 'right', 'Source row identity mismatch')
        path = m.project_path(root, source_input/'right'/row['file'])
        require(path.is_file() and path.stat().st_size == row['cdr_bytes'], 'Source CDR missing/size mismatch')
        sources.append((path, identity(path)))
        if number > 1:
            delta = (row['host_monotonic_ns']-rows[number-2]['host_monotonic_ns'])/1e9
            require(delta >= .2, 'Original frames violate the unchanged 5 Hz acceptance limit')
            offsets.append(offsets[-1]+delta)
    require(offsets[-1] <= 100., 'Recorded spacing exceeds this bounded 100-second replay')
    require(prior_checkpoint['sources']['right']['frames'] == len(rows), 'Source checkpoint count differs')
    config = copy.deepcopy(config)
    config.update(archive_checkpoint_policy='on_close', archive_publication_mode='queued_realtime')
    require(config['input_rate_hz'] == 5., 'Keep the recorded 5 Hz input cap unchanged')
    output.mkdir(exist_ok=False)
    save_new(output/'archive_replay_config.json', config)
    report = {'status': 'FAIL', 'validation_level': 'ARCHIVE_REPLAY', 'source_session': args.source_session,
        'source_directory': str(source), 'output_directory': str(output),
        'hardware_started': False, 'ros_initialized': False, 'ros_publications': 0,
        'control_commands_sent': False, 'imu_wheel_reintegrated': False,
        'checkpoint_policy': 'on_close', 'source_frames': len(rows), 'recorded_span_s': offsets[-1],
        'pace': 'Original host-monotonic intervals, at most 5 Hz; never catch up in a burst',
        'clock': 'Original guard_receive_monotonic_ns for gate/clock; original message bytes and stamps unchanged',
        'synthetic_injection': 'should_stop becomes true only after each immutable archive acceptance; published=0',
        'serialization_injection': 'MappingInput serializer returns this frame original CDR bytes, not the rclpy reencoding',
        'roundtrip_contract': 'All decoded SourceFrame fields and cloud.data bytes equal after reencoding; CDR padding/layout bytes may differ and are reported, not assumed equivalent storage',
        'limitation': 'Tests real-byte archive writes and final persistence only; no live source freshness, IMU/wheel integration, SLAM or control acceptance.',
        'bootstrap': 'Reuse recorded transforms through the real bootstrap archive job; no IMU samples retimed or fabricated',
        'source_metadata_before': before, 'rows': [], 'peak_pending_outputs': 0, 'peak_pending_reserved_bytes': 0,
        'max_written_not_checkpointed': 0, 'max_replay_dispatch_lateness_s': 0.,
        'reencoding_frames_with_byte_differences': 0, 'reencoding_different_bytes_total': 0,
        'input_source_sha256': file_hash(root/'src/wc_runtime/mapping_input.py')}
    engine = None
    observer = SyncObserver()
    stop = threading.Event()
    previous_signals = {sig: signal.signal(sig, lambda *_: stop.set()) for sig in (signal.SIGINT, signal.SIGTERM)}
    os.fsync = observer
    now_ns = rows[0]['guard_receive_monotonic_ns']

    def tick():
        # Sample before consuming completions so a just-accepted queue peak is
        # not hidden by poll_io releasing several reservations immediately.
        report['peak_pending_outputs'] = max(report['peak_pending_outputs'], len(engine.pending))
        report['peak_pending_reserved_bytes'] = max(report['peak_pending_reserved_bytes'], engine.pending_bytes)
        engine.poll_io(now_ns, publish=False, clock=lambda: now_ns)
        engine.ensure_active()
        value = engine.status(now_ns)
        require(value['storage']['checkpoint'] is None and not (output/'input/checkpoint.json').exists(),
                'Unexpected runtime durable checkpoint')
        require(value['storage']['data_sync_workers']['jobs_started'] == 0, 'Runtime checkpoint started data fsync')
        report['peak_pending_outputs'] = max(report['peak_pending_outputs'], value['pending_outputs'])
        report['peak_pending_reserved_bytes'] = max(report['peak_pending_reserved_bytes'], value['pending_reserved_bytes'])
        report['max_written_not_checkpointed'] = max(report['max_written_not_checkpointed'], value['archived_outputs'])
        require(not stop.is_set(), 'Replay interrupted; preserve incomplete result')
        return value

    try:
        engine = m.MappingInput(config, root, output/'input', started_ns=now_ns, close_timeout_s=90.)
        engine.worker.submit({'kind': 'bootstrap', 'completed': copy.deepcopy(bootstrap), 'now_ns': now_ns})
        limit = time.monotonic()+10.
        while engine.transforms is None:
            tick()
            require(time.monotonic() < limit, 'Recorded bootstrap archive job timed out')
            time.sleep(.01)
        observer.set_phase('runtime_replay')
        started = time.monotonic()
        last_dispatch = None
        for row, (path, original_identity), offset in zip(rows, sources, offsets):
            # Preserve minimum inter-dispatch spacing even if disk/decode work is slow.
            due = started+offset
            if last_dispatch is not None:
                due = max(due, last_dispatch+.2)
            while time.monotonic() < due:
                tick()
                time.sleep(min(.01, max(0., due-time.monotonic())))
            tick()
            require(time.monotonic()-started <= 115., 'Replay execution exceeded pacing allowance')
            raw = path.read_bytes()
            require(identity(path) == original_identity and digest(raw) == row['cdr_sha256'], 'Original CDR changed')
            message = deserialize_message(raw, SourceFrame)
            roundtrip = bytes(serialize_message(message))
            decoded_again = deserialize_message(roundtrip, SourceFrame)
            require(message == decoded_again, 'ROS roundtrip changed decoded SourceFrame fields')
            require(bytes(message.cloud.data) == bytes(decoded_again.cloud.data), 'ROS roundtrip changed cloud.data bytes')
            difference = encoding_difference(raw, roundtrip)
            difference.update(decoded_fields_equal=True, cloud_data_bytes_equal=True)
            report['reencoding_frames_with_byte_differences'] += int(difference['different_byte_count'] > 0)
            report['reencoding_different_bytes_total'] += difference['different_byte_count']
            require(message.host_monotonic_ns == row['host_monotonic_ns'] and
                    m._stamp(message.header.stamp) == row['header_stamp_ns'], 'Recorded CDR/index time mismatch')
            now_ns = row['guard_receive_monotonic_ns']
            accepted_before = engine.enqueued_outputs
            def no_publish(_cloud):
                raise RuntimeError('Unexpected publication callback during archive-only replay')
            def original_cdr(candidate, expected=message, original=raw):
                require(candidate is expected, 'Original-CDR serializer received another message')
                return original
            accepted = engine.receive_source(message, now_ns, original_cdr, no_publish,
                clock=lambda: now_ns, should_stop=lambda: engine.enqueued_outputs > accepted_before)
            require(accepted and engine.enqueued_outputs == accepted_before+1, 'Real source frame was not accepted')
            last_dispatch = time.monotonic()
            report['max_replay_dispatch_lateness_s'] = max(report['max_replay_dispatch_lateness_s'], last_dispatch-due)
            report['rows'].append({'archive_index': row['archive_index'], 'cdr_bytes': len(raw),
                'cdr_sha256': digest(raw), 'frame_sequence': message.frame_sequence,
                'original_stamp_ns': row['header_stamp_ns'], 'original_host_monotonic_ns': message.host_monotonic_ns,
                'cdr_reencoding': difference})
            tick()
        # Finish buffered writes while still testing that runtime does no fsync.
        limit = time.monotonic()+10.
        while engine.archived_outputs < len(rows):
            tick()
            require(time.monotonic() < limit, 'Runtime archive writer did not drain in 10 seconds')
            time.sleep(.01)
        report['runtime_duration_s'] = time.monotonic()-started
        report['runtime_status'] = tick()
        report['runtime_fsync_calls'] = observer.snapshot().get('runtime_replay', {}).get('calls', 0)
        require(report['runtime_fsync_calls'] == 0, 'Observed a runtime fsync under on_close')
        require(report['max_written_not_checkpointed'] > 256 and engine.published_frames == 0,
                'Did not exercise more than 256 written, unsynced outputs with zero publications')
        report['runtime_written_bytes'] = sum(p.stat().st_size for p in (output/'input/right/frames').glob('*.cdr'))
        require(report['runtime_written_bytes'] == sum(r['cdr_bytes'] for r in rows), 'Runtime CDR byte total differs')
    except BaseException as error:
        report['error'] = type(error).__name__+': '+str(error)
        report['traceback'] = traceback.format_exc()
    finally:
        observer.set_phase('close')
        closed_at = time.monotonic()
        try:
            if engine is not None:
                engine.close()
                report['final_status'] = engine.status(now_ns)
        except BaseException as error:
            report['close_error'] = type(error).__name__+': '+str(error)
        report['close_duration_s'] = time.monotonic()-closed_at
        report['fsync_observation'] = observer.snapshot()
        os.fsync = observer.original
        for sig, handler in previous_signals.items():
            signal.signal(sig, handler)
    try:
        require('error' not in report and 'close_error' not in report, 'Replay or close failed; no acceptance')
        report['archive_audit'] = input_archive_summary({'directory': str(output), 'mode': 'right', 'session_id': args.source_session})
        final = report['final_status']
        require(final['published_frames'] == 0 and final['outputs_discarded_on_close'] == len(rows), 'Publication accounting differs')
        require(final['storage']['checkpoint']['final'] is True and final['archived_outputs'] == len(rows), 'Incomplete final checkpoint')
        for row, (source_path, original_identity) in zip(rows, sources):
            target = output/'input/right'/row['file']
            require(target.read_bytes() == source_path.read_bytes(), 'Archived CDR bytes differ from source')
            require(file_hash(target) == row['cdr_sha256'] and file_hash(source_path) == row['cdr_sha256'] and
                    identity(source_path) == original_identity, 'CDR hash/source identity changed')
        for p in originals:
            record = before[str(p.relative_to(source))]
            require(file_hash(p) == record['sha256'] and identity(p) == record['identity'], 'Source metadata changed')
        report.update(status='PASS', source_unchanged=True, all_cdr_bytes_and_hashes_equal=True,
                      final_checkpoint_covers_all=True, archived_frames=len(rows), published_frames=0)
    except BaseException as error:
        report['audit_error'] = type(error).__name__+': '+str(error)
    save_new(output/'result.json', report)
    print(json.dumps({'status': report['status'], 'validation_level': 'ARCHIVE_REPLAY',
        'report': str(output/'result.json'), 'archived_frames': report.get('archived_frames'),
        'published_frames': 0, 'runtime_fsync_calls': report.get('runtime_fsync_calls'),
        'close_duration_s': report['close_duration_s'], 'error': report.get('error', report.get('audit_error'))}))
    return 0 if report['status'] == 'PASS' else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-session', type=simple_name, default=SOURCE_SESSION)
    parser.add_argument('--run-name', type=simple_name, default='archive_replay_01')
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.worker:
        return worker(args)
    _, _, output = locations(args)
    command = [sys.executable, str(Path(__file__).resolve()), '--worker',
               '--source-session', args.source_session, '--run-name', args.run_name]
    stop = threading.Event()
    previous_signals = {sig: signal.signal(sig, lambda *_: stop.set()) for sig in (signal.SIGINT, signal.SIGTERM)}
    child = None
    try:
        child = subprocess.Popen(command, stdin=subprocess.DEVNULL, start_new_session=True)
        deadline = time.monotonic()+TOTAL_TIMEOUT_S
        while child.poll() is None:
            if stop.is_set() or time.monotonic() >= deadline:
                print(json.dumps({'status': 'FAIL', 'validation_level': 'ARCHIVE_REPLAY',
                    'error': 'INTERRUPTED' if stop.is_set() else 'WORKER_TOTAL_TIMEOUT', 'output': str(output)}))
                return 1
            try:
                child.wait(timeout=.2)
            except subprocess.TimeoutExpired:
                pass
        return child.returncode
    finally:
        try:
            if child is not None and child.poll() is None:
                child.terminate()  # Directly owned worker; no process-name searches.
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait(timeout=5)
        finally:
            for sig, handler in previous_signals.items():
                signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
