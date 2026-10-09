#!/usr/bin/env python3
"""Read-only, bounded resource evidence for one registered synthetic map session.

No ROS imports, child commands, signals, sensor access or command-line reads.
Discovery follows /proc/<owned-pid>/task/*/children. Kernels without that optional
file use a UID/PPID/start-ticks-only /proc/stat index; resource counters are
sampled only after ancestry reaches the registered supervisor. This is resource
observation, never a realtime or mapping verdict.
"""

from __future__ import annotations

import argparse
import errno
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import pwd
import re
import shutil
import socket
import stat
import sys
import tempfile
import time
import traceback


ROOT = Path('/home/nvidia/wheelchair')
MAX_PROCESSES = 256
MAX_TASKS = 4096
MAX_PROC_ENTRIES = 65536
MAX_INPUT_BYTES = 1024 * 1024
MAX_DURATION = 600.0


class Blocked(RuntimeError):
    """An explicit boundary prevents further scoped collection."""


class ChildrenUnavailable(RuntimeError):
    """This live kernel/task does not provide the optional children file."""


def require(condition, reason):
    if not condition:
        raise Blocked(reason)


def session_name(value):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}', value):
        raise argparse.ArgumentTypeError('session must be a simple ASCII identifier')
    return value


def duration_value(value):
    try:
        duration = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('duration must be numeric') from exc
    if not math.isfinite(duration) or not 1.0 <= duration <= MAX_DURATION:
        raise argparse.ArgumentTypeError('duration must be finite and 1..600 seconds')
    return duration


def scoped_path(path, parent):
    path = Path(path).absolute()
    parent = Path(parent).resolve(strict=True)
    require(path.resolve(strict=False).is_relative_to(parent), 'path outside permitted project area')
    cursor = path
    while cursor != cursor.parent:
        require(not cursor.is_symlink(), 'symlink in project input/output path')
        cursor = cursor.parent
    return path


def read_json(path):
    with path.open('rb') as stream:
        metadata = os.fstat(stream.fileno())
        require(stat.S_ISREG(metadata.st_mode), 'JSON input is not a regular file')
        data = stream.read(MAX_INPUT_BYTES + 1)
    require(len(data) <= MAX_INPUT_BYTES, 'JSON input exceeds the size bound')
    value = json.loads(data)
    require(isinstance(value, dict), 'JSON input must be an object')
    return value, hashlib.sha256(data).hexdigest()


def read_small(path, limit=65536):
    """Bounded unbuffered proc/sysfs read; EAGAIN remains an explicit OSError.

    TextIOWrapper can turn a raw nonblocking EAGAIN/None into a TypeError.
    os.read exposes the actual errno instead, without retrying or polling a
    thermal driver. sysfs thermal-zone symlinks are intentional read-only input.
    """
    descriptor = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
    try:
        chunks, size = [], 0
        while True:
            data = os.read(descriptor, min(4096, limit + 1 - size))
            if data is None:
                raise BlockingIOError(errno.EAGAIN, 'raw read temporarily unavailable', str(path))
            if not data:
                break
            chunks.append(data)
            size += len(data)
            require(size <= limit, 'kernel file exceeds the explicit byte bound: ' + str(path))
        return b''.join(chunks).decode('utf-8')
    except OSError as exc:
        raise OSError(exc.errno, str(exc), str(path)) from exc
    finally:
        os.close(descriptor)


def target_identity():
    identity = dict(hostname=socket.gethostname(), user=pwd.getpwuid(os.getuid()).pw_name,
                    effective_user=pwd.getpwuid(os.geteuid()).pw_name,
                    architecture=platform.machine(), project_root=str(ROOT))
    require(identity['hostname'] == 'ubuntu' and identity['user'] == 'nvidia'
            and identity['effective_user'] == 'nvidia' and identity['architecture'] == 'aarch64',
            'requires the verified ubuntu/nvidia/aarch64 target')
    require(ROOT.is_dir() and not ROOT.is_symlink(), 'project root missing or redirected')
    return identity


def process_stat(pid):
    """Read only an already scoped PID; Linux stat fields are indexed after comm."""
    require(isinstance(pid, int) and not isinstance(pid, bool) and pid > 1, 'invalid process PID')
    path = Path('/proc') / str(pid)
    require(path.stat().st_uid == os.getuid(), 'process UID differs from the project owner')
    data = read_small(path / 'stat')
    require(len(data) <= 65536, 'proc stat exceeds the size bound')
    require(data.split(' ', 1)[0] == str(pid), 'proc PID field mismatch')
    fields = data[data.rindex(')') + 2:].split()
    require(len(fields) >= 22, 'incomplete proc stat')
    return dict(pid=pid, start_ticks=int(fields[19]), state=fields[0], ppid=int(fields[1]),
                user_ticks=int(fields[11]), system_ticks=int(fields[12]),
                threads=int(fields[17]), rss_bytes=max(0, int(fields[21])) * os.sysconf('SC_PAGE_SIZE'),
                read_monotonic_ns=time.monotonic_ns())


def same_process(current, expected):
    require(current['start_ticks'] == int(expected['start_ticks']),
            'PID identity changed: %s' % current['pid'])


def owned_children(parent):
    """Threads may fork too; read children for this owned process's tasks only."""
    current = process_stat(parent['pid'])
    same_process(current, parent)
    tasks = list((Path('/proc') / str(parent['pid']) / 'task').iterdir())
    require(len(tasks) <= MAX_TASKS, 'owned process task count exceeds the explicit bound')
    children = set()
    for task in tasks:
        if not task.name.isdecimal():
            continue
        try:
            data = read_small(task / 'children')
        except FileNotFoundError:
            if task.is_dir():
                # Linux exposes this file only with CONFIG_CHECKPOINT_RESTORE.
                # A live task without it is not evidence of an empty tree.
                same_process(process_stat(parent['pid']), parent)
                raise ChildrenUnavailable('live task does not expose proc children')
            continue  # An owned thread actually exited during observation.
        require(len(data) <= 65536, 'children list exceeds the explicit bound')
        children.update(int(value) for value in data.split())
        require(len(children) <= MAX_PROCESSES, 'owned child list exceeds the explicit process bound')
    same_process(process_stat(parent['pid']), parent)
    return children


def process_identity(pid):
    """Discovery-only fields: no cmdline and no unrelated resource counters."""
    path = Path('/proc') / str(pid)
    if path.stat().st_uid != os.getuid():
        return None
    text = read_small(path / 'stat')
    require(len(text) <= 65536 and text.split(' ', 1)[0] == str(pid), 'invalid discovery proc stat')
    fields = text[text.rindex(')') + 2:].split()
    require(len(fields) >= 20, 'incomplete discovery proc stat')
    return dict(pid=pid, ppid=int(fields[1]), start_ticks=int(fields[19]), state=fields[0])


def parent_index():
    """A bounded identity index for kernels lacking /proc task children."""
    index, visited = {}, 0
    for path in Path('/proc').iterdir():
        if not path.name.isdecimal():
            continue
        visited += 1
        require(visited <= MAX_PROC_ENTRIES, 'proc identity index exceeds the explicit bound')
        try:
            record = process_identity(int(path.name))
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        if record is not None:
            index.setdefault(record['ppid'], []).append(record)
    return index


def registered_components(manifest):
    records = manifest.get('children')
    require(isinstance(records, list) and 0 < len(records) < MAX_PROCESSES,
            'RUNNING manifest has no bounded registered component list')
    result = {}
    for record in records:
        require(isinstance(record, dict), 'invalid manifest component entry')
        identity = supervisor_identity(dict(supervisor_pid=record.get('pid'),
                                            supervisor_start_ticks=record.get('start_ticks')))
        require(identity['pid'] not in result, 'duplicate manifest component PID')
        result[identity['pid']] = identity['start_ticks']
    return result


def discover_tree(supervisor, known_identities, components=None):
    root = process_stat(supervisor['pid'])
    same_process(root, supervisor)
    require(root['state'] not in ('Z', 'X', 'x'), 'registered supervisor is no longer running')
    pending, found, events = [root], {}, []
    components = components or {}
    for pid, ticks in components.items():
        component = process_identity(pid)
        require(component is not None, 'registered component UID differs from the session owner')
        same_process(component, dict(start_ticks=ticks))
        require(component['ppid'] == root['pid'], 'registered component no longer belongs to supervisor')
    fallback_index = None
    while pending:
        node = pending.pop()
        pid = node['pid']
        if pid in found:
            same_process(node, found[pid])
            continue
        require(len(found) < MAX_PROCESSES, 'owned process count exceeds the explicit bound')
        if pid in known_identities:
            require(node['start_ticks'] == known_identities[pid], 'previously owned PID identity changed: %s' % pid)
        known_identities[pid] = node['start_ticks']
        found[pid] = node
        try:
            if fallback_index is None:
                try:
                    children = {child_pid: None for child_pid in owned_children(node)}
                except ChildrenUnavailable:
                    fallback_index = parent_index()
                    events.append(dict(event='proc_children_unavailable_identity_index_used'))
                    children = {child['pid']: child for child in fallback_index.get(pid, ())}
            else:
                children = {child['pid']: child for child in fallback_index.get(pid, ())}
            if pid == root['pid']:
                # Manifest entries are additional candidates, never a bypass
                # of current UID, PPID and exact start-tick checks below.
                for child_pid, ticks in components.items():
                    children[child_pid] = dict(pid=child_pid, ppid=pid, start_ticks=ticks)
        except (FileNotFoundError, ProcessLookupError):
            if pid == supervisor['pid']:
                raise Blocked('registered supervisor exited during tree observation')
            events.append(dict(event='owned_process_exited_during_scan', pid=pid,
                               start_ticks=node['start_ticks']))
            continue
        for child_pid, candidate in sorted(children.items()):
            try:
                identity = process_identity(child_pid)
                if identity is None:
                    continue
                if candidate is not None:
                    same_process(identity, candidate)
                if identity['ppid'] != pid:
                    events.append(dict(event='stale_child_entry_excluded', parent_pid=pid, pid=child_pid))
                    continue
                child = process_stat(child_pid)
                same_process(child, identity)
            except (FileNotFoundError, ProcessLookupError):
                continue
            # A stale children entry does not confer ownership after reparenting.
            if child['ppid'] != pid:
                events.append(dict(event='stale_child_entry_excluded', parent_pid=pid, pid=child_pid))
                continue
            pending.append(child)
    same_process(process_stat(supervisor['pid']), supervisor)
    for pid, node in found.items():
        current = process_identity(pid)
        require(current is not None, 'owned process became unavailable during ancestry verification')
        same_process(current, node)
        require(current['ppid'] == node['ppid'], 'owned ancestry changed during resource sampling')
        if pid != supervisor['pid']:
            require(node['ppid'] in found, 'owned process no longer has an observed parent chain')
    require(set(components).issubset(found), 'registered components were not observable in the owned tree')
    return sorted(found.values(), key=lambda value: value['pid']), events


def session_inputs(session, manifest_path):
    plan_path = scoped_path(manifest_path.parent / 'plan.json', ROOT)
    plan, plan_hash = read_json(plan_path)
    require(plan.get('session_id') == session and plan.get('role') == 'map', 'plan session/role mismatch')
    paths = set()
    commands = plan.get('commands')
    require(isinstance(commands, list) and bool(commands), 'plan commands are missing')
    for command in commands:
        require(isinstance(command, list) and all(isinstance(arg, str) for arg in command), 'invalid plan argv')
        for index, argument in enumerate(command):
            if argument.startswith('session_config:='):
                paths.add(argument.partition(':=')[2])
            elif argument == '--session-config':
                require(index + 1 < len(command), 'session config argument is missing')
                paths.add(command[index + 1])
    require(len(paths) == 1, 'plan must identify exactly one session configuration')
    config_path = scoped_path(paths.pop(), ROOT)
    config, config_hash = read_json(config_path)
    require(config.get('session_id') == session, 'configuration session identity mismatch')
    require(config.get('source_mode') == 'synthetic', 'resource sampler is restricted to declared synthetic sessions')
    manifest, manifest_hash = read_json(manifest_path)
    require(manifest.get('session_id') == session and manifest.get('role') == 'map', 'manifest session/role mismatch')
    return manifest, dict(source_mode=config['source_mode'], sensor_mode=config.get('sensor_mode'),
                          session_config=str(config_path), session_config_sha256=config_hash,
                          plan_path=str(plan_path), plan_sha256=plan_hash,
                          initial_manifest_sha256=manifest_hash)


def supervisor_identity(manifest):
    pid, ticks = manifest.get('supervisor_pid'), manifest.get('supervisor_start_ticks')
    require(isinstance(pid, int) and not isinstance(pid, bool) and pid > 1, 'invalid manifest supervisor PID')
    require(isinstance(ticks, (str, int)) and not isinstance(ticks, bool)
            and str(ticks).isdecimal() and int(ticks) > 0, 'invalid manifest supervisor start ticks')
    return dict(pid=pid, start_ticks=int(ticks))


def temperature_sample():
    entries = []
    try:
        zones = sorted(Path('/sys/class/thermal').glob('thermal_zone*'))
        require(len(zones) <= 256, 'thermal zone count exceeds the explicit bound')
    except OSError as exc:
        return dict(status='UNKNOWN', reason=type(exc).__name__, zones=[])
    for zone in zones:
        item = dict(zone=zone.name, type=None, raw_temp_millidegrees_c=None, degrees_c=None, status='UNKNOWN')
        errors = []
        try:
            item['type'] = read_small(zone / 'type', 256).strip()
        except (OSError, ValueError) as exc:
            errors.append(dict(field='type', error=type(exc).__name__, errno=getattr(exc, 'errno', None), reason=str(exc)))
        try:
            raw = int(read_small(zone / 'temp', 256).strip())
            item.update(raw_temp_millidegrees_c=raw, degrees_c=raw / 1000.0, status='READABLE')
        except (OSError, ValueError) as exc:
            errors.append(dict(field='temp', error=type(exc).__name__, errno=getattr(exc, 'errno', None), reason=str(exc)))
        if errors:
            item['read_errors'] = errors
        entries.append(item)
    return dict(status='READABLE' if any(x['status'] == 'READABLE' for x in entries) else 'UNKNOWN', zones=entries)


def disk_sample():
    try:
        usage = shutil.disk_usage(ROOT)
        return dict(status='READABLE', path=str(ROOT), total_bytes=usage.total,
                    used_bytes=usage.used, free_bytes=usage.free)
    except OSError as exc:
        return dict(status='UNKNOWN', path=str(ROOT), reason=type(exc).__name__, free_bytes=None)


def metric_summary(values):
    values = sorted(value for value in values if value is not None)
    if not values:
        return dict(count=0, max=None, p50=None, p95=None)
    def percentile(fraction):
        index = (len(values) - 1) * fraction
        lower, upper = math.floor(index), math.ceil(index)
        return values[lower] + (values[upper] - values[lower]) * (index - lower)
    return dict(count=len(values), max=values[-1], p50=percentile(0.50), p95=percentile(0.95))


def summarize(samples):
    aggregates = [item['aggregate'] for item in samples]
    free = [item['disk']['free_bytes'] for item in samples if item['disk']['free_bytes'] is not None]
    return dict(sample_count=len(samples),
                cpu_percent_one_core=metric_summary([x['cpu_percent_one_core'] for x in aggregates]),
                rss_bytes_sum=metric_summary([x['rss_bytes_sum'] for x in aggregates]),
                threads_sum=metric_summary([x['threads_sum'] for x in aggregates]),
                process_count=metric_summary([x['process_count'] for x in aggregates]),
                minimum_free_disk_bytes=min(free) if free else None,
                measurement_scope='REGISTERED_SYNTHETIC_SESSION_PROCESS_TREE_ONLY',
                verdict='RESOURCE_OBSERVATIONS_ONLY_NO_REALTIME_PASS')


def collect(args, report, manifest_path):
    wait_start = time.monotonic()
    last_reason = 'map manifest has not appeared'
    while True:
        try:
            manifest, metadata = session_inputs(args.session, manifest_path)
            report['metadata'].update(metadata)
            report['level'] = 'SYNTHETIC'
            if manifest.get('state') == 'RUNNING':
                break
            require(manifest.get('state') in ('STARTING', None), 'map manifest is terminal before sampling: ' + str(manifest.get('state')))
            last_reason = 'map supervisor is not RUNNING yet'
        except FileNotFoundError:
            pass
        if time.monotonic() - wait_start >= 10.0:
            raise Blocked('startup_timeout_10s: ' + last_reason)
        time.sleep(min(0.2, max(0.0, 10.0 - (time.monotonic() - wait_start))))

    supervisor = supervisor_identity(manifest)
    components = registered_components(manifest)
    report['metadata']['supervisor_identity'] = supervisor
    report['metadata']['registered_component_identities'] = components
    report['metadata']['startup_wait_seconds'] = time.monotonic() - wait_start
    first = time.monotonic()
    deadline, scheduled = first + args.duration, first
    previous, known = {}, {}
    tick_rate = os.sysconf('SC_CLK_TCK')
    while True:
        now = time.monotonic()
        if now < scheduled:
            time.sleep(min(scheduled - now, max(0.0, deadline - now)))
        started_ns, wall_ns = time.monotonic_ns(), time.time_ns()
        current_manifest, manifest_hash = read_json(manifest_path)
        require(current_manifest.get('session_id') == args.session and current_manifest.get('role') == 'map', 'manifest session/role changed')
        require(supervisor_identity(current_manifest) == supervisor, 'registered supervisor identity changed')
        require(registered_components(current_manifest) == components, 'registered component identities changed')
        require(current_manifest.get('state') == 'RUNNING', 'session_stopped_before_requested_duration: ' + str(current_manifest.get('state')))
        processes, events = discover_tree(supervisor, known, components)
        require(len(processes) > 1, 'registered supervisor has no observable owned component')
        cpu_values, delta_ticks, baseline_count = [], 0, 0
        for process in processes:
            key = (process['pid'], process['start_ticks'])
            older = previous.get(key)
            process['cpu_percent_one_core'] = None
            if older is not None:
                delta = process['user_ticks'] + process['system_ticks'] - older['user_ticks'] - older['system_ticks']
                elapsed_ns = process['read_monotonic_ns'] - older['read_monotonic_ns']
                require(delta >= 0 and elapsed_ns > 0, 'owned CPU counter/time regressed')
                process['cpu_percent_one_core'] = 100.0 * delta / tick_rate / (elapsed_ns / 1e9)
                cpu_values.append(process['cpu_percent_one_core'])
                delta_ticks += delta
            else:
                baseline_count += 1
        current_keys = {(p['pid'], p['start_ticks']) for p in processes}
        departed = [dict(pid=key[0], start_ticks=key[1]) for key in previous if key not in current_keys]
        sample = dict(monotonic_ns=started_ns, realtime_ns=wall_ns,
                      since_collection_start_s=started_ns / 1e9 - first,
                      schedule_lateness_s=max(0.0, started_ns / 1e9 - scheduled),
                      manifest_state=current_manifest['state'], manifest_sha256=manifest_hash,
                      aggregate=dict(cpu_percent_one_core=sum(cpu_values) if cpu_values else None,
                                     observed_delta_cpu_ticks=delta_ticks,
                                     cpu_processes_with_baseline=len(cpu_values),
                                     cpu_processes_without_baseline=baseline_count,
                                     rss_bytes_sum=sum(p['rss_bytes'] for p in processes),
                                     threads_sum=sum(p['threads'] for p in processes), process_count=len(processes)),
                      processes=processes, departed_since_previous_sample=departed, events=events,
                      disk=disk_sample(), thermal=temperature_sample())
        sample['scan_duration_s'] = (time.monotonic_ns() - started_ns) / 1e9
        report['samples'].append(sample)
        previous = {(p['pid'], p['start_ticks']): p for p in processes}
        if time.monotonic() >= deadline:
            break
        # Skip missed slots instead of producing bursts that look like 1 Hz data.
        scheduled = min(deadline, first + math.floor(time.monotonic() - first) + 1.0)
    _, config_hash = read_json(Path(report['metadata']['session_config']))
    _, plan_hash = read_json(Path(report['metadata']['plan_path']))
    require(config_hash == report['metadata']['session_config_sha256'], 'session configuration changed while sampling')
    require(plan_hash == report['metadata']['plan_sha256'], 'session plan changed while sampling')
    report.update(status='RECORDED', reason='requested_duration_reached',
                  observed_duration_s=time.monotonic() - first)


def write_atomic_new(output, report):
    """fsync then link publishes atomically; an existing name is never replaced."""
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', prefix='.resource-', suffix='.tmp',
                                         dir=output.parent, delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, output)
        directory = os.open(output.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True, type=session_name)
    parser.add_argument('--duration', default=110.0, type=duration_value)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    report = dict(schema='wc_resource_observation_v1', session_id=args.session, role='map',
                  status='BLOCKED', level='UNVERIFIED', requested_duration_s=args.duration,
                  requested_interval_s=1.0, started_realtime_ns=time.time_ns(),
                  metadata=dict(source_mode=None), samples=[],
                  scope=dict(realtime_performance='NOT_VALIDATED', real_hardware='NOT_TESTED',
                             mapping='NOT_VALIDATED', synchronization='NOT_ESTABLISHED',
                             common_measurement_latency='UNKNOWN_NOT_MEASURED', signals_sent=0),
                  method=dict(cpu='sum of per-process delta(user+system ticks)/actual elapsed; 100 percent is one logical core',
                              process_discovery='manifest PID+start-ticks and current UID/ancestry; optional proc children or identity-only PPID index; no cmdline reads',
                              cpu_limits='new processes lack a previous baseline; exited CPU tails and between-sample short-lived processes are not measured',
                              rss='sum of per-process resident pages; shared pages can be counted more than once',
                              sampling='1 Hz monotonic schedule; actual read timestamps/lateness and skipped slots are retained',
                              thermal='Linux thermal sysfs temperatures in millidegrees C; unreadable fields are UNKNOWN',
                              percentiles='linear interpolation over observed sample values; no acceptance thresholds'))
    output = None
    try:
        report['metadata'].update(target_identity())
        output = scoped_path(args.output, ROOT / 'reports')
        require(output.suffix == '.json' and not output.exists(), 'output must be a new reports/*.json file')
        output.parent.mkdir(parents=True, exist_ok=True)
        manifest_path = scoped_path(ROOT / '.phase1_runtime' / 'sessions' / args.session / 'map' / 'manifest.json', ROOT)
        report['metadata'].update(manifest_path=str(manifest_path), clock_ticks_per_second=os.sysconf('SC_CLK_TCK'),
                                  page_size_bytes=os.sysconf('SC_PAGE_SIZE'), logical_cpu_count=os.cpu_count())
        collect(args, report, manifest_path)
    except KeyboardInterrupt:
        report.update(status='BLOCKED', reason='sampler_interrupted_no_signal_sent_to_session')
    except Exception as exc:
        report.update(status='BLOCKED', reason=type(exc).__name__ + ': ' + str(exc),
                      failure_traceback=traceback.format_exc()[-32768:])
    report['ended_realtime_ns'] = time.time_ns()
    report['summary'] = summarize(report['samples'])
    if output is not None:
        try:
            write_atomic_new(output, report)
        except OSError as exc:
            print(json.dumps(dict(status='BLOCKED', reason='report_write_failed: ' + str(exc),
                                  failure_traceback=traceback.format_exc()[-32768:])), file=sys.stderr)
            return 2
    print(json.dumps(dict(status=report['status'], level=report['level'], session_id=args.session,
                          output=str(output) if output is not None else None,
                          reason=report['reason'], summary=report['summary']), allow_nan=False))
    return 0 if report['status'] == 'RECORDED' else 2


if __name__ == '__main__':
    raise SystemExit(main())
