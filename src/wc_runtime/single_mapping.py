"""Managed single-lidar 3D mapping experiment with original data and native DB.

Separate from formal dual-source acceptance. No motor, IMU, encoder, camera,
navigation, guessed mounting transform or calibrated world ground is used.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
from .project_paths import ros_setup_path
import sqlite3
import subprocess
import sys
import time

from .cli import ROOT, RUN, begin, device_preflight, environment, name, ros_command, target
from .prepare_picker_input import project_path, read_json, write_new, json_bytes
from .supervisor import shutdown_policy, stop_registered, write_json
from .project_config import selected_config_path, lidar_config_directory


def session_directory(session):
    return project_path(ROOT, Path('reports/single_mapping')/name(session))


def hash_file(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def make_plan(session, side, duration, port, root=ROOT, *, lidar_directory=None):
    """Pure argv construction: each native publisher has a single experiment owner."""
    name(session)
    if side not in ('left', 'right') or type(duration) is not int or not 40 <= duration <= 1800:
        raise ValueError('Single mapping requires left/right and 40..1800 seconds')
    if type(port) is not int or not 1024 <= port <= 65535:
        raise ValueError('Use an explicit unprivileged loopback viewer port')
    directory = project_path(root, Path('reports/single_mapping')/session)
    namespace = '/wc_mapping/single_'+side
    source = '/wc_mapping/lidar_'+side
    topics = [source+'/source_frame', source+'/source_frame_filtered', source+'/diagnostics',
              namespace+'/scan_cloud', namespace+'/odom', namespace+'/odom_info',
              namespace+'/info', namespace+'/mapData', namespace+'/mapGraph', namespace+'/cloud_map',
              namespace+'/tf', namespace+'/tf_static']
    commands = [
        ros_command(['ros2', 'bag', 'record', '-s', 'sqlite3', '--max-bag-size', '536870912',
                     '-o', directory/'bag', *topics]),
        ros_command(['ros2', 'launch', root/'src/wc_bringup/launch/single_mapping.launch.py',
                     'side:='+side, 'session_root:='+str(directory)]),
        ros_command([sys.executable, '-s', '-m', 'wc_runtime.single_mapping_input',
                     '--session-id', session, '--side', side, '--output-root', directory/'input', '--rate-hz', '5']),
        ros_command([sys.executable, '-s', '-m', 'wc_runtime.single_mapping_monitor',
                     '--session-id', session, '--side', side, '--output-root', directory/'monitor', '--port', str(port)]),
        ros_command(['ros2', 'launch', 'wc_xt_driver', 'dual_sources.launch.py',
                     'source_mode:=single_'+side, 'session_id:='+session, 'run_root:='+str(root/'.phase1_runtime'),
                     'config_directory:='+str(lidar_directory if lidar_directory is not None else root/'src/wc_xt_driver/config'),
                     'allow_hardware:=true', 'read_only_probe:=false', 'device_config_policy:=preserve_current',
                     'publish_cloud_mirror:=true', 'max_runtime_seconds:='+str(duration+15)]),
    ]
    return {'session_id': session, 'role': 'single_mapping', 'commands': commands,
            'environment': environment(), 'duration_s': duration, 'sigint_grace_s': 30.0,
            'allow_component_exit': False,
            'locks': ['sensor_owner.lock', 'domain-83-source.lock', 'domain-83-processing.lock',
                      'single-mapping.lock', 'single-viewer-'+str(port)+'.lock'],
            'experiment': {'kind': 'SINGLE_LIDAR_EXPERIMENT', 'source_mode': 'real', 'sensor_mode': 'single_'+side,
                'directory': str(directory), 'viewer_port': port, 'input_rate_hz': 5,
                'time_source': 'arrival_only', 'time_model_validated': False,
                'ground_reference_validated': False, 'metric_accuracy_validated': False,
                'formal_acceptance': False, 'navigation_validated': False, 'bag_topics': topics,
                'reference': 'Initial pose of selected lidar; no base_link, left-right or IMU extrinsic inferred'}}


def start(session, side, duration, port):
    target()
    if Path.cwd().resolve() != ROOT:
        raise RuntimeError('Run from '+str(ROOT))
    plan = make_plan(session, side, duration, port, lidar_directory=lidar_config_directory(ROOT))
    device_preflight()
    import socket
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', port))
    directory = session_directory(session)
    runtime = project_path(ROOT, RUN/'sessions'/session/'single_mapping')
    if directory.exists() or runtime.exists():
        raise RuntimeError('New session required; old data is never overwritten')
    config = read_json(ROOT, selected_config_path(ROOT, 'live_unvalidated.json'))
    plan['experiment']['sensor_id'] = config['sensor_ids'][side]
    required = ['src/wc_runtime/single_mapping_input.py', 'src/wc_runtime/single_mapping_monitor.py',
                'src/wc_bringup/launch/single_mapping.launch.py', 'src/wc_runtime/single_mapping.py']
    plan['experiment']['source_hashes'] = {p: hash_file(project_path(ROOT, p)) for p in required}
    for executable in ('rtabmap_odom/icp_odometry', 'rtabmap_slam/rtabmap'):
        if not (ros_setup_path().parent/'lib'/executable).is_file():
            raise RuntimeError('Reviewed native executable missing: '+executable)
    directory.mkdir(parents=True, exist_ok=False)
    (directory/'slam').mkdir()
    runtime.mkdir(parents=True, exist_ok=False)
    write_new(directory/'experiment.json', json_bytes(plan['experiment']))
    write_new(runtime/'plan.json', json_bytes(plan))
    env = dict(os.environ); env.update(environment()); env['OPENBLAS_NUM_THREADS'] = '1'
    with (runtime/'supervisor.log').open('xb') as log:
        child = subprocess.Popen([sys.executable, '-m', 'wc_runtime.supervisor', '--plan', str(runtime/'plan.json')],
            cwd=ROOT, env=env, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    deadline = time.monotonic()+10
    manifest = runtime/'manifest.json'
    while time.monotonic() < deadline:
        if manifest.exists():
            state = read_json(ROOT, manifest)
            if state['state'] == 'RUNNING':
                return {'status': 'STARTED_NOT_YET_VERIFIED', 'session_id': session, 'directory': str(directory),
                        'viewer': 'http://127.0.0.1:'+str(port)+'/', 'manifest': str(manifest)}
            if state['state'] in ('FAILED', 'STOPPED'):
                raise RuntimeError('Single mapping startup failed: '+str(state))
        if child.poll() is not None:
            raise RuntimeError('Supervisor exited; inspect '+str(runtime))
        time.sleep(.1)
    raise RuntimeError('Startup unconfirmed; inspect '+str(runtime)+' before retrying')


def inspect(session):
    directory = session_directory(session)
    result = {'session_id': session, 'directory': str(directory)}
    for key, path in {'experiment': directory/'experiment.json', 'input': directory/'input/status.json',
                      'monitor': directory/'monitor/status.json',
                      'runtime': RUN/'sessions'/session/'single_mapping/manifest.json'}.items():
        result[key] = read_json(ROOT, path) if path.exists() else None
    return result


def stop(session):
    manifest = project_path(ROOT, RUN/'sessions'/name(session)/'single_mapping/manifest.json')
    state = read_json(ROOT, manifest)
    stop_registered(manifest)
    deadline = time.monotonic()+shutdown_policy(state)['cli_stop_wait_s']
    while time.monotonic() < deadline:
        state = read_json(ROOT, manifest)
        if state['state'] != 'RUNNING':
            return inspect(session)
        time.sleep(.2)
    raise RuntimeError('Owned session did not finish cleanup; inspect '+str(manifest))


def preview(session, port):
    """Reopen an existing stopped monitor snapshot under a bounded process owner."""
    from .single_mapping_monitor import OfflineState
    directory = session_directory(session)
    state = inspect(session)
    if not state['runtime'] or state['runtime']['state'] == 'RUNNING':
        raise RuntimeError('Stop recording normally before opening its offline preview')
    if type(port) is not int or not 1024 <= port <= 65535:
        raise ValueError('Use an explicit unprivileged viewer port')
    OfflineState(directory/'monitor')
    import socket
    with socket.socket() as sock: sock.bind(('127.0.0.1', port))
    viewer_session = name(session[:40]+'_view_'+time.strftime('%H%M%S', time.gmtime()))
    command = ros_command([sys.executable, '-s', '-m', 'wc_runtime.single_mapping_monitor',
                           '--offline', directory/'monitor', '--port', str(port)])
    begin(viewer_session, 'single_map_viewer', [command], 10800,
          locks=['single-viewer-'+str(port)+'.lock'])
    return {'status': 'OFFLINE_PREVIEW_STARTED', 'source_session': session,
            'viewer_session': viewer_session, 'duration_s': 10800,
            'url': 'http://127.0.0.1:'+str(port)+'/', 'hardware_started': False}


def export(session):
    """Read only the normally closed native DB; write new independent export files."""
    state = inspect(session); directory = session_directory(session)
    manifest = state['runtime']
    if not manifest or manifest.get('state') != 'STOPPED' or manifest.get('exit_code') != 0 or manifest.get('cleanup_errors'):
        raise RuntimeError('Export requires normal closed capture; failed evidence remains available for diagnosis')
    if not state['input'] or state['input'].get('failure_reason') or not state['input'].get('published_frames'):
        raise RuntimeError('Export requires nonfailed archived lidar input')
    if not state['monitor'] or state['monitor'].get('failure'):
        raise RuntimeError('Export requires a nonfailed mapping monitor')
    db = project_path(ROOT, directory/'slam/rtabmap.db')
    if not db.is_file() or (db.parent/(db.name+'-wal')).exists():
        raise RuntimeError('Native DB missing or WAL still present; verify closure before export')
    before = hash_file(db)
    process_log = project_path(ROOT, RUN/'sessions'/session/'single_mapping/process-1.log').read_text(errors='replace')
    if not any('Saving database/long-term memory...done!' in line and str(db) in line for line in process_log.splitlines()):
        raise RuntimeError('Native RTAB-Map database closure was not confirmed in its log')
    import yaml
    metadata = project_path(ROOT, directory/'bag/metadata.yaml')
    if not metadata.is_file(): raise RuntimeError('Closed rosbag metadata missing')
    bag = yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
    bag_counts = {row['topic_metadata']['name']: int(row['message_count']) for row in bag['topics_with_message_count']}
    prefix = '/wc_mapping/single_'+state['experiment']['sensor_mode'].removeprefix('single_')
    source = '/wc_mapping/lidar_'+state['experiment']['sensor_mode'].removeprefix('single_')
    for topic in (source+'/source_frame', source+'/source_frame_filtered', prefix+'/scan_cloud',
                  prefix+'/odom', prefix+'/odom_info', prefix+'/cloud_map'):
        if bag_counts.get(topic, 0) <= 0: raise RuntimeError('Required original recording topic empty: '+topic)
    for value in bag['relative_file_paths']:
        bag_db = project_path(ROOT, directory/'bag'/value)
        with sqlite3.connect(bag_db.as_uri()+'?mode=ro', uri=True) as connection:
            if connection.execute('PRAGMA integrity_check').fetchall() != [('ok',)]:
                raise RuntimeError('Recorded rosbag SQLite integrity failed')
    with sqlite3.connect(db.as_uri()+'?mode=ro', uri=True) as connection:
        check = connection.execute('PRAGMA integrity_check').fetchall()
        if check != [('ok',)]: raise RuntimeError('Native SQLite integrity check failed')
        tables = {row[0] for row in connection.execute("SELECT name FROM sqlite_master WHERE type='table'")}
        nodes = connection.execute('SELECT COUNT(*) FROM Node').fetchone()[0] if 'Node' in tables else 0
        if not nodes: raise RuntimeError('Native DB has no retained map nodes')
    output = project_path(ROOT, directory/'export'); output.mkdir(exist_ok=False)
    export_db = output/'native_export_copy.db'
    with sqlite3.connect(db.as_uri()+'?mode=ro', uri=True) as source_db:
        with sqlite3.connect(export_db) as copy_db:
            source_db.backup(copy_db)
    command = [str(ros_setup_path().parent/'bin/rtabmap-export'), '--scan', '--cloud', '--poses', '--poses_format', '11',
               '--opt', '2', '--voxel', '0.03', '--output', 'single_'+state['experiment']['sensor_mode'].removeprefix('single_')+'_map',
               '--output_dir', str(output), str(export_db)]
    with (output/'export.log').open('xb') as log:
        completed = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, timeout=180)
    if completed.returncode or hash_file(db) != before:
        raise RuntimeError('Native export failed or changed closed DB; inspect '+str(output))
    files = {p.name: {'bytes': p.stat().st_size, 'sha256': hash_file(p)} for p in output.iterdir() if p.is_file()}
    vertices = {}
    for ply in output.glob('*.ply'):
        with ply.open('rb') as stream:
            if stream.readline().strip() != b'ply': raise RuntimeError('Invalid exported PLY header')
            for _ in range(200):
                line = stream.readline(4096).strip()
                if line.startswith(b'element vertex '): vertices[ply.name] = int(line.split()[-1])
                if line == b'end_header': break
            else: raise RuntimeError('Unterminated exported PLY header')
    if not vertices or not any(count > 0 for count in vertices.values()):
        raise RuntimeError('No nonempty exported 3D PLY produced')
    result = {'status': 'EXPORTED_EXPERIMENTAL_MAP', 'source_mode': 'real',
              'sensor_mode': state['experiment']['sensor_mode'], 'session_id': session,
              'native_database': str(db), 'database_sha256': before, 'retained_nodes': nodes,
              'output': str(output), 'files': files, 'command': command,
              'ply_vertex_counts': vertices, 'recorded_topic_counts': bag_counts,
              'original_database_close_confirmed': True, 'bag_sqlite_integrity': 'ok',
              'geometry_accuracy_validated': False, 'navigation_validated': False,
              'motion_route_completed': False, 'route_note': 'Export alone cannot establish a completed dynamic survey'}
    write_new(output/'result.json', json_bytes(result))
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    p = commands.add_parser('start'); p.add_argument('--session', required=True, type=name)
    p.add_argument('--side', choices=('left', 'right'), default='right'); p.add_argument('--duration', type=int, default=300)
    p.add_argument('--port', type=int, default=8770)
    p = commands.add_parser('preview'); p.add_argument('--session', required=True, type=name)
    p.add_argument('--port', type=int, default=8770)
    for command in ('inspect', 'stop', 'export'):
        commands.add_parser(command).add_argument('--session', required=True, type=name)
    args = parser.parse_args(argv)
    try:
        target()
        if args.command == 'start': result = start(args.session, args.side, args.duration, args.port)
        elif args.command == 'preview': result = preview(args.session, args.port)
        else: result = globals()[args.command](args.session)
        print(json.dumps(result, ensure_ascii=False, allow_nan=False)); return 0
    except (ValueError, RuntimeError, OSError, KeyError, subprocess.SubprocessError) as error:
        print(json.dumps({'status': 'FAILED', 'reason': str(error)}, ensure_ascii=False), file=sys.stderr); return 2


if __name__ == '__main__':
    raise SystemExit(main())
