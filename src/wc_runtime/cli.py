"""Project CLI. Hardware and processing are separate explicit operations."""
import argparse
import getpass
import json
import os
from pathlib import Path
import platform
import re
import shutil
import socket
import subprocess
import sys
import time

from .supervisor import stop_registered, write_json, shutdown_policy

ROOT = Path('/home/nvidia/wheelchair')
RUN = ROOT/'.phase1_runtime'
TOPICS = ['/wc_mapping/lidar_left/source_frame', '/wc_mapping/lidar_right/source_frame',
          '/wc_mapping/lidar_left/source_frame_filtered', '/wc_mapping/lidar_right/source_frame_filtered',
          '/wc_mapping/imu/source_frame', '/wc_mapping/imu/data_raw',
          '/wc_mapping/wheel/feedback_raw', '/wc_mapping/wheel/odom', '/wc_mapping/wheel/diagnostics',
          '/wc_mapping/wheel/odom_preview', '/wc_mapping/wheel/preview_diagnostics',
          '/wc_mapping/diagnostics', '/wc_mapping/lidar_left/diagnostics',
          '/wc_mapping/lidar_right/diagnostics', '/wc_mapping/imu/diagnostics']
H30_DEVICE = '/dev/smartwheel_h30_imu'
H30_BY_ID = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_0000000015-if00'
H30_SERIAL = '0000000015'


def target():
    if platform.machine() != 'aarch64' or getpass.getuser() != 'nvidia' or socket.gethostname() != 'ubuntu':
        raise RuntimeError('This operation requires the verified Orin target ubuntu/nvidia/aarch64')
    if not ROOT.is_dir() or ROOT.is_symlink():
        raise RuntimeError('Project root is missing or redirected')


def name(value):
    if not re.fullmatch('[A-Za-z0-9][A-Za-z0-9_.-]{0,63}',value):
        raise argparse.ArgumentTypeError('Use a simple ASCII session identifier')
    return value


def ros_command(arguments):
    # Positional arguments are passed independently; user text is never shell code.
    return ['bash','--noprofile','--norc','-c',
            'source /opt/ros/humble/setup.bash && source /home/nvidia/wheelchair/install/main/setup.bash && export PYTHONPATH=/home/nvidia/wheelchair/src${PYTHONPATH:+:$PYTHONPATH} && exec "$@"',
            'wc_phase1',*map(str,arguments)]


def environment():
    env = dict(PYTHONNOUSERSITE='1', PYTHONPATH=str(ROOT/'src'),
               ROS_DOMAIN_ID='83', ROS_LOCALHOST_ONLY='1')
    return env


def scoped_path(path, parent=None):
    from .storage_policy import resolve_storage_path
    value = resolve_storage_path(ROOT, path)
    if parent is not None:
        allowed = resolve_storage_path(ROOT, parent)
        if not value.is_relative_to(allowed):
            raise RuntimeError('Path outside allowed project area')
    return value


def begin(session, role, commands, duration, locks=(), allow_component_exit=False, *, recording=None):
    directory = RUN/'sessions'/session/role
    if directory.exists():
        raise RuntimeError('Session/role already exists; use a new session ID to preserve evidence')
    directory.mkdir(parents=True)
    plan=dict(session_id=session,role=role,commands=commands,environment=environment(),
              duration_s=duration,locks=list(locks),allow_component_exit=allow_component_exit)
    if recording is not None:
        plan['recording'] = recording
    if role in ('record', 'drivers'):
        # Sensor SDK shutdown includes device stop/joins and fixed sleeps.
        plan['sigint_grace_s'] = shutdown_policy(plan)['sigint_grace_s']
    elif role == 'cameras':
        # Four UVC releases can wait on shared USB link power management.
        # Allow the child's 5s normal + 1s TERM + 1s KILL budget to finish.
        plan['sigint_grace_s'] = 10.0
    write_json(directory/'plan.json',plan)
    log=(directory/'supervisor.log').open('ab',buffering=0)
    env=os.environ.copy()
    env.update(environment())
    process=subprocess.Popen([sys.executable,'-m','wc_runtime.supervisor','--plan',str(directory/'plan.json')],
        stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=env,cwd=ROOT)
    log.close()
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        if (directory/'manifest.json').exists():
            data=json.loads((directory/'manifest.json').read_text())
            if data['state']=='RUNNING':
                print(json.dumps(data,indent=2))
                return 0
            raise RuntimeError(data.get('stop_reason',data['state']))
        if process.poll() is not None:
            raise RuntimeError('Supervisor exited; inspect '+str(directory/'supervisor.log'))
        time.sleep(0.1)
    raise RuntimeError('Supervisor startup not confirmed; inspect session manifest before retrying')


def device_preflight():
    if shutil.disk_usage(ROOT).free<2*1024**3:
        raise RuntimeError('Less than 2 GiB free; acquisition refused')
    addresses=subprocess.check_output(['ip','-j','addr','show','dev','eno1'],text=True)
    present={item['local'] for interface in json.loads(addresses) for item in interface['addr_info']}
    if not {'192.168.0.100','192.168.1.100'}.issubset(present):
        raise RuntimeError('Expected existing receiver addresses absent; do not reconfigure automatically')
    sockets=subprocess.check_output(['ss','-H','-u','-l','-n'],text=True)
    if re.search(r'(?:^|\s)\S*:7687\s',sockets):
        raise RuntimeError('UDP 7687 already owned; inspect and coordinate its owner first')
    required=ROOT/'install/main/wc_xt_driver/lib/wc_xt_driver/xt_source_node'
    if not required.is_file():
        raise RuntimeError('Reviewed native driver has not been built')


def imu_preflight():
    """Read identity/ownership before scheduling any sensor; never open tty.

    The H30 process repeats its own identity and exclusive-lease checks at
    acquisition time. No device number or alternative serial port is guessed.
    """
    from wc_imu.ros_node import verify_device_identity, require_unoccupied
    actual, _ = verify_device_identity(H30_DEVICE, H30_BY_ID, H30_SERIAL)
    require_unoccupied(actual)
    return {'state': 'READ_ONLY_PREFLIGHT_PASS', 'device_alias': H30_DEVICE,
            'expected_by_id': H30_BY_ID, 'hardware_serial': H30_SERIAL,
            'resolved_device': str(actual), 'checked_unix_ns': time.time_ns(),
            'serial_opened': False}


def driver_command(args, probe):
    policy = getattr(args, 'device_config_policy', 'preserve_current')
    if policy not in ('preserve_current', 'apply_xtcfg'):
        raise ValueError('Unknown device configuration policy')
    return ros_command(['ros2','launch','wc_xt_driver','dual_sources.launch.py',
        'source_mode:='+args.mode,'session_id:='+args.session,'run_root:='+str(RUN),
        'allow_hardware:=true','read_only_probe:='+str(probe).lower(),
        'device_config_policy:='+policy,
        'publish_cloud_mirror:=true','max_runtime_seconds:='+str(args.duration)])


def main(argv=None):
    values = list(sys.argv[1:] if argv is None else argv)
    if values and values[0] in ('capture', 'compare'):
        if values[0] == 'capture':
            from .capture import main as operation
        else:
            from .mapping_compare import main as operation
        try:
            return operation(values[1:])
        except (OSError, ValueError, RuntimeError, KeyError) as exc:
            print(json.dumps({'status':'ERROR', 'command':values[0], 'reason':str(exc)}, ensure_ascii=False),file=sys.stderr)
            return 2
    parser=argparse.ArgumentParser(description='Dual XT-M60 mapping project; stop affects only project processes, not vehicle motion.')
    sub=parser.add_subparsers(dest='command',required=True)
    doctor=sub.add_parser('doctor',help='Read-only environment and configuration report')
    doctor.add_argument('--session-root', type=Path)
    doctor.add_argument('--verify-archive', action='store_true', help='Recompute source/index/bag and immutable file hashes')
    from .capture import PROFILES
    doctor.add_argument('--profile', choices=PROFILES, default='all_sensors')
    sub.add_parser('build',help='Locked target build of actual ROS packages')
    sub.add_parser('test',help='Run target test suite and preserve evidence')
    status=sub.add_parser('status');status.add_argument('--session',type=name)
    stop=sub.add_parser('stop');stop.add_argument('--session',required=True,type=name)
    for command in ('drivers','record'):
        p=sub.add_parser(command)
        p.add_argument('--session',required=True,type=name)
        p.add_argument('--mode',choices=('dual','single_left','single_right'),default='dual')
        p.add_argument('--duration',type=int,default=30)
        p.add_argument('--device-config-policy', choices=('preserve_current','apply_xtcfg'),
                       default='preserve_current',
                       help='Preserve device imaging/frequency settings by default; apply_xtcfg explicitly restores supplied imaging settings. Host filters use each side config in either capture policy.')
        if command=='drivers':
            p.add_argument('--probe',action='store_true',help='Read device identity/config without starting measurement')
        else:
            p.add_argument('--imu-poll-period-ms', type=float, choices=(2.5, 5.0, 10.0), default=10.0,
                           help='Host serial polling period; does not change arrival-only timestamp meaning')
            p.add_argument('--lidar-only',action='store_true',
                           help='Explicit dual-lidar recording without H30 acquisition or IMU topics; requires --mode dual')
    mapping=sub.add_parser('map')
    mapping.add_argument('--session-config',type=Path,required=True)
    mapping.add_argument('--duration',type=int,default=0)
    mapping.add_argument('--with-synthetic-source',action='store_true')
    mapping.add_argument('--icp-only',action='store_true',help='Diagnostic processing without graph, not phase-1 acceptance')
    wheel=sub.add_parser('wheel',help='Decode existing passive feedback; no serial or controller commands')
    wheel.add_argument('--session',required=True,type=name)
    wheel.add_argument('--config',type=Path,default=ROOT/'config/wheel_unvalidated.json')
    wheel.add_argument('--duration',type=int,default=0)
    wheel.add_argument('--use-sim-time',action='store_true',help='Use bag /clock for existing feedback replay')
    cameras=sub.add_parser('cameras',help='Four USB camera previews with explicit direction-to-port configuration')
    cameras.add_argument('--session',required=True,type=name)
    cameras.add_argument('--config',type=Path,default=ROOT/'config/cameras.json')
    cameras.add_argument('--profile',choices=('monitor_320','detail_640','detail_640_10fps'),default='monitor_320')
    cameras.add_argument('--duration',type=int,default=3600)
    encoder=sub.add_parser('encoder',help='Exclusive fixed FC03 feedback capture; no motor control')
    encoder.add_argument('--session',required=True,type=name)
    encoder.add_argument('--config',type=Path,default=ROOT/'config/wheel_feedback_current.json')
    encoder.add_argument('--duration',type=int,default=30)
    encoder.add_argument('--rate-hz',type=int,choices=(10,20,50),default=10)
    encoder.add_argument('--allow-read-queries',action='store_true')
    encoder.add_argument('--preview-history-calibration',action='store_true',
                         help='Explicit unvalidated preview using historical wheel scale and geometry')
    rviz=sub.add_parser('rviz')
    rviz.add_argument('--view',choices=('2d','3d','left','right','cameras'),default='3d')
    rviz.add_argument('--session',type=name,required=True)
    for command in ('calibrate','save','load','verify','inspect','goals','list'):
        p=sub.add_parser(command,add_help=False)
        p.add_argument('arguments',nargs=argparse.REMAINDER)
    replay=sub.add_parser('replay')
    replay.add_argument('--session',required=True,type=name)
    replay.add_argument('--bag',type=Path,required=True)
    args,unknown=parser.parse_known_args(argv)
    if unknown and args.command not in ('calibrate','save','load','verify','inspect','goals','list'):
        parser.error('unrecognized arguments: '+' '.join(unknown))
    try:
        target()
        if args.command=='doctor':
            if args.session_root:
                if args.verify_archive:
                    from .capture_audit import main as audit
                    return audit(['--dataset',str(scoped_path(args.session_root))])
                from .runtime_health import collect_session_health
                print(json.dumps(collect_session_health(scoped_path(args.session_root)), ensure_ascii=False, indent=2))
                return 0
            if args.verify_archive:
                raise ValueError('--verify-archive requires --session-root')
            from importlib.util import find_spec
            report=dict(hostname=socket.gethostname(),user=getpass.getuser(),architecture=platform.machine(),
                project_root=str(ROOT),ros_distro='humble',free_bytes=shutil.disk_usage(ROOT).free,
                python=platform.python_version(),modules={k:find_spec(k) is not None for k in ('numpy','scipy','rclpy','pytest')},
                live_configuration=json.loads((ROOT/'config/live_unvalidated.json').read_text()),
                old_workspace_reference='READ_ONLY_REVIEW_AUTHORIZED_20260912; old launchers not executed',source_mode='No acquisition')
            from .hardware_setup import resolve_hardware_setup
            from .capture import preflight
            report['geometry'] = resolve_hardware_setup(json.loads((ROOT/'config/hardware_setup.json').read_text(encoding='utf-8'))).get('geometry_report', {})
            report['capture_preflight'] = preflight(ROOT, args.profile, 30)
            from .capture import source_selection
            report['capture_profile'] = args.profile
            report['capture_source_selection'] = source_selection(args.profile)
            print(json.dumps(report,indent=2))
            return 0
        if args.command=='build':
            (RUN/'locks').mkdir(parents=True,exist_ok=True)
            command=['flock','-n',str(RUN/'locks/heavy_build.lock'),'colcon','--log-base',str(scoped_path('reports/builds/colcon')),
                'build','--base-paths',str(ROOT/'src'),'--build-base',str(ROOT/'build/main'),
                '--install-base',str(ROOT/'install/main'),'--executor','sequential','--event-handlers','console_direct+']
            env=os.environ.copy();env.update(environment());env['CMAKE_BUILD_PARALLEL_LEVEL']='2';env['MAKEFLAGS']='-j2'
            return subprocess.call(['bash','--noprofile','--norc','-c','source /opt/ros/humble/setup.bash && exec "$@"','wc_build',*command],env=env,cwd=ROOT)
        if args.command=='test':
            return subprocess.call(ros_command(['bash',ROOT/'tests/run_target_tests.sh']),cwd=ROOT)
        if args.command in ('status','stop'):
            directory=RUN/'sessions'
            paths=sorted((directory/args.session).glob('*/manifest.json')) if args.session else sorted(directory.glob('*/*/manifest.json'))
            if args.command=='stop':
                if not paths:raise RuntimeError('No matching managed session')
                wait_s=max(shutdown_policy(json.loads(path.read_text()))['cli_stop_wait_s'] for path in paths)
                for path in paths:stop_registered(path)
                deadline=time.monotonic()+wait_s
                while time.monotonic()<deadline and any(json.loads(p.read_text()).get('state')=='RUNNING' for p in paths):time.sleep(0.2)
            data=[json.loads(p.read_text()) for p in paths]
            print(json.dumps(data,indent=2))
            return 1 if args.command=='stop' and any(x['state']!='STOPPED' or x.get('exit_code',0)!=0 for x in data) else 0
        if args.command in ('drivers','record'):
            if not 1<=args.duration<=3600:raise RuntimeError('Acquisition duration must be 1..3600 seconds')
            recording=None
            if args.command=='record':
                if args.lidar_only and args.mode!='dual':
                    raise RuntimeError('--lidar-only requires --mode dual; both lidar sources must remain enabled')
                # Fail before driver construction/begin or any sensor process.
                # Explicit lidar-only is never selected because H30 is missing.
                imu_check = ({'state':'NOT_REQUESTED','reason':'explicit --lidar-only','serial_opened':False}
                             if args.lidar_only else imu_preflight())
            device_preflight()
            commands=[driver_command(args,args.command=='drivers' and args.probe)]
            if args.command=='record':
                output=scoped_path(Path('data/bags')/args.session)
                if output.exists():raise RuntimeError('Bag path already exists')
                output.parent.mkdir(parents=True,exist_ok=True)
                topics=list(TOPICS) if args.mode=='dual' else [topic for topic in TOPICS if 'lidar_'+('right' if args.mode=='single_left' else 'left') not in topic]
                if args.lidar_only:
                    topics=[topic for topic in topics if not topic.startswith('/wc_mapping/imu/')]
                commands.insert(0,ros_command(['ros2','bag','record','-s','sqlite3','--max-bag-size','536870912',
                    '-o',str(output),*topics]))
                if not args.lidar_only:
                    commands.append(ros_command([sys.executable,'-m','wc_imu.ros_node',
                        '--device',H30_DEVICE,'--expected-by-id',H30_BY_ID,
                        '--hardware-serial',H30_SERIAL,'--sensor-id','H30-'+H30_SERIAL,
                        '--session-id',args.session,'--run-root',str(RUN),
                        '--duration',str(args.duration),'--stale-timeout','2.0',
                        '--poll-period-ms',str(args.imu_poll_period_ms)]))
                recording={'mode':'lidar_only' if args.lidar_only else 'lidar_with_h30',
                    'sensor_mode':args.mode,'imu_included':not args.lidar_only,
                    'device_config_policy':args.device_config_policy,
                    'explicit_lidar_only':args.lidar_only,'topics':topics,'imu_preflight':imu_check,
                    'imu_poll_period_ms':None if args.lidar_only else args.imu_poll_period_ms}
            options={'recording':recording} if recording is not None else {}
            return begin(args.session,args.command,commands,args.duration+3,['sensor_owner.lock','domain-83-source.lock'],True,**options)
        if args.command=='map':
            if args.duration<0:raise RuntimeError('Mapping duration must be nonnegative')
            path=scoped_path(args.session_config)
            config=json.loads(path.read_text())
            session=name(config['session_id'])
            if config['sensor_mode']!='dual':raise RuntimeError('Formal map command requires dual sources')
            if config['source_mode']=='real' and (config['calibration']['status']!='VALIDATED' or config.get('quality_profile',{}).get('status')!='VALIDATED'):
                raise RuntimeError('Live map blocked: independently validated extrinsics, time and quality required')
            commands=[ros_command(['ros2','launch','wc_bringup','processing.launch.py','session_config:='+str(path),
                                  'graph_ack_required:='+('false' if args.icp_only else 'true')])]
            if not args.icp_only:
                commands += [ros_command(['ros2','run','wc_slam','wc_graph_node','--ros-args','-p','session_config:='+str(path)])]
                commands += [ros_command([sys.executable,'-m','wc_maps.live_map','--session-config',str(path)])]
            if args.with_synthetic_source:
                if config['source_mode']!='synthetic':raise RuntimeError('Synthetic source cannot feed a real session')
                commands.append(ros_command([sys.executable,ROOT/'tests/integration/synthetic_source.py','--session-config',str(path)]))
            locks=['map-'+session+'.lock','domain-83-processing.lock']
            if args.with_synthetic_source:locks.append('domain-83-source.lock')
            return begin(session,'map',commands,args.duration,locks,bool(args.with_synthetic_source))
        if args.command=='wheel':
            if args.duration<0:raise RuntimeError('Wheel processing duration must be nonnegative')
            path=scoped_path(args.config)
            from wc_motion.ros_node import FeedbackProcessor
            FeedbackProcessor(json.loads(path.read_text()))
            output=scoped_path(Path('data/wheel_feedback')/(args.session+'.jsonl'))
            output.parent.mkdir(parents=True,exist_ok=True)
            if output.exists():raise RuntimeError('Wheel feedback output already exists')
            extra=['--ros-args','-p','use_sim_time:=true'] if args.use_sim_time else []
            return begin(args.session,'wheel',[ros_command([sys.executable,'-m','wc_motion.ros_node',
                '--config',str(path),'--raw-output',str(output),*extra])],args.duration,['domain-83-wheel.lock'])
        if args.command=='cameras':
            if not 1<=args.duration<=43200:raise RuntimeError('Camera duration must be 1..43200 seconds')
            path=scoped_path(args.config)
            from wc_cameras.config import load_config, ROLES
            load_config(path)
            commands=[ros_command([sys.executable,'-m','wc_cameras.node','--config',str(path),
                '--role',role,'--profile',args.profile,'--session-id',args.session,
                '--run-root',str(RUN),'--duration',str(args.duration)]) for role in ROLES]
            return begin(args.session,'cameras',commands,args.duration+3,['domain-83-cameras.lock'],True)
        if args.command=='encoder':
            if not args.allow_read_queries:raise RuntimeError('Explicit --allow-read-queries required')
            if not 1<=args.duration<=300:raise RuntimeError('Encoder duration must be 1..300 seconds')
            path=scoped_path(args.config)
            from wc_motion.feedback_transport import validate_config
            validate_config(json.loads(path.read_text()))
            output=scoped_path(Path('data/wheel_feedback')/(args.session+'.transactions.jsonl'))
            output.parent.mkdir(parents=True,exist_ok=True)
            if output.exists():raise RuntimeError('Encoder evidence already exists')
            extra=['--preview-history-calibration'] if args.preview_history_calibration else []
            commands=[ros_command([sys.executable,'-m','wc_motion.feedback_transport','--config',str(path),
                '--output',str(output),'--run-root',str(RUN),'--samples',str(args.duration*args.rate_hz),
                '--rate-hz',str(args.rate_hz),'--allow-read-queries','--publish-ros',*extra])]
            return begin(args.session,'encoder',commands,args.duration+3,['domain-83-encoder.lock'],True)
        if args.command=='rviz':
            env=os.environ.copy();env.update(environment())
            if not env.get('DISPLAY'):
                for comm in Path('/proc').glob('[0-9]*/comm'):
                    try:
                        if comm.stat().st_uid==os.getuid() and comm.read_text().strip()=='gnome-shell':
                            for item in (comm.parent/'environ').read_bytes().split(b'\0'):
                                if item.startswith((b'DISPLAY=',b'XAUTHORITY=')):
                                    key,value=item.decode().split('=',1);env[key]=value
                    except (OSError,UnicodeError):pass
            if not env.get('DISPLAY'):raise RuntimeError('No authorized graphical session was found')
            view='cameras.rviz' if args.view=='cameras' else ('sensor_' if args.view in ('left','right') else 'mapping_')+args.view+'.rviz'
            return subprocess.call(ros_command(['rviz2','-d',ROOT/'config/rviz'/view]),env=env)
        if args.command=='replay':
            bag=scoped_path(args.bag,ROOT/'data'/'bags')
            # Replay measured envelopes; regenerate wheel odometry from feedback.
            # Historical TF, derived odometry and control topics are excluded.
            replay_topics=[topic for topic in TOPICS if topic not in
                ('/wc_mapping/wheel/odom','/wc_mapping/wheel/odom_preview',
                 '/wc_mapping/wheel/preview_diagnostics','/wc_mapping/wheel/diagnostics')]
            return begin(args.session,'replay',[ros_command(['ros2','bag','play',str(bag),'--clock','--topics',*replay_topics])],0,['domain-83-source.lock'],True)
        if args.command=='calibrate':
            from wc_calibration.cli import main as calibrate
            return calibrate([*unknown,*args.arguments])
        from wc_maps.__main__ import main as maps
        return maps(['--root',str(scoped_path('maps')),args.command,*unknown,*args.arguments])
    except (OSError,ValueError,RuntimeError,KeyError) as exc:
        print(json.dumps({'status':'ERROR','reason':str(exc)},ensure_ascii=False),file=sys.stderr)
        return 2


if __name__=='__main__':
    raise SystemExit(main())
