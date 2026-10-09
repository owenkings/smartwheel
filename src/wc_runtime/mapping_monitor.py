"""RViz health callbacks with bounded asynchronous output evidence writes."""
import argparse
from collections import deque
import copy
import json
import math
import os
from pathlib import Path
import signal
import threading
import time

import numpy as np

from .prepare_picker_input import project_path, read_json
from .mapping_shutdown import persistence_wait
from wc_sensors.pointcloud import decode_pointcloud2


def native_guess_values(guess):
    """Native null uses an all-zero quaternion; it is not an identity guess."""
    p,q=guess.translation,guess.rotation
    values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
    if not all(math.isfinite(v) for v in values):raise ValueError('Nonfinite native ICP guess')
    norm_squared=sum(v*v for v in values[3:])
    if norm_squared==0:return None
    if not math.isclose(norm_squared,1.0,rel_tol=0,abs_tol=1e-3):
        raise ValueError('Native ICP guess quaternion is not unit length')
    return values


def grid_arrays(message):
    """Validate the received ROS occupancy grid before archiving any pixels."""
    if message.header.frame_id!='mapping_map':raise ValueError('Unexpected grid frame')
    width,height=message.info.width,message.info.height
    if type(width) is not int or type(height) is not int or min(width,height)<=0 or \
            width*height!=len(message.data) or not 0<width*height<=10_000_000:
        raise ValueError('Invalid bounded occupancy grid size')
    resolution=message.info.resolution
    if not math.isfinite(resolution) or resolution<=0:
        raise ValueError('Grid resolution must be finite positive metres per cell')
    data=np.asarray(message.data)
    if data.dtype.kind not in 'iu' or not ((data>=-1)&(data<=100)).all():
        raise ValueError('Invalid occupancy values')
    p,q=message.info.origin.position,message.info.origin.orientation
    values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
    if not all(math.isfinite(v) for v in values):raise ValueError('Nonfinite grid origin')
    if abs(q.x)+abs(q.y)+abs(q.z)>1e-5 or abs(abs(q.w)-1)>1e-5:
        raise ValueError('Export expects native identity grid orientation')
    metadata={'resolution':float(resolution),'width':width,'height':height,
              'origin':[p.x,p.y,p.z],'frame_id':message.header.frame_id}
    return data.astype(np.int16).reshape(height,width),metadata


def _write_snapshot(root, kind, value, *, durable=False):
    """Disk-worker-only atomic replacement; runtime previews do not fsync."""
    filename = {'status': 'status.json', 'cloud': 'latest_cloud.npz', 'grid': 'latest_grid.npz'}[kind]
    path = root/filename
    temporary = path.with_name(path.name+'.tmp')
    with temporary.open('wb') as stream:
        if kind == 'status':
            stream.write((json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2)+'\n').encode('utf-8'))
        elif kind == 'cloud':
            np.savez(stream, xyz=value, frame_id='mapping_map')
        else:
            data, metadata = value
            np.savez(stream, data=data, metadata=json.dumps(metadata, allow_nan=False))
        stream.flush()
        if durable:
            os.fsync(stream.fileno())
    os.replace(temporary, path)


class AsyncHealthArchive:
    """FIFO tracking plus one replaceable deep snapshot per display output.

    The worker exclusively owns its file descriptors. Close drains every
    accepted tracking record and each latest map, synchronizes them, and only
    then publishes the final synchronized status. No ROS callback performs I/O.
    """
    def __init__(self, root, *, max_tracking=4096, write_snapshot=None):
        if type(max_tracking) is not int or not 1 <= max_tracking <= 100000:
            raise ValueError('Invalid bounded tracking archive capacity')
        self.root = Path(root)
        self.max_tracking = max_tracking
        self.write_snapshot = write_snapshot or _write_snapshot
        self.condition = threading.Condition()
        self.tracking = deque()
        self.latest = {}
        self.generations = {key: 0 for key in ('status', 'cloud', 'grid')}
        self.written_generations = dict(self.generations)
        self.tracking_accepted = self.tracking_written = 0
        self.closing = False
        self.final_status = None
        self.final = False
        self.failure = None
        self.in_flight = None
        self.max_write_duration_s = 0.
        self.cursor = 0
        self.thread = threading.Thread(target=self._run, name='mapping-health-archive', daemon=True)
        self.thread.start()

    def check(self):
        with self.condition:
            if self.failure:
                raise RuntimeError('Health archive failed: '+self.failure)

    def record(self, value):
        snapshot = copy.deepcopy(value)
        with self.condition:
            self.check()
            if self.closing:
                raise RuntimeError('Health archive already closing')
            if len(self.tracking) >= self.max_tracking:
                raise RuntimeError('Health tracking FIFO full; no accepted record discarded')
            self.tracking.append(snapshot)
            self.tracking_accepted += 1
            self.condition.notify()

    def snapshot(self, kind, value):
        if kind not in self.generations:
            raise ValueError('Unknown health snapshot kind')
        snapshot = copy.deepcopy(value)
        with self.condition:
            self.check()
            if self.closing:
                raise RuntimeError('Health archive already closing')
            self.generations[kind] += 1
            self.latest[kind] = (self.generations[kind], snapshot)
            self.condition.notify()

    def storage(self):
        with self.condition:
            return {'pending_tracking': len(self.tracking), 'tracking_accepted': self.tracking_accepted,
                    'tracking_written': self.tracking_written, 'pending_snapshots': sorted(self.latest),
                    'snapshot_generations_received': dict(self.generations),
                    'snapshot_generations_written': dict(self.written_generations),
                    'write_in_flight': self.in_flight, 'max_write_duration_s': self.max_write_duration_s,
                    'final': self.final, 'failure': self.failure,
                    'snapshot_policy': 'Latest status/cloud/grid coalesce; receive counts are not disk write counts.',
                    'durability': 'Runtime atomic previews; successful close synchronizes tracking, latest maps and final status.'}

    def _next(self):
        # Fair scheduling prevents a slow status/map write from starving FIFO
        # tracking, or sustained tracking from starving the latest map preview.
        kinds = ('tracking', 'cloud', 'grid', 'status')
        for offset in range(len(kinds)):
            index = (self.cursor+offset) % len(kinds)
            kind = kinds[index]
            if kind == 'tracking' and self.tracking:
                self.cursor = (index+1) % len(kinds)
                return kind, None, self.tracking.popleft()
            if kind in self.latest:
                generation, value = self.latest.pop(kind)
                self.cursor = (index+1) % len(kinds)
                return kind, generation, value
        return None

    def _run(self):
        try:
            with (self.root/'tracking.jsonl').open('x', encoding='utf-8') as records:
                while True:
                    with self.condition:
                        job = self._next()
                        while job is None and not self.closing:
                            self.condition.wait()
                            job = self._next()
                        if job is None:
                            break
                        kind, generation, value = job
                        self.in_flight = kind
                    started = time.monotonic()
                    if kind == 'tracking':
                        records.write(json.dumps(value, allow_nan=False, separators=(',', ':'))+'\n')
                        records.flush()
                    else:
                        self.write_snapshot(self.root, kind, value)
                    with self.condition:
                        self.in_flight = None
                        self.max_write_duration_s = max(self.max_write_duration_s, time.monotonic()-started)
                        if kind == 'tracking':
                            self.tracking_written += 1
                        else:
                            self.written_generations[kind] = generation
                records.flush()
                os.fsync(records.fileno())
            # Tracking is closed by its owner before final status is certified.
            for filename in ('latest_cloud.npz', 'latest_grid.npz'):
                path = self.root/filename
                if path.exists():
                    # Windows _commit rejects a read-only descriptor; this is
                    # our own completed snapshot and r+b does not alter bytes.
                    with path.open('r+b') as stream:
                        os.fsync(stream.fileno())
            with self.condition:
                final = copy.deepcopy(self.final_status)
                storage = self.storage()
                storage.update(final=True, pending_tracking=0, pending_snapshots=[], write_in_flight=None)
                final['storage'] = storage
            self.write_snapshot(self.root, 'status', final, durable=True)
            if hasattr(os, 'O_DIRECTORY'):
                descriptor = os.open(self.root, os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(descriptor)
                finally:
                    os.close(descriptor)
            with self.condition:
                self.final = True
        except BaseException as error:
            with self.condition:
                self.failure = str(error) or type(error).__name__
                self.in_flight = None
                self.condition.notify_all()

    def close(self, final_status, *, timeout_s=25.):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('Health close timeout must be finite positive')
        with self.condition:
            if not self.closing:
                self.final_status = copy.deepcopy(final_status)
                self.latest.pop('status', None)  # final status supersedes its pending preview
                self.closing = True
                self.condition.notify_all()
        self.thread.join(timeout_s)
        if self.thread.is_alive():
            raise RuntimeError('Health archive close timed out; final persistence incomplete')
        self.check()


def required_outputs(odometry_source):
    if odometry_source=='wheel_imu':
        return ('odom','prior_odom','grid_map','cloud_map')
    if odometry_source=='icp':
        return ('odom','odom_info','prior_odom','grid_map','cloud_map')
    raise ValueError('Unknown mapping odometry source')


class Health:
    def __init__(self, root, session, mode, clock=time.monotonic, *, archive_factory=AsyncHealthArchive,
                 odometry_source='icp', continuous_mapping=False):
        if type(continuous_mapping) is not bool:
            raise ValueError('continuous_mapping must be boolean')
        self.continuous_mapping=continuous_mapping
        self.required_outputs=required_outputs(odometry_source)
        self.odometry_source=odometry_source
        self.root, self.session, self.mode, self.clock = Path(root), session, mode, clock
        self.root.mkdir(exist_ok=False)
        self.started = clock(); self.last = {}; self.counts = {}; self.failure = None
        self.lost = self.total_lost = 0; self.map_points = 0; self.grid_cells = 0
        self.guess_nonidentity = 0; self.guess_samples = 0; self.stopped = False
        self.guess_null = 0; self.guess_valid = 0
        self.persistence_close_timeout_s = 25.
        self.archive = archive_factory(self.root)
        self.snapshot()

    def receive(self, key):
        self.last[key] = self.clock(); self.counts[key] = self.counts.get(key, 0)+1

    def fail(self, reason):
        self.failure = self.failure or str(reason)

    def snapshot(self, *, persist=True):
        value = {'schema_version': 1, 'session_id': self.session, 'mode': self.mode,
                 'odometry_source':self.odometry_source,'icp_tracking_required':self.odometry_source=='icp',
                 'status': 'FAILED' if self.failure else ('STOPPED' if self.stopped else 'RUNNING'),
                  'lifecycle': 'FAILED' if self.failure else ('STOPPED' if self.stopped else 'RUNNING'),
                  'map_quality_status': 'UNVALIDATED',
                  'data_status': 'FAILED' if self.failure else ('WAITING' if any(k not in self.last for k in self.required_outputs)
                      else ('STALE' if any(self.clock()-self.last[k] > 3 for k in self.required_outputs)
                            else 'OUTPUT_OBSERVED')),
                 'elapsed_s': self.clock()-self.started, 'failure': self.failure,
                 'counts': dict(self.counts), 'age_s': {k:self.clock()-v for k,v in self.last.items()},
                 'continuous_mapping':self.continuous_mapping,
                 'waiting_outputs':[k for k in self.required_outputs if k not in self.last],
                 'data_freshness_shutdown':not self.continuous_mapping,
                 'consecutive_lost': self.lost if self.odometry_source=='icp' else None,
                 'total_lost': self.total_lost if self.odometry_source=='icp' else None,
                 'map_points': self.map_points, 'grid_cells': self.grid_cells,
                 'persistence_close_timeout_s': self.persistence_close_timeout_s,
                 'icp_guess_samples': self.guess_samples, 'icp_guess_nonidentity_samples': self.guess_nonidentity,
                 'icp_guess_null_samples': self.guess_null, 'icp_guess_valid_samples': self.guess_valid,
                 'source_mode': 'real', 'formal_acceptance': False, 'navigation_validated': False,
                 'storage': self.archive.storage()}
        if persist:
            self.archive.snapshot('status', value)
        return value

    def tick(self):
        self.archive.check()
        if not self.continuous_mapping and self.clock()-self.started > 30:
            missing = [k for k in self.required_outputs if k not in self.last]
            if missing: self.fail('等待真实建图输出超时: '+', '.join(missing))
        for key in (k for k in self.required_outputs if k in ('odom','odom_info','prior_odom')):
            if not self.continuous_mapping and key in self.last and self.clock()-self.last[key] > 3:
                self.fail(key+' 超过 3 秒未更新')
        self.snapshot()
        return self.failure is None

    def receive_info(self, message):
        if self.odometry_source!='icp':raise ValueError('ICP OdomInfo is not part of wheel/IMU mapping')
        if message.header.frame_id!='mapping_odom':raise ValueError('Unexpected native OdomInfo frame')
        self.receive('odom_info');self.lost=self.lost+1 if message.lost else 0
        self.total_lost+=int(message.lost)
        if self.lost>=5:self.fail('ICP 连续五帧失去跟踪')
        values=native_guess_values(message.guess)
        self.guess_samples+=1
        if values is None:self.guess_null+=1
        else:
            self.guess_valid+=1
            if sum(v*v for v in values[:6])>1e-12:self.guess_nonidentity+=1
        self.archive.record({'stamp_ns':message.header.stamp.sec*10**9+message.header.stamp.nanosec,
            'lost':message.lost,'native_guess_xyz_xyzw':values,
            'icp_inliers_ratio':float(message.icp_inliers_ratio)})

    def receive_cloud(self, message):
        if message.header.frame_id!='mapping_map':raise ValueError('Unexpected map cloud frame')
        xyz=decode_pointcloud2(message)
        if len(xyz)>2_000_000:raise ValueError('Map snapshot exceeds 2 million point budget')
        self.map_points=int(np.isfinite(xyz).all(axis=1).sum());self.receive('cloud_map')
        self.archive.snapshot('cloud', xyz)

    def receive_grid(self, message):
        data,metadata=grid_arrays(message)
        self.grid_cells=data.size;self.receive('grid_map')
        self.archive.snapshot('grid', (data, metadata))

    def close(self, *, timeout_s=25.):
        self.persistence_close_timeout_s = persistence_wait(timeout_s)
        self.stopped = True
        try:
            self.archive.close(self.snapshot(persist=False), timeout_s=self.persistence_close_timeout_s)
        except Exception as error:
            self.fail(str(error))
            raise


def _parse_args(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session-root',type=Path,required=True)
    parser.add_argument('--close-timeout-s',type=persistence_wait,default=25.,
                        help='Stop-time health archive wait in (0,90] seconds; tracking limits unchanged')
    return parser.parse_args(argv)


def main(argv=None):
    args=_parse_args(argv)
    from .cli import ROOT,target
    target(); directory=project_path(ROOT,args.session_root)
    config=read_json(ROOT,directory/'runtime_config.json')
    state=Health(directory/'health',config['session_id'],config['mode'],
                 odometry_source=config.get('odometry_source','icp'),
                 continuous_mapping=config.get('continuous_mapping',False))
    state.persistence_close_timeout_s=args.close_timeout_s
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from nav_msgs.msg import Odometry,OccupancyGrid
    from sensor_msgs.msg import PointCloud2
    from rtabmap_msgs.msg import OdomInfo
    from visualization_msgs.msg import Marker,MarkerArray
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('mapping_app_health')
    stopped=False
    def stop(*_):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    qos=QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE)
    latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher=node.create_publisher(MarkerArray,'/wc_mapping/app/status_markers',latched)
    def guarded(fn):
        def receive(message):
            try:fn(message)
            except Exception as error:state.fail(type(error).__name__+': '+str(error))
        return receive
    def odom(message):
        if message.header.frame_id!='mapping_odom' or message.child_frame_id!='mapping_reference':
            raise ValueError('Unexpected native odometry frame')
        state.receive('odom')
    def prior(message):
        if message.header.frame_id!='prior_odom' or message.child_frame_id!='mapping_reference':
            raise ValueError('Unexpected prior odometry frame')
        state.receive('prior_odom')
    subscriptions=[node.create_subscription(Odometry,'/wc_mapping/app/odom',guarded(odom),qos),
        node.create_subscription(Odometry,'/wc_mapping/app/prior_odom',guarded(prior),qos),
        node.create_subscription(PointCloud2,'/wc_mapping/app/cloud_map',guarded(state.receive_cloud),latched),
        node.create_subscription(OccupancyGrid,'/wc_mapping/app/grid_map',guarded(state.receive_grid),latched)]
    if state.odometry_source=='icp':
        subscriptions.append(node.create_subscription(OdomInfo,'/wc_mapping/app/odom_info',guarded(state.receive_info),qos))
    last=0
    try:
        while not stopped and rclpy.ok() and state.failure is None:
            rclpy.spin_once(node,timeout_sec=.1)
            if time.monotonic()-last>=1:
                state.tick();last=time.monotonic()
                marker=Marker();marker.header.frame_id='mapping_map';marker.header.stamp=node.get_clock().now().to_msg()
                marker.ns='mapping_health';marker.id=0;marker.type=Marker.TEXT_VIEW_FACING;marker.action=Marker.ADD
                marker.pose.orientation.w=1.;marker.pose.position.z=1.;marker.scale.z=.12
                marker.color.r=1. if state.failure else .3;marker.color.g=.3 if state.failure else 1.;marker.color.b=.5;marker.color.a=1.
                motion=('WHEEL + IMU ODOM '+str(state.counts.get('odom',0))+' | ICP DISABLED'
                        if state.odometry_source=='wheel_imu' else
                        'ICP '+str(state.counts.get('odom',0))+' | LOST '+str(state.total_lost))
                marker.text=('MODE '+config['mode']+' | EXPERIMENT\nIMU + WHEEL: '+
                    ('RECEIVED' if state.counts.get('prior_odom') else 'INITIALIZING')+
                    '\n'+motion+
                    '\n3D '+str(state.map_points)+' | 2D '+str(state.grid_cells)+
                    ('\nFAILED: '+state.failure if state.failure else ''))
                publisher.publish(MarkerArray(markers=[marker]))
    except Exception as error:state.fail(str(error))
    finally:
        try:state.close(timeout_s=args.close_timeout_s)
        except Exception as error:
            state.fail(str(error));node.get_logger().error('Health archive close failed: '+str(error))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 1 if state.failure else 0


if __name__=='__main__':raise SystemExit(main())
