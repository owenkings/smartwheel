"""Actual installed robot_localization library through a hardware-free worker.

This module never substitutes a Python approximation when the native dependency
is unavailable. The planar policy and the 15-state model remain distinguishable.
"""
import atexit
import copy
import itertools
import os
from pathlib import Path
import queue
import subprocess
import threading

import numpy as np
from scipy.spatial.transform import Rotation

from .mapping_planar import validate_planar_config

STATE_ORDER = ['x','y','z','roll','pitch','yaw','vx','vy','vz','vroll','vpitch','vyaw','ax','ay','az']
PROJECTION = [0,1,5,6,11]
# Official ekf_node default Q, explicitly frozen for reproducibility. The two
# acceleration-state weights below adopt the same experimental input config;
# Q*dt in RL and acceleration GQG.T in five-state are different models.
OFFICIAL_Q = [.05,.05,.06,.03,.03,.06,.025,.025,.04,.01,.01,.02,.01,.01,.015]

def native_parameters(config, axle_xy=(0.,0.)):
    config = validate_planar_config(config)
    state = np.zeros(15); state[:2] = axle_xy
    covariance = np.full(15, 1e-6)
    covariance[[0,1,5]] = config['initial_pose_variance']
    covariance[[6,11]] = config['initial_velocity_variance']
    process = np.asarray(OFFICIAL_Q).copy()
    process[12] = config['linear_acceleration_variance']
    # Yaw acceleration is not a native RL state. Never invent one.
    return state, covariance, process

def worker_path():
    explicit = os.environ.get('WC_RL_WORKER')
    if explicit:
        path = Path(explicit)
    else:
        from ament_index_python.packages import get_package_prefix
        path = Path(get_package_prefix('wc_estimation'))/'lib/wc_estimation/wc_rl_worker'
    if not path.is_file():
        raise RuntimeError('actual robot_localization worker unavailable: '+str(path))
    return path

class _Worker:
    def __init__(self):
        self.path = worker_path()
        self.process = subprocess.Popen([str(self.path)], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                        text=True, bufsize=1)
        self.lock, self.responses = threading.Lock(), queue.Queue()
        self.ids = itertools.count(1)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()
        atexit.register(self.close)

    def _read(self):
        for line in self.process.stdout:
            self.responses.put(line.rstrip('\n'))
        self.responses.put('ERROR native worker exited')

    def command(self, *values):
        with self.lock:
            if self.process.poll() is not None:
                raise RuntimeError('robot_localization worker exited')
            self.process.stdin.write(' '.join(str(v) for v in values)+'\n')
            self.process.stdin.flush()
            try:
                response = self.responses.get(timeout=10.)
            except queue.Empty as error:
                self.process.kill()
                raise TimeoutError('robot_localization worker did not acknowledge in 10 s') from error
            if response.startswith('ERROR'):
                raise RuntimeError(response)
            return response.split()

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                self.process.kill(); self.process.wait(timeout=2.)

_worker = None
def get_worker():
    global _worker
    if _worker is None or _worker.process.poll() is not None:
        _worker = _Worker()
    return _worker

class RobotLocalizationEKF:
    def __init__(self, config, axle_xy=(0.,0.)):
        self.config = validate_planar_config(config)
        self.worker = get_worker(); self.handle = next(self.worker.ids)
        x,p,q = native_parameters(self.config, axle_xy)
        self.worker.command('new',self.handle,*x,*p,*q)
        self.process_covariance = q.tolist()
        self.counts = {'wheel':0,'gyro':0}; self.rejected_counts = {'wheel':0,'gyro':0}
        self.last_sequence = {'wheel':-1,'imu':-1}; self.last_update = {}
        self.gap_intervals = 0

    def __del__(self):
        worker, handle = getattr(self,'worker',None), getattr(self,'handle',None)
        if worker is not None and handle is not None and worker.process.poll() is None:
            try: worker.command('drop',handle)
            except (RuntimeError, OSError, TimeoutError): pass

    def copy(self):
        result = object.__new__(type(self))
        result.__dict__ = {key: (value if key == 'worker' else copy.deepcopy(value)) for key,value in self.__dict__.items()}
        result.handle = next(self.worker.ids)
        self.worker.command('clone',self.handle,result.handle)
        return result

    def state(self):
        result = self.worker.command('get',self.handle)
        if result[0] != 'STATE' or len(result) != 242:
            raise RuntimeError('invalid native state response')
        values=np.asarray(result[2:],dtype=float)
        if not np.isfinite(values).all():
            raise RuntimeError('nonfinite actual library state')
        return values[:15], values[15:].reshape(15,15)

    @property
    def x(self): return self.state()[0][PROJECTION]
    @property
    def P(self): return self.state()[1][np.ix_(PROJECTION,PROJECTION)]

    def predict(self,dt,*,observed=True):
        if not np.isfinite(dt) or dt<0:
            raise ValueError('nonnegative finite prediction interval required')
        self.worker.command('predict',self.handle,float(dt),int(observed))
        if not observed and dt: self.gap_intervals += 1

    def decorrelate_gap(self): self.worker.command('decorrelate',self.handle)

    def _update(self,kind,sequence,values,variances,*,command_kind=None):
        stream = 'imu' if kind == 'gyro' else kind
        if sequence <= self.last_sequence[stream]: return None
        if not np.isfinite(values).all(): raise ValueError('finite native measurements required')
        sigma = self.config['innovation_gate_sigma']
        result = self.worker.command(command_kind or kind,self.handle,sigma,*values,*variances)
        if result[0] != 'UPDATE': raise RuntimeError('invalid native update response')
        accepted = bool(int(result[1])); nis=float(result[2])
        initial_measurement=sum(self.counts.values())==0 and sum(self.rejected_counts.values())==0
        self.last_sequence[stream]=sequence
        (self.counts if accepted else self.rejected_counts)[kind] += 1
        self.last_update[kind]={'sequence':sequence,'accepted':accepted,'nis':nis,
            'threshold_nis':sigma*sigma,'candidate_sigma':sigma,
            'threshold_status':'EXPERIMENTAL_NOT_CALIBRATED',
            'innovation':list(map(float,result[3:])),
            'measurement_dimensions':(['vx','vyaw'] if kind=='wheel' else ['vyaw'])+['z','roll','pitch','vz','vroll','vpitch','az'],
            'reason':('INITIAL_MEASUREMENT_INITIALIZATION' if initial_measurement else 'ACCEPTED') if accepted else 'MAHALANOBIS_GATE_REJECTED'}
        return accepted

    def wheel(self,sequence,velocity,yaw_rate):
        return self._update('wheel',sequence,[velocity,yaw_rate],
                            [self.config['wheel_v_variance'],self.config['wheel_w_variance']])

    def wheel_covariance(self,sequence,velocity,yaw_rate,covariance):
        """Versioned full 2x2 wheel covariance; legacy wheel() is unchanged."""
        R = np.asarray(covariance,dtype=float)
        if R.shape != (2,2) or not np.isfinite(R).all() or not np.array_equal(R,R.T) or \
                np.linalg.eigvalsh(R).min() <= 0:
            raise ValueError('finite symmetric positive definite 2x2 wheel covariance required')
        if sequence <= self.last_sequence['wheel']: return None
        if not getattr(self.worker,'wheel_covariance_v1',False):
            capabilities = self.worker.command('capabilities')
            if capabilities != ['CAPABILITIES','2','wheel_cov_v1']:
                raise RuntimeError('installed worker lacks full wheel covariance protocol v1; rebuild wc_estimation')
            self.worker.wheel_covariance_v1 = True
        result = self._update('wheel',sequence,[velocity,yaw_rate],
                              [R[0,0],R[0,1],R[1,1]],command_kind='wheel_cov_v1')
        if result is not None:
            self.last_update['wheel'].update(measurement_covariance=R.tolist(),
                covariance_protocol='wheel_cov_v1',measurement=[float(velocity),float(yaw_rate)])
        return result

    def gyro(self,sequence,yaw_rate):
        return self._update('gyro',sequence,[yaw_rate],[self.config['gyro_w_variance']])

    def output(self,t_reference_axle):
        x,p=self.state(); t=np.asarray(t_reference_axle,dtype=float)
        R=Rotation.from_euler('xyz',x[3:6]).as_matrix()
        pose=np.eye(4); pose[:3,:3]=R
        pose[:2,3]=x[:2]-(R@t)[:2]
        # Match five-state reference-origin Z policy rather than pretending
        # the lidar has been placed on the axle origin.
        angular=x[9:12].copy(); linear=x[6:9]-np.cross(angular,t)
        J=np.zeros((6,15)); J[0,0]=J[1,1]=J[2,2]=1.
        J[3:6,3:6]=np.eye(3)
        for axis in range(3):
            delta=np.zeros(3); delta[axis]=1e-6
            rp=Rotation.from_euler('xyz',x[3:6]+delta).as_matrix()
            rm=Rotation.from_euler('xyz',x[3:6]-delta).as_matrix()
            J[:2,3+axis]=-((rp-rm)@t)[:2]/2e-6
        B=np.zeros((6,15)); B[:3,6:9]=np.eye(3); B[3:,9:12]=np.eye(3)
        B[:3,9:12]=np.array([[0.,-t[2],t[1]],[t[2],0.,-t[0]],[-t[1],t[0],0.]])
        return pose,linear,angular,J@p@J.T,B@p@B.T

    def report(self):
        x,p=self.state()
        return {'estimator':'robot_localization','implementation':'actual installed C++ Ekf library',
            'worker':str(self.worker.path),'native_state_order':STATE_ORDER,'native_state':x.tolist(),
            'native_covariance':p.tolist(),'state_order':['axle_x_m','axle_y_m','yaw_rad','vx_m_s','wz_rad_s'],
            'state':x[PROJECTION].tolist(),'covariance':p[np.ix_(PROJECTION,PROJECTION)].tolist(),
            'measurement_updates':dict(self.counts),'measurement_rejections':dict(self.rejected_counts),
            'last_sequence':dict(self.last_sequence),'last_update':copy.deepcopy(self.last_update),
            'gap_intervals':self.gap_intervals,'process_covariance_diagonal':self.process_covariance,
            'two_d_mode':'RosFilter seven zero constraints, variance 1e-6',
            'startup_policy':'official processMeasurement initializes first updated dimensions from their measurement covariance; no startup gate',
            'gap_policy':'freeze pose, decorrelate pose/velocity, Q*dt covariance inflation',
            'noise_status':'UNVALIDATED_MODEL_WEIGHTS',
            'equivalence_status':'official ekf_node regression required on target; no hardware accuracy claim'}
