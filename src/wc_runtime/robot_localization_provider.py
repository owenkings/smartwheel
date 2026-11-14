"""Actual installed robot_localization library through a hardware-free worker.

This module never substitutes a Python approximation when the native dependency
is unavailable. The planar policy and the 15-state model remain distinguishable.
"""
import atexit
import collections
import copy
import itertools
import os
from pathlib import Path
import select
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
    """One native worker process. It answers every line with one line, in order.

    The calling thread reads the answers itself; there is no reader thread or
    queue in between. post() sends a command without waiting for its answer.
    The next command() or sync() reads all outstanding answers in order before
    its own, so an ERROR still raises before any later value is used. A posted
    answer is either checked as an acknowledgement or handed to the callback
    given with the command. States collected by the garbage collector are
    dropped with the next command sent.
    """
    MAX_POSTED = 128  # bounded, so neither pipe fills while answers are unread

    def __init__(self):
        self.path = worker_path()
        self.process = subprocess.Popen([str(self.path)], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
        self.lock = threading.Lock()
        self.ids = itertools.count(1)
        self._input, self._output = self.process.stdin.fileno(), self.process.stdout.fileno()
        self._poll = select.poll()
        self._poll.register(self._output, select.POLLIN)
        self._received = bytearray()
        self._unsent = []
        # One entry per queued or written line still to be answered: True for an
        # acknowledgement (an ERROR answer raises), a callback for a posted
        # command that needs its answer, False for a command() answer left
        # unread by an interrupted call (skipped, keeping later answers aligned).
        self._pending = collections.deque()
        self._released = collections.deque()
        atexit.register(self.close)

    def release(self, handle):
        """Forget a collected state. Never blocks, so __del__ may call it from any thread."""
        self._released.append(handle)

    def _queue(self, values):
        while self._released:
            self._unsent.append('drop %s\n' % self._released.popleft())
            self._pending.append(True)
            if len(self._pending) >= self.MAX_POSTED:
                self._flush()
        self._unsent.append(' '.join(str(v) for v in values)+'\n')

    def _flush(self):
        error, _ = self._exchange()
        if error is not None:
            raise error

    def _write(self):
        data = memoryview(''.join(self._unsent).encode())
        self._unsent.clear()
        try:
            while data:
                data = data[os.write(self._input, data):]
        except BrokenPipeError as error:
            raise RuntimeError('robot_localization worker exited') from error

    def _readline(self):
        while True:
            end = self._received.find(b'\n')
            if end >= 0:
                line = self._received[:end].decode()
                del self._received[:end+1]
                return line
            if not self._poll.poll(10000):
                self.process.kill()
                raise TimeoutError('robot_localization worker did not acknowledge in 10 s')
            chunk = os.read(self._output, 65536)
            if not chunk:
                return 'ERROR native worker exited'
            self._received += chunk

    def _exchange(self):
        """Write queued lines, then read every outstanding answer; return the first failure and the last answer."""
        self._write()
        error = answer = None
        while self._pending:
            answer = self._readline()
            check = self._pending.popleft()
            if check is True:
                if error is None and answer.startswith('ERROR'):
                    error = RuntimeError(answer)
            elif check is not False:
                try:
                    check(answer)
                except Exception as failure:  # later answers are still read, keeping the stream aligned
                    if error is None:
                        error = failure
        return error, answer

    def post(self, *values, answer=None):
        """Send a command without waiting; answer(line) receives its answer, otherwise it must not be an ERROR."""
        with self.lock:
            self._queue(values)
            self._pending.append(True if answer is None else answer)
            if len(self._pending) >= self.MAX_POSTED:
                self._flush()
            else:
                self._write()  # the worker starts on it while the caller goes on

    def sync(self):
        """Write every queued command and read all outstanding answers."""
        with self.lock:
            if self._pending:
                if self.process.poll() is not None:
                    raise RuntimeError('robot_localization worker exited')
                self._flush()

    def command(self, *values):
        with self.lock:
            if self.process.poll() is not None:
                raise RuntimeError('robot_localization worker exited')
            self._queue(values)
            self._pending.append(False)
            error, response = self._exchange()
            if error is None and response.startswith('ERROR'):
                error = RuntimeError(response)
            if error is not None:
                raise error
            return response.split()

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                self.process.kill(); self.process.wait(timeout=2.)

def _update_entry(kind,sequence,sigma):
    """A last_update entry in its fixed key order; the native answer fills the None fields."""
    return {'sequence':sequence,'accepted':None,'nis':None,
        'threshold_nis':sigma*sigma,'candidate_sigma':sigma,
        'threshold_status':'EXPERIMENTAL_NOT_CALIBRATED','innovation':None,
        'measurement_dimensions':(['vx','vyaw'] if kind=='wheel' else ['vyaw'])+['z','roll','pitch','vz','vroll','vpitch','az'],
        'reason':None}

def _complete_update(entry,result,initial_measurement):
    if result[0] != 'UPDATE': raise RuntimeError('invalid native update response')
    accepted = bool(int(result[1]))
    entry.update(accepted=accepted,nis=float(result[2]),innovation=list(map(float,result[3:])),
        reason=('INITIAL_MEASUREMENT_INITIALIZATION' if initial_measurement else 'ACCEPTED') if accepted else 'MAHALANOBIS_GATE_REJECTED')
    return accepted

class _PostedUpdate:
    """An update sent without waiting. Its answer completes the shared entry;
    each state holding it adds it to its own counts afterwards."""
    __slots__ = ('kind','entry','initial','accepted')

    def __init__(self,kind,entry,initial):
        self.kind,self.entry,self.initial,self.accepted = kind,entry,initial,None

    def __call__(self,answer):
        if answer.startswith('ERROR'): raise RuntimeError(answer)
        self.accepted = _complete_update(self.entry,answer.split(),self.initial)

# deepcopy returns these unchanged; copy() skips the call for them.
_ATOMIC = frozenset({int,float,bool,str,type(None)})

class _Posted:
    """Returned by an update whose answer is read later; never None, so it reads as applied."""
    __slots__ = ()
    def __repr__(self): return 'POSTED'

POSTED = _Posted()

def _reference_pose(x,t_reference_axle):
    t=np.asarray(t_reference_axle,dtype=float)
    R=Rotation.from_euler('xyz',x[3:6]).as_matrix()
    pose=np.eye(4); pose[:3,:3]=R
    pose[:2,3]=x[:2]-(R@t)[:2]
    return pose,t

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
        self._counts = {'wheel':0,'gyro':0}; self._rejected_counts = {'wheel':0,'gyro':0}
        self._unfolded = ()  # posted updates not yet added to the counts, oldest first
        self.update_count = 0
        # MotionPrior's replays post updates and read their answers with the next
        # exchange; direct callers get each update's result immediately.
        self.posted_updates = False
        self.last_sequence = {'wheel':-1,'imu':-1}; self.last_update = {}
        self.gap_intervals = 0

    def _fold(self,wait=True):
        """Add posted updates to the counts; wait=False adds only those already answered."""
        unfolded = self._unfolded
        if not unfolded:
            return
        if wait and unfolded[-1].accepted is None:  # answers are read in order
            self.worker.sync()
        done = 0
        for update in unfolded:
            if update.accepted is None:
                break
            (self._counts if update.accepted else self._rejected_counts)[update.kind] += 1
            done += 1
        self._unfolded = unfolded[done:]
        if wait and self._unfolded:
            raise RuntimeError('robot_localization update answer missing')

    @property
    def counts(self):
        self._fold(); return self._counts
    @counts.setter
    def counts(self,value):
        self._fold(); self._counts = value
    @property
    def rejected_counts(self):
        self._fold(); return self._rejected_counts
    @rejected_counts.setter
    def rejected_counts(self,value):
        self._fold(); self._rejected_counts = value

    def settle(self):
        """Read every outstanding native answer now, so a native error raises here."""
        self.worker.sync(); self._fold()

    def __del__(self):
        worker, handle = getattr(self,'worker',None), getattr(self,'handle',None)
        if worker is not None and handle is not None:
            worker.release(handle)  # the native state is dropped with the next exchange

    # Shared by copies: never changed after __init__, or (_unfolded) an
    # immutable tuple of posted updates that every copy counts on its own.
    _SHARED = frozenset({'worker','config','process_covariance','_unfolded'})
    # Copied one level deep: each last_update entry is a new dict built by one
    # update, completed by its answer and by the caller of that update, and not
    # changed after that.
    _FLAT = frozenset({'_counts','_rejected_counts','last_sequence','last_update'})

    def copy(self):
        self._fold(wait=False)  # keeps _unfolded to the updates still unanswered
        result = object.__new__(type(self))
        result.__dict__ = {key: (value if key in self._SHARED or type(value) in _ATOMIC else
                                 dict(value) if key in self._FLAT else copy.deepcopy(value))
                           for key,value in self.__dict__.items()}
        result.handle = next(self.worker.ids)
        self.worker.post('clone',self.handle,result.handle)
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
        self.worker.post('predict',self.handle,float(dt),int(observed))
        if not observed and dt: self.gap_intervals += 1

    def decorrelate_gap(self): self.worker.post('decorrelate',self.handle)

    def _update(self,kind,sequence,values,variances,*,command_kind=None):
        stream = 'imu' if kind == 'gyro' else kind
        if sequence <= self.last_sequence[stream]: return None
        if not np.isfinite(values).all(): raise ValueError('finite native measurements required')
        sigma = self.config['innovation_gate_sigma']
        command = (command_kind or kind,self.handle,sigma,*values,*variances)
        entry = _update_entry(kind,sequence,sigma)
        if self.posted_updates:
            self._fold(wait=False)
            # Whether this is the lineage's first update depends only on how
            # many updates came before it, not on their answers.
            update = _PostedUpdate(kind,entry,self.update_count == 0)
            self.worker.post(*command,answer=update)
            self._unfolded += (update,)
            accepted = POSTED
        else:
            result = self.worker.command(*command)
            initial_measurement=sum(self.counts.values())==0 and sum(self.rejected_counts.values())==0
            accepted = _complete_update(entry,result,initial_measurement)
            (self.counts if accepted else self.rejected_counts)[kind] += 1
        self.update_count += 1
        self.last_sequence[stream]=sequence
        self.last_update[kind]=entry
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

    def pose(self,t_reference_axle):
        """The pose that output() returns first, without its Jacobian and covariances."""
        return _reference_pose(self.state()[0],t_reference_axle)[0]

    def output(self,t_reference_axle):
        x,p=self.state()
        pose,t=_reference_pose(x,t_reference_axle)
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
            'gap_intervals':self.gap_intervals,'process_covariance_diagonal':list(self.process_covariance),
            'two_d_mode':'RosFilter seven zero constraints, variance 1e-6',
            'startup_policy':'official processMeasurement initializes first updated dimensions from their measurement covariance; no startup gate',
            'gap_policy':'freeze pose, decorrelate pose/velocity, Q*dt covariance inflation',
            'noise_status':'UNVALIDATED_MODEL_WEIGHTS',
            'equivalence_status':'official ekf_node regression required on target; no hardware accuracy claim'}
