"""Isolated AMCL functionality replay; no independent accuracy claim or motion.

The replay base is the recorded lidar reference or an explicitly configured,
unvalidated planar axle reference. Neither is a validated physical base_link.
A supplied initial map pose is used once; global relocalization is
not tested. Map reuse from the same recording is explicitly declared.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
from numbers import Real
import subprocess
import time
import traceback
import uuid

import numpy as np


class AMCLReplayError(RuntimeError):
    pass


SCAN_SETTINGS = {
    'angle_min_rad': -math.pi / 3, 'angle_max_rad': math.pi / 3,
    'angle_increment_rad': math.pi / 240, 'beams': 161,
    'range_min_m': .30, 'range_max_m': 8.0,
    'height_min_m': -.50, 'height_max_m': .50,
    'missing_return': 'POSITIVE_INFINITY_NO_INVENTED_OBSTACLE_OR_FREE_ENDPOINT',
    'height_basis': 'EXPLICIT_BAND_IN_PREPARED_LIDAR_REFERENCE_NOT_MEASURED_FLOOR',
    'per_point_acquisition_timing_known': False,
}

EVALUATION_SCOPES = ('SAME_RECORDING_MAP_REUSE_NOT_INDEPENDENT_HOLDOUT',
                     'DISJOINT_TIME_TRAINING_MAP_SHARED_ODOMETRY_NOT_ABSOLUTE_TRUTH',
                     'SYNTHETIC_KNOWN_TRUTH_NOT_PHYSICAL_ACCURACY',
                     'CROSS_RECORDING_SAVED_MAP_ALIGNMENT_PRIOR_NOT_ABSOLUTE_TRUTH')
# Default: force one filter update per original scan. The opt-in alternative
# leaves AMCL's own update_min_d/update_min_a gating active, as on a vehicle;
# every processed scan is then acknowledged by AMCL's map->odom broadcast.
UPDATE_POLICIES = ('REQUEST_NOMOTION_UPDATE_BEFORE_EACH_ORIGINAL_SCAN', 'AMCL_MOTION_GATED_UPDATES')
# Model parameters a caller may set explicitly. Frames, topics, clock, map and
# initial-pose handling stay owned by the replay; the laser model has its own argument.
AMCL_OVERRIDE_TYPES = {
    'alpha1': float, 'alpha2': float, 'alpha3': float, 'alpha4': float, 'alpha5': float,
    'beam_skip_distance': float, 'beam_skip_error_threshold': float, 'beam_skip_threshold': float,
    'do_beamskip': bool, 'lambda_short': float, 'laser_likelihood_max_dist': float,
    'laser_max_range': float, 'laser_min_range': float, 'max_beams': int, 'max_particles': int,
    'min_particles': int, 'pf_err': float, 'pf_z': float, 'recovery_alpha_fast': float,
    'recovery_alpha_slow': float, 'resample_interval': int, 'sigma_hit': float,
    'transform_tolerance': float, 'update_min_a': float, 'update_min_d': float,
    'z_hit': float, 'z_max': float, 'z_rand': float, 'z_short': float,
}
DEFAULT_INITIAL_STD = (.20, .20, math.radians(5.0))


def amcl_parameter_overrides(values):
    """Validate explicit AMCL model overrides; coerce to the declared ROS parameter types."""
    if values is None:
        return {}
    if not isinstance(values, dict):
        raise ValueError('AMCL overrides must be a mapping')
    result = {}
    for name, value in values.items():
        kind = AMCL_OVERRIDE_TYPES.get(name)
        if kind is None:
            raise ValueError('AMCL parameter is not overridable here: ' + repr(name))
        if kind is bool:
            if type(value) is not bool:
                raise ValueError(name + ' must be a boolean')
            result[name] = value
            continue
        if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real) or not math.isfinite(value):
            raise ValueError(name + ' must be a finite number')
        if kind is int:
            if float(value) != int(value) or not 1 <= int(value) <= 100_000:
                raise ValueError(name + ' must be a positive integer')
            result[name] = int(value)
        else:
            if float(value) < 0:
                raise ValueError(name + ' must be nonnegative')
            result[name] = float(value)
    if result.get('min_particles', 1) > result.get('max_particles', 100_000):
        raise ValueError('min_particles cannot exceed max_particles')
    return result


def initial_standard_deviations(value):
    """Operator-style initial uncertainty (x m, y m, yaw rad); default 0.20 m, 0.20 m, 5 degrees."""
    if value is None:
        return DEFAULT_INITIAL_STD
    try:
        if isinstance(value, (str, bytes, dict)) or len(value) != 3:
            raise ValueError
        if any(isinstance(v, (bool, np.bool_)) or not isinstance(v, Real) for v in value):
            raise ValueError
        std = tuple(float(v) for v in value)
    except (TypeError, ValueError) as exc:
        raise ValueError('initial standard deviations require three finite numbers') from exc
    if not all(math.isfinite(v) and v > 0 for v in std) or max(std[:2]) > 2.0 or std[2] > math.pi:
        raise ValueError('initial standard deviations must be positive, xy <= 2 m and yaw <= pi')
    return std


def _height_band(value):
    try:
        if isinstance(value, (str, bytes, dict)) or len(value) != 2:
            raise ValueError
        if any(isinstance(v, (bool, np.bool_)) or not isinstance(v, Real) for v in value):
            raise ValueError
        low, high = map(float, value)
    except (TypeError, ValueError) as exc:
        raise ValueError('scan height requires two finite numeric bounds') from exc
    if not all(math.isfinite(v) for v in (low, high)) or not -2 < low < high < 2:
        raise ValueError('scan height must satisfy finite -2 < minimum < maximum < 2 meters')
    return low, high


def build_scan(points, height_min_m=-.50, height_max_m=.50):
    """Project prepared FLU XYZ into 0.75-degree nearest-observed range bins."""
    height_min, height_max = _height_band((height_min_m, height_max_m))
    xyz = np.asarray(points)
    if (xyz.ndim != 2 or xyz.shape[1] != 3 or not 1 <= len(xyz) <= 100_000
            or xyz.dtype.kind not in 'fiu'):
        raise ValueError('scan source must be a bounded numeric Nx3 array')
    xyz = xyz.astype(float, copy=False)
    finite = np.isfinite(xyz).all(axis=1)
    good = xyz[finite]
    horizontal = np.hypot(good[:, 0], good[:, 1])
    angles = np.arctan2(good[:, 1], good[:, 0])
    accepted = ((horizontal >= .30) & (horizontal <= 8.0)
                & (good[:, 2] >= height_min) & (good[:, 2] <= height_max)
                & (angles >= -math.pi / 3) & (angles <= math.pi / 3))
    indices = np.floor((angles[accepted] + math.pi / 3) / (math.pi / 240) + .5).astype(np.int64)
    if np.any(indices < 0) or np.any(indices >= 161):
        raise ValueError('scan angular bin arithmetic exceeded declared field of view')
    ranges = np.full(161, math.inf, dtype=np.float32)
    np.minimum.at(ranges, indices, horizontal[accepted].astype(np.float32))
    observed = np.isfinite(ranges)
    settings = {**SCAN_SETTINGS, 'height_min_m': height_min, 'height_max_m': height_max}
    stats = {'input_rows': len(xyz), 'finite_rows': int(finite.sum()),
             'accepted_rows': int(accepted.sum()), 'observed_bins': int(observed.sum()),
             'missing_bins': int((~observed).sum()),
             'minimum_observed_range_m': float(ranges[observed].min()) if observed.any() else None,
             'maximum_observed_range_m': float(ranges[observed].max()) if observed.any() else None,
             'projection': 'NEAREST_ACTUAL_OBSERVATION_PER_AZIMUTH_BIN',
             'scan_settings': settings}
    return ranges, stats


def _transform(value, label):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-10, rtol=0)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6, rtol=0)
            or not math.isclose(float(np.linalg.det(matrix[:3, :3])), 1, abs_tol=1e-6)):
        raise ValueError(label + ' must be a finite rigid 4x4 transform')
    return matrix.copy()


def planar_base_from_lidar(value):
    """Validate an explicitly supplied SE(2) lever arm; never drop a Z term."""
    matrix = _transform(value, 'base_from_lidar')
    if (not np.allclose(matrix[2], [0, 0, 1, 0], atol=1e-10, rtol=0)
            or not np.allclose(matrix[:2, 2], [0, 0], atol=1e-10, rtol=0)):
        raise ValueError('base_from_lidar must be explicit planar SE(2); no Z translation or roll/pitch projection')
    return matrix


def base_pose_from_lidar(lidar_pose, base_from_lidar=None):
    """Convert T_parent_lidar to T_parent_base with an explicit fixed lever arm."""
    pose = _transform(lidar_pose, 'lidar_pose')
    return pose if base_from_lidar is None else pose @ np.linalg.inv(planar_base_from_lidar(base_from_lidar))


def prepare_frames(frames, initial_pose, scan_height_m=(-.50, .50)):
    if not isinstance(frames, (list, tuple)) or not 2 <= len(frames) <= 2000:
        raise ValueError('two to 2000 explicitly ordered replay frames required')
    height_min, height_max = _height_band(scan_height_m)
    initial = _transform(initial_pose, 'initial_pose')
    prepared = []
    previous = -1
    total_points = 0
    for frame in frames:
        if not isinstance(frame, dict):
            raise ValueError('replay frame must be an object')
        stamp = frame.get('stamp_ns')
        if type(stamp) is not int or not 0 < stamp < 2**31 * 10**9 or stamp <= previous:
            raise ValueError('replay source stamps must be increasing positive ROS integer nanoseconds')
        previous = stamp
        ranges, stats = build_scan(frame.get('points'), height_min, height_max)
        total_points += stats['input_rows']
        if total_points > 10_000_000:
            raise ValueError('replay point budget exceeded')
        if stats['observed_bins'] < 3:
            raise ValueError('replay frame has fewer than three observed scan bins: ' + str(stamp))
        prepared.append({'stamp_ns': stamp, 'ranges': ranges,
                         'scan_stats': stats, 'odometry_T': _transform(frame.get('odometry_T'), 'odometry_T')})
    return prepared, initial


def validate_isolation():
    if os.name != 'posix':
        raise AMCLReplayError('AMCL replay requires the supported Linux target')
    if os.environ.get('ROS_DOMAIN_ID') != '95' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise AMCLReplayError('Set ROS_DOMAIN_ID=95 and ROS_LOCALHOST_ONLY=1 in a fresh process')
    if os.environ.get('ROS_DISCOVERY_SERVER'):
        raise AMCLReplayError('AMCL replay must not use an external discovery server')


def _sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def run_amcl_replay(map_yaml, frames, initial_pose, output_dir, timeout_s=180,
                    *, scan_height_m=(-.50, .50), base_from_lidar=None, laser_model='likelihood_field',
                    evaluation_scope='SAME_RECORDING_MAP_REUSE_NOT_INDEPENDENT_HOLDOUT',
                    amcl_overrides=None, initial_std=None,
                    update_policy='REQUEST_NOMOTION_UPDATE_BEFORE_EACH_ORIGINAL_SCAN'):
    """Run map_server + AMCL on original-stamp scans and frozen prior odometry.

    Returns a JSON-serializable result, including partial evidence on runtime
    failure. Invalid input/isolation fails before starting ROS. Original map
    and input arrays are never changed. Only this invocation's children stop.

    Opt-in options (defaults reproduce earlier runs): ``amcl_overrides`` sets
    AMCL model parameters, ``initial_std`` the initial-pose uncertainty, and
    ``update_policy='AMCL_MOTION_GATED_UPDATES'`` keeps AMCL's own motion gating
    instead of forcing an update per scan; each scan is then acknowledged by the
    map->odom broadcast stamped scan + transform_tolerance, and every broadcast
    is kept in ``map_odom_trajectory``.
    """
    timeout = float(timeout_s)
    if not math.isfinite(timeout) or not 10 <= timeout <= 900:
        raise ValueError('finite timeout_s in [10,900] required')
    if evaluation_scope not in EVALUATION_SCOPES:
        raise ValueError('explicit supported evaluation scope required')
    if laser_model not in ('likelihood_field', 'likelihood_field_prob'):
        raise ValueError('unsupported AMCL laser model')
    if update_policy not in UPDATE_POLICIES:
        raise ValueError('unsupported AMCL update policy')
    overrides = amcl_parameter_overrides(amcl_overrides)
    std = initial_standard_deviations(initial_std)
    gated = update_policy == 'AMCL_MOTION_GATED_UPDATES'
    prepared, initial = prepare_frames(frames, initial_pose, scan_height_m)
    fixed_base_lidar = None if base_from_lidar is None else planar_base_from_lidar(base_from_lidar)
    lidar_from_base = np.eye(4) if fixed_base_lidar is None else np.linalg.inv(fixed_base_lidar)
    initial_base = initial @ lidar_from_base
    for frame in prepared:
        frame['base_odometry_T'] = frame['odometry_T'] @ lidar_from_base
    validate_isolation()
    import yaml
    source = Path(map_yaml).expanduser().absolute()
    if '..' in source.parts or any(p.is_symlink() for p in (source, *source.parents)):
        raise ValueError('unlinked ordinary map paths required')
    meta = yaml.safe_load(source.read_text(encoding='utf-8'))
    if not isinstance(meta, dict) or not isinstance(meta.get('image'), str):
        raise ValueError('map YAML requires an explicit image')
    image_relative = Path(meta['image'])
    if image_relative.is_absolute() or '..' in image_relative.parts:
        raise ValueError('map image must remain below its YAML directory')
    image = source.parent / image_relative
    if not image.is_file() or any(p.is_symlink() for p in (image, *image.parents)):
        raise ValueError('map image must be an ordinary existing file')
    origin = meta.get('origin')
    if (not isinstance(origin, (list, tuple)) or len(origin) != 3
            or not all(type(x) in (int, float) and math.isfinite(x) for x in origin)
            or abs(math.atan2(math.sin(origin[2]), math.cos(origin[2]))) > 1e-9):
        raise ValueError('AMCL replay requires a finite axis-aligned map origin')
    resolution = meta.get('resolution')
    if type(resolution) not in (int, float) or not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('positive finite map resolution required')
    directory = Path(output_dir).expanduser().absolute()
    if '..' in directory.parts or any(p.is_symlink() for p in (directory, *directory.parents)):
        raise ValueError('unlinked ordinary output paths required')
    directory.mkdir(parents=True, exist_ok=True)
    if any((directory / name).exists() for name in ['amcl_params.yaml', 'map_server.log', 'amcl.log', 'result.json']):
        raise FileExistsError('fresh AMCL output files required')
    source_hashes = {str(p): _sha(p) for p in [source, image]}
    namespace = '/wc_amcl_replay_' + uuid.uuid4().hex[:12]
    map_frame = namespace[1:] + '/map'
    odom_frame = namespace[1:] + '/frozen_prior_odom'
    laser_frame = namespace[1:] + '/recorded_lidar_reference'
    base_frame = (laser_frame if fixed_base_lidar is None else
                  namespace[1:] + '/configured_planar_axle_reference')
    map_topic, scan_topic, clock_topic = (namespace + suffix for suffix in ['/map', '/scan', '/clock'])
    params_file = directory / 'amcl_params.yaml'
    parameters = {
        namespace + '/map_server': {'ros__parameters': {
            'use_sim_time': True, 'yaml_filename': str(source), 'topic_name': map_topic,
            'frame_id': map_frame}},
        namespace + '/amcl': {'ros__parameters': {
            'use_sim_time': True, 'global_frame_id': map_frame,
            'odom_frame_id': odom_frame, 'base_frame_id': base_frame,
            'scan_topic': scan_topic, 'map_topic': map_topic, 'first_map_only': True,
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'laser_model_type': laser_model, 'laser_min_range': .30,
            'laser_max_range': 8.0, 'max_beams': 120,
            'min_particles': 500, 'max_particles': 2000,
            'do_beamskip': False, 'pf_err': .05, 'pf_z': .99,
            'resample_interval': 1, 'sigma_hit': .10, 'z_hit': .95, 'z_rand': .05,
            'laser_likelihood_max_dist': 2.0,
            'recovery_alpha_fast': 0.0, 'recovery_alpha_slow': 0.0,
            'alpha1': .20, 'alpha2': .20, 'alpha3': .20, 'alpha4': .20, 'alpha5': .20,
            'update_min_d': 0.0, 'update_min_a': 0.0, 'transform_tolerance': .20,
            'tf_broadcast': True, 'set_initial_pose': False,
            'always_reset_initial_pose': True, 'save_pose_rate': -1.0}},
    }
    parameters[namespace + '/amcl']['ros__parameters'].update(overrides)
    tolerance_ns = int(round(parameters[namespace + '/amcl']['ros__parameters']['transform_tolerance'] * 1e9))
    params_file.write_text(yaml.safe_dump(parameters, sort_keys=False), encoding='utf-8')
    result = {
        'schema_version': 1, 'status': 'STARTING', 'map_yaml': str(source), 'source_sha256': source_hashes,
        'mode': 'SINGLE_LIDAR_AMCL_REPLAY_FUNCTIONALITY',
        'evaluation_scope': evaluation_scope,
        'global_relocalization_performed': False, 'navigation_validated': False,
        'hardware_started': False, 'control_topics_published': False,
        'base_frame_semantics': ('RECORDED_LIDAR_REFERENCE_NOT_REAL_WHEELCHAIR_BASE_LINK'
                                 if fixed_base_lidar is None else 'CONFIGURED_PLANAR_AXLE_UNVALIDATED'),
        'input_odometry_frame_semantics': 'FROZEN_ODOM_FROM_PREPARED_LIDAR_REFERENCE',
        'returned_pose_frame_semantics': ('RECORDED_LIDAR_REFERENCE' if fixed_base_lidar is None
                                          else 'CONFIGURED_PLANAR_AXLE_UNVALIDATED'),
        'base_from_lidar': None if fixed_base_lidar is None else fixed_base_lidar.tolist(),
        'namespace': namespace, 'map_frame': map_frame, 'odom_frame': odom_frame,
        'base_frame': base_frame, 'laser_frame': laser_frame,
        'ros_domain_id': 95, 'localhost_only': True, 'params_file': str(params_file),
        'scan_settings': dict(prepared[0]['scan_stats']['scan_settings']),
        'model_weights_validated': False,
        'initial_pose': initial_base.tolist(), 'initial_lidar_pose': initial.tolist(),
        'initial_base_pose': initial_base.tolist(), 'initial_pose_publications': 0,
        'initial_standard_deviations': {'x_m': std[0], 'y_m': std[1], 'yaw_deg': math.degrees(std[2])},
        'input_frames': len(prepared), 'published_frames': 0, 'acknowledged_frames': 0,
        'trajectory': [], 'scan_stats': [], 'unacknowledged_stamps_ns': [],
        'output_dir': str(directory), 'source_arrays_modified': False,
        'force_update_policy': update_policy,
        'primary_interface_source': 'https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_amcl/src/amcl_node.cpp',
    }
    if overrides:
        result['amcl_parameter_overrides'] = dict(overrides)
    if gated:
        result['map_odom_trajectory'] = []
        result['acknowledgement'] = 'MAP_TO_ODOM_BROADCAST_STAMPED_SCAN_PLUS_TRANSFORM_TOLERANCE'
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import parameter_value_to_python
    from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from std_srvs.srv import Empty
    from rosgraph_msgs.msg import Clock
    from builtin_interfaces.msg import Time
    from tf2_msgs.msg import TFMessage
    from tf2_ros import StaticTransformBroadcaster
    from scipy.spatial.transform import Rotation
    from .nav2_planner import _stop_owned

    binaries = {'map_server': Path('/opt/ros/humble/lib/nav2_map_server/map_server'),
                'amcl': Path('/opt/ros/humble/lib/nav2_amcl/amcl')}
    for binary in binaries.values():
        if not binary.is_file():
            raise AMCLReplayError('installed Nav2 executable missing: ' + str(binary))
    versions = subprocess.run(['dpkg-query', '-W', '-f=${Package} ${Version}\n',
                               'ros-humble-nav2-map-server', 'ros-humble-nav2-amcl'],
                              capture_output=True, text=True, timeout=5)
    if versions.returncode:
        raise AMCLReplayError('cannot record installed AMCL versions: ' + versions.stderr[:500])
    result['package_versions'] = versions.stdout.strip().splitlines()
    remaps = ['--ros-args', '-r', '__ns:=' + namespace,
              '-r', '/tf:=' + namespace + '/tf', '-r', '/tf_static:=' + namespace + '/tf_static',
              '-r', '/clock:=' + clock_topic]
    context = Context()
    executor = node = clock_pub = None
    children, handles, poses = [], [], {}
    result['logs'], result['process_args'] = {}, {}
    deadline = time.monotonic() + timeout
    current_stamp = prepared[0]['stamp_ns']
    last_clock_wall = 0.0

    def msg_time(stamp):
        sec, nanosec = divmod(int(stamp), 10**9)
        return Time(sec=sec, nanosec=nanosec)

    def healthy():
        nonlocal last_clock_wall
        now = time.monotonic()
        if now >= deadline:
            raise AMCLReplayError('AMCL replay wall deadline exceeded')
        for name, child in children:
            if child.poll() is not None:
                raise AMCLReplayError(name + ' exited early with ' + str(child.returncode))
        if clock_pub is not None and now - last_clock_wall >= .05:
            clock_pub.publish(Clock(clock=msg_time(current_stamp)))
            last_clock_wall = now

    def spin_until(predicate, local_timeout=None):
        local_end = min(deadline, time.monotonic() + local_timeout) if local_timeout is not None else deadline
        while not predicate():
            healthy()
            if time.monotonic() >= local_end:
                return False
            executor.spin_once(timeout_sec=min(.05, max(.001, local_end - time.monotonic())))
        return True

    def future_result(future, local_timeout=10):
        if not spin_until(future.done, local_timeout):
            raise AMCLReplayError('AMCL replay service response timed out')
        if future.exception() is not None:
            raise AMCLReplayError(str(future.exception()))
        return future.result()

    def transition(name, code):
        client = node.create_client(ChangeState, namespace + '/' + name + '/change_state')
        try:
            while not client.wait_for_service(timeout_sec=.2):
                healthy()
            request = ChangeState.Request()
            request.transition.id = code
            if not future_result(client.call_async(request)).success:
                raise AMCLReplayError('lifecycle transition failed for ' + name)
        finally:
            node.destroy_client(client)

    def fill_pose(message, matrix):
        message.position.x, message.position.y, message.position.z = map(float, matrix[:3, 3])
        q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
        message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = map(float, q)

    def pose_callback(message):
        stamp = int(message.header.stamp.sec) * 10**9 + int(message.header.stamp.nanosec)
        if message.header.frame_id != map_frame:
            raise AMCLReplayError('AMCL pose used an unexpected map frame')
        p, q = message.pose.pose.position, message.pose.pose.orientation
        covariance = [float(v) for v in message.pose.covariance]
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, *covariance]
        if not all(math.isfinite(v) for v in values) or abs(sum(v * v for v in [q.x, q.y, q.z, q.w]) - 1) > 1e-3:
            raise AMCLReplayError('AMCL returned a nonfinite pose or invalid orientation')
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        poses[stamp] = {'stamp_ns': stamp, 'x_m': float(p.x), 'y_m': float(p.y), 'yaw_rad': yaw,
                        'z_m': float(p.z), 'quaternion_xyzw': [q.x, q.y, q.z, q.w],
                        'covariance': covariance}

    started = time.monotonic()
    try:
        rclpy.init(args=[], context=context)
        node = rclpy.create_node('amcl_replay_client', namespace=namespace, context=context,
                                enable_rosout=False, start_parameter_services=False,
                                cli_args=remaps + ['-p', 'use_sim_time:=true'], use_global_arguments=False)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        discover_end = min(deadline, time.monotonic() + 2)
        while time.monotonic() < discover_end:
            executor.spin_once(timeout_sec=.1)
            foreign = [(name, ns) for name, ns in node.get_node_names_and_namespaces()
                       if (name, ns) != (node.get_name(), node.get_namespace())]
            if foreign:
                raise AMCLReplayError('isolated domain 95 is occupied: ' + repr(foreign))
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        clock_pub = node.create_publisher(Clock, clock_topic, reliable)
        tf_pub = node.create_publisher(TFMessage, namespace + '/tf', reliable)
        scan_pub = node.create_publisher(LaserScan, scan_topic, reliable)
        initial_pub = node.create_publisher(PoseWithCovarianceStamped, namespace + '/initialpose', reliable)
        received = {}
        map_sub = node.create_subscription(OccupancyGrid, map_topic,
                                           lambda msg: received.__setitem__('map', msg), transient)
        pose_sub = node.create_subscription(PoseWithCovarianceStamped, namespace + '/amcl_pose', pose_callback, transient)
        map_odom = {}

        def tf_callback(message):
            for transform in message.transforms:
                if transform.header.frame_id != map_frame or transform.child_frame_id != odom_frame:
                    continue
                expires = int(transform.header.stamp.sec) * 10**9 + int(transform.header.stamp.nanosec)
                t, q = transform.transform.translation, transform.transform.rotation
                values = [t.x, t.y, q.x, q.y, q.z, q.w]
                if not all(math.isfinite(v) for v in values):
                    raise AMCLReplayError('AMCL broadcast a nonfinite map->odom transform')
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                map_odom[expires - tolerance_ns] = {'x_m': float(t.x), 'y_m': float(t.y), 'yaw_rad': yaw}

        def acknowledged(stamp):
            return any(abs(key - stamp) <= 1000 for key in map_odom) if gated else stamp in poses

        tf_sub = (node.create_subscription(TFMessage, namespace + '/tf', tf_callback,
                                           QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
                  if gated else None)
        static_broadcaster = None
        if fixed_base_lidar is not None:
            static_broadcaster = StaticTransformBroadcaster(node)
            fixed_tf = TransformStamped()
            fixed_tf.header.stamp = msg_time(current_stamp)
            fixed_tf.header.frame_id = base_frame
            fixed_tf.child_frame_id = laser_frame
            fixed_tf.transform.translation.x, fixed_tf.transform.translation.y, fixed_tf.transform.translation.z = map(float, fixed_base_lidar[:3, 3])
            fixed_q = Rotation.from_matrix(fixed_base_lidar[:3, :3]).as_quat()
            fixed_tf.transform.rotation.x, fixed_tf.transform.rotation.y, fixed_tf.transform.rotation.z, fixed_tf.transform.rotation.w = map(float, fixed_q)
            static_broadcaster.sendTransform(fixed_tf)
        env = os.environ.copy()
        env.update(ROS_DOMAIN_ID='95', ROS_LOCALHOST_ONLY='1', PYTHONNOUSERSITE='1',
                   PYTHONDONTWRITEBYTECODE='1', ROS_LOG_DIR=str(directory / 'ros_logs'))
        for name in ['map_server', 'amcl']:
            handle = (directory / (name + '.log')).open('x', encoding='utf-8')
            handles.append(handle)
            command = [str(binaries[name]), *remaps, '--params-file', str(params_file)]
            result['process_args'][name] = command
            result['logs'][name] = str(directory / (name + '.log'))
            child = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                     env=env, start_new_session=True)
            children.append((name, child))
            transition(name, Transition.TRANSITION_CONFIGURE)
            transition(name, Transition.TRANSITION_ACTIVATE)
            if name == 'map_server' and not spin_until(lambda: 'map' in received, 10):
                raise AMCLReplayError('map server did not publish the saved map')
        result['parameter_readback'] = {}
        for name in ['map_server', 'amcl']:
            expected = parameters[namespace + '/' + name]['ros__parameters']
            client = node.create_client(GetParameters, namespace + '/' + name + '/get_parameters')
            try:
                while not client.wait_for_service(timeout_sec=.2):
                    healthy()
                request = GetParameters.Request()
                request.names = list(expected)
                response = future_result(client.call_async(request))
                actual = {key: parameter_value_to_python(value) for key, value in zip(expected, response.values)}
                result['parameter_readback'][name] = actual
                result.setdefault('parameter_readback_transport', {})[name] = {'request': repr(request), 'response': repr(response)}
                if actual != expected:
                    raise AMCLReplayError('AMCL replay parameter readback mismatch for ' + name)
            finally:
                node.destroy_client(client)
        if not spin_until(lambda: scan_pub.get_subscription_count() >= 1 and
                          tf_pub.get_subscription_count() >= 1 and initial_pub.get_subscription_count() >= 1, 10):
            raise AMCLReplayError('AMCL scan/TF/initial-pose subscriptions not ready')
        nomotion = node.create_client(Empty, namespace + '/request_nomotion_update')
        while not nomotion.wait_for_service(timeout_sec=.2):
            healthy()

        def publish_tf(frame):
            transform = TransformStamped()
            transform.header.stamp = msg_time(frame['stamp_ns'])
            transform.header.frame_id = odom_frame
            transform.child_frame_id = base_frame
            pose = frame['base_odometry_T']
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, pose[:3, 3])
            q = Rotation.from_matrix(pose[:3, :3]).as_quat()
            transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = map(float, q)
            tf_pub.publish(TFMessage(transforms=[transform]))

        healthy()
        publish_tf(prepared[0])
        settle_end = time.monotonic() + .20
        while time.monotonic() < settle_end:
            healthy()
            executor.spin_once(timeout_sec=.02)
        initial_message = PoseWithCovarianceStamped()
        initial_message.header.stamp = msg_time(current_stamp)
        initial_message.header.frame_id = map_frame
        fill_pose(initial_message.pose.pose, initial_base)
        initial_message.pose.covariance[0] = std[0]**2
        initial_message.pose.covariance[7] = std[1]**2
        initial_message.pose.covariance[35] = std[2]**2
        initial_pub.publish(initial_message)
        result['initial_pose_publications'] += 1
        settle_end = time.monotonic() + .20
        while time.monotonic() < settle_end:
            healthy()
            executor.spin_once(timeout_sec=.02)
        for index, frame in enumerate(prepared):
            current_stamp = frame['stamp_ns']
            healthy()
            publish_tf(frame)
            if not gated:
                future_result(nomotion.call_async(Empty.Request()), 3)
            else:
                # Let the odometry transform arrive before its scan, as in a continuous TF stream.
                spin_until(lambda: False, .01)
            scan = LaserScan()
            scan.header.stamp = msg_time(current_stamp)
            scan.header.frame_id = laser_frame
            scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi / 3, math.pi / 3, math.pi / 240
            scan.range_min, scan.range_max = .30, 8.0
            scan.time_increment = 0.0
            scan.scan_time = ((prepared[index + 1]['stamp_ns'] - current_stamp) / 1e9
                              if index + 1 < len(prepared) else 0.0)
            scan.ranges = frame['ranges'].tolist()
            scan_pub.publish(scan)
            result['published_frames'] += 1
            result['scan_stats'].append({'stamp_ns': current_stamp, **frame['scan_stats']})
            if spin_until(lambda: acknowledged(current_stamp), 5):
                result['acknowledged_frames'] += 1
            else:
                result['unacknowledged_stamps_ns'].append(current_stamp)
            if (index + 1) % 20 == 0:
                print('AMCL_REPLAY', index + 1, '/', len(prepared), 'poses', len(poses), flush=True)
        if gated:
            # A filter update publishes its pose next to the broadcast; let the last one arrive.
            spin_until(lambda: False, .3)
        result['status'] = 'AMCL_REPLAY_COMPLETE' if result['acknowledged_frames'] == len(prepared) else 'PARTIAL_AMCL_POSES'
        result['sources_unchanged'] = all(_sha(path) == value for path, value in source_hashes.items())
        if not result['sources_unchanged']:
            raise AMCLReplayError('source map changed during replay')
    except Exception as exc:
        result.update(status='FAIL', error=type(exc).__name__ + ': ' + str(exc))
        (directory / 'amcl_failure_traceback.log').write_text(traceback.format_exc(), encoding='utf-8')
    finally:
        for name, child in reversed(children):
            _stop_owned(child)
        result['child_exit_codes'] = {name: child.poll() for name, child in children}
        for handle in handles:
            handle.close()
        if executor is not None:
            if node is not None:
                executor.remove_node(node)
            executor.shutdown(timeout_sec=1.0)
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
        result['wall_elapsed_s'] = time.monotonic() - started
        allowed = {frame['stamp_ns'] for frame in prepared}
        result['trajectory'] = [poses[stamp] for stamp in sorted(poses) if stamp in allowed]
        result['pose_frames'] = len(result['trajectory'])
        if gated:
            rows = []
            for frame in prepared:
                key = min(map_odom, key=lambda k: abs(k - frame['stamp_ns']), default=None)
                if key is not None and abs(key - frame['stamp_ns']) <= 1000:
                    rows.append({'stamp_ns': frame['stamp_ns'], **map_odom[key],
                                 'filter_updated': frame['stamp_ns'] in poses})
            result['map_odom_trajectory'] = rows
        (directory / 'result.json').write_text(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2) + '\n', encoding='utf-8')
    return result
