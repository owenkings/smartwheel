"""Native scan mapping with explicit wheel/IMU or ICP odometry authority.

The controller owns devices, the exclusive session, calibration and the prior.
Its cloud is already expressed in mapping_reference using initial IMU gravity.
That frame's origin is the reference sensor, not an assumed floor height.

By default, upstream wheel/IMU odometry supplies the cloud's exact timestamp;
this launch starts SLAM only and disables scan-based pose correction. With the
explicit icp option, native ICP publishes mapping_odom -> prior_odom and the
prior owns prior_odom -> mapping_reference. Its Odometry message still names
mapping_odom -> mapping_reference. All TF transport stays in this app's scope.

Core parameters follow upstream Parameters.h and LocalMapMaker.hpp. Normal
clustering is an experimental ground classifier: a dominant tabletop can be
mistaken for ground. The projected grid is not a validated navigation map.
"""
from pathlib import Path
import json
from wc_runtime.storage_policy import StoragePolicy


from wc_runtime.project_paths import project_root as resolve_project_root

PROJECT_ROOT = resolve_project_root(start=__file__)
NAMESPACE = '/wc_mapping/app'


def build_spec(session_root, *, project_root=None, odometry_source='wheel_imu', runtime_config=None):
    """Validate an explicit, new database destination without creating files."""
    if odometry_source not in ('wheel_imu', 'icp'):
        raise ValueError('odometry_source must be wheel_imu or icp')
    root = Path(project_root) if project_root is not None else PROJECT_ROOT
    session = Path(session_root)
    if (not root.is_absolute() or not session.is_absolute()
            or '..' in root.parts or '..' in session.parts):
        raise ValueError('session_root must be an explicit absolute project path')
    storage = StoragePolicy(root)
    try:
        session = storage.resolve(session)
    except ValueError as error:
        raise ValueError('session_root must be inside the project or configured USB archive: '+str(error)) from error
    # Native offline regression workspaces may be ordinary project subfolders;
    # only data/reports/maps are relocated by policy, without local fallback.
    location_root = (storage.archive_root if storage.enabled and session.is_relative_to(storage.archive_root)
                     else root)
    try:
        relative = session.relative_to(location_root)
    except ValueError as exc:
        raise ValueError('session_root must be inside the project') from exc
    if not relative.parts or relative.parts[0] in (
            '.git', '.codex', '.agents', '.phase1_runtime'):
        raise ValueError('session_root must name a separate project output directory')
    for path in (session, *session.parents):
        if path.is_symlink() or getattr(path, 'is_junction', lambda: False)():
            raise ValueError('symlink/junction session paths are not allowed')
    root = location_root.resolve(strict=True)
    session = session.resolve(strict=True)
    if not session.is_dir() or not session.is_relative_to(root) or session == root:
        raise ValueError('session_root must be an existing contained directory')
    slam_dir = session / 'slam'
    if (not slam_dir.is_dir() or slam_dir.is_symlink()
            or getattr(slam_dir, 'is_junction', lambda: False)()):
        raise ValueError('controller must create the real session_root/slam directory')
    database = slam_dir / 'rtabmap.db'
    for suffix in ('', '-wal', '-shm', '-journal'):
        candidate = Path(str(database) + suffix)
        if candidate.exists() or candidate.is_symlink():
            raise ValueError('refusing an existing database or SQLite sidecar: ' + str(candidate))

    remappings = [
        ('scan_cloud', NAMESPACE + '/scan_cloud'),
        ('scan', NAMESPACE + '/unused_scan'),
        ('odom', NAMESPACE + '/odom'),
        ('odom_info', NAMESPACE + '/odom_info'),
        ('imu', NAMESPACE + '/unused_imu'),
        ('/tf', NAMESPACE + '/tf'),
        ('/tf_static', NAMESPACE + '/tf_static'),
        ('/diagnostics', NAMESPACE + '/diagnostics'),
    ]
    # Experimental matching thresholds; these numbers do not specify accuracy.
    registration = {
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'false',
        'Icp/PointToPlane': 'true',
        'Icp/PointToPlaneK': '20',
        'Icp/CorrespondenceRatio': '0.15',
        'Icp/MaxCorrespondenceDistance': '0.25',
        'Icp/Iterations': '30',
        'Icp/VoxelSize': '0.03',
    }
    odometry = {
        **registration,
        'frame_id': 'mapping_reference',
        'odom_frame_id': 'mapping_odom',
        'guess_frame_id': 'prior_odom',
        'publish_tf': True,
        'use_sim_time': False,
        'wait_for_transform': 0.2,
        'qos': 1,  # rmw RELIABLE, matching the prior's input cloud publisher.
        'topic_queue_size': 2,
        'scan_cloud_is_2d': False,
        'scan_voxel_size': 0.03,
        'scan_normal_k': 20,
        'deskewing': False,  # The device has no verified per-point timestamps.
        'wait_imu_to_init': False,  # Initial gravity and motion prior are upstream.
        'ground_truth_frame_id': '',
        'publish_null_when_lost': True,
        'Odom/Strategy': '0',
        'Odom/ResetCountdown': '0',
        'OdomF2M/ScanMaxSize': '20000',
    }
    slam = {
        **(registration if odometry_source == 'icp' else {
            'Reg/Strategy': '1', 'Reg/Force3DoF': 'false'}),
        'frame_id': 'mapping_reference',
        # Nonempty odom_frame_id disables native odometry-message subscription.
        'odom_frame_id': '',
        'odom_frame_id_init': 'mapping_odom',
        'map_frame_id': 'mapping_map',
        'publish_tf': True,
        'use_sim_time': False,
        'wait_for_transform': 0.2,
        'database_path': str(database),
        'delete_db_on_start': False,
        # Native 0.23.7 keeps SQLite in RAM and writes database_path on normal
        # close. Avoid random SQLite page writes competing with raw archives
        # on this Orin's slow eMMC. Raw sensor recording remains on disk;
        # a crash before close can lose this in-memory mapping database.
        'DbSqlite3/InMemory': 'true',
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_stereo': False,
        'subscribe_rgbd': False,
        'subscribe_scan': False,
        'subscribe_scan_cloud': True,
        'subscribe_scan_descriptor': False,
        'subscribe_sensor_data': False,
        'subscribe_user_data': False,
        'subscribe_odom': True,
        'subscribe_odom_info': odometry_source == 'icp',
        'scan_cloud_is_2d': False,
        # Both authorities publish odom with the exact cloud stamp. Only the
        # ICP branch additionally synchronizes native OdomInfo.
        'approx_sync': False,
        'topic_queue_size': 10,
        'sync_queue_size': 10,
        'qos': 1,
        'qos_scan': 1,
        'qos_odom': 1,
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityOdomGuess': 'true',
        'Optimizer/GravitySigma': '0',  # No second, uncalibrated raw IMU path.
        'RGBD/CreateOccupancyGrid': 'true',
        'Grid/Sensor': '0',
        'Grid/3D': 'true',  # Native cloud_map retains Z; grid_map is its 2D product.
        'Grid/CellSize': '0.03',
        'Grid/RangeMax': '0',
        'Grid/NormalsSegmentation': 'true',
        'Grid/GroundIsObstacle': 'false',
        'Grid/FlatObstacleDetected': 'true',
        'Grid/MaxGroundAngle': '20',  # Degrees from +Z, an initial classifier setting.
        'Grid/NormalK': '20',
        'Grid/ClusterRadius': '0.1',
        'Grid/MinClusterSize': '10',
        # Zero disables these height filters. Ground can be below sensor Z=0.
        'Grid/MinGroundHeight': '0',
        'Grid/MaxGroundHeight': '0',
        'Grid/MaxObstacleHeight': '0',
        # Native segmentation already applies pose roll/pitch; this adds pose Z.
        'Grid/MapFrameProjection': 'true',
        # Dual observations have different origins: no fictitious common rays.
        'Grid/RayTracing': 'false',
        'Rtabmap/WorkingDirectory': str(slam_dir),
    }
    if odometry_source == 'wheel_imu':
        # Upstream wheel/IMU poses remain the odometry authority. Keep RGB-D
        # graph mapping enabled for native scan cloud / occupancy products,
        # but disable 0.23.7 scan refinement, proximity and loop-closure paths.
        # No ICP correspondence/quality threshold is applied to this branch.
        slam.update({
            'RGBD/Enabled': 'true',
            'RGBD/NeighborLinkRefining': 'false',
            'RGBD/ProximityByTime': 'false',
            'RGBD/ProximityBySpace': 'false',
            'RGBD/ProximityPathMaxNeighbors': '0',
            'Rtabmap/LoopThr': '1',
            'RGBD/AggressiveLoopThr': '1',
        })
    # The controller writes this immutable snapshot before any process starts.
    # Absence preserves historical diagnostic launch behavior; production maps
    # always carry the resolved model and explicit experimental grid profile.
    config_file = session/'runtime_config.json'
    if runtime_config is None and config_file.exists():
        if config_file.is_symlink() or config_file.stat().st_size > 262144:
            raise ValueError('invalid runtime configuration snapshot')
        runtime_config = json.loads(config_file.read_text(encoding='utf-8'))
    profile = None
    if runtime_config is not None:
        if runtime_config.get('odometry_source', odometry_source) != odometry_source:
            raise ValueError('runtime odometry authority differs from launch argument')
        from wc_runtime.mapping_profile import resolve_native_profile
        profile = resolve_native_profile(runtime_config)
        slam.update(profile['parameters'])
        if odometry_source == 'icp':
            odometry['Reg/Force3DoF'] = profile['parameters']['Reg/Force3DoF']
    # Humble Node prepends --ros-args to ros_arguments. Reduce per-frame INFO
    # log I/O while retaining WARN/ERROR; odom_info and recorded topics remain.
    # This is an I/O reduction experiment, not a confirmed cause of output gaps.
    native_ros_arguments = ['--log-level', 'warn']
    # Native scan SLAM consumes OdomInfo with ignoreData=true. Keep its scalar
    # quality, covariance, transforms and guess, without repeatedly transporting
    # and recording the diagnostic local scan map. Only ICP's publishers move;
    # the SLAM subscriber and app health/bag topic remain unchanged.
    odometry_remappings = [(source, NAMESPACE+'/unused_odom_info_full'
                           if source == 'odom_info' else destination)
                          for source, destination in remappings]
    odometry_remappings.append(('odom_info_lite', NAMESPACE+'/odom_info'))
    nodes = [
        {'package': 'rtabmap_slam', 'executable': 'rtabmap',
         'name': 'rtabmap', 'namespace': NAMESPACE,
         'ros_arguments': list(native_ros_arguments),
         'parameters': [slam], 'remappings': remappings + [
             ('map', NAMESPACE + '/grid_map'),
             ('global_pose', NAMESPACE + '/unused_global_pose'),
             ('gps/fix', NAMESPACE + '/unused_gps'),
         ]},
    ]
    if odometry_source == 'icp':
        nodes.insert(0, {
            'package': 'rtabmap_odom', 'executable': 'icp_odometry',
            'name': 'icp_odometry', 'namespace': NAMESPACE,
            'ros_arguments': list(native_ros_arguments),
            'parameters': [odometry], 'remappings': odometry_remappings,
        })
    return {
        'namespace': NAMESPACE,
        'odometry_source': odometry_source,
        'native_mapping_profile': profile,
        'database_path': str(database),
        'frames': {'reference': 'mapping_reference', 'prior': 'prior_odom',
                   'odom': 'mapping_odom', 'map': 'mapping_map'},
        'nodes': nodes,
    }


def configure(context):
    # Pure validation above remains importable in tests without ROS installed.
    from launch.actions import EmitEvent, RegisterEventHandler
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node
    from wc_runtime.mapping_shutdown import NATIVE_SIGTERM_S

    spec = build_spec(LaunchConfiguration('session_root').perform(context),
                      project_root=LaunchConfiguration('project_root').perform(context),
                      odometry_source=LaunchConfiguration('odometry_source').perform(context))
    processes = [Node(**node, output='screen', sigterm_timeout=str(int(NATIVE_SIGTERM_S)),
                      sigkill_timeout='5') for node in spec['nodes']]
    handlers = [RegisterEventHandler(OnProcessExit(target_action=process,
                on_exit=[EmitEvent(event=Shutdown(
                    reason='A required mapping application node exited'))]))
                for process in processes]
    return handlers + processes


def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction

    return LaunchDescription([
        DeclareLaunchArgument('session_root'),
        DeclareLaunchArgument('project_root', default_value=str(PROJECT_ROOT)),
        DeclareLaunchArgument('odometry_source', default_value='wheel_imu',
                              choices=['wheel_imu', 'icp']),
        OpaqueFunction(function=configure),
    ])
