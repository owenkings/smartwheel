"""Experimental single-lidar 6DoF processing; device ownership stays with the guard.

The initial lidar frame defines this experiment's coordinates, without asserting
that it is level or calibrated to the wheelchair. Stop the launch normally and
wait for RTAB-Map to close its database before offline export. The caller owns
the exclusive session/device locks; this launch never deletes or resumes a DB.

Parameter contracts were checked against upstream rtabmap_ros ros2 sources:
OdometryROS.cpp, CommonDataSubscriber.cpp, CoreWrapper.cpp and MapsManager.cpp,
plus rtabmap/core/Parameters.h. Target installed version is checked by controller.
"""
from pathlib import Path
from wc_runtime.storage_policy import resolve_storage_path


from wc_runtime.project_paths import project_root as resolve_project_root

PROJECT_ROOT = resolve_project_root(start=__file__)


def build_spec(side, session_root, *, project_root=None):
    """Validate output containment before constructing either processing node."""
    if side not in ('left', 'right'):
        raise ValueError('side must be left or right')
    root = Path(project_root) if project_root is not None else PROJECT_ROOT
    session = Path(session_root)
    if not root.is_absolute() or not session.is_absolute() or '..' in session.parts:
        raise ValueError('session_root must be an explicit absolute project path')
    session = resolve_storage_path(root, session)
    allowed = resolve_storage_path(root, 'reports/single_mapping')
    try:
        relative = session.relative_to(allowed)
    except ValueError as exc:
        raise ValueError('session_root must be below reports/single_mapping') from exc
    if not relative.parts:
        raise ValueError('session_root must name a separate session directory')
    # Check both lexical components and resolved containment, including links
    # within the project. The controller creates directories before launching.
    for path in (session, *session.parents):
        if path.is_symlink() or getattr(path, 'is_junction', lambda: False)():
            raise ValueError('symlink/junction session paths are not allowed')
        if path == root:
            break
    session = session.resolve(strict=True)
    allowed = allowed.resolve(strict=True)
    if not session.is_dir() or not session.is_relative_to(allowed):
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

    namespace = '/wc_mapping/single_' + side
    lidar_frame = 'lidar_' + side
    odom_frame = 'single_' + side + '_odom'
    map_frame = 'single_' + side + '_map'
    remappings = [
        ('scan_cloud', namespace + '/scan_cloud'),
        ('scan', namespace + '/unused_scan'),
        ('odom', namespace + '/odom'),
        ('odom_info', namespace + '/odom_info'),
        ('imu', namespace + '/unused_imu'),
        ('/tf', namespace + '/tf'),
        ('/tf_static', namespace + '/tf_static'),
        ('/diagnostics', namespace + '/diagnostics'),
    ]
    # These are experimental matching thresholds, not accuracy claims.
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
        'frame_id': lidar_frame,
        'odom_frame_id': odom_frame,
        'publish_tf': True,
        'use_sim_time': False,
        'wait_for_transform': 0.2,
        'qos': 1,  # rmw RELIABLE; independent of the guard's queue depth 2.
        'topic_queue_size': 2,
        'scan_cloud_is_2d': False,
        'scan_voxel_size': 0.03,
        'scan_normal_k': 20,
        'deskewing': False,  # No measured per-point timestamps or motion prior.
        'wait_imu_to_init': False,
        'guess_frame_id': '',
        'ground_truth_frame_id': '',
        'publish_null_when_lost': True,
        'Odom/Strategy': '0',
        'Odom/ResetCountdown': '0',
        'OdomF2M/ScanMaxSize': '20000',
    }
    slam = {
        **registration,
        'frame_id': lidar_frame,
        # CommonDataSubscriber disables odom subscription when this is nonempty.
        'odom_frame_id': '',
        'odom_frame_id_init': odom_frame,
        'map_frame_id': map_frame,
        'publish_tf': True,
        'use_sim_time': False,
        'wait_for_transform': 0.2,
        'database_path': str(database),
        'delete_db_on_start': False,
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
        'subscribe_odom_info': True,
        'scan_cloud_is_2d': False,
        # Native odometry copies the input stamp to both odom and odom_info.
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
        'Optimizer/GravitySigma': '0',
        'RGBD/CreateOccupancyGrid': 'true',
        'Grid/Sensor': '0',
        'Grid/3D': 'true',
        # Preserve geometry without assuming that sensor Z identifies ground.
        # Native cloud_map joins these cells; they are not navigation labels.
        'Grid/GroundIsObstacle': 'true',
        'Grid/CellSize': '0.03',
        'Grid/RayTracing': 'false',
        'Grid/RangeMax': '0',
        'Rtabmap/WorkingDirectory': str(slam_dir),
    }
    return {
        'namespace': namespace,
        'database_path': str(database),
        'frames': {'lidar': lidar_frame, 'odom': odom_frame, 'map': map_frame},
        'nodes': [
            {'package': 'rtabmap_odom', 'executable': 'icp_odometry',
             'name': 'icp_odometry', 'namespace': namespace,
             'parameters': [odometry], 'remappings': list(remappings)},
            {'package': 'rtabmap_slam', 'executable': 'rtabmap',
             'name': 'rtabmap', 'namespace': namespace,
             'parameters': [slam], 'remappings': remappings + [
                 ('global_pose', namespace + '/unused_global_pose'),
                 ('gps/fix', namespace + '/unused_gps'),
             ]},
        ],
    }


def configure(context):
    # Delayed imports allow filesystem/pipeline contracts to be tested without ROS.
    from launch.actions import EmitEvent, RegisterEventHandler
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node

    spec = build_spec(LaunchConfiguration('side').perform(context),
                      LaunchConfiguration('session_root').perform(context),
                      project_root=LaunchConfiguration('project_root').perform(context))
    processes = [Node(**node, output='screen', sigterm_timeout='20',
                      sigkill_timeout='5') for node in spec['nodes']]
    # Register handlers before starting either process, including startup failure.
    handlers = [RegisterEventHandler(OnProcessExit(target_action=process,
                on_exit=[EmitEvent(event=Shutdown(
                    reason='A required single-lidar processing node exited'))]))
                for process in processes]
    return handlers + processes


def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction

    return LaunchDescription([
        DeclareLaunchArgument('side', choices=['left', 'right']),
        DeclareLaunchArgument('session_root'),
        DeclareLaunchArgument('project_root', default_value=str(PROJECT_ROOT)),
        OpaqueFunction(function=configure),
    ])
