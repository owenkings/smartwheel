"""Explicit RTAB grid settings with heights relative to the configured floor.

This is an experimental flat-floor model, not a measured floor calibration.
Native scan points retain XYZ; a planar motion model does not flatten clouds.
"""
import copy
import math


DEFAULT_PROFILE = {
    'schema_version': 1,
    'status': 'EXPERIMENT',
    'cell_size_m': .05,
    'range_min_m': .3,
    'range_max_m': 8.,
    'min_ground_height_m': -.1,
    'max_ground_height_m': .15,
    'max_obstacle_height_m': 1.8,
    'max_ground_angle_deg': 30.,
    'normal_neighbors': 10,
    'cluster_radius_m': .1,
    'min_cluster_points': 5,
    'noise_radius_m': .1,
    'noise_min_neighbors': 3,
    'ray_tracing': 'single_sensor_only',
    'linear_update_m': .05,
    'angular_update_rad': .05,
    'detection_rate_hz': 1.,
    'keep_unlinked_nodes': False,
}


def validate_profile(value):
    if value is None:
        return None
    if not isinstance(value, dict) or set(value)-set(DEFAULT_PROFILE):
        raise ValueError('map_profile contains unsupported fields')
    profile = {**copy.deepcopy(DEFAULT_PROFILE), **copy.deepcopy(value)}
    if profile['schema_version'] != 1 or profile['status'] != 'EXPERIMENT':
        raise ValueError('map_profile requires schema_version 1 / EXPERIMENT')
    for name in ('cell_size_m', 'range_min_m', 'range_max_m', 'max_ground_angle_deg',
                 'cluster_radius_m', 'noise_radius_m', 'linear_update_m',
                 'angular_update_rad', 'detection_rate_hz'):
        v = profile[name]
        if type(v) not in (int, float) or not math.isfinite(v) or v <= 0:
            raise ValueError('map_profile positive finite number required: '+name)
    for name in ('min_ground_height_m', 'max_ground_height_m', 'max_obstacle_height_m'):
        v = profile[name]
        if type(v) not in (int, float) or not math.isfinite(v):
            raise ValueError('map_profile finite height required: '+name)
    for name in ('normal_neighbors', 'min_cluster_points', 'noise_min_neighbors'):
        if type(profile[name]) is not int or not 1 <= profile[name] <= 100:
            raise ValueError('map_profile integer 1..100 required: '+name)
    if not .01 <= profile['cell_size_m'] <= .25 or not profile['range_min_m'] < profile['range_max_m'] <= 30:
        raise ValueError('map_profile cell size/range outside reviewed bounds')
    if not -2 <= profile['min_ground_height_m'] < profile['max_ground_height_m'] < profile['max_obstacle_height_m'] <= 5:
        raise ValueError('map_profile heights must be ordered relative to ground')
    if not 0 < profile['max_ground_angle_deg'] < 90 or not 0 < profile['detection_rate_hz'] <= 5:
        raise ValueError('map_profile ground angle/rate outside reviewed bounds')
    if profile['ray_tracing'] not in ('off', 'single_sensor_only'):
        raise ValueError('map_profile ray_tracing must be off or single_sensor_only')
    if type(profile['keep_unlinked_nodes']) is not bool:
        raise ValueError('map_profile keep_unlinked_nodes must be boolean')
    return profile


def resolve_native_profile(config):
    profile = validate_profile(config.get('map_profile'))
    model = config.get('motion_model', 'se3_gyro')
    if model not in ('planar_ekf', 'se3_gyro'):
        raise ValueError('motion_model must be planar_ekf or se3_gyro')
    params = {'Reg/Force3DoF': 'true' if model == 'planar_ekf' else 'false'}
    if profile is None:
        return {'motion_model': model, 'parameters': params, 'grid_profile': None}
    if model != 'planar_ekf':
        raise ValueError('ground-relative map_profile requires the explicit planar_ekf floor model')
    mode = config.get('mode')
    if mode not in ('left', 'right', 'all'):
        raise ValueError('native grid profile requires left/right/all mode')
    base = 'left' if mode == 'all' else mode
    height = float(config['mounts'][base]['t_axle_lidar_m'][2])+float(config['wheel_candidate']['wheel_radius_m'])
    if not math.isfinite(height) or not 0 < height <= 3:
        raise ValueError('configured sensor height above candidate ground is invalid')
    ground_z = -height
    heights = {name: ground_z+profile[key] for name, key in (
        ('Grid/MinGroundHeight', 'min_ground_height_m'),
        ('Grid/MaxGroundHeight', 'max_ground_height_m'),
        ('Grid/MaxObstacleHeight', 'max_obstacle_height_m'))}
    # In native RTAB, exactly zero disables a height bound. Do not silently
    # turn a requested physical bound off for a different installation height.
    if any(abs(v) < 1e-9 for v in heights.values()):
        raise ValueError('native grid height resolves to zero (disabled sentinel); choose explicit nonzero bounds')
    params.update({
        'Grid/3D': 'true',
        'Grid/CellSize': str(profile['cell_size_m']),
        'Grid/RangeMin': str(profile['range_min_m']),
        'Grid/RangeMax': str(profile['range_max_m']),
        'Grid/NormalsSegmentation': 'true',
        'Grid/MaxGroundAngle': str(profile['max_ground_angle_deg']),
        'Grid/NormalK': str(profile['normal_neighbors']),
        'Grid/ClusterRadius': str(profile['cluster_radius_m']),
        'Grid/MinClusterSize': str(profile['min_cluster_points']),
        'Grid/NoiseFilteringRadius': str(profile['noise_radius_m']),
        'Grid/NoiseFilteringMinNeighbors': str(profile['noise_min_neighbors']),
        'Grid/MapFrameProjection': 'false',
        'Grid/RayTracing': 'true' if mode != 'all' and profile['ray_tracing'] == 'single_sensor_only' else 'false',
        'RGBD/LinearUpdate': str(profile['linear_update_m']),
        'RGBD/AngularUpdate': str(profile['angular_update_rad']),
        'Rtabmap/DetectionRate': str(profile['detection_rate_hz']),
        'Mem/NotLinkedNodesKept': str(profile['keep_unlinked_nodes']).lower(),
        **{k: str(v) for k, v in heights.items()},
    })
    return {'motion_model': model, 'parameters': params, 'grid_profile': profile,
            'reference_sensor': base, 'candidate_ground_z_in_reference_m': ground_z,
            'ground_basis': 'configured_lidar_z_above_axle_plus_unvalidated_wheel_radius',
            'ground_height_validated': False,
            'ray_origin': 'selected_single_sensor' if params['Grid/RayTracing'] == 'true' else 'disabled',
            'dual_ray_tracing_implemented': False, 'cloud_xyz_retained': True}
