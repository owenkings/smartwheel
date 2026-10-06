"""One copyable state contract; MotionPrior owns the shared causal pose_at API.

Neither estimator owns a clock, subscriptions, TF authority or sensor decoding.
Native queries clone the actual library state, including its full covariance.
"""
def create_estimator(name, config, axle_xy=(0., 0.)):
    if name == 'five_state':
        from .mapping_planar import PlanarEKF
        return PlanarEKF(config, axle_xy)
    if name == 'robot_localization':
        from .robot_localization_provider import RobotLocalizationEKF
        return RobotLocalizationEKF(config, axle_xy)
    raise ValueError('unknown estimator: '+str(name))
