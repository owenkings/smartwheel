"""Explicitly UNVALIDATED wheel-speed/distance preview from historical settings.

Arrival intervals and old empirical units are useful for parameter comparison,
but this module cannot produce formal odometry or a common-time motion prior.
"""

import math

from .feedback_transport import QUERY, bounded
from .protocol import FeedbackError, integer, parse_exchange


class HistoryPreview:
    def __init__(self, config, *, continuous=False, expected_device_id=None):
        if type(continuous) is not bool:
            raise FeedbackError('continuous preview must be an explicit boolean')
        if config.get('schema') != 'wc_wheel_history_preview_v1' or config.get('state') != 'UNVALIDATED' or config.get('formal_odometry_eligible') is not False:
            raise FeedbackError('historical preview must explicitly remain UNVALIDATED')
        if not config.get('provenance'):
            raise FeedbackError('historical parameter provenance required')
        self.config = config
        from wc_runtime.device_bindings import identity_token
        configured = config.get('device_id')
        try:
            for identity in (configured, expected_device_id):
                if identity is not None:
                    identity_token(identity, prefix='ZLAC8030D-')
        except ValueError as error:
            raise FeedbackError(str(error)) from error
        if configured is not None and expected_device_id is not None and configured != expected_device_id:
            raise FeedbackError('wheel conversion identity differs from expected input identity')
        self.device_id = expected_device_id or configured
        self.continuous = continuous
        for name, low, high in (('wheel_radius_m', .01, 1), ('track_width_m', .1, 2),
                               ('register_to_wheel_rpm', .0001, 100), ('max_gap_s', .01, 1), ('max_wheel_speed_m_s', .01, 3)):
            bounded(config.get(name), name, low, high)
        for key in ('left_sign', 'right_sign'):
            if type(config.get(key)) is not int or config[key] not in (-1, 1):
                raise FeedbackError('explicit candidate wheel direction signs required')
        self.previous = None
        self.x = self.y = self.yaw = self.distance = self.travel = 0.0
        self.left_travel = self.right_travel = 0.0
        self.blocked = None
        self.gap_count = self.missing_sequences = 0
        self.uncovered_duration_s = 0.0

    def block(self, reason):
        self.blocked = str(reason)

    def update(self, record):
        if self.blocked:
            raise FeedbackError('preview integration BLOCKED: '+self.blocked)
        try:
            return self._update(record)
        except (FeedbackError, ValueError, TypeError, KeyError) as error:
            self.block(str(error))
            raise

    def _update(self, record):
        if record.get('schema') != 'wc_wheel_feedback_v1' or record.get('status') != 'RESPONSE_VALID':
            raise FeedbackError('preview requires valid identified feedback')
        from wc_runtime.device_bindings import identity_token
        try:
            identity = identity_token(record.get('device_id'), prefix='ZLAC8030D-')
        except ValueError as error:
            raise FeedbackError(str(error)) from error
        # Old archives did not declare an expected controller; bind their first
        # input identity and reject mixed devices, without consulting live config.
        if self.device_id is None:
            self.device_id = identity
        if identity != self.device_id:
            raise FeedbackError('preview feedback device identity changed')
        request, response = bytes.fromhex(record['request_hex']), bytes.fromhex(record['response_hex'])
        if request != QUERY:
            raise FeedbackError('preview request is not the reviewed feedback query')
        _, _, words = parse_exchange(request, response)
        stamp = integer(record['stamp_ns'], 'arrival timestamp', 1)
        receive = integer(record['receive_monotonic_ns'], 'arrival monotonic time', 1)
        sequence = integer(record['sequence'], 'sequence', 0)
        epoch = record['stream_epoch']
        if not isinstance(epoch, str) or not epoch:
            raise FeedbackError('feedback stream epoch required')
        if record.get('time_valid') is not False or record.get('time_source') != 'arrival_only':
            raise FeedbackError('preview must retain arrival-only timing classification')
        signed = [word-65536 if word >= 32768 else word for word in words]
        scale = self.config['register_to_wheel_rpm']
        wheel_rpm = [signed[0]*scale*self.config['left_sign'], signed[1]*scale*self.config['right_sign']]
        speeds = [rpm*(2*math.pi/60)*self.config['wheel_radius_m'] for rpm in wheel_rpm]
        if max(abs(v) for v in speeds) > self.config['max_wheel_speed_m_s']:
            raise FeedbackError('candidate wheel speed exceeds preview plausibility bound')
        dt = interval = None
        gap = False
        missing = 0
        if self.previous is not None:
            previous = self.previous
            if epoch != previous['epoch'] or sequence <= previous['sequence'] or stamp <= previous['stamp'] or receive <= previous['receive']:
                raise FeedbackError('preview identity/sequence/time discontinuity')
            missing = sequence-previous['sequence']-1
            if missing and not self.continuous:
                raise FeedbackError('preview identity/sequence/time discontinuity')
            interval = (receive-previous['receive'])*1e-9
            if interval > self.config['max_gap_s'] and not self.continuous:
                raise FeedbackError('preview feedback gap; integration stopped')
            gap = bool(missing or interval > self.config['max_gap_s'])
            if gap:
                # Keep the last measured pose across missing coverage. The next
                # real sample becomes the origin for a new integration segment;
                # neither held velocity nor interpolation fills the blank gap.
                self.gap_count += 1
                self.missing_sequences += missing
                self.uncovered_duration_s += interval
            else:
                dt = interval
        if dt is not None:
            dl, dr = ((speeds[i]+self.previous['speeds'][i])*.5*dt for i in range(2))
            distance, turn = (dl+dr)/2, (dr-dl)/self.config['track_width_m']
            half = turn/2
            sinc = math.sin(half)/half if abs(half) > 1e-10 else 1-half*half/6
            self.x += distance*sinc*math.cos(self.yaw+half)
            self.y += distance*sinc*math.sin(self.yaw+half)
            self.yaw = (self.yaw+turn+math.pi)%(2*math.pi)-math.pi
            self.distance += distance
            self.travel += abs(distance)
            self.left_travel += dl
            self.right_travel += dr
        self.previous = {'stamp': stamp, 'receive': receive, 'sequence': sequence, 'epoch': epoch, 'speeds': speeds}
        return {'schema': 'wc_wheel_history_preview_sample_v1', 'state': 'UNVALIDATED',
            'formal_odometry_eligible': False, 'publishes_tf': False, 'hardware_feedback': True,
            'stamp_ns': stamp, 'receive_monotonic_ns': receive, 'stream_epoch': epoch, 'sequence': sequence,
            'time_valid': False, 'time_source': 'arrival_only', 'uncertainty_ns': None,
            'frame_id': 'wheel_odom_preview', 'child_frame_id': 'axle_preview',
            'origin': 'relative_first_feedback_axle_pose_not_sensor_extrinsics',
            'left_raw_signed': signed[0], 'right_raw_signed': signed[1],
            'left_wheel_rpm_candidate': wheel_rpm[0], 'right_wheel_rpm_candidate': wheel_rpm[1],
            'linear_velocity_m_s': sum(speeds)/2, 'angular_velocity_rad_s': (speeds[1]-speeds[0])/self.config['track_width_m'],
            'x_m': self.x, 'y_m': self.y, 'yaw_rad': self.yaw,
            'signed_distance_m': self.distance, 'travel_distance_m': self.travel,
            'left_signed_distance_m': self.left_travel, 'right_signed_distance_m': self.right_travel,
            'integration_interval_s': dt, 'source_interval_s': interval,
            'continuous_mapping': self.continuous, 'integration_gap': gap,
            'missing_sequences_this_interval': missing, 'integration_gap_count': self.gap_count,
            'missing_sequences': self.missing_sequences, 'uncovered_duration_s': self.uncovered_duration_s,
            'motion_coverage_complete': self.gap_count == 0,
            'gap_handling': 'POSE_HELD; integration resumes between subsequent covered samples' if gap else None,
            'precision_evidence': 'NONE',
            'parameter_provenance': self.config['provenance']}


def assign_preview_odometry(sample, message):
    if sample.get('state') != 'UNVALIDATED' or sample.get('formal_odometry_eligible') is not False:
        raise FeedbackError('preview state cannot be promoted through this adapter')
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(sample['stamp_ns'], 1_000_000_000)
    message.header.frame_id = 'wheel_odom_preview'
    message.child_frame_id = 'axle_preview'
    message.pose.pose.position.x, message.pose.pose.position.y = sample['x_m'], sample['y_m']
    message.pose.pose.orientation.z = math.sin(sample['yaw_rad']/2)
    message.pose.pose.orientation.w = math.cos(sample['yaw_rad']/2)
    message.twist.twist.linear.x = sample['linear_velocity_m_s']
    message.twist.twist.angular.z = sample['angular_velocity_rad_s']
    for index in (0, 7, 14, 21, 28, 35):
        message.pose.covariance[index] = message.twist.covariance[index] = 1e6
    return message
