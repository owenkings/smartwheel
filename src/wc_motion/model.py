"""Differential-wheel odometry with explicit units, timing and geometry.

The wheel_odom origin is the first accepted axle pose, an arbitrary local
reference, not map localization. No TF is published by this package.
"""

from dataclasses import asdict, dataclass
from math import cos, sin, pi, isfinite

from .protocol import FeedbackError, integer


def positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not isfinite(value) or value <= 0:
        raise FeedbackError(f'explicit positive finite {name} required')
    return value


@dataclass(frozen=True)
class Geometry:
    left_radius_m: float
    right_radius_m: float
    track_width_m: float
    left_sign: int
    right_sign: int
    left_motor_turns_per_wheel_turn: float
    right_motor_turns_per_wheel_turn: float
    counts_per_motor_turn: int | None
    max_wheel_speed_m_s: float
    max_gap_s: float
    linear_velocity_variance: float
    angular_velocity_variance: float

    def __post_init__(self):
        for name in ('left_radius_m', 'right_radius_m', 'track_width_m',
                     'left_motor_turns_per_wheel_turn', 'right_motor_turns_per_wheel_turn',
                     'max_wheel_speed_m_s', 'max_gap_s', 'linear_velocity_variance', 'angular_velocity_variance'):
            positive(getattr(self, name), name)
        if any(type(value) is not int or value not in (-1, 1) for value in (self.left_sign, self.right_sign)):
            raise FeedbackError('explicit measured left/right forward signs required')
        if self.counts_per_motor_turn is not None:
            integer(self.counts_per_motor_turn, 'counts_per_motor_turn', 1)


@dataclass(frozen=True)
class WheelObservation:
    stamp_ns: int
    time_valid: bool
    time_source: str
    uncertainty_ns: int | None
    stream_epoch: str
    sequence: int
    left_velocity_rpm: float | None = None
    right_velocity_rpm: float | None = None
    left_position_counts: int | None = None
    right_position_counts: int | None = None
    counter_bits: int | None = None

    @classmethod
    def from_record(cls, record, profile):
        values = profile.decode(record)
        field = profile.fields.get('left_position')
        return cls(stamp_ns=record['stamp_ns'], time_valid=record['time_valid'],
                   time_source=record['time_source'], uncertainty_ns=record['uncertainty_ns'],
                   stream_epoch=record['stream_epoch'], sequence=record['sequence'],
                   left_velocity_rpm=values.get('left_velocity'), right_velocity_rpm=values.get('right_velocity'),
                   left_position_counts=values.get('left_position'), right_position_counts=values.get('right_position'),
                   counter_bits=None if field is None else 16 * field.words)

    def validate(self):
        integer(self.stamp_ns, 'stamp_ns', 1)
        integer(self.sequence, 'sequence', 0)
        if self.time_valid is not True or self.time_source not in ('common_measurement', 'external_common_time'):
            raise FeedbackError('common measurement time required; arrival-only feedback cannot produce odometry')
        integer(self.uncertainty_ns, 'uncertainty_ns', 0)
        if not isinstance(self.stream_epoch, str) or not self.stream_epoch:
            raise FeedbackError('explicit acquisition epoch required')
        velocity = (self.left_velocity_rpm, self.right_velocity_rpm)
        position = (self.left_position_counts, self.right_position_counts)
        if any(v is None for v in velocity) != all(v is None for v in velocity):
            raise FeedbackError('both wheel velocities required together')
        if any(v is None for v in position) != all(v is None for v in position):
            raise FeedbackError('both wheel positions required together')
        if velocity[0] is None and position[0] is None:
            raise FeedbackError('no actual wheel feedback')
        if velocity[0] is not None and any(isinstance(v, bool) or not isinstance(v, (float, int)) or not isfinite(v) for v in velocity):
            raise FeedbackError('finite actual wheel velocities required')
        if position[0] is not None:
            if self.counter_bits not in (16, 32):
                raise FeedbackError('verified position counter width required')
            for value in position:
                integer(value, 'position count', -(1 << (self.counter_bits - 1)), (1 << self.counter_bits) - 1)


@dataclass(frozen=True)
class MotionEstimate:
    stamp_ns: int
    stream_epoch: str
    sequence: int
    x_m: float
    y_m: float
    yaw_rad: float
    linear_velocity_m_s: float
    angular_velocity_rad_s: float
    time_valid: bool
    time_source: str
    uncertainty_ns: int
    linear_velocity_variance: float
    angular_velocity_variance: float
    source: str
    frame_id: str = 'wheel_odom'
    child_frame_id: str = 'axle_link'

    def to_dict(self):
        return asdict(self)


def wrapped_delta(now, previous, bits):
    modulus = 1 << bits
    delta = (now - previous + modulus // 2) % modulus - modulus // 2
    if abs(delta) == modulus // 2:
        raise FeedbackError('ambiguous half-range counter jump')
    return delta


class WheelOdometry:
    def __init__(self, geometry):
        self.geometry = geometry
        self.previous = None
        self.x = self.y = self.yaw = 0.0
        self.blocked = None

    def reset(self):
        """Explicit new local odometry origin; caller must use a new stream epoch."""
        self.previous = None
        self.x = self.y = self.yaw = 0.0
        self.blocked = None

    def _rpm_speeds(self, observation):
        g = self.geometry
        if observation.left_velocity_rpm is None:
            return None
        left = observation.left_velocity_rpm * (2 * pi / 60) * g.left_radius_m * g.left_sign / g.left_motor_turns_per_wheel_turn
        right = observation.right_velocity_rpm * (2 * pi / 60) * g.right_radius_m * g.right_sign / g.right_motor_turns_per_wheel_turn
        if max(abs(left), abs(right)) > g.max_wheel_speed_m_s:
            raise FeedbackError('wheel speed exceeds explicit plausibility bound')
        return left, right

    def update(self, observation):
        """Return MotionEstimate or None for first position-only sample.

        Discontinuity/invalid input latches BLOCKED, preserving last pose. No
        silent restart, zero-motion substitution, or integration across gaps.
        """
        if self.blocked:
            raise FeedbackError('motion model blocked: ' + self.blocked)
        try:
            return self._update(observation)
        except (FeedbackError, TypeError, KeyError) as error:
            self.blocked = str(error)
            raise

    def _update(self, obs):
        obs.validate()
        g, previous = self.geometry, self.previous
        speeds = self._rpm_speeds(obs)
        position_mode = obs.left_position_counts is not None
        if position_mode and g.counts_per_motor_turn is None:
            raise FeedbackError('position conversion requires encoder counts per motor revolution')
        if previous is not None:
            if obs.stream_epoch != previous.stream_epoch or obs.sequence <= previous.sequence or obs.stamp_ns <= previous.stamp_ns:
                raise FeedbackError('wheel identity/time discontinuity; explicit reset required')
            if position_mode != (previous.left_position_counts is not None) or obs.counter_bits != previous.counter_bits:
                raise FeedbackError('feedback mode changed during stream')
            dt = (obs.stamp_ns - previous.stamp_ns) * 1e-9
            if dt > g.max_gap_s:
                raise FeedbackError('wheel feedback gap too large')
            if position_mode:
                # Bound travel so rollover cannot hide one or more full turns
                # of the integer counter between accepted samples.
                resolutions = (2*pi*g.left_radius_m/g.left_motor_turns_per_wheel_turn/g.counts_per_motor_turn,
                               2*pi*g.right_radius_m/g.right_motor_turns_per_wheel_turn/g.counts_per_motor_turn)
                if any(g.max_wheel_speed_m_s * dt / resolution >= (1 << (obs.counter_bits - 1)) for resolution in resolutions):
                    raise FeedbackError('position interval admits ambiguous counter wrap')
                dl = wrapped_delta(obs.left_position_counts, previous.left_position_counts, obs.counter_bits) * resolutions[0] * g.left_sign
                dr = wrapped_delta(obs.right_position_counts, previous.right_position_counts, obs.counter_bits) * resolutions[1] * g.right_sign
                derived = dl / dt, dr / dt
                if max(abs(value) for value in derived) > g.max_wheel_speed_m_s:
                    raise FeedbackError('encoder count discontinuity / implausible wheel travel')
                speeds = speeds or derived
            else:
                before = self._rpm_speeds(previous)
                dl, dr = ((before[i] + speeds[i]) * .5 * dt for i in range(2))
            distance, turn = (dl + dr) / 2, (dr - dl) / g.track_width_m
            # Exact integration of a constant-curvature segment (SE(2)).
            half = turn / 2
            sinc = sin(half) / half if abs(half) > 1e-10 else 1 - half * half / 6
            self.x += distance * sinc * cos(self.yaw + half)
            self.y += distance * sinc * sin(self.yaw + half)
            self.yaw = (self.yaw + turn + pi) % (2 * pi) - pi
        self.previous = obs
        if speeds is None:
            return None
        return MotionEstimate(obs.stamp_ns, obs.stream_epoch, obs.sequence, self.x, self.y, self.yaw,
                              (speeds[0] + speeds[1]) / 2, (speeds[1] - speeds[0]) / g.track_width_m,
                              True, obs.time_source, obs.uncertainty_ns, g.linear_velocity_variance,
                              g.angular_velocity_variance, 'encoder_counts' if position_mode else 'actual_wheel_velocity')


def axle_twist_at_rig(estimate, *, rig_origin_in_axle_m, yaw_axle_from_rig_rad):
    """Planar rigid-body velocity at rig origin, expressed in rig coordinates.

    Input translation is the rig origin expressed in axle_link, not lidar
    baseline. A full 3D transform is required by the main estimator for tilted
    rigs; this helper deliberately supports the planar special case only.
    """
    if len(rig_origin_in_axle_m) != 2 or not all(isfinite(x) for x in (*rig_origin_in_axle_m, yaw_axle_from_rig_rad)):
        raise FeedbackError('explicit finite planar axle-to-rig relationship required')
    x, y = rig_origin_in_axle_m
    w, v = estimate.angular_velocity_rad_s, estimate.linear_velocity_m_s
    vx, vy = v - w * y, w * x
    c, s = cos(yaw_axle_from_rig_rad), sin(yaw_axle_from_rig_rad)
    return c * vx + s * vy, -s * vx + c * vy, w
