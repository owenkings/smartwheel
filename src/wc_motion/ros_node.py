"""ROS 2 wheel odometry from existing passive feedback records. Never opens tty.

Input: /wc_mapping/wheel/feedback_raw std_msgs/String, wc_wheel_feedback_v1.
Outputs: /wc_mapping/wheel/odom nav_msgs/Odometry (no TF), diagnostics String.
Raw-only mode accepts/preserves records without invented register semantics.
"""

import argparse
import json
import math
from pathlib import Path
import time

from .model import Geometry, WheelObservation, WheelOdometry
from .protocol import FeedbackError, FeedbackProfile


class FeedbackProcessor:
    def __init__(self, config):
        self.raw_only = config.get('mode') == 'raw_only'
        if config.get('mode') not in ('raw_only', 'wheel_odometry'):
            raise FeedbackError('explicit raw_only or wheel_odometry mode required')
        self.profile = self.model = None
        self.received = self.accepted = 0
        self.last_estimate = None
        self.last_error = None
        if not self.raw_only:
            self.profile = FeedbackProfile(config['protocol'])
            self.model = WheelOdometry(Geometry(**config['geometry']))
        self.maximum_record_bytes = 16384

    def process(self, text):
        self.received += 1
        if len(text.encode('utf-8')) > self.maximum_record_bytes:
            raise FeedbackError('raw feedback record exceeds bound')
        data = json.loads(text, parse_constant=lambda value: (_ for _ in ()).throw(FeedbackError('nonfinite JSON')))
        if not isinstance(data, dict):
            raise FeedbackError('feedback must be a JSON object')
        if self.raw_only:
            return data, None
        try:
            observation = WheelObservation.from_record(data, self.profile)
            estimate = self.model.update(observation)
            if estimate is not None:
                self.last_estimate = estimate
                self.accepted += 1
            return data, estimate
        except (ValueError, KeyError, TypeError) as error:
            self.last_error = str(error)
            # Malformed frames do not move the pose. The next valid sample must
            # still satisfy the model's maximum-gap constraint.
            raise

    def status(self, stale=False):
        estimate = self.last_estimate
        return {'schema': 'wc_wheel_diagnostics_v1',
                'state': 'RAW_ONLY' if self.raw_only else ('BLOCKED' if self.model.blocked else ('STALE' if stale else 'FEEDBACK')),
                'received': self.received, 'accepted': self.accepted,
                'last_error': self.last_error, 'publishes_tf': False,
                'time_valid': bool(estimate and not stale and not self.raw_only and not self.model.blocked),
                'time_source': estimate.time_source if estimate else None,
                'stamp_ns': estimate.stamp_ns if estimate else None,
                'uncertainty_ns': estimate.uncertainty_ns if estimate else None,
                'stream_epoch': estimate.stream_epoch if estimate else None,
                'sequence': estimate.sequence if estimate else None,
                'origin': 'arbitrary_first_accepted_axle_pose', 'localized_in_map': False}


def assign_odometry(estimate, message):
    sec, nanos = divmod(estimate.stamp_ns, 1_000_000_000)
    if not -(2**31) <= sec < 2**31:
        raise FeedbackError('wheel timestamp outside ROS Time sec range')
    message.header.stamp.sec, message.header.stamp.nanosec = sec, nanos
    message.header.frame_id, message.child_frame_id = 'wheel_odom', 'axle_link'
    message.pose.pose.position.x, message.pose.pose.position.y = estimate.x_m, estimate.y_m
    message.pose.pose.orientation.z = math.sin(estimate.yaw_rad / 2)
    message.pose.pose.orientation.w = math.cos(estimate.yaw_rad / 2)
    message.twist.twist.linear.x = estimate.linear_velocity_m_s
    message.twist.twist.angular.z = estimate.angular_velocity_rad_s
    # A pose covariance model is not calibrated. Keep pose uncertainty explicit
    # and large; consumers should fuse measured twist, not this integrated pose.
    for index in (0, 7, 14, 21, 28, 35):
        message.pose.covariance[index] = 1e6
        message.twist.covariance[index] = 1e6
    message.twist.covariance[0] = estimate.linear_velocity_variance
    message.twist.covariance[35] = estimate.angular_velocity_variance
    return message


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--raw-output', type=Path)
    parser.add_argument('--stale-timeout', type=float, default=1.)
    args, ros_args = parser.parse_known_args(argv)
    if not math.isfinite(args.stale_timeout) or args.stale_timeout <= 0:
        raise FeedbackError('positive finite stale timeout required')
    processor = FeedbackProcessor(json.loads(args.config.read_text(encoding='utf-8')))
    # ROS imports are deliberately delayed so all decoder/model tests are offline.
    import rclpy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    rclpy.init(args=ros_args)
    node = rclpy.create_node('wheel_feedback_odometry')
    qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
    odom = node.create_publisher(Odometry, '/wc_mapping/wheel/odom', qos)
    diagnostics = node.create_publisher(String, '/wc_mapping/wheel/diagnostics', qos)
    output = args.raw_output.open('x', encoding='utf-8') if args.raw_output else None
    last_valid_monotonic = [None]

    def publish_status():
        now = time.monotonic()
        stale = last_valid_monotonic[0] is None or now - last_valid_monotonic[0] > args.stale_timeout
        diagnostics.publish(String(data=json.dumps(processor.status(stale), allow_nan=False)))

    def callback(message):
        try:
            raw, estimate = processor.process(message.data)
            if output:
                output.write(json.dumps(raw, allow_nan=False) + '\n')
                output.flush()
            if estimate is not None:
                odom.publish(assign_odometry(estimate, Odometry()))
                last_valid_monotonic[0] = time.monotonic()
        except (ValueError, KeyError, TypeError) as error:
            processor.last_error = str(error)
            node.get_logger().error('wheel feedback rejected: ' + str(error))
        publish_status()

    subscription = node.create_subscription(String, '/wc_mapping/wheel/feedback_raw', callback, qos)
    timer = node.create_timer(.5, publish_status)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if output:
            output.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
