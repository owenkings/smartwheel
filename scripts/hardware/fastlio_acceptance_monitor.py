#!/usr/bin/env python3
"""Bounded, read-only FAST-LIO timing/jump/STOP acceptance monitor."""

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = max(
        0,
        min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1),
    )
    return ordered[index]


def quaternion_step_deg(first, second):
    dot = abs(
        first.x * second.x
        + first.y * second.y
        + first.z * second.z
        + first.w * second.w
    )
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


class FastLioAcceptanceMonitor(Node):
    def __init__(self, args):
        super().__init__("fastlio_acceptance_monitor")
        self.args = args
        self.started_monotonic = time.monotonic()
        self.odom_receive_times = []
        self.odom_header_times = []
        self.cloud_receive_times = []
        self.position_steps = []
        self.rotation_steps_deg = []
        self.first_position = None
        self.previous_position = None
        self.previous_orientation = None
        self.stationary_radius_max_m = 0.0
        self.stop_anchor = None
        self.stop_since_monotonic = None
        self.stopped_drift_max_m = 0.0
        self.stop_hold_samples = 0
        self.wheel_receive_monotonic = None
        self.wheel_linear_mps = None
        self.wheel_angular_rps = None
        self.wheel_linear_abs_max_mps = 0.0
        self.wheel_angular_abs_max_rps = 0.0
        self.feedback_receive_monotonic = None
        self.feedback_healthy = False
        self.feedback_true_messages = 0
        self.feedback_false_messages = 0
        self.quality_messages = 0
        self.quality_rejected_messages = 0
        self.quality_temporal_jump_messages = 0
        self.quality_relative_drop_messages = 0
        self.zero_pose_covariance_frames = 0

        self.create_subscription(Odometry, args.odom_topic, self.on_odom, 50)
        self.create_subscription(Odometry, args.wheel_topic, self.on_wheel, 50)
        self.create_subscription(Bool, args.health_topic, self.on_health, 20)
        self.create_subscription(
            PointCloud2,
            args.cloud_topic,
            self.on_cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String, args.quality_topic, self.on_quality, 20
        )

    def on_health(self, msg):
        self.feedback_receive_monotonic = time.monotonic()
        self.feedback_healthy = bool(msg.data)
        if self.feedback_healthy:
            self.feedback_true_messages += 1
        else:
            self.feedback_false_messages += 1

    def on_wheel(self, msg):
        self.wheel_receive_monotonic = time.monotonic()
        self.wheel_linear_mps = float(msg.twist.twist.linear.x)
        self.wheel_angular_rps = float(msg.twist.twist.angular.z)
        self.wheel_linear_abs_max_mps = max(
            self.wheel_linear_abs_max_mps, abs(self.wheel_linear_mps)
        )
        self.wheel_angular_abs_max_rps = max(
            self.wheel_angular_abs_max_rps, abs(self.wheel_angular_rps)
        )

    def on_cloud(self, _msg):
        self.cloud_receive_times.append(time.monotonic())

    def on_quality(self, msg):
        try:
            quality = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        self.quality_messages += 1
        if not bool(quality.get("accepted", True)):
            self.quality_rejected_messages += 1
        if bool(quality.get("temporal_jump_detected", False)):
            self.quality_temporal_jump_messages += 1
        if bool(quality.get("relative_validity_drop_detected", False)):
            self.quality_relative_drop_messages += 1

    def on_odom(self, msg):
        receive_time = time.monotonic()
        self.odom_receive_times.append(receive_time)
        self.odom_header_times.append(stamp_seconds(msg.header.stamp))
        position = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
        )
        orientation = msg.pose.pose.orientation
        if self.first_position is None:
            self.first_position = position
        else:
            self.stationary_radius_max_m = max(
                self.stationary_radius_max_m,
                math.dist(position, self.first_position),
            )
        if self.previous_position is not None:
            self.position_steps.append(
                math.dist(position, self.previous_position)
            )
            self.rotation_steps_deg.append(
                quaternion_step_deg(orientation, self.previous_orientation)
            )
        self.previous_position = position
        self.previous_orientation = orientation
        if all(abs(float(value)) <= 1.0e-15 for value in msg.pose.covariance):
            self.zero_pose_covariance_frames += 1

        wheel_fresh = (
            self.wheel_receive_monotonic is not None
            and receive_time - self.wheel_receive_monotonic
            <= self.args.wheel_stale_sec
        )
        health_fresh = (
            self.feedback_receive_monotonic is not None
            and receive_time - self.feedback_receive_monotonic
            <= self.args.health_stale_sec
        )
        stopped = (
            wheel_fresh
            and health_fresh
            and self.feedback_healthy
            and abs(self.wheel_linear_mps) <= self.args.stop_linear_mps
            and abs(self.wheel_angular_rps) <= self.args.stop_angular_rps
        )
        if not stopped:
            self.stop_anchor = None
            self.stop_since_monotonic = None
            return
        if self.stop_anchor is None:
            self.stop_anchor = position
            self.stop_since_monotonic = receive_time
            return
        if receive_time - self.stop_since_monotonic >= self.args.stop_hold_sec:
            self.stop_hold_samples += 1
            self.stopped_drift_max_m = max(
                self.stopped_drift_max_m, math.dist(position, self.stop_anchor)
            )

    @staticmethod
    def gap_metrics(receive_times):
        gaps = [b - a for a, b in zip(receive_times, receive_times[1:])]
        return {
            "max_sec": max(gaps, default=None),
            "p95_sec": percentile(gaps, 0.95),
            "over_0_5_sec": sum(gap > 0.5 for gap in gaps),
        }

    def report(self):
        finished = time.monotonic()
        elapsed = finished - self.started_monotonic
        odom_gaps = self.gap_metrics(self.odom_receive_times)
        cloud_gaps = self.gap_metrics(self.cloud_receive_times)
        tail_silence = (
            finished - self.odom_receive_times[-1]
            if self.odom_receive_times
            else elapsed
        )
        wheel_tail_age = (
            finished - self.wheel_receive_monotonic
            if self.wheel_receive_monotonic is not None
            else None
        )
        feedback_tail_age = (
            finished - self.feedback_receive_monotonic
            if self.feedback_receive_monotonic is not None
            else None
        )
        cloud_tail_silence = (
            finished - self.cloud_receive_times[-1]
            if self.cloud_receive_times
            else elapsed
        )
        max_odom_gap = max(
            [
                value
                for value in (odom_gaps["max_sec"], tail_silence)
                if value is not None
            ],
            default=None,
        )
        max_cloud_gap = max(
            [
                value
                for value in (cloud_gaps["max_sec"], cloud_tail_silence)
                if value is not None
            ],
            default=None,
        )
        header_gaps = [
            b - a
            for a, b in zip(self.odom_header_times, self.odom_header_times[1:])
        ]
        result = {
            "mode": self.args.mode,
            "elapsed_sec": elapsed,
            "odom": {
                "samples": len(self.odom_receive_times),
                "receive_rate_hz": (
                    len(self.odom_receive_times) / elapsed
                    if elapsed > 0.0
                    else 0.0
                ),
                "receive_gap": odom_gaps,
                "tail_silence_sec": tail_silence,
                "max_gap_including_tail_sec": max_odom_gap,
                "header_gap_max_sec": max(header_gaps, default=None),
                "header_nonmonotonic_count": sum(
                    gap <= 0.0 for gap in header_gaps
                ),
                "position_step_max_m": max(self.position_steps, default=None),
                "position_step_p95_m": percentile(self.position_steps, 0.95),
                "rotation_step_max_deg": max(
                    self.rotation_steps_deg, default=None
                ),
                "stationary_radius_max_m": self.stationary_radius_max_m,
                "stopped_drift_max_m": self.stopped_drift_max_m,
                "stop_hold_samples": self.stop_hold_samples,
                "zero_pose_covariance_frames": (
                    self.zero_pose_covariance_frames
                ),
            },
            "registered_cloud": {
                "samples": len(self.cloud_receive_times),
                "receive_gap": cloud_gaps,
                "tail_silence_sec": cloud_tail_silence,
                "max_gap_including_tail_sec": max_cloud_gap,
            },
            "wheel": {
                "linear_abs_max_mps": self.wheel_linear_abs_max_mps,
                "angular_abs_max_rps": self.wheel_angular_abs_max_rps,
                "tail_age_sec": wheel_tail_age,
            },
            "feedback_health": {
                "true_messages": self.feedback_true_messages,
                "false_messages": self.feedback_false_messages,
                "current_healthy": self.feedback_healthy,
                "tail_age_sec": feedback_tail_age,
            },
            "quality": {
                "messages": self.quality_messages,
                "rejected_messages": self.quality_rejected_messages,
                "temporal_jump_messages": self.quality_temporal_jump_messages,
                "relative_drop_messages": self.quality_relative_drop_messages,
            },
        }
        failures = []
        if len(self.odom_receive_times) < 2:
            failures.append("fewer than two /Odometry samples")
        if (
            max_odom_gap is not None
            and max_odom_gap > self.args.max_odom_gap_sec
        ):
            failures.append("odometry output gap exceeds limit")
        if (
            self.position_steps
            and max(self.position_steps) > self.args.max_step_m
        ):
            failures.append("position step exceeds limit")
        if (
            self.rotation_steps_deg
            and max(self.rotation_steps_deg) > self.args.max_step_deg
        ):
            failures.append("rotation step exceeds limit")
        if self.zero_pose_covariance_frames:
            failures.append(
                "published odometry contains all-zero pose covariance"
            )
        if (
            self.args.mode == "stationary"
            and self.stationary_radius_max_m
            > self.args.max_stationary_radius_m
        ):
            failures.append("stationary activity radius exceeds limit")
        if self.stopped_drift_max_m > self.args.max_stopped_drift_m:
            failures.append("post-STOP drift exceeds limit")
        if self.args.require_feedback_health:
            if not self.feedback_true_messages:
                failures.append("no healthy wheel-feedback messages")
            if (
                feedback_tail_age is None
                or feedback_tail_age > self.args.health_stale_sec
                or not self.feedback_healthy
            ):
                failures.append("wheel-feedback health is stale or unhealthy")
            if (
                wheel_tail_age is None
                or wheel_tail_age > self.args.wheel_stale_sec
            ):
                failures.append("wheel odometry is stale")
        if self.args.require_stop_window and not self.stop_hold_samples:
            failures.append("no continuous healthy STOP window was observed")
        if self.args.require_cloud:
            if len(self.cloud_receive_times) < 2:
                failures.append("fewer than two registered-cloud samples")
            if (
                max_cloud_gap is not None
                and max_cloud_gap > self.args.max_cloud_gap_sec
            ):
                failures.append("registered-cloud output gap exceeds limit")
        if self.args.require_quality and not self.quality_messages:
            failures.append("no LiDAR quality messages")
        result["limits"] = {
            "max_odom_gap_sec": self.args.max_odom_gap_sec,
            "max_cloud_gap_sec": self.args.max_cloud_gap_sec,
            "max_step_m": self.args.max_step_m,
            "max_step_deg": self.args.max_step_deg,
            "max_stationary_radius_m": self.args.max_stationary_radius_m,
            "max_stopped_drift_m": self.args.max_stopped_drift_m,
        }
        result["passed"] = not failures
        result["failures"] = failures
        return result


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-sec", type=float, default=120.0)
    parser.add_argument(
        "--mode", choices=("stationary", "dynamic"), default="dynamic"
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--odom-topic", default="/Odometry")
    parser.add_argument("--wheel-topic", default="/wheel/odom")
    parser.add_argument(
        "--health-topic", default="/base/wheel_feedback_healthy"
    )
    parser.add_argument("--cloud-topic", default="/cloud_registered")
    parser.add_argument("--quality-topic", default="/xtm60/right/quality")
    parser.add_argument("--max-odom-gap-sec", type=float, default=0.50)
    parser.add_argument("--max-cloud-gap-sec", type=float, default=0.50)
    parser.add_argument("--max-step-m", type=float, default=0.05)
    parser.add_argument("--max-step-deg", type=float, default=2.0)
    parser.add_argument(
        "--max-stationary-radius-m", type=float, default=0.05
    )
    parser.add_argument(
        "--max-stopped-drift-m", type=float, default=0.05
    )
    parser.add_argument("--stop-linear-mps", type=float, default=0.02)
    parser.add_argument("--stop-angular-rps", type=float, default=0.02)
    parser.add_argument("--stop-hold-sec", type=float, default=0.30)
    parser.add_argument("--wheel-stale-sec", type=float, default=0.25)
    parser.add_argument("--health-stale-sec", type=float, default=0.25)
    parser.add_argument(
        "--require-feedback-health", action="store_true"
    )
    parser.add_argument("--require-stop-window", action="store_true")
    parser.add_argument("--require-cloud", action="store_true")
    parser.add_argument("--require-quality", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.duration_sec <= 0.0:
        raise SystemExit("--duration-sec must be positive")
    rclpy.init()
    node = FastLioAcceptanceMonitor(args)
    deadline = time.monotonic() + args.duration_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.10)
    except KeyboardInterrupt:
        pass
    result = node.report()
    rendered = json.dumps(result, indent=2, sort_keys=True)
    print(rendered)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if result["passed"] else 2)


if __name__ == "__main__":
    main()
