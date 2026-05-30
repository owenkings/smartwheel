"""Loosely-coupled wheel vs LIVO odometry consistency monitor.

Compares short-window linear/angular velocity and accumulated displacement
between /wheel/odom and /livo/odom. Publishes a JSON status and a 0..1
consistency score for the safety_supervisor to consume. It NEVER commands an
e-stop itself; it only reports. Large divergence usually means LIVO drift,
wheel slip, or an encoder fault.
"""
import json
import math
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _stamp_sec(msg, node) -> float:
    s = msg.header.stamp
    t = s.sec + s.nanosec * 1e-9
    return t if t > 0.0 else node.get_clock().now().nanoseconds * 1e-9


class WheelLivoConsistencyMonitor(Node):
    def __init__(self):
        super().__init__("wheel_livo_consistency_monitor")
        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter("livo_odom_topic", "/livo/odom")
        self.declare_parameter("output_status_topic", "/livo_wheel/status")
        self.declare_parameter("score_topic", "/livo_wheel/consistency_score")
        self.declare_parameter("window_sec", 1.0)
        self.declare_parameter("max_linear_velocity_diff", 0.15)
        self.declare_parameter("max_angular_velocity_diff", 0.30)
        self.declare_parameter("max_distance_diff", 0.20)
        self.declare_parameter("warning_hold_sec", 3.0)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("source_timeout_sec", 1.0)

        self.window = float(self.get_parameter("window_sec").value)
        self.max_dv = float(self.get_parameter("max_linear_velocity_diff").value)
        self.max_dw = float(self.get_parameter("max_angular_velocity_diff").value)
        self.max_dd = float(self.get_parameter("max_distance_diff").value)
        self.hold = float(self.get_parameter("warning_hold_sec").value)
        self.timeout = float(self.get_parameter("source_timeout_sec").value)

        self.wheel = deque()
        self.livo = deque()
        self.wheel_last = 0.0
        self.livo_last = 0.0
        self.warn_until = 0.0

        self.create_subscription(Odometry, self.get_parameter("wheel_odom_topic").value,
                                 lambda m: self._on_odom(m, self.wheel, "wheel"), 20)
        self.create_subscription(Odometry, self.get_parameter("livo_odom_topic").value,
                                 lambda m: self._on_odom(m, self.livo, "livo"), 20)
        self.status_pub = self.create_publisher(String, self.get_parameter("output_status_topic").value, 10)
        self.score_pub = self.create_publisher(Float32, self.get_parameter("score_topic").value, 10)
        self.timer = self.create_timer(1.0 / max(1.0, float(self.get_parameter("publish_rate_hz").value)), self._tick)
        self.get_logger().info("wheel_livo_consistency_monitor started (report-only, no e-stop)")

    def _on_odom(self, msg, buf, which):
        t = _stamp_sec(msg, self)
        p = msg.pose.pose
        buf.append((t, p.position.x, p.position.y, _yaw(p.orientation)))
        if which == "wheel":
            self.wheel_last = self.get_clock().now().nanoseconds * 1e-9
        else:
            self.livo_last = self.get_clock().now().nanoseconds * 1e-9
        cutoff = t - max(self.window * 2.0, 2.0)
        while buf and buf[0][0] < cutoff:
            buf.popleft()

    def _window_motion(self, buf):
        if len(buf) < 2:
            return None
        t_new = buf[-1][0]
        seg = [s for s in buf if t_new - s[0] <= self.window]
        if len(seg) < 2:
            return None
        t0, x0, y0, yaw0 = seg[0]
        t1, x1, y1, yaw1 = seg[-1]
        dt = t1 - t0
        if dt <= 1e-3:
            return None
        dist = math.hypot(x1 - x0, y1 - y0)
        dyaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
        return dist, abs(dyaw), dist / dt, abs(dyaw) / dt

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        wheel_online = (now - self.wheel_last) <= self.timeout and self.wheel_last > 0.0
        livo_online = (now - self.livo_last) <= self.timeout and self.livo_last > 0.0

        status = {"wheel_online": wheel_online, "livo_online": livo_online,
                  "warning": False, "state": "OK"}

        if not livo_online:
            status["state"] = "WAITING_LIVO"
            status["note"] = "no /livo/odom (LIVO backend off or not converged); not comparing"
            self.status_pub.publish(String(data=json.dumps(status)))
            self.score_pub.publish(Float32(data=1.0))
            return

        wm = self._window_motion(self.wheel)
        lm = self._window_motion(self.livo)
        if wm is None or lm is None:
            status["state"] = "WARMUP"
            self.status_pub.publish(String(data=json.dumps(status)))
            self.score_pub.publish(Float32(data=1.0))
            return

        dist_diff = abs(wm[0] - lm[0])
        dv = abs(wm[2] - lm[2])
        dw = abs(wm[3] - lm[3])
        r_dist = dist_diff / self.max_dd if self.max_dd > 0 else 0.0
        r_v = dv / self.max_dv if self.max_dv > 0 else 0.0
        r_w = dw / self.max_dw if self.max_dw > 0 else 0.0
        worst = max(r_dist, r_v, r_w)
        score = max(0.0, min(1.0, 1.0 - worst))

        exceeded = worst > 1.0
        if exceeded:
            self.warn_until = now + self.hold
        warning = now < self.warn_until

        status.update({
            "warning": bool(warning),
            "state": "DIVERGENCE" if warning else "OK",
            "linear_vel_diff": round(dv, 3), "angular_vel_diff": round(dw, 3),
            "short_term_distance_diff": round(dist_diff, 3),
            "wheel_speed": round(wm[2], 3), "livo_speed": round(lm[2], 3),
            "consistency_score": round(score, 3),
            "thresholds": {"linear": self.max_dv, "angular": self.max_dw, "distance": self.max_dd},
        })
        if exceeded:
            self.get_logger().warning(
                f"wheel/LIVO divergence: dv={dv:.3f} dw={dw:.3f} dist={dist_diff:.3f} score={score:.2f}")
        self.status_pub.publish(String(data=json.dumps(status)))
        self.score_pub.publish(Float32(data=float(score)))


def main(args=None):
    rclpy.init(args=args)
    node = WheelLivoConsistencyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
