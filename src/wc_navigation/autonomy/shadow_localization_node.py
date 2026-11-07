"""Static transforms and localization samples for the fixed and AMCL shadow modes.

The replay's own static transforms are remapped away in these modes. This node
publishes static ``base_link -> laser`` (the frozen lever arm) and, in fixed
mode, a static ``map -> odom`` equal to the rigid alignment. It samples
``map -> odom`` from TF at 10 Hz and records every ``/amcl_pose`` with its
covariance in ``localization_samples.csv`` (flushed per row). It publishes no
velocity and subscribes to no command.
"""
from __future__ import annotations

import argparse
import csv
import math

from .ros_util import latched_qos, load_config, lookup_pose, require_isolated_domain, yaw_to_quaternion

FIELDS = ("kind", "sim_time_s", "stamp_s", "x", "y", "yaw", "cov_xx", "cov_xy", "cov_yy", "cov_yawyaw")


class LocalizationHelper:
    def __init__(self, node, config):
        from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
        from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
        self.node = node
        mode = config["mode"]
        if mode not in ("fixed", "amcl"):
            raise ValueError("helper mode must be fixed or amcl")
        lever = [float(v) for v in config["lever_xy"]]
        statics = [self._transform(TransformStamped, "base_link", "laser", (lever[0], lever[1], 0.0))]
        if mode == "fixed":
            statics.append(self._transform(TransformStamped, "map", "odom", config["map_odom_xyyaw"]))
        self.static_tf = StaticTransformBroadcaster(node)
        self.static_tf.sendTransform(statics)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.stream = open(config["samples_path"], "x", newline="", encoding="utf-8")
        self.writer = csv.writer(self.stream)
        self.writer.writerow(FIELDS)
        self.stream.flush()
        self.last_stamp = None
        if mode == "amcl":
            node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_pose, latched_qos())
        node.create_timer(0.1, self.sample)

    def _transform(self, kind, parent, child, pose):
        t = kind()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y = float(pose[0]), float(pose[1])
        qx, qy, qz, qw = yaw_to_quaternion(float(pose[2]))
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = qx, qy, qz, qw
        return t

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def write(self, row):
        self.writer.writerow([row[0]] + [("%.9f" % v) if isinstance(v, float) else v for v in row[1:]])
        self.stream.flush()

    def on_pose(self, message):
        from .ros_util import quaternion_to_yaw, stamp_seconds
        p, c = message.pose.pose, message.pose.covariance
        self.write(["amcl_pose", self.now(), stamp_seconds(message.header.stamp), p.position.x, p.position.y,
                    quaternion_to_yaw(p.orientation), float(c[0]), float(c[1]), float(c[7]), float(c[35])])

    def sample(self):
        now = self.now()
        if now <= 0:
            return
        found = lookup_pose(self.buffer, "map", "odom")
        if found is None:
            return
        (x, y, yaw), stamp = found
        if stamp == self.last_stamp and self.last_stamp is not None and stamp > 0:
            return
        self.last_stamp = stamp
        self.write(["map_odom", now, stamp, x, y, yaw, math.nan, math.nan, math.nan, math.nan])

    def close(self):
        self.stream.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    args, ros_args = parser.parse_known_args(argv)
    require_isolated_domain()
    config = load_config(args.config)
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import Parameter
    rclpy.init(args=ros_args)
    node = rclpy.create_node("wc_shadow_localization", parameter_overrides=[Parameter("use_sim_time", value=True)])
    helper = None
    try:
        helper = LocalizationHelper(node, config)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if helper is not None:
            helper.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
