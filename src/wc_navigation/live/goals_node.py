"""Operator goal sequence for a live shadow run (NavigateToPose); the chair is pushed, not driven.

Goals are map-frame poses (x, y, yaw) given before the run. The next goal is
sent once Nav2 reports the current one reached, or once the pushed chair comes
within ``advance_radius_m`` of it, so a route of waypoints keeps Nav2 planning
ahead of the person pushing. A failed or rejected goal is re-sent after
``retry_s``. Status JSON goes to ``/wc_live/goal_status``; nothing here touches
a velocity topic.
"""
from __future__ import annotations

import argparse
import math

from ..autonomy.ros_util import json_message, latched_qos, load_config, lookup_pose, pose_stamped

SUCCEEDED = 4


class GoalSequence:
    def __init__(self, node, config):
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformListener
        self.node = node
        self.goals = [tuple(float(v) for v in g) for g in config["goals"]]
        if not self.goals or any(len(g) != 3 or not all(math.isfinite(v) for v in g) for g in self.goals):
            raise ValueError("goals must be finite map-frame [x, y, yaw]")
        self.advance = float(config.get("advance_radius_m", 1.0))
        self.retry = float(config.get("retry_s", 2.0))
        self.Action = NavigateToPose
        self.client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.status = node.create_publisher(String, "/wc_live/goal_status", latched_qos())
        self.index = 0
        self.active = None
        self.last_sent = -math.inf
        self.records = []
        self.finished = False
        node.create_timer(0.25, self.tick)

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def publish(self, state, **extra):
        self.status.publish(json_message({"state": state, "goal_index": self.index, "goals": len(self.goals),
                                          "time_s": round(self.now(), 3), **extra}))

    def send(self, now):
        x, y, yaw = self.goals[self.index]
        goal = self.Action.Goal()
        goal.pose = pose_stamped("map", self.node.get_clock().now().to_msg(), x, y, yaw)
        self.active = {"future": self.client.send_goal_async(goal), "handle": None, "result": None,
                       "index": self.index, "sent_s": now}
        self.last_sent = now
        self.publish("NAVIGATING", goal=[x, y, yaw])

    def advance_to_next(self, now, reason):
        self.records.append({"index": self.index, "reason": reason, "time_s": round(now, 3)})
        if self.active is not None and self.active.get("handle") is not None:
            self.active["handle"].cancel_goal_async()
        self.active = None
        self.index += 1
        if self.index >= len(self.goals):
            self.finished = True
            self.publish("FINISHED", records=self.records)

    def tick(self):
        if self.finished:
            return
        now = self.now()
        if now <= 0 or not self.client.server_is_ready():
            return
        found = lookup_pose(self.buffer, "map", "base_link")
        if found is not None:
            x, y, _ = found[0]
            gx, gy, _ = self.goals[self.index]
            if math.hypot(gx - x, gy - y) <= self.advance:
                return self.advance_to_next(now, "CHAIR_WITHIN_ADVANCE_RADIUS")
        p = self.active
        if p is None:
            if now - self.last_sent >= self.retry:
                self.send(now)
            return
        if p["handle"] is None and p["future"].done():
            p["handle"] = p["future"].result()
            if p["handle"] is None or not p["handle"].accepted:
                self.active = None
                self.publish("REJECTED")
                return
            p["result"] = p["handle"].get_result_async()
        if p["result"] is not None and p["result"].done():
            status = int(p["result"].result().status)
            if status == SUCCEEDED:
                return self.advance_to_next(now, "NAV2_REPORTED_REACHED")
            self.active = None
            self.publish("FAILED_RETRYING", status=status)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    args, ros_args = parser.parse_known_args(argv)
    config = load_config(args.config)
    from .supervisor import require_live_domain
    require_live_domain(config)
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import Parameter
    rclpy.init(args=ros_args)
    node = rclpy.create_node("wc_live_goals", parameter_overrides=[
        Parameter("use_sim_time", value=bool(config.get("use_sim_time", False)))])
    try:
        GoalSequence(node, config)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
