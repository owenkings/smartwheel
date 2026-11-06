"""Sequential NavigateToPose mission on a saved map (AMCL localization).

Goals are map-frame poses chosen before the run. Each goal has a time limit;
a failed goal is recorded and, unless ``stop_on_failure``, the next goal is
attempted from wherever the robot stopped. Results go to mission_result.json.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from .ros_util import json_message, latched_qos, load_config, lookup_pose, pose_stamped, require_isolated_domain

SUCCEEDED = 4


class Mission:
    def __init__(self, node, config):
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformListener
        self.node, self.cfg = node, config
        self.output = Path(config["output_dir"])
        self.NavigateToPose = NavigateToPose
        self.nav = ActionClient(node, NavigateToPose, "/navigate_to_pose")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.status_pub = node.create_publisher(String, "/wc_nav/mission_status", latched_qos())
        self.goals = [tuple(float(v) for v in g) for g in config["goals"]]
        if not self.goals or any(len(g) != 3 or not all(math.isfinite(v) for v in g) for g in self.goals):
            raise ValueError("mission needs finite map-frame goals [x, y, yaw]")
        self.index = 0
        self.records = []
        self.pending = None
        self.start_time = None
        self.finished = None
        node.create_timer(0.5, self.tick)

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def pose(self):
        found = lookup_pose(self.buffer, "map", "base_link")
        return None if found is None else found[0]

    def tick(self):
        if self.finished is not None:
            return
        now = self.now()
        if now <= 0:
            return
        if self.start_time is None:
            if not self.nav.server_is_ready() or self.pose() is None:
                return
            self.start_time = now
        if now - self.start_time > float(self.cfg.get("max_duration_s", 900.0)):
            if self.pending and self.pending.get("handle") is not None:
                self.pending["handle"].cancel_goal_async()
            return self.finish("STOPPED_DURATION_LIMIT")
        if self.pending is None:
            if self.index >= len(self.goals):
                ok = all(r["succeeded"] for r in self.records)
                return self.finish("COMPLETE" if ok else "COMPLETE_WITH_FAILURES")
            x, y, yaw = self.goals[self.index]
            start = self.pose()
            distance = math.dist(start[:2], (x, y)) if start else 5.0
            timeout = max(float(self.cfg.get("goal_timeout_min_s", 60.0)),
                          4.0 * distance / max(0.05, float(self.cfg.get("nominal_speed", 0.2))) + 30.0)
            goal = self.NavigateToPose.Goal()
            goal.pose = pose_stamped("map", self.node.get_clock().now().to_msg(), x, y, yaw)
            self.pending = {"future": self.nav.send_goal_async(goal), "handle": None, "result": None,
                            "deadline": now + timeout,
                            "record": {"index": self.index, "goal": [x, y, yaw], "start_pose": list(start or ()),
                                       "sent_time_s": round(now, 3), "timeout_s": round(timeout, 1)}}
            self.status_pub.publish(json_message({"state": "NAVIGATING", "goal_index": self.index, "goal": [x, y, yaw]}))
            return
        p = self.pending
        status = None
        if p["handle"] is None:
            if p["future"].done():
                p["handle"] = p["future"].result()
                if p["handle"] is None or not p["handle"].accepted:
                    status = 6
                else:
                    p["result"] = p["handle"].get_result_async()
            elif now > p["deadline"]:
                status = 5
        if status is None and p["result"] is not None:
            if p["result"].done():
                status = p["result"].result().status
            elif now > p["deadline"]:
                p["handle"].cancel_goal_async()
                p["record"]["timed_out"] = True
                status = 5
        if status is None:
            return
        end = self.pose()
        record = p["record"]
        record.update(status=int(status), succeeded=status == SUCCEEDED, finished_time_s=round(now, 3),
                      end_pose=list(end or ()))
        self.records.append(record)
        self.status_pub.publish(json_message({"state": "GOAL_FINISHED", "record": record}))
        self.pending = None
        self.index += 1
        if status != SUCCEEDED and self.cfg.get("stop_on_failure", False):
            return self.finish("STOPPED_ON_FAILURE")

    def finish(self, reason):
        result = {"schema_version": 1, "status": reason, "goals": [list(g) for g in self.goals],
                  "records": self.records, "hardware_control": False, "physical_navigation_validated": False}
        with (self.output / "mission_result.json").open("x", encoding="utf-8") as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        self.finished = reason
        self.status_pub.publish(json_message({"state": "FINISHED", "status": reason}))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    args, ros_args = parser.parse_known_args(argv)
    require_isolated_domain()
    config = load_config(args.config)
    import rclpy
    from rclpy.parameter import Parameter
    rclpy.init(args=ros_args)
    node = rclpy.create_node("wc_mission", parameter_overrides=[Parameter("use_sim_time", value=True)])
    mission = Mission(node, config)
    from rclpy.executors import ExternalShutdownException
    try:
        while rclpy.ok() and mission.finished is None:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if mission.finished == "COMPLETE" else 2 if mission.finished else 1


if __name__ == "__main__":
    raise SystemExit(main())
