"""Shadow mission along a recording, in one of two modes.

``navigate``: keep NavigateToPose aimed at the recorded axle pose ``lead_s``
after the current recording time (the window end at most), so the planner
chooses the route. ``follow``: send the driver's own recorded path for the next
``lead_s`` straight to the controller (FollowPath), so planning and the saved
map are left out and only the controller, collision monitor and gate are judged.

The request is re-sent every ``update_period_s`` once its end has moved, and
the server preempts the previous one. A request that failed is retried after
``retry_s``; a reached one is only replaced when the end moves. The task ends
when the replay reports FINISHED and writes shadow_mission_result.json.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from .ros_util import json_message, latched_qos, load_config, pose_stamped, require_isolated_domain
from .shadow import carrot_goal, goal_moved, path_ahead

SUCCEEDED = 4
MODES = ("navigate", "follow")


class Carrot:
    def __init__(self, node, config):
        from nav2_msgs.action import FollowPath, NavigateToPose
        from rclpy.action import ActionClient
        from std_msgs.msg import String
        from .replay_node import window_frames
        self.node, self.cfg = node, config
        self.output = Path(config["output_dir"])
        self.mode = config.get("mode", "navigate")
        if self.mode not in MODES:
            raise ValueError("mode must be navigate or follow")
        with np.load(config["recording"]) as data:
            recording = {key: data[key] for key in ("times_s", "base_poses")}
        index = window_frames(recording, config["window_s"])
        self.times, self.poses = recording["times_s"][index], recording["base_poses"][index]
        self.end = float(self.times[-1])
        self.lead = float(config.get("lead_s", 15.0))
        self.period = float(config.get("update_period_s", 4.0))
        self.retry = float(config.get("retry_s", 2.0))
        self.min_move = float(config.get("min_goal_change_m", 0.3))
        self.min_turn = float(config.get("min_goal_change_rad", 0.3))
        if not (0.0 < self.lead <= 60.0 and 0.5 <= self.period <= 30.0):
            raise ValueError("carrot lead must be in (0, 60] s and the update period in [0.5, 30] s")
        self.Action = NavigateToPose if self.mode == "navigate" else FollowPath
        self.nav = ActionClient(node, self.Action, "/navigate_to_pose" if self.mode == "navigate" else "/follow_path")
        self.status_pub = node.create_publisher(String, "/wc_nav/mission_status", latched_qos())
        node.create_subscription(String, "/wc_shadow/status", self.on_replay, latched_qos())
        self.replay = None
        self.ready = False
        self.records = []
        self.active = None
        self.last_sent = -math.inf
        self.finished = None
        node.create_timer(0.25, self.tick)

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def on_replay(self, message):
        self.replay = json.loads(message.data)

    def tick(self):
        if self.finished is not None:
            return
        now = self.now()
        replay = self.replay or {}
        state = replay.get("state")
        if not self.ready:
            if state == "HOLD" and now > 0 and self.nav.server_is_ready():
                self.ready = True
                self.status_pub.publish(json_message({"state": "READY", "mode": self.mode, "lead_s": self.lead}))
            return
        self.poll(now)
        if state == "FINISHED":
            return self.finish("COMPLETE")
        if state not in ("PLAYING", "TAIL_HOLD") or replay.get("playback_start_s") is None:
            return
        r = float(replay["window_s"][0]) + now - float(replay["playback_start_s"])
        goal = carrot_goal(self.times, self.poses, r, self.lead, self.end)
        last = self.records[-1] if self.records else None
        moved = goal_moved(goal, tuple(last["goal"]) if last else None, self.min_move, self.min_turn)
        if self.active is not None:
            due = moved and now - self.last_sent >= self.period
        elif last is None:
            due = True
        elif last.get("status") == SUCCEEDED:
            due = moved
        else:
            due = now - self.last_sent >= self.retry
        if due:
            self.send(goal, r, now)

    def request(self, goal, r):
        stamp = self.node.get_clock().now().to_msg()
        message = self.Action.Goal()
        if self.mode == "navigate":
            message.pose = pose_stamped("map", stamp, *goal)
            return message, {}
        from nav_msgs.msg import Path as PathMessage
        path = PathMessage()
        path.header.frame_id, path.header.stamp = "map", stamp
        points = path_ahead(self.times, self.poses, r, self.lead, self.end)
        path.poses = [pose_stamped("map", stamp, *p) for p in points]
        message.path = path
        message.controller_id, message.goal_checker_id = "FollowPath", "general_goal_checker"
        return message, {"path_points": len(points)}

    def send(self, goal, r, now):
        if self.active is not None:
            self.active["record"]["outcome"] = "PREEMPTED"
            self.active["record"]["finished_time_s"] = round(now, 3)
        message, extra = self.request(goal, r)
        record = {"index": len(self.records), "goal": [round(v, 4) for v in goal], "sent_time_s": round(now, 3),
                  "recording_time_s": round(r, 3), "goal_recording_time_s": round(min(r + self.lead, self.end), 3),
                  **extra}
        self.records.append(record)
        self.active = {"future": self.nav.send_goal_async(message), "handle": None, "result": None,
                       "record": record}
        self.last_sent = now
        self.status_pub.publish(json_message({"state": "NAVIGATING", "goal_index": record["index"],
                                              "goal": record["goal"], "mode": self.mode}))

    def poll(self, now):
        p = self.active
        if p is None:
            return
        if p["handle"] is None and p["future"].done():
            p["handle"] = p["future"].result()
            if p["handle"] is None or not p["handle"].accepted:
                p["record"].update(status=6, outcome="REJECTED", finished_time_s=round(now, 3))
                self.active = None
                return
            p["result"] = p["handle"].get_result_async()
        if p["result"] is not None and p["result"].done():
            status = int(p["result"].result().status)
            p["record"].update(status=status, outcome="SUCCEEDED" if status == SUCCEEDED else "FAILED",
                               finished_time_s=round(now, 3))
            self.active = None

    def finish(self, reason):
        if self.active is not None and self.active.get("handle") is not None:
            self.active["handle"].cancel_goal_async()
            self.active["record"].update(outcome="CANCELED_AT_END", finished_time_s=round(self.now(), 3))
        self.active = None
        counts = {}
        for record in self.records:
            counts[record.get("outcome", "UNFINISHED")] = counts.get(record.get("outcome", "UNFINISHED"), 0) + 1
        result = {"schema_version": 1, "status": reason, "mode": self.mode, "lead_s": self.lead,
                  "update_period_s": self.period, "goals_sent": len(self.records), "outcome_counts": counts,
                  "records": self.records, "goal_frame": "map (recording odometry frame)",
                  "hardware_control": False, "physical_navigation_validated": False}
        with (self.output / "shadow_mission_result.json").open("x", encoding="utf-8") as stream:
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
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import Parameter
    rclpy.init(args=ros_args)
    node = rclpy.create_node("wc_shadow_carrot", parameter_overrides=[Parameter("use_sim_time", value=True)])
    carrot = Carrot(node, config)
    try:
        while rclpy.ok() and carrot.finished is None:
            rclpy.spin_once(node, timeout_sec=0.2)
        for _ in range(5):  # let the final cancel request go out
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if carrot.finished == "COMPLETE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
