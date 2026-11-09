"""Sequential NavigateToPose mission on a saved map (AMCL localization).

Goals are map-frame poses chosen before the run. Each goal has a time limit;
a failed goal is recorded and, unless ``stop_on_failure``, the next goal is
attempted from wherever the robot stopped. Results go to mission_result.json.

With a ``goal_heading`` section in the config (``--goal-heading turn_at_goal``,
see planning/goal_heading.py) each goal first picks an arrival heading from
ComputePathToPose answers, navigates with that heading, and then turns in place
to the requested heading (Nav2 Spin) only when the area the outline sweeps is
clear in the saved map and the local costmap; otherwise the chair stays and the
heading difference is recorded. Each record then has a ``heading`` entry and
``succeeded`` also needs the requested heading. Without the section the mission
is unchanged.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from .ros_util import (json_message, latched_qos, load_config, lookup_pose, pose_stamped,
                       quaternion_to_yaw, require_isolated_domain)

SUCCEEDED, CANCELED, ABORTED = 4, 5, 6
SPIN_SPEED_FLOOR_RADPS = 0.15        # behavior_server min_rotational_vel: bounds the turn's time allowance


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
        self.heading = config.get("goal_heading") or None
        if self.heading:
            self._init_heading(node)
        node.create_timer(0.5, self.tick)

    # -- goal heading (turn_at_goal) ----------------------------------------------
    def _init_heading(self, node):
        from nav2_msgs.action import ComputePathToPose, Spin
        from nav_msgs.msg import OccupancyGrid
        from rclpy.action import ActionClient
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from ..planning.body import Body
        h = self.heading
        if h.get("mode") != "turn_at_goal":
            raise ValueError("goal_heading mode must be turn_at_goal")
        if not 0.0 < float(h["yaw_tolerance_rad"]) < math.pi or float(h["sweep_margin_m"]) < 0.0:
            raise ValueError("goal_heading needs a yaw tolerance in (0, pi) and a non-negative sweep margin")
        self.ComputePathToPose, self.Spin = ComputePathToPose, Spin
        self.plan_client = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
        self.spin_client = ActionClient(node, Spin, "/spin")
        self.body = Body.from_config(h["body"])
        self.saved_world = None
        self.local_costmap = None
        node.create_subscription(OccupancyGrid, "/map", self.on_map, latched_qos())
        # Volatile: matches the costmap publisher whatever its durability; the next update arrives within 0.5 s.
        node.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.on_local_costmap,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,
                                            reliability=ReliabilityPolicy.RELIABLE))
        # The plan requests and the turn are polled at 20 Hz: twelve requests then take about the planning time,
        # not twelve half-second ticks. The 0.5 s mission tick is unchanged.
        node.create_timer(0.05, self.fast_tick)

    def fast_tick(self):
        if self.finished is not None or self.pending is None or self.pending.get("phase") not in ("select", "turn"):
            return
        now = self.now()
        if now <= 0:
            return
        if self.pending["phase"] == "select":
            self.poll_select(now)
        else:
            self.poll_turn(now)

    @staticmethod
    def _grid(message):
        info = message.info
        data = np.asarray(message.data, dtype=np.int16).reshape(info.height, info.width)
        origin = (info.origin.position.x, info.origin.position.y, quaternion_to_yaw(info.origin.orientation))
        return data, float(info.resolution), origin

    def on_map(self, message):
        from ..planning.goal_heading import grid_world, saved_map_blocked
        data, resolution, origin = self._grid(message)
        self.saved_world = grid_world(data, resolution, origin, blocked=saved_map_blocked)

    def on_local_costmap(self, message):
        self.local_costmap = (message.header.frame_id, *self._grid(message))

    def heading_ready(self):
        return (self.plan_client.server_is_ready() and self.spin_client.server_is_ready()
                and self.saved_world is not None)

    def start_heading_goal(self, now):
        from ..planning.goal_heading import candidate_headings
        x, y, yaw = self.goals[self.index]
        start = self.pose()
        h = self.heading
        candidates = candidate_headings(yaw, h["candidates"]) if h["heading_aware"] else [yaw]
        record = {"index": self.index, "goal": [x, y, yaw], "start_pose": list(start or ()),
                  "sent_time_s": round(now, 3)}
        self.pending = {"phase": "select", "record": record, "candidates": candidates, "results": [],
                        "future": None, "handle": None, "result": None, "deadline": None}
        self.status_pub.publish(json_message({"state": "SELECTING_HEADING", "goal_index": self.index,
                                              "goal": [x, y, yaw]}))
        self.send_candidate(now)

    def send_candidate(self, now):
        p = self.pending
        x, y, _ = self.goals[self.index]
        goal = self.ComputePathToPose.Goal()
        goal.goal = pose_stamped("map", self.node.get_clock().now().to_msg(), x, y,
                                 p["candidates"][len(p["results"])])
        goal.planner_id = str(self.heading.get("planner_id", "GridBased"))
        goal.use_start = False
        p.update(future=self.plan_client.send_goal_async(goal), handle=None, result=None,
                 deadline=now + float(self.heading.get("plan_timeout_s", 10.0)))

    def poll_action(self, now):
        """(status, result) once the pending action ends, else None; an overdue action is cancelled (CANCELED)."""
        p = self.pending
        if p["handle"] is None:
            if not p["future"].done():
                return (CANCELED, None) if now > p["deadline"] else None
            p["handle"] = p["future"].result()
            if p["handle"] is None or not p["handle"].accepted:
                return ABORTED, None
            p["result"] = p["handle"].get_result_async()
        if p["result"].done():
            answer = p["result"].result()
            return answer.status, answer.result
        if now > p["deadline"]:
            p["handle"].cancel_goal_async()
            return CANCELED, None
        return None

    def poll_select(self, now):
        from ..planning.body import wrap
        from ..planning.goal_heading import arrival_direction, choose_arrival, path_length
        found = self.poll_action(now)
        if found is None:
            return
        status, result = found
        p, h = self.pending, self.heading
        xy = None
        if status == SUCCEEDED and result is not None and len(result.path.poses) > 1:
            xy = [(q.pose.position.x, q.pose.position.y) for q in result.path.poses]
        direction = None if xy is None else arrival_direction(xy, float(h.get("arrival_back_m", 0.5)))
        p["results"].append({
            "heading_rad": round(p["candidates"][len(p["results"])], 4), "status": int(status),
            "length_m": None if xy is None else round(path_length(xy), 3),
            "arrival_rad": None if direction is None else round(direction, 4)})
        if len(p["results"]) < len(p["candidates"]):
            return self.send_candidate(now)
        x, y, yaw = self.goals[self.index]
        tolerance = float(h["yaw_tolerance_rad"])
        if h["heading_aware"]:
            heading, reason, shortest = choose_arrival(
                yaw, [(r["heading_rad"], r["length_m"]) for r in p["results"]],
                slack_fraction=float(h["length_slack_fraction"]), slack_m=float(h["length_slack_m"]))
        else:
            direction, shortest = p["results"][0]["arrival_rad"], p["results"][0]["length_m"]
            if direction is None:
                heading, reason = yaw, "NO_PATH_FOR_ANY_CANDIDATE"
            elif abs(wrap(direction - yaw)) <= tolerance:
                heading, reason = yaw, "REQUESTED_HEADING_WITHIN_TOLERANCE"
            else:
                heading, reason = direction, "PATH_DIRECTION_AT_ARRIVAL"
        p["record"]["heading"] = {"mode": "turn_at_goal", "requested_rad": yaw, "arrival_rad": round(heading, 4),
                                  "choice": reason, "shortest_m": shortest, "candidates": p["results"],
                                  "selection_s": round(now - p["record"]["sent_time_s"], 3)}
        self.send_navigation(x, y, heading, p["record"], now)

    def local_world(self):
        from ..planning.goal_heading import costmap_blocked, grid_world
        if self.local_costmap is None:
            return None, None
        frame, data, resolution, origin = self.local_costmap
        return frame, grid_world(data, resolution, origin, blocked=costmap_blocked)

    def start_turn(self, now, nav_status):
        """After arriving: turn in place to the requested heading when the swept area is clear, else finish."""
        from builtin_interfaces.msg import Duration
        from ..planning.goal_heading import decide_turn
        record, h = self.pending["record"], self.heading
        _, _, yaw = self.goals[self.index]
        pose_map = self.pose()
        frame, local = self.local_world()
        found = lookup_pose(self.buffer, frame, "base_link") if frame else None
        checks = []
        if pose_map is not None and self.saved_world is not None:
            checks.append(("saved_map", self.saved_world, pose_map))
        if found is not None and local is not None:
            checks.append(("local_costmap", local, found[0]))
        if pose_map is None:
            record["heading"]["turn"] = {"angle": None, "reason": "NO_POSE"}
            return self.finish_goal(nav_status, now, heading_reached=False)
        decision = decide_turn(checks if len(checks) == 2 else [], pose_map[2], yaw, self.body,
                               margin=float(h["sweep_margin_m"]), tolerance=float(h["yaw_tolerance_rad"]))
        decision["decided_time_s"] = round(now, 3)
        record["heading"]["turn"] = decision
        if decision["angle"] is None:
            return self.finish_goal(nav_status, now, heading_reached=decision["reason"] == "WITHIN_TOLERANCE")
        angle = float(decision["angle"])
        allowance = int(abs(angle) / SPIN_SPEED_FLOOR_RADPS) + 30
        goal = self.Spin.Goal()
        goal.target_yaw = angle
        goal.time_allowance = Duration(sec=allowance)
        self.pending.update(phase="turn", nav_status=int(nav_status), future=self.spin_client.send_goal_async(goal),
                            handle=None, result=None, deadline=now + allowance + 10.0)
        self.status_pub.publish(json_message({"state": "TURNING", "goal_index": self.index,
                                              "target_yaw": round(angle, 4)}))

    def poll_turn(self, now):
        from ..planning.body import wrap
        found = self.poll_action(now)
        if found is None:
            return
        status, _ = found
        turn = self.pending["record"]["heading"]["turn"]
        turn["spin_status"] = int(status)
        turn["spin_s"] = round(now - turn["decided_time_s"], 3)
        pose = self.pose()
        _, _, yaw = self.goals[self.index]
        reached = (status == SUCCEEDED and pose is not None
                   and abs(wrap(yaw - pose[2])) <= float(self.heading["yaw_tolerance_rad"]))
        self.finish_goal(self.pending["nav_status"], now, heading_reached=reached)

    # -- mission ------------------------------------------------------------------
    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def pose(self):
        found = lookup_pose(self.buffer, "map", "base_link")
        return None if found is None else found[0]

    def send_navigation(self, x, y, yaw, record, now):
        start = record.get("start_pose") or None
        distance = math.dist(start[:2], (x, y)) if start else 5.0
        timeout = max(float(self.cfg.get("goal_timeout_min_s", 60.0)),
                      4.0 * distance / max(0.05, float(self.cfg.get("nominal_speed", 0.2))) + 30.0)
        goal = self.NavigateToPose.Goal()
        goal.pose = pose_stamped("map", self.node.get_clock().now().to_msg(), x, y, yaw)
        if "sent_time_s" in record:
            record["navigate_sent_time_s"] = round(now, 3)
        else:
            record["sent_time_s"] = round(now, 3)
        record["timeout_s"] = round(timeout, 1)
        self.pending = {"phase": "navigate", "future": self.nav.send_goal_async(goal), "handle": None,
                        "result": None, "deadline": now + timeout, "record": record}
        self.status_pub.publish(json_message({"state": "NAVIGATING", "goal_index": self.index,
                                              "goal": [x, y, yaw]}))

    def tick(self):
        if self.finished is not None:
            return
        now = self.now()
        if now <= 0:
            return
        if self.start_time is None:
            if not self.nav.server_is_ready() or self.pose() is None:
                return
            if self.heading and not self.heading_ready():
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
            if self.heading:
                return self.start_heading_goal(now)
            x, y, yaw = self.goals[self.index]
            start = self.pose()
            record = {"index": self.index, "goal": [x, y, yaw], "start_pose": list(start or ())}
            return self.send_navigation(x, y, yaw, record, now)
        if self.pending["phase"] == "select":
            return self.poll_select(now)
        if self.pending["phase"] == "turn":
            return self.poll_turn(now)
        p = self.pending
        status = None
        if p["handle"] is None:
            if p["future"].done():
                p["handle"] = p["future"].result()
                if p["handle"] is None or not p["handle"].accepted:
                    status = ABORTED
                else:
                    p["result"] = p["handle"].get_result_async()
            elif now > p["deadline"]:
                status = CANCELED
        if status is None and p["result"] is not None:
            if p["result"].done():
                status = p["result"].result().status
            elif now > p["deadline"]:
                p["handle"].cancel_goal_async()
                p["record"]["timed_out"] = True
                status = CANCELED
        if status is None:
            return
        if self.heading:
            if status == SUCCEEDED:
                return self.start_turn(now, status)
            return self.finish_goal(status, now, heading_reached=False)
        self.finish_goal(status, now)

    def finish_goal(self, status, now, heading_reached=None):
        from ..planning.body import wrap
        end = self.pose()
        record = self.pending["record"]
        record.update(status=int(status), succeeded=status == SUCCEEDED, finished_time_s=round(now, 3),
                      end_pose=list(end or ()))
        if heading_reached is not None:
            heading = record.setdefault("heading", {"mode": "turn_at_goal"})
            heading["reached"] = bool(status == SUCCEEDED and heading_reached)
            heading["final_error_rad"] = None if end is None else round(wrap(record["goal"][2] - end[2]), 4)
            record["succeeded"] = heading["reached"]
        self.records.append(record)
        self.status_pub.publish(json_message({"state": "GOAL_FINISHED", "record": record}))
        self.pending = None
        self.index += 1
        if not record["succeeded"] and self.cfg.get("stop_on_failure", False):
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
