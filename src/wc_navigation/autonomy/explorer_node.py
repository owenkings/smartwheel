"""Autonomous frontier exploration through the Nav2 NavigateToPose action.

State machine: wait for map/TF/actions, optional initial 360 degree look-around
(the forward sensor covers about 115 degrees), then repeatedly select the best
reachable frontier, navigate, and blacklist failures or frontiers that persist
after a visit. When no frontier remains it optionally returns to the start and
saves the final map. Motion still passes the collision monitor and safety gate.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from ..geometry import Grid
from .frontier import select_goal, trinary
from .maps import read_back, save_trinary_map
from .ros_util import (json_message, latched_qos, load_config, lookup_pose, pose_stamped,
                       quaternion_to_yaw, require_isolated_domain)

SUCCEEDED, CANCELED, ABORTED = 4, 5, 6


class Explorer:
    def __init__(self, node, config):
        from nav2_msgs.action import NavigateToPose, Spin
        from nav_msgs.msg import OccupancyGrid
        from rclpy.action import ActionClient
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformListener
        self.node = node
        self.cfg = config
        self.output = Path(config["output_dir"])
        self.NavigateToPose, self.Spin = NavigateToPose, Spin
        self.nav = ActionClient(node, NavigateToPose, "/navigate_to_pose")
        self.spin = ActionClient(node, Spin, "/spin")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.status_pub = node.create_publisher(String, "/wc_nav/explorer_status", latched_qos())
        node.create_subscription(OccupancyGrid, "/map", self.on_map, latched_qos())
        self.map = None
        self.map_updates = 0
        self.state = "WAIT_READY"
        self.start_time = None
        self.home = None
        self.blacklist = []
        self.goals = []
        self.reached = []
        self.consecutive_failures = 0
        self.start_failures = 0
        self.end_reason = None
        self.pending = None   # dict(kind, future, handle, result_future, sent, deadline, record)
        self.finished = None
        self.last_report = {}
        node.create_timer(0.5, self.tick)

    # -- helpers -----------------------------------------------------------------
    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def publish(self, **extra):
        self.status_pub.publish(json_message({"state": self.state, "time_s": round(self.now(), 3),
                                              "goals_sent": len(self.goals), "blacklist": len(self.blacklist),
                                              **extra}))

    def on_map(self, message):
        info = message.info
        data = np.asarray(message.data, dtype=np.int16).reshape(info.height, info.width)
        origin = (info.origin.position.x, info.origin.position.y, quaternion_to_yaw(info.origin.orientation))
        self.map = (trinary(data), float(info.resolution), origin)
        self.map_updates += 1

    def robot_pose(self):
        found = lookup_pose(self.buffer, "map", "base_link")
        return None if found is None else found[0]

    def grid(self):
        data, resolution, origin = self.map
        return Grid(data, resolution, origin)

    # -- actions -----------------------------------------------------------------
    def send_navigation(self, x, y, yaw, kind, record):
        goal = self.NavigateToPose.Goal()
        goal.pose = pose_stamped("map", self.node.get_clock().now().to_msg(), x, y, yaw)
        timeout = max(float(self.cfg.get("goal_timeout_min_s", 60.0)),
                      3.0 * record.get("path_distance_m", 5.0) / max(0.05, float(self.cfg.get("nominal_speed", 0.2))) + 30.0)
        record.update(goal_xy=[x, y], goal_yaw=yaw, sent_time_s=round(self.now(), 3), timeout_s=round(timeout, 1))
        self.pending = {"kind": kind, "future": self.nav.send_goal_async(goal), "handle": None,
                        "result_future": None, "deadline": self.now() + timeout, "record": record}

    def send_spin(self, angle):
        from builtin_interfaces.msg import Duration
        goal = self.Spin.Goal()
        goal.target_yaw = float(angle)
        goal.time_allowance = Duration(sec=int(abs(angle) / 0.15) + 30)
        record = {"kind": "spin", "target_yaw": angle, "sent_time_s": round(self.now(), 3)}
        self.pending = {"kind": "spin", "future": self.spin.send_goal_async(goal), "handle": None,
                        "result_future": None, "deadline": self.now() + int(abs(angle) / 0.15) + 40, "record": record}

    def poll_pending(self):
        """Return the final status code when the pending action finishes, else None."""
        p = self.pending
        if p["handle"] is None:
            if not p["future"].done():
                return CANCELED if self.now() > p["deadline"] else None
            p["handle"] = p["future"].result()
            if p["handle"] is None or not p["handle"].accepted:
                return ABORTED
            p["result_future"] = p["handle"].get_result_async()
        if p["result_future"].done():
            return p["result_future"].result().status
        if self.now() > p["deadline"]:
            p["handle"].cancel_goal_async()
            p["record"]["timed_out"] = True
            return CANCELED
        return None

    # -- state machine -------------------------------------------------------------
    def tick(self):
        if self.finished is not None:
            return
        now = self.now()
        if now <= 0:
            return
        if self.start_time is not None and now - self.start_time > float(self.cfg.get("max_duration_s", 900.0)):
            if self.pending and self.pending["handle"] is not None:
                self.pending["handle"].cancel_goal_async()
            return self.finish("STOPPED_DURATION_LIMIT")
        if self.state == "WAIT_READY":
            pose = self.robot_pose()
            if self.map is None or pose is None or not self.nav.server_is_ready() or not self.spin.server_is_ready():
                return self.publish(ready={"map": self.map is not None, "tf": pose is not None,
                                           "navigate": self.nav.server_is_ready(), "spin": self.spin.server_is_ready()})
            self.start_time, self.home = now, pose
            if self.cfg.get("initial_spin", True):
                self.state = "INITIAL_SPIN"
                self.send_spin(2.0 * math.pi)
            else:
                self.state = "SELECT"
            return self.publish(home=list(pose))
        if self.state in ("INITIAL_SPIN", "LOOK_AROUND", "NAVIGATING", "RETURN_HOME"):
            status = self.poll_pending()
            if status is None:
                return
            record = self.pending["record"]
            record.update(status=int(status), finished_time_s=round(now, 3),
                          succeeded=status == SUCCEEDED, end_pose=list(self.robot_pose() or ()))
            self.pending = None
            if self.state in ("INITIAL_SPIN", "LOOK_AROUND"):
                self.goals.append(record)
                self.state = "SELECT"
            elif self.state == "RETURN_HOME":
                self.goals.append(record)
                reason = self.end_reason or "COMPLETE"
                return self.finish(reason if status == SUCCEEDED else reason + "_RETURN_HOME_FAILED")
            else:
                self.goals.append(record)
                if status == SUCCEEDED:
                    self.consecutive_failures = 0
                    self.reached.append((tuple(record["goal_xy"]), tuple(record["frontier_centroid_xy"])))
                else:
                    self.consecutive_failures += 1
                    self.blacklist.append((*record["goal_xy"], 0.6))
                    self.blacklist.append((*record["frontier_centroid_xy"], 0.8))
                    if self.consecutive_failures >= int(self.cfg.get("consecutive_failure_limit", 5)):
                        # Keep the map and go home; the status says exploration is incomplete.
                        return self.begin_return("INCOMPLETE_REPEATED_GOAL_FAILURES")
                self.state = "SELECT"
            return self.publish(last=record)
        if self.state == "SELECT":
            if len([g for g in self.goals if g.get("kind") == "frontier"]) >= int(self.cfg.get("max_goals", 40)):
                return self.begin_return("STOPPED_GOAL_LIMIT")
            pose = self.robot_pose()
            if pose is None or self.map is None:
                return self.publish(waiting="pose_or_map")
            for _ in range(6):
                candidate, report = select_goal(
                    self.grid(), pose[:2], radius_m=self.cfg["radius_m"], margin_m=self.cfg["margin_m"],
                    min_frontier_length_m=self.cfg.get("min_frontier_length_m", 0.5),
                    max_goal_offset_m=self.cfg.get("max_goal_offset_m", 3.0),
                    gain_weight=self.cfg.get("gain_weight", 1.0), blacklist=self.blacklist)
                self.last_report = report
                if candidate is None:
                    break
                persisted = any(math.dist(candidate.goal_xy, g) < 0.5 and math.dist(candidate.frontier_centroid_xy, c) < 1.0
                                for g, c in self.reached)
                if not persisted:
                    break
                # Visited before and the frontier is still there: not resolvable from here.
                self.blacklist.append((*candidate.frontier_centroid_xy, 1.0))
                candidate = None
            if candidate is None and (report.get("robot_outside_map") or report.get("robot_cell_blocked")):
                # The map does not yet contain a usable start; this is not completion.
                self.start_failures += 1
                if self.start_failures > int(self.cfg.get("look_around_retries", 2)):
                    return self.finish("FAILED_START_NOT_IN_MAP")
                self.state = "LOOK_AROUND"
                self.send_spin(2.0 * math.pi)
                return self.publish(look_around=self.start_failures, report=report)
            self.start_failures = 0
            if candidate is None:
                # No reachable, non-blacklisted frontier is left. Remaining
                # frontiers are listed in the result rather than hidden.
                return self.begin_return("COMPLETE", report)
            self.state = "NAVIGATING"
            self.send_navigation(candidate.goal_xy[0], candidate.goal_xy[1], candidate.goal_yaw, "frontier",
                                 {"kind": "frontier", "frontier_index": candidate.frontier_index,
                                  "frontier_centroid_xy": list(candidate.frontier_centroid_xy),
                                  "gain_m": candidate.gain_m, "path_distance_m": candidate.path_distance_m,
                                  "score": candidate.score, "frontier_report": report,
                                  "robot_pose": list(pose)})
            return self.publish(selected=self.pending["record"]["goal_xy"])

    def begin_return(self, reason, report=None):
        self.end_reason = reason
        if self.cfg.get("return_home", True) and self.home is not None:
            # Arrive facing the approach direction. Requiring the start heading
            # added a final turn in place; in run 20261105T082134Z the robot
            # stopped 0.28 m from home and turned back and forth for 50 s
            # until the goal timed out. Position is what "home" means here.
            pose = self.robot_pose()
            yaw, policy = self.home[2], "START_HEADING"
            if pose is not None and math.dist(pose[:2], self.home[:2]) > 0.3:
                yaw, policy = math.atan2(self.home[1] - pose[1], self.home[0] - pose[0]), "APPROACH_DIRECTION"
            self.state = "RETURN_HOME"
            self.send_navigation(self.home[0], self.home[1], yaw, "return_home",
                                 {"kind": "return_home", "end_reason": reason, "heading_policy": policy,
                                  "frontier_report": report or self.last_report})
            return self.publish(returning=True, end_reason=reason)
        return self.finish(reason)

    def finish(self, reason):
        self.state = "FINISHED"
        result = {"schema_version": 1, "status": reason, "home_pose_map": list(self.home) if self.home else None,
                  "goals": self.goals, "blacklist": [list(b) for b in self.blacklist],
                  "map_updates": self.map_updates, "last_frontier_report": self.last_report,
                  "elapsed_sim_s": None if self.start_time is None else round(self.now() - self.start_time, 3),
                  "hardware_control": False, "physical_navigation_validated": False}
        if self.map is not None:
            data, resolution, origin = self.map
            try:
                spec, pgm = save_trinary_map(self.output, "map", data, resolution, origin)
                _, loaded, _ = read_back(spec)
                result["map"] = {"yaml": str(spec), "pgm": str(pgm), "readback_identical": bool(np.array_equal(loaded, data)),
                                 "width": int(data.shape[1]), "height": int(data.shape[0]),
                                 "resolution_m": resolution, "origin": list(origin),
                                 "free_cells": int((data == 0).sum()), "occupied_cells": int((data == 100).sum()),
                                 "unknown_cells": int((data < 0).sum())}
            except Exception as exc:
                result["map_error"] = type(exc).__name__ + ": " + str(exc)
        with (self.output / "explorer_result.json").open("x", encoding="utf-8") as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        self.finished = reason
        self.publish(finished=reason)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    args, ros_args = parser.parse_known_args(argv)
    require_isolated_domain()
    config = load_config(args.config)
    import rclpy
    from rclpy.parameter import Parameter
    rclpy.init(args=ros_args)
    node = rclpy.create_node("wc_explorer", parameter_overrides=[Parameter("use_sim_time", value=True)])
    explorer = Explorer(node, config)
    from rclpy.executors import ExternalShutdownException
    try:
        while rclpy.ok() and explorer.finished is None:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    status = explorer.finished
    if status == "COMPLETE":
        return 0
    return 2 if status and status.startswith(("STOPPED", "COMPLETE_", "INCOMPLETE")) else 1


if __name__ == "__main__":
    raise SystemExit(main())
