"""ROS node of the people reaction (``--people-reaction``; explicit option, default off).

Wires ``people.py`` into Nav2 without sending any velocity:

- reads window C's ``/obstacles/tracks`` (``wc_obstacle_tracks/1``; in simulation the simulator's truth tracks,
  ``--tracks truth``), the chair's pose (TF odom -> base_link) and velocity (``/odom``);
- publishes the speed limit on ``/speed_limit`` (nav2_msgs/SpeedLimit, absolute m/s; 0 means no limit), which
  the controller server applies;
- with ``--people-keepout`` (``keepout``; off by default) publishes a KeepoutFilter mask per costmap with its
  CostmapFilterInfo (type keepout, base 0, multiplier 1): the local mask in the odom frame around the chair,
  the global mask in the map frame over the whole map. In the ZL1107 corridor a zone around a person standing
  in the lane closes the way, Nav2 fails the plan and gives the goal up within about 10 s (simulation, 11-09),
  so the zones are an explicit extra.
  Only people that have stood still for 3 s get a zone, at their current position with the standing clearance
  (``masked_people``); walkers get the speed limit only.

Humble's KeepoutFilter only writes inside each costmap update's window and widens that window on a new mask to
the cells (0, 0)..(mask width, mask height) of the costmap; old keepout cells outside the window stay. So every
mask has at least the costmap's cell count (local: ``local_cells``, more than the 6 m window; global: the map's
own grid), and a fresh mask goes out while people are near and once more after the last one has gone.

The collision monitor (window C) and the gate (window D) are unchanged and stay the last stops. If the tracks
stop arriving (older than ``stale_after_s``) the reaction has no people and lifts its limit; the obstacles are
still in the costmaps and the monitor as before. Decisions go to ``/wc_nav/people_status`` and ``people.jsonl``.
"""
from __future__ import annotations

import argparse
import array
import json
import math
from pathlib import Path

LOCAL_MASK, LOCAL_INFO = "/wc_nav/people/local_mask", "/wc_nav/people/local_filter_info"
GLOBAL_MASK, GLOBAL_INFO = "/wc_nav/people/global_mask", "/wc_nav/people/global_filter_info"
SPEED_LIMIT_TOPIC = "/speed_limit"
STATUS_TOPIC = "/wc_nav/people_status"
HEADING_FREE_PLANNERS = ("navfn", "smac_2d", "theta_star")     # check only the centre cell of the footprint


def nav2_footprint(outline, radius_m, padding):
    """The padded footprint Nav2 uses: the outline (or, without one, Nav2's 16-point circle of ``radius_m``)
    with each vertex moved by ``padding`` along the signs of its coordinates (nav2_costmap_2d::padFootprint)."""
    if outline:
        points = [tuple(float(v) for v in p) for p in outline]
    else:
        points = [(radius_m * math.cos(2 * math.pi * k / 16), radius_m * math.sin(2 * math.pi * k / 16))
                  for k in range(16)]

    def sign(v):
        return 0.0 if v == 0.0 else math.copysign(1.0, v)
    return [(x + sign(x) * padding, y + sign(y) * padding) for x, y in points]


def footprint_radii(points):
    """(inscribed, circumscribed) radius as nav2_costmap_2d::calculateMinAndMaxDistances computes them."""
    def segment_distance(a, b):
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        t = 0.0 if length2 == 0.0 else max(0.0, min(1.0, -(ax * dx + ay * dy) / length2))
        return math.hypot(ax + t * dx, ay + t * dy)
    low, high = math.inf, 0.0
    for k, a in enumerate(points):
        b = points[(k + 1) % len(points)]
        vertex, edge = math.hypot(*a), segment_distance(a, b)
        low, high = min(low, vertex, edge), max(high, vertex, edge)
    return low, high


def local_origin(chair_xy, cells, resolution):
    """Origin of a ``cells`` x ``cells`` mask centred on the chair, on the resolution grid."""
    half = cells * resolution / 2.0
    return tuple(math.floor((float(v) - half) / resolution) * resolution for v in chair_xy)


def speed_limit_value(limit_mps):
    """SpeedLimit.speed_limit for a reaction limit: 0.0 (Nav2's 'no limit') when there is none."""
    return 0.0 if limit_mps is None else float(limit_mps)


SETTLE_S = 3.0          # a person gets a keepout zone after standing this long


def masked_people(people, standing_since, now, settle_s=SETTLE_S):
    """People that get a keepout zone: those that have stood still for ``settle_s``; updates ``standing_since``.

    Walking people get none, and neither do walkers pausing for a moment: a zone over a walker's predicted
    positions closes a corridor for as long as they walk across it, and a zone at a pausing walker's side of the
    corridor comes and goes with each pause (simulated crossing pedestrian, 11-09). Nav2 then fails the plan
    and gives the goal up after its recovery rounds instead of waiting. Walkers are handled by the speed limit
    (slow down, yield) and stay ordinary obstacles in both costmaps.
    """
    ids = set()
    for p in people:
        ids.add(p.id)
        if p.moving:
            standing_since.pop(p.id, None)
        else:
            standing_since.setdefault(p.id, float(now))
    for gone in set(standing_since) - ids:
        del standing_since[gone]
    return [p for p in people if not p.moving and float(now) - standing_since[p.id] >= settle_s]


def node_config(nav2, profile, body, planning, output_dir, *, use_sim_time=True, update_hz=5.0, local_cells=140):
    """Config of the people node from the generated Nav2 parameters (costmap padding and inflation)."""
    def costmap(name):
        p = nav2[name][name]["ros__parameters"]
        outline = body.outline if "footprint" in p else ()       # without an outline the costmap's robot_radius
        inscribed, circumscribed = footprint_radii(nav2_footprint(outline, float(p.get("robot_radius", 0.0)),
                                                                  float(p.get("footprint_padding", 0.0))))
        return {"inscribed_m": round(inscribed, 4), "circumscribed_m": round(circumscribed, 4),
                "cost_scaling_factor": float(p["inflation_layer"]["cost_scaling_factor"]),
                "resolution": float(p["resolution"])}
    local = costmap("local_costmap")
    window_cells = int(round(float(nav2["local_costmap"]["local_costmap"]["ros__parameters"]["width"])
                             / local["resolution"]))
    if local_cells < window_cells:
        raise ValueError("the local mask must have at least the local costmap's %d cells" % window_cells)
    local.update(cells=int(local_cells), center_point=False, mask_topic=LOCAL_MASK, info_topic=LOCAL_INFO)
    glob = costmap("global_costmap")
    glob.update(center_point=planning.planner in HEADING_FREE_PLANNERS, mask_topic=GLOBAL_MASK, info_topic=GLOBAL_INFO)
    return {"use_sim_time": bool(use_sim_time), "update_hz": float(update_hz), "body": body.as_config(),
            "max_linear": float(profile.max_linear), "keepout": bool(planning.people_keepout),
            "local": local, "global": glob,
            "global_range_m": 10.0, "global_period_s": 0.5,
            "log_jsonl": str(Path(output_dir) / "people.jsonl"),
            "snapshots_npz": str(Path(output_dir) / "people_costmaps.npz")}


class PeopleNode:
    def __init__(self, node, config):
        from nav2_msgs.msg import CostmapFilterInfo, SpeedLimit
        from nav_msgs.msg import OccupancyGrid, Odometry
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformListener
        from ..autonomy.ros_util import latched_qos
        from ..autonomy.sim_node import TRACKS_TOPIC
        from .body import Body
        from .people import PeopleConfig, Reaction
        self.node = node
        self.cfg = PeopleConfig(**config.get("people", {}))
        self.body = Body.from_config(config["body"])
        self.local, self.glob = config["local"], config["global"]
        self.keepout = bool(config.get("keepout", False))
        self.global_range_m = float(config.get("global_range_m", 10.0))
        self.global_period_s = float(config.get("global_period_s", 0.5))
        self.reaction = Reaction(self.cfg)
        self.OccupancyGrid, self.SpeedLimit = OccupancyGrid, SpeedLimit
        self.message = None
        self.velocity = (0.0, 0.0)
        self.map_info = None
        self.people_before = {"local": False}
        self.global_key = ()
        self.standing_since = {}
        self.last_limit = ("unset",)
        self.last_limit_time = -math.inf
        self.last_global_time = -math.inf
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.limit_pub = node.create_publisher(SpeedLimit, SPEED_LIMIT_TOPIC, 10)
        self.status_pub = node.create_publisher(String, STATUS_TOPIC, latched_qos())
        self.mask_pub = {"local": node.create_publisher(OccupancyGrid, self.local["mask_topic"], latched_qos()),
                         "global": node.create_publisher(OccupancyGrid, self.glob["mask_topic"], latched_qos())}
        self.info_pub = {}
        for name, side in (("local", self.local), ("global", self.glob)) if self.keepout else ():
            info = CostmapFilterInfo()
            info.type = 0                                   # keepout
            info.filter_mask_topic = side["mask_topic"]
            info.base, info.multiplier = 0.0, 1.0           # anything else only logs errors (window C, 11-08)
            self.info_pub[name] = node.create_publisher(CostmapFilterInfo, side["info_topic"], latched_qos())
            self.info_pub[name].publish(info)
        node.create_subscription(String, TRACKS_TOPIC, self.on_tracks, 10)
        node.create_subscription(Odometry, "/odom", self.on_odom, 20)
        node.create_subscription(OccupancyGrid, "/map", self.on_map, latched_qos())
        self.log = open(config["log_jsonl"], "x", encoding="utf-8") if config.get("log_jsonl") else None
        # Costmap snapshots (validation: keepout cells left behind), once per second: the local window and a
        # crop of the global costmap around the chair.
        # local_origin rows: [x, y] of the odom-frame window plus map -> odom [x, y, yaw]; global: [x, y] in map.
        self.snapshot_path = config.get("snapshots_npz")
        self.snapshots = {"local": [], "global": []}
        self.last_snapshot = {"local": -math.inf, "global": -math.inf}
        if self.snapshot_path:
            for name in ("local", "global"):
                node.create_subscription(OccupancyGrid, "/%s_costmap/costmap" % name,
                                         lambda m, n=name: self.on_costmap(n, m), latched_qos())
        node.create_timer(1.0 / float(config.get("update_hz", 5.0)), self.update)

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def on_tracks(self, message):
        from ..perception.tracks import check_message
        try:
            self.message = check_message(json.loads(message.data))
        except ValueError as exc:
            self.node.get_logger().warn("ignoring a tracks message: %s" % exc)

    def on_odom(self, message):
        self.velocity = (float(message.twist.twist.linear.x), float(message.twist.twist.angular.z))

    def on_map(self, message):
        info = message.info
        self.map_info = {"width": int(info.width), "height": int(info.height), "resolution": float(info.resolution),
                         "origin": (float(info.origin.position.x), float(info.origin.position.y))}

    def on_costmap(self, name, message):
        import numpy as np
        now = self.now()
        if now - self.last_snapshot[name] < 1.0:
            return
        info = message.info
        grid = np.frombuffer(message.data, dtype=np.int8).reshape(int(info.height), int(info.width))
        res = float(info.resolution)
        origin = [float(info.origin.position.x), float(info.origin.position.y)]
        if name == "global":
            from ..autonomy.ros_util import lookup_pose
            found = lookup_pose(self.buffer, "map", "base_link")
            if found is None:
                return
            half = 100                                      # 10 m x 10 m around the chair
            col = int((found[0][0] - origin[0]) / res) - half
            row = int((found[0][1] - origin[1]) / res) - half
            crop = np.full((2 * half, 2 * half), -1, dtype=np.int8)
            r0, c0 = max(0, row), max(0, col)
            r1, c1 = min(grid.shape[0], row + 2 * half), min(grid.shape[1], col + 2 * half)
            if r0 < r1 and c0 < c1:
                crop[r0 - row:r1 - row, c0 - col:c1 - col] = grid[r0:r1, c0:c1]
            grid, origin = crop, [origin[0] + col * res, origin[1] + row * res]
        else:                                               # local window (odom frame): keep map -> odom with it
            from ..autonomy.ros_util import lookup_pose
            found = lookup_pose(self.buffer, "map", "odom")
            if found is None:
                return
            origin = origin + list(found[0])
        self.snapshots[name].append((now, origin, res, grid.copy()))
        self.last_snapshot[name] = now

    def _grid(self, frame, data, resolution, origin, stamp):
        grid = self.OccupancyGrid()
        grid.header.frame_id = frame
        grid.header.stamp = stamp
        grid.info.resolution = float(resolution)
        grid.info.height, grid.info.width = (int(v) for v in data.shape)
        grid.info.origin.position.x, grid.info.origin.position.y = float(origin[0]), float(origin[1])
        grid.info.origin.orientation.w = 1.0
        grid.data = array.array("b", data.astype("int8").ravel().tobytes())    # no per-element checks
        return grid

    def _mask(self, side, people, shape, resolution, origin, extra):
        from .people import keepout_mask
        return keepout_mask(shape, resolution, origin, people, self.cfg, inscribed_m=side["inscribed_m"],
                            circumscribed_m=side["circumscribed_m"], cost_scaling_factor=side["cost_scaling_factor"],
                            center_point=side["center_point"], extra_m=extra)

    def update(self):
        from ..autonomy.ros_util import json_message, lookup_pose
        from .people import lag_margin, people_from_message, transform_people
        now = self.now()
        if now <= 0.0:
            return                                      # no simulated clock yet
        found = lookup_pose(self.buffer, "odom", "base_link")
        if found is None:
            return
        chair = found[0]
        if self.message is None:
            people, age = [], None
        else:
            people, age = people_from_message(self.message, now, self.cfg)
        decision = self.reaction.update(now, people, chair, self.velocity, self.body)
        stamp = self.node.get_clock().now().to_msg()
        limit = speed_limit_value(decision["limit_mps"])
        if (limit,) != self.last_limit or now - self.last_limit_time >= 1.0:
            message = self.SpeedLimit()
            message.header.stamp = stamp
            message.percentage = False
            message.speed_limit = limit
            self.limit_pub.publish(message)
            self.last_limit, self.last_limit_time = (limit,), now
        extra = lag_margin(self.velocity, self.cfg)
        lethal = {}
        standing = masked_people(people, self.standing_since, now) if self.keepout else []
        if standing or self.people_before["local"]:
            res, cells = self.local["resolution"], self.local["cells"]
            origin = local_origin(chair[:2], cells, res)
            data = self._mask(self.local, standing, (cells, cells), res, origin, extra)
            self.mask_pub["local"].publish(self._grid("odom", data, res, origin, stamp))
            lethal["local"] = int((data == 100).sum())
        self.people_before["local"] = bool(standing)
        # Global mask: standing people within global_range_m, in the map frame. Every new mask makes Humble refresh
        # the whole global costmap, so it goes out only when they change (by a cell) and at most every
        # global_period_s.
        correction = lookup_pose(self.buffer, "map", "odom")
        if self.keepout and self.map_info is not None and correction is not None:
            info = self.map_info
            near = transform_people([p for p in standing if math.hypot(p.x - chair[0], p.y - chair[1])
                                     <= self.global_range_m], correction[0])
            key = tuple(sorted((p.id, round(p.x / info["resolution"]), round(p.y / info["resolution"]))
                               for p in near))
            if key != self.global_key and now - self.last_global_time >= self.global_period_s:
                data = self._mask(self.glob, near, (info["height"], info["width"]), info["resolution"],
                                  info["origin"], extra)
                self.mask_pub["global"].publish(self._grid("map", data, info["resolution"], info["origin"], stamp))
                self.last_global_time, self.global_key = now, key
                lethal["global"] = int((data == 100).sum())
        deciding = decision.get("deciding") or {}
        status = {"time_s": round(now, 3), "state": decision["state"], "limit_mps": decision["limit_mps"],
                  "people": decision["people"], "tracks_age_s": None if age is None else round(age, 3),
                  "deciding_id": deciding.get("id"),
                  "deciding_gap_m": None if not deciding else round(min(deciding["gap_now_m"], deciding["gap_yield_m"]), 3),
                  "yield_s": decision["yield_s"], "treated_as_standing": decision["treated_as_standing"],
                  "chair": [round(v, 3) for v in chair], "chair_v": round(self.velocity[0], 3),
                  "mask_lethal_cells": lethal}
        self.status_pub.publish(json_message(status))
        if self.log is not None:
            self.log.write(json.dumps(status, allow_nan=False) + "\n")
            self.log.flush()

    def close(self):
        if self.log is not None:
            self.log.close()
        if self.snapshot_path and any(self.snapshots.values()):
            import numpy as np
            arrays = {}
            for name, items in self.snapshots.items():
                if not items:
                    continue
                arrays[name + "_time_s"] = np.array([t for t, _, _, _ in items])
                arrays[name + "_origin"] = np.array([o for _, o, _, _ in items])
                arrays[name + "_resolution"] = np.array([r for _, _, r, _ in items])
                arrays[name + "_grid"] = np.stack([g for _, _, _, g in items])
            np.savez(self.snapshot_path, **arrays)          # uncompressed: done within the supervisor's 4 s


def main(argv=None):
    from ..autonomy.ros_util import load_config, require_isolated_domain
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    args, ros_args = parser.parse_known_args(argv)
    require_isolated_domain()
    config = load_config(args.config)
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import Parameter
    rclpy.init(args=ros_args)
    node = rclpy.create_node("wc_people_reaction", parameter_overrides=[
        Parameter("use_sim_time", value=bool(config.get("use_sim_time", True)))])
    people = PeopleNode(node, config)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise                   # a real failure while running; errors after the shutdown signal are expected
    finally:
        people.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
