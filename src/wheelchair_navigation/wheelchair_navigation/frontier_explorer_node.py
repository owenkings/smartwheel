#!/usr/bin/env python3
"""Frontier exploration for autonomous RViz 3D mapping.

Reads the SLAM occupancy grid (/rtabmap/grid_map), finds free/unknown frontiers,
clusters and scores them, then drives the chair toward the best safe frontier by
sending Nav2 NavigateToPose goals. It NEVER publishes /cmd_vel itself: speed is
owned by the Nav2 controller and the safety_supervisor (/cmd_vel_nav ->
/cmd_vel_safe). It refuses to send goals unless a map, TF and the action server
are available and safety_state permits motion.
"""
import math

try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped, Point
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray
    from nav2_msgs.action import NavigateToPose
    import tf2_ros
except ImportError:  # allow py_compile / tooling without ROS
    rclpy = None
    Node = object
    ActionClient = OccupancyGrid = PoseStamped = String = Marker = MarkerArray = None
    NavigateToPose = tf2_ros = None

FREE_MAX = 25      # occupancy <= this (and >=0) is free
OCC_MIN = 65       # occupancy >= this is an obstacle
UNKNOWN = -1


def find_frontier_cells(data, w, h):
    """Free cells (4-neighbour) adjacent to unknown space."""
    cells = []
    for j in range(h):
        row = j * w
        for i in range(w):
            v = data[row + i]
            if v < 0 or v > FREE_MAX:
                continue
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ni, nj = i + di, j + dj
                if 0 <= ni < w and 0 <= nj < h and data[nj * w + ni] == UNKNOWN:
                    cells.append((i, j))
                    break
    return cells


def cluster_cells(cells):
    """8-connected clustering of frontier cells."""
    cellset = set(cells)
    clusters = []
    seen = set()
    for c in cells:
        if c in seen:
            continue
        stack = [c]
        seen.add(c)
        comp = []
        while stack:
            ci, cj = stack.pop()
            comp.append((ci, cj))
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    n = (ci + di, cj + dj)
                    if n in cellset and n not in seen:
                        seen.add(n)
                        stack.append(n)
        clusters.append(comp)
    return clusters


def cell_clear(data, w, h, ci, cj, radius_cells):
    """True if no obstacle cell within radius_cells of (ci,cj)."""
    for dj in range(-radius_cells, radius_cells + 1):
        for di in range(-radius_cells, radius_cells + 1):
            ni, nj = ci + di, cj + dj
            if 0 <= ni < w and 0 <= nj < h and data[nj * w + ni] >= OCC_MIN:
                return False
    return True


class FrontierExplorerNode(Node):
    def __init__(self):
        super().__init__("frontier_explorer_node")
        p = self.declare_parameter
        p("map_topic", "/rtabmap/grid_map")
        p("base_frame", "base_link")
        p("global_frame", "map")
        p("auto_start", True)
        p("min_frontier_size", 8)
        p("robot_radius", 0.35)
        p("safety_margin", 0.25)
        p("min_goal_distance", 0.5)
        p("max_goal_distance", 4.0)
        p("goal_timeout_sec", 45.0)
        p("exploration_timeout_sec", 600.0)
        p("stop_on_safety_warning", True)
        p("stop_on_safety_emergency", True)
        p("safety_timeout_sec", 2.0)
        p("period_sec", 2.0)

        g = lambda n: self.get_parameter(n).value
        self.map_topic = g("map_topic")
        self.base_frame = g("base_frame")
        self.global_frame = g("global_frame")
        self.auto_start = bool(g("auto_start"))
        self.min_frontier_size = int(g("min_frontier_size"))
        self.robot_radius = float(g("robot_radius"))
        self.safety_margin = float(g("safety_margin"))
        self.min_goal_distance = float(g("min_goal_distance"))
        self.max_goal_distance = float(g("max_goal_distance"))
        self.goal_timeout_sec = float(g("goal_timeout_sec"))
        self.exploration_timeout_sec = float(g("exploration_timeout_sec"))
        self.stop_on_safety_warning = bool(g("stop_on_safety_warning"))
        self.stop_on_safety_emergency = bool(g("stop_on_safety_emergency"))
        self.safety_timeout_sec = float(g("safety_timeout_sec"))

        self.grid = None
        self.safety_state = "UNKNOWN"
        self.safety_last = None
        self.goal_handle = None
        self.goal_sent_at = None
        self.current_goal_xy = None
        self.started_at = self.now()
        self.blacklist = []  # (x, y) world points that failed

        self.frontier_pub = self.create_publisher(MarkerArray, "/exploration/frontiers", 1)
        self.goal_viz_pub = self.create_publisher(PoseStamped, "/exploration/selected_goal", 1)
        self.status_pub = self.create_publisher(String, "/exploration/status", 1)
        self.estop_pub = self.create_publisher(String, "/emergency_stop_command", 1)
        self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, 1)
        self.create_subscription(String, "/safety_state", self._on_safety, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.create_timer(float(g("period_sec")), self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_map(self, msg):
        self.grid = msg

    def _on_safety(self, msg):
        self.safety_state = (msg.data.split(":", 1)[0] or "UNKNOWN").strip().upper()
        self.safety_last = self.now()

    def _publish_status(self, text):
        self.status_pub.publish(String(data=text))

    def _safety_reason(self):
        """Return (kind, text) when exploration must be blocked, else None.
        Strict whitelist: only explore on a FRESH, explicitly-safe state."""
        if self.safety_last is None or self.now() - self.safety_last > self.safety_timeout_sec:
            return ("block", f"safety stale/no message ({self.safety_state})")
        s = self.safety_state
        if s in ("", "UNKNOWN", "UNINITIALIZED"):
            return ("block", f"safety={s or 'EMPTY'}")
        if "EMERGENCY" in s or s == "SENSOR_FAULT":
            return ("emergency", f"safety={s}")
        allow = {"CLEAR", "OK", "ACTIVE", "IDLE"}
        if not self.stop_on_safety_warning:
            allow |= {"WARNING", "WARN", "SLOWDOWN", "STOP"}
        if s in allow:
            return None
        return ("block", f"safety={s}")

    def _cancel_goal(self):
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
        self.goal_handle = None
        self.goal_sent_at = None

    def tick(self):
        if not self.auto_start:
            self._publish_status("IDLE: autonomous_exploration disabled")
            return
        if self.now() - self.started_at > self.exploration_timeout_sec:
            self._cancel_goal()
            self._publish_status("DONE: exploration timeout reached")
            return

        reason = self._safety_reason()
        if reason is not None:
            kind, text = reason
            self._cancel_goal()
            if kind == "emergency" and self.stop_on_safety_emergency:
                self.estop_pub.publish(String(data="stop"))
            self._publish_status("BLOCKED: " + text)
            return
        if self.grid is None:
            self._publish_status("BLOCKED: no map on " + self.map_topic)
            return
        if not self.nav_client.server_is_ready():
            self.nav_client.wait_for_server(timeout_sec=0.0)
            self._publish_status("BLOCKED: /navigate_to_pose action server not available")
            return
        robot = self._robot_xy()
        if robot is None:
            self._publish_status("BLOCKED: no TF %s->%s" % (self.global_frame, self.base_frame))
            return

        # goal timeout while pursuing
        if self.goal_handle is not None:
            if self.goal_sent_at is not None and self.now() - self.goal_sent_at > self.goal_timeout_sec:
                self.get_logger().warn("frontier goal timed out; blacklisting and reselecting")
                self._blacklist_current()
                self._cancel_goal()
            else:
                self._publish_status("EXPLORING: pursuing goal")
                return

        goal, frontiers = self._select_goal(robot)
        self._publish_frontiers(frontiers, goal)
        if goal is None:
            self._publish_status("DONE: no reachable frontier (explored or blacklisted)")
            return
        self._send_goal(goal)

    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.x, t.y, yaw)

    def _select_goal(self, robot):
        grid = self.grid
        w, h = grid.info.width, grid.info.height
        res = grid.info.resolution
        ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
        data = grid.data
        cells = find_frontier_cells(data, w, h)
        radius_cells = max(1, int(math.ceil((self.robot_radius + self.safety_margin) / max(res, 1e-3))))
        rx, ry, ryaw = robot
        candidates = []  # (cost, x, y, size)
        centroids = []
        for comp in cluster_cells(cells):
            if len(comp) < self.min_frontier_size:
                continue
            ci = sum(c[0] for c in comp) / len(comp)
            cj = sum(c[1] for c in comp) / len(comp)
            x = ox + (ci + 0.5) * res
            y = oy + (cj + 0.5) * res
            centroids.append((x, y))
            dist = math.hypot(x - rx, y - ry)
            if dist < self.min_goal_distance or dist > self.max_goal_distance:
                continue
            if self._blacklisted(x, y):
                continue
            if not cell_clear(data, w, h, int(round(ci)), int(round(cj)), radius_cells):
                continue
            heading = abs(self._wrap(math.atan2(y - ry, x - rx) - ryaw))
            cost = 1.0 * dist + 0.6 * heading - 0.02 * len(comp)
            candidates.append((cost, x, y, len(comp)))
        if not candidates:
            return None, centroids
        candidates.sort(key=lambda c: c[0])
        best = candidates[0]
        return (best[1], best[2], math.atan2(best[2] - ry, best[1] - rx)), centroids

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def _blacklisted(self, x, y):
        return any(math.hypot(x - bx, y - by) < 0.5 for bx, by in self.blacklist)

    def _blacklist_current(self):
        if self.current_goal_xy is not None:
            self.blacklist.append(self.current_goal_xy)

    def _send_goal(self, goal):
        x, y, yaw = goal
        self.current_goal_xy = (x, y)
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_viz_pub.publish(pose)
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose
        self.goal_sent_at = self.now()
        self._publish_status(f"EXPLORING: goal ({x:.2f}, {y:.2f})")
        future = self.nav_client.send_goal_async(nav_goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn("NavigateToPose goal rejected; blacklisting")
            self._blacklist_current()
            self.goal_handle = None
            self.goal_sent_at = None
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        status = getattr(future.result(), "status", 0)
        if status != 4:  # 4 == SUCCEEDED
            self._blacklist_current()
        self.goal_handle = None
        self.goal_sent_at = None

    def _publish_frontiers(self, centroids, goal):
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = self.global_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "frontiers"
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color.g = 1.0
        m.color.a = 0.9
        for x, y in centroids:
            m.points.append(Point(x=x, y=y, z=0.05))
        arr.markers.append(m)
        self.frontier_pub.publish(arr)


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = FrontierExplorerNode()
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
