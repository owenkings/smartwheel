"""Supervise live-chain runs in an isolated localhost domain: rosbag tests and the (prepared) vehicle shadow.

``inputs`` checks the live input chain alone: ``ros2 bag play --clock`` of the
raw sensor topics of a recording, the live inputs node and the recorder; the
result is compared with the frozen frontend of the same recording.

``shadow`` runs the whole framework as on the vehicle: live inputs, map_server
+ AMCL on a saved map from an operator start pose, the Nav2 servers, the safety
gate, an operator goal sequence and the recorder. The gated command goes
nowhere. With ``--source bag`` the sensor topics come from a recording on
recorded time; ``--source vehicle`` (wall time, sensor topics relayed one way
from the vehicle's sensor domain) is prepared but refuses to start without the
explicit approval described in docs/operations/navigation_live_shadow.md.

``shadow --closed-loop`` (D.md 2026-11-10 16:45) drives instead: window A's base bridge
(``wc_motion.auto_bridge``) starts after the Nav2 readback and before the gate, the gate runs its closed-loop
configuration (base in automatic control, the bridge as the only enable and estop source, monotonic ages), this
supervisor publishes no enable at all, the goals wait for the operator's "start automatic", and the bridge is
stopped first.

Every process runs in WC_AUTONOMY_ROS_DOMAIN (96-101, localhost only) under the
checkout's domain lock; only processes started here are stopped. The first
Ctrl-C (or SIGTERM/SIGHUP) starts the shutdown; later ones cannot interrupt it
(``InterruptGuard``).
"""
from __future__ import annotations

import contextlib
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from ..autonomy import nav2_config
from ..autonomy.bringup import EXECUTABLES, PACKAGES, RunError, Supervisor, _parameter_value, _same
from ..autonomy.ros_util import json_message, latched_qos, lookup_pose, require_isolated_domain, simulation_domain
from ..nav2_planner import _stop_owned
from .config import source_topics
from .front_check import TOPIC as FRONT_TOPIC

VEHICLE_APPROVAL_ENV = "WC_LIVE_SHADOW_VEHICLE_APPROVAL"
INPUT_ROLES = ("scan", "odometry")          # separate processes: lidar work never delays wheel/IMU callbacks
OBSTACLES_STATUS = "/wc_live/inputs_status/obstacles"   # perception.live_obstacles.STATUS_TOPIC (window C)
INTERRUPTED = "OPERATOR_INTERRUPT"
# Closed loop (D.md 2026-11-10 16:45): window A's base bridge, stopped first so the wheel owner ends automatic
# control at once; its status topic (gate_node.BASE_STATUS_TOPIC).
BRIDGE = "base_bridge"
BASE_STATUS_TOPIC = "/wc_nav/base_status"
GATE_STATUS_TOPIC = "/wc_nav/gate_status"
GOAL_STATUS_TOPIC = "/wc_live/goal_status"


class StartCheckStop(Exception):
    """The start check did not let the run go on (INITIAL_POSE_MISMATCH and the like)."""

    def __init__(self, status):
        super().__init__(status)
        self.status = status


class BridgeExited(RunError):
    """The closed loop's base bridge exited: the run ends; its reason goes to result.json (closed_loop.bridge_exit)."""

    def __init__(self, code, log):
        super().__init__("base_bridge exited with %s; log: %s" % (code, log))
        self.code, self.log = code, log


def _say(text):
    """Operator message on stderr; a closed terminal (SIGHUP) must not break the shutdown."""
    try:
        print(text, file=sys.stderr, flush=True)
    except (OSError, ValueError):
        pass


class InterruptGuard:
    """SIGINT, SIGTERM and SIGHUP of one live-chain command, in the main thread.

    The first signal raises KeyboardInterrupt, which starts the shutdown. Inside a ``deferred()`` block (starting
    a child process) it is held until the block ends, so a process that was just started is always registered
    and stopped. Every later signal, and every signal after ``begin_shutdown()``, only prints that the shutdown
    is running: a second Ctrl-C can no longer cut the stopping of this command's processes or the result file.
    """

    SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)

    def __init__(self):
        self.received = []
        self.shutting_down = False
        self._depth = 0
        self._pending = False
        self._previous = {}

    def install(self):
        for number in self.SIGNALS:
            self._previous[number] = signal.signal(number, self._handle)
        return self

    def restore(self):
        for number, handler in self._previous.items():
            signal.signal(number, handler)
        self._previous = {}

    def _handle(self, number, frame):
        self.received.append(signal.Signals(number).name)
        if self.shutting_down:
            _say("收尾中：正在停本命令启动的进程并写 result.json，再按 Ctrl-C 不会打断")
            return
        self.shutting_down = True
        if self._depth:
            self._pending = True
            return
        raise KeyboardInterrupt

    def begin_shutdown(self):
        self.shutting_down = True

    @contextlib.contextmanager
    def deferred(self):
        self._depth += 1
        try:
            yield
        finally:
            self._depth -= 1
        if not self._depth and self._pending:
            self._pending = False
            raise KeyboardInterrupt


def require_live_domain(config=None):
    """Live-chain nodes run only in the isolated autonomy domains (96-101, localhost)."""
    require_isolated_domain()


def with_sim_time(params, value):
    """Copy of a ROS parameter tree with every use_sim_time set to ``value``."""
    if isinstance(params, dict):
        return {k: (bool(value) if k == "use_sim_time" else with_sim_time(v, value)) for k, v in params.items()}
    if isinstance(params, list):
        return [with_sim_time(v, value) for v in params]
    return copy.deepcopy(params)


class LiveMonitor:
    """Supervisor node: statuses, parameter readback, operator start pose and the enable heartbeat.

    In the closed loop there is no enable publisher here at all: the base bridge is the only source of
    /wc_nav/enable (the gate checks that), and its /wc_nav/base_status is read for the operator's status line.
    """

    def __init__(self, context, use_sim_time, closed_loop=False):
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from std_msgs.msg import Bool, String
        from tf2_ros import Buffer, TransformListener
        self.node = rclpy.create_node("wc_live_supervisor", context=context,
                                      parameter_overrides=[Parameter("use_sim_time", value=bool(use_sim_time))])
        self.executor = SingleThreadedExecutor(context=context)
        self.executor.add_node(self.node)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.status = {}
        self.enabled = False
        self.Bool, self.Pose = Bool, PoseWithCovarianceStamped
        for topic in ("/wc_live/inputs_status/scan", "/wc_live/inputs_status/odometry", "/wc_nav/gate_status",
                      "/wc_live/goal_status", "/wc_live/relay_status", OBSTACLES_STATUS):
            self.node.create_subscription(String, topic, lambda m, t=topic: self.status.__setitem__(
                t, json.loads(m.data)), latched_qos())
        self.enable_pub = None
        # The wheel owner's (auto_state, reason, immediate_reason) as reported through the bridge, adjacent
        # repeats merged: why the bridge exited is told from this when the owner closes its socket (D.md 23:40).
        self.owner_history = []
        if closed_loop:
            # the bridge publishes it reliable and volatile at 10 Hz (a transient local reader would not match)
            self.node.create_subscription(String, BASE_STATUS_TOPIC, self._on_base_status, 10)
        else:
            self.enable_pub = self.node.create_publisher(Bool, "/wc_nav/enable", 10)
            self.node.create_timer(0.25, self._heartbeat)
        self.initial_pub = self.node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        # standing scans and the odometry pose for the start check (live/start_check.py)
        from collections import deque
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        self.scans = deque(maxlen=50)
        self.odom_pose = None
        self.node.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.node.create_subscription(Odometry, "/odom", self._on_odom, 20)
        # front check (live/front_check.py), set up by the supervisor once the map is loaded
        self.front, self.front_pub, self.front_value = None, None, None
        self.front_before_start = []         # median counts published before the gate was first ACTIVE
        self.gate_was_active = False

    def setup_front_check(self, check):
        """Start counting scan points in the map's free space ahead (``check``: front_check.FrontCheck)."""
        from std_msgs.msg import String
        self.front = check
        self.front_pub = self.node.create_publisher(String, FRONT_TOPIC, latched_qos())
        self.node.create_timer(0.5, self._publish_front)

    def _publish_front(self, changed_only=False):
        """2 Hz from the timer, and at once from a scan that changes the verdict."""
        value = self.front.verdict(time.monotonic_ns(), self.now())
        previous = self.front_value
        if changed_only and previous is not None and \
                (value["clear"], value["reason"]) == (previous["clear"], previous["reason"]):
            return
        self.front_value = value
        self.status[FRONT_TOPIC] = value
        if self.state(GATE_STATUS_TOPIC) == "ACTIVE":
            self.gate_was_active = True
        if not changed_only and not self.gate_was_active and value["points"] is not None:
            self.front_before_start.append(value["points"])      # sampled at 2 Hz while standing before the start
        self.front_pub.publish(json_message(value))

    def _on_base_status(self, message):
        value = json.loads(message.data)
        self.status[BASE_STATUS_TOPIC] = value
        owner = value.get("owner") if isinstance(value.get("owner"), dict) else {}
        entry = {"auto_state": owner.get("auto_state"), "reason": owner.get("reason"),
                 "immediate_reason": owner.get("immediate_reason")}
        if not self.owner_history or {k: self.owner_history[-1][k] for k in entry} != entry:
            self.owner_history.append(dict(entry, time_s=round(self.now(), 3)))
            del self.owner_history[:-50]

    def _on_scan(self, message):
        self.scans.append((list(message.ranges), float(message.angle_min), float(message.angle_increment)))
        if self.front is not None:
            found = self.map_pose()
            pose = None
            if found is not None and abs(self.now() - found[1]) <= 1.0:   # front_check.POSE_MAX_AGE_S
                pose = found[0]
            self.front.add_scan(message.ranges, message.angle_min, message.angle_increment, pose)
            self._publish_front(changed_only=True)

    def _on_odom(self, message):
        from ..autonomy.ros_util import quaternion_to_yaw
        position = message.pose.pose.position
        self.odom_pose = (float(position.x), float(position.y), quaternion_to_yaw(message.pose.pose.orientation))

    def _heartbeat(self):
        message = self.Bool()
        message.data = bool(self.enabled)
        self.enable_pub.publish(message)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=min(0.05, max(0.0, end - time.monotonic())))

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def foreign_nodes(self):
        own = (self.node.get_name(), self.node.get_namespace())
        return [n for n in self.node.get_node_names_and_namespaces() if n != own]

    def state(self, topic):
        return (self.status.get(topic) or {}).get("state")

    def map_pose(self):
        return lookup_pose(self.buffer, "map", "base_link")

    def trigger(self, service, timeout_s=2.0):
        from std_srvs.srv import Trigger
        client = self.node.create_client(Trigger, service)
        try:
            if not client.wait_for_service(timeout_sec=0.5):
                return None
            future = client.call_async(Trigger.Request())
            end = time.monotonic() + timeout_s
            while not future.done() and time.monotonic() < end:
                self.executor.spin_once(timeout_sec=0.05)
            return future.result().success if future.done() and future.result() is not None else None
        finally:
            self.node.destroy_client(client)

    def get_parameters(self, node_name, names, timeout_s=5.0):
        from rcl_interfaces.srv import GetParameters
        client = self.node.create_client(GetParameters, "/" + node_name + "/get_parameters")
        try:
            if not client.wait_for_service(timeout_sec=timeout_s):
                raise RunError("parameter service unavailable: " + node_name)
            future = client.call_async(GetParameters.Request(names=list(names)))
            end = time.monotonic() + timeout_s
            while not future.done() and time.monotonic() < end:
                self.executor.spin_once(timeout_sec=0.05)
            if not future.done() or future.result() is None:
                raise RunError("parameter readback timed out: " + node_name)
            return dict(zip(names, (_parameter_value(v) for v in future.result().values)))
        finally:
            self.node.destroy_client(client)

    def publish_initial_pose(self, pose, covariance):
        from ..autonomy.ros_util import pose_stamped
        message = self.Pose()
        stamped = pose_stamped("map", self.node.get_clock().now().to_msg(), *pose)
        message.header, message.pose.pose = stamped.header, stamped.pose
        message.pose.covariance = [float(v) for v in covariance]
        self.initial_pub.publish(message)

    def close(self):
        self.executor.remove_node(self.node)
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()


class LiveSupervisor(Supervisor):
    """Start order: recorder -> inputs (-> relay) -> [map_server + AMCL] -> source -> [start pose -> start check ->
    Nav2 -> readback -> smoother feedback (-> base bridge) -> gate -> goals]. The bracketed steps are the shadow
    run; Nav2's costmaps only activate once sensor data, odometry TF and AMCL's map->odom exist."""

    def __init__(self, project_root, output_dir, config, guard=None):
        super().__init__(project_root, output_dir, config)
        self.guard = guard

    def launch(self, name, args):
        if self.guard is None:
            return super().launch(name, args)
        with self.guard.deferred():
            if self.guard.shutting_down:
                raise KeyboardInterrupt   # a stop was asked for: start nothing new
            return super().launch(name, args)

    def stop_all(self):
        """Stop this command's processes, newest first, each on its own: one failure does not skip the rest.

        The closed loop's base bridge goes first: when its socket closes, the wheel owner ends automatic control
        and writes zero, before the gate's decisions could only go stale.
        """
        errors = {}
        running = sum(1 for _, process in self.processes if process.poll() is None)
        if running:
            _say("收尾：停本命令启动的 %d 个进程…" % running)
        for name, process in sorted(reversed(self.processes), key=lambda item: item[0] != BRIDGE):
            try:
                _stop_owned(process)
            except Exception as exc:
                errors[name] = type(exc).__name__ + ": " + str(exc)
            self.report["process_exit_codes"][name] = process.poll()
        for handle in self.handles:
            try:
                handle.close()
            except OSError:
                pass
        if errors:
            self.report["stop_errors"] = errors
        if running:
            _say("收尾：进程已停" + ("，%d 个停止出错（见 result.json）" % len(errors) if errors else ""))

    def node_config(self, name, value):
        return self.write(name + ".json", value)

    def launch_module(self, name, module, config_path, extra=()):
        return self.launch(name, [sys.executable, "-B", "-s", "-m", module, "--config", config_path, *extra])

    def start_bag(self):
        cfg = self.config
        topics = source_topics(bool(cfg.get("obstacles")))
        command = ["ros2", "bag", "play", cfg["bag"], "--clock", "100", "--rate", str(cfg.get("bag_rate", 1.0)),
                   "--topics", *topics]
        if cfg.get("bag_remap_prefix"):
            command += ["--remap"] + ["%s:=%s%s" % (t, cfg["bag_remap_prefix"], t) for t in topics]
        return self.launch("bag_play", command)

    def start_common(self, monitor, files):
        cfg = self.config
        self.launch_module("recorder", "wc_navigation.live.recorder_node", files["recorder"])
        for role in INPUT_ROLES:
            self.launch_module("inputs_" + role, "wc_navigation.live.inputs_node", files["inputs_" + role])
        if cfg.get("relay"):
            # +10 on the command's niceness (vehicle: 10 -> 19): the relay's loopback copy competes with the
            # panel's recorder for CPU (window A).
            self.launch("relay", ["nice", "-n", "10", sys.executable, "-B", "-s", "-m", "wc_navigation.live.relay_node",
                                  "--config", files["relay"]])
            # On the vehicle the relay first waits until the panel's own consumers are subscribed.
            wait_s = 600 if cfg["relay"].get("wait_for_panel_consumers") else 30
            self.wait_for(monitor, lambda: monitor.state("/wc_live/relay_status") == "RUNNING", wait_s, "relay_ready")
            self.report["relay_self_check"] = (monitor.status.get("/wc_live/relay_status") or {}).get("self_check")
        self.wait_for(monitor, lambda: all(monitor.state("/wc_live/inputs_status/" + role) is not None
                                           for role in INPUT_ROLES), 30, "inputs_ready")
        if files.get("obstacles"):
            # window C's 3D obstacle input (off by default), waited for on its own; +10 on the command's niceness
            # like the relay (vehicle: 10 -> 19), so its ~0.6 core yields to the inputs' odometry
            self.launch("obstacles", ["nice", "-n", "10", sys.executable, "-B", "-s", "-m",
                                      "wc_navigation.perception.obstacles_node", "--config", files["obstacles"]])
            self.wait_for(monitor, lambda: monitor.state(OBSTACLES_STATUS) is not None, 30, "obstacles_ready")

    def prepare_shadow(self, profile):
        from ..autonomy.shadow_localization import amcl_parameters
        cfg = self.config
        files = {}
        files["bt_to_pose"] = self.write("bt_navigate_to_pose.xml", nav2_config.behavior_tree_xml())
        files["bt_through"] = self.write("bt_navigate_through_poses.xml",
                                         nav2_config.behavior_tree_xml(through_poses=True))
        sim = cfg["use_sim_time"]
        planning = self.planning()
        nav2_params = nav2_config.navigation_parameters(
            profile, mode="navigate", bt_to_pose_path=files["bt_to_pose"],
            bt_through_poses_path=files["bt_through"], planning=planning)
        if cfg.get("closed_loop"):
            # window B 2026-11-10: the rotation speeds are not in Nav2's readback list; the closed loop checks them,
            # and the values B kept for it on 2026-11-12 (B's adjust_readback adds the two it changed)
            self.closed_readback = closed_loop_rotation_readback(nav2_params, profile.max_angular)
            for node, values in closed_loop_kept_readback(nav2_params).items():
                self.closed_readback.setdefault(node, {}).update(values)
        files["nav2"] = self.write("nav2_params.yaml", with_sim_time(nav2_params, sim))
        if planning.smoother_feedback == "gated":
            # window B's relay: the gate's output as the smoother's feedback (restart from rest after a gate stop)
            from ..planning.velocity_feedback import node_config as feedback_config
            files["feedback"] = self.node_config("velocity_feedback_config", feedback_config(self.output,
                                                                                             use_sim_time=sim))
        localization = amcl_parameters(profile, {"map_yaml": cfg["map_yaml"], "amcl": cfg["amcl"]})
        files["localization"] = self.write("localization_params.yaml", with_sim_time(localization, sim))
        limits = cfg["gate_limits"]
        monitor = {"window_s": 2.0, "max_translation_m": limits["correction_translation_m"],
                   "max_yaw_rad": math.radians(limits["correction_yaw_deg"]), "hold_s": 3.0,
                   "reference": limits["correction_reference"]}
        covariance = {k: limits[k] for k in ("covariance_mode", "slow_std_m", "slow_linear") if k in limits}
        gate = {"max_linear": profile.max_linear, "max_angular": profile.max_angular, "require_covariance": True,
                "covariance_xy_std_max_m": limits["covariance_xy_std_m"],
                "covariance_yaw_std_max_rad": math.radians(limits["covariance_yaw_std_deg"]), **covariance}
        gate_node = {"use_sim_time": sim, "correction_monitor": monitor, "gate": gate}
        if (cfg.get("front_check") or {}).get("mode") == "block":
            gate["front_hold"] = True          # START_BLOCKED until the front check reports clear (each start)
        closed = cfg.get("closed_loop")
        if closed:
            # velocity_channel.md 9.2: the base must run automatic control, the bridge is the only enable and estop
            # source, the enable is stale after 0.5 s, and ages run on the monotonic clock
            gate.update(require_base=True, enable_source_node=closed["bridge_node"],
                        enable_timeout_s=closed["enable_timeout_s"])
            gate_node["clock"] = "monotonic"
        files["gate"] = self.node_config("gate_config", gate_node)
        files["goals"] = self.node_config("goals_config", {
            "use_sim_time": sim, "goals": cfg["goals"], "advance_radius_m": cfg["advance_radius_m"],
            "closed_loop": bool(closed)})
        return files

    def readback_expectations(self, profile):
        """Nav2's readback list for this run; the closed loop adds the two rotation speeds (prepare_shadow)."""
        expected = {node: dict(values) for node, values in nav2_config.readback_expectations(
            profile, "navigate", planning=self.planning()).items()}
        for node, values in (getattr(self, "closed_readback", None) or {}).items():
            expected.setdefault(node, {}).update(values)
        return expected

    def planning(self):
        """Window B's planning choice for the shadow's Nav2: the smoother feedback and timeout, and in the closed loop
        the values fixed by cli.py: ``closed_loop.controller_params`` (window B 2026-11-12 02:10) and
        ``closed_loop.global_live_scans`` (false: the global costmap has the saved map only, user 2026-11-12)."""
        from ..planning.stack import PlanningChoice
        cfg = self.config
        defaults = PlanningChoice()
        closed = cfg.get("closed_loop") or {}
        return PlanningChoice(smoother_feedback=cfg.get("smoother_feedback", defaults.smoother_feedback),
                              smoother_timeout_s=float(cfg.get("smoother_timeout_s", defaults.smoother_timeout_s)),
                              controller_params=tuple(tuple(p) for p in closed.get("controller_params", ())),
                              global_live_scans=bool(closed.get("global_live_scans", defaults.global_live_scans)))

    def start_localization(self, monitor, files):
        self.launch_ros("map_server", "map_server", files["localization"])
        self.launch_ros("amcl", "amcl", files["localization"])
        self.launch_ros("lifecycle_manager_localization", "lifecycle_manager", files["localization"],
                        ["-r", "__node:=lifecycle_manager_localization"])
        self.wait_for(monitor, lambda: monitor.trigger("/lifecycle_manager_localization/is_active") is True,
                      60, "localization_active")

    def start_shadow(self, monitor, files, profile):
        """Nav2 and the gate; the costmaps need map->base_link, so sensors and AMCL must already run."""
        for name in nav2_config.NAV2_NODES:
            self.launch_ros(name, name, files["nav2"])
        self.launch_ros("lifecycle_manager_navigation", "lifecycle_manager", files["nav2"],
                        ["-r", "__node:=lifecycle_manager_navigation"])
        self.wait_for(monitor, lambda: monitor.trigger("/lifecycle_manager_navigation/is_active") is True,
                      120, "nav2_active")
        readback = {}
        for node_name, expected in self.readback_expectations(profile).items():
            actual = monitor.get_parameters(node_name, list(expected))
            readback[node_name] = {"expected": expected, "actual": actual}
            bad = [k for k in expected if not _same(actual.get(k), expected[k])]
            if bad:
                raise RunError(f"parameter readback mismatch on {node_name}: {bad} -> {actual}")
        self.report["parameter_readback"] = readback
        if "feedback" in files:                     # --smoother-feedback gated (default since 2026-11-10)
            from ..planning.stack import FEEDBACK_TOPIC
            self.launch_module("velocity_feedback", "wc_navigation.planning.velocity_feedback", files["feedback"])
            self.wait_for(monitor, lambda: monitor.node.count_publishers(FEEDBACK_TOPIC) > 0, 30,
                          "velocity_feedback_ready")
        closed = self.config.get("closed_loop")
        if closed:
            self.start_bridge(monitor, closed)
        self.launch_module("safety_gate", "wc_navigation.autonomy.gate_node", files["gate"])
        monitor.enabled = not closed               # the shadow's own enable heartbeat; none in the closed loop

    def start_bridge(self, monitor, closed):
        """Window A's base bridge (wc_motion.auto_bridge), before the gate: the gate needs it as the only publisher
        of /wc_nav/enable and /wc_nav/estop. It refuses without today's WC_AUTO_DRIVE_APPROVAL and exits when the
        wheel owner's socket closes; either ends the run (check_alive)."""
        self.launch(BRIDGE, [sys.executable, "-B", "-s", "-m", "wc_motion.auto_bridge",
                             "--project-root", closed["panel_root"], "--session-root", closed["session_root"],
                             "--session-id", closed["session_id"]])
        topics = (BASE_STATUS_TOPIC, "/wc_nav/enable", "/wc_nav/estop")
        self.wait_for(monitor, lambda: all(monitor.node.count_publishers(t) > 0 for t in topics)
                      and monitor.status.get(BASE_STATUS_TOPIC) is not None, 30, "bridge_ready")
        _say(closed_loop_line(monitor.status))

    def initial_pose(self, monitor):
        cfg = self.config
        covariance = nav2_config.initial_pose_covariance(cfg["initial_pose_std"][0],
                                                         math.radians(cfg["initial_pose_std"][1]))
        published = {"count": 0, "last": -1e9}

        def localized():
            if monitor.map_pose() is not None:
                return True
            if time.monotonic() - published["last"] > 1.0:
                monitor.publish_initial_pose(cfg["initial_pose"], covariance)
                published["count"] += 1
                published["last"] = time.monotonic()
            return False

        self.wait_for(monitor, localized, 60, "amcl_tf")
        self.report["initial_pose"] = {"pose_map": cfg["initial_pose"], "covariance": covariance,
                                       "messages_sent": published["count"]}
        self.start_odom = monitor.odom_pose          # where the chair stood when AMCL took the start pose

    def map_scorer(self):
        """The saved map's distance to the nearest occupied cell (start check and front check share it)."""
        if getattr(self, "_map_scorer", None) is None:
            from ..autonomy.maps import read_back
            from .amcl_cross import EndpointScorer
            _, data, meta = read_back(self.config["map_yaml"])
            self._map_scorer = EndpointScorer(data, meta["resolution"], meta["origin"])
        return self._map_scorer

    def laser_lever(self):
        """The laser's (x, y) in base_link (the live chain's static base_link->laser has no rotation)."""
        return json.loads(Path(self.config["live_config_file"]).read_text(encoding="utf-8"))["lever_xy"]

    def start_front_check(self, monitor):
        """Count scan points in the map's free space ahead from now on (``--front-check block|warn``)."""
        settings = self.config.get("front_check") or {}
        if settings.get("mode", "off") == "off":
            return
        from .front_check import FrontCheck
        profile = self.config["profile"]
        monitor.setup_front_check(FrontCheck(
            self.map_scorer(), self.laser_lever(), profile.get("footprint") or (), profile.get("radius_m", 0.55),
            mode=settings["mode"], max_points=int(settings["max_points"]), range_m=settings["range_m"],
            wall_tolerance_m=settings["wall_tolerance_m"], frames=settings["frames"]))

    def check_alive(self, allow=()):
        """The base bridge first: its exit (closed loop) is told apart, with the reason (BridgeExited)."""
        for name, process in self.processes:
            if name == BRIDGE and process.poll() is not None and name not in allow:
                raise BridgeExited(process.returncode, self.logs.get(name))
        return super().check_alive(allow)

    def start_check(self, monitor):
        """Standing scans against the map at the given start pose, before Nav2 and the gate start."""
        from . import start_check as check
        cfg = self.config
        mode = cfg.get("start_check", "block")
        if mode == "off":
            self.report["start_check"] = {"status": "OFF", "mode": mode}
            return
        scorer = self.map_scorer()
        lever = self.laser_lever()
        reference = getattr(self, "start_odom", None)
        monitor.scans.clear()
        status = None
        end = time.monotonic() + 15.0
        while len(monitor.scans) < check.MIN_SCANS and time.monotonic() < end:
            self.check_alive(allow=("bag_play",))
            monitor.spin(0.1)
            if reference is None:
                reference = monitor.odom_pose
            elif monitor.odom_pose is not None and check.moved(monitor.odom_pose, reference):
                status = "START_CHECK_NOT_STANDING"
                break
        scans = list(monitor.scans)
        result = (check.evaluate(scorer, scans, lever, cfg["initial_pose"]) if status is None
                  else {"status": status, "scans": len(scans)})
        result.update(mode=mode, standing_verified=reference is not None)
        self.report["start_check"] = result
        self.report["stages"].append({"stage": "start_check", "status": result["status"],
                                      "wall_s": round(time.monotonic() - self.t0, 2), "sim_s": round(monitor.now(), 3)})
        _say(check.operator_message(result, mode))
        if check.stops_run(result["status"], mode):
            raise StartCheckStop(result["status"])

    def run(self):
        """Run until the source ends, the duration limit or the operator's stop; always returns the report."""
        self.t0 = time.monotonic()
        try:
            self._run()
        except KeyboardInterrupt:
            # interrupted before the processes' try block, or (without a guard) during its clean-up
            if self.guard is not None:
                self.guard.begin_shutdown()
            self.report["stopped"] = INTERRUPTED
            self.stop_all()
        if self.guard is not None:
            self.report["signals"] = list(self.guard.received)
        self.report["logs"] = self.logs
        self.report["process_args"] = self.commands
        self.report["wall_s"] = round(time.monotonic() - self.t0, 2)
        return self.report

    def _run(self):
        require_live_domain()
        cfg = self.config
        if cfg.get("niceness") is not None:
            # vehicle: the panel's recording comes first (2026-11-09); every child inherits this niceness
            self.report["niceness"] = lower_priority(int(cfg["niceness"]))
        self.report["ros_domain_id"] = int(simulation_domain())
        versions = subprocess.run(["dpkg-query", "-W", "-f=${Package} ${Version}\n", *PACKAGES],
                                  capture_output=True, text=True, timeout=10.0)
        self.report["package_versions"] = versions.stdout.strip().splitlines()
        (self.output / "logs").mkdir()
        files = {"inputs_" + role: self.node_config("inputs_%s_config" % role, {
                     "role": role, "use_sim_time": cfg["use_sim_time"], "live": cfg["live_config_file"],
                     "scan_rate_hz": cfg.get("scan_rate_hz", 10.0), "odom_period_s": cfg.get("odom_period_s", 0.02),
                     "lidar_time_offset_ms": cfg["lidar_time_offset_ms"],
                     "source_prefix": cfg.get("source_prefix", "")}) for role in INPUT_ROLES}
        files.update({
                 "recorder": self.node_config("recorder_config", {
                     "use_sim_time": cfg["use_sim_time"], "output_dir": str(self.output),
                     "record_scan_ranges": cfg.get("record_scan_ranges", False)})})
        if cfg.get("relay"):
            files["relay"] = self.node_config("relay_config", cfg["relay"])
        if cfg.get("obstacles"):
            files["obstacles"] = self.node_config("obstacles_config", {
                "use_sim_time": cfg["use_sim_time"], "live": cfg["live_config_file"], "obstacles": cfg["obstacles"],
                "source_prefix": cfg.get("source_prefix", ""), "output_dir": str(self.output)})
        profile = nav2_config.NavProfile(**cfg["profile"]) if cfg["kind"] == "shadow" else None
        if profile is not None:
            for name in (*nav2_config.NAV2_NODES, "lifecycle_manager", "map_server", "amcl"):
                if not EXECUTABLES[name].is_file():
                    raise RunError("installed executable missing: " + str(EXECUTABLES[name]))
            files.update(self.prepare_shadow(profile))
        self.report["files"] = {k: str(v) for k, v in files.items()}
        import rclpy
        from rclpy.context import Context
        context = Context()
        rclpy.init(args=[], context=context)
        monitor = None
        closed = cfg.get("closed_loop")
        bridge_exit = None
        try:
            monitor = LiveMonitor(context, cfg["use_sim_time"], closed_loop=bool(closed))
            monitor.spin(2.0)
            foreign = monitor.foreign_nodes()
            if foreign:
                raise RunError("ROS domain %s is already in use: %s" % (simulation_domain(), foreign))
            self.start_common(monitor, files)
            if profile is not None:
                self.start_localization(monitor, files)
            bag = self.start_bag() if cfg["source"] == "bag" else None
            if profile is not None:
                self.initial_pose(monitor)
                self.start_check(monitor)
                self.start_front_check(monitor)            # verdicts flow before the gate starts (front_hold)
                self.start_shadow(monitor, files, profile)
                self.launch_module("goals", "wc_navigation.live.goals_node", files["goals"])
            budget = cfg.get("max_duration_s", 900.0)
            last_line = 0.0
            while True:
                if bag is not None and bag.poll() is not None:
                    break
                if time.monotonic() - self.t0 > budget:
                    self.report["stopped"] = "DURATION_LIMIT"
                    break
                self.check_alive(allow=("bag_play", "goals"))      # the bridge or the gate exiting ends the run
                monitor.spin(0.2)
                if closed and time.monotonic() - last_line >= 1.0:
                    _say(closed_loop_line(monitor.status))         # also while standing before "start automatic"
                    last_line = time.monotonic()
            self.report["stages"].append({"stage": "source_finished", "wall_s": round(time.monotonic() - self.t0, 2),
                                          "bag_exit_code": None if bag is None else bag.returncode})
            monitor.enabled = False
            monitor.spin(2.0)
            self.report["final_status"] = dict(monitor.status)
        except KeyboardInterrupt:
            self.report["stopped"] = INTERRUPTED
            _say("收到停止信号，开始收尾（再按 Ctrl-C 不会打断）")
            if monitor is not None:
                self.report["final_status"] = dict(monitor.status)
        except StartCheckStop as stop:
            self.report["stopped"] = stop.status          # Nav2 and the gate were never started
        except BridgeExited as exc:
            # 总管 2026-11-11 item 4: say why. Read the owner's journal now, before the panel moves its session.
            self.report["error"] = type(exc).__name__ + ": " + str(exc)
            self.report["stopped"] = "BASE_BRIDGE_EXITED"
            bridge_exit = {"exit_code": exc.code, "bridge_failure": bridge_failure(exc.log),
                           "session": session_journal_outcome((closed or {}).get("session_root"))}
        except Exception as exc:
            self.report["error"] = type(exc).__name__ + ": " + str(exc)
        finally:
            if self.guard is not None:
                self.guard.begin_shutdown()
            if monitor is not None:
                monitor.enabled = False
                try:
                    monitor.spin(0.3)
                except Exception:
                    pass
                if closed:
                    self.report["closed_loop"] = closed_loop_report(monitor.status)
                    self.report["closed_loop"]["front_check"] = front_check_report(monitor)
                    if bridge_exit is not None:
                        self.report["closed_loop"]["bridge_exit"] = bridge_exit
            self.stop_all()
            if bridge_exit is not None:
                if bridge_exit["session"].get("status") != "READ" or not bridge_exit["session"].get("fault"):
                    later = session_journal_outcome((closed or {}).get("session_root"))   # the journal may lag
                    if later.get("status") == "READ":
                        bridge_exit["session"] = later
                owner = self.report.get("closed_loop", {})
                bridge_exit.update(owner_last=owner_last(owner.get("base_status")),
                                   owner_reasons=list(monitor.owner_history) if monitor is not None else [])
                self.report["stop_reason"] = bridge_exit_text(bridge_exit)
                _say(self.report["stop_reason"])
            try:
                if monitor is not None:
                    monitor.close()
                if context.ok():
                    context.shutdown()
            except Exception as exc:
                self.report["close_error"] = type(exc).__name__ + ": " + str(exc)


def closed_loop_rotation_readback(params, max_angular):
    """Readback of the closed loop's rotation speeds, taken from the Nav2 parameters this run writes.

    RPP turns in place at ``FollowPath.rotate_to_heading_angular_vel`` and the behaviour server's Spin is limited by
    ``max_rotational_vel``; Nav2's readback list has neither (window B, 2026-11-10). Both must be given and must not
    exceed the closed loop's angular limit, else the run does not start (RunError).
    """
    def value(node, *path):
        found = (params.get(node) or {}).get("ros__parameters") or {}
        for key in path:
            found = found.get(key) if isinstance(found, dict) else None
        if isinstance(found, bool) or not isinstance(found, (int, float)):
            raise RunError("closed loop: %s %s missing in the Nav2 parameters" % (node, ".".join(path)))
        if not 0.0 < float(found) <= float(max_angular) + 1e-9:
            raise RunError("closed loop: %s %s = %s above the angular limit %s rad/s"
                           % (node, ".".join(path), found, max_angular))
        return float(found)
    return {"controller_server": {"FollowPath.rotate_to_heading_angular_vel":
                                  value("controller_server", "FollowPath", "rotate_to_heading_angular_vel")},
            "behavior_server": {"max_rotational_vel": value("behavior_server", "max_rotational_vel")}}


def closed_loop_kept_readback(params):
    """Readback of the controller values window B kept for the closed loop (2026-11-12 02:10), from the Nav2
    parameters this run writes: RPP still turns in place (``rotate_to_heading_min_angle`` needs
    ``use_rotate_to_heading``), and the goal checker's tolerances. A missing value stops the run (RunError)."""
    server = (params.get("controller_server") or {}).get("ros__parameters") or {}
    follow, goal = server.get("FollowPath") or {}, server.get("general_goal_checker") or {}
    kept = {"FollowPath.use_rotate_to_heading": follow.get("use_rotate_to_heading"),
            "general_goal_checker.xy_goal_tolerance": goal.get("xy_goal_tolerance"),
            "general_goal_checker.yaw_goal_tolerance": goal.get("yaw_goal_tolerance")}
    missing = [key for key, value in kept.items() if value is None]
    if missing:
        raise RunError("closed loop: controller_server %s missing in the Nav2 parameters" % ", ".join(missing))
    return {"controller_server": kept}


def _switch(*values):
    """按下 when any value is True, 否 when one is False and none True, else 未知 (null: not readable)."""
    if any(v is True for v in values):
        return "按下"
    return "否" if any(v is False for v in values) else "未知"


def closed_loop_line(status):
    """The operator's one-line view of the closed loop (every second, also standing before "start automatic")."""
    gate = status.get(GATE_STATUS_TOPIC) or {}
    base = status.get(BASE_STATUS_TOPIC) or {}
    goal = status.get(GOAL_STATUS_TOPIC) or {}
    conditions = gate.get("conditions") or {}
    reasons = ",".join(gate.get("reasons") or [])
    parts = ["闭环状态：闸门 " + str(gate.get("state", "未启动")) + ("(" + reasons + ")" if reasons else "")]
    preview = gate.get("preview") or {}
    if preview:      # what it would pass with the enable and the base in order: the shadow's view while standing
        why = ",".join(preview.get("reasons") or [])
        if preview.get("state") == "ACTIVE":
            parts.append("影子判断 会放行 %.2f m/s %.2f rad/s%s" % (float(preview.get("linear") or 0.0),
                                                              float(preview.get("angular") or 0.0),
                                                              "(" + why + ")" if why else ""))
        else:
            parts.append("影子判断 %s%s" % (preview.get("state"), "(" + why + ")" if why else ""))
    if conditions:
        parts.append("定位 " + ("可信" if conditions.get("localization_ok")
                                else "不可信(%s)" % conditions.get("localization_reason")))
    if FRONT_TOPIC in status:            # --front-check block|warn (总管 2026-11-11 item 2)
        from .front_check import operator_text
        parts.append(operator_text(status[FRONT_TOPIC]))
    if base:
        parts.append("底盘 %s%s" % (base.get("auto_state") or "未知",
                                    "" if base.get("base_auto") else "(%s)" % base.get("reason")))
        parts.append("使能 " + ("是" if base.get("enable") else "否"))
        parts.append("急停 " + _switch(base.get("estop"), base.get("estop_request")))
    else:
        parts.append("底盘 没有桥的状态")
    if gate:
        parts.append("闸门输出 %.2f m/s %.2f rad/s" % (float(gate.get("linear") or 0.0), float(gate.get("angular") or 0.0)))
    if goal:
        parts.append("目标 %s %d/%d" % (goal.get("state"), int(goal.get("goal_index", 0)) + 1, int(goal.get("goals", 0))))
    return "；".join(parts)


def bridge_failure(log_path):
    """The base bridge's own reason: its last JSON line with ``failure`` (wc_motion.auto_bridge), else the log's
    last non-empty line (a refusal at start prints plain text)."""
    try:
        lines = [line.strip() for line in Path(log_path).read_text(encoding="utf-8", errors="replace").splitlines()]
    except (OSError, TypeError):
        return None
    for line in reversed(lines):
        if line.startswith("{"):
            try:
                value = json.loads(line)
            except ValueError:
                continue
            if isinstance(value, dict) and value.get("failure"):
                return str(value["failure"])
    tail = [line for line in lines if line]
    return tail[-1][:300] if tail else None


SESSION_JOURNAL = Path("sources") / "wheel_feedback.jsonl"     # the wheel owner's journal in a panel session
SESSION_EVENTS = ("auto_state", "manual_fault", "capture_failed", "auto_bridge_lost")


def session_journal_outcome(session_root, tail_bytes=65536):
    """Read only: the last owner events (state, fault) at the end of the panel session's wheel journal.

    The journal outlives the owner's socket: a fault that ends the session (today's NOT_STATIONARY_AFTER_STOP)
    is written there although the bridge never saw a status carrying it. The panel may already have moved the
    session away; then the status is UNREADABLE and nothing else is tried.
    """
    if not session_root:
        return {"status": "NO_SESSION"}
    path = Path(session_root) / SESSION_JOURNAL
    try:
        with path.open("rb") as stream:
            stream.seek(0, os.SEEK_END)
            size = stream.tell()
            stream.seek(max(0, size - tail_bytes))
            data = stream.read()
    except OSError as exc:
        return {"status": "UNREADABLE", "path": str(path), "error": type(exc).__name__}
    lines = data.decode("utf-8", errors="replace").splitlines()
    if size > tail_bytes:
        lines = lines[1:]                     # the first line may be cut
    events = []
    for line in lines:
        try:
            value = json.loads(line)
        except ValueError:
            continue
        if isinstance(value, dict) and value.get("event") in SESSION_EVENTS:
            events.append({k: value[k] for k in ("event", "state", "reason", "error") if value.get(k) is not None})
    faults = [e.get("reason") or e.get("error") for e in events if e["event"] in ("manual_fault", "capture_failed")]
    return {"status": "READ", "path": str(path), "events": events[-8:], "fault": faults[-1] if faults else None}


def owner_last(base_status):
    """The wheel owner's last state as the bridge reported it (None without a base status)."""
    owner = (base_status or {}).get("owner") if isinstance((base_status or {}).get("owner"), dict) else None
    if owner is None:
        return None
    return {k: owner.get(k) for k in ("auto_state", "reason", "immediate_reason", "refusal", "estop_reasons")}


def bridge_exit_text(bridge_exit):
    """One operator line: why the base bridge, and with it the run, stopped."""
    parts = ["底盘桥退出（退出码 %s%s）" % (bridge_exit.get("exit_code"),
                                       "：" + bridge_exit["bridge_failure"] if bridge_exit.get("bridge_failure") else "")]
    last = bridge_exit.get("owner_last")
    if last:
        why = last.get("immediate_reason") or last.get("reason")
        parts.append("轮控所有者最后报 %s%s" % (last.get("auto_state"), "（%s）" % why if why else ""))
    session = bridge_exit.get("session") or {}
    if session.get("fault"):
        parts.append("会话轮控日志：%s" % session["fault"])
    elif session.get("status") == "UNREADABLE":
        parts.append("会话轮控日志读不到")
    return "；".join(parts)


def front_check_report(monitor):
    """result.json: the front check's mode, counts while standing before the first start, and its last verdict."""
    value = monitor.front_value
    if monitor.front is None:
        return {"mode": "off"}
    counts = list(monitor.front_before_start)
    summary = None
    if counts:
        ordered = sorted(counts)
        summary = {"samples": len(ordered), "min": ordered[0], "median": ordered[len(ordered) // 2], "max": ordered[-1]}
    return {"mode": monitor.front.mode, "max_points": monitor.front.max_points, "before_start": summary,
            "last": value, "no_pose_scans": monitor.front.no_pose}


def closed_loop_report(status):
    """result.json's closed_loop block: the last bridge status (with its counters and the owner's status) and the
    gate's identity and state counts as last published."""
    gate = status.get(GATE_STATUS_TOPIC) or {}
    return {"base_status": status.get(BASE_STATUS_TOPIC), "gate_id": gate.get("gate_id"), "gate_seq": gate.get("seq"),
            "gate_state": gate.get("state"), "gate_counts": gate.get("counts"), "gate_sources": gate.get("sources"),
            "goal_status": status.get(GOAL_STATUS_TOPIC)}


def lower_priority(target):
    """Raise this process's niceness to at least ``target`` (children inherit it); never raises the priority."""
    current = os.nice(0)
    if current < target:
        current = os.nice(target - current)
    return current


def vehicle_approved():
    """The vehicle mode needs the user's explicit, dated approval in the environment of this one command."""
    value = os.environ.get(VEHICLE_APPROVAL_ENV, "")
    return bool(value) and value.startswith("APPROVED_BY_USER_")
