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

VEHICLE_APPROVAL_ENV = "WC_LIVE_SHADOW_VEHICLE_APPROVAL"
INPUT_ROLES = ("scan", "odometry")          # separate processes: lidar work never delays wheel/IMU callbacks
OBSTACLES_STATUS = "/wc_live/inputs_status/obstacles"   # perception.live_obstacles.STATUS_TOPIC (window C)
INTERRUPTED = "OPERATOR_INTERRUPT"


class StartCheckStop(Exception):
    """The start check did not let the run go on (INITIAL_POSE_MISMATCH and the like)."""

    def __init__(self, status):
        super().__init__(status)
        self.status = status


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
    """Supervisor node: statuses, parameter readback, operator start pose and the enable heartbeat."""

    def __init__(self, context, use_sim_time):
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
        self.enable_pub = self.node.create_publisher(Bool, "/wc_nav/enable", 10)
        self.initial_pub = self.node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.node.create_timer(0.25, self._heartbeat)
        # standing scans and the odometry pose for the start check (live/start_check.py)
        from collections import deque
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        self.scans = deque(maxlen=50)
        self.odom_pose = None
        self.node.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.node.create_subscription(Odometry, "/odom", self._on_odom, 20)

    def _on_scan(self, message):
        self.scans.append((list(message.ranges), float(message.angle_min), float(message.angle_increment)))

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
    """Start order: recorder -> inputs (-> relay) -> [map_server + AMCL] -> source -> [start pose -> Nav2 ->
    readback -> gate -> goals]. The bracketed steps are the shadow run; Nav2's costmaps only activate once
    sensor data, odometry TF and AMCL's map->odom exist."""

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
        """Stop this command's processes, newest first, each on its own: one failure does not skip the rest."""
        errors = {}
        running = sum(1 for _, process in self.processes if process.poll() is None)
        if running:
            _say("收尾：停本命令启动的 %d 个进程…" % running)
        for name, process in reversed(self.processes):
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
        files["nav2"] = self.write("nav2_params.yaml", with_sim_time(nav2_config.navigation_parameters(
            profile, mode="navigate", bt_to_pose_path=files["bt_to_pose"],
            bt_through_poses_path=files["bt_through"]), sim))
        localization = amcl_parameters(profile, {"map_yaml": cfg["map_yaml"], "amcl": cfg["amcl"]})
        files["localization"] = self.write("localization_params.yaml", with_sim_time(localization, sim))
        limits = cfg["gate_limits"]
        monitor = {"window_s": 2.0, "max_translation_m": limits["correction_translation_m"],
                   "max_yaw_rad": math.radians(limits["correction_yaw_deg"]), "hold_s": 3.0,
                   "reference": limits["correction_reference"]}
        covariance = {k: limits[k] for k in ("covariance_mode", "slow_std_m", "slow_linear") if k in limits}
        files["gate"] = self.node_config("gate_config", {
            "use_sim_time": sim, "correction_monitor": monitor,
            "gate": {"max_linear": profile.max_linear, "max_angular": profile.max_angular, "require_covariance": True,
                     "covariance_xy_std_max_m": limits["covariance_xy_std_m"],
                     "covariance_yaw_std_max_rad": math.radians(limits["covariance_yaw_std_deg"]), **covariance}})
        files["goals"] = self.node_config("goals_config", {
            "use_sim_time": sim, "goals": cfg["goals"], "advance_radius_m": cfg["advance_radius_m"]})
        return files

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
        for node_name, expected in nav2_config.readback_expectations(profile, "navigate").items():
            actual = monitor.get_parameters(node_name, list(expected))
            readback[node_name] = {"expected": expected, "actual": actual}
            bad = [k for k in expected if not _same(actual.get(k), expected[k])]
            if bad:
                raise RunError(f"parameter readback mismatch on {node_name}: {bad} -> {actual}")
        self.report["parameter_readback"] = readback
        self.launch_module("safety_gate", "wc_navigation.autonomy.gate_node", files["gate"])
        monitor.enabled = True

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

    def start_check(self, monitor):
        """Standing scans against the map at the given start pose, before Nav2 and the gate start."""
        from . import start_check as check
        cfg = self.config
        mode = cfg.get("start_check", "block")
        if mode == "off":
            self.report["start_check"] = {"status": "OFF", "mode": mode}
            return
        from ..autonomy.maps import read_back
        from .amcl_cross import EndpointScorer
        _, data, meta = read_back(cfg["map_yaml"])
        scorer = EndpointScorer(data, meta["resolution"], meta["origin"])
        lever = json.loads(Path(cfg["live_config_file"]).read_text(encoding="utf-8"))["lever_xy"]
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
        try:
            monitor = LiveMonitor(context, cfg["use_sim_time"])
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
                self.start_shadow(monitor, files, profile)
                self.launch_module("goals", "wc_navigation.live.goals_node", files["goals"])
            budget = cfg.get("max_duration_s", 900.0)
            while True:
                if bag is not None and bag.poll() is not None:
                    break
                if time.monotonic() - self.t0 > budget:
                    self.report["stopped"] = "DURATION_LIMIT"
                    break
                self.check_alive(allow=("bag_play", "goals"))
                monitor.spin(0.2)
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
            self.stop_all()
            try:
                if monitor is not None:
                    monitor.close()
                if context.ok():
                    context.shutdown()
            except Exception as exc:
                self.report["close_error"] = type(exc).__name__ + ": " + str(exc)


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
