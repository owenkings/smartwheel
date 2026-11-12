"""Supervise one closed-loop autonomy run in a dedicated simulation domain (96, or 97).

Start order: simulator (clock) -> slam_toolbox (explore) or map_server + AMCL
(navigate) -> Nav2 servers under a lifecycle manager -> parameter readback ->
safety gate with an enable heartbeat -> explorer or mission. The supervisor
records ground truth, odometry, the localization estimate and every gated
command, then stops only the process groups it started.
"""
from __future__ import annotations

from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

from ..nav2_planner import _stop_owned
from . import nav2_config
from .ros_util import (compose, inverse, json_message, latched_qos, lookup_pose, quaternion_to_yaw,
                       require_isolated_domain, simulation_domain)

LIB = Path("/opt/ros/humble/lib")
EXECUTABLES = {
    "controller_server": LIB / "nav2_controller/controller_server",
    "planner_server": LIB / "nav2_planner/planner_server",
    "behavior_server": LIB / "nav2_behaviors/behavior_server",
    "bt_navigator": LIB / "nav2_bt_navigator/bt_navigator",
    "velocity_smoother": LIB / "nav2_velocity_smoother/velocity_smoother",
    "collision_monitor": LIB / "nav2_collision_monitor/collision_monitor",
    "lifecycle_manager": LIB / "nav2_lifecycle_manager/lifecycle_manager",
    "map_server": LIB / "nav2_map_server/map_server",
    "amcl": LIB / "nav2_amcl/amcl",
    "slam_toolbox": LIB / "slam_toolbox/async_slam_toolbox_node",
}
REMAPS = {
    "controller_server": ["-r", "cmd_vel:=" + nav2_config.CMD_NAV],
    "behavior_server": ["-r", "cmd_vel:=" + nav2_config.CMD_NAV],
    "velocity_smoother": ["-r", "cmd_vel:=" + nav2_config.CMD_NAV,
                          "-r", "cmd_vel_smoothed:=" + nav2_config.CMD_SMOOTHED],
}
PACKAGES = ["ros-humble-nav2-controller", "ros-humble-nav2-planner", "ros-humble-nav2-behaviors",
            "ros-humble-nav2-bt-navigator", "ros-humble-nav2-velocity-smoother",
            "ros-humble-nav2-collision-monitor", "ros-humble-nav2-lifecycle-manager",
            "ros-humble-nav2-map-server", "ros-humble-nav2-amcl",
            "ros-humble-nav2-regulated-pure-pursuit-controller", "ros-humble-nav2-navfn-planner",
            "ros-humble-nav2-smac-planner", "ros-humble-nav2-theta-star-planner",
            "ros-humble-nav2-rotation-shim-controller", "ros-humble-nav2-mppi-controller",
            "ros-humble-slam-toolbox"]


class RunError(RuntimeError):
    """The supervised run could not be started or completed as configured."""


def planning_choice(config):
    """Planner/controller selection of a run config (NavFn + RPP when absent)."""
    from ..planning.stack import PlanningChoice
    return PlanningChoice.from_dict(config.get("planning"))


def truth_localization(config):
    """Simulated navigation with map->odom from ground truth instead of AMCL (explicit option)."""
    return config.get("mode") == "navigate" and config.get("localization") == "truth"


@contextmanager
def domain_lock(project_root):
    """Exclusive owner of this run's simulation domain (96-101) within this checkout.

    The lock file lives under ``project_root``; separate checkouts running in
    parallel must use distinct domains (the foreign-node check is the backstop).
    """
    import fcntl
    domain = simulation_domain()
    directory = Path(project_root) / ".phase1_runtime/locks"
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / ("ros_domain_" + domain + ".lock")
    with path.open("a+b") as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RunError("another autonomy simulation owns ROS domain " + domain) from exc
        try:
            stream.seek(0)
            stream.truncate()
            stream.write(json.dumps({"pid": os.getpid(), "mode": "AUTONOMY_SIMULATION",
                                     "ros_domain_id": int(domain)}).encode())
            stream.flush()
            yield path
        finally:
            fcntl.flock(stream, fcntl.LOCK_UN)


def _parameter_value(value):
    kind = value.type
    return {1: lambda v: v.bool_value, 2: lambda v: v.integer_value, 3: lambda v: v.double_value,
            4: lambda v: v.string_value, 6: lambda v: list(v.bool_array_value),
            7: lambda v: list(v.integer_array_value), 8: lambda v: list(v.double_array_value),
            9: lambda v: list(v.string_array_value)}.get(kind, lambda v: None)(value)


def _same(actual, expected):
    if isinstance(expected, bool) or isinstance(actual, bool):
        return actual is expected
    if isinstance(expected, (int, float)) and isinstance(actual, (int, float)):
        return math.isclose(float(actual), float(expected), rel_tol=1e-9, abs_tol=1e-9)
    if isinstance(expected, list) and isinstance(actual, list):
        return len(actual) == len(expected) and all(_same(a, e) for a, e in zip(actual, expected))
    return actual == expected


class Recorder:
    """In-supervisor ROS node on simulated time: observation and enable heartbeat."""

    def __init__(self, context, world_T_map):
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import OccupancyGrid, Odometry
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import Bool, String
        from tf2_ros import Buffer, TransformListener
        self.node = rclpy.create_node("wc_autonomy_supervisor", context=context,
                                      parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.executor = SingleThreadedExecutor(context=context)
        self.executor.add_node(self.node)
        self.world_T_map = tuple(world_T_map)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.Bool = Bool
        self.clock_messages = 0
        self.truth = self.odom = None
        self.cmd_gated = (0.0, 0.0)
        self.cmd_nav = (0.0, 0.0)
        self.rows = []
        self.events = []
        self.gate_status = None
        self.sim_status = None
        self.map_messages = 0
        self.enabled = False
        self.enable_pause = None    # (from, to) simulated seconds: the heartbeat sends false (--enable-pause)
        node = self.node
        node.create_subscription(Clock, "/clock", self._on_clock, 10)
        node.create_subscription(Odometry, "/wc_sim/ground_truth", lambda m: setattr(self, "truth", self._pose(m)), 20)
        node.create_subscription(Odometry, "/odom", lambda m: setattr(self, "odom", self._pose(m)), 20)
        node.create_subscription(Twist, nav2_config.CMD_GATED,
                                 lambda m: setattr(self, "cmd_gated", (m.linear.x, m.angular.z)), 20)
        node.create_subscription(Twist, nav2_config.CMD_NAV,
                                 lambda m: setattr(self, "cmd_nav", (m.linear.x, m.angular.z)), 20)
        node.create_subscription(String, "/wc_nav/gate_status", self._on_gate, latched_qos())
        node.create_subscription(String, "/wc_sim/status", lambda m: setattr(self, "sim_status", json.loads(m.data)),
                                 latched_qos())
        node.create_subscription(String, "/wc_sim/collision", lambda m: self._event("collision", m), latched_qos(20))
        for topic in ("/wc_nav/explorer_status", "/wc_nav/mission_status", "/wc_nav/mapper_status"):
            node.create_subscription(String, topic, lambda m, t=topic: self._event(t.rsplit("/", 1)[-1], m),
                                     latched_qos())
        node.create_subscription(OccupancyGrid, "/map", self._on_map, latched_qos())
        # window B: every global plan (map frame, every 2nd pose), for replanning and side-flip counts
        from nav_msgs.msg import Path as PathMsg
        self.plans = []
        node.create_subscription(PathMsg, "/plan", self._on_plan, 10)
        from geometry_msgs.msg import PoseWithCovarianceStamped
        self.PoseWithCovarianceStamped = PoseWithCovarianceStamped
        self.initial_pose_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.enable_pub = node.create_publisher(Bool, "/wc_nav/enable", 10)
        node.create_timer(0.2, self._sample)
        node.create_timer(0.25, self._heartbeat)

    @staticmethod
    def _pose(message):
        p = message.pose.pose
        return (p.position.x, p.position.y, quaternion_to_yaw(p.orientation))

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _on_clock(self, message):
        self.clock_messages += 1

    def _on_map(self, message):
        self.map_messages += 1

    def _on_plan(self, message):
        if len(self.plans) < 5000 and message.poses:
            poses = message.poses[::2] + ([message.poses[-1]] if len(message.poses) % 2 == 0 else [])
            self.plans.append({"time_s": round(self.now(), 3), "frame": message.header.frame_id,
                               "xy": [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in poses]})

    def _event(self, kind, message):
        try:
            value = json.loads(message.data)
        except ValueError:
            value = {"raw": message.data}
        self.events.append({"kind": kind, "time_s": round(self.now(), 3), "value": value})

    def _on_gate(self, message):
        value = json.loads(message.data)
        if self.gate_status is None or value.get("state") != self.gate_status.get("state"):
            self.events.append({"kind": "gate_state", "time_s": value.get("time_s"),
                                "value": {k: value.get(k) for k in ("state", "previous_state", "reasons")}})
        self.gate_status = value

    def estimate_world(self):
        found = lookup_pose(self.buffer, "map", "base_link")
        if found is None:
            return None
        return compose(self.world_T_map, found[0])

    def _sample(self):
        if self.truth is None:
            return
        estimate = self.estimate_world()
        self.rows.append([round(self.now(), 3), *self.truth, *(self.odom or (math.nan,) * 3),
                          *(estimate or (math.nan,) * 3), *self.cmd_gated, *self.cmd_nav])

    def _heartbeat(self):
        message = self.Bool()
        paused = self.enable_pause is not None and self.enable_pause[0] <= self.now() < self.enable_pause[1]
        message.data = bool(self.enabled) and not paused
        self.enable_pub.publish(message)

    def publish_initial_pose(self, pose, covariance):
        """Operator-style approximate start pose in the map frame."""
        from .ros_util import pose_stamped
        message = self.PoseWithCovarianceStamped()
        stamped = pose_stamped("map", self.node.get_clock().now().to_msg(), *pose)
        message.header = stamped.header
        message.pose.pose = stamped.pose
        message.pose.covariance = [float(v) for v in covariance]
        self.initial_pose_pub.publish(message)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=min(0.05, max(0.0, end - time.monotonic())))

    def foreign_nodes(self):
        own = (self.node.get_name(), self.node.get_namespace())
        return [n for n in self.node.get_node_names_and_namespaces() if n != own]

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

    def close(self):
        self.executor.remove_node(self.node)
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()


class Supervisor:
    def __init__(self, project_root, output_dir, config):
        self.project = Path(project_root).resolve()
        self.output = Path(output_dir)
        self.config = config
        self.processes = []
        self.handles = []
        self.logs = {}
        self.commands = {}
        self.report = {"stages": [], "process_exit_codes": {}}

    # -- files and processes ---------------------------------------------------------
    def write(self, name, value):
        path = self.output / name
        with path.open("x", encoding="utf-8") as stream:
            if name.endswith((".yaml", ".yml")):
                import yaml
                stream.write(yaml.safe_dump(value, sort_keys=False))
            elif isinstance(value, str):
                stream.write(value)
            else:
                json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        return path

    def environment(self):
        env = os.environ.copy()
        env.update(ROS_DOMAIN_ID=simulation_domain(), WC_AUTONOMY_ROS_DOMAIN=simulation_domain(),
                   ROS_LOCALHOST_ONLY="1", PYTHONNOUSERSITE="1",
                   PYTHONDONTWRITEBYTECODE="1", RCUTILS_COLORIZED_OUTPUT="0",
                   ROS_LOG_DIR=str(self.output / "ros_logs"),
                   PYTHONPATH=str(self.project / "src") + os.pathsep + env.get("PYTHONPATH", ""))
        env.pop("ROS_DISCOVERY_SERVER", None)
        return env

    def launch(self, name, args):
        log = self.output / "logs" / (name + ".log")
        handle = log.open("x", encoding="utf-8")
        self.handles.append(handle)
        process = subprocess.Popen([str(a) for a in args], stdout=handle, stderr=subprocess.STDOUT,
                                   env=self.environment(), start_new_session=True, cwd=str(self.output))
        self.processes.append((name, process))
        self.logs[name] = str(log)
        self.commands[name] = [str(a) for a in args]
        return process

    def launch_ros(self, name, executable, params, extra=()):
        return self.launch(name, [EXECUTABLES[executable], "--ros-args", "--params-file", params,
                                  *REMAPS.get(name, []), *extra])

    def launch_python(self, name, module, config_path):
        return self.launch(name, [sys.executable, "-B", "-s", "-m", module, "--config", config_path])

    def check_alive(self, allow=()):
        for name, process in self.processes:
            code = process.poll()
            if code is not None and name not in allow:
                raise RunError(f"{name} exited with {code}; log: {self.logs[name]}")

    def wait_for(self, recorder, predicate, timeout_s, label, allow=()):
        end = time.monotonic() + timeout_s
        while not predicate():
            if time.monotonic() > end:
                raise RunError("timed out waiting for " + label)
            self.check_alive(allow)
            recorder.spin(0.1)
        self.report["stages"].append({"stage": label, "wall_s": round(time.monotonic() - self.t0, 2),
                                      "sim_s": round(recorder.now(), 3)})

    def stop_all(self):
        for name, process in reversed(self.processes):
            _stop_owned(process)
            self.report["process_exit_codes"][name] = process.returncode
        for handle in self.handles:
            handle.close()

    # -- run ---------------------------------------------------------------------------
    def mapper(self):
        mapper = self.config.get("mapper", "odom_grid")
        if mapper not in nav2_config.MAPPERS:
            raise RunError("unknown mapper: " + str(mapper))
        return mapper

    def prepare(self):
        cfg = self.config
        mode = cfg["mode"]
        profile = nav2_config.NavProfile(**cfg["profile"])
        (self.output / "logs").mkdir()
        files = {}
        tree = planning_choice(cfg).behavior_tree_options()
        files["bt_to_pose"] = self.write("bt_navigate_to_pose.xml", nav2_config.behavior_tree_xml(**tree))
        files["bt_through"] = self.write("bt_navigate_through_poses.xml",
                                         nav2_config.behavior_tree_xml(through_poses=True, **tree))
        nav2 = nav2_config.navigation_parameters(
            profile, mode=mode, bt_to_pose_path=files["bt_to_pose"], bt_through_poses_path=files["bt_through"],
            planning=planning_choice(cfg))
        files["nav2"] = self.write("nav2_params.yaml", nav2)
        if planning_choice(cfg).smoother_feedback == "gated":   # window B: smoother feedback relay (explicit option)
            from ..planning.velocity_feedback import node_config
            files["feedback"] = self.write("velocity_feedback_config.json", node_config(self.output))
        if mode == "explore":
            if self.mapper() == "slam_toolbox":
                files["mapping"] = self.write("slam_params.yaml", nav2_config.slam_parameters(
                    profile, loop_closure=bool(cfg.get("slam_loop_closure", False))))
            else:
                files["mapping"] = self.write("mapper_config.json", nav2_config.odom_grid_mapper_config(
                    profile, scan_matching=self.mapper() == "odom_icp"))
        elif truth_localization(cfg):
            # map_server only: the simulator publishes map->odom from ground truth (no AMCL).
            localization = nav2_config.localization_parameters(profile, cfg["map_yaml"])
            manager = dict(localization["lifecycle_manager_localization"]["ros__parameters"], node_names=["map_server"])
            files["mapping"] = self.write("map_server_params.yaml", {
                "map_server": localization["map_server"], "lifecycle_manager_localization": {"ros__parameters": manager}})
        else:
            files["mapping"] = self.write("localization_params.yaml", nav2_config.localization_parameters(
                profile, cfg["map_yaml"]))
        from ..planning.body import Body
        sim = {
            "world": cfg["world"], "start_pose": cfg["start_pose_world"], "laser": cfg["laser"],
            "collision_radius_m": profile.radius_m,
            # Same body as the costmaps and the collision monitor: the circle or the profile outline.
            "collision_body": Body.from_profile(profile, source=cfg.get("body_model")).as_config(),
            "odometry": cfg["odometry"], "real_time_factor": cfg["real_time_factor"], "seed": cfg["seed"]}
        if cfg.get("drive_response"):                          # window B: --drive-response (measured wheels)
            sim.update(wheel_response=dict(cfg["drive_response"]["wheel_response"]),
                       base_limits=dict(cfg["drive_response"]["base_limits"]))
        if mode == "navigate" and truth_localization(cfg):
            sim["map_to_odom_from_truth"] = {"world_T_map": list(cfg["world_T_map"])}
        if cfg.get("tracks"):                                   # window B: --tracks truth (actors as tracks)
            sim["tracks"] = dict(cfg["tracks"])
        if cfg.get("ghosts"):                                   # window B: --ghosts (false lidar points)
            sim["ghosts"] = list(cfg["ghosts"]["ghosts"])
        files["sim"] = self.write("sim_config.json", sim)
        if planning_choice(cfg).people_reaction:                # window B: --people-reaction
            from ..planning.people_node import node_config as people_config
            files["people"] = self.write("people_config.json", people_config(
                nav2, profile, Body.from_profile(profile, source=cfg.get("body_model")), planning_choice(cfg),
                self.output))
        files["gate"] = self.write("gate_config.json", {"gate": {
            "max_linear": profile.max_linear, "max_angular": profile.max_angular,
            "require_covariance": mode == "navigate" and not truth_localization(cfg),
            **cfg.get("gate_options", {})}})
        if mode == "explore":
            files["task"] = self.write("explorer_config.json", {
                # Planning radius: the circle, or the outline's inscribed radius the planner passes with.
                "output_dir": str(self.output), "radius_m": profile.body_inscribed_m, "margin_m": profile.margin_m,
                "max_duration_s": cfg["max_duration_s"], "nominal_speed": profile.max_linear * 0.6,
                **cfg.get("explorer", {})})
        else:
            mission = dict(cfg.get("mission", {}))
            if "goal_heading" in mission:   # the turn decision uses the goal checker's own yaw tolerance
                checker = nav2["controller_server"]["ros__parameters"]["general_goal_checker"]
                mission["goal_heading"] = dict(mission["goal_heading"],
                                               yaw_tolerance_rad=float(checker["yaw_goal_tolerance"]))
            files["task"] = self.write("mission_config.json", {
                "output_dir": str(self.output), "goals": cfg["goals_map"], "max_duration_s": cfg["max_duration_s"],
                "nominal_speed": profile.max_linear * 0.6, **mission})
        return profile, files

    def run(self):
        require_isolated_domain()
        domain = simulation_domain()
        self.report["ros_domain_id"] = int(domain)
        for name, path in EXECUTABLES.items():
            if not path.is_file():
                raise RunError("installed executable missing: " + str(path))
        versions = subprocess.run(["dpkg-query", "-W", "-f=${Package} ${Version}\n", *PACKAGES],
                                  capture_output=True, text=True, timeout=10.0)
        self.report["package_versions"] = versions.stdout.strip().splitlines()
        cfg = self.config
        mode = cfg["mode"]
        profile, files = self.prepare()
        self.report["files"] = {k: str(v) for k, v in files.items()}
        import rclpy
        from rclpy.context import Context
        context = Context()
        rclpy.init(args=[], context=context)
        recorder = None
        self.t0 = time.monotonic()
        wall_budget = cfg["max_duration_s"] / cfg["real_time_factor"] * 1.3 + 240.0
        task_status = None
        try:
            recorder = Recorder(context, cfg["world_T_map"])
            recorder.spin(2.0)
            foreign = recorder.foreign_nodes()
            if foreign:
                raise RunError(f"ROS domain {domain} is already in use: {foreign}")
            self.launch_python("sim", "wc_navigation.autonomy.sim_node", files["sim"])
            self.wait_for(recorder, lambda: recorder.clock_messages > 10 and recorder.truth is not None, 30, "sim_clock")
            if mode == "explore":
                if self.mapper() == "slam_toolbox":
                    self.launch_ros("slam_toolbox", "slam_toolbox", files["mapping"])
                else:
                    self.launch_python("mapper", "wc_navigation.autonomy.mapper_node", files["mapping"])
                self.wait_for(recorder, lambda: recorder.map_messages > 0 and recorder.estimate_world() is not None,
                              60, "map_and_tf")
            elif truth_localization(cfg):
                self.launch_ros("map_server", "map_server", files["mapping"])
                self.launch_ros("lifecycle_manager_localization", "lifecycle_manager", files["mapping"],
                                ["-r", "__node:=lifecycle_manager_localization"])
                self.wait_for(recorder, lambda: recorder.trigger("/lifecycle_manager_localization/is_active") is True,
                              60, "map_server_active")
                self.wait_for(recorder, lambda: recorder.map_messages > 0 and recorder.estimate_world() is not None,
                              30, "truth_map_to_odom")
                self.report["initial_pose"] = {"localization": "GROUND_TRUTH_MAP_TO_ODOM_NO_AMCL"}
            else:
                self.launch_ros("map_server", "map_server", files["mapping"])
                self.launch_ros("amcl", "amcl", files["mapping"])
                self.launch_ros("lifecycle_manager_localization", "lifecycle_manager", files["mapping"],
                                ["-r", "__node:=lifecycle_manager_localization"])
                self.wait_for(recorder, lambda: recorder.trigger("/lifecycle_manager_localization/is_active") is True,
                              60, "localization_active")
                covariance = nav2_config.initial_pose_covariance(*cfg["initial_pose_std"])
                published = {"count": 0, "last": -1e9}

                def localized():
                    # Re-send until AMCL publishes map->odom; stop as soon as it does,
                    # because every /initialpose message re-initializes the filter.
                    if recorder.estimate_world() is not None:
                        return True
                    if time.monotonic() - published["last"] > 1.0:
                        recorder.publish_initial_pose(cfg["initial_pose_map"], covariance)
                        published["count"] += 1
                        published["last"] = time.monotonic()
                    return False

                self.wait_for(recorder, localized, 30, "amcl_tf")
                self.report["initial_pose"] = {"pose_map": cfg["initial_pose_map"], "covariance": covariance,
                                               "messages_sent": published["count"]}
            for name in nav2_config.NAV2_NODES:
                self.launch_ros(name, name, files["nav2"])
            self.launch_ros("lifecycle_manager_navigation", "lifecycle_manager", files["nav2"],
                            ["-r", "__node:=lifecycle_manager_navigation"])
            self.wait_for(recorder, lambda: recorder.trigger("/lifecycle_manager_navigation/is_active") is True,
                          120, "nav2_active")
            readback = {}
            for node_name, expected in nav2_config.readback_expectations(profile, mode,
                                                                         planning=planning_choice(cfg)).items():
                actual = recorder.get_parameters(node_name, list(expected))
                readback[node_name] = {"expected": expected, "actual": actual}
                bad = [k for k in expected if not _same(actual.get(k), expected[k])]
                if bad:
                    raise RunError(f"parameter readback mismatch on {node_name}: {bad} -> {actual}")
            self.report["parameter_readback"] = readback
            if "feedback" in files:                             # window B: --smoother-feedback gated
                from ..planning.stack import FEEDBACK_TOPIC
                self.launch_python("velocity_feedback", "wc_navigation.planning.velocity_feedback", files["feedback"])
                self.wait_for(recorder, lambda: recorder.node.count_publishers(FEEDBACK_TOPIC) > 0, 30,
                              "velocity_feedback_ready")
            if "people" in files:                               # window B: --people-reaction
                from ..planning.people_node import SPEED_LIMIT_TOPIC
                self.launch_python("people", "wc_navigation.planning.people_node", files["people"])
                self.wait_for(recorder, lambda: recorder.node.count_publishers(SPEED_LIMIT_TOPIC) > 0, 30,
                              "people_reaction_ready")
            self.launch_python("safety_gate", "wc_navigation.autonomy.gate_node", files["gate"])
            recorder.enabled = True
            ready_states = ("COMMAND_STALE", "ACTIVE")
            self.wait_for(recorder, lambda: (recorder.gate_status or {}).get("state") in ready_states, 60, "gate_ready")
            task = "explorer" if mode == "explore" else "mission"
            module = "wc_navigation.autonomy." + ("explorer_node" if mode == "explore" else "mission_node")
            process = self.launch_python(task, module, files["task"])
            sim_start = recorder.now()
            pause = cfg.get("enable_pause")                     # window B: validation option --enable-pause
            if pause:
                recorder.enable_pause = (sim_start + pause["after_s"], sim_start + pause["after_s"] + pause["duration_s"])
                self.report["enable_pause_sim_s"] = [round(t, 3) for t in recorder.enable_pause]
            while process.poll() is None:
                if time.monotonic() - self.t0 > wall_budget:
                    raise RunError("wall-clock budget exceeded")
                self.check_alive(allow=(task,))
                recorder.spin(0.2)
            task_status = process.returncode
            self.report["stages"].append({"stage": task + "_finished", "exit_code": task_status,
                                          "wall_s": round(time.monotonic() - self.t0, 2),
                                          "sim_task_s": round(recorder.now() - sim_start, 3)})
            recorder.enabled = False
            recorder.spin(1.0)
        except Exception as exc:
            self.report["error"] = type(exc).__name__ + ": " + str(exc)
        finally:
            if recorder is not None:
                recorder.enabled = False
                try:
                    recorder.spin(0.5)
                except Exception:
                    pass
            self.stop_all()
            if recorder is not None:
                self.report["recorded"] = {"rows": recorder.rows, "events": recorder.events,
                                           "final_gate_status": recorder.gate_status,
                                           "final_sim_status": recorder.sim_status,
                                           "plans": recorder.plans}
                recorder.close()
            if context.ok():
                context.shutdown()
        self.report["task_exit_code"] = task_status
        self.report["logs"] = self.logs
        self.report["process_args"] = self.commands
        self.report["wall_s"] = round(time.monotonic() - self.t0, 2)
        return self.report


def world_pose(world_T_map, map_pose):
    return compose(world_T_map, map_pose)


def map_pose(world_T_map, world_pose_value):
    return compose(inverse(world_T_map), world_pose_value)
