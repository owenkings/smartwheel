"""Run the installed Nav2 global planner in a localhost-only software sandbox.

Only a map server and a planner server are launched.  There is no controller,
velocity publisher, hardware driver, or link to the wheelchair runtime here.
ROS imports are lazy so that geometry and report tools remain usable elsewhere.
"""
from __future__ import annotations

import math
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback
import uuid


class Nav2PlanningError(RuntimeError):
    """The isolated planner could not produce a usable path."""


def _xy(value, name):
    try:
        if isinstance(value, (str, bytes, dict)) or len(value) != 2:
            raise ValueError
        point = [float(value[0]), float(value[1])]
    except (TypeError, ValueError, IndexError) as exc:
        raise ValueError(f"{name} must contain exactly two finite coordinates") from exc
    if not all(math.isfinite(v) for v in point):
        raise ValueError(f"{name} must contain exactly two finite coordinates")
    return point


def _stop_owned(process):
    """Reap only a child process group created by this invocation."""
    if process.poll() is not None:
        return
    try:
        if os.getpgid(process.pid) != process.pid:
            return
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=4.0)
        except subprocess.TimeoutExpired:
            if process.poll() is None and os.getpgid(process.pid) == process.pid:
                os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=4.0)
    except ProcessLookupError:
        process.wait(timeout=1.0)


def plan_nav2(map_yaml, start_xy, goal_xy, radius_m, margin_m, output_dir,
              timeout_s=60):
    """Return a Navfn path without starting an executable navigation controller.

    Call from a fresh Linux process with Humble sourced, ``ROS_DOMAIN_ID=93``
    and ``ROS_LOCALHOST_ONLY=1``.  A foreign ROS node in that domain aborts the
    run before sandbox nodes are launched.  ``radius_m`` is an explicit offline
    circular-footprint assumption; it does not certify the wheelchair outline.

    The map must be axis aligned because Nav2's 2-D costmap does not rotate its
    grid. Unknown space is blocked and inflated. The caller must independently
    check the returned continuous path against its conservative occupancy model.
    No fallback reduces the footprint, clears unknown cells, or relaxes goals.
    """
    start = _xy(start_xy, "start_xy")
    goal = _xy(goal_xy, "goal_xy")
    try:
        radius, margin, timeout = float(radius_m), float(margin_m), float(timeout_s)
    except (TypeError, ValueError) as exc:
        raise ValueError("radius, margin and timeout must be finite numeric values") from exc
    if not all(math.isfinite(v) for v in (radius, margin, timeout)):
        raise ValueError("radius, margin and timeout must be finite")
    if radius <= 0 or margin < 0 or timeout < 5 or timeout > 300:
        raise ValueError("radius > 0, margin >= 0, and 5 <= timeout_s <= 300 required")
    if os.name != "posix":
        raise Nav2PlanningError("Nav2 sandbox requires the supported Linux target")
    if os.environ.get("ROS_DOMAIN_ID") != "93" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise Nav2PlanningError("Set ROS_DOMAIN_ID=93 and ROS_LOCALHOST_ONLY=1 in a fresh process")

    import yaml

    source = Path(map_yaml).expanduser().resolve(strict=True)
    meta = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(meta, dict):
        raise ValueError("map YAML must be a mapping")
    origin = meta.get("origin")
    if not isinstance(origin, (list, tuple)) or len(origin) != 3:
        raise ValueError("map origin must contain x, y and yaw")
    try:
        origin = [float(v) for v in origin]
        resolution = float(meta["resolution"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("map origin/resolution must be finite numeric values") from exc
    if not all(math.isfinite(v) for v in origin) or not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("map origin/resolution must be finite, with positive resolution")
    if abs(math.atan2(math.sin(origin[2]), math.cos(origin[2]))) > 1e-9:
        raise ValueError("Nav2 planner sandbox requires an axis-aligned map (origin yaw = 0)")
    directory = Path(output_dir).expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=True)
    namespace = "/wc_offline_nav_" + uuid.uuid4().hex[:12]
    map_frame = namespace[1:] + "/map"
    base_frame = namespace[1:] + "/base_link"
    map_topic = namespace + "/map"
    params_file = directory / "nav2_planner_params.yaml"
    if params_file.exists():
        raise FileExistsError(f"Refusing to overwrite sandbox parameters: {params_file}")

    # Add a full grid diagonal to the stated envelope for sub-cell geometry.
    # The pure geometry checker is the final acceptance gate, independently of
    # Navfn's cell-centre and gradient interpolation behaviour.
    nav_radius = radius + margin + resolution * math.sqrt(2.0)
    parameters = {
        namespace + "/map_server": {"ros__parameters": {
            "use_sim_time": False, "yaml_filename": str(source),
            "topic_name": map_topic, "frame_id": map_frame,
            "bond_heartbeat_period": 0.0,
        }},
        namespace + "/planner_server": {"ros__parameters": {
            "use_sim_time": False, "expected_planner_frequency": 1.0,
            "planner_plugins": ["GridBased"], "bond_heartbeat_period": 0.0,
            "GridBased": {"plugin": "nav2_navfn_planner/NavfnPlanner",
                          "tolerance": 0.0, "use_astar": True,
                          "allow_unknown": False, "use_final_approach_orientation": True},
        }},
        namespace + "/global_costmap/global_costmap": {"ros__parameters": {
            "use_sim_time": False, "global_frame": map_frame,
            "robot_base_frame": base_frame, "map_topic": map_topic,
            "resolution": resolution, "robot_radius": nav_radius,
            "footprint_padding": 0.0, "rolling_window": False,
            "track_unknown_space": True, "unknown_cost_value": 255,
            "update_frequency": 2.0, "publish_frequency": 2.0,
            "always_send_full_costmap": True, "transform_tolerance": 0.5,
            "plugins": ["static_layer", "inflation_layer"],
            "static_layer": {"plugin": "nav2_costmap_2d::StaticLayer",
                             "map_topic": map_topic, "enabled": True,
                             "map_subscribe_transient_local": True},
            "inflation_layer": {"plugin": "nav2_costmap_2d::InflationLayer",
                                "enabled": True, "cost_scaling_factor": 3.0,
                                "inflation_radius": nav_radius + 0.20,
                                "inflate_unknown": True, "inflate_around_unknown": True},
        }},
    }

    # Import only after argument/environment checks, before creating ROS state.
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from nav2_msgs.action import ComputePathToPose
    from nav_msgs.msg import OccupancyGrid
    from action_msgs.msg import GoalStatus
    from tf2_ros import StaticTransformBroadcaster

    bins = {"map_server": Path("/opt/ros/humble/lib/nav2_map_server/map_server"),
            "planner_server": Path("/opt/ros/humble/lib/nav2_planner/planner_server")}
    for executable in bins.values():
        if not executable.is_file():
            raise Nav2PlanningError(f"Installed Nav2 executable missing: {executable}")
    versions = subprocess.run(
        ["dpkg-query", "-W", "-f=${Package} ${Version}\n", "ros-humble-nav2-map-server",
         "ros-humble-nav2-planner", "ros-humble-nav2-navfn-planner"],
        capture_output=True, text=True, timeout=5.0,
    )
    if versions.returncode:
        raise Nav2PlanningError("Cannot record installed Nav2 package versions: " + versions.stderr[:500])

    context = Context()
    node = None
    executor = None
    children = []
    logs = {}
    handles = []
    commands = {}
    deadline = time.monotonic() + timeout
    remaps = ["--ros-args", "-r", "__ns:=" + namespace,
              "-r", "/tf:=" + namespace + "/tf",
              "-r", "/tf_static:=" + namespace + "/tf_static"]

    def healthy():
        if time.monotonic() >= deadline:
            raise Nav2PlanningError(f"Nav2 planning deadline exceeded; logs: {directory}")
        for name, process in children:
            code = process.poll()
            if code is not None:
                raise Nav2PlanningError(f"{name} exited with {code}; log: {logs[name]}")

    def spin_until(predicate):
        while not predicate():
            healthy()
            executor.spin_once(timeout_sec=min(0.1, max(0.001, deadline - time.monotonic())))

    def future_result(future):
        spin_until(future.done)
        error = future.exception()
        if error is not None:
            raise Nav2PlanningError(str(error)) from error
        return future.result()

    def lifecycle(name, transition_id):
        client = node.create_client(ChangeState, namespace + "/" + name + "/change_state")
        try:
            while not client.wait_for_service(timeout_sec=0.2):
                healthy()
            request = ChangeState.Request()
            request.transition.id = transition_id
            result = future_result(client.call_async(request))
            if not result.success:
                raise Nav2PlanningError(f"Lifecycle transition {transition_id} failed for {name}; logs: {directory}")
        finally:
            node.destroy_client(client)

    try:
        rclpy.init(args=[], context=context)
        node = rclpy.create_node("offline_planning_client", namespace=namespace,
                                context=context, enable_rosout=False,
                                start_parameter_services=False,
                                cli_args=remaps, use_global_arguments=False)
        # The global rclpy.spin_once helper uses the default context. This node
        # deliberately owns a private context, so its executor must match it.
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        # Give DDS discovery a bounded window before publishing TF or creating
        # either lifecycle server. The domain is dedicated to this one run.
        discovery_end = min(deadline, time.monotonic() + 2.0)
        while time.monotonic() < discovery_end:
            executor.spin_once(timeout_sec=0.1)
            foreign = [(name, ns) for name, ns in node.get_node_names_and_namespaces()
                       if (name, ns) != (node.get_name(), node.get_namespace())]
            if foreign:
                raise Nav2PlanningError(f"ROS domain 93 is already in use: {foreign}")
        healthy()
        params_file.write_text(yaml.safe_dump(parameters, sort_keys=False), encoding="utf-8")
        transform = TransformStamped()
        transform.header.frame_id = map_frame
        transform.child_frame_id = base_frame
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.transform.translation.x = start[0]
        transform.transform.translation.y = start[1]
        transform.transform.rotation.w = 1.0
        broadcaster = StaticTransformBroadcaster(node)
        broadcaster.sendTransform(transform)

        received = {}
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        map_sub = node.create_subscription(OccupancyGrid, map_topic,
                                           lambda message: received.__setitem__("map", message), map_qos)
        costmap_sub = node.create_subscription(OccupancyGrid, namespace + "/global_costmap/costmap",
                                               lambda message: received.__setitem__("costmap", message), 1)
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = "93"
        env["ROS_LOCALHOST_ONLY"] = "1"
        env["PYTHONNOUSERSITE"] = "1"
        env["PYTHONDONTWRITEBYTECODE"] = "1"
        env["ROS_LOG_DIR"] = str(directory / "ros_logs")
        for name in ("map_server", "planner_server"):
            log = directory / (name + ".log")
            if log.exists():
                raise FileExistsError(f"Refusing to overwrite log: {log}")
            handle = log.open("x", encoding="utf-8")
            handles.append(handle)
            args = [str(bins[name]), *remaps, "--params-file", str(params_file)]
            commands[name] = args
            logs[name] = str(log)
            process = subprocess.Popen(args, stdout=handle, stderr=subprocess.STDOUT,
                                       env=env, start_new_session=True)
            children.append((name, process))
            lifecycle(name, Transition.TRANSITION_CONFIGURE)
            lifecycle(name, Transition.TRANSITION_ACTIVATE)
            if name == "map_server":
                spin_until(lambda: "map" in received)

        def costmap_ready():
            if "costmap" not in received:
                return False
            a, b = received["map"], received["costmap"]
            return (b.header.frame_id == map_frame and a.info.width == b.info.width
                    and a.info.height == b.info.height and b.info.width > 0
                    and len(b.data) == b.info.width * b.info.height
                    and abs(a.info.resolution - b.info.resolution) < 1e-6)

        spin_until(costmap_ready)
        client = ActionClient(node, ComputePathToPose, namespace + "/compute_path_to_pose")
        while not client.wait_for_server(timeout_sec=0.2):
            healthy()

        def pose(point):
            p = PoseStamped()
            p.header.frame_id = map_frame
            p.header.stamp = node.get_clock().now().to_msg()
            p.pose.position.x, p.pose.position.y = point
            p.pose.orientation.w = 1.0
            return p

        request = ComputePathToPose.Goal()
        request.start = pose(start)
        request.goal = pose(goal)
        request.use_start = True
        request.planner_id = "GridBased"
        goal_handle = future_result(client.send_goal_async(request))
        if not goal_handle.accepted:
            raise Nav2PlanningError("Nav2 rejected the planning request")
        response = future_result(goal_handle.get_result_async())
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            raise Nav2PlanningError(f"Nav2 did not find a path (action status {response.status}); logs: {directory}")
        path = response.result.path
        points = [[float(p.pose.position.x), float(p.pose.position.y)] for p in path.poses]
        if (path.header.frame_id != map_frame or not points
                or not all(math.isfinite(v) for p in points for v in p)):
            raise Nav2PlanningError("Nav2 returned an empty, nonfinite, or incorrectly framed path")
        if math.dist(points[0], start) > resolution * 2 or math.dist(points[-1], goal) > resolution * 2:
            raise Nav2PlanningError("Nav2 path endpoints do not match the requested start and goal")
        duration = response.result.planning_time
        return {"path": points, "planner_id": "GridBased", "algorithm": "Nav2 Navfn A*",
                "planning_time_s": float(duration.sec) + float(duration.nanosec) / 1e9,
                "namespace": namespace, "frame_id": map_frame, "base_frame": base_frame,
                "params_file": str(params_file), "logs": logs, "process_args": commands,
                "package_versions": versions.stdout.strip().splitlines(),
                "ros_domain_id": 93, "localhost_only": True,
                "mode": "OFFLINE_PLANNING_ONLY", "hardware_control": False,
                "footprint_assumed_radius_m": radius, "margin_m": margin,
                "nav2_radius_with_grid_allowance_m": nav_radius,
                "unknown_space_allowed": False, "independent_collision_check_required": True}
    except Exception as exc:
        trace_file = directory / "nav2_failure_traceback.log"
        try:
            with trace_file.open("x", encoding="utf-8") as trace:
                trace.write(traceback.format_exc())
        except OSError:
            # Preserve the original cause even if configured storage vanished.
            trace_file = None
        detail = f"; traceback: {trace_file}" if trace_file else ""
        raise Nav2PlanningError(f"Nav2 sandbox failed: {exc}; logs: {directory}{detail}") from exc
    finally:
        # No broad pkill, ROS-wide shutdown command, or external PID cleanup.
        for _, process in reversed(children):
            _stop_owned(process)
        for handle in handles:
            handle.close()
        if executor is not None:
            if node is not None:
                executor.remove_node(node)
            executor.shutdown(timeout_sec=1.0)
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
