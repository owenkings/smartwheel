"""Bounded ROS graph-revision rebuilding with original left/right ray origins."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import threading
import time

import numpy as np

from .builder import MapError
from .graph_snapshot import rebuild_message_revision
from .ros_view import quaternion_xyzw, trajectory_segments
from .store import _json, _no_symlinks


class LatestRebuildWorker:
    """One running rebuild, one pending request and one completed result.

    Pending requests coalesce, but a complete revision remains publishable
    while a newer full rebuild runs. Otherwise continuous input can starve map
    output forever. An unpolled error is retained for the owner to latch.
    """

    def __init__(self, build):
        self._build = build
        self._condition = threading.Condition()
        self._generation = 0
        self._pending = None
        self._result = None
        self._stopping = False
        self._thread = threading.Thread(target=self._run, name="wc-map-rebuild", daemon=True)
        self._thread.start()

    def submit(self, payload):
        with self._condition:
            self._generation += 1
            self._pending = (self._generation, payload)
            self._condition.notify()
            return self._generation

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._pending is not None or self._stopping)
                if self._stopping:
                    return
                token, payload = self._pending
                self._pending = None
            result, error = None, None
            try:
                result = self._build(payload)
            except Exception as failure:
                error = str(failure)
            with self._condition:
                if not self._stopping and (self._result is None or self._result[2] is None):
                    self._result = (token, result, error)

    def poll(self):
        with self._condition:
            result, self._result = self._result, None
            return result

    def close(self):
        with self._condition:
            self._stopping = True
            self._pending = self._result = None
            self._condition.notify()
        # A running rebuild is read-only. The daemon cannot keep a stopped ROS
        # process alive or publish a result after its node is destroyed.
        self._thread.join(timeout=5)
        return not self._thread.is_alive()


def create_live_map_node(session: dict, context=None):
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Point, PoseStamped
    from nav_msgs.msg import OccupancyGrid, Path as RosPath
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from std_srvs.srv import Trigger
    from visualization_msgs.msg import Marker, MarkerArray
    from wc_interfaces.msg import GraphSnapshot, MapStatus

    root = _no_symlinks(Path(session["session_root"]))
    config = session.get("map_config") or _json(root / "map_config.json")
    calibration = session.get("calibration") or _json(root / "calibration.json")
    if session.get("source_mode") not in ("synthetic", "real") or not session.get("session_id"):
        raise MapError("map coordinator requires an explicit source_mode/session_id")

    class LiveMapNode(Node):
        def __init__(self):
            super().__init__("wc_multi_origin_map", context=context)
            self.qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.map_publishers = {topic: self.create_publisher(message_type, "/wc_mapping/" + topic, self.qos)
                for topic, message_type in (("map/cloud", PointCloud2), ("map/grid", OccupancyGrid), ("map/path", RosPath),
                    ("map/path_segments", MarkerArray), ("ui/status_marker", MarkerArray), ("status", MapStatus),
                    ("map/diagnostics", DiagnosticArray))}
            self.frontend = None
            self.frontend_received = 0.0
            self.requested = None
            self.completed = None
            self.blocked_reason = None
            self.worker = LatestRebuildWorker(lambda message: rebuild_message_revision(root, message, config, calibration))
            self.create_subscription(GraphSnapshot, "/wc_mapping/graph/snapshot", self.graph, self.qos)
            self.create_subscription(MapStatus, "/wc_mapping/frontend/status", self.frontend_status, self.qos)
            self.pause_clients = [self.create_client(Trigger, path) for path in ("/wc_mapping/pause_graph", "/wc_mapping/pause_frontend")]
            self.timer = self.create_timer(0.1, self.tick)

        def frontend_status(self, message):
            if message.session_id == session["session_id"] and message.source_mode == session["source_mode"]:
                self.frontend = message
                self.frontend_received = time.monotonic()

        def fail(self, reason):
            if self.blocked_reason is None:
                self.blocked_reason = str(reason)
                self.get_logger().error(self.blocked_reason)
                for client in self.pause_clients:
                    if client.service_is_ready():
                        client.call_async(Trigger.Request())
            # The error is latched. Recovery requires a reviewed new coordinator
            # session; source reconnects do not silently restart map acceptance.

        def graph(self, message):
            if self.blocked_reason:
                return
            if message.session_id != session["session_id"] or message.source_mode != session["source_mode"]:
                self.fail("GraphSnapshot session/source mode mismatch")
                return
            if self.requested is not None:
                if message.graph_epoch != self.requested.graph_epoch:
                    self.fail("graph epoch changed without an explicit new map session")
                    return
                if message.revision <= self.requested.revision:
                    if message.revision == self.requested.revision:
                        return  # Durable duplicate of the same graph snapshot.
                    self.fail("graph revision moved backwards")
                    return
            if not message.raw_index_complete:
                self.fail("GraphSnapshot raw index is incomplete")
                return
            self.requested = message
            self.worker.submit(message)

        def publish_geometry(self, assembled):
            graph, mapped = assembled["graph"], assembled["mapped"]
            frame = graph["frame_id"]
            sec, nanosec = divmod(graph["stamp_ns"], 1000000000)

            def header(message):
                message.header.frame_id = frame
                message.header.stamp.sec, message.header.stamp.nanosec = sec, nanosec

            cloud = PointCloud2(); header(cloud)
            points = np.asarray(mapped["cloud"], dtype="<f4")
            if not np.isfinite(points).all():
                raise MapError("map cloud exceeds float32 publication range")
            cloud.height, cloud.width = 1, len(points)
            cloud.fields = [PointField(name=name, offset=4*index, datatype=PointField.FLOAT32, count=1) for index, name in enumerate("xyz")]
            cloud.point_step, cloud.row_step, cloud.is_dense = 12, 12*len(points), True
            cloud.data = points.tobytes()
            grid = OccupancyGrid(); header(grid)
            grid.info.resolution = float(mapped["grid"]["resolution"])
            grid.info.width, grid.info.height = mapped["grid"]["width"], mapped["grid"]["height"]
            grid.info.origin.position.x, grid.info.origin.position.y = (float(value) for value in mapped["grid"]["origin"][:2])
            grid.info.origin.orientation.w = 1.0
            grid.data = mapped["grid"]["data"]
            segments = trajectory_segments(graph, assembled["trajectory"])
            path = RosPath(); header(path)
            for record in segments[-1]:
                pose = PoseStamped(); header(pose)
                transform = np.asarray(record["T_map_rig"])
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (float(v) for v in transform[:3, 3])
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion_xyzw(transform)
                path.poses.append(pose)
            deletion = Marker(); deletion.action = Marker.DELETEALL
            markers = MarkerArray(markers=[deletion])
            for index, segment in enumerate(segments):
                marker = Marker(); header(marker)
                marker.ns, marker.id = "rig_trajectory_segments", index
                marker.type = Marker.LINE_STRIP if len(segment) > 1 else Marker.POINTS
                marker.action = Marker.ADD; marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = 0.04
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.65, 0.0, 1.0
                for record in segment:
                    xyz = np.asarray(record["T_map_rig"])[:3, 3]
                    marker.points.append(Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])))
                markers.markers.append(marker)
            # All messages are fully constructed before any new revision is
            # published. Status marks completed only after this consistent set.
            self.map_publishers["map/cloud"].publish(cloud)
            self.map_publishers["map/grid"].publish(grid)
            self.map_publishers["map/path"].publish(path)
            self.map_publishers["map/path_segments"].publish(markers)

        def tick(self):
            finished = self.worker.poll()
            if finished is not None and not self.blocked_reason:
                _, assembled, error = finished
                if error:
                    self.fail(error)
                else:
                    try:
                        # Publish one fully reconstructed revision as a unit,
                        # even when a newer graph request is already pending.
                        # Status separately declares the newer requested graph.
                        graph = assembled["graph"]
                        if (graph["session_id"] != session["session_id"] or graph["source_mode"] != session["source_mode"]
                                or graph["graph_epoch"] != self.requested.graph_epoch
                                or graph["revision"] > self.requested.revision):
                            raise MapError("completed map belongs to a different session/epoch or an unrequested graph revision")
                        if self.completed is None or graph["revision"] > self.completed["graph"]["revision"]:
                            self.publish_geometry(assembled)
                            self.completed = assembled
                    except Exception as failure:
                        self.fail(str(failure))
            status = MapStatus()
            status.header.stamp = self.get_clock().now().to_msg()
            status.header.frame_id = self.requested.header.frame_id if self.requested else session.get("graph", {}).get("map_frame", "map")
            status.session_id, status.source_mode, status.sensor_mode = session["session_id"], session["source_mode"], "dual"
            status.graph_epoch = self.requested.graph_epoch if self.requested else str(session.get("graph_epoch", ""))
            status.graph_revision = self.requested.revision if self.requested else 0
            status.completed_map_revision = self.completed["graph"]["revision"] if self.completed else 0
            status.consistent_snapshot = self.completed is not None and status.graph_revision == status.completed_map_revision
            status.rebuilding = self.requested is not None and not status.consistent_snapshot and not self.blocked_reason
            map_behind_graph = self.completed is not None and status.completed_map_revision < status.graph_revision
            fresh_status = self.frontend is not None and time.monotonic() - self.frontend_received < 1.0
            if fresh_status:
                for field in ("left_fresh", "right_fresh", "calibration_valid", "time_valid", "tracking_valid", "latest_accepted_time_ns"):
                    setattr(status, field, getattr(self.frontend, field))
                status.pause_reasons = list(self.frontend.pause_reasons)
            else:
                status.pause_reasons = ["FRONTEND_STATUS_UNAVAILABLE_OR_STALE"]
            if fresh_status and self.requested is not None:
                graph_time_ns = self.requested.header.stamp.sec*1000000000 + self.requested.header.stamp.nanosec
                if status.latest_accepted_time_ns > graph_time_ns:
                    status.pause_reasons.append('GRAPH_BEHIND_ACCEPTED_FRONTEND')
                for field in ("left_fresh", "right_fresh", "calibration_valid", "time_valid", "tracking_valid"):
                    if not getattr(status, field):
                        status.pause_reasons.append("FRONTEND_" + field.upper() + "_FALSE")
            if self.blocked_reason:
                status.pause_reasons.append(self.blocked_reason)
            if not config.get("ground_valid", False):
                status.pause_reasons.append("GROUND_REFERENCE_UNVALIDATED")
            # Lag is visible without asserting that the older complete set is
            # the newest graph. It is a rebuild condition, not an input fault.
            invalid_reasons = bool(status.pause_reasons)
            if map_behind_graph:
                status.pause_reasons.append("MAP_BEHIND_GRAPH")
            if invalid_reasons:
                status.state = "PAUSED_INVALID"
            elif status.rebuilding:
                status.state = "REBUILDING"
            elif status.consistent_snapshot:
                status.state = "MAPPING_DUAL"
            else:
                status.state = "PREFLIGHT"
            status.navigation_validated = False
            self.map_publishers["status"].publish(status)
            marker = Marker(); marker.header = status.header
            marker.ns, marker.id, marker.type, marker.action = "map_revision_status", 0, Marker.TEXT_VIEW_FACING, Marker.ADD
            marker.pose.orientation.w = 1.0; marker.pose.position.z = 2.5; marker.scale.z = 0.3
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.8, 0.2, 1.0
            marker.text = (f"{status.state} source={status.source_mode} sensors=dual\n"
                           f"graph={status.graph_revision} completed={status.completed_map_revision}\n"
                           + "; ".join(status.pause_reasons) + "\nnavigation_validated=false")
            self.map_publishers["ui/status_marker"].publish(MarkerArray(markers=[marker]))
            diagnostic = DiagnosticStatus(name="multi_origin_map", hardware_id="software_map_only",
                level=DiagnosticStatus.ERROR if self.blocked_reason else DiagnosticStatus.WARN if status.pause_reasons else DiagnosticStatus.OK,
                message=status.state)
            diagnostic.values = [KeyValue(key="graph_revision", value=str(status.graph_revision)),
                                 KeyValue(key="completed_map_revision", value=str(status.completed_map_revision)),
                                 KeyValue(key="source_mode", value=status.source_mode), KeyValue(key="reasons", value=json.dumps(status.pause_reasons))]
            self.map_publishers["map/diagnostics"].publish(DiagnosticArray(header=status.header, status=[diagnostic]))

        def destroy_node(self):
            self.worker.close()
            return super().destroy_node()

    return LiveMapNode()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-config", required=True)
    args, ros_args = parser.parse_known_args(argv)
    import rclpy
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_live_map_node(_json(Path(args.session_config)))
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
