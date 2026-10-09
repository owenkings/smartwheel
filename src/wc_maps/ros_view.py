"""ROS 2 read-only publisher for an integrity-verified saved map version.

No TF broadcaster, driver, command-velocity publisher, navigation client or
goal subscriber exists here. Goal annotation changes remain an explicit CLI
operation. All map timestamps are zero: persisted geometry carries no claim
of current robot position or a new sensor measurement timestamp.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sqlite3
import stat
import sys
import textwrap

import numpy as np

from .builder import MapError, identity, rigid
from .store import MapStore, _no_symlinks


def read_ascii_ply(path, expected_revision: int, max_points: int = 5000000) -> np.ndarray:
    """Read only the precise PLY format actually emitted by MapStore."""
    path = _no_symlinks(Path(path))
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_NONBLOCK", 0))
    with os.fdopen(descriptor, "r", encoding="ascii") as stream:
        if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
            raise MapError("PLY path must be a regular file")
        if stream.readline().strip() != "ply" or stream.readline().strip() != "format ascii 1.0":
            raise MapError("viewer supports the map store's ASCII PLY format only")
        count = None
        revision = None
        properties = []
        for _ in range(20):
            line = stream.readline().strip()
            if line == "end_header":
                break
            if line.startswith("comment graph_revision "):
                revision = int(line.split()[-1])
            elif line.startswith("element vertex "):
                count = int(line.split()[-1])
            elif line.startswith("property "):
                properties.append(line)
            else:
                raise MapError("unexpected PLY header")
        else:
            raise MapError("PLY header exceeds expected format")
        if revision != expected_revision or count is None or count < 1 or count > max_points:
            raise MapError("PLY revision/point count is invalid")
        if properties != ["property double x", "property double y", "property double z"]:
            raise MapError("PLY fields differ from the saved xyz contract")
        points = np.empty((count, 3), dtype=np.float32)
        for index in range(count):
            fields = stream.readline().split()
            if len(fields) != 3:
                raise MapError("PLY point count/field count mismatch")
            points[index] = [float(value) for value in fields]
        if stream.read().strip() or not np.isfinite(points).all():
            raise MapError("PLY has extra or nonfinite point data")
        return points


def quaternion_xyzw(matrix) -> list[float]:
    rotation = rigid(matrix, "trajectory pose")[:3, :3]
    trace = float(np.trace(rotation))
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        result = [(rotation[2, 1] - rotation[1, 2]) / s, (rotation[0, 2] - rotation[2, 0]) / s,
                  (rotation[1, 0] - rotation[0, 1]) / s, s / 4]
    else:
        axis = int(np.argmax(np.diag(rotation)))
        if axis == 0:
            s = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
            result = [s / 4, (rotation[0, 1] + rotation[1, 0]) / s,
                      (rotation[0, 2] + rotation[2, 0]) / s, (rotation[2, 1] - rotation[1, 2]) / s]
        elif axis == 1:
            s = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
            result = [(rotation[0, 1] + rotation[1, 0]) / s, s / 4,
                      (rotation[1, 2] + rotation[2, 1]) / s, (rotation[0, 2] - rotation[2, 0]) / s]
        else:
            s = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
            result = [(rotation[0, 2] + rotation[2, 0]) / s, (rotation[1, 2] + rotation[2, 1]) / s,
                      s / 4, (rotation[1, 0] - rotation[0, 1]) / s]
    quaternion = np.asarray(result)
    quaternion /= np.linalg.norm(quaternion)
    return quaternion.tolist()


def trajectory_segments(graph: dict, trajectory: dict) -> list[list[dict]]:
    """Do not join poses across an odometry epoch boundary."""
    epochs = {identity(node["node_id"], "node_id"): node["odom_epoch"] for node in graph["nodes"]}
    segments: list[list[dict]] = []
    previous = None
    for pose in trajectory["poses"]:
        key = identity(pose["node_id"], "trajectory node_id")
        if key not in epochs:
            raise MapError("trajectory references an unknown node")
        current = epochs[key]
        if not segments or current != previous:
            segments.append([])
        segments[-1].append(pose)
        previous = current
    return segments


def view_payload(store: MapStore, map_id: str, version: str) -> dict:
    loaded = store.load(map_id, version)
    loaded["points"] = read_ascii_ply(loaded["cloud_path"], loaded["manifest"]["graph_revision"])
    loaded["trajectory_segments"] = trajectory_segments(loaded["graph"], loaded["trajectory"])
    return loaded


def create_viewer_node(store: MapStore, map_id: str, version: str,
                       namespace: str = "/wc_mapping", context=None):
    # Deferred imports keep all offline map operations independent of ROS.
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Point, PoseStamped
    from nav_msgs.msg import OccupancyGrid, Path as RosPath
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from visualization_msgs.msg import Marker, MarkerArray
    from wc_interfaces.msg import MapStatus

    payload = view_payload(store, map_id, version)

    class SavedMapViewer(Node):
        def __init__(self):
            super().__init__("saved_map_viewer", namespace=namespace, context=context)
            self.payload = payload
            self.snapshot_messages = {}
            self.snapshot_publishers = {}
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            manifest = payload["manifest"]
            frame = manifest["frame_id"]

            def publish(topic, message_type, message):
                publisher = self.create_publisher(message_type, topic, qos)
                self.snapshot_publishers[topic] = publisher
                self.snapshot_messages[topic] = message
                publisher.publish(message)

            cloud = PointCloud2()
            cloud.header.frame_id = frame
            cloud.height = 1
            cloud.width = len(payload["points"])
            cloud.fields = [PointField(name=name, offset=4 * index, datatype=PointField.FLOAT32, count=1)
                            for index, name in enumerate(("x", "y", "z"))]
            cloud.is_bigendian = False
            cloud.point_step = 12
            cloud.row_step = cloud.width * cloud.point_step
            cloud.is_dense = True
            cloud.data = payload["points"].astype("<f4", copy=False).tobytes()
            publish("map/cloud", PointCloud2, cloud)

            stored_grid = payload["grid"]
            grid = OccupancyGrid()
            grid.header.frame_id = frame
            grid.info.resolution = float(stored_grid["resolution"])
            grid.info.width = stored_grid["width"]
            grid.info.height = stored_grid["height"]
            grid.info.origin.position.x = float(stored_grid["origin"][0])
            grid.info.origin.position.y = float(stored_grid["origin"][1])
            grid.info.origin.orientation.w = 1.0
            grid.data = stored_grid["data"]
            publish("map/grid", OccupancyGrid, grid)

            def pose_message(record):
                pose = PoseStamped()
                pose.header.frame_id = frame
                matrix = np.asarray(record["T_map_rig"])
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (float(value) for value in matrix[:3, 3])
                q = quaternion_xyzw(record["T_map_rig"])
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
                return pose

            segments = payload["trajectory_segments"]
            path = RosPath()
            path.header.frame_id = frame
            path.poses = [pose_message(record) for record in segments[-1]]
            publish("map/path", RosPath, path)

            segment_markers = MarkerArray()
            for index, segment in enumerate(segments):
                marker = Marker()
                marker.header.frame_id = frame
                marker.ns = "rig_trajectory_segments"
                marker.id = index
                marker.type = Marker.LINE_STRIP if len(segment) > 1 else Marker.POINTS
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.04
                marker.scale.y = 0.04
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.65, 0.0, 1.0
                for record in segment:
                    xyz = np.asarray(record["T_map_rig"])[:3, 3]
                    marker.points.append(Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])))
                segment_markers.markers.append(marker)
            publish("map/path_segments", MarkerArray, segment_markers)

            status = MapStatus()
            status.header.frame_id = frame
            status.session_id = manifest["snapshot"]["session_id"]
            status.source_mode = manifest["source_mode"]
            status.sensor_mode = manifest["sensor_mode"]
            status.state = "LOADED_VIEW_ONLY"
            status.left_fresh = status.right_fresh = False
            status.calibration_valid = status.time_valid = status.tracking_valid = False
            status.pause_reasons = ["VIEW_ONLY_NO_CURRENT_LOCALIZATION", "SAVED_GEOMETRY_TIMESTAMPS_UNSPECIFIED"]
            if len(segments) > 1:
                status.pause_reasons.append("PATH_TOPIC_LAST_CONTIGUOUS_EPOCH_FULL_HISTORY_IN_PATH_SEGMENTS")
            status.latest_accepted_time_ns = 0
            status.graph_epoch = str(manifest["snapshot"]["graph_epoch"])
            status.graph_revision = manifest["graph_revision"]
            status.completed_map_revision = manifest["completed_map_revision"]
            status.rebuilding = False
            status.consistent_snapshot = True
            status.map_id = map_id
            status.map_version = version
            status.navigation_validated = False
            publish("status", MapStatus, status)

            diagnostics = DiagnosticArray()
            diagnostics.header.frame_id = frame
            detail = DiagnosticStatus()
            detail.level = DiagnosticStatus.WARN
            detail.name = "saved_map_viewer"
            detail.hardware_id = "none_view_only"
            detail.message = "Saved map verified; no current localization or navigation validation"
            values = {"state": status.state, "map_id": map_id, "version": version,
                      "source_mode": status.source_mode, "sensor_mode": status.sensor_mode,
                      "graph_revision": status.graph_revision, "frame_id": frame,
                      "navigation_validated": False, "current_pose": None,
                      "historical_sensor_ids": payload["quality"].get("sensor_ids"),
                      "historical_source_point_counts": payload["quality"].get("source_point_counts"),
                      "trajectory_segment_count": len(segments), "timestamp_policy": "zero_unspecified_saved_geometry"}
            detail.values = [KeyValue(key=key, value=json.dumps(value, ensure_ascii=False)) for key, value in values.items()]
            diagnostics.status = [detail]
            publish("diagnostics", DiagnosticArray, diagnostics)

            status_marker = Marker()
            status_marker.header.frame_id = frame
            status_marker.ns = "saved_map_status"
            status_marker.id = 0
            status_marker.type = Marker.TEXT_VIEW_FACING
            status_marker.action = Marker.ADD
            status_marker.pose.orientation.w = 1.0
            status_marker.pose.position.x = float(stored_grid["origin"][0]) + stored_grid['width']*stored_grid['resolution']/2
            status_marker.pose.position.y = float(stored_grid["origin"][1]) + stored_grid['height']*stored_grid['resolution'] + 0.7
            status_marker.pose.position.z = 2.5
            status_marker.scale.z = 0.18
            status_marker.color.r, status_marker.color.g, status_marker.color.b, status_marker.color.a = 1.0, 0.8, 0.2, 1.0
            status_marker.text = (f"VIEW_ONLY\n{status.source_mode.upper()}_{status.sensor_mode.upper()}\n{version}_rev{status.graph_revision}\n"
                                  "NO_LOCALIZATION\nNAV_UNVERIFIED")
            publish("ui/status_marker", MarkerArray, MarkerArray(markers=[status_marker]))

            goal_markers = MarkerArray()
            for index, goal in enumerate(payload["goals"]["goals"]):
                marker = Marker()
                marker.header.frame_id = frame
                marker.ns = "version_bound_goal_annotations"
                marker.id = index
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (float(value) for value in goal["position"])
                marker.pose.position.z += 0.3
                marker.pose.orientation.w = 1.0
                marker.scale.z = 0.18
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.7, 0.2, 1.0
                marker.text = "\n".join(textwrap.wrap(goal['name'], width=18)) + f"\n[{goal['review_state']}]\n{version}"
                goal_markers.markers.append(marker)
            publish("ui/goal_markers", MarkerArray, goal_markers)
            self.get_logger().info(f"Loaded verified map {map_id}/{version} at revision {status.graph_revision}; view-only, no TF or current pose")

    return SavedMapViewer()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Publish an integrity-verified saved map to ROS 2; strictly view-only")
    parser.add_argument("--root", required=True)
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--version", required=True)
    parser.add_argument("--namespace", default="/wc_mapping")
    args, ros_args = parser.parse_known_args(argv)
    node = None
    initialized = False
    try:
        import rclpy
        rclpy.init(args=ros_args)
        initialized = True
        node = create_viewer_node(MapStore(args.root), args.map_id, args.version, args.namespace)
        rclpy.spin(node)
        return 0
    except KeyboardInterrupt:
        return 0
    except (ImportError, MapError, OSError, ValueError, KeyError, TypeError, sqlite3.Error) as exc:
        print(json.dumps({"status": "ERROR", "error": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
