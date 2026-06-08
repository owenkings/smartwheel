import math
from pathlib import Path

import yaml

import rclpy
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from wheelchair_navigation.semantic_keepout import GridGeometry, rasterize_keepout_zones
from wheelchair_navigation.semantic_map_store import default_semantic_map_path


def quaternion_to_yaw(orientation) -> float:
    siny = 2.0 * (
        float(orientation.w) * float(orientation.z)
        + float(orientation.x) * float(orientation.y)
    )
    cosy = 1.0 - 2.0 * (
        float(orientation.y) * float(orientation.y)
        + float(orientation.z) * float(orientation.z)
    )
    return math.atan2(siny, cosy)


def validate_no_go_zones(data):
    if not isinstance(data, dict):
        raise ValueError("semantic map root must be a mapping")
    zones = data.get("no_go_zones", [])
    if not isinstance(zones, list):
        raise ValueError("no_go_zones must be a list")
    for zone_index, zone in enumerate(zones):
        if not isinstance(zone, dict):
            raise ValueError(f"no_go_zones[{zone_index}] must be a mapping")
        polygon = zone.get("polygon")
        if not isinstance(polygon, list) or len(polygon) < 3:
            raise ValueError(
                f"no_go_zones[{zone_index}].polygon must contain at least 3 points"
            )
        for point_index, point in enumerate(polygon):
            if not isinstance(point, (list, tuple)) or len(point) < 2:
                raise ValueError(
                    f"no_go_zones[{zone_index}].polygon[{point_index}] "
                    "must contain x and y"
                )
            x = float(point[0])
            y = float(point[1])
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError(
                    f"no_go_zones[{zone_index}].polygon[{point_index}] "
                    "must contain finite coordinates"
                )
    return zones


class SemanticKeepoutNode(Node):
    def __init__(self):
        super().__init__("semantic_keepout_node")
        self.declare_parameter("semantic_map_path", default_semantic_map_path())
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("mask_topic", "/semantic_keepout_mask")
        self.declare_parameter("filter_info_topic", "/semantic_keepout_filter_info")
        self.declare_parameter("status_topic", "/semantic_keepout/status")
        self.declare_parameter("reload_period_sec", 1.0)
        self.declare_parameter("filter_target_frame", "odom")
        self.declare_parameter("require_filter_transform", True)

        self.semantic_map_path = Path(
            str(self.get_parameter("semantic_map_path").value)
        )
        self.mask_topic = str(self.get_parameter("mask_topic").value)
        self.filter_target_frame = str(
            self.get_parameter("filter_target_frame").value
        )
        self.require_filter_transform = bool(
            self.get_parameter("require_filter_transform").value
        )
        self._last_map = None
        self._last_file_signature = object()
        self._last_valid_zones = None
        self._configuration_valid = False
        self._last_error = ""
        self._last_mask_summary = None
        self._filter_info_published = False
        self._waiting_for_transform_logged = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.mask_pub = self.create_publisher(
            OccupancyGrid, self.mask_topic, latched_qos
        )
        self.info_pub = self.create_publisher(
            CostmapFilterInfo,
            str(self.get_parameter("filter_info_topic").value),
            latched_qos,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            latched_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self.on_map,
            latched_qos,
        )
        period = max(0.2, float(self.get_parameter("reload_period_sec").value))
        self.create_timer(period, self.reload_if_changed)

    def on_map(self, msg):
        self._last_map = msg
        self.publish_mask()

    def file_signature(self):
        try:
            stat = self.semantic_map_path.stat()
            return (stat.st_ino, stat.st_mtime_ns, stat.st_size)
        except OSError:
            return None

    def reload_if_changed(self):
        if (
            self.file_signature() != self._last_file_signature
            and self._last_map is not None
        ):
            self.publish_mask()
        elif self._last_mask_summary is not None and not self._filter_info_published:
            self.publish_filter_info_if_ready()

    def load_zones(self):
        self._last_file_signature = self.file_signature()
        try:
            with self.semantic_map_path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
            zones = validate_no_go_zones(data)
        except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
            self._configuration_valid = False
            self._last_error = str(exc)
            self.get_logger().error(f"failed to load semantic map: {exc}")
            return None
        self._last_valid_zones = list(zones)
        self._configuration_valid = True
        self._last_error = ""
        return self._last_valid_zones

    def publish_status(self, text):
        status = String()
        status.data = text
        self.status_pub.publish(status)

    def filter_transform_ready(self, mask_frame):
        if not self.require_filter_transform:
            return True
        target_frame = self.filter_target_frame.strip()
        if not target_frame or target_frame == mask_frame:
            return True
        return self.tf_buffer.can_transform(
            mask_frame,
            target_frame,
            Time(),
            timeout=Duration(seconds=0.0),
        )

    def publish_filter_info_if_ready(self):
        summary = self._last_mask_summary
        if summary is None or not self._configuration_valid:
            return False
        mask_frame = summary["frame_id"]
        if not self.filter_transform_ready(mask_frame):
            if not self._waiting_for_transform_logged:
                self.get_logger().info(
                    f"waiting for {self.filter_target_frame} -> {mask_frame} "
                    "before enabling keepout filter"
                )
                self._waiting_for_transform_logged = True
            self.publish_status(
                f"WAITING_FOR_TRANSFORM: {self.filter_target_frame}->{mask_frame}; "
                f"zones={summary['zones']} occupied_cells={summary['occupied_cells']}"
            )
            return False

        info = CostmapFilterInfo()
        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = mask_frame
        info.type = 0
        info.filter_mask_topic = self.mask_topic
        info.base = 0.0
        info.multiplier = 1.0
        self.info_pub.publish(info)
        self._filter_info_published = True
        self._waiting_for_transform_logged = False
        self.publish_status(
            f"READY: zones={summary['zones']} "
            f"occupied_cells={summary['occupied_cells']} "
            f"map={summary['width']}x{summary['height']}"
        )
        return True

    def publish_mask(self):
        source = self._last_map
        if source is None:
            return
        if self.file_signature() != self._last_file_signature:
            self.load_zones()
        if not self._configuration_valid:
            retained = self._last_valid_zones is not None
            self.publish_status(
                f"ERROR: semantic map unavailable; retained_previous={str(retained).lower()} "
                f"reason={self._last_error or 'configuration not loaded'}"
            )
            return
        geometry = GridGeometry(
            width=int(source.info.width),
            height=int(source.info.height),
            resolution=float(source.info.resolution),
            origin_x=float(source.info.origin.position.x),
            origin_y=float(source.info.origin.position.y),
            origin_yaw=quaternion_to_yaw(source.info.origin.orientation),
        )
        zones = self._last_valid_zones
        mask = OccupancyGrid()
        mask.header.stamp = self.get_clock().now().to_msg()
        mask.header.frame_id = source.header.frame_id or "map"
        mask.info = source.info
        mask.data = rasterize_keepout_zones(zones, geometry)
        self.mask_pub.publish(mask)
        occupied_cells = sum(1 for value in mask.data if value > 0)
        self._last_mask_summary = {
            "frame_id": mask.header.frame_id,
            "zones": len(zones),
            "occupied_cells": occupied_cells,
            "width": geometry.width,
            "height": geometry.height,
        }
        self._filter_info_published = False
        self.publish_filter_info_if_ready()


def main(args=None):
    rclpy.init(args=args)
    node = SemanticKeepoutNode()
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
