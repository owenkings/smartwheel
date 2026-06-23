"""Adapt the XT-M60 left point cloud into a FAST-LIO2-friendly PointCloud2.

XT-M60 is a whole-frame flash-ToF snapshot: standard sensor_msgs/PointCloud2
with x/y/z/intensity, NO per-point time and NO ring field, ~10 Hz, in the
xtm60_left_link sensor frame. FAST-LIO2 consumes it through the generic
Velodyne/Ouster XYZI path with deskew turned off.

This node:
  * subscribes to /xtm60/left/points (BEST_EFFORT sensor_data QoS),
  * drops invalid / placeholder points (NaN and the (0,0,0) origin) by reusing
    cloud_utils.read_xyz_intensity (NaN-skip) + cloud_utils.filter_by_range
    (r > 0), and
  * republishes /lio/cloud_in (XYZI, frame_id preserved = xtm60_left_link).

Optional behaviour controlled by parameters:
  * restamp_to_now=true   -> overwrite header.stamp with the current time so the
                             cloud shares the IMU wall-clock time base (req 2.6).
  * add_zero_time_field   -> append a float32 'time'=0 field per point for
                             FAST-LIO parsers that strictly require it (req 2.3).
"""
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    qos_profile_sensor_data,
)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from wheelchair_3d_mapping import cloud_utils


class LioCloudAdapterNode(Node):
    def __init__(self):
        super().__init__("lio_cloud_adapter_node")
        self.declare_parameter("input_topic", "/xtm60/left/points")
        self.declare_parameter("output_topic", "/lio/cloud_in")
        # When true, overwrite header.stamp with "now" to align the cloud time
        # base with the IMU wall clock if FAST-LIO rejects the source stamps.
        self.declare_parameter("restamp_to_now", False)
        # When true, append a per-point float32 'time'=0 field so FAST-LIO
        # parsers that strictly require a time field are satisfied. The single
        # whole-frame timestamp semantics are preserved (no deskew).
        self.declare_parameter("add_zero_time_field", False)
        # Near-clip used together with the (0,0,0)/NaN rejection.
        self.declare_parameter("min_range", 0.0)
        self.declare_parameter("max_range", 1000.0)
        # Output QoS reliability: "reliable" (FAST-LIO default expectation) or
        # "best_effort" to mirror the sensor stream.
        self.declare_parameter("output_qos", "reliable")

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.restamp_to_now = bool(self.get_parameter("restamp_to_now").value)
        self.add_zero_time_field = bool(self.get_parameter("add_zero_time_field").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)

        pub_qos = self._resolve_output_qos(self.get_parameter("output_qos").value)

        self.pub = self.create_publisher(PointCloud2, self.output_topic, pub_qos)
        self.create_subscription(
            PointCloud2, self.input_topic, self._on_cloud, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"lio_cloud_adapter: in={self.input_topic} out={self.output_topic} "
            f"restamp_to_now={self.restamp_to_now} "
            f"add_zero_time_field={self.add_zero_time_field}"
        )

    @staticmethod
    def _resolve_output_qos(name):
        if str(name).lower() == "best_effort":
            return qos_profile_sensor_data
        return QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

    def adapt(self, msg: PointCloud2) -> PointCloud2:
        """Pure transform: read, clean, restamp and rebuild the output cloud.

        Exposed separately from the subscription callback so it is trivially
        unit-testable without a running ROS graph.
        """
        xyz, inten = cloud_utils.read_xyz_intensity(msg)
        # Drop (0,0,0) placeholders and any non-finite points (r > 0 term).
        xyz, inten = cloud_utils.filter_by_range(
            xyz, inten, self.min_range, self.max_range
        )

        header = msg.header
        if self.restamp_to_now:
            # Copy so we never mutate the incoming message's header.
            from std_msgs.msg import Header

            new_header = Header()
            new_header.frame_id = msg.header.frame_id
            new_header.stamp = self.get_clock().now().to_msg()
            header = new_header

        if self.add_zero_time_field:
            # FAST-LIO velodyne path (lidar_type=2) needs both 'ring' and 'time'.
            # ring=0 (valid index) + per-point time ramp 0..0.1s so sync_packages
            # gives a full 10Hz IMU window per frame (else IMU isn't integrated and
            # yaw won't track). Config uses timestamp_unit=0 (seconds). (Task 7 B.)
            return cloud_utils.make_velodyne_cloud(header, xyz, inten,
                                                   ring=None, sweep_time=0.1)
        return cloud_utils.make_xyzi_cloud(header, xyz, inten)

    def _on_cloud(self, msg: PointCloud2):
        self.pub.publish(self.adapt(msg))


def main(args=None):
    rclpy.init(args=args)
    node = LioCloudAdapterNode()
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
