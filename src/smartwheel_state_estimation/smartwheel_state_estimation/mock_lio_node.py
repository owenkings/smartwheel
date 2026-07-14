import copy

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class MockLioNode(Node):
    def __init__(self) -> None:
        super().__init__("mock_lio")
        self.declare_parameter("input_topic", "/sim/ground_truth/odom")
        self.declare_parameter("output_topic", "/lio/odom")
        self.declare_parameter("path_topic", "/lio/path")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("max_path_poses", 5000)
        self._frame = str(self.get_parameter("odom_frame_id").value)
        self._max_path = int(self.get_parameter("max_path_poses").value)
        self._odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 20
        )
        self._path_pub = self.create_publisher(Path, str(self.get_parameter("path_topic").value), 5)
        self._path = Path()
        self._path.header.frame_id = self._frame
        self.create_subscription(
            Odometry, str(self.get_parameter("input_topic").value), self._on_ground_truth, 20
        )
        self.get_logger().warning(
            "mock-LIO copies synthetic ground truth; it does not represent FAST-LIO2 performance"
        )

    def _on_ground_truth(self, source: Odometry) -> None:
        odom = copy.deepcopy(source)
        odom.header.frame_id = self._frame
        odom.child_frame_id = "base_link"
        self._odom_pub.publish(odom)
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self._path.poses.append(pose)
        if len(self._path.poses) > self._max_path:
            self._path.poses = self._path.poses[-self._max_path :]
        self._path.header.stamp = odom.header.stamp
        self._path_pub.publish(self._path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockLioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

