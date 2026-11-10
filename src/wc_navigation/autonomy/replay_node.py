"""Recording replay for a shadow run: clock, odometry, TF and recorded scans; no command input.

Publishes ``/clock``; ``/odom`` and ``odom->base_link`` with the recorded planar
axle pose, interpolated at 50 Hz; static ``map->odom`` as identity, because the
map was built in the recording's odometry frame (left out with the config key
``publish_map_odom: false`` when another node owns map->odom); static
``base_link->laser`` with the frozen lever arm; ``/scan`` with the prepared
161-bin scans at their recorded relative times; the recorded pose on
``/wc_shadow/recorded`` and status on ``/wc_shadow/status``. ROS arguments after
``--ros-args`` (for example remaps) go to ``rclpy.init``.

Until ``/wc_shadow/start`` the pose holds at the first window frame and that
frame's scan is repeated; after the last frame the last pose and scan are held
for ``tail_hold_s``. Repeated scans are counted in the status. Nothing here
subscribes to a velocity command, so the recorded motion never reacts to Nav2.

Only when the recording carries obstacle clouds (shadow ``--obstacle-input``
cloud or scan_and_cloud), each lidar's filtered cloud is also published on
``/obstacles/<side>/points`` and its clearing rays on ``/obstacles/<side>/clearing``,
in frame ``obstacles_lidar_<side>`` at its own capture time, with static
``base_link->obstacles_lidar_<side>`` at the frozen 3D lever arm; held frames repeat the
first or last cloud like the scan. With ``--obstacle-scan filtered`` the left
lidar's filtered 161-bin scan goes out on ``/obstacles/left/scan`` in ``laser``
with each left cloud.

With ``lidar_time_offset_s`` in the recording (``--lidar-time-offset-ms``) every
lidar message (/scan, clouds, clearing rays, filtered scan) is stamped that much
earlier than its frame, so tf pairs it with the recorded pose of stamp - offset;
it still goes out when its frame is due. Odometry and tf are unchanged.
"""
from __future__ import annotations

import argparse

import numpy as np

from ..perception import shadow_obstacles
from ..perception.nav2_obstacles import CLEAR_TOPICS, CLOUD_FRAMES, CLOUD_TOPICS, FILTERED_SCAN_TOPIC, pointcloud2
from .ros_util import json_message, latched_qos, load_config, require_isolated_domain, yaw_to_quaternion
from .shadow import interpolate_poses, segment_twist
from .sim_node import _time_msg


def window_frames(recording, window):
    """Indices of the prepared frames inside ``window`` (recording seconds)."""
    times = recording["times_s"]
    index = np.nonzero((times >= window[0] - 1e-9) & (times <= window[1] + 1e-9))[0]
    if len(index) < 2:
        raise ValueError("the replay window holds fewer than two recorded frames")
    return index


class Replay:
    def __init__(self, node, config):
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan
        from std_srvs.srv import Trigger
        from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
        from std_msgs.msg import String
        self.node = node
        with np.load(config["recording"]) as data:
            recording = {key: data[key] for key in data.files}
        index = window_frames(recording, config["window_s"])
        self.times = recording["times_s"][index]
        self.poses = recording["base_poses"][index]
        self.ranges = recording["ranges"][index]
        self.v, self.w = segment_twist(self.times, self.poses)
        self.angle_min = float(recording["angle_min"])
        self.angle_increment = float(recording["angle_increment"])
        self.range_min, self.range_max = (float(v) for v in recording["range_limits"])
        # The lidar stamps are late by this against the odometry: every lidar message goes out stamped that
        # much earlier, so tf pairs it with the recorded pose of stamp - offset (as the live chain does).
        self.lidar_offset_s = float(recording.get("lidar_time_offset_s", 0.0))
        lever = [float(v) for v in recording["lever_xy"]]
        self.rtf = float(config.get("real_time_factor", 1.0))
        if not 0.1 <= self.rtf <= 2.0:
            raise ValueError("real_time_factor must be within 0.1..2")
        self.tick_hz = 100.0
        self.dt = self.rtf / self.tick_hz
        self.sim_time = float(config.get("start_time_s", 100.0))
        self.odom_period = 1.0 / float(config.get("odom_hz", 50.0))
        self.hold_period = float(config.get("hold_scan_period_s", 0.3))
        self.tail_hold_s = float(config.get("tail_hold_s", 5.0))
        self.state = "HOLD"
        self.playback_start = self.tail_start = None
        self.next_frame = 0
        self.frames_published = self.hold_scans = 0
        self.next_odom = self.next_hold_scan = self.next_status = self.sim_time
        self.last_pose_time = None
        self.Clock, self.Odometry, self.LaserScan, self.TransformStamped = Clock, Odometry, LaserScan, TransformStamped
        self.clock_pub = node.create_publisher(Clock, "/clock", 10)
        self.odom_pub = node.create_publisher(Odometry, "/odom", 20)
        self.recorded_pub = node.create_publisher(Odometry, "/wc_shadow/recorded", 20)
        self.scan_pub = node.create_publisher(LaserScan, "/scan", 10)
        self.status_pub = node.create_publisher(String, "/wc_shadow/status", latched_qos())
        self.tf = TransformBroadcaster(node)
        self.static_tf = StaticTransformBroadcaster(node)
        statics = [self._transform("base_link", "laser", (lever[0], lever[1], 0.0))]
        # publish_map_odom false: another node (a localization mode) owns map->odom.
        self.publishes_map_odom = bool(config.get("publish_map_odom", True))
        if self.publishes_map_odom:
            statics.insert(0, self._transform("map", "odom", (0.0, 0.0, 0.0)))
        self.static_tf.sendTransform(statics)
        self.recording = recording
        self.clouds = {}
        self.frames_node = None
        self.filtered_scan_pub = None
        if "obstacle_left_scan_ranges" in recording:
            self.filtered_scan_pub = node.create_publisher(LaserScan, FILTERED_SCAN_TOPIC, 10)
        cloud_statics = []
        if shadow_obstacles.cloud_sides(recording):
            from sensor_msgs.msg import PointCloud2, PointField
            self.PointCloud2, self.PointField = PointCloud2, PointField
        for side in shadow_obstacles.cloud_sides(recording):
            t = shadow_obstacles.relative_times(recording, side)
            frames = np.flatnonzero((t >= self.times[0] - 1e-9) & (t <= self.times[-1] + 1e-9))
            if not len(frames):
                continue
            arm = [float(v) for v in recording["obstacle_%s_lever" % side]]
            cloud_statics.append(self._transform("base_link", CLOUD_FRAMES[side], (arm[0], arm[1], 0.0), z=arm[2]))
            self.clouds[side] = {"t": t[frames], "frames": frames, "next": 0, "published": 0,
                                 "publisher": node.create_publisher(PointCloud2, CLOUD_TOPICS[side], 10),
                                 "clearing": node.create_publisher(PointCloud2, CLEAR_TOPICS[side], 10)}
        if cloud_statics:
            # The fixed/amcl localization modes remap this process's /tf_static away (their helper then
            # publishes map->odom and base_link->laser). The lidar frames go out from a node that ignores
            # process-wide remaps, so they reach Nav2 in every localization mode.
            import rclpy
            self.frames_node = rclpy.create_node("wc_shadow_obstacle_frames", context=node.context,
                                                 use_global_arguments=False)
            self.frames_tf = StaticTransformBroadcaster(self.frames_node)
            self.frames_tf.sendTransform(cloud_statics)
        node.create_service(Trigger, "/wc_shadow/start", self.on_start)
        node.create_timer(1.0 / self.tick_hz, self.tick)

    def _transform(self, parent, child, pose, stamp=None, z=0.0):
        t = self.TransformStamped()
        t.header.stamp = stamp if stamp is not None else _time_msg(self.sim_time)
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y = float(pose[0]), float(pose[1])
        t.transform.translation.z = float(z)
        qx, qy, qz, qw = yaw_to_quaternion(float(pose[2]))
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = qx, qy, qz, qw
        return t

    def on_start(self, request, response):
        if self.state != "HOLD":
            response.success, response.message = False, "replay already started"
            return response
        self.state, self.playback_start = "PLAYING", self.sim_time
        response.success, response.message = True, "playback started at %.3f s" % self.sim_time
        self.publish_status()
        return response

    def recording_time(self):
        if self.state == "HOLD":
            return float(self.times[0])
        if self.state == "PLAYING":
            return min(float(self.times[0]) + self.sim_time - self.playback_start, float(self.times[-1]))
        return float(self.times[-1])

    def twist(self, r):
        if self.state != "PLAYING":
            return 0.0, 0.0
        k = int(np.clip(np.searchsorted(self.times, r, side="right") - 1, 0, len(self.v) - 1))
        return float(self.v[k]), float(self.w[k])

    def tick(self):
        self.sim_time += self.dt
        clock = self.Clock()
        clock.clock = _time_msg(self.sim_time)
        self.clock_pub.publish(clock)
        if self.state == "PLAYING" and float(self.times[0]) + self.sim_time - self.playback_start >= float(self.times[-1]):
            self.flush_frames(float(self.times[-1]))
            self.flush_clouds(float(self.times[-1]))
            self.state, self.tail_start = "TAIL_HOLD", self.sim_time
            self.next_hold_scan = self.sim_time + self.hold_period
            self.publish_status()
        elif self.state == "TAIL_HOLD" and self.sim_time - self.tail_start >= self.tail_hold_s:
            self.state = "FINISHED"
            self.publish_status()
        r = self.recording_time()
        if self.sim_time + 1e-9 >= self.next_odom:
            self.next_odom += self.odom_period
            self.publish_pose(r)
        if self.state == "PLAYING":
            self.flush_frames(r)
            self.flush_clouds(r)
        elif self.sim_time + 1e-9 >= self.next_hold_scan:
            # The chair stands still here; repeat the nearest real scan with a fresh stamp.
            self.next_hold_scan += self.hold_period
            self.publish_pose(r)
            self.publish_scan(0 if self.state == "HOLD" else len(self.times) - 1, self.sim_time)
            for side, cloud in self.clouds.items():
                self.publish_cloud(side, 0 if self.state == "HOLD" else len(cloud["t"]) - 1, self.sim_time)
            self.hold_scans += 1
        if self.sim_time + 1e-9 >= self.next_status:
            self.next_status += 1.0
            self.publish_status()

    def flush_frames(self, r):
        while self.next_frame < len(self.times) and self.times[self.next_frame] <= r + 1e-9:
            k = self.next_frame
            stamp = self.playback_start + float(self.times[k] - self.times[0])
            self.publish_pose(r)
            self.publish_scan(k, stamp)
            self.next_frame += 1
            self.frames_published += 1

    def flush_clouds(self, r):
        """Publish every obstacle-cloud frame captured up to recording time ``r`` at its capture stamp."""
        for side, cloud in self.clouds.items():
            while cloud["next"] < len(cloud["t"]) and cloud["t"][cloud["next"]] <= r + 1e-9:
                k = cloud["next"]
                self.publish_cloud(side, k, self.playback_start + float(cloud["t"][k] - self.times[0]))
                cloud["next"] += 1

    def publish_cloud(self, side, k, stamp_s):
        cloud = self.clouds[side]
        frame = int(cloud["frames"][k])
        stamp = _time_msg(stamp_s - self.lidar_offset_s)
        points = shadow_obstacles.frame_points(self.recording, side, frame)
        cloud["publisher"].publish(pointcloud2(self.PointCloud2, self.PointField, points, CLOUD_FRAMES[side], stamp))
        rays = shadow_obstacles.frame_clearing(self.recording, side, frame)
        cloud["clearing"].publish(pointcloud2(self.PointCloud2, self.PointField, rays, CLOUD_FRAMES[side], stamp))
        ranges = shadow_obstacles.frame_scan(self.recording, side, frame)
        if ranges is not None and self.filtered_scan_pub is not None:
            self.filtered_scan_pub.publish(self.scan_message(ranges, stamp))
        cloud["published"] += 1

    def publish_pose(self, r):
        if self.sim_time == self.last_pose_time:
            return  # one transform per stamp; tf2 rejects repeated stamps
        self.last_pose_time = self.sim_time
        x, y, yaw = interpolate_poses(self.times, self.poses, r)[0]
        v, w = self.twist(r)
        stamp = _time_msg(self.sim_time)
        for publisher, frame, child in ((self.odom_pub, "odom", "base_link"),
                                        (self.recorded_pub, "map", "wc_shadow_recorded_base")):
            message = self.Odometry()
            message.header.stamp = stamp
            message.header.frame_id, message.child_frame_id = frame, child
            message.pose.pose.position.x, message.pose.pose.position.y = float(x), float(y)
            qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
            o = message.pose.pose.orientation
            o.x, o.y, o.z, o.w = qx, qy, qz, qw
            message.twist.twist.linear.x, message.twist.twist.angular.z = v, w
            publisher.publish(message)
        self.tf.sendTransform(self._transform("odom", "base_link", (x, y, yaw), stamp))

    def scan_message(self, ranges, stamp):
        """LaserScan in ``laser`` with the recording's 161-bin geometry."""
        message = self.LaserScan()
        message.header.stamp = stamp
        message.header.frame_id = "laser"
        message.angle_min = self.angle_min
        message.angle_increment = self.angle_increment
        message.angle_max = self.angle_min + self.angle_increment * (len(ranges) - 1)
        message.time_increment, message.scan_time = 0.0, 0.0
        message.range_min, message.range_max = self.range_min, self.range_max
        message.ranges = [float(v) for v in ranges]
        return message

    def publish_scan(self, k, stamp_s):
        self.scan_pub.publish(self.scan_message(self.ranges[k], _time_msg(stamp_s - self.lidar_offset_s)))

    def publish_status(self):
        self.status_pub.publish(json_message({
            "state": self.state, "sim_time_s": round(self.sim_time, 3),
            "playback_start_s": self.playback_start, "tail_start_s": self.tail_start,
            "window_s": [float(self.times[0]), float(self.times[-1])],
            "recording_time_s": round(self.recording_time(), 3),
            "frames_total": int(len(self.times)), "frames_published": self.frames_published,
            "hold_scans_republished": self.hold_scans, "real_time_factor": self.rtf,
            "obstacle_clouds_published": {side: c["published"] for side, c in self.clouds.items()},
            "filtered_scan_published": self.filtered_scan_pub is not None,
            "lidar_time_offset_s": self.lidar_offset_s,
            "map_odom_published": self.publishes_map_odom,
            "localization": "IDENTITY_MAP_IS_RECORDING_ODOMETRY_FRAME" if self.publishes_map_odom
            else "MAP_TO_ODOM_FROM_ANOTHER_NODE",
            "commands_subscribed": False, "hardware_control": False}))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    args, ros_args = parser.parse_known_args(argv)
    require_isolated_domain()
    config = load_config(args.config)
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=ros_args)
    node = rclpy.create_node("wc_shadow_replay")
    replay = None
    try:
        replay = Replay(node, config)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if replay is not None and replay.frames_node is not None:
            replay.frames_node.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
