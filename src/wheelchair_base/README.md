# wheelchair_base

This package contains the first ROS2 base driver for the wheelchair.

- Subscribes `/cmd_vel_safe`.
- Publishes `/wheel/odom`.
- Optionally broadcasts `odom -> base_link`.
- Talks to ZLAC8030-like dual wheel servo drivers through configurable Modbus RTU registers.

The ZLAC8030 register map must be verified against the exact driver manual and KeepLINK wiring mode before enabling real writes. By default the YAML keeps command registers disabled so the node can run in open-loop odometry mode without moving hardware.

Typical run:

```bash
ros2 launch wheelchair_bringup base.launch.py mode:=mock
ros2 topic pub /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
ros2 topic echo /wheel/odom
```
