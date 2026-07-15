# SmartWheel RViz Plugins

Six RViz2 `rviz_common::Panel` plugins implement the mock-only mapping operator
workbench. The panels use the RViz ROS node and queued Qt signals; image and map
callbacks retain at most one pending GUI update.

The package does not open hardware devices and the teleop panel publishes only
`/teleop/cmd_vel`. The mock workbench safety node is the sole publisher of
`/cmd_vel_safe`; no panel publishes a motor command.

