# Known Limitations

- Stage A uses deterministic synthetic sensors and mock-LIO. It is not evidence of
  FAST-LIO2 accuracy or real XT-M60/H30 compatibility.
- XT-M60 SDK fields, units, timestamp semantics, and per-point timing are unknown.
- H30 transport/protocol, axes, orientation availability, and covariance are unknown.
- Wheel geometry, encoder scaling, direction, and push-mode feedback are unknown.
- All real sensor extrinsics and camera calibration are unknown.
- `dual_lio` is intentionally `NOT_IMPLEMENTED`; only `dual_map_only` is exercised.
- Online visual loop closure is optional and limited to one camera by contract. Stage A
  validates only synthetic offline projection/colorization.
- RTAB-Map, slam_toolbox, and robot_localization are distribution packages. Stage A
  launch ownership is tested, but real-data tuning remains Phase B work.
- Free disk space was 12 GiB at audit time, which is marginal for multi-camera rosbag.
- Legacy `wheelchair_*` packages remain in the workspace and must not be mixed into the
  mapping-v2 launch graph.
- FD07-34R ultrasonic support is deliberately absent from mapping v2.

