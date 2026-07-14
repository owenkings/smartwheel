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
  launch ownership is runtime-tested, but real-data tuning remains Phase B work.
- FAST-LIO2 and its build dependencies are pinned and compile on the current ARM64
  machine, but FAST-LIO2 is not executed in Stage A. The reviewed `mock_lio_node` uses
  an independent cumulative motion input with configurable drift and noise; it is still
  only a kinematic error model, not a LiDAR-inertial estimator.
- `single_lidar_mapping.launch.py` exposes the FAST-LIO2 replacement boundary; a real
  FAST-LIO2 process cannot be launched until its pinned dependency and XT-M60 contract
  are available in Stage B.
- `enable_online_visual_loop` is reserved but not implemented. RTAB-Map Stage A runs
  LiDAR-cloud ICP without image bag-of-words loop closure.
- Offline replay is validated at `replay_rate:=1.0`. An 8x adversarial replay dropped
  synchronized RTAB-Map inputs and exported distorted geometry, so RTAB-Map replay now
  rejects rates above 1.0 and waits for backend settling before export.
- The custom ray-cast occupancy map is exported from the local odometry accumulator,
  while 3D geometry comes from RTAB-Map optimized keyframe poses. D1 must compare this
  against the RTAB-Map grid and slam_toolbox using real dimensions.
- Offline colorization has front-facing, image-boundary, simple z-buffer, and view-score
  checks, but it does not apply lens distortion and has not been validated with real
  images or calibrated camera poses.
- Dual-LiDAR map-only pairing enforces a timestamp tolerance and full rigid transforms,
  but does not motion-compensate paired frames. C3 must close this before moving tests.
- `rtabmap.db` and files under `raw_bag/` are marked as externally managed in the map
  manifest and are not checksummed until their owning processes stop. Exporter-owned
  finalized products retain size and SHA-256 entries.
- Mapping-manager frequency and timing checks use a short startup window. Hardware
  acceptance still requires sustained diagnostics and the 30-minute Stage B run.
- Free disk space was 12 GiB at audit time, which is marginal for multi-camera rosbag.
- Legacy `wheelchair_*` packages remain in the workspace and must not be mixed into the
  mapping-v2 launch graph.
- Workspace-wide `colcon test-result --verbose` is nonzero because the retained
  `src/third_party/livox_ros_driver2` package runs lint over vendored RapidJSON and has
  upstream copyright, cpplint, flake8, lint-cmake, and uncrustify failures. Mapping-v2's
  isolated result is 66 tests with zero errors or failures; third-party sources were not
  rewritten to hide this pre-existing exception.
- The 66 mapping-v2 tests are unit/file-level tests. There is no automated
  `launch_testing` suite yet; Stage A-R ROS graph, rosbag, RTAB-Map, and slam_toolbox
  evidence was executed manually and recorded in the review report.
- FD07-34R ultrasonic support is deliberately absent from mapping v2.
