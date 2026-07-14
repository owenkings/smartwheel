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
- FAST-LIO2 is pinned as a third-party dependency and has an input adapter contract, but
  is not cloned or executed in Stage A. The runtime `mock_lio_node` copies ground truth.
- `single_lidar_mapping.launch.py` exposes the FAST-LIO2 replacement boundary; a real
  FAST-LIO2 process cannot be launched until its pinned dependency and XT-M60 contract
  are available in Stage B.
- `enable_online_visual_loop` is reserved but not implemented. RTAB-Map Stage A runs
  LiDAR-cloud ICP without image bag-of-words loop closure.
- Offline replay is validated at `replay_rate:=1.0`. A 10x stress replay dropped
  odometry callbacks and produced distorted geometry; high-rate replay is unsupported.
- The custom ray-cast occupancy map is exported. RTAB-Map's occupancy state remains in
  `rtabmap.db` and on its ROS topic; a second RTAB-generated PGM is not exported.
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
  isolated result is 43 tests with zero errors or failures; third-party sources were not
  rewritten to hide this pre-existing exception.
- FD07-34R ultrasonic support is deliberately absent from mapping v2.
