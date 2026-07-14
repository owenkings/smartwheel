# Changelog

All notable changes to the clean mapping architecture are documented here.

## [Unreleased]

### Added

- Stage A environment and legacy reuse audits.
- Frame, topic, ownership, and timing contracts for mapping v2.
- Pinned third-party dependency metadata and license inventory.
- Hardware profile templates with explicit unknown values and a simulation-only profile.
- Phase B hardware bring-up runbook. No hardware validation has been performed.
- Sixteen-package mapping-v2 ROS 2 architecture with mock/vendor boundaries.
- Deterministic closed-loop indoor simulator for dual LiDAR, IMU, wheel encoders, and
  four cameras, including noise, offsets, dropouts, slip, occlusion, and blur fixtures.
- Timestamp-gated dual-LiDAR fusion, wheel odometry, mock-LIO/state selection, residual
  diagnostics, and exclusive RTAB-Map/slam_toolbox backend ownership.
- Ray-cast occupancy, PCD/PLY/XYZRGB export, trajectory, quality report, manifest, and
  versioned map directories.
- Safe WSAD teleop with deadman, acceleration limiting, watchdog stop, and a default-off
  hardware transport gate.
- Complete simulation, rosbag recording, namespaced offline replay, manual export
  service, and PointCloud2-to-LaserScan baseline workflow.
- Unit and integration coverage for kinematics, overflow, units, time, cloud fields,
  pairing, transforms, filtering, TF ownership, map products, teleop, and hardware gate.
