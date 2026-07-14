# Stage A Delivery Report

> **Superseded by Stage A-R review.** This file records the original Stage A claims and
> is not acceptance evidence. The review found ground-truth leakage, zero-error metric
> circularity, non-backend map export, automatic success transitions, and command
> watchdog defects. Use `STAGE_A_REVIEW_REPORT.md`, `STAGE_A_DEFECTS.md`, and
> `STAGE_A_ACCEPTANCE.md` for the audited result.

Date: 2026-07-14
Branch: `feature/mapping-v2-clean-architecture`

## Scope and safety

Stage A is complete using synthetic data only. No serial, CAN, RS485, USB camera,
LiDAR network, vendor SDK, motor transport, `sudo`, or physical device probe was used.
The generated evidence is not real XT-M60, H30, wheel, camera, motor, or FAST-LIO2
validation.

## New package tree

```text
src/
├── smartwheel_interfaces
├── smartwheel_description
├── smartwheel_sensor_api
├── xtm60_ros2_driver
├── h30_imu_driver
├── wheel_odom_driver
├── camera_array_driver
├── dual_lidar_fusion
├── smartwheel_state_estimation
├── smartwheel_global_mapping
├── smartwheel_map_products
├── smartwheel_mapping_manager
├── smartwheel_teleop
├── smartwheel_bringup
├── smartwheel_sim
└── smartwheel_tests
```

## Package responsibilities

| Package | Responsibility |
| --- | --- |
| `smartwheel_interfaces` | Mapping status, hardware status, wheel encoder, and map-task contracts |
| `smartwheel_description` | Parameterized Xacro and static sensor/wheel frame tree |
| `smartwheel_sensor_api` | Backend interfaces, units/time/cloud validation, PointCloud2 helpers, mocks |
| `xtm60_ros2_driver` | Stage A mock and isolated vendor-SDK adapter boundary |
| `h30_imu_driver` | Stage A mock/parser boundary and diagnostics |
| `wheel_odom_driver` | Encoder rollover, signs, scale, differential kinematics, covariance |
| `camera_array_driver` | Four-camera naming, mock image/camera info, drop diagnostics |
| `dual_lidar_fusion` | Timestamp pairing, extrinsics, validation, voxel filtering, map-only merge |
| `smartwheel_state_estimation` | Mock-LIO, selected odometry/TF owner, wheel-LIO residual diagnostics |
| `smartwheel_global_mapping` | FAST-LIO input adapter, backend ownership, RTAB-Map/toolbox configs, scan adapter |
| `smartwheel_map_products` | Ray-cast grid, geometry/color export, trajectory, quality, manifest |
| `smartwheel_mapping_manager` | Preflight checks, required state sequence, task service, failure reasons |
| `smartwheel_teleop` | WSAD intent, deadman, ramps, timeout/watchdog stop, hardware gate |
| `smartwheel_bringup` | Seven required launch compositions and hardware profiles |
| `smartwheel_sim` | Seeded indoor world, closed route, synthetic sensors and fault fixtures |
| `smartwheel_tests` | Cross-package architecture, contract, and end-to-end product tests |

## Commands

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
colcon test
colcon test-result --verbose

ros2 launch smartwheel_bringup sim_mapping.launch.py map_name:=stage_a_demo
ros2 launch smartwheel_bringup sim_mapping.launch.py \
  map_name:=recorded_run record_bag:=true
ros2 launch smartwheel_bringup offline_mapping.launch.py \
  bag_path:=/absolute/path/to/raw_bag map_name:=replayed_run replay_rate:=1.0
ros2 launch smartwheel_bringup map_export.launch.py
ros2 service call /map_export/export std_srvs/srv/Trigger '{}'
```

The exact full build completed all 28 packages. The isolated 16-package mapping-v2
suite completed 43 tests with zero errors, failures, or skips. The workspace-wide test
audit is nonzero only in the pre-existing `src/third_party/livox_ros_driver2` lint suite
(vendored RapidJSON copyright/style findings and an upstream uncrustify parser error).
No third-party source was modified to mask those failures.

## Runtime evidence

The accepted RTAB-Map simulation produced 302 estimated poses and 302 ground-truth
poses, `0 m` trajectory RMSE, `0 m` endpoint error, 24,971 geometry/colored points,
occupied/free/unknown cells, full room bounds, and a 16-node RTAB-Map 0.23.7 database.
Every required artifact was nonempty.

The recording test produced a 3.0 MiB rosbag with 2,114 messages, including 15 left,
15 right, and 15 merged clouds; 150 IMU messages; 151 fused and ground-truth odometry
messages; static/dynamic TF; status; diagnostics; and the completion marker. A 1x
offline replay retained all 151 poses, regenerated the expected map extent, and rebuilt
a seven-node RTAB-Map database.

The optional `slam_toolbox` path had one scan publisher and one toolbox subscriber,
published a 285 x 190 grid at 0.05 m/cell, and exclusively published `map -> odom` while
RTAB-Map was absent.

The final manager-health run reached `READY` with all source-rate, timestamp, dual
LiDAR delta, LiDAR/IMU delta, point-unit, wheel, static-TF, disk, and bag-policy checks
reported `PASS`.

## Completed

- Environment, legacy reuse, frame/topic, dependency, and license audits.
- Explicit mock and all-null/TODO hardware profiles.
- All requested package boundaries, launch files, state sequence, map products, and
  Stage A safety gates.
- Deterministic unit/integration coverage and live RTAB-Map/toolbox validation.
- Versioned record, replay, export, manifest, profile, and quality-report workflows.

## Not completed by design

- Any physical hardware access or validation.
- XT-M60 SDK implementation and real point/timestamp semantics.
- Real H30 transport/protocol and coordinate validation.
- Real FAST-LIO2 execution/tuning; Stage A uses mock-LIO ground truth.
- `dual_lio`; it emits `NOT_IMPLEMENTED` and only `dual_map_only` is exercised.
- Online visual loop closure and real-camera colorization/calibration.

## Stage B information required

Two XT-M60 IPs, SDK path and official configs; SDK point/intensity/time fields; each
LiDAR pose, height, centre spacing and overlap; H30 protocol, pose and axes; wheel
radius, track, CPR, gear ratio and signs; motor/encoder protocol evidence; four camera
identifiers, modes, intrinsics and poses; and confirmation that encoders remain powered
and valid while the chair is manually pushed.

## Risks

Unknown timestamp semantics and narrow Flash-LiDAR overlap can invalidate LIO and
fusion assumptions. Wheel slip and uncertain push-mode feedback limit fallback odometry.
Camera synchronization and calibration can dominate colorization quality. Available
disk is marginal for four-camera raw bags. Follow `docs/HARDWARE_BRINGUP_RUNBOOK.md`
without skipping the single-sensor scale, axes, timestamp, and static tests.

## Git history

1. `docs(mapping): add stage A audits and contracts`
2. `feat(sensors): add mapping v2 interfaces and mock backends`
3. `feat(mapping): add odometry fusion and safe teleop core`
4. `feat(sim): add deterministic indoor sensor simulator`
5. `feat(mapping): add map products backends and task manager`
6. `feat(bringup): add Stage A mapping workflows`
7. `fix(mapping): harden runtime health and artifact export`
8. `test(mapping): add Stage A acceptance coverage`
9. `docs(mapping): complete Stage A delivery report`
