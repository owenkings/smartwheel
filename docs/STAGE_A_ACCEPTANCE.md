# Stage A-R Acceptance Gate

Date: 2026-07-14
Branch: `review/mapping-v2-stage-a`
Audited base: `1db393a`
Remediation commits: `333ce65`, `16f06ae`, `59443eb`, `4b3ca3b`, `17267c1`

## Scope

This gate covers synthetic Stage A architecture and its fail-closed boundaries only.
It does not validate XT-M60, H30, encoders, cameras, motors, ultrasonic sensors,
FAST-LIO2 performance, localization, navigation, or passenger safety.

## Gate checks

| Check | Result | Evidence |
| --- | --- | --- |
| Ground truth excluded from odometry and mapping inputs | PASS | `/sim/ground_truth/odom` is consumed by the evaluator and recorder only; architecture regression test passes. |
| Non-circular trajectory metric | PASS | Accepted synthetic runs have nonzero RMSE; first-pose SE(2) alignment and timestamp association are tested. |
| RTAB geometry is globally optimized backend output | PASS | Export requires `/rtabmap/optimized_cloud`; accepted report records backend source. |
| Fixed or pre-generated success artifacts rejected | PASS | Manager validates runtime evidence and output manifest; Stage A code has no references to existing `maps/lab_map_*` artifacts. |
| 2D map uses ray casting | PASS WITH LIMITATION | Bresenham rays create free/occupied/unknown cells. Global/probabilistic D1 work remains A-R-013. |
| Colorization is genuine projection | PARTIAL | Projection and z-buffer logic exist, but distortion and real calibrated runtime evidence remain A-R-012. |
| TF ownership | PASS | Runtime RTAB graph had robot-state, state-selector, and RTAB publishers with separate responsibilities; RTAB mode had no `/map`. slam_toolbox mode had one `/map` publisher. |
| `hardware_enabled=false` opens no hardware transport | PASS | Source contains no transport implementation in mapping-v2 drivers; real mode with false fails explicitly. Sim/offline launches reject hardware true. |
| WSAD timeout stop path | PASS FOR TOPIC-LEVEL STAGE A | Deadman and freshness gate raw command; downstream watchdog relays or zeros. No motor driver was launched or tested. |
| Clean Stage A build | PASS | 16 packages built from empty build/install/log directories. |
| Stage A tests | PASS | 66 tests, 0 errors, 0 failures, 0 skipped. |
| Full workspace build | PASS | 28 packages built from empty build/install/log directories, including ARM64 FAST-LIO. |
| Full workspace tests | EXCEPTION | 173 project functional tests pass; five third-party Livox lint targets fail on untouched upstream/vendored files. |
| Automated ROS launch tests | FAIL, MAJOR | Manual runtime evidence exists; no `launch_testing` suite. Tracked as A-R-010. |
| BLOCKER/CRITICAL defect closure | PASS | 0 BLOCKER and 0 CRITICAL open. |

## Runtime acceptance evidence

- RTAB loop enabled: 27,215 optimized points, 1,135 estimated poses, RMSE 0.0382 m,
  endpoint error 0.0303 m, 30 graph poses, and seven unique spatial closure links.
- RTAB loop disabled: 29,556 optimized points, RMSE 0.0380 m, 30 graph poses, and no
  non-neighbor closure links. Similar RMSE is expected for this short, low-drift model;
  it is not proof that loop closure is unnecessary.
- slam_toolbox: one `/map` publisher, map products on separate topics, manager READY.
- rosbag: SQLite3 bag, 58.9 MiB, 11,211 messages, 45.861 s duration.
- Offline 1x replay: 1,105 estimated poses, 26,925 backend points, RMSE 0.0430 m,
  endpoint error 0.0389 m, matching the source online run.
- Offline 8x replay: deliberately failed equivalence with 464 poses and RMSE 0.529 m;
  accelerated RTAB replay is now rejected.

## Devices

Used: Jetson compute environment only.

Not used: both XT-M60 units, H30, motor drivers, encoders, all cameras, all FD07-34R
units, physical emergency stop, serial ports, CAN, RS485, USB camera devices, and LiDAR
network endpoints. No motor command process was launched.

## Parameter sources

All runtime parameters came from repository mock profiles, ROS package defaults, and
synthetic seed `20260714`. No hardware IP, serial path, unit, frequency, calibration,
or extrinsic was inferred or added.

## Next-stage prerequisites

Before B0, the user must explicitly approve this gate and provide official source
material for XT-M60, H30, motor/encoder, cameras, FD07-34R, and mechanical installation.
B0 must remain documentation-only and must not connect to any device.

Before B1, additionally close A-R-014 by preparing a single-LiDAR diagnostic/RViz view
from confirmed vendor fields; do not reuse guessed legacy geometry or timestamps.

## Decision

PROCEED

This means **PROCEED TO STAGE B0 DOCUMENT AUDIT ONLY AFTER EXPLICIT USER APPROVAL**.
It does not permit B1, device connection, motor commands, FAST-LIO2 hardware testing,
or any passenger test.
