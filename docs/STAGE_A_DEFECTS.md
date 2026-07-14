# Stage A-R Defect Register

Date: 2026-07-14
Branch: `review/mapping-v2-stage-a`
Audited base: `1db393a`
Latest remediation code commit before this report: `17267c1`

## Severity policy

- BLOCKER: prohibits Stage B0 and all hardware work.
- CRITICAL: must be closed before Stage B0.
- MAJOR: must be closed before the stage named in the disposition.
- MINOR: tracked cleanup that does not open a hardware or mapping gate.

## Closed defects

| ID | Severity | Original defect | Remediation and evidence | Status |
| --- | --- | --- | --- | --- |
| A-R-001 | BLOCKER | `mock_lio_node` copied `/sim/ground_truth/odom`, making zero RMSE circular rather than estimated performance. | Replaced the subscription with simulator-only `SimMotion` increments and deterministic scale, bias, random-walk, and white-noise terms. Ground-truth odometry is now referenced only by the evaluator/recorder. Runtime RMSE is nonzero. Commit `333ce65`. | CLOSED |
| A-R-002 | BLOCKER | RTAB mode exported a local odometry accumulator and presented it as the global map. | Added an RTAB `GetMap(global=true, optimized=true)` keyframe-cloud assembler. Export now requires `/rtabmap/optimized_cloud` and records `geometry_source=backend:/rtabmap/optimized_cloud`; missing/corrupt backend data fails export. Commit `16f06ae`. | CLOSED |
| A-R-003 | CRITICAL | Mapping manager advanced through phases and could report success without backend map, loop, export, or quality evidence. | Manager now waits for backend map/info, spatial loop evidence when enabled, export completion, manifest contents, point/pose counts, occupancy states, and computed RMSE. Timeout or failed evidence produces FAILED. Commit `16f06ae`. | CLOSED |
| A-R-004 | CRITICAL | Custom map products published `/map`, allowing competition with RTAB-Map or slam_toolbox for the global map product. | Custom products moved to `/map_products/occupancy` and `/map_products/cloud`; RTAB map is `/rtabmap/map`. Runtime slam_toolbox test found exactly one `/map` publisher. Commit `16f06ae`. | CLOSED |
| A-R-005 | CRITICAL | The command watchdog observed commands but was not the downstream command gate. | Teleop publishes raw `/motor/command`; watchdog alone relays to `/motor/command_safe`, publishes timeout zero, and publishes shutdown zero. No Stage A motor transport consumes either topic. Commit `59443eb`. | CLOSED |
| A-R-006 | CRITICAL | RTAB temporal-neighbor constraints could be counted as loop closure evidence, and the manager's info topic did not match the remap. | Set spatial proximity on, temporal proximity off, remapped info to `/rtabmap/info`, and require proximity/loop evidence before a loop-enabled run can pass. A loop-enabled database had seven unique type-2 spatial links; loop-disabled had none. Commit `16f06ae`. | CLOSED |
| A-R-007 | CRITICAL | Accelerated offline replay silently dropped synchronized RTAB inputs, and recorded `/sim/completed` could trigger export before the backend drained. | RTAB replay rejects rates above 1.0, remaps the recorded completion event, validates settle delay, and exports after playback plus settle time. A 1x replay matched its online source metrics. Commit `16f06ae`. | CLOSED |
| A-R-008 | MAJOR | `odom -> base_link` TF used wall-clock publication time rather than the selected odometry timestamp. | State selector now preserves the odometry header timestamp in TF. Commit `333ce65`. | CLOSED |
| A-R-009 | MAJOR | The dependency manifest omitted `livox_ros_driver2`, put `ikd-Tree` outside FAST-LIO's required path, and mixed a reference source into build dependencies. | Build manifest now pins FAST-LIO, nested ikd-Tree, and Livox at exact commits; the upstream mathematical reference has a separate manifest. Empty-directory local clone layout was verified. Commit `4b3ca3b`. | CLOSED |

## Open defects and limitations

| ID | Severity | Defect or limitation | Required disposition | Status |
| --- | --- | --- | --- | --- |
| A-R-010 | MAJOR | The original 43 tests and the reviewed 66 tests are unit/file-level tests; there is no automated `launch_testing` ROS graph suite. | Add launch tests for fail-closed startup, TF ownership, backend evidence, export failure, rosbag replay, and watchdog timeout before relying on these graphs for hardware bringup automation. Manual runtime evidence is documented for Stage A-R. | OPEN; before B1 automation |
| A-R-011 | MAJOR | Dual-LiDAR map-only pairing enforces timestamps and rigid transforms but has no ego-motion compensation between paired frames. | Implement and validate motion compensation only after real timestamps and extrinsics are known. | OPEN; before C3 moving test |
| A-R-012 | MAJOR | Colorization checks front-facing points, image bounds, nearest depth, and view score, but does not apply lens distortion and has no calibrated real-camera runtime evidence. | Implement distortion-aware projection and calibrated visibility tests. | OPEN; before C5 |
| A-R-013 | MAJOR | The exported 2D grid uses Bresenham ray casting, but uses the local odometry accumulator rather than globally optimized RTAB geometry and performs binary rather than probabilistic updates. | Compare RTAB grid and slam_toolbox using measured dimensions; select one global backend and validate unknown/free/occupied semantics. | OPEN; before D1 |
| A-R-014 | MAJOR | Mapping-v2 launches do not start RViz and provide no reviewed RViz layout. Legacy RViz configs are not evidence for the new architecture. | Add a B1 visualization configuration for one XT-M60, diagnostics, TF, and status before live bringup. Add camera/map panels only in their later gates. | OPEN; before B1 |
| A-R-015 | MAJOR | Mock-LIO and the 14 adversarial cases are analytical error-propagation models, not FAST-LIO2 execution. | Replace mock-LIO with recorded single-XT-M60 plus H30 FAST-LIO2 evidence at C1. Never report Stage A metrics as hardware accuracy. | OPEN; before C1 |
| A-R-016 | MINOR | `camera_array_driver` has no package-local tests; only cross-package contracts exercise camera naming. | Add image encoding, camera-info, drop, and unique-device tests during B4. | OPEN; before B4 |
| A-R-017 | MINOR | Workspace-wide tests are nonzero because the ignored third-party Livox checkout fails five upstream formatting/copyright lint targets over vendored code. | Keep third-party source unmodified; isolate upstream lint policy from project-owned functional tests. | OPEN; non-functional third-party exception |

## Counts

- BLOCKER: 2 closed, 0 open.
- CRITICAL: 5 closed, 0 open.
- MAJOR: 2 additional closed, 6 open.
- MINOR: 0 closed, 2 open.

No open BLOCKER or CRITICAL remains. Open MAJOR items do not authorize skipping their
named future gate.
