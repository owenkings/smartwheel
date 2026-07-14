# Stage A-R Review Report

Date: 2026-07-14
Branch: `review/mapping-v2-stage-a`
Audited branch tip: `feature/mapping-v2-clean-architecture` at `1db393a`
Latest remediation code commit before this report: `17267c1`

## Executive result

The original Stage A summary was not reliable. The original mock-LIO consumed ground
truth odometry directly, so its zero RMSE was circular. RTAB mode exported locally
accumulated geometry rather than an optimized RTAB map. The manager could advance to
success without backend or quality evidence, custom products competed for `/map`, and
the command watchdog was not in the downstream command path.

Those BLOCKER and CRITICAL issues were corrected and exercised in live synthetic ROS 2
graphs. The reviewed architecture now fails closed, reports nonzero error, requires
optimized backend geometry, distinguishes real loop evidence, and reproduces a recorded
online run at 1x replay. Remaining MAJOR limitations are explicitly assigned to later
gates and are not hardware evidence.

## Safety and access record

No real device was connected or queried. No serial port, CAN interface, RS485 bus, USB
camera, LiDAR network endpoint, vendor SDK, or motor transport was accessed. Both
`sim_mapping` and `offline_mapping` reject hardware enablement. A direct XT-M60 real-mode
attempt with `hardware_enabled=false` terminated with an explicit RuntimeError before
any transport exists.

## Required audit questions

| Audit question | Finding after remediation |
| --- | --- |
| 1. Is ground truth evaluator-only? | `/sim/ground_truth/odom` is evaluator/recorder-only. The simulator necessarily derives synthetic messages from one trajectory; mock-LIO receives only a simulator motion-increment contract and injects independent configured errors, not a ground-truth pose. |
| 2. Does RTAB/map consume ground-truth odometry? | No. RTAB subscribes `/odom/fused` plus `/lidar/merged/points`; mapping packages contain no ground-truth topic reference. |
| 3. Was zero RMSE circular? | Yes in the audited base. Fixed. Accepted runtime RMSE is nonzero. |
| 4. Does mock-LIO model noise and drift? | Yes: distance/yaw scale, fixed bias, random walk, white noise, seed, and covariance are configurable. It remains an analytical mock, not FAST-LIO2. |
| 5. Are success metrics hard-coded? | Original manager behavior was effectively fixed success. It now validates measured backend, loop, export, manifest, occupancy, count, and RMSE evidence. Thresholds are parameters. |
| 6. Are pre-generated maps used as output? | No mapping-v2 source references existing map artifacts. Runtime output directories are unique and backend geometry is required. Old repository maps remain legacy data only. |
| 7. Is 2D mapping ray-cast? | Yes. It uses Bresenham free-space rays with occupied endpoints and unknown initialization, not XY black-point projection. See A-R-013 for D1 limitations. |
| 8. Is point-cloud coloring real projection? | Partially. Code performs camera-frame transform, positive depth, image bounds, simple nearest-depth visibility, and view scoring. It lacks distortion and calibrated real runtime validation; A-R-012 remains open. |
| 9. Are TF edges multiply published? | Source ownership is separated: robot_state_publisher owns the robot tree, state selector owns `odom -> base_link`, selected map backend owns `map -> odom`. Runtime `/tf` showed these three publishers; no second mapping backend ran. |
| 10. Can RTAB and slam_toolbox compete for `map -> odom`? | Launch conditions are mutually exclusive. RTAB runtime had no `/map`; slam_toolbox runtime had exactly one `/map` publisher. |
| 11. Does `hardware_enabled=false` avoid real devices? | Yes for mapping-v2. Real adapters are not implemented and fail explicitly; no silent fallback exists. Legacy packages must not be mixed into the graph. |
| 12. Does WSAD have timeout stop? | Yes at Stage A topic level: deadman/freshness outputs zero, watchdog owns the downstream safe topic, and shutdown emits zero. No motor behavior was validated. |
| 13. Are 28 packages mostly shells? | No. There are 16 mapping-v2 packages, 10 legacy packages, and 2 ignored third-party packages. Interface/description packages are intentionally data-only. Real hardware adapters are explicit fail-closed placeholders, not completed drivers. |
| 14. Did the original 43 tests cover runtime? | No. They were unit/file tests. The reviewed suite has 66 such tests. Live ROS graph evidence was added manually, but automated launch tests remain A-R-010. |
| 15. Are third-party dependencies pinned? | Yes after repair: FAST-LIO `2fffc570...`, nested ikd-Tree `e2e3f4...`, Livox ROS 2 `378d1c...`; upstream reference `7cc417...` is separate. |
| 16. Is a clean environment reproducible? | Stage A's 16 packages and the current 28-package workspace build cleanly from empty output dirs. Dependency layout was reproduced from local exact commits. `python3-vcstool` is not installed on this machine, so network `vcs import` was not executed; the prerequisite and manifest are documented. |

## Adversarial simulation

Command output: `/tmp/smartwheel-stage-a-review.s88fud/adversarial_results.json`.
Seed: `20260714`. This is deterministic analytical error propagation, not FAST-LIO2,
Gazebo physics, or hardware evidence.

| Scenario | Trajectory RMSE m | Endpoint m | Map p95 m | Detection/result |
| --- | ---: | ---: | ---: | --- |
| Ideal data | 0.124 | 0.027 | 0.234 | Loop correction applied; nonzero baseline noise |
| IMU noise and bias | 5.072 | 0.955 | 8.425 | IMU degradation detected |
| Wheel scale error | 0.171 | 0.027 | 0.316 | Wheel inconsistency detected |
| Left/right wheel mismatch | 0.123 | 0.027 | 0.235 | Wheel inconsistency detected |
| Wheel slip | 0.133 | 0.026 | 0.251 | Wheel inconsistency detected |
| LiDAR frame drop | 0.120 | 0.035 | 0.215 | 20% configured drop detected |
| IMU frame drop | 0.116 | 0.030 | 0.261 | 25% configured drop detected |
| LiDAR-IMU time offset | 0.124 | 0.273 | 0.248 | 0.32 s offset rejected; no loop correction |
| Dual-LiDAR time offset | 0.145 | 0.267 | 0.269 | 0.08 s pairing rejected; no loop correction |
| Extrinsic perturbation | 0.124 | 0.027 | 0.733 | Map degradation detected |
| Long corridor | 0.249 | 0.062 | 8.146 | Geometric degeneracy detected |
| Narrow FOV/occlusion | 0.277 | 0.065 | 0.493 | Geometric degeneracy detected |
| Loop closure disabled | 0.145 | 0.267 | 0.269 | No loop evidence/correction |
| Loop closure enabled | 0.124 | 0.027 | 0.234 | Loop evidence/correction present |

The analytical detectors respond in all requested fault categories. They do not prove
that FAST-LIO2, RTAB-Map, or real diagnostics detect the same faults.

## Live ROS runtime evidence

### RTAB-Map, loop enabled

Output:
`/tmp/smartwheel-stage-a-review.s88fud/runtime-space-loop-on-robust/space_loop_on_robust_20260714_140146_623615`

- Manager reached READY only after all gates.
- 27,215 optimized backend points and 1,135 estimated trajectory poses.
- SE(2)-aligned trajectory RMSE 0.038216 m; endpoint error 0.030297 m.
- Database: 30 graph poses, 29 unique neighbor edges, seven unique type-2 spatial
  closure links.

### RTAB-Map, loop disabled

Output:
`/tmp/smartwheel-stage-a-review.s88fud/runtime-space-loop-off/space_loop_off_20260714_140329_665718`

- 29,556 optimized points; RMSE 0.038018 m; endpoint error 0.029819 m.
- Database: 30 poses, neighbor links only, no closure links.
- The short mock trajectory has low local drift, so similar RMSE does not establish
  real loop-closure value.

### slam_toolbox

Output:
`/tmp/smartwheel-stage-a-review.s88fud/runtime-slam-toolbox/slam_toolbox_review_20260714_140442_584836`

- Manager READY.
- `/map` publisher count was one and the publisher was `slam_toolbox`.
- Custom map products remained on `/map_products/*`.

### Bag record and replay

Online output:
`/tmp/smartwheel-stage-a-review.s88fud/runtime-bag-record/bag_record_20260714_140605_491035`

- 58.9 MiB SQLite3 bag, 11,211 messages, 45.861 s duration.
- Included left/right/merged LiDAR, IMU, encoders, wheel odom, mock LIO, fused odom,
  ground truth for evaluation, TF, TF static, status, and completion.
- Cameras were intentionally disabled in this run.

Offline 1x output:
`/tmp/smartwheel-stage-a-review.s88fud/runtime-offline-replay-1x/offline_replay_1x_20260714_141019_096660`

- 1,105 estimated poses, 26,925 backend points, RMSE 0.042988 m, endpoint 0.038940 m.
- Metrics match the source online run. Database has 29 nodes and spatial closure links.
- The earlier 8x replay produced only 464 poses and RMSE 0.528840 m. This failure led
  to the replay-rate and delayed-export hard gate.

### TF and map-topic audit

- `/tf --verbose --no-daemon` showed robot_state_publisher, state selector, and RTAB.
- `/tf_static` had one publisher, robot_state_publisher.
- RTAB mode had no `/map`; `/rtabmap/map` had exactly one RTAB publisher.
- Direct TF samples confirmed `map -> odom` and static `base_link -> xtm60_left_link`.
  The dynamic `odom -> base_link` sample had expired because the finite simulator had
  completed; source ownership and successful live mapping runs provide the remaining
  evidence. Automated edge-conflict monitoring remains part of A-R-010.

## Build and test results

### Stage A clean build

- Empty output roots: `final-stage-a-build`, `final-stage-a-install`, and
  `final-stage-a-log` under the temporary review directory.
- Result: 16 packages built.
- Result: 66 tests, 0 errors, 0 failures, 0 skipped.
- `camera_array_driver` and `smartwheel_bringup` collect zero package-local tests.

### Full workspace clean build

- Empty output roots: `final-full-build`, `final-full-install`, and `final-full-log`.
- Result: all 28 discovered packages built; FAST-LIO compiled on aarch64.
- 173 project functional tests passed.
- `livox_ros_driver2` failed five upstream lint targets: copyright, cpplint, flake8,
  lint-cmake, and uncrustify. No third-party source was modified to hide these results.

## RViz status

The mapping-v2 launch files do not start RViz and do not define an audited RViz layout.
They publish ROS topics for point clouds, occupancy products, camera images when enabled,
TF, diagnostics, and status, but those are not automatically displayed. Existing
legacy `wheelchair_*` RViz files are not evidence for this branch. A single-XT-M60 B1
RViz/diagnostic layout is required before device bringup; camera and final map layouts
belong to B4/C4/D1 respectively.

## Key commands executed

```bash
git status
git branch --show-current
git log --oneline --decorate -20
git diff
git diff --cached
git switch -c review/mapping-v2-stage-a

colcon build --base-paths src --packages-select <16 Stage A packages> \
  --build-base /tmp/.../final-stage-a-build \
  --install-base /tmp/.../final-stage-a-install --symlink-install
colcon test --packages-select <16 Stage A packages> \
  --build-base /tmp/.../final-stage-a-build \
  --install-base /tmp/.../final-stage-a-install
colcon test-result --test-result-base /tmp/.../final-stage-a-build --all

colcon build --base-paths src \
  --build-base /tmp/.../final-full-build \
  --install-base /tmp/.../final-full-install --symlink-install
colcon test --base-paths src \
  --build-base /tmp/.../final-full-build \
  --install-base /tmp/.../final-full-install

python3 -m smartwheel_sim.adversarial --output /tmp/.../adversarial_results.json
ros2 launch smartwheel_bringup sim_mapping.launch.py mapping_backend:=rtabmap ...
ros2 launch smartwheel_bringup sim_mapping.launch.py mapping_backend:=slam_toolbox ...
ros2 launch smartwheel_bringup offline_mapping.launch.py replay_rate:=1.0 ...
ros2 launch smartwheel_bringup offline_mapping.launch.py replay_rate:=2.0 ...
ros2 topic info /tf --verbose --no-daemon
ros2 topic info /tf_static --verbose --no-daemon
ros2 topic info /map --verbose --no-daemon
ros2 topic info /rtabmap/map --verbose --no-daemon
```

## Modified areas

- Simulator, mock-LIO, error metrics, and adversarial model.
- RTAB optimized keyframe cloud assembly and backend-required export.
- Mapping manager evidence/quality gates and replay timing guards.
- Map-topic and TF timestamp ownership.
- Teleop/watchdog command path.
- Third-party dependency manifests and regression tests.
- Stage A status, limitations, topic contract, defect, acceptance, and review docs.

The user's staged `docs/goal.md` was not modified and is not part of the review commits.

## Risks and disposition

The reviewed synthetic architecture is suitable to proceed to B0 documentation audit,
not to hardware connection. Real point fields, timestamp semantics, per-point timing,
units, axes, external sync, extrinsics, wheel geometry, camera calibration, and all
safety behavior remain unknown. See `STAGE_A_DEFECTS.md` for stage-specific open MAJOR
items and `STAGE_A_ACCEPTANCE.md` for the hard-gate decision.
