# Right XT-M60 Stage 1 Mapping — BLOCKED_CONFLICT

Current status: **BLOCKED_CONFLICT**. The right-lidar Stage 1 mapping entry is not authorized to start hardware or mapping processes. No runtime transform is approved for `base_link -> xtm60_right_link`.

## Current fail-closed decision

The calibration contract is `src/wheelchair_bringup/config/right_lidar_stage1_calibration_contract.json`. It is strict JSON with `schema_version=1`, contract ID `right_lidar_stage1_calibration`, `status=BLOCKED_CONFLICT`, and `runtime_transform=null`.

The deployment identity (`device_ip=192.168.1.101`, `host_bind_ip=192.168.1.100`, UDP port `7687`, interface `eno1`) identifies only the intended deployment endpoint. It is not calibration evidence and cannot authorize a transform.

The evidence is intentionally non-consumable as a runtime transform:

- the 2026-06-23 ground-plane summary reports height `0.510 m`, pitch `-1.04 deg`, roll `-9.10 deg`, and plane residual standard deviation `0.0043 m`; it is partial ground-plane evidence only;
- the first full-transform candidate from that epoch is retained only as a source reference and is `REJECTED_BAD` by its verification result;
- a recomputation from the same bag reports height `0.499 m` and residual standard deviation `0.0082 m`, but solves ground leveling only, not a complete installation transform;
- the 2026-07-24 provisional result uses translation z `0.735 m` and provisional RPY `[1.707116522011628, 0.026526854616668576, 1.5744345976274012] rad` from a different installation epoch.

Every evidence record has an installation epoch, claim scope, disposition, source, and `runtime_eligible=false`. The rejected full matrix is not copied into the contract.

## Permanent software blockers

`right_lidar_stage1_mapping.launch.py` performs no module-import-time file I/O and constructs exactly one `OpaqueFunction`. Its callback reads only the fixed contract path. Missing/unreadable/malformed/non-finite/schema-invalid contracts fail as `CONTRACT_INVALID`; `APPROVED` status or a non-null runtime transform fails as `CONTRACT_TAMPERED`; the reviewed contract always fails as `BLOCKED_CONFLICT`. No hardware, ROS node, include, process, timer, or bypass launch argument remains in this entry.

`scripts/hardware/run_right_lidar_stage1_static_acceptance.sh` is now a four-line blocker. It prints `BLOCKED_CONFLICT` to stderr and exits `78`; it must not be used to start or inspect hardware.

Unblocking requires a separately reviewed, same-installation-epoch calibration process and an explicit contract/design change. Editing the JSON to claim approval is tampering, not an approval procedure.

## Quality-report semantics

New map-product producers use `trajectory_evaluation.schema_version=1`. Endpoint displacement is the finite Euclidean displacement between the first and last valid trajectory x/y coordinates. Loop closure remains explicitly `UNAVAILABLE` with null error and source until an actual measured producer exists. Ground-truth RMSE remains a separate field and is never inferred from endpoint displacement or a legacy loop value.

Writers reject non-finite JSON, recursively expand nested Markdown keys, render null values as `UNAVAILABLE`, and retain manifest `format_version=1`. Existing historical bundles are not rewritten.

## Historical record — current commands are not executable or authorized

Everything in this section is retained only as historical evidence from 2026-07-29. It does not authorize a current radar session, launch, save/reload operation, route, or acceptance run.

The historical entry command was:

```bash
ros2 launch wheelchair_bringup right_lidar_stage1_mapping.launch.py
```

Historical save/reload examples were:

```bash
/home/nvidia/smartwheel/scripts/mapping/right_lidar_map_control.sh save /home/nvidia/smartwheel/maps/stage1/first_route
/home/nvidia/smartwheel/scripts/mapping/right_lidar_map_control.sh reload /home/nvidia/smartwheel/maps/stage1/first_route_posegraph
```

These commands are shown solely to identify the old workflow and **must not be run under the current `BLOCKED_CONFLICT` state**.

The bounded stationary run recorded in `docs/hardware/evidence/right_lidar_stage1_20260729_185017` historically observed a right-only organized XYZI stream, a saved occupancy grid and pose graph, an intensity-preserving exported bundle, and clean shutdown. Its bag was `/home/nvidia/smartwheel/bags/hardware/right_lidar_stage1_20260729_185019`; the versioned export was `/home/nvidia/smartwheel/maps/versions/right_lidar_stage1_20260729_185022`. Those artifacts remain evidence of that historical session only and do not establish a currently valid extrinsic or authorize new hardware operation.

The historical run was stationary and `hardware_validated=false`. It did not validate route motion, map growth during motion, straight/turn/small-loop behavior, RTAB-Map loop closure, final odometry accuracy, LiDAR–IMU fusion, FAST-LIO2, ground driving, physical E-stop closure, navigation, or passenger behavior. None of those capabilities is claimed as passed now.
