# Dual LiDAR acquisition-time compensation — 2026-09-10

## Scope and evidence

The production/formal fusion implementation is
`src/wheelchair_3d_mapping/wheelchair_3d_mapping/dual_lidar_cloud_fusion_node.py`.
The separate `dual_lidar_fusion` package is used by the Stage A simulation path.

Previously, each cloud was transformed into `base_link` at its own acquisition
time and then both were concatenated under the newer timestamp. Those are two
different moving frames: proximity of timestamps alone does not align geometry.
At 1 m/s, a 50 ms stagger produces a 5 cm displacement in a world-fixed wall.

## Implemented behavior

With `motion_compensation:=true`, each input is transformed to `base_link` at the
newer input timestamp through an explicitly selected continuous fixed frame.
This uses tf2 `lookup_transform_full`, not a latest-TF lookup or a relabelled
timestamp. Raw input messages/timestamps and intensity remain unchanged. The
output timestamp is the actual newer acquisition timestamp.

- Reject zero, malformed, duplicate/backward, future, or stale source stamps.
- Reject pairs beyond the configured acquisition-time separation.
- Reject if the exact two-time TF is missing; do not use identity/latest TF.
- Retain a bounded pending pair while its exact-time odometry TF arrives,
  preventing the newest sensor callback from continuously outrunning LIO.
- Do not emit the same consumed sensor frame again on subsequent timer ticks.
- Publish compensation state, rejection cause/count, and source pair delta in
  the status JSON. This node explicitly does not certify hardware clock sync.

The formal launch enables this compensation. `contract_fastlio` uses the
continuous `camera_init` frame. External odometry must explicitly name its
continuous frame and match the local TF parent; the existing formal TF contract
still requires `map -> camera_init -> body -> base_link`. A loop-corrected `map`
frame is never a suitable motion-compensation reference.

## What is deliberately unchanged

Formal acquisition still requires its APPROVED calibration/hardware contract,
validated timestamp provenance, and a pair separation no greater than 20 ms.
No gate is widened to declare an observed 50 ms pair formally synchronized.

The 0/50 ms emission stagger and periodic SDK realignment are optical-
interference mitigations, not geometric synchronization. This software change
does not change adapter phase settings, start/stop measurements, or write any
device configuration. Simultaneous emission must not be enabled without
resolving the documented optical interference.

Host receive timestamps can provide a common host clock but include transport/
callback latency. They do not prove exposure-time synchronization. Unknown
timestamp offsets, rolling acquisition within a frame, moving objects, and
optical interference are not solved by rigid ego-motion compensation.

## Offline checks

Tests cover a synthetic 50 ms translation, rotation using a real tf2 Buffer,
exact-time TF failure/recovery, pending-pair retention, intensity and original
timestamp preservation, deduplication, zero/future/stale/backward stamps, and
an excessive pair gap. Formal-launch tests reject ambiguous or loop-corrected
fixed frames. Run with the ROS and workspace setup files sourced:

```bash
python3 -m pytest -q src/wheelchair_3d_mapping/test
```

These are software tests, not a dual-LiDAR hardware-sync or map-quality pass.
