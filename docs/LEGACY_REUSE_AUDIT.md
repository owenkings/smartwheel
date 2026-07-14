# Legacy Reuse Audit

Audit date: 2026-07-14

This audit is the boundary between the legacy `wheelchair_*` stack and the clean
mapping-v2 packages. Legacy code remains in place and is never launched by the new
Stage A entry points.

## Eligible for Refactored Reuse

These behaviors have useful tests or clear safety properties. Mapping v2 may reproduce
the behavior behind new interfaces, but must not import a legacy node wholesale.

| Behavior | Evidence | Decision |
| --- | --- | --- |
| W/A/S/D and Space semantics | `wheelchair_bringup/src/teleop_panel.cpp` tracks held keys, combines linear/angular input, and stops on release/Space | Reimplement as headless, testable controller logic with timeout, ramp, deadman, and hardware gate |
| Differential-drive equations | `wheelchair_base/kinematics.py` and `test/test_kinematics.py` cover twist/RPM round-trip and integration | Reimplement with encoder-count input, overflow handling, signs, CPR, ratio, covariance, and no hardware defaults |
| Motor stop ordering | `wheelchair_base/test/test_zlac_shutdown.py` verifies zero then disable behavior | Preserve as a Phase B adapter contract; Stage A has no serial transport |
| Modbus CRC and signed conversion | `wheelchair_base/modbus_rtu.py` and tests cover framing helpers | Reference only until the exact controller manual and register map are supplied |

No real motor protocol, register address, serial path, encoder scale, or direction is
declared verified by this audit. The old code contains model- and installation-specific
defaults, so the new Stage A stack intentionally has no real motor backend.

## Reference Only

The following may inform tests and failure modes, but must not be imported or included
from mapping-v2 launch files:

- `wheelchair_sensors`: legacy XT-M60, H30, camera, mock, and ultrasonic nodes.
- `wheelchair_description`: legacy URDF and static transform configuration.
- `wheelchair_bringup`: legacy launch composition and profiles.
- `wheelchair_3d_mapping`: old FAST-LIO adapter, cloud fusion, RGB colorizer, mapping
  launch, and occupancy projection.
- `wheelchair_mapping`: old save and post-processing scripts.
- `wheelchair_navigation`: old state machines and navigation workflow.
- `wheelchair_safety` and `wheelchair_diagnostics`: useful policy ideas, but topic and
  ownership assumptions belong to the legacy graph.

Specific reasons include mixed responsibilities, assumptions about source timestamps,
hard-coded or installation-derived defaults, multiple optional TF publishers, and
launch graphs that can compose conflicting state estimators or map backends.

## Must Be Retired from Mapping V2

- Hard-coded real device paths such as `/dev/smartwheel_zlac8030` and `/dev/ttyACM0`.
- Hard-coded IP addresses, register maps, dimensions, wheel signs, and extrinsics.
- Any launch combination with more than one publisher for `map -> odom` or
  `odom -> base_link`.
- Unconditional latest-frame dual-LiDAR concatenation without source-time pairing.
- Re-stamping sensor data to wall time as a silent repair for incompatible clocks.
- Publishing synthetic or commanded values as measured real feedback after a read
  failure.
- Combining mock and real backends inside a node without a fail-closed backend gate.
- All FD07-34R/ultrasonic nodes, topics, configuration, tests, and launch includes.

## Reuse Rules

1. New packages may depend only on `smartwheel_*`, standard ROS packages, and pinned
   third-party dependencies.
2. Hardware adapters expose protocol-neutral interfaces. Vendor details remain explicit
   TODOs until Phase B capability reports exist.
3. `hardware_enabled` defaults to `false`; false means no transport object is created.
4. Mock messages are marked by mock profiles and never masquerade as hardware data.
5. TF edges have one documented owner and launch-time uniqueness checks.

