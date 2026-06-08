# Smartwheel capability audit - 2026-06-06

Last runtime verification: 2026-06-08.

## Current conclusion

The repository is an integrated indoor wheelchair prototype, not only a 3D
mapping demo. It has the intended three layers: RTAB-Map machine maps, Nav2
navigation maps, and a user-facing semantic/POI map. It is not yet ready for
unattended autonomous operation or passenger use.

## Implemented in code

- Dual XT-M60 adapters, TF fusion, RTAB-Map 3D cloud and projected 2D grid.
- H30 IMU plus wheel odometry EKF publishing `odom -> base_link`.
- Nav2 planning/control, named POIs, route preview, and navigation status.
- A user 2D map UI with pose, path, POIs, rooms, no-go-zone editing, mapping
  controls, map versioning, 2D save, and 3D PLY export.
- Saved map versions now record a global active navigation map. On service
  startup, a valid active or recent non-BAD map is loaded automatically when
  `SMARTWHEEL_MAP` is not explicitly set; missing/corrupt maps fall back to the
  packaged empty map.
- A startup localization coordinator supports explicit `disabled`, named-goal,
  fixed-pose, and external map-anchor modes. It retries `/initialpose`, waits
  for an AMCL pose acknowledgment, publishes structured status for the UI, and
  has no motor-control interface. Autostart remains fail-safe `disabled` until
  the charging pose or an AprilTag/UWB anchor is calibrated.
- Safety supervisor enforcing configurable normal/slow/stop/emergency distance
  bands, four ultrasonic freshness, physical/software E-stop, sensor watchdog,
  and a single `/cmd_vel_safe` motor path.
- Autonomous mapping launch with two independent gates: motor enable and an
  explicit `/autonomy/enable` arm signal. Dual lidar is mandatory for motion.
- Voice intent path for named navigation, charging POI return, stop/resume, and
  current-location replies. Microphone, STT, TTS, and speaker remain disabled
  hardware-neutral placeholders.

## Verified on stationary hardware

- H30 IMU approximately 200 Hz; EKF odometry approximately 30 Hz.
- Four ultrasonic sensors returned real measurements.
- Nav2 action server, RTAB-Map, map/grid topics, TF chain, safety supervisor,
  watchdog, and base status started with motor writes disabled.
- Existing `~/.ros/rtabmap.db` was preserved; preflight used a temporary DB.
- Left XT-M60 (`192.168.0.101`, firmware 2.34.2) produced approximately 10 Hz.
- Runtime configuration and host routes confirm left `192.168.0.101` via
  `192.168.0.100`, and right `192.168.1.101` via `192.168.1.100`; both answer
  ICMP without packet loss.
- The vendor SDK always binds local UDP 7687. The installed bind shim now pins
  each SDK process to its own host-subnet IP, preventing cross-radar false data.
- The semantic keepout node and Nav2 KeepoutFilter were loaded in the real
  stationary service on 2026-06-08. The live filter-info topic references
  `/semantic_keepout_mask`; the current semantic layer contained zero no-go
  zones. Motor control remained disabled and `/cmd_vel_safe` remained zero.
- A runtime audit found that the base driver still wrote zero-speed registers
  while `motion_control_enabled=false`. This was corrected: disabled mode now
  performs no periodic speed-register writes, while shutdown retains the
  independent stop/disable fallback. Regression tests and a stationary service
  restart confirmed `motion_initialized=false` with no command-write warnings.
- The service now automatically loaded the latest valid non-BAD saved map
  (`421x488`, 0.05 m/cell) instead of the packaged empty map. AMCL still awaits
  an initial pose, so `map -> odom` and autonomous navigation remain unavailable.
- The startup localization coordinator was loaded in the real service with
  `mode=disabled`. The UI reported `DISABLED`, zero attempts, and no target;
  the base remained read-only with both requested RPM values at zero. The full
  workspace build passed and the test suite reported 99 tests with no failures.

## Current hard blockers

- Right XT-M60 (`192.168.1.101`, firmware 2.34.1) is **confirmed hardware-faulty
  (2026-06-08)** and will not be used further. Although it answers ICMP, its TCP
  control port refuses/times out the SDK handshake (`PortOpening-Disconnected` /
  `TxRxVerifying-Disconnected`) and, when the handshake briefly succeeds, it
  reports chip ID `0 0`, stays in `Connected-Init`, and emits no point-cloud UDP.
  Port alignment (device `udp_dest_port` 7688 -> 7687 to match the bind shim) and
  single-radar isolation were both verified, ruling out software/port/dual-open
  causes. The unit needs vendor firmware repair or replacement. **The system runs
  on the left radar alone in the meantime (see single-lidar fallback below).**
- Front ultrasonic readings included values below 0.5 m. The requested safety
  policy therefore requires STOP. Clear the physical test area to greater than
  0.5 m, preferably greater than 1.5 m, before any motion test.
- Autonomous *motion* still requires both radars (the fusion node is launched
  with `allow_single_lidar_fallback:=false` whenever motors are enabled), so
  passenger/autonomous driving stays blocked until the right radar is restored.
  Stationary mapping and the full sensor/UI/Nav-planning chain run on one radar.

## Not yet implemented as product capability

- AprilTag/UWB detection and robust arbitrary-position global lidar matching.
  The startup pose coordinator and external anchor topic now exist, but the
  physical charging marker and its map pose are not calibrated; the current
  `charging` POI at `(0, 0, 0)` is a placeholder and must not be auto-enabled.
- Charging-dock precision alignment, charging contact control, and battery-low
  automatic return. “Return to charging point” currently means normal Nav2
  navigation to a named POI only.
- Preferred-route enforcement is not implemented. Semantic no-go polygons are
  now converted to a live Nav2 keepout mask and enforced by both costmaps.
- Automatic room segmentation and semantic labeling.
- Camera-based pedestrian, door-state, or docking-marker recognition in the
  safety/navigation loop. Cameras are optional display/colorization inputs.
- Passenger-rated safety validation, redundant braking, fault injection,
  regulatory validation, and long-duration reliability testing.

## Required acceptance order

1. Repair the right radar and verify both raw point topics independently and
   simultaneously at stable rates with the bind shim active.
2. Clear obstacles and obtain `READY_TO_PLAN` plus `READY_TO_ARM` from
   `scripts/check_autonomous_mapping_status.sh` while motors remain disabled.
3. Perform an off-ground motor-direction and emergency-stop test.
4. Run a supervised, empty-chair, low-speed mapping trial in a clear area.
5. Save and inspect the RTAB database, 3D PLY, 2D Nav2 map, and map-quality
   report before enabling point-to-point navigation.
