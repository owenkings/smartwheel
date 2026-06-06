# Smartwheel capability audit - 2026-06-06

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
- The vendor SDK always binds local UDP 7687. The installed bind shim now pins
  each SDK process to its own host-subnet IP, preventing cross-radar false data.

## Current hard blockers

- Right XT-M60 (`192.168.1.101`, firmware 2.34.1) remains in
  `Connected-Init`, reports chip ID `0 0`, and emits no point-cloud UDP. Raw
  traffic inspection found only its discovery broadcast. It needs a full power
  cycle, vendor firmware repair/upgrade, or replacement, followed by isolated
  and dual-radar 10 Hz acceptance tests.
- Front ultrasonic readings included values below 0.5 m. The requested safety
  policy therefore requires STOP. Clear the physical test area to greater than
  0.5 m, preferably greater than 1.5 m, before any motion test.
- Because these gates failed, independent autonomous motion was not armed and a
  formal autonomous map was not generated. This is the correct fail-closed
  result.

## Not yet implemented as product capability

- Automatic startup relocalization using AprilTag/charging marker, UWB, or
  robust global lidar matching. The UI currently supports manual initial pose.
- Charging-dock precision alignment, charging contact control, and battery-low
  automatic return. “Return to charging point” currently means normal Nav2
  navigation to a named POI only.
- Nav2 enforcement of semantic no-go zones and preferred routes. They are
  currently stored and rendered but are not keepout/speed-filter masks.
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
