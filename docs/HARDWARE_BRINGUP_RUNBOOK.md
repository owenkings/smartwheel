# Phase B Hardware Bring-up Runbook

Status: prepared only. Do not execute until the user explicitly says `开始阶段B`.

Stage A has not opened or validated any physical interface. Every value collected in
Phase B must replace an explicit `null`/TODO in `config/hardware_profile.template.yaml`;
do not transfer simulation values into a real profile.

## Required Inputs

- Both XT-M60 IP addresses, official SDK path, official configuration files, and SDK
  examples/data structures.
- LiDAR centre distance, each LiDAR XYZ/RPY relative to `base_link`, height, and measured
  field-of-view overlap.
- SDK point unit, axes, fields, intensity meaning, frame timestamp definition, per-point
  timing, exposure mode, frame rate, network protocol, and threading model.
- H30 protocol/manual, port, baud, mounting XYZ/RPY, axes, rate, timestamp source, and
  orientation availability.
- Wheel radius, track width, encoder CPR, gear ratio, register source, velocity unit,
  left/right signs, and whether encoders remain powered while pushing.
- Four stable camera identifiers, modes, lens/intrinsics, distortion models, timestamps,
  and calibrated XYZ/RPY.
- Confirmation that the drive can be back-driven and how emergency stop/watchdog are
  wired.

## Ordered Procedure

1. Inspect the official XT-M60 GUI, SDK, examples, configuration, and data structures.
2. Create `docs/XT-M60_SDK_CAPABILITY_REPORT.md` with every SDK fact and source.
3. Connect one XT-M60 only; publish raw cloud and diagnostics without any preprocessing.
4. Verify scale and axes against measured targets, then record static and slow-motion bags.
5. Connect H30; verify axes, gravity, angular velocity, covariance, rate, and timestamps.
6. Run single-LiDAR FAST-LIO2 for stationary, straight, slow-turn, and small-loop tests.
7. Calibrate wheel odometry and compare it against measured distance/yaw.
8. Add the second LiDAR in `dual_map_only`; measure overlap and pairing latency.
9. Run RTAB-Map external-odometry loop closure and export 3D/2D products.
10. Add at most one forward camera for online visual loop closure.
11. Record all four cameras and validate offline XYZRGB colorization.
12. Evaluate `dual_lio` or FAST-LIVO2 only after the default pipeline passes acceptance.

## Push Mode Gate

Before `motion_mode:=push`, determine whether the encoders remain powered, whether
counts continue updating, and whether the controller permits back-driving. If feedback
is absent, `/wheel/odom` must report unavailable diagnostics and must not integrate
commanded or zero values as measured motion.

## Stop Conditions

Stop immediately on timestamp regression, invalid point scale, inconsistent axes,
unexpected motor enable/write, driver overheating/fault, TF duplication, corrupted bag,
or a component that substitutes synthetic values after a hardware read failure.

