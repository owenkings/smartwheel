# Provisional Sensor Installation Measurements

Recorded: 2026-07-22
Source: user measurements and XT-M60 vendor manual
Status: **measurement input only — not calibrated extrinsics**

## Confirmed layout description

- Two XT-M60 units are mounted left and right. Their user-measured reference-point
  spacing is approximately `0.60 m`.
- Four cameras are installed as left-front, right-front, left-side, and right-side.
- Each front camera is approximately `0.05 m` farther outboard than the XT-M60 on the
  same side.
- LiDAR and camera reference height is approximately `0.735 m` above the floor.
- For each XT-M60, the height reference was the midpoint of the two optical windows. The
  user measured the combined two-window length as approximately `37 mm`.

The XT-M60 2025-09 manual identifies the **lower window as the VCSEL transmitter** and
the **upper window as the receiver**. It defines default camera-mode axes as `+Z`
forward, `+X` left, and `+Y` up, but it does not unambiguously identify the SDK coordinate
origin inside the housing.

## Values that must not yet be written into TF

If the two LiDAR reference points are symmetric about the wheelchair centreline, their
provisional lateral offsets would be approximately `±0.30 m`. If camera and LiDAR
reference points are comparable, the two front-camera references would be approximately
`0.70 m` apart. Neither inference is currently established and neither is an approved TF.

Unknowns include:

- wheelchair centreline and `base_link` reference point;
- SDK origin relative to the window midpoint;
- longitudinal and vertical offsets between LiDAR and same-side front camera;
- exact optical centre of every camera;
- precise position and view direction of both side cameras;
- roll, pitch, and yaw for all six devices;
- installation repeatability, bracket flex, and loaded-wheelchair floor attitude.

## Height discrepancy retained for calibration

A B1 five-frame floor-plane fit estimated `0.788 m` from the LiDAR SDK origin to the
plane and `y=-0.791 m` at sensor X/Z. The user measurement is `0.735 m` at the optical
window midpoint, a difference of approximately `53 mm`.

This difference can arise from an SDK origin not at the window midpoint, mounting/floor
tilt, measurement reference error, or plane-fit/scene uncertainty. Do not replace either
measurement with the other. The floor fit had plane normal
`[0.0442, 0.9960, -0.0782]`, 19.47% inliers, and 95th-percentile residual `26.6 mm`.

## Required calibration after mechanical fixation

1. Define and physically mark `base_link`, centreline, floor plane, and each device
   reference point.
2. Obtain vendor CAD/origin information if available; otherwise establish the SDK origin
   using a reproducible planar/target experiment.
3. Measure XYZ offsets from one datum using the same tool and loaded/unloaded condition.
4. Estimate LiDAR roll/pitch from a level floor and yaw/lateral sign from controlled
   planar targets; do not use the single B1 scene as final calibration.
5. Calibrate camera intrinsics first, then camera-to-LiDAR extrinsics with synchronized
   images/clouds and a fixed calibration target.
6. Record raw bags/images, method, residuals, repeat trials, accepted thresholds, and the
   final transform version. If repeatability or residuals fail the threshold, retain the
   previous provisional status.

The user can assist with calibration, but successful accuracy is not assumed in advance.
