# H30 IMU B2 Bring-up Report

Date: 2026-07-22 (Asia/Shanghai)

Branch: `feature/mapping-v2-rviz-workbench`

Decision: **B2 STATIC PASS; LEVEL ROLL/PITCH MOUNT CALIBRATION APPLIED IN TF**

This is an isolated raw-IMU result and does not by itself authorize FAST-LIO2, mapping,
navigation, or motor operation. The H30 was the only hardware device opened during each
IMU capture. No configuration, reset, or calibration command was sent to it.

## Device identity and transport

- Project device: `/dev/smartwheel_h30_imu -> /dev/ttyACM0`
- Stable by-id link:
  `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B8E666933-if00`
- USB VID:PID: `1a86:55d4`
- USB serial: `1a86_USB_Single_Serial_5B8E666933`
- Kernel driver: `cdc_acm`
- Protocol: Yesense binary stream, header `0x59 0x53`, little-endian
- Baud rate: `460800`

`/dev/ttyACM1` is a different `1a86:55d3` device with serial `5C66036979`; it was not
opened. The FD07-34R RS485 adapter at `/dev/ttyUSB0` was also not opened.

The installed udev rule matches H30 by VID:PID but not by USB serial. It is adequate for
the current single-H30 system, but a serial-specific rule is recommended if another
`1a86:55d4` device is ever added.

## Read-only 30-second stationary test

Machine-readable evidence is stored in
`docs/hardware/evidence/H30_B2_STATIC.json`. It was produced by the read-only tool
`scripts/hardware/h30_static_diagnostic.py`.

| Measurement | Result | Assessment |
| --- | ---: | --- |
| Duration | 30.005 s | PASS |
| Parsed samples | 6001 | PASS |
| Stream rate | 199.999 Hz | PASS; H30 is currently configured near 200 Hz |
| Acceleration norm, mean | 9.81378 m/s^2 | PASS |
| Acceleration norm, population stddev | 0.00639 m/s^2 | PASS |
| Angular-rate norm, mean | 0.001682 rad/s | PASS for stationary bring-up |
| Quaternion norm, mean | 0.9999995 | PASS |
| Roll, mean / stddev | +0.3726 / 0.0091 deg | small, stable mounting tilt |
| Pitch, mean / stddev | -0.2733 / 0.0050 deg | small, stable mounting tilt |
| Device timestamp samples | 0 / 6001 | LIMITATION; host receipt time is required |

Mean stationary angular rates were:

- x: `-0.0005292 rad/s` (`-0.0303 deg/s`)
- y: `-0.0001254 rad/s` (`-0.0072 deg/s`)
- z: `+0.0003216 rad/s` (`+0.0184 deg/s`)

These values are sufficiently small for raw bring-up. They are measurements, not a
temperature-characterized gyro-bias calibration.

## ROS 2 adapter verification

Only `wheelchair_sensors/imu_adapter_node` was launched. The configuration was changed
from a 100 Hz serial poll to a 200 Hz poll because the connected H30 emits approximately
200 samples/s. This avoids the regular two-sample bursts observed with the old 100 Hz
poll. `use_device_timestamp: false` is now explicit because the current stream contains
no timestamp TLV.

After rebuilding `wheelchair_bringup`:

- `/imu/data` was present and averaged approximately `199.9-200.1 Hz` over a 12-second
  observation;
- `frame_id` was `imu_link`;
- orientation, angular velocity, linear acceleration, and all configured covariance
  diagonals were present;
- one complete message was successfully received;
- the IMU node exited after the bounded test and no hardware node was left running.

Host-time publication still has USB scheduling and ROS executor jitter. A device-timestamp
profile, if supported by later H30 configuration, would be preferable for LiDAR-inertial
synchronization and must be validated before enabling `use_device_timestamp`.

## Later stationary repeat

After the remaining non-motor devices had been tested, the same read-only diagnostic was
repeated for 30 seconds. Evidence is stored in
`docs/hardware/evidence/H30_B7_STATIC_REPEAT.json`.

| Measurement | First B2 capture | Later repeat |
| --- | ---: | ---: |
| Parsed samples | 6001 | 5999 |
| Stream rate | 199.999 Hz | 199.932 Hz |
| Acceleration norm, mean | 9.81378 m/s^2 | 9.80051 m/s^2 |
| Acceleration norm, stddev | 0.00639 m/s^2 | 0.01015 m/s^2 |
| Angular-rate norm, mean | 0.001682 rad/s | 0.002093 rad/s |
| Quaternion norm, mean | 0.9999995 | 0.9999995 |
| Roll, mean / stddev | +0.3726 / 0.0091 deg | +0.5622 / 0.0081 deg |
| Pitch, mean / stddev | -0.2733 / 0.0050 deg | +0.0902 / 0.0062 deg |
| Device timestamps | absent | absent |

All six raw diagnostic checks passed again. Within each capture the attitude is stable,
but the repeat shifted by about `+0.190 deg` roll and `+0.363 deg` pitch relative to the
first capture. This is larger than either capture's short-term noise and can be caused by
a changed floor/chassis pose, settling or movement of the mounting, or a different
thermal/startup condition. With no independently level reference, the two captures do
not identify which component is sensor mounting error.

## Mounting and calibration decision

The first and repeat stationary attitudes suggest about `0.46 deg` and `0.57 deg` total
roll/pitch tilt respectively. Neither capture was used for an attitude reset because the
wheelchair chassis was not independently verified level; resetting on an unknown surface
would permanently absorb floor slope into the sensor reference. No gyro zero-bias command
was issued because the measured bias is small and a short room-temperature sample is not
a full thermal calibration. No magnetic calibration was attempted because it requires
controlled physical motion in the final installation environment.

Before FAST-LIO2 acceptance, perform these supervised checks:

1. Put the wheelchair on a verified level reference plane, let the IMU thermally settle,
   and repeat the stationary capture.
2. Apply controlled positive roll, pitch, and yaw motions to confirm all signs and the
   H30 FLU body-frame convention. Static gravity verifies the current near-level pose but
   cannot prove every dynamic axis sign.
3. Prefer a measured static `base_link -> imu_link` transform correction over a persistent
   attitude reset. The two unverified captures imply different provisional corrections
   (approximately `[-0.373, +0.273] deg` and `[-0.562, -0.090] deg` for roll/pitch), so
   neither may be applied as a calibrated value yet.
4. Run the longer `imu_utils` noise characterization only when estimating stochastic
   noise/covariance. It does not correct physical mounting tilt.
5. Perform magnetic calibration only if magnetometer-derived heading will actually be
   consumed, and repeat it after changing the installation or nearby ferromagnetic parts.

## User-confirmed level calibration

The user subsequently confirmed that the wheelchair was level, stationary, and free of
nearby disturbance. A new 60-second read-only capture produced 11,999 samples at
`199.966 Hz` and is stored in
`docs/hardware/evidence/H30_LEVEL_STATIC_PRE_CAL_60S.json`.

| Measurement | Mean / result |
| --- | ---: |
| Device roll | `+0.571160 deg` |
| Device pitch | `+0.064734 deg` |
| Roll/pitch stddev | `0.010009 / 0.007371 deg` |
| Acceleration xyz | `[-0.010782, +0.096994, +9.798939] m/s^2` |
| Angular velocity xyz | `[-0.000540, -0.000681, +0.000069] rad/s` |
| Acceleration norm | `9.799430 m/s^2` |

The raw gravity vector independently yields a physical `base_link -> imu_link` mounting
rotation of roll `+0.567121 deg` and pitch `+0.063040 deg`. This agrees with the H30
Euler output to `0.004040 deg` and `0.001694 deg`. Applying the corresponding RPY
`[0.0098981264, 0.0011002597, 0] rad` rotates the mean acceleration to approximately
`[0, 0, 9.799425] m/s^2` in `base_link`.

This correction is applied as a static TF in the current real-sensor URDF and mirrored
in the audit/layout YAML. It is recoverable and preserves the actual IMU measurement
axes needed by LIO. No persistent H30 attitude-reset command was sent. The vendor PDF
documents the GUI attitude reset but does not publish its serial frame, so guessing that
write would be unsafe.

No gyro zero-bias write was sent either. The official manual recommends it when pose
output fluctuates significantly; this 60-second attitude was stable and the measured
stationary bias remained small. Magnetic calibration was not attempted because it
requires controlled rotation around the axes rather than a static level pose.

Machine-readable calibration derivation and limitations are stored in
`docs/hardware/evidence/H30_LEVEL_MOUNT_CALIBRATION.json`.

## B2 gate conclusion

H30 identification, serial parsing, repeated stationary gravity/angular-rate checks,
quaternion sanity, ROS publication, and level roll/pitch mounting calibration pass for
isolated raw-IMU use. Three limitations remain:

1. dynamic axis signs have not been verified by controlled motion;
2. the current stream lacks device timestamps and uses host receipt time;
3. yaw and xyz extrinsics are not observable from the level static capture and remain
   to be measured; recheck roll/pitch if the bracket or chassis installation changes.

The later successful XT-M60 tests remove the earlier network-specific blocker, but this
IMU result still does not validate LiDAR/IMU time alignment, extrinsics, or FAST-LIO2.
