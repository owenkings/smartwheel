# Stage B0 Hardware Document Audit

Date: 2026-07-22
Branch: `feature/mapping-v2-rviz-workbench`
Audited HEAD: `b50c187fc9fb9d36aeef59e1ef0c618707db1597`
Decision: **B0 COMPLETE — SOURCE LEDGER SUPPLEMENTED ON 2026-07-22**

## Scope and current authorization

The user explicitly approved B0 documentation audit, B1 single XT-M60 testing, and B2
H30 testing. All four cameras, four FD07-34R sensors, one H30, and two XT-M60 units are
physically connected. The motor is not connected and must remain disabled.

B0 did not open a serial device, camera, LiDAR socket, CAN interface, or motor interface.
The existing staged user file `docs/goal.md` was not edited or unstaged.

## Source ledger

### XT-M60

- Vendor SDK checkout: `/home/nvidia/smartwheel/xtsdk_py`.
- Upstream: `https://github.com/XT-Toffuture/xtsdk_py.git`.
- Base commit: `1770ebe479dc9bdfcdff090f9b02fd61d76ebd6a`.
- Runtime: Jetson Python 3.10.12 with the matching aarch64 CPython 3.10 module and
  `libxtsdk_shared.so` present.
- The SDK checkout is not clean. Nine text files differ from the base commit, apparently
  largely from line-ending conversion, and generated logs/configuration are untracked.
  It must not be treated as a pristine vendor archive or modified during B1.
- Audited README SHA-256:
  `aa7dc6e4248ef5d18db3c41f36ab61a2edcdf7ced8d67f2154c513f8817b0295`.
- Active exported configuration SHA-256:
  `2f2595661d42fdfefa93592b4edb88916c1be340ff8c467f6fe357552a1d5c6e`.
- Vendor product manual `XT-M60产品手册-202509.pdf` (v2.0, 2025-09), SHA-256
  `CF376A4592033F3597B9F730806A65316F57D9CA19417ABA726BFBDAC5C39977`.
- User's Windows upper-computer export `C:\Users\admin\Desktop\xintan.xtcfg`, SHA-256
  `1611F653AA13B35AEA38DE844A130840342A73CA8AA1DB47910B7C8588E0D9F4`;
  repository evidence copy `docs/hardware/vendor/xintan_windows_export_20260722.xtcfg`.
- The SDK-root export and the new Windows export are different sources. The new export
  and device readback use `int_time_3=20`; the older SDK-root file used `30`.

Vendor-confirmed capabilities from the SDK README:

- XT-M60 is a Flash LiDAR with global exposure.
- Output is frame-level XYZI; there is no ring channel and all points in one frame are
  described as the same instant.
- The frame carries `timeStampS` and `timeStampNS` and point/depth/amplitude arrays.
- `setConnectIpaddress`, `setUdpDestIp`, `getDevInfo`, `getDevConfig`, device filters,
  integration time, HDR, amplitude threshold, modulation frequency, and maximum FPS are
  available.
- The 2025-09 manual defines default camera-mode coordinates as `+Z` forward, `+X` left,
  and `+Y` up. It distinguishes output/receiver FOV `120 x 45 deg` from VCSEL/emitter
  FOV `130 x 52 deg`, and specifies `160 x 60` output at `0.75 deg` angular sampling.
- The lower optical window is the VCSEL transmitter and the upper window is the receiver;
  connector-down is the documented installation orientation. The manual does not
  unambiguously locate the SDK coordinate origin within the housing.
- Neither the README nor manual establishes numeric point units or timestamp epoch/clock
  reset semantics. B1 must measure unit consistency and timestamp behaviour.

### H30 / Yesense

The following local vendor PDFs were reviewed and visually checked:

- `WHEELTEC_H30惯导模块用户手册_20251025.pdf`, SHA-256
  `A379E8929F4CD8AE76AACA2B84A417AEA7F51F5641936BCF785E84F6BD5D211C`.
- `YIS通讯协议.pdf`, SHA-256
  `7155E26794108659E648ED238B83A707AD6A55EB9E4044273F4A03852F838BF5`.
- `YIS系列产品磁场校准用户手册.pdf`, SHA-256
  `06CD31FEE21D7A4B382248683BD4FF6C3E95F198FD4916EB92D15EAD01DD07CC`.
- `IMU误差标定教程_2023.10.19.pdf`, SHA-256
  `61C16B1666DF5FE739C9D95CEA4A519A7E829EF38B04A9F0E2368C049AE71038`.

Vendor-confirmed facts:

- Default Type-C transport is UART. The reliable connection instruction uses 460800 baud;
  two pin-description paragraphs contain the apparent typo `406800`, so B2 must detect
  the actual stream rather than relying on the typo.
- Body coordinates are FLU and geographic coordinates are ENU. Euler order is Z-Y-X.
- Yesense frames start with `0x59 0x53`, use TLV data, little-endian signed 32-bit values,
  and a two-byte checksum.
- Acceleration scale is `1e-6 m/s^2`; angular velocity scale is `1e-6 deg/s` and must be
  converted to rad/s for ROS. Quaternion scale is `1e-6`.
- Supported configured output rates include 1, 2, 5, 10, 20, 25, 50, 100, and 200 Hz.
- Gyro zero-bias calibration requires the device to remain completely stationary.
- A non-zero roll/pitch caused by mounting-plane error may be handled by attitude reset,
  but no persistent reset may be written until the wheelchair reference plane is known.
- Magnetometer calibration must be performed after final installation and away from
  changing ferromagnetic/electrical interference. A 2D calibration covers a full slow
  360-degree level rotation; 3D calibration requires many orientations. It cannot be
  performed safely by an unattended remote process.
- The two-hour `imu_utils` procedure estimates stochastic noise parameters; it is not a
  substitute for mounting alignment or gyro zero-bias calibration.

### FD07-34R and cameras

- User-confirmed FD07-34R addresses are 1, 2, 3, and 4 on one RS485 bus.
- Supplied `传感器说明文档.pdf`, SHA-256
  `76BB13744956417F7A9C42A3D2FB00434E96B3D291A58E766E18BC0C876A1CBD`, is the
  FD07-3-family vendor document. Its model table identifies **FD07-34R** as the
  large-angle RS485-output variant.
- Vendor specifications applicable to this model/family include `3–300 cm` range,
  `3 cm` blind zone, nominal large-angle option around `80 deg ±10 deg`, `9600 bps`,
  addresses `0x01–0xFC`, and broadcast `0xFD`. Whether the angle is represented as a
  full cone or another convention still needs physical confirmation before changing
  geometry code.
- The current project uses actual-value holding register `0x0001`. The vendor document
  requires adjacent actual-value operations strictly greater than `100 ms`; processed
  register `0x0002` requires greater than `300 ms`.
- B0 changes the actual-value inter-sensor delay from 10 ms to 110 ms and the four-sensor
  cycle request to 2 Hz. It does not open the RS485 port.
- The old configured `0.45 rad` field of view is not a vendor-confirmed value and must
  not be treated as authoritative. Autonomous-versus-triggered measurement behaviour,
  no-target encoding, multi-sensor crosstalk, and exact cone convention remain for the
  later FD hardware stage.
- Four cameras are connected, but stable `/dev/v4l/by-id` identities, modes, intrinsics,
  timestamps, and physical role assignment remain unverified. Camera capture is deferred
  until the separately approved camera stage.

User-supplied installation measurements are provisional inputs, not calibrated TF:

- two LiDAR reference points are approximately `0.60 m` apart;
- LiDAR and camera reference height is approximately `0.735 m`;
- the LiDAR height was measured at the midpoint of the two optical windows, whose
  combined length is approximately `37 mm`;
- camera roles are left-front, right-front, left-side, and right-side;
- each front camera is approximately `0.05 m` outboard of its same-side LiDAR.

Longitudinal offsets, optical centres, side-camera positions, mounting angles, wheelchair
centreline, and the relationship between the window midpoint and SDK origin remain
unverified. These measurements must not be copied directly into TF.

## Audited addressing and fallback policy

The user's current IPs match the current branch and June development history:

- primary/left XT-M60: `192.168.0.101`, host receive address `192.168.0.100`;
- right XT-M60: `192.168.1.101`, host receive address `192.168.1.100`.

The much older `main` README mentions `192.168.0.201` for the right unit, but the actual
configs in that branch and all later branches use `192.168.1.101`. It is conflicting
historical prose, not an approved automatic fallback. Vendor example `192.168.0.117` is
also not project evidence. B1 therefore tests only the left unit at `192.168.0.101`.

## B1 execution gate

B1 may proceed under these constraints:

1. Start only the left XT-M60 process; do not start the right process, H30, cameras,
   FD07-34R, wheel stack, or motor stack.
2. Inspect host routes and ping before loading the SDK.
3. Establish a baseline with `apply_device_config=false` so integration time, HDR,
   filters, and FPS are not overwritten during the first read.
4. Setting the runtime UDP destination to `192.168.0.100:7687` is allowed because it is
   required to receive frames; do not change the radar IP or persist unrelated settings.
5. Record raw point count, fields, dimensions, frame ID, rate, timestamp monotonicity,
   finite ratio, coordinate ranges, amplitude distribution, and measured-distance scale.
6. Do not fabricate per-point time. The vendor describes global frame exposure.
7. Stop on SDK connection instability, timestamp regression, implausible scale, mixed
   packets, overheating/fault status, or any unexpected motor/serial access.

## B2 execution gate

B2 may proceed only after B1 produces a usable single-LiDAR baseline:

1. Identify the H30 by USB identity and stable symlink before opening it.
2. Start at 460800 baud, 8N1, and validate `0x59 0x53` frames and checksums.
3. Record a stationary sample before writing any persistent calibration command.
4. Measure stream rate, checksum loss, host/device timestamp behavior, mean and standard
   deviation of acceleration/angular velocity, gravity norm, quaternion norm, and mean
   roll/pitch/yaw.
5. Treat measured roll/pitch as a combination of actual wheelchair attitude, mounting
   tilt, sensor bias, and filter state. Do not silently zero it.
6. If the reference plane is known and the device remains stationary, produce a proposed
   software mounting correction. Persistent attitude reset, gyro-zero write, or magnetic
   calibration requires a documented physical procedure and before/after evidence.

## B0 conclusion

The supplemented source material and current driver implementation are sufficient for
the guarded single-left-LiDAR B1 and isolated H30 B2 already performed. They are not
sufficient to enable dual LiDAR, camera acceptance, FD07-34R safety decisions, motor
control, autonomous navigation, or passenger testing. Final extrinsics still require a
fixed mechanical installation and a reproducible calibration procedure.
