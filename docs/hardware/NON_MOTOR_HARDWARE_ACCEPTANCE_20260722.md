# Non-Motor Hardware Detection Summary

Date: 2026-07-22; camera direct-port retests updated through 2026-07-23 14:25 (Asia/Shanghai)
Branch: `feature/mapping-v2-rviz-workbench`
Audited HEAD: `b50c187fc9fb9d36aeef59e1ef0c618707db1597`
Overall decision: **PARTIAL — ALL AVAILABLE NON-MOTOR PATHS WERE CHECKED, WITH PHYSICAL BLOCKERS**

## Safety and change boundary

- The motor was not connected, enabled, commanded, or accessed.
- The ZLAC check used Modbus function `0x03` only. The normal ZLAC ROS driver was not
  launched because its shutdown path writes an emergency-stop control word even when
  motion is disabled.
- The two XT-M60s were explicitly configured once from the verified Windows upper-
  computer export. Normal startup remains `apply_device_config=false`; later measurement
  sessions performed only the SDK-required runtime UDP destination request.
- H30 device tests were read-only. Level roll/pitch correction was applied in the
  recoverable ROS static TF; no attitude reset, zero-bias, magnetic calibration, or
  persistent device command was sent.
- FD07-34R tests used only function `0x03`, actual-value register `0x0001`.
- No commit or push was made. Each completed device group was explicitly staged, while
  the user's pre-existing staged `docs/goal.md` remained untouched.

## Result matrix

| Device/path | Result | Measured evidence | Remaining issue |
| --- | --- | --- | --- |
| Left XT-M60 | Pass with scope limits | organized `160×60` XYZI at about 10 Hz; runtime TF restored with historical reliable roll/pitch | current floor fit invalid; no final yaw/xyz, synchronization, fusion, or mapping validation |
| Right XT-M60 | Pass with limits | raw/ROS about 10 Hz; 30-frame floor fit corrected axes/tilt and restored runtime TF | yaw constrained not measured; x provisional; amplitude exceeds documented `0–2039` |
| Dual XT-M60 | Pass with limits | simultaneous two-instance acquisition; one runtime TF owner for both radar frames | no hardware time-sync, dual-cloud alignment, final extrinsic, fusion, or mapping acceptance |
| H30 IMU | Pass with limits | 60 s level run at `199.966 Hz`; gravity-derived mount R/P `+0.567/+0.063 deg` applied in TF | no device timestamp; no dynamic-axis/yaw/xyz extrinsic test |
| FD07-34R addresses 1–4 | Pass with limits | post-fix 160/160 replies; minimum request-start interval `110.143 ms`; four ROS topics near 2 Hz | FOV convention, no-target behavior, crosstalk and supply margin unconfirmed |
| Camera array | Pass with limits; right-front rate failed | four labelled SuperSpeed by-path devices; earlier 30-minute soak `8.912-9.444 Hz`; corrected compressed RViz graph | optimized two-minute profile: three paths `8.096-8.175 Hz`, right-front only `4.925 Hz`; right-front corrupt-JPEG chain, rotations and calibration remain |
| ZLAC/encoders | Blocked without motor/controller side | USB bridge opens, but 50/50 queries return only local echo and zero controller bytes | cannot validate encoder feedback until controller/motor side is connected and powered under a separately approved safety test |

## Camera details

The early history remains relevant: both external-hub and first direct-Orin sessions
exposed only two or three devices under load, with corrupt JPEG, UVC `-71/-110`,
USB3 reset/disconnect/re-enumeration, and one USB2 fallback. The external hub was
therefore not the sole cause. No over-current event was logged. Global
`uvcvideo quirks=128` and two-second USB autosuspend remain unmodified diagnostic
variables, not confirmed causes.

After the user reseated the four direct cables, all four `0bda:5858` cameras enumerated
simultaneously at `5000M`. They share serial `200901010001`, so `/dev/v4l/by-id`
collides and numeric `/dev/videoN` order changes after reconnect. The user confirmed
the persistent physical path map:

| Physical camera | USB path | Capture node in the verified enumeration | Legacy ROS slot/topic |
| --- | --- | --- | --- |
| left-front | `2-3.1` | `/dev/video6` | `left` / `/camera/left` |
| left-side | `2-3.2` | `/dev/video4` | `front` / `/camera/front` |
| right-side | `2-3.3` | `/dev/video2` | `rear` / `/camera/rear` |
| right-front | `2-3.4` | `/dev/video0` | `right` / `/camera/right` |

The production `camera_quad.yaml` now uses the four
`/dev/v4l/by-path/...video-index0` links. `front` and `rear` remain compatibility slot
names and must not be interpreted as center-front/rear-facing physical cameras.
Saved-frame inspection produced provisional rotations of left-front `180 deg`,
left-side `180 deg`, right-side `270 deg`, and right-front `180 deg`; the user must
still confirm orientation in the labelled live view. These are display corrections,
not intrinsic or extrinsic calibration.

The exact `feature/fastlio-narrow-fov-mapping` (`45b1163`) request
(`/dev/video0/2/4/6`, MJPG, `640x480@30`) opened all four cameras individually and
concurrently for 15 seconds. Concurrent measured rates were `14.664`, `18.541`,
`14.797`, and `14.801 Hz`; all four produced distinct non-black, non-frozen images with
zero read failures. The 30 fps qualification itself was not met.

The ROS adapter uses per-camera callback groups, rate-limited reconnect attempts,
JPEG `/image_raw/compressed` transport, and raw serialization only while a raw
subscriber exists. Four raw RViz views were visible, but a 15-second diagnostic
received only 8-9 frames per topic (`0.67-0.73 Hz`), so the normal four-view workbench
must use compressed transport.

A final four-topic compressed soak then ran for 30 minutes with these production
by-path identities and rotations:

| Physical camera / ROS topic | Messages | Receive rate | Maximum receive gap | Result |
| --- | ---: | ---: | ---: | --- |
| left-front `/camera/left` | 16,762 | `9.312 Hz` | `0.336 s` | pass |
| left-side `/camera/front` | 17,001 | `9.444 Hz` | `0.383 s` | pass |
| right-side `/camera/rear` | 16,888 | `9.382 Hz` | `0.353 s` | pass |
| right-front `/camera/right` | 16,029 | `8.912 Hz` | `0.438 s` | pass |

All four diagnostics passed message-count, sustained-rate, test-window coverage,
non-empty payload, expected shape/encoding, and strictly increasing timestamp checks.
All cameras remained at `5000M`, with no kernel USB reset or disconnect during the
soak. The foreground wrapper later required explicit `SIGTERM` while waiting for the
`ros2 run` launcher to exit; the camera diagnostics themselves had already completed
and written passing results.

The adapter log contained 1,493 unlabelled libjpeg warning lines, including 359
`premature end` and two `bad Huffman` lines. Warning-line count cannot be converted
directly into a bad-frame count, and all published messages still passed the diagnostic
checks, but this prevents an unconditional camera acceptance. A follow-up two-minute
four-process attribution run logged `0/0/0/81` corrupt-JPEG lines for
left-front/left-side/right-side/right-front. A 71-second right-front-only control still
logged 20 lines without a kernel reset or disconnect. The issue is therefore localized
to the installed right-front camera/cable/`2-3.4` chain and does not require four-camera
load; the current evidence cannot distinguish camera body, cable, or receptacle.
Camera operation is therefore **pass with limits**: role/persistence and 30-minute
transport stability pass; orientation confirmation, one-variable A/B isolation within
the right-front chain, and intrinsic/extrinsic calibration remain open.

The later RViz performance correction installed/declared compressed transport, made it
the saved workbench default, moved the custom panel's 10 Hz throttle before JPEG decode,
and changed the real adapter to a GStreamer latest-frame dispatcher so DDS publication
cannot block USB capture. The operator profile is now native MJPEG `320x240`, with only
right-front repaired at JPEG quality 40. In the final two-minute publish test, left-side,
left-front, and right-side measured `8.175`, `8.122`, and `8.096 Hz`; right-front measured
only `4.925 Hz`. Orin was MAXN at about `48-49 degC`, and no USB/UVC kernel error was
recorded. Software candidates including right-front `160x120` and native passthrough
did not fix the sustained limit. Right-front therefore remains a hardware-chain A/B
swap item and is not accepted as a smooth display stream.

## FD07-34R details

The 30-second raw poll completed 60 cycles across addresses 1–4:

- 240/240 responses passed CRC, address, function, and payload checks;
- no Modbus write function was used;
- request start spacing was at least `0.110137856 s`, satisfying the manual's strict
  `>100 ms` rule for register `0x0001`;
- median values were address 1 `708 mm`, address 2 `514 mm`, address 3 `376 mm`, and
  address 4 `254 mm` for the scene present during the test.

The original ROS adapter published about `1.406 Hz` because each approximately 94 ms
response was followed by another 110 ms sleep. It now schedules request starts 110 ms
apart, including the polling-cycle boundary, so response time counts toward the interval.
A 20-second repeat completed 160/160 valid transactions with a minimum `110.143 ms`
request spacing. The four ROS topics then delivered `1.99969-1.99985 Hz`, monotonic
headers, and in-range values. The existing `0.45 rad` field of view remains unverified
because the manual's approximately `80°±10°` beam value does not establish the exact ROS
cone convention. No voltage/current or full-system supply-margin measurement was made.

## H30 calibration decision

The user subsequently confirmed a level, stationary chassis with no nearby disturbance.
A 60-second run produced 11,999 samples at `199.966 Hz`. Mean device roll/pitch was
`+0.571160/+0.064734 deg`; raw gravity independently derived physical mounting roll/pitch
`+0.567121/+0.063040 deg`, agreeing within `0.0041/0.0017 deg`. The corresponding
`base_link -> imu_link` static TF `[0.0098981264, 0.0011002597, 0] rad` rotates mean
acceleration to approximately `[0,0,9.799425] m/s^2` and is applied in the real-sensor
URDF. Xacro generation and `check_urdf` pass.

No persistent device attitude reset was sent because the vendor manual does not publish
its serial frame. No gyro zero-bias write was needed: attitude standard deviation was
only about `0.010/0.007 deg` and the stationary bias remained small. Magnetic calibration
requires controlled rotation and was not attempted. Dynamic axis signs, yaw/xyz
extrinsics, and device timestamps remain open.

## Acceptance boundary and next action

The available non-motor hardware has now been exercised to the extent possible without
energizing the motor/controller side. This does **not** make the wheelchair hardware-
validated and does not open passenger, navigation, FAST-LIO2, RTAB-Map, physical emergency
stop, or autonomous-motion acceptance.

The immediate physical actions are:

1. visually confirm the provisional four-camera rotations, then A/B swap the right-front
   camera body, cable, and receptacle one variable at a time;
2. calibrate camera intrinsics and final camera/base extrinsics after mounts are fixed;
3. if disconnects recur, use the labelled camera/cable/receptacle matrix and evaluate
   autosuspend/`quirks=128` only one variable at a time;
4. perform controlled H30 positive roll/pitch/yaw motions and measure yaw/xyz extrinsics;
5. obtain vendor clarification for right-radar amplitudes `2040–2165`; the SDK shutdown
   fault itself has been fixed and both adapters now exit with status 0;
6. only under a new explicit motor-safety authorization, power/connect the controller and
   test encoder feedback without commanding motion.

## Detailed reports

- `docs/hardware/XT_M60_SINGLE_BRINGUP_REPORT.md`
- `docs/hardware/XT_M60_DUAL_BRINGUP_REPORT.md`
- `docs/hardware/H30_IMU_BRINGUP_REPORT.md`
- `docs/hardware/FD07_ARRAY_REPORT.md`
- `docs/hardware/CAMERA_ARRAY_B4_REPORT.md`
- `docs/hardware/ZLAC_ENCODER_READONLY_REPORT.md`
