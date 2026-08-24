# Camera Array B4 Detection Report

Date: 2026-07-22; direct-port retests updated through 2026-07-23 14:25
Branch: `feature/mapping-v2-rviz-workbench`
Audited HEAD: `b50c187fc9fb9d36aeef59e1ef0c618707db1597`
Decision: **PASS WITH LIMITS — FOUR DIRECT USB3 CAMERAS AND FOUR ROS JPEG STREAMS VERIFIED**

## Physical enumeration

The user expects four cameras: left-front, right-front, left-side, and right-side. Linux
initially enumerated only three physical UVC devices (`0bda:5858 USB Camera3`):

| Physical USB path | Primary node | Alternate node | Result |
| --- | --- | --- | --- |
| `2-3.3.2` | `/dev/video0` | `/dev/video1` | real image |
| `2-3.3.3` | `/dev/video2` | `/dev/video3` | real image |
| `2-3.3.4.2` | `/dev/video4` | `/dev/video5` | real image, unstable/slow |

The six `/dev/video*` nodes do **not** represent six cameras. Each physical UVC device
creates two video nodes. No fourth USB camera or `/dev/video6` pair existed in `lsusb`,
`lsusb -t`, `/dev/v4l/by-path`, or the kernel UVC log.

During a later three-way concurrent load test, OpenCV reported corrupt MJPEG data and
the device at USB path `2-3.3.4.2` stopped opening. A fresh `lsusb`, `lsusb -t`, and
`/dev/v4l/by-path` inventory then showed only the first two physical cameras. The third
camera did not merely lose its `/dev/video4` binding: its USB device disappeared from
the downstream Corechips hub. The fourth camera never enumerated during this session.
The hub reports no over-current event, but this is consistent with a marginal hub
power/adapter/cable/port path and cannot be repaired in software while the USB device
descriptor is absent. The unprivileged `nvidia` account also cannot issue a USB device
reset, and no passwordless sudo path is configured.

All three cameras publish the same USB serial `200901010001`. The kernel
`/dev/v4l/by-id` link consequently collides and currently points only at `video4/5`;
it cannot identify all cameras. The USB path is the only stable identity observed in
this session.

## Image validation

One primary node per physical device (`video0`, `video2`, `video4`) was opened alone and
concurrently at requested MJPG `640 x 480 @ 30 Hz`. All three produced actual indoor
scene images with non-zero spatial content, normal brightness, changing frames, and no
read failure during the bounded OpenCV runs. JPEG samples are stored outside Git at:

```text
/home/nvidia/smartwheel/bags/hardware/camera_b4_20260722_203644
```

Visual inspection found:

- `video0`: clear real view, image upside down (approximately 180-degree correction);
- `video2`: clear real view, image upside down (approximately 180-degree correction);
- `video4`: clear real view, image rotated approximately 90 degrees.

These visual directions are not sufficient to assign the three device paths reliably to
left-front/right-front/left-side/right-side. No role mapping, rotation value, intrinsic,
or extrinsic was written to production configuration.

## Rate and stability result

The OpenCV diagnostic decodes to BGR like the current ROS adapter and therefore includes
CPU decode cost. A separate GStreamer V4L2/MJPEG test was used to distinguish diagnostic
overhead from camera transport.

Approximate GStreamer sequential decode rates:

| Node | Sequential rate |
| --- | --- |
| `video0` | `30.2 Hz` |
| `video2` | `25.0 Hz` |
| `video4` | `4.4 Hz` |

Approximate three-pipeline concurrent decode rates:

| Node | Concurrent average |
| --- | --- |
| `video0` | `16.3 Hz` |
| `video2` | `10.7 Hz` |
| `video4` | `5.3 Hz` |

A compressed-buffer-only test on `video4` delivered 22 MJPEG buffers in about 5.4
seconds, confirming that its approximately 4 Hz limit is upstream of JPEG decoding.
OpenCV also emitted corrupt-JPEG warnings during array testing. The kernel log did not
record a new UVC reset, bandwidth, or disconnect error during the test.

The current `camera_quad.yaml` assumes primary nodes `0/2/4/6` and generic roles
`front/left/right/rear`. It therefore cannot represent the user-confirmed four-role
layout in the current hardware state and must not be treated as validated.

## ROS transport correction

Two-camera ROS tests isolated a separate software throughput issue. Publishing
uncompressed `sensor_msgs/Image` at 640 x 480 across DDS yielded only approximately
`0.83-0.99 Hz` per camera with three streams, and `1.83-1.86 Hz` per camera with two.
The adapter now also publishes standard JPEG transport at
`/camera/<role>/image_raw/compressed`, uses per-camera callback groups, retries missing
cameras every three seconds, and serializes raw BGR frames only while a raw subscriber
is actually connected. The custom RViz camera panel now exposes a persistent
`raw`/`compressed` transport selector.

With both raw and compressed publishers enabled, but the display/diagnostic subscribed
to `compressed`, the two cameras that remained online each delivered 640 x 480 JPEG at
approximately `9.97 Hz` for 15 seconds. All messages were non-empty and decodable, and
all header timestamps increased strictly. The adapter exited with status 0. This fixes
the ROS multi-view transport bottleneck; it does not repair the missing USB devices.

## 2026-07-23 direct Orin USB3 retest

The user removed the external USB expansion hub and connected the four cameras directly
to the four Orin USB3 receptacles. This removed the external hub as a variable, but did
not make four cameras available:

- `lsusb` initially exposed only three `0bda:5858` devices, at the Orin board's internal
  Realtek USB3 hub paths `2-3.2`, `2-3.3`, and `2-3.4`;
- all three negotiated SuperSpeed `5000M`;
- no fourth camera appeared on either the USB2 or USB3 bus, and the board-hub downstream
  port not represented by those three paths produced no connection event;
- the identical serial `200901010001` still made `/dev/v4l/by-id` unusable, so all tests
  selected one `video-index0` node per `/dev/v4l/by-path`.

All three enumerated devices produced distinct real `640 x 480` images at least once.
The first four-second sequential MJPEG test measured approximately `15.00 Hz` on
`2-3.2`, `4.54 Hz` on `2-3.3`, and `14.99 Hz` on `2-3.4`. The `2-3.3` stream emitted
corrupt-JPEG warnings, UVC probe errors `-71` and `-110`, U1/U2 disable failures, and a
SuperSpeed reset. A later isolated recovery run delivered only four frames at about
`2.90 Hz`, then no frames. The device repeatedly disconnected and re-enumerated before
finally disappearing from `lsusb`.

At the project's 10 Hz publication target, direct OpenCV capture initially kept
`2-3.2` and `2-3.4` running for the 15-second concurrent window while `2-3.3` returned
no frames. The production ROS adapter test then showed that this was not sustained as a
three-camera system:

| Direct path | 15 s ROS compressed result | Decision |
| --- | --- | --- |
| `2-3.2` | 151 messages over `15.002 s`, receive rate `9.999 Hz` | pass |
| `2-3.3` | no messages; repeated reset/disconnect/re-enumeration | fail |
| `2-3.4` | 21 messages over only `2.038 s`, then disconnect/re-enumeration | fail |
| fourth camera | never enumerated | fail |

The original ROS diagnostic checked the mean interval only. A one- or two-second burst
could therefore appear to meet the rate threshold despite going silent for the rest of
the 15-second test. It now also requires a minimum message count and at least 80 percent
coverage of the requested window. The raw camera-array diagnostic also sleeps after a
failed read instead of entering a CPU-consuming tight loop.

The host has a global `/etc/modprobe.d/uvcvideo.conf` setting
`options uvcvideo quirks=128` dated 2026-05-30. USB runtime power control is `auto` with
a two-second autosuspend setting. These are legitimate follow-up variables, but were
not changed because the `nvidia` account has no non-interactive sudo and this retest was
not authorized to make system-wide driver changes. Neither setting is proven to be the
cause. No hub over-current event was logged.

This result eliminates the user's external hub as the sole explanation. It does not yet
distinguish a particular camera/cable fault from a particular Orin receptacle, the
board's internal USB hub/power path, or the historical UVC driver settings. That
distinction requires a labelled one-camera-at-a-time port-swap matrix.

## 2026-07-23 final four-camera recovery and verification

After further labelled unplug/replug work, the user's live monitor showed all four
physical cameras at the same time. The frozen inventory at 13:34:59 was:

| USB path | Capture node in this enumeration | Link speed |
| --- | --- | ---: |
| `2-3.1` | `/dev/video6` | `5000M` |
| `2-3.2` | `/dev/video4` | `5000M` |
| `2-3.3` | `/dev/video2` | `5000M` |
| `2-3.4` | `/dev/video0` | `5000M` |

All devices have the same serial, so the numeric node order above is not persistent.
The four `/dev/v4l/by-path/...video-index0` links are the valid identities.

The previous development branch explicitly requested by the user was
`feature/fastlio-narrow-fov-mapping` at `45b1163`. Its `camera_quad.yaml` used
`/dev/video0/2/4/6`, MJPG, `640 x 480`, and requested 30 fps. Its camera adapter used
ordinary OpenCV `VideoCapture`; there was no special driver or enumeration workaround.
Those camera files are unchanged between `45b1163` and the current committed
`b50c187` baseline. The previous branch could display four real panels whenever Linux
actually exposed four capture nodes, matching the user's recollection.

The same `0/2/4/6`, MJPG, `640 x 480 @ 30` request was repeated against the recovered
inventory. All four cameras opened individually and together, produced different real
images, passed the non-black/non-frozen/content/resolution checks, and had zero read
failures:

| Node | Sequential frames / measured rate | 15 s concurrent frames / measured rate |
| --- | ---: | ---: |
| `/dev/video0` | `40 / 15.206 Hz` | `216 / 14.664 Hz` |
| `/dev/video2` | `62 / 23.864 Hz` | `273 / 18.541 Hz` |
| `/dev/video4` | `40 / 14.994 Hz` | `218 / 14.797 Hz` |
| `/dev/video6` | `39 / 14.990 Hz` | `218 / 14.801 Hz` |

The diagnostic's aggregate 30 fps check remained false because three devices ran near
15 fps and one near 18.5 fps, below its two-thirds-of-request threshold. That is a rate
qualification, not a four-camera detection failure. A few corrupt-JPEG warnings were
still printed, so longer stability testing remains necessary.

Four labelled GStreamer windows then opened simultaneously on the Orin desktop. A
separate 15-second ROS compressed-transport test used the four stable by-path links and
passed every sustained-window check:

| Temporary ROS test alias | USB path | Messages | Receive rate | Span |
| --- | --- | ---: | ---: | ---: |
| `front` | `2-3.4` | 146 | `9.756 Hz` | `14.863 s` |
| `left` | `2-3.3` | 147 | `9.830 Hz` | `14.853 s` |
| `right` | `2-3.2` | 143 | `9.533 Hz` | `14.896 s` |
| `rear` | `2-3.1` | 138 | `9.210 Hz` | `14.875 s` |

All messages were non-empty JPEG, decoded as `640 x 480`, and had strictly increasing
header timestamps. Both the adapter and diagnostic exited with status 0. The four
cameras remained enumerated at `5000M`, and no new USB disconnect or reset was recorded
during the direct or ROS verification.

The user then confirmed the persistent physical map:

| Physical camera | USB path | Legacy ROS slot/topic |
| --- | --- | --- |
| left-front | `2-3.1` | `left` / `/camera/left` |
| left-side | `2-3.2` | `front` / `/camera/front` |
| right-side | `2-3.3` | `rear` / `/camera/rear` |
| right-front | `2-3.4` | `right` / `/camera/right` |

The production `camera_quad.yaml` now pins all four
`/dev/v4l/by-path/...video-index0` devices. The `front` and `rear` names are retained
only as legacy compatibility slots; they do not represent center-front or rear-facing
physical cameras. Saved-frame inspection produced provisional rotations of left-front
`180 deg`, left-side `180 deg`, right-side `270 deg`, and right-front `180 deg`.
These require user confirmation in the labelled live view and are not calibration.

## RViz raw-view and 30-minute soak

The exact old branch's four raw RViz panels opened and displayed all four topics. A
parallel 15-second diagnostic received only 8-9 raw frames per topic, approximately
`0.67-0.73 Hz`. Raw transport is therefore available for bounded calibration work, but
the normal four-view RViz workbench should select compressed transport.

The final compressed soak ran for 30 minutes with the production by-path identities and
provisional rotations:

| Physical camera / ROS topic | Messages | Receive rate | Maximum receive gap | Result |
| --- | ---: | ---: | ---: | --- |
| left-front `/camera/left` | 16,762 | `9.312 Hz` | `0.336 s` | pass |
| left-side `/camera/front` | 17,001 | `9.444 Hz` | `0.383 s` | pass |
| right-side `/camera/rear` | 16,888 | `9.382 Hz` | `0.353 s` | pass |
| right-front `/camera/right` | 16,029 | `8.912 Hz` | `0.438 s` | pass |

Every topic passed the minimum message count, sustained rate, 80-percent window
coverage, non-empty message, expected shape/encoding, and strictly increasing header
timestamp checks. Four SuperSpeed devices remained at `5000M`; the kernel recorded no
USB reset or disconnect during the soak.

The adapter log nevertheless contained 1,493 unlabelled libjpeg warning lines,
including 359 `premature end` and two `bad Huffman` lines. Warning-line count is not a
bad-frame count, and all published messages passed the diagnostic. The foreground
wrapper also required explicit `SIGTERM` while waiting for the `ros2 run` launcher to
exit after all four diagnostics had completed; this was a cleanup limitation, not a
stream interruption.

A follow-up attribution run kept all four cameras active but placed each camera in its
own adapter process and stderr log. Over approximately two minutes, left-front,
left-side, and right-side produced zero corrupt-JPEG warning lines; right-front
`2-3.4` produced 81, including 20 `premature end` lines. A 71-second right-front-only
control still produced 20 corrupt-JPEG lines, including four `premature end` lines,
without any kernel USB reset, disconnect, or UVC error. The warnings therefore do not
require four-camera concurrency and are localized to the installed right-front
camera/cable/`2-3.4` chain. This does not yet distinguish the camera body from its cable
or receptacle; a one-variable-at-a-time A/B swap is required.

## Compressed RViz correction and sustained-rate limit

The workbench had two independent display-path defects. Its four panels defaulted to
raw transport, and the Orin did not have the Humble compressed image transport plugin
installed. The dependency was installed and declared, the saved workbench now selects
compressed topics explicitly, and the live ROS graph showed one RViz subscriber on
each `/compressed` topic with zero subscribers on all four raw topics. CameraInfo QoS
was also changed to volatile best-effort to match the real publishers.

The custom camera panel previously used `image_transport` for compressed input. That
plugin decoded every JPEG before the panel applied `MaxDisplayFps`, so the display
throttle could not reduce four-camera decode load. The panel now subscribes directly
to `sensor_msgs/CompressedImage`, applies its 10 Hz display gate before JPEG decode,
and catches a malformed-frame decode failure without stopping the subscription.

The production operator-display profile now uses camera-advertised native MJPEG at a
`320x240@30` request, four isolated adapter processes, by-path identities, a bounded
latest-frame condition-variable dispatcher, and no continuous raw serialization. Only
the known-bad right-front chain is repaired in GStreamer (`jpegdec -> jpegenc`, quality
40); the other three streams remain byte-for-byte MJPEG. This is an RViz operator-view
profile, not the future calibrated mapping/colorization profile.

A two-minute four-thread GStreamer/V4L2 isolation, without ROS or RViz, measured:

| Physical camera | Receive rate | Complete SOI/EOI JPEG failures |
| --- | ---: | ---: |
| left-front `2-3.1` | `8.996 Hz` | 0 |
| left-side `2-3.2` | `11.879 Hz` | 0 |
| right-side `2-3.3` | `12.045 Hz` | 0 |
| right-front `2-3.4` (quality-40 repair) | `7.255 Hz` | 0 |

The final two-minute ROS publish test, with four additional `ros2 topic hz`
subscribers and no RViz, measured left-side `8.175 Hz`, left-front `8.122 Hz`,
right-side `8.096 Hz`, and right-front `4.925 Hz`. All four processes remained alive;
there was no kernel USB/UVC reset, disconnect, timeout, or bandwidth error, and Orin
was in MAXN at only about `48-49 degC`. The result therefore is not thermal throttling
or USB re-enumeration. The right-front long-window rate remains a failure against the
`7.5 Hz` display target.

Short RViz tests can exceed the target, but longer tests repeatedly expose the same
right-front degradation. Candidate changes were retained as negative evidence: 15 Hz
timer qualification, one-process four-camera decode, JPEG repair quality 70, native
right-front passthrough, and right-front `160x120` scaling did not close the sustained
limit. The `160x120` candidate was slower and was reverted. Software work materially
improved the normal four-view path, but it cannot certify the current right-front
camera/cable/receptacle chain as smooth or clean.

## Evidence and recovery conditions

- `docs/hardware/evidence/CAMERA_ARRAY_B4.json`
- `docs/hardware/evidence/CAMERA_DIRECT_CONCURRENT_640.json`
- `docs/hardware/evidence/CAMERA_TWO_COMPRESSED_FRONT.json`
- `docs/hardware/evidence/CAMERA_TWO_COMPRESSED_LEFT.json`
- `docs/hardware/evidence/CAMERA_FRONT_ROS_INDEPENDENT.json`
- `docs/hardware/evidence/CAMERA_LEFT_ROS_INDEPENDENT.json`
- `docs/hardware/evidence/CAMERA_RIGHT_ROS_INDEPENDENT.json`
- `docs/hardware/evidence/CAMERA_THREE_ROS_INDEPENDENT.json`
- `docs/hardware/evidence/CAMERA_DIRECT_USB3_20260723.json`
- `docs/hardware/evidence/CAMERA_DIRECT_USB3_10HZ_20260723.json`
- `docs/hardware/evidence/CAMERA_DIRECT_USB3_PORT33_RECOVERY_20260723.json`
- `docs/hardware/evidence/CAMERA_DIRECT_USB3_THREE_COMPRESSED_20260723.json`
- `docs/hardware/evidence/CAMERA_DIRECT_USB3_TWO_COMPRESSED_20260723.json`
- `docs/hardware/evidence/CAMERA_FOUR_45B_COMPAT_20260723.json`
- `docs/hardware/evidence/CAMERA_FOUR_ROS_COMPRESSED_20260723.json`
- `docs/hardware/evidence/CAMERA_FOUR_RVIZ_RAW_20260723.json`
- `docs/hardware/evidence/CAMERA_SOAK30_LEFT_FRONT_20260723.json`
- `docs/hardware/evidence/CAMERA_SOAK30_LEFT_SIDE_20260723.json`
- `docs/hardware/evidence/CAMERA_SOAK30_RIGHT_FRONT_20260723.json`
- `docs/hardware/evidence/CAMERA_SOAK30_RIGHT_SIDE_20260723.json`
- `docs/hardware/evidence/CAMERA_MJPEG_WARNING_ISOLATION_20260723.json`
- `docs/hardware/evidence/CAMERA_15HZ_LEFT_FRONT_20260723.json`
- `docs/hardware/evidence/CAMERA_15HZ_LEFT_SIDE_20260723.json`
- `docs/hardware/evidence/CAMERA_15HZ_RIGHT_FRONT_20260723.json`
- `docs/hardware/evidence/CAMERA_15HZ_RIGHT_SIDE_20260723.json`
- `docs/hardware/evidence/CAMERA_RVIZ_COMPRESSED_GRAPH_20260723.txt`
- `docs/hardware/evidence/CAMERA_RVIZ_COMPRESSED_RATE_LEFT_FRONT_20260723.txt`
- `docs/hardware/evidence/CAMERA_RVIZ_COMPRESSED_RATE_LEFT_SIDE_20260723.txt`
- `docs/hardware/evidence/CAMERA_RVIZ_COMPRESSED_RATE_RIGHT_FRONT_20260723.txt`
- `docs/hardware/evidence/CAMERA_RVIZ_COMPRESSED_RATE_RIGHT_SIDE_20260723.txt`
- `docs/hardware/evidence/CAMERA_RVIZ_COMPRESSED_LEFT_FRONT_20260723.json`
- `docs/hardware/evidence/CAMERA_RVIZ_COMPRESSED_LEFT_SIDE_20260723.json`
- `docs/hardware/evidence/CAMERA_RVIZ_COMPRESSED_RIGHT_FRONT_20260723.json`
- `docs/hardware/evidence/CAMERA_RVIZ_COMPRESSED_RIGHT_SIDE_20260723.json`
- `docs/hardware/evidence/CAMERA_GSTREAMER_THREADED_LONG_20260723.json`
- `docs/hardware/evidence/CAMERA_OPTIMIZED_PUBLISH_LONG_LEFT_FRONT_20260723.txt`
- `docs/hardware/evidence/CAMERA_OPTIMIZED_PUBLISH_LONG_LEFT_SIDE_20260723.txt`
- `docs/hardware/evidence/CAMERA_OPTIMIZED_PUBLISH_LONG_RIGHT_FRONT_20260723.txt`
- `docs/hardware/evidence/CAMERA_OPTIMIZED_PUBLISH_LONG_RIGHT_SIDE_20260723.txt`
- `docs/hardware/evidence/CAMERA_OPTIMIZED_SINGLE_DECODER_STRESS5_FAILED_20260723.json`
- `docs/hardware/evidence/CAMERA_OPTIMIZED_SOAK2_LEFT_FRONT_20260723.json`
- `docs/hardware/evidence/CAMERA_OPTIMIZED_SOAK2_LEFT_SIDE_20260723.json`
- `docs/hardware/evidence/CAMERA_OPTIMIZED_SOAK2_RIGHT_FRONT_20260723.json`
- `docs/hardware/evidence/CAMERA_OPTIMIZED_SOAK2_RIGHT_SIDE_20260723.json`
- `scripts/hardware/camera_array_diagnostic.py`
- `scripts/hardware/camera_ros_diagnostic.py`
- `scripts/hardware/camera_port_identify.sh`

Before the camera installation can be treated as complete:

1. visually confirm the provisional rotations in the labelled live view;
2. A/B swap the right-front camera body, cable and receptacle one variable at a time to
   distinguish the source within the confirmed `2-3.4` installed chain;
3. only if disconnects recur, isolate camera/cable versus receptacle and then evaluate
   USB autosuspend and the historical `quirks=128` setting one variable at a time;
4. calibrate intrinsics/extrinsics after the mounting and role map are fixed.

This result validates four-camera basic enumeration, four distinct real images,
15-second concurrent direct capture, the user-confirmed physical by-path map, the
earlier 30-minute ROS JPEG soak, and the corrected compressed RViz subscription path.
It does not validate the current right-front chain against the sustained 7.5 Hz target,
final rotation, clean-MJPEG quality, synchronization, calibration, visual odometry,
color mapping, or camera safety use.
