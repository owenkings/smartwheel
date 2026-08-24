# XT-M60 Single-LiDAR B1 Bring-up Report

Date: 2026-07-22
Branch: `feature/mapping-v2-rviz-workbench`
Audited HEAD: `b50c187fc9fb9d36aeef59e1ef0c618707db1597`
Device under test: left XT-M60 at `192.168.0.101`
Decision: **B1 PASS WITH LIMITS — SINGLE-LEFT BASIC LINK AND RAW CLOUD ONLY**

## Scope and safety isolation

- Only the left XT-M60 SDK/ROS process was started. The right SDK, H30, cameras,
  FD07-34R bus, wheel/encoder stack, motor, CAN, and RS485 remained stopped.
- The final baseline forced `apply_device_config=false`; no radar IP, saved imaging
  parameter, saved filter, HDR, integration time, or FPS value was written.
- The SDK runtime UDP destination was set to `192.168.0.100:7687`, which is required to
  receive frames. This runtime operation must not be described as a completely
  write-free session.
- The motor is not connected and was not touched. The existing staged user file
  `docs/goal.md` was not edited, unstaged, or included in any commit.

This PASS does not validate dual-LiDAR operation, FAST-LIO2, base TF/extrinsics, time
sync, mapping quality, cameras, FD07-34R, encoders, motors, emergency stop, navigation,
or passenger safety. Project-level `hardware_validated=false` remains required.

## Recovery chronology and network evidence

The first B1 attempt failed before SDK startup: `eno1` had carrier but no RX traffic,
ARP resolution failed, and configured/historical candidates returned no ICMP reply.
That failure is retained as physical recovery history rather than erased.

After the user restored/confirmed the connected equipment, the same existing host
configuration became usable:

```text
interface: eno1, UP/LOWER_UP
host addresses: 192.168.0.100/24 and 192.168.1.100/24
left radar:  192.168.0.101, MAC 78:22:f6:33:c6:34
right radar: 192.168.1.101, MAC 78:22:f6:33:c7:37
```

The final left gate was 20/20 ping replies, 0% loss, with RTT min/mean/max
`0.679/0.790/1.365 ms`. The right address also became ARP/ICMP reachable, but only its
network reachability was checked; its SDK, identity, configuration, and point stream
were not opened. The user-confirmed gateways are left `192.168.0.1` and right
`192.168.1.1`, both with `/24` masks. These gateways are not involved in same-subnet
host-to-radar traffic.

## Device identity and stored configuration

Read-only SDK queries identified the left unit as:

| Field | Value |
| --- | --- |
| Serial/model | `XTM60B20250324000151` |
| Firmware | `XTFW-RT-V2.34.2` |
| Boot version | `BT-N1066-V0.28` |
| Device IP/mask/gateway | `192.168.0.101 / 255.255.255.0 / 192.168.0.1` |
| Saved UDP destination | `192.168.0.100:7687` |

The stored integrations `[1600, 200, 20, 1600]`, grayscale integration `2000`, HDR `1`,
minimum amplitude `70`, maximum FPS `10`, and the first four modulation selections
agree with the 2026-07-22 Windows export where comparable. The SDK example consumes
four exported frequency values and supplies enum `2` as the fifth API argument; the
export's separate `freq5` key was therefore not treated as the same field.

The device API reports stored `imgType=1`, while the Windows export and requested SDK
stream use `image_type=4`. Their enum/field equivalence is not established, so the
report does not claim the entire saved configuration is identical to the export.

Relevant configuration source:

- Windows export `C:\Users\admin\Desktop\xintan.xtcfg`, SHA-256
  `1611F653AA13B35AEA38DE844A130840342A73CA8AA1DB47910B7C8588E0D9F4`.
- Repository copy: `docs/hardware/vendor/xintan_windows_export_20260722.xtcfg`.
- The older SDK-root export has a different hash and `int_time_3=30`; it remains
  historical evidence, not the selected 2026-07-22 value.

Normal left bring-up now defaults to `apply_device_config=false`. The verified values
remain in YAML for an explicit future configuration operation, but routine startup must
not rewrite the radar automatically.

## Raw SDK frame baseline

The bounded raw diagnostic received all 20 requested frames:

- image type requested: `4`;
- dimensions: `160 x 60`;
- point, distance, and amplitude arrays: `9600` entries each;
- SDK frame interval: mean `0.100 s`, range `0.098–0.102 s`;
- SDK timestamp: strictly increasing, around `5802–5803 s`.

The SDK timestamp is non-Unix and uptime-like, but the manufacturer source reviewed here
does not prove its epoch or reset semantics. ROS acceptance therefore uses host time
with `use_sdk_timestamps=false`.

For 15,716 finite samples, `norm(SDK XYZ) / raw distance unit` averaged
`0.0009999999977 m/unit`. This supports raw distance units behaving as millimetres and
SDK XYZ already being metres, so `point_unit_scale=1.0` is correct for this SDK build.
This is an internal consistency check, not an external tape-measure range calibration.

## Published point cloud and invalid-pixel correction

The vendor SDK documents amplitude `0–2039` as valid and `>=64000` as invalid-pixel
markers. A repeat scene exposed `64001` in the organized cloud, where it had previously
been published as a real intensity. The adapter was corrected so:

- unordered output drops such points;
- organized output preserves the `160 x 60` pixel position as
  `(NaN, NaN, NaN, 0)`;
- exact boundary `64000`, value `64001`, and valid boundary `2039` are covered by tests.

Final live diagnostic (`XT_M60_LEFT_B1_CLOUD.json`):

| Metric | Result |
| --- | --- |
| Duration/messages | `20.063 s`, `200` clouds |
| Receive/header rate | `9.947 / 9.950 Hz` |
| Header regressions | `0` |
| Layout | organized `160 x 60`, XYZI float32, point step `16` |
| Frame | `xtm60_left_link` |
| Total/finite pixels | `1,920,000 / 374,359` (`19.498%`) |
| Finite range | `0.507–4.067 m`, median `1.257 m` |
| Published intensity | `48–894`, median `265` |
| Invalid amplitude sentinels | none published as finite points |

All nine automated cloud checks passed, including 10 Hz rate, strict timestamps,
organized dimensions, XYZI fields, metre-scale plausibility, no `>=64000` sentinel,
and intensity within the documented range. The finite ratio is scene/config dependent;
it is recorded for comparison and is not by itself a mapping-quality acceptance value.

The host poll timer was changed from 10 to 20 Hz to avoid phase-loss against the device's
10 Hz latest-frame buffer. It does not change the radar output rate.

## Bag, replay, and RViz evidence

Final filtered bag:

```text
/home/nvidia/smartwheel/bags/hardware/b1_xtm60_left_filtered_20260722_200403
duration: 21.294838349 s
size: 31.7 MiB
/xtm60/left/points: 213 messages
/xtm60/left/status: 427 messages
```

Replay of that exact bag produced 213 clouds at `10.022 Hz`; all nine checks passed,
maximum intensity remained `894`, and no header regression or invalid sentinel appeared.
The live and replay JSON files include the bag path in their `source` field.

A dedicated `xtm60_b1.rviz` configuration successfully started on display `:1` with
fixed frame `xtm60_left_link`; `/rviz` subscribed to `/xtm60/left/points` using sensor
data/Best Effort QoS. No screenshot was retained to avoid capturing unrelated desktop
content. This proves process/config/topic compatibility only, not base TF, extrinsics,
scene accuracy, or FAST-LIO2 suitability.

## Coordinates, temperature, and provisional installation measurements

The 2025-09 vendor manual defines default camera-mode axes as `+Z` forward, `+X` left,
and `+Y` up. A five-frame floor fit found all used points at positive Z and the floor at
approximately `y=-0.791 m`, supporting `+Z` forward and `+Y` up for this stream. The
`+X` sign comes from the manual; no controlled moving target independently tested it.

The floor-plane sensor distance estimate was `0.788 m` with normal
`[0.0442, 0.9960, -0.0782]`, 19.47% inliers, and 95th-percentile residual `26.6 mm`.
The user's tape measurement is approximately `0.735 m`, referenced to the midpoint of
the two optical windows. The `53 mm` difference is retained. Possible contributors are
an SDK origin not located at that midpoint, mounting/floor tilt, reference-point error,
or plane-fit uncertainty. Neither value is promoted to a final TF.

The manual identifies the lower window as the VCSEL transmitter and the upper window as
the receiver, but does not unambiguously locate the SDK coordinate origin. Connector-down
is the documented installation orientation. The user-reported two-window combined length
is about `37 mm`.

Other user-measured provisional geometry:

- left/right LiDAR reference spacing: approximately `0.60 m`;
- four cameras: left-front, right-front, left-side, right-side;
- each front camera is approximately `0.05 m` outboard of its same-side LiDAR;
- LiDAR and camera reference height: approximately `0.735 m`.

These are measurement inputs, not calibrated extrinsics. Symmetric `±0.30 m` LiDAR
offsets or a `0.70 m` front-camera spacing may only be inferred after the wheelchair
centreline and reference points are confirmed. Longitudinal offsets, side-camera
positions, camera optical centres, and all roll/pitch/yaw values remain unknown.

The 20-frame SDK temperature sample reported sensor `49.73–51.77 °C` and VCSEL
`42.8 °C`, with no fault or over-temperature event observed. These internal readings are
not the same as enclosure or ambient temperature; no thermal soak acceptance was done.

## Software verification

- `wheelchair_sensors` and `wheelchair_bringup` built successfully after the changes.
- Isolated `wheelchair_sensors` result: **15 tests, 0 errors, 0 failures, 0 skipped**.
- A workspace-wide `colcon test-result` also reports old 2026-07-15 third-party
  `livox_ros_driver2` lint failures. The package-limited result above is the applicable
  B1 regression result; no third-party test was disabled or modified.
- One automation cleanup initially waited on the `ros2 run` wrapper rather than its
  child. The exact child received SIGINT, exited, and the same bag then completed replay.
  Final process checks found no B1 hardware process left running.

## Artifacts

- `docs/hardware/evidence/XT_M60_LEFT_DEVICE_INFO.json`
- `docs/hardware/evidence/XT_M60_LEFT_B1_SDK_FRAMES.json`
- `docs/hardware/evidence/XT_M60_LEFT_B1_CLOUD.json`
- `docs/hardware/evidence/XT_M60_LEFT_B1_BAG_REPLAY.json`
- `docs/hardware/evidence/XT_M60_LEFT_B1_GROUND_PLANE.json`
- `scripts/hardware/xtm60_device_info.py`
- `scripts/hardware/xtm60_sdk_frame_diagnostic.py`
- `scripts/hardware/xtm60_cloud_diagnostic.py`
- `scripts/hardware/xtm60_ground_plane_diagnostic.py`
- `src/wheelchair_bringup/rviz/xtm60_b1.rviz`

## Gate conclusion

B1 is complete for the **single left XT-M60 basic network/SDK/raw-cloud path**. It is now
reasonable to preserve this baseline and stop. B2 was already completed as an isolated
H30 test while the first B1 network attempt was blocked. No additional hardware stage is
authorized by this PASS; wait for explicit user direction before dual-LiDAR, FAST-LIO2,
camera capture, FD07-34R, encoder, motor, mapping, or navigation work.
