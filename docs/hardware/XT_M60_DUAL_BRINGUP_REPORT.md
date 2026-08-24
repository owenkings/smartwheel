# XT-M60 Right and Dual-LiDAR Bring-up Report

Date: 2026-07-22; independent-instance/per-IP export retest updated 2026-07-23
Branch: `feature/mapping-v2-rviz-workbench`
Audited HEAD: `b50c187fc9fb9d36aeef59e1ef0c618707db1597`
Devices: left `192.168.0.101`, right `192.168.1.101`
Decision: **PASS WITH LIMITS — TWO INDEPENDENT SDK INSTANCES AND DUAL LIVE COEXISTENCE**

## Scope and write boundary

- The right radar was tested alone, then both XT-M60 units were run together.
- Initial ROS bring-up forced `apply_device_config=false`. The user subsequently
  explicitly required both radars to use the Windows upper-computer export, so a
  dedicated bounded operation wrote only its supported imaging fields and captured
  before/after read-back. IP, mask, gateway, and unrelated settings were not changed.
- As required by the vendor SDK, each measurement session requested the already
  configured runtime UDP destination (`192.168.0.100:7687` for left and
  `192.168.1.100:7687` for right). This is a runtime operation and must not be
  described as a completely write-free session.
- Cameras, H30, FD07-34R, encoder/controller, motor, mapping, and navigation nodes
  remained stopped during these LiDAR tests. The motor was not accessed.
- The user's staged `docs/goal.md` was not edited, unstaged, or committed.

## Right device identity and saved configuration

The read-only SDK query identified the right unit as:

| Field | Value |
| --- | --- |
| Serial/model | `XTM60B20250324000134` |
| Firmware | `XTFW-RT-V2.34.2` |
| Boot version | `BT-N1066-V0.28` |
| Device IP/mask/gateway | `192.168.1.101 / 255.255.255.0 / 192.168.1.1` |
| Saved UDP destination | `192.168.1.100:7687` |
| Saved maximum FPS | `10` |
| Integration times before export apply | `[1600, 200, 30, 1600]` |
| Integration times after export apply | `[1600, 200, 20, 1600]` |

The right unit originally differed from the left unit's confirmed third integration
time (`20`). The authorized export operation changed the right unit from `30` to `20`,
and the YAML now matches that stored value. Routine startup still defaults to
`apply_device_config=false`, so it does not repeatedly rewrite either device. The host
polling timer is 20 Hz, matching the proven left strategy for a 10 Hz latest-frame
buffer; this does not change radar output rate.

Right reachability was 20/20 ICMP replies with no loss and RTT min/mean/max
`0.239/0.777/1.473 ms`.

## Authorized upper-computer export application

The applied source was the repository copy of the user's Windows export, SHA-256
`1611F653AA13B35AEA38DE844A130840342A73CA8AA1DB47910B7C8588E0D9F4`. Both devices
passed all eight configuration checks and continued producing 10 Hz frames. Confirmed
read-back on both devices was:

- grayscale/exposure times `2000 / [1600, 200, 20, 1600] us`;
- temporal HDR `1`, minimum amplitude `70`, maximum FPS `10`;
- modulation selections corresponding to the official SDK example's
  `0,0,0,3,2` call;
- output image type `4` after measurement start.

The SDK filters supported by the official example were also loaded from the export:
median `3`, Kalman `0.30/300` with range argument `2000`, edge `150`, dust `9000/2`,
post-process `5/1/9`, and reflective `0.5/2.0`. Export fields `int5` and the spatial
filter are not exposed by the official M60 example/API used here. Its fifth modulation
argument is hard-coded to enum `2` by the vendor example rather than consuming the
export's separate `freq5=3`; this implementation follows that example instead of
guessing a different mapping.

## Right raw SDK frames

The first bounded attempt connected and reported successful start/runtime UDP calls
but received zero frames during its 15-second frame window. The failure is preserved
as `XT_M60_RIGHT_SDK_FRAMES_BIND_SHIM_FAILED.json`.

A second isolated attempt without the per-address bind shim received all 10 requested
frames:

- `160 x 60`, with 9,600 point/distance/amplitude entries per frame;
- strictly increasing SDK timestamps and mean interval `0.0963 s`;
- 44,606 coordinate/distance comparisons with median scale approximately
  `0.001 m` per raw distance unit;
- sensor temperature `47.74 °C`, VCSEL temperature `42.3 °C`;
- no device imaging configuration was written.

The later dual test used the bind shim on both processes and both streams succeeded.
The earlier zero-frame condition was therefore transient/recoverable and was not
reproduced as a deterministic bind-shim fault.

## Right ROS cloud, bag, and replay

The right-only live diagnostic received 167 clouds over 20.014 seconds:

| Metric | Result |
| --- | --- |
| Receive/header rate | `9.971 / 9.970 Hz` |
| Layout | organized `160 x 60`, XYZI float32, point step `16` |
| Frame | `xtm60_right_link` |
| Total/finite pixels | `1,603,200 / 1,213,795` (`75.711%`) |
| Finite range | `0.135–12.589 m`, median `1.753 m` |
| Published intensity | `71–2165`, median `547` |
| Header regressions | `0` |
| Published `>=64000` sentinels | none |

Right-only bag:

```text
/home/nvidia/smartwheel/bags/hardware/b6_xtm60_right_filtered_20260722_210124
duration: 18.605140395 s
size: 25.6 MiB
/xtm60/right/points: 172 messages
/xtm60/right/status: 373 messages
```

Replay of all 172 clouds retained the organized shape, frame, strictly increasing
headers, and a `10.003 Hz` header rate. The same maximum intensity (`2165`) was
reproduced exactly.

The vendor material reviewed for B1 explicitly calls `0–2039` valid amplitude and
`>=64000` invalid-pixel markers. Values `2040–63999` are not explained by that source.
The right stream's repeatable `2154–2165` maximum therefore fails the conservative
`intensity_within_documented_range` check, although no invalid-pixel sentinel was
published as a finite point. Do not silently discard the unexplained range until the
vendor confirms its semantics or a controlled target test establishes it.

Both ROS nodes initially reproduced a segmentation fault after SIGINT. ROS 2 Humble can
consume SIGINT as `ExternalShutdownException` instead of raising `KeyboardInterrupt`, so
the existing vendor-pybind-safe exit path was skipped and the SDK object was destructed
twice during interpreter teardown. The main loop now always calls the guarded SDK
stop/shutdown path for real mode and bypasses the second pybind destruction. A final
simultaneous run exited with left/right process codes `0/0`; no segmentation fault was
reported.

## Dual live coexistence

Both installed ROS adapter processes were then started together with the project's
per-subnet bind shim, the same local UDP port, distinct host IPs, distinct node names,
and `apply_device_config=false`.

| Metric | Left | Right |
| --- | ---: | ---: |
| Diagnostic messages | 201 | 179 |
| Receive rate | `9.976 Hz` | `10.036 Hz` |
| Header rate | `9.975 Hz` | `10.029 Hz` |
| Frame | `xtm60_left_link` | `xtm60_right_link` |
| Organized layout | `160 x 60` | `160 x 60` |
| Finite fraction | `73.163%` | `78.622%` |
| Intensity maximum | `1600` | `2157` |
| Header regressions | 0 | 0 |
| Published `>=64000` sentinels | 0 | 0 |

The unequal message totals reflect stream/diagnostic startup timing; rates are computed
from successive received messages. Both adapters simultaneously sustained their
expected rates, topic/frame separation was correct, and both processes exited cleanly.
The right maximum remained above `2039` after both units received the same export, so the
amplitude difference is not explained by the former `int_time_3=30` value. This validates
basic live coexistence only. It does not validate extrinsics, inter-sensor time synchronization,
fusion accuracy, FAST-LIO2, RTAB-Map, blind-zone coverage, or passenger safety.

## 2026-07-23 per-IP exports and explicit two-instance test

The user supplied two new upper-computer exports named for the device IPs:

- `192.168.0.101.xtcfg` for the left radar;
- `192.168.1.101.xtcfg` for the right radar.

They have different source filenames and timestamps but are byte-identical, each 685
bytes with SHA-256
`15D7C4A05F810F3AC74CFD10CD96E0CB50068BEA3951C9FC066DCAB743F4F6D7`.
Each file is retained separately under `docs/hardware/vendor` so the device association
remains explicit. Both contain `freq1..5=0,0,0,3,2`; this newer export no longer has the
older report's `freq5=3` discrepancy.

The preferred architecture was tested first and passed, so the alternative "one SDK
object manages two radar IPs" was intentionally not tested:

- process 1 created its own `XtSdk()` object, connected only to `192.168.0.101`, bound
  UDP receive to `192.168.0.100:7687`, and loaded the left export;
- process 2 independently created its own `XtSdk()` object, connected only to
  `192.168.1.101`, bound UDP receive to `192.168.1.100:7687`, and loaded the right
  export.

The two configuration/read-back processes ran simultaneously and exited `0/0`. In
about 9.9 seconds they received 39 left and 40 right frames; both SDK timestamp interval
means were exactly `0.100 s`. The subsequent production ROS nodes also ran as two
processes/two SDK objects for 60 seconds:

| Metric | Left | Right |
| --- | ---: | ---: |
| Messages | 599 | 600 |
| Receive rate | `9.973 Hz` | `9.984 Hz` |
| Header rate | `9.974 Hz` | `9.984 Hz` |
| Frame | `xtm60_left_link` | `xtm60_right_link` |
| Organized layout | `160 x 60` | `160 x 60` |
| Finite fraction | `78.960%` | `71.115%` |
| Intensity maximum | `2121` | `2347` |
| Header regressions | 0 | 0 |
| Published `>=64000` sentinels | 0 | 0 |

Both status topics reported `connected; measuring`. Topic and frame separation,
organized shape, metre scale, monotonic headers, and approximately 10 Hz rate all
passed. This is the required two-radar concurrent acquisition architecture: the second
radar supplements field of view and the streams will be fused in a later stage. It is
not a claim of simultaneous exposure or hardware clock synchronization.

The new scene/configuration also produced values above the documented `0-2039` amplitude
range on both units, not just the right (`2121` left, `2347` right). Values `>=64000`
were still correctly treated as invalid sentinels. The intermediate range remains
unexplained and is preserved rather than silently clipped.

After both 60-second diagnostics had completed and their JSON files were persisted, the
Orin became unreachable and booted again at `2026-07-23 14:53:45 +08:00`. `last -x`
contains no orderly shutdown entry, the prior journal stops abruptly, and the
unprivileged account cannot read `/sys/fs/pstore`. The prior kernel journal contains no
panic, thermal, voltage, or watchdog event near the end of the boot. The reboot cause
is therefore unknown and must not be attributed to either the dual-radar processes or
the SDK without reproduction.

The investigation did expose a separate pre-existing user service fault:
`smartwheel.service` referenced missing
`/home/nvidia/smartwheel/scripts/autostart_smartwheel.sh`, had
`Restart=on-failure/RestartSec=5`, an override that enabled XT-M60 at boot, and had
already exceeded 14,000 failed restarts. It was doing no sensor work because exec
failed. The broken service was stopped and disabled after reboot; final state was
`inactive/dead`, `disabled`, restart counter zero. This reversible safety action also
prevents unapproved automatic radar startup. It does not explain the hard reboot by
itself.

Consequently, two-instance concurrent acquisition passes the bounded 60-second test,
but long-duration host stability remains open.

## 2026-07-24 right orientation correction

The SDK point axes were reconfirmed as `+z` forward, `+x` left and `+y` up. The current
URDF claimed a `ground_plane_calibrator_node` owned both radar TF edges, but neither its
source nor executable exists in this branch/install space. As a result, normal
`robot_state_publisher` bring-up had no `base_link -> xtm60_left/right_link` edge, while
the fallback YAML still used a body-frame `rpy=[0,-0.0873,0]` that is incompatible with
the SDK axes.

Two simultaneous read-only ground fits were collected. The expanded 30-frame/600-
iteration right repeat selected a floor below the sensor with `23.029%` inliers,
`12.17 mm` mean residual, distance `0.7541 m`, and sensor-frame normal
`[-0.026524, 0.990374, -0.135851]`. This corresponds to an approximately `7.81 deg`
nose-down installation and `-1.53 deg` roll about the optical-forward axis. The
current left fit selected an above-sensor/non-floor surface and was rejected.

The restored URDF is now the single TF owner. It uses the user's approximate `0.60 m`
separation and `0.735 m` optical-centre height (`y=+/-0.30`, `z=0.735`); longitudinal
`x=0.45 m` remains the previous unverified estimate. Left roll/pitch retains the last
reliable floor-visible calibration from `45b1163`. Right roll/pitch uses the new floor
fit. The right rotation is `rpy=[1.7071165220,0.0265268546,1.5744345976]`, quaternion
`[0.525700,0.539967,0.458534,0.470979]` in `xyzw` order. It maps the fitted floor
normal to base `[0,0,1]` and places the floor at `z=-0.0191 m`.

Horizontal yaw cannot be observed from a flat floor. It is explicitly constrained to
optical-forward = base `+x` (`0 deg`) rather than claimed as calibrated. Runtime
`tf2_echo` verified one `/tf_static` publisher and the intended left/right translations
and rotations. Neither dual-cloud alignment, FAST-LIO2, synchronization nor mapping was
started. The existing FAST-LIO2 extrinsics remain historical/stale and must not be used
as final values.

## Artifacts

- `docs/hardware/evidence/XT_M60_RIGHT_DEVICE_INFO.json`
- `docs/hardware/evidence/XT_M60_RIGHT_SDK_FRAMES_BIND_SHIM_FAILED.json`
- `docs/hardware/evidence/XT_M60_RIGHT_SDK_FRAMES.json`
- `docs/hardware/evidence/XT_M60_RIGHT_B6_CLOUD.json`
- `docs/hardware/evidence/XT_M60_RIGHT_B6_BAG_REPLAY.json`
- `docs/hardware/evidence/XT_M60_DUAL_LEFT_CLOUD.json`
- `docs/hardware/evidence/XT_M60_DUAL_RIGHT_CLOUD.json`
- `docs/hardware/evidence/XT_M60_LEFT_EXPORT_CONFIG_APPLIED.json`
- `docs/hardware/evidence/XT_M60_RIGHT_EXPORT_CONFIG_APPLIED.json`
- `docs/hardware/evidence/XT_M60_DUAL_POST_EXPORT_LEFT_CLOUD.json`
- `docs/hardware/evidence/XT_M60_DUAL_POST_EXPORT_RIGHT_CLOUD.json`
- `docs/hardware/vendor/192.168.0.101_20260723.xtcfg`
- `docs/hardware/vendor/192.168.1.101_20260723.xtcfg`
- `docs/hardware/evidence/XT_M60_LEFT_EXPORT_CONFIG_20260723.json`
- `docs/hardware/evidence/XT_M60_RIGHT_EXPORT_CONFIG_20260723.json`
- `docs/hardware/evidence/XT_M60_DUAL_TWO_INSTANCES_LEFT_20260723.json`
- `docs/hardware/evidence/XT_M60_DUAL_TWO_INSTANCES_RIGHT_20260723.json`
- `docs/hardware/evidence/XT_M60_LEFT_GROUND_CURRENT_INITIAL_20260724.json`
- `docs/hardware/evidence/XT_M60_RIGHT_GROUND_CURRENT_INITIAL_20260724.json`
- `docs/hardware/evidence/XT_M60_LEFT_GROUND_CURRENT_REPEAT_20260724.json`
- `docs/hardware/evidence/XT_M60_RIGHT_GROUND_CURRENT_REPEAT_20260724.json`
- `docs/hardware/evidence/XT_M60_RIGHT_ORIENTATION_PROVISIONAL_20260724.json`
- `docs/hardware/evidence/XT_M60_TF_RUNTIME_20260724.txt`
- `scripts/hardware/xtm60_apply_export_config.py`
- `scripts/hardware/xtm60_orientation_diagnostic.py`

## Gate conclusion

The right XT-M60 basic SDK/ROS path, per-IP authorized export configuration, two
independent SDK objects/processes, simultaneous dual-LiDAR publication, and clean
bounded configuration-process shutdown pass. The alternative single-object architecture
was not tested because the user's preferred two-instance approach passed. Unexplained
amplitude values above 2039 now occur on both radars and remain open for vendor
interpretation; they are not silently discarded. The unexplained post-test hard reboot
also prevents a long-duration stability claim. Hardware-wide validation remains false,
and no mapping or navigation gate is opened by this result.
