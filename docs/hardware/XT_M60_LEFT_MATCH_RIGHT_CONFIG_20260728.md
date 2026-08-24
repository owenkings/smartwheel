# XT-M60 left imaging matched to right — 2026-07-28

## Authorization and scope

The user explicitly authorized changing the physical left XT-M60 configuration
to match the physical right XT-M60. The user also reconfirmed that the two
optical centres have zero relative fore/aft separation; only their lateral
separation is about `0.60 m`.

This operation:

- used right `192.168.1.101`, serial `XTM60B20250324000134`, as the source;
- used left `192.168.0.101`, serial `XTM60B20250324000151`, as the target;
- did not change either device's IP, gateway, MAC, serial, UDP subnet identity,
  calibration blob or TF;
- did not start measurement during the write;
- kept normal production startup at `apply_device_config: false`.

## Repeated readback before the write

Three fresh read-only connections to each unit showed that all stable exposed
imaging fields already matched except the third HDR exposure:

| Field | Right source | Left target |
| --- | ---: | ---: |
| grayscale integration | `2000 us` | `2000 us` |
| HDR integrations | `[1600,200,30,1600] us` | `[1600,200,20,1600] us` |
| HDR mode | `1` | `1` |
| minimum amplitude | `70` | `70` |
| maximum FPS | `10` | `10` |
| modulation/ROI/binning/compensation | equal | equal |

`RespDevConfig.isFilterOn` was deliberately excluded. It changed without any
write across consecutive read-only connections:

- right: `60 -> 192 -> 179` (and `213` in the mirror run);
- left: `65 -> 13 -> 130` (then `214 -> 36` inside one mirror run).

It therefore cannot be treated as a stable persistent device configuration or
copied as a guessed bit mask.

## Write and persistence result

The guarded mirror script verified both serial numbers, stopped the target,
then issued:

- `setIntTimesus(2000,1600,200,30,1600,0)`;
- `setHdrMode(1)`;
- `setMinAmplitude(70)`;
- `setMaxFps(10)`.

All calls returned success. Immediate readback had no difference from the
right source across the stable imaging fields. A completely new read-only SDK
connection then confirmed that the left unit persisted
`[1600,200,30,1600] us`.

Production left and right YAML now both record `int_time_3: 30`. This
intentionally supersedes the older Windows exports' `int3=20`, because the user
selected the better-performing right unit's current persistent setting as the
source. Automatic device writes remain disabled.

## Post-write dual runtime

The normal dual ROS workbench was run with ImageType 4 and optional SDK filters
disabled. Both organized `160x60` XYZI streams passed all diagnostic checks:

| Metric | Left after mirror | Right reference | Left before mirror |
| --- | ---: | ---: | ---: |
| rate | `10.000 Hz` | `9.943 Hz` | `10.069 Hz` |
| valid fraction | `60.54%` | `99.49%` | `45.63%` |
| intensity median | `280` | `511` | `158` |
| intensity P95 | `966` | `1451` | `625` |
| range median | `1.517 m` | `2.263 m` | `1.356 m` |

The copied exposure materially improved the left return density and amplitude,
but did not make the two devices equivalent. The remaining gap follows the
physical sensor after the user's position swap and must be investigated as
optics/device processing/calibration, not hidden by a false longitudinal TF.

No TF was changed. Current TF still has equal left/right longitudinal
coordinates and the known provisional angular mismatch remains open.

## Shutdown

The bounded validation ended by signalling both adapters through their normal
SDK cleanup path and stopping the workbench. Final checks passed:

- service `inactive`;
- no `xtm60_adapter_node`;
- zero UDP-ready sockets during a three-second listen on both
  `192.168.0.100:7687` and `192.168.1.100:7687`.

## Evidence

- `docs/hardware/evidence/XT_M60_RIGHT_CONFIG_SOURCE_BEFORE_20260728.json`
- `docs/hardware/evidence/XT_M60_LEFT_CONFIG_TARGET_BEFORE_20260728.json`
- `docs/hardware/evidence/XT_M60_RIGHT_CONFIG_SOURCE_READBACK2_20260728.json`
- `docs/hardware/evidence/XT_M60_LEFT_CONFIG_TARGET_READBACK2_20260728.json`
- `docs/hardware/evidence/XT_M60_RIGHT_CONFIG_SOURCE_READBACK3_20260728.json`
- `docs/hardware/evidence/XT_M60_LEFT_CONFIG_TARGET_READBACK3_20260728.json`
- `docs/hardware/evidence/XT_M60_LEFT_MIRROR_RIGHT_IMAGING_20260728.json`
- `docs/hardware/evidence/XT_M60_LEFT_CONFIG_PERSISTENCE_AFTER_20260728.json`
- `docs/hardware/evidence/XT_M60_LEFT_POST_RIGHT_CONFIG_MIRROR_ROS_20260728.json`
- `docs/hardware/evidence/XT_M60_RIGHT_POST_RIGHT_CONFIG_MIRROR_ROS_20260728.json`
- `docs/hardware/evidence/XT_M60_DUAL_POST_RIGHT_CONFIG_MIRROR_RVIZ_20260728.png`
- `scripts/hardware/xtm60_mirror_right_imaging_to_left.py`
