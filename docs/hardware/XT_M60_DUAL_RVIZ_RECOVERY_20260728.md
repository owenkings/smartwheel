# XT-M60 dual-radar RViz recovery — 2026-07-28

## Problem

The user observed that covering the physical right radar removed the entire RViz
cloud while covering the physical left radar changed nothing. Earlier ROS
diagnostics had also placed the left stream at an implausible `36.356 m` median
range, so topic presence and a nominal 10 Hz rate were not evidence that both
radars contributed usable geometry.

No device configuration was written during this recovery.

## Isolation

The two devices were sampled separately through the vendor SDK with ImageType 4,
their correct per-subnet bind addresses, and no optional host-SDK filter chain.
Both raw streams were geometrically plausible:

| Stream | Raw median | Raw P05-P95 | Valid pixels | Rate |
| --- | ---: | ---: | ---: | ---: |
| left / `192.168.0.101` | `1.278 m` | `0.619..4.046 m` | about `60.2%` | about `10 Hz` |
| right / `192.168.1.101` | `1.922 m` | `1.042..5.546 m` | about `99.1%` | about `10 Hz` |

For both devices, SDK point-vector norm divided by `distData` was
`0.001 m/mm`, proving that raw distance and point coordinates agreed. This
rules out a dead left transmitter, an RViz-only display fault, and a corrupted
XYZ conversion as the immediate cause.

The difference from the failed workbench path was the adapter's optional SDK
filter chain (`setSdkMedianFilter`, edge, Kalman, dust, postprocess and
reflective filters). With that chain enabled, the left ROS cloud moved to a
`36.356 m` median. Without it, the left raw cloud returned to the nearby room.

A read-only device-config refresh also found that the two saved configurations
currently differ (`isFilterOn=55` versus `125`, and integration slot 3 is
`20 us` versus `30 us`) despite the two Windows exports being byte-identical.
That drift remains recorded but was not changed, because raw ImageType 4 data
from both devices was already valid without a device write.

## Change

Production `xtm60_left.yaml` and `xtm60_right.yaml` now set:

```yaml
enable_sdk_filters: false
```

Both sides use the same policy. The device's saved exposure/HDR/modulation
configuration remains untouched (`apply_device_config: false`). The individual
offending optional filter has not yet been isolated; the chain must not be
re-enabled wholesale.

The raw-frame diagnostic was extended to save distance, point-range, amplitude,
valid-pixel, centre-pixel and consecutive-frame-change distributions so this
failure cannot again be hidden by a simple rate check.

## Dual-runtime validation

The full read-only hardware workbench was rebuilt and started with motor writes
disabled. Both ROS PointCloud2 streams were organized `160x60` XYZI near 10 Hz:

| Stream | Messages / duration | Rate | Median | P05-P95 | Valid fraction |
| --- | ---: | ---: | ---: | ---: | ---: |
| `/xtm60/left/points` | 81 / 8 s | `10.069 Hz` | `1.356 m` | `0.484..4.125 m` | `45.625%` |
| `/xtm60/right/points` | 77 / 8 s | `10.066 Hz` | `1.945 m` | `1.060..5.549 m` | `99.147%` |

RViz display isolation then proved:

- left display only: recognizable near-field walls/objects remained visible;
- right display only: its independent room geometry remained visible;
- both enabled: the two current-frame PointCloud+Amp views appeared together.

This resolves the software double-display failure. A future physical hand-cover
check is still useful as a user-visible regression check, but it is no longer
needed to prove that two independent ROS displays contain scene geometry.

## PointCloud+Amp display-range correction

Both adapters were already configured independently for ImageType 4,
`publish_intensity: true` and organized XYZI output. Both RViz displays also
used the `intensity` field and the rainbow transformer. The missing setting was
not PointCloud+Amp itself: both displays shared the same fixed `0..2039`
visualization range even though their measured amplitude distributions differ
substantially:

| Stream | Median amplitude | P95 amplitude | Observed max |
| --- | ---: | ---: | ---: |
| left | `158` | `625` | `1508` |
| right | `583` | `1445` | `1787` |

The common range compressed most left returns into the lower end of the colour
map and made the left cloud look unlike the richer right display. RViz's
per-frame automatic intensity bounds were tested and rejected because the
central render became blank in this runtime. The production workbench instead
uses stable, per-display visualization bounds:

- left: `50..700`;
- right: `50..1600`.

This affects only the RViz colour lookup. PointCloud2 intensity values are not
clipped, normalized or rewritten, and recorded/fused XYZI remains raw. The
bounded read-only workbench check showed a dense multicolour dual cloud after
this change. It does not solve spatial mismatch: final left/right xyz/rpy,
time offset and overlap calibration are still pending, so geometry that does
not align must be handled as an extrinsic/synchronization task rather than a
colour problem.

## User position-swap and overlap observation

The user subsequently moved/rotated the wheelchair so that the physical left
radar occupied the location/view previously used by the right radar. The left
unit still did not reproduce the right unit's dense planar return. A passing
person appeared at corresponding far-cloud locations in both streams, proving
that the two sensors were observing common scene content even though their
rendered clouds did not align.

The user also corrected the mechanical fact: the optical centres are separated
laterally by about `0.60 m` across the wheelchair and have no relative
fore/aft separation. The model already has equal longitudinal coordinates
(`x=0.45 m` for both) and `y=+/-0.30 m`; the common absolute `x=0.45 m`
remains an older unverified base-origin estimate.

The current TFs nevertheless point their optical-forward axes at different
vertical angles: left is about `+1.1 deg` and right about `-7.8 deg`, a relative
difference of about `8.9 deg`. At 5 m that can create about `0.78 m` of
distance-dependent apparent displacement. This is a transform issue, not a
configured fore/aft baseline.

There is also a separate acquisition asymmetry. The latest ROS measurements
had about `45.6%` valid pixels on the left and `99.1%` on the right. Since the
position swap did not swap that behaviour, the earlier explanation based only
on the scene/surface is rejected. Normal startup forces ImageType 4 but uses
`apply_device_config: false`, so it does not reapply the full Windows export.
Read-only saved-config values currently differ (`isFilterOn=55/125` and third
exposure `20/30 us`) even though the two export files are byte-identical.
Device state, optics and the remaining SDK/device processing path must be
isolated before final extrinsic calibration. No device configuration was
written during this observation.

## Shutdown

The workbench service used `KillSignal=SIGINT` with a 20-second stop window.
Both adapters logged TCP/UDP closure and exited. Final checks found:

- service `inactive`;
- no `xtm60_adapter_node` process;
- no UDP packet during separate 3-second listens on
  `192.168.0.100:7687` and `192.168.1.100:7687`.

The radars are not being left hot after this test.

## Evidence

- `docs/hardware/evidence/XT_M60_LEFT_RAW_DISTRIBUTION_20260728.json`
- `docs/hardware/evidence/XT_M60_RIGHT_RAW_DISTRIBUTION_20260728.json`
- `docs/hardware/evidence/XT_M60_LEFT_FILTERS_DISABLED_ROS_20260728.json`
- `docs/hardware/evidence/XT_M60_RIGHT_FILTERS_DISABLED_ROS_20260728.json`
- `docs/hardware/evidence/XT_M60_DUAL_FILTERS_DISABLED_20260728.png`
- `docs/hardware/evidence/XT_M60_LEFT_ONLY_FILTERS_DISABLED_20260728.png`
- `docs/hardware/evidence/XT_M60_RIGHT_ONLY_FILTERS_DISABLED_20260728.png`
- `docs/hardware/evidence/XT_M60_AUTO_BOUNDS_REJECTED_20260728.png`
- `docs/hardware/evidence/XT_M60_DUAL_PER_SENSOR_INTENSITY_20260728.png`
