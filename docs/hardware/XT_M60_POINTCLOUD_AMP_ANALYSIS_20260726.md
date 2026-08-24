# XT-M60 PointCloud+Amp analysis (2026-07-26)

## Decision

Preserve and use the XT-M60 amplitude channel throughout the mapping pipeline.
The authoritative radar representation is an organized `XYZI` cloud, where `I`
is the SDK frame's per-pixel amplitude. Amplitude is useful for display, saved
cloud products, diagnostics and potentially algorithm-specific weighting, but it
must never be substituted for geometric range or described as RGB camera color.

## Windows upper-computer evidence

The user supplied XT-Toffuture V2.10.6 and the two exported configurations:

- `C:/Users/admin/Desktop/192.168.0.101.xtcfg`
- `C:/Users/admin/Desktop/192.168.1.101.xtcfg`

Both exports are identical and select:

```text
imgType=4
renderType=2
pclFilterOn=1
maxfps=10
minLSB=70
```

Their filters are median 3, Kalman factor 0.30/threshold 300, edge threshold
150, dust 9000/two frames, post-process 5, dynamic window 9/motion size 3,
reflective thresholds 0.5 to 2.0, and spatial alpha 0.7/delta 70/two
iterations.

The bundled SDK identifies image type 4 as `IMG_POINTCLOUDAMP`. Its English
README states that the flash LiDAR uses global exposure and returns one frame
timestamp together with:

```text
distData   filtered depth pixels
amplData   uint16 amplitude pixels (documented valid range 0..2039)
points     undistorted XYZI points
```

The SDK's image example explicitly starts `IMG_POINTCLOUDAMP` and renders
`frame.amplData`. The upper-computer executable and PDB additionally show:

- PCL and VTK visualization libraries;
- source object/path `xtgui/src/pointcloudviewer.cpp`;
- `pcl::PointXYZI` and an application-specific
  `PointCloudColorHandlerXt<pcl::PointXYZI>::getColor`;
- UI/config identifiers `PointCloud+Amp`, `PointCloud Render Type`,
  `comboBox_pcloudRendertype` and `Setting/renderType`.

The application source code itself is not included. Therefore it is well
supported that the upper computer colors XYZI with an application-specific
handler, but the exact `renderType=2` lookup table/scaling cannot be claimed from
the binary package alone.

## Current ROS implementation

The current Orin implementation already requests `image_type: 4` independently
from both SDK instances. It publishes organized `160x60` PointCloud2 messages
with fields `x`, `y`, `z`, `intensity`, preferring `frame.amplData[index]` over
the point object's fallback intensity. Vendor invalid amplitude sentinels
`>=64000` become invalid points instead of false obstacles.

Both production YAML files enable `publish_intensity: true` and
`organized_cloud: true`, and mirror the exported device/filter parameters while
leaving `apply_device_config: false` during routine startup. This means normal
bring-up uses the already stored radar configuration and does not repeat writes.

The previously saved live diagnostics prove organized XYZI at about 10 Hz. The
left sample measured amplitude 48..1849. The right sample measured 39..2529,
which exceeds the SDK README's documented 2039 maximum and remains a separate
vendor question; no value reached the explicit invalid sentinel range.

`dual_lidar_cloud_fusion_node` concatenates intensity with XYZ, and
`lio_cloud_adapter_node` preserves it when producing both ordinary XYZI and the
FAST-LIO-compatible point layout. Unit tests already cover intensity alignment
through filtering, fusion and adaptation.

## Display and mapping policy

The hardware workbench previously colored the raw clouds with RViz `AxisColor`.
That explains why the same PointCloud+Amp stream did not resemble the Windows
upper-computer display. It now uses the `intensity` channel, a fixed documented
0..2039 range, rainbow mapping and flat three-pixel squares. Values above 2039
saturate visually but are retained in the ROS message.

The read-only hardware workbench was rebuilt and restarted with motor writes
disabled. Both live clouds rendered with amplitude-dependent color at the same
time as the four cameras and `/map`; the dense surfaces and material variation
were visibly clearer than the former coordinate-axis coloring. This validates
the ROS/RViz display path, not the proprietary upper-computer's exact LUT.

For the mapping stage:

1. Keep both raw topics, `/points_merged` and every LIO adapter output as XYZI.
2. Verify the field schema and amplitude statistics at each live stage before
   accepting the mapping result.
3. Use geometry and IMU for pose estimation unless a selected algorithm has a
   documented intensity term; amplitude must not be invented as geometric
   accuracy.
4. Preserve amplitude in registered/accumulated cloud products when the backend
   supports it. If RTAB-Map's emitted `/rtabmap/cloud_map` drops intensity, add a
   separate intensity-preserving accumulated cloud rather than silently losing
   the channel.
5. Compare geometry-only and documented intensity-aware alternatives using the
   same bag before allowing amplitude to influence localization.

This decision does not validate FAST-LIO2, dual-LiDAR synchronization/final
extrinsics, accumulated mapping, loop closure, navigation or passenger safety.

Runtime evidence: `docs/hardware/evidence/HARDWARE_WORKBENCH_IMAGE_FIRST_20260726.png`.

## Per-radar isolation follow-up

After the user reported that only one radar appeared to achieve the desired
PointCloud+Amp effect, both RViz displays were isolated without changing either
device configuration.

The labels in the current ROS configuration are decisive:

- `/xtm60/left/points`, frame `xtm60_left_link`, is the `192.168.0.101`
  instance. It publishes organized XYZI at about 10.00 Hz, but the fresh
  12-second sample placed the median range at `36.356 m` (5th to 95th
  percentile `11.848..49.100 m`). With the RViz camera framed around the
  wheelchair it therefore contributes almost no visible scene geometry.
- `/xtm60/right/points`, frame `xtm60_right_link`, is the `192.168.1.101`
  instance. It publishes organized XYZI at about 10.05 Hz, with median range
  `2.506 m` (5th to 95th percentile `1.170..5.700 m`). This is the stream that
  produces the dense, recognizable, amplitude-coloured room scene in the
  current workbench.

Both samples contain `x/y/z/intensity`, are `160x60`, and use ImageType 4.
Their fresh intensity ranges were `52..2034` (left) and `64..1937` (right), so
this particular run did not reproduce the earlier right-side value above 2039.
The visual mismatch is therefore not caused by one RViz display still using
AxisColor or by a missing intensity field. It is a geometric/range-data
problem on the current left/IP-`192.168.0.101` stream, or a physical
left/right identification/configuration issue upstream of ROS.

Do not hide this by changing the left RViz intensity scale: amplitude colouring
cannot make a 36 m median range look like the nearby room. The next safe
diagnostic is to compare `192.168.0.101` in the official upper computer against
the same physical scene, verify the serial-number-to-side assignment, and
re-read the device's saved range/filter configuration. Reapplying configuration
is a device write and must not be repeated automatically.

A controlled follow-up stopped the right SDK process cleanly (`tcp closed`,
`udp closed`) and then sampled the still-running left stream for another 12
seconds. The left range distribution was unchanged: median `36.366 m`, P05-P95
`11.852..49.098 m`. This rules out the second ROS SDK instance or its active
right-radar measurement as the immediate cause of the left result in this
session. It does not rule out device-local state, optics/environment,
calibration, or physical identity/configuration mismatch.

The startup log associates the current nodes as follows:

- left/IP `192.168.0.101`: model/serial string `XTM60B20250324000151`;
- right/IP `192.168.1.101`: model/serial string `XTM60B20250324000134`.

The user then performed a direct physical occlusion check: fully covering the
physical right radar made the visible cloud disappear, while covering the
physical left radar produced no visible change. This confirms that the current
recognizable scene is entirely supplied by the physical right radar. Although
the left SDK reports connected/measuring and ROS publishes frames, it is not
producing usable scene-responsive near-field data. Treating topic activity as
"left radar working" would therefore be incorrect.

The radars become very hot during prolonged operation. At the end of every
real-radar task, stop both SDK measurements rather than leaving RViz/bring-up
running. A plain service stop can reach systemd's force-kill path before the
vendor node finishes cleanup, so stop the two adapter processes with SIGINT
first, allow their SDK `stop()`/`shutdown()` path to finish, then stop the
remaining service and verify that no adapter process or UDP stream remains.
The final 2026-07-26 shutdown was checked with the service inactive, no
`xtm60_adapter_node` process, and no packet received during separate 3-second
listens on `192.168.0.100:7687` and `192.168.1.100:7687`.

Evidence:

- `docs/hardware/evidence/XT_M60_AMP_LEFT_20260726.json`
- `docs/hardware/evidence/XT_M60_AMP_RIGHT_20260726.json`
- `docs/hardware/evidence/XT_M60_AMP_LEFT_RIGHT_STOPPED_20260726.json`
- `docs/hardware/evidence/XT_M60_AMP_LEFT_ONLY_20260726.png`
- `docs/hardware/evidence/XT_M60_AMP_RIGHT_ONLY_20260726.png`
- `docs/hardware/evidence/XT_M60_SHUTDOWN_20260726.txt`

## 2026-07-28 resolution

The earlier conclusion that the left unit did not provide usable near-field
geometry is now superseded at the software-cause level. A raw SDK A/B test
showed that the physical left radar was working when the adapter's optional
SDK filter chain was omitted: its median raw range was `1.278 m`, compared with
`1.922 m` on the right, and both point norms matched `distData` at
`0.001 m/mm`.

Production left/right YAML now set `enable_sdk_filters: false`. The rebuilt
dual workbench measured left/right ROS medians `1.356/1.945 m` at
`10.069/10.066 Hz`, and RViz isolation showed recognizable geometry from each
radar independently and together. No device configuration was written.

The individual offending optional filter remains unknown, so the full
median/edge/Kalman/dust/postprocess/reflective chain must not be re-enabled.
See `docs/hardware/XT_M60_DUAL_RVIZ_RECOVERY_20260728.md` for the current
evidence and decision.
