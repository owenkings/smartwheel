# Real-Hardware RViz Workbench Follow-up — 2026-07-26

## Scope and safety state

This follow-up addresses three issues reported immediately after the guarded RViz
motor-control test:

1. a held W/A/S/D key produced taps instead of continuous input;
2. the RViz session showed no camera image;
3. the session lacked the agreed 3D and 2D views.

The original session had intentionally started only the motor safety chain and RViz,
so no camera, radar, scan, or mapping publisher existed. All corrective hardware-view
tests in this report used `motion_control_enabled:=false`; no motor write or motion
command was sent. The final occupancy grid is a live, unsaved preview, not a mapping
quality acceptance result.

## Corrections

### Held-key handling

`SmartWheel/Teleop` no longer discards Qt auto-repeat events. A 120 ms generation-based
release debounce bridges the synthetic press/release pairs produced by the remote
desktop. The final release still clears the key, while STOP, loss of application/window
focus, panel hide/close, and the safety supervisor remain independent immediate-stop
paths. The output remains locked to `/teleop/cmd_vel`.

### One real-hardware launch entry

`wheelchair_bringup/launch/manual_teleop.launch.py` now defaults to:

- two independent XT-M60 instances (`radar:=both`);
- the qualified four-camera `camera_quad.yaml` profile;
- one camera process per UVC chain and direct compressed display subscriptions;
- H30, four FD07-34R channels, wheel feedback, watchdog, emergency-stop publisher,
  safety supervisor, dual point-cloud projections, and `/scan` merger;
- `SmartWheel/Teleop -> /teleop/cmd_vel -> /cmd_vel_safe`;
- the new `hardware_operator_workbench.rviz` layout;
- a live `slam_toolbox` `/map` preview, which can be disabled with
  `enable_2d_mapping:=false`.

The safe default is still `motion_control_enabled:=false`.

### RViz layout

The central RenderPanel displays the left and right live XT-M60 point clouds in
`base_link`, plus the merged `/scan`, robot model, TF, and grid. The raw wheel
odometry display is disabled while valid controller feedback is unavailable; its
large covariance visualization was the source of the former yellow-brown overlay.
Compatibility topic names are retained, while each camera panel shows its physical
identity:

| RViz tab/topic slot | Physical camera | USB path |
| --- | --- | --- |
| Front Camera, `/camera/front` | left-side | `2-3.2` |
| Left Camera, `/camera/left` | left-front | `2-3.1` |
| Right Camera, `/camera/right` | right-front | `2-3.4` |
| Rear Camera, `/camera/rear` | right-side | `2-3.3` |

The final operator layout is a fixed starting arrangement that remains normally
resizable/floatable in RViz:

- left top: left-front camera;
- left middle: left-side camera;
- left bottom: SmartWheel Teleop, tabified with Displays, Views and SmartWheel System
  Status;
- right top: right-front camera;
- right middle: right-side camera;
- right bottom: SmartWheel 2D Map, with a larger horizontal dock allocation.

The 2D panel now puts the map canvas first. Status, grid size, zoom, mapping state and
the honest product state (`not saved (live preview)`) are below it. Topic/output
details are collapsed by default. Auto-fit is enabled so a growing live map remains
fully visible; it can be disabled before wheel zoom/drag-to-pan, while double-click
fit, resizing and floating remain available. `slam_toolbox` starts eight
seconds after the sensor/TF chain so an early transform queue cannot leave the live
preview at `0 x 0`.

The final image-first pass goes further: each camera and the 2D map now devotes
its normal dock body to the image/canvas, leaving only one compact `Details`
button. Physical labels, topics, transport, resolution, FPS, timestamps, latency,
mapping state, product path and display options appear only after that button is
expanded; the camera details use their own scroll area. Aspect ratio remains on
by default, so portrait camera views keep letterboxing instead of being distorted
or cropped.

Both live XT-M60 displays now use the already-published `intensity` field from
`IMG_POINTCLOUDAMP` rather than coordinate `AxisColor`. They use the documented
0..2039 amplitude range, rainbow mapping and flat squares. The ROS messages retain
out-of-range values for diagnostics; the known right-radar maximum above 2039 is
not clipped in stored data.

The saved Views list contains `Live Sensor Preview` and `Live 2D Overview`. The word
"Preview" is deliberate: the center currently shows current dual-LiDAR frames, not
an accumulated LIO/RTAB-Map result.

### LaserScan contract

Both point-cloud projection and scan-merger code previously computed a non-integral
angle span with `ceil(span / increment) + 1`. For the merged range this declared
`[-1.5708, +1.5708]` at `0.0087 rad` but emitted 363 samples; `slam_toolbox` expected
362 and rejected the scan.

The beam count now uses the largest complete set of increments that does not pass the
configured maximum, and published `angle_max` is the realized last-beam angle. The
runtime contract is:

```text
angle_min       -1.5707999468 rad
angle_max       +1.5699000359 rad
angle_increment  0.0087000001 rad
beam_count       362
```

## Runtime verification

- `smartwheel_rviz_plugins`: build passed; 27 tests, 0 errors/failures/skips. The
  suite includes the remote-desktop held-key regression, camera physical-label round
  trip, and the map-panel product/settings round trip.
- `wheelchair_perception`: build passed; 10 tests, 0 errors/failures/skips. New tests
  cover non-integral scan spans and prevent a beam beyond `angle_max`.
- `wheelchair_bringup`: launch syntax, build, installed paths, and declared arguments
  passed. The integrated launch remained active with motor writes disabled.
- Dual live clouds: left `9.869 Hz`, right `9.996 Hz`; organized `160x60` XYZI and
  increasing timestamps passed. The right sample again exceeded the documented
  intensity range (maximum `2529`), so that independent vendor issue remains open.
- Merged `/scan`: approximately `9-10 Hz`; the previous 363-versus-362 rejection no
  longer appears.
- Four compressed cameras, simultaneous 12-second check: all topics passed, all
  payloads nonempty JPEG `320x240`, increasing timestamps, and complete-window
  coverage. Measured publication rates were approximately `23.13/23.23/21.91/23.44
  Hz` for front/left/right/rear compatibility slots. RViz still limits GUI decode to
  10 Hz.
- Live `/map`: published successfully at `37x63`, `0.05 m/cell`; the RViz panel showed
  `MAP ONLINE` together with the 3D point clouds and a real camera image.
- Final rearranged-layout restart: the delayed mapper first produced `47x69` and
  grew to `50x89` at `0.05 m/cell`; the four physical camera views, current-frame
  dual point clouds and map-first panel remained visible together after a further
  45-second render check. The base status remained
  `real_motion_enabled=false; motion_control_enabled=false`.
- Final image-first restart: all four camera controls/status blocks and the map
  metadata were collapsed behind one-line `Details` controls. Camera and map
  details were each expanded and collapsed successfully; camera details scrolled
  inside the dock. PointCloud+Amp intensity rendering remained live concurrently.
- Each of the four compressed topics had one RViz subscriber; each radar point cloud
  and `/scan` had active RViz/perception subscribers.

Evidence:

- `docs/hardware/evidence/HARDWARE_WORKBENCH_CAMERAS_20260726.json`
- `docs/hardware/evidence/HARDWARE_WORKBENCH_LEFT_CLOUD_20260726.json`
- `docs/hardware/evidence/HARDWARE_WORKBENCH_RIGHT_CLOUD_20260726.json`
- `docs/hardware/evidence/HARDWARE_WORKBENCH_RVIZ_20260726.png`
- `docs/hardware/evidence/HARDWARE_WORKBENCH_LAYOUT_20260726.png`
- `docs/hardware/evidence/HARDWARE_WORKBENCH_IMAGE_FIRST_20260726.png`

## Remaining limits

- The displayed 3D scene is two live, TF-oriented raw clouds. It is not a validated
  accumulated 3D map, dual-LiDAR fusion result, or FAST-LIO2 result.
- The small 2D grid proves the UI and scan-to-map plumbing only. The chair was
  stationary/off-ground; map geometry, odometry, loop closure, persistence, and
  navigation suitability remain unvalidated.
- LiDAR final extrinsics/hardware synchronization, H30 dynamic/yaw/xyz calibration,
  camera intrinsics/extrinsics, right XT-M60 intensity semantics, and passenger-safe
  operation remain open.
- Enabling motor writes for another session still requires an explicit current
  confirmation that both wheels are off the ground, no passenger is present, the area
  is clear, and the physical emergency stop is ready.
