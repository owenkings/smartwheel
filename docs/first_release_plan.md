# SmartWheel First-Release Plan

## Scope

This phase delivers a deliberately simple, releasable Web 2D user-map demo. It
does not replace SLAM, Nav2, the Native GUI, RViz, or the safety layer.

The release path is:

1. RTAB-Map consumes LiDAR-primary `/points_merged`.
2. `/rtabmap/cloud_map` remains the primary 3D algorithm/debug map.
3. `/rtabmap/grid_map` is the projected 2D navigation and Web map; `/map` is
   the fallback.
4. Existing FastAPI `wheelchair_ui.app` serves the ordinary user/demo UI.
5. Navigation requests go to Nav2, never directly to a velocity topic.

## Release Content

- Map-ready and map-not-ready states without a fabricated grid.
- Wheelchair pose with source, age, and confidence.
- Current goal, Nav2 path, POIs, room polygons, and no-go polygons.
- Click modes for temporary goals, POIs, initial pose, rooms, and no-go zones.
- Safety, navigation, localization, map, sensor, and 3D mapping status.
- Software emergency stop and release.
- YAML-backed POI and semantic-map persistence.
- Web-only launch/run script and a first-release ROS self-check.
- Pure coordinate conversion functions and hardware-independent tests.

## Safety Gate

Real motion remains explicit opt-in. Both `base.launch.py` and
`full_system.launch.py` default `motion_control_enabled:=false`.

The real navigation chain is:

```text
/goal_pose, /named_goal_command, or NavigateToPose
  -> Nav2
  -> /cmd_vel_nav
  -> safety_supervisor
  -> /cmd_vel_safe
  -> base driver
```

The Web/Native shared `RosBridge` does not create `Twist` publishers. Software
stop uses `/emergency_stop_sw` and `/emergency_stop_command`.

## Current Sensors

- XT-M60: `/xtm60/left/points`, `/xtm60/right/points`
- Fusion: `/points_merged`
- H30 IMU: `/imu/data`
- Ultrasonic: `range_0=left-front`, `range_1=left`,
  `range_2=right-front`, `range_3=right`
- Cameras: left and right USB cameras only
- Wheel/base: `/wheel/odom`, `/base/status`

Cameras currently provide status and optional `/rgb_cloud_map` coloring.
`subscribe_rgb:=false` keeps them out of the RTAB-Map geometry loop by default.

## Deferred Work

P2/P3 work includes automatic room segmentation, automatic relocalization,
charging-dock return, 3DGS/mesh presentation, GMSL camera integration, and
costmap enforcement for Web-drawn no-go zones. 3D reconstruction is deferred,
not abandoned, and remains outside the real-time navigation safety loop.

## Release Gates

```bash
colcon build --symlink-install
PYTHONPATH=src/wheelchair_ui:src/wheelchair_navigation \
  pytest -q src/wheelchair_ui/test/test_map_canvas_utils.py
bash scripts/check_first_release_ui.sh
```

AGX Orin verification must cover real topic rates, map QoS/fallback, AMCL or
RTAB-Map pose alignment, Nav2 action acceptance, software/physical emergency
stop, `/cmd_vel_nav -> /cmd_vel_safe`, browser rendering, persistence, and
off-ground motor tests before any clear-area motion test.
