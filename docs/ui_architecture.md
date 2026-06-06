# UI Architecture

## Surfaces

### Web User UI

`wheelchair_ui.app` is the FastAPI backend. Static files live in
`wheelchair_ui/static/`. This is the ordinary user and mentor-demo surface:
2D map, pose, goals, path, POIs, semantic polygons, status, and software stop.

### Native GUI

`wheelchair_ui.native_gui` remains the engineering control and diagnostics
surface. It shares `RosBridge`, named-goal storage, semantic-map storage, and
mapping management with the Web UI.

### RViz

RViz remains the engineering 3D visualization surface. It displays
`/rtabmap/cloud_map`, projected maps, point clouds, TF, plans, and debugging
markers. Ordinary users are not directed to RViz point clouds as the main
experience.

## Data Flow

```text
left/right XT-M60 -> /points_merged -> RTAB-Map
                                      |-> /rtabmap/cloud_map (3D primary)
                                      |-> /rtabmap/grid_map  (2D projection)

/rtabmap/grid_map --preferred--+
/map ---------------fallback--+-> RosBridge -> FastAPI JSON -> Canvas 2D

Nav2 path topics ----------------> RosBridge -> Canvas path
AMCL/wheel odom -----------------> RosBridge -> pose + source/confidence
sensor/status topics ------------> RosBridge -> status panels
```

The map bridge stores both map candidates independently. A late `/map` message
cannot overwrite a fresh `/rtabmap/grid_map`; fallback occurs only after the
primary source timeout.

## Navigation Ownership

Web commands are high-level goals:

```text
Web -> RosBridge -> NavigateToPose
                   -> Nav2 -> /cmd_vel_nav
                           -> safety_supervisor
                           -> /cmd_vel_safe -> base
```

Named goals are persisted in `named_goals.yaml`. Rooms/no-go zones are persisted
in `semantic_map.yaml`. The bridge validates clicked goals against occupancy,
unknown space, map bounds, clearance, localization, and Nav2 availability.

`RosBridge` has no `Twist` publisher. Software stop is sent only through
`/emergency_stop_sw` and `/emergency_stop_command`. Real motor writes are
disabled by default and require `motion_control_enabled:=true`.

## Camera Role

The current site profile enables only left/right USB cameras. They appear in
status and may color `/rgb_cloud_map`. RTAB-Map remains LiDAR-primary because
`subscribe_rgb` defaults false. A later upgrade can add calibrated camera
intrinsics/extrinsics and synchronization, then evaluate RGB-assisted mapping
without making camera availability a navigation geometry dependency.

## Coordinate Contract

`map_canvas_utils.py` defines the reference conversions:

- OccupancyGrid origin and resolution map world meters to cells.
- ROS map y increases upward; Canvas y increases downward.
- map bounds exclude the maximum outer edge.
- paths preserve point order through conversion.

The browser uses the same equations and vertically flips OccupancyGrid rows
before drawing, so cells, clicks, pose, paths, and polygons share one frame.
