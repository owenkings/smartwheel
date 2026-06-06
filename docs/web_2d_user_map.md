# Web 2D User Map

## Run

Build once:

```bash
colcon build --symlink-install
```

Start only the Web UI:

```bash
bash scripts/run_web_user_map.sh
```

Open `http://localhost:8080` or the Orin address printed by the script. This
command does not start sensors, Nav2, the safety supervisor, or motor motion.

Other local entry points:

```bash
# 3D mapping, no base motion enabled by this command
ros2 launch wheelchair_3d_mapping rtabmap_3d_mapping.launch.py \
  bringup_sensors:=true

# Navigation stack and Web UI, real motor writes still disabled
ros2 launch wheelchair_bringup full_system.launch.py \
  enable_xtm60_radar:=true motion_control_enabled:=false

# High-risk autonomous exploration; interactive confirmation is required
bash scripts/run_autonomous_rviz_mapping.sh
```

## Main Workflow

The first screen shows the 2D occupancy map and operational status.

- `临时目标`: click a free map cell, then send it to `POST /api/navigate`.
- `保存 POI`: click, enter a name, and save; YAML persistence survives refresh.
- `初始位姿`: click and publish `POST /api/initial_pose`.
- `房间`: click at least three vertices, name the polygon, and save.
- `禁行区`: click at least three vertices, name the polygon, and save.
- `软急停` and `解除急停`: publish the supervised software-stop commands.

Rooms and no-go zones are display/persistence layers in this release. They do
not yet alter Nav2 costmaps.

## Map Behavior

`GET /api/map` selects a fresh `/rtabmap/grid_map` first. If that source times
out and `/map` exists, it falls back to `/map`. A successful snapshot includes:

```json
{
  "ok": true,
  "source_topic": "/rtabmap/grid_map",
  "age_sec": 0.2,
  "frame_id": "map",
  "width": 400,
  "height": 300,
  "resolution": 0.05,
  "origin": {"x": -10.0, "y": -7.5},
  "data": []
}
```

Without either map, the API returns:

```json
{"ok": false, "reason": "map not ready"}
```

The browser displays `地图未就绪`; it does not draw a fake map.

## API Compatibility

The Web backend retains existing hyphenated routes and adds:

```text
POST /api/navigate
POST /api/navigate_named
POST /api/initial_pose
POST /api/release_stop
GET  /api/semantic_map
POST /api/semantic_map
POST /api/no_go_zones
DELETE /api/no_go_zones/{name}
```

Malformed JSON and invalid typed bodies return FastAPI JSON errors. Navigation
rejection returns the bridge reason, including map, localization, clearance, or
Nav2 action-server readiness failures.

## Verification

Run the ROS self-check:

```bash
bash scripts/check_first_release_ui.sh
```

Coordinate tests:

```bash
PYTHONPATH=src/wheelchair_ui:src/wheelchair_navigation \
  pytest -q src/wheelchair_ui/test/test_map_canvas_utils.py
```

Browser checks:

1. Stop map publishers and confirm `地图未就绪`.
2. Start `/rtabmap/grid_map`; confirm source, dimensions, and occupied cells.
3. Confirm pose arrow direction and reported pose source.
4. Click a free point and send it; inspect `/navigation/goal_status`.
5. Save/delete a POI, refresh, and confirm persistence.
6. Draw/save/delete room and no-go polygons.
7. Trigger/release software stop and inspect `/safety_state`.
8. Confirm path rendering from `/navigation/preview_path`, `/plan`,
   `/global_plan`, or `/received_global_plan`.

The Web UI never publishes `/cmd_vel`, `/cmd_vel_nav`, or `/cmd_vel_safe`.
