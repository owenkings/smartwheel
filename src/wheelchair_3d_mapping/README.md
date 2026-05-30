# wheelchair_3d_mapping

LiDAR-Visual-Inertial-Wheel 3D SLAM integration layer for the wheelchair.

This package does NOT reimplement FAST-LIVO2 / R3LIVE. It provides the glue
around an external LIVO backend: dual-LiDAR cloud fusion, wheel/LIVO
consistency monitoring, 3D→2D occupancy projection for Nav2, and an optional
RGB colorizer. It builds and runs whether or not the external backend is
installed (use `backend:=none` for sensor/fusion-only bring-up).

## Nodes

| Node | In | Out |
| --- | --- | --- |
| `dual_lidar_cloud_fusion_node` | `/xtm60/left/points`, `/xtm60/right/points`, TF | `/points_merged`, `/points_merged/status` |
| `wheel_livo_consistency_monitor` | `/wheel/odom`, `/livo/odom` | `/livo_wheel/status`, `/livo_wheel/consistency_score` |
| `cloud_to_occupancy_grid_node` | `/livo/cloud_registered` (configurable) | `/map_2d_from_3d` |
| `rgb_cloud_colorizer_node` | cloud + `/main_camera/image_raw` + `/main_camera/camera_info` | `/rgb_cloud_map` |

## Launch

```bash
ros2 launch wheelchair_3d_mapping dual_lidar_fusion.launch.py
ros2 launch wheelchair_3d_mapping livo_3d_mapping.launch.py backend:=fast_livo2
ros2 launch wheelchair_3d_mapping livo_wheel_fusion.launch.py tf_owner:=ekf
ros2 launch wheelchair_3d_mapping cloud_to_2d_map.launch.py
```

Full system entry point lives in `wheelchair_bringup`:

```bash
ros2 launch wheelchair_bringup bringup_3d_slam.launch.py backend:=fast_livo2 main_camera:=left tf_owner:=ekf
```

See `docs/3d_slam_livo_wheel_architecture.md` for the full design and
`docs/fast_livo2_r3live_integration.md` for installing the external backend.
