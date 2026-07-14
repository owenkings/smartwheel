# Frame and Topic Contract

## Coordinate Convention

All mapping-v2 components follow REP-103: right-handed coordinates, +X forward, +Y
left, +Z up, metres, radians, and ROS timestamps. A node must reject non-finite
geometry, impossible scale, or non-monotonic source time instead of silently repairing
it.

## TF Tree and Ownership

```text
map                         global_mapping backend (exactly one publisher)
└── odom                    state_estimation (exactly one publisher)
    └── base_link           robot_state_publisher owns all static descendants
        ├── imu_link
        ├── xtm60_left_link
        ├── xtm60_right_link
        ├── camera_front_link
        ├── camera_left_link
        ├── camera_right_link
        └── camera_rear_link
```

`rtabmap` and `slam_toolbox` are mutually exclusive map backends. FAST-LIO2, wheel
odometry, mock ground truth, and `robot_localization` must not also publish either
dynamic edge unless selected as its sole owner. The H30 IMU consumed by FAST-LIO2 is
not fused again by `robot_localization` in `lio_primary` mode.

## Topic Contract

| Topic | Type | Frame | Default QoS | Producer |
| --- | --- | --- | --- | --- |
| `/lidar/left/points_raw` | `sensor_msgs/PointCloud2` | `xtm60_left_link` | sensor data | left LiDAR backend |
| `/lidar/right/points_raw` | `sensor_msgs/PointCloud2` | `xtm60_right_link` | sensor data | right LiDAR backend |
| `/lidar/primary/points_lio` | `sensor_msgs/PointCloud2` | primary LiDAR frame | sensor data | validated LIO adapter |
| `/lidar/left/points_registered` | `sensor_msgs/PointCloud2` | `base_link` | sensor data | dual-LiDAR fusion |
| `/lidar/right/points_registered` | `sensor_msgs/PointCloud2` | `base_link` | sensor data | dual-LiDAR fusion |
| `/lidar/merged/points` | `sensor_msgs/PointCloud2` | `base_link` | sensor data | dual-LiDAR fusion |
| `/imu/data_raw` | `sensor_msgs/Imu` | `imu_link` | sensor data | H30 backend |
| `/wheel/encoder_counts` | `smartwheel_interfaces/WheelEncoder` | n/a | reliable depth 20 | wheel backend/simulator |
| `/wheel/odom` | `nav_msgs/Odometry` | `odom` / `base_link` | reliable depth 20 | wheel odom driver |
| `/lio/odom` | `nav_msgs/Odometry` | `odom` / `base_link` | reliable depth 20 | FAST-LIO2 or mock-LIO |
| `/lio/path` | `nav_msgs/Path` | `odom` | reliable depth 5 | FAST-LIO2 or mock-LIO |
| `/odom/fused` | `nav_msgs/Odometry` | `odom` / `base_link` | reliable depth 20 | state selector |
| `/camera/{name}/image_raw` | `sensor_msgs/Image` | camera link | sensor data | camera backend |
| `/camera/{name}/camera_info` | `sensor_msgs/CameraInfo` | camera link | reliable transient local | camera backend |
| `/map` | `nav_msgs/OccupancyGrid` | `map` | reliable transient local | active map backend/product node |
| `/map_cloud` | `sensor_msgs/PointCloud2` | `map` | reliable transient local | map accumulator |
| `/mapping/status` | `smartwheel_interfaces/MappingStatus` | n/a | reliable transient local | mapping manager |
| `/hardware/status` | `smartwheel_interfaces/HardwareStatus` | n/a | reliable depth 10 | hardware facade/simulator |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | n/a | reliable depth 10 | all components |
| `/sim/ground_truth/odom` | `nav_msgs/Odometry` | `map` / `base_link_gt` | reliable depth 20 | simulator only |
| `/sim/completed` | `std_msgs/Bool` | n/a | reliable transient local | simulator only |

All frame IDs, topics, rates, timeouts, and queue depths are ROS parameters. The table
defines defaults, not permission to hard-code real hardware properties.

## Timing and Pairing

- Source acquisition timestamps are preserved.
- Point clouds must contain `x`, `y`, and `z` float fields. `intensity` is optional.
- Dual-LiDAR frames are paired by source timestamp within the configured tolerance.
- Unpaired stale frames are dropped with diagnostics; they are never joined to the
  newest frame unconditionally.
- Zero timestamps are accepted only from explicit mock fixtures that declare that
  behavior. Real backends fail diagnostics on zero or regressing time.
- `dual_lio` is an interface-only Stage A mode and emits `NOT_IMPLEMENTED`.

