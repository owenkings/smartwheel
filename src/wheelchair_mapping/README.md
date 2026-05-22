# wheelchair_mapping

建图相关 launch 和地图后处理占位。

```bash
ros2 launch wheelchair_mapping online_mapping.launch.py
ros2 run nav2_map_server map_saver_cli -f maps/indoor_map
python3 src/wheelchair_mapping/scripts/map_postprocess.py maps/indoor_map.yaml
python3 src/wheelchair_mapping/scripts/vectorize_occupancy_map.py maps/indoor_map.yaml --output src/wheelchair_navigation/config/semantic_map.yaml
```

`vectorize_occupancy_map.py` 生成的墙线只用于 UI 显示和语义标注，不替代 Nav2/AMCL 使用的静态栅格地图。

地图维护模式的原则：普通导航不修改静态地图；长期障碍需要多次观测并由用户确认后才写入静态地图。
