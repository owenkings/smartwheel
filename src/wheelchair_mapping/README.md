# wheelchair_mapping

建图相关 launch 和地图后处理占位。

```bash
ros2 launch wheelchair_mapping online_mapping.launch.py
ros2 run nav2_map_server map_saver_cli -f maps/indoor_map
python3 src/wheelchair_mapping/scripts/map_postprocess.py maps/indoor_map.yaml
python3 src/wheelchair_mapping/scripts/map_quality_check.py maps/indoor_map.yaml
python3 src/wheelchair_mapping/scripts/vectorize_occupancy_map.py maps/indoor_map.yaml --output src/wheelchair_navigation/config/semantic_map.yaml
```

`map_quality_check.py` 会给出 `GOOD / WARNING / BAD` 和可读原因，例如地图过小、未知区域过多、可通行区域碎片化。原生 GUI 保存地图后会自动运行该检查，并把 JSON 报告写到同名 `_quality.json` 文件。

地图保存采用版本化结构：每次保存都会写入 `maps/versions/<地图名>_<时间戳>.yaml`，同时更新 `maps/<地图名>.yaml` 作为当前别名。`maps/map_versions.json` 记录版本、质量报告、建图日志和当前版本，原生 GUI 的系统页可查看最近版本并切换当前版本。

`vectorize_occupancy_map.py` 生成的墙线只用于 UI 显示和语义标注，不替代 Nav2/AMCL 使用的静态栅格地图。

地图维护模式的原则：普通导航不修改静态地图；长期障碍需要多次观测并由用户确认后才写入静态地图。
