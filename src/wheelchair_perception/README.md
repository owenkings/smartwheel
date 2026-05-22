# wheelchair_perception

感知原型包。

- `pointcloud_to_laserscan_node`：订阅 `/xtm60/points`，变换到 `base_link`，高度/角度过滤后发布 `/scan`
- `obstacle_detector_node`：从 `/scan` 提取简单障碍物 Marker，便于 RViz 调试
- `dynamic_obstacle_layer_node`：地图维护模式占位。临时障碍只进入 local costmap，不写静态地图

XT-M60 为前向有限视场雷达，`angle_min/angle_max` 默认匹配约 120 度水平 FOV。
