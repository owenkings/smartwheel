# wheelchair_description

URDF/Xacro 和传感器固定 TF。

```bash
ros2 launch wheelchair_description display.launch.py
```

坐标约定：

- `base_link`：X 向前，Y 向左，Z 向上
- `laser_link`、`imu_link`、四路 camera、六路 ultrasonic 均为 `base_link` 下的固定关节

外参是第一版估计值，真实项目必须重新测量并在 RViz 中验证。
