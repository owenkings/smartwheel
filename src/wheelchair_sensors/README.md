# wheelchair_sensors

传感器适配包。

- `mock_sensor_node`：发布 mock 点云、IMU、超声波、前向图像、轮速里程计、急停和 mock TF
- `xtm60_adapter_node`：XT-M60 官方 `xintan_sdk` 读取节点，发布 `/xtm60/points` 和 `/xtm60/status`
- `imu_adapter_node`：H30 Yesense 串口解析，发布 `/imu/data`
- `ultrasonic_adapter_node`：FD07-34 Modbus RTU 轮询，发布 `/ultrasonic/range_0` 到 `/ultrasonic/range_5`
- `camera_adapter_node`：OpenCV/V4L2 采集，默认发布前/左摄像头图像

XT-M60 真实模式要求 `XTSDK_PY_ROOT` 或参数 `sdk_root` 指向官方 `xtsdk_py` 根目录。AGX Orin 上应使用 `lib/linux/aarch64` 下与 Python 版本匹配的 `.so`。

真实模式常用配置在 `wheelchair_bringup/config/h30_imu.yaml`、`ultrasonic.yaml`、`camera.yaml`。H30 默认 `/dev/ttyUSB0` + `460800`，超声波默认 `/dev/ttyUSB1` + `9600`，摄像头默认设备索引 `0` 和 `1`。
