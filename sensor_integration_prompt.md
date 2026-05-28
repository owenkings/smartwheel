# 传感器接入方法详细说明（Codex ROS2 版）

本文档详细说明 Codex ROS2 版本中三类传感器（XT-M60 激光雷达、H30 IMU、FD07-34 超声波）的数据接入方式，包括硬件协议、代码实现逻辑和配置方法。可直接作为 prompt 提供给另一个 AI，使其理解并能修改/扩展这些接入代码。

---

## 一、整体架构

每个传感器对应一个独立的 ROS2 Python 节点（adapter node），运行在独立进程中：

```
XT-M60 雷达 → xtm60_adapter_node → 发布 /xtm60/points (PointCloud2)
H30 IMU    → imu_adapter_node   → 发布 /imu/data (sensor_msgs/Imu)
超声波      → ultrasonic_adapter_node → 发布 /ultrasonic/0/range ~ /ultrasonic/5/range (Range)
```

所有节点支持两种模式：
- `mode: real` — 连接真实硬件
- `mode: mock` — 发布模拟数据，用于无硬件时调试全链路

配置文件位于 `src/wheelchair_bringup/config/` 目录下的 YAML 文件中。

---

## 二、XT-M60 3D 激光雷达接入

### 2.1 硬件连接

- 接口：以太网（TCP 控制 + UDP 数据）
- 默认 IP：192.168.0.101（可在 YAML 中修改）
- 需要将主机有线网卡设置为同网段（如 192.168.0.100/24）
- AGX Orin 上：`sudo ip addr add 192.168.0.100/24 dev eth0`

### 2.2 SDK 依赖

使用新探官方 Python SDK `xtsdk_py`。SDK 需要预先解压到 Orin 上：

```bash
sudo mkdir -p /opt/xtsdk_py-main
sudo cp -r /path/to/xtsdk_py-main/* /opt/xtsdk_py-main/
export XTSDK_PY_ROOT=/opt/xtsdk_py-main
```

SDK 目录结构要求：
- `<sdk_root>/cfg/` — SDK 配置文件
- `<sdk_root>/lib/linux/aarch64/` — ARM64 共享库（.so 文件）

代码会自动搜索 SDK 路径（按优先级）：
1. ROS 参数 `sdk_root`
2. 环境变量 `XTSDK_PY_ROOT`
3. 项目目录下的 `third_party/xtsdk_py` 等常见位置

### 2.3 接入流程（代码逻辑）

```python
# 1. 找到 SDK 根目录
sdk_root = find_xtsdk_root(configured_root)

# 2. 配置 Python 导入路径（加载 .so 共享库）
configure_xtsdk_import_path(sdk_root)
# - Linux ARM64: 先用 ctypes.CDLL 加载 libxtsdk_shared.so
# - 然后把 lib/linux/aarch64/ 和 cfg/ 加入 sys.path

# 3. import xintan_sdk
import xintan_sdk

# 4. 创建 SDK 实例并注册回调
sdk = xintan_sdk.XtSdk()
sdk.setCallback(on_event_callback, on_frame_callback)

# 5. 配置连接方式
sdk.setConnectIpaddress("192.168.0.101")  # 以太网模式

# 6. 可选：启用 SDK 内置滤波器
sdk.setSdkMedianFilter(3)
sdk.setSdkEdgeFilter(150)
sdk.setSdkKalmanFilter(300, 200, 2000)

# 7. 启动 SDK（SDK 内部创建网络接收线程）
sdk.startup()

# 8. 等待连接成功后启动测量
if sdk.isconnect():
    image_type = xintan_sdk.ImageType(4)  # 点云模式
    sdk.start(image_type, False)
```

### 2.4 数据回调处理

SDK 在内部线程中调用 `on_frame_callback(frame)`，回调中提取点云：

```python
def on_frame_callback(frame):
    if not frame.hasPointcloud:
        return
    points = []
    for point in frame.points:
        x = point.x * unit_scale  # unit_scale=1.0 表示米，0.001 表示毫米转米
        y = point.y * unit_scale
        z = point.z * unit_scale
        # 过滤无效点和超出范围的点
        distance = sqrt(x*x + y*y + z*z)
        if distance < range_min or distance > range_max:
            continue
        intensity = point.i  # 或从 frame.amplData 获取
        points.append((x, y, z, intensity))
    # 线程安全：存入共享变量，由 ROS timer 发布
    with lock:
        latest_points = points
```

### 2.5 ROS2 发布

节点以 10Hz timer 轮询最新点云并发布：

```python
def tick():
    points, stamp = adapter.take_latest_points()
    if points:
        # 构建 PointCloud2 消息
        header.frame_id = "laser_link"
        header.stamp = sdk_timestamp 或 ros_clock.now()
        msg = point_cloud2.create_cloud(header, fields_xyzi, points)
        publisher.publish(msg)
```

### 2.6 点云转 2D LaserScan

由于 slam_toolbox 和 Nav2 costmap 需要 2D LaserScan，有一个独立节点 `pointcloud_to_laserscan_node` 做投影：

```
/xtm60/points (PointCloud2, frame=laser_link)
    ↓ TF 变换到 base_link
    ↓ 高度过滤 z_min=-0.10m ~ z_max=1.20m
    ↓ 投影到 XY 平面，按角度分 bin
    ↓ 每个 bin 取最近距离
/scan (LaserScan, frame=base_link)
```

关键参数：
- `angle_min: -1.0472`（-60°）
- `angle_max: 1.0472`（+60°）
- `angle_increment: 0.0087`（~0.5°，共 241 个 beam）
- `range_min: 0.08m`
- `range_max: 8.0m`
- `z_min: -0.10m`（过滤地面）
- `z_max: 1.20m`（过滤过高点）

### 2.7 配置文件

`src/wheelchair_bringup/config/xtm60_sdk.yaml`:
```yaml
xtm60_adapter_node:
  ros__parameters:
    sdk_root: ""                    # 或 /opt/xtsdk_py-main
    connection_mode: ethernet       # ethernet 或 usb
    ip_address: "192.168.0.101"
    serial_port: ""                 # USB 模式时填写
    image_type: 4                   # 点云模式
    frame_id: laser_link
    publish_rate_hz: 10.0
    point_unit_scale: 1.0           # 1.0=米, 0.001=毫米转米
    range_min: 0.05
    range_max: 20.0
    publish_intensity: true
    enable_sdk_filters: true
    kalman_factor: 300
    kalman_threshold: 200
    kalman_range: 2000
    median_size: 3
    edge_threshold: 150
```

---

## 三、WHEELTEC H30 IMU 接入

### 3.1 硬件连接

- 接口：USB 转串口（RS232）
- 默认串口：`/dev/ttyUSB0`（Linux）
- 波特率：460800 baud（H30 默认，不是 115200）
- 数据格式：8N1（8 数据位，无校验，1 停止位）
- 采样率：200Hz（节点以 100Hz 发布，每次 tick 读取缓冲区中所有帧取最新一帧）

Linux 权限：`sudo usermod -a -G dialout $USER`（重新登录生效）

### 3.2 Yesense 二进制帧协议

H30 使用 Yesense 标准二进制输出协议：

```
帧结构：
[0x59] [0x53] [TID_H] [TID_L] [Payload_Len] [TLV Payload...] [CK_A] [CK_B]
  帧头固定      事务ID(2字节)   载荷长度(1字节)   数据载荷         校验和(2字节)
```

校验算法（Fletcher-like）：
```python
def yesense_checksum(data: bytes) -> (int, int):
    """对 TID + payload_len + payload 字节计算校验"""
    check_a = 0
    check_b = 0
    for byte in data:
        check_a = (check_a + byte) & 0xFF
        check_b = (check_b + check_a) & 0xFF
    return check_a, check_b
```

### 3.3 TLV 载荷解析

载荷内部是 TLV（Type-Length-Value）结构，每个数据项：
```
[Data_ID (1字节)] [Data_Len (1字节)] [Data (Data_Len 字节)]
```

已知 Data_ID：
| ID | 含义 | 长度 | 数据格式 |
|----|------|------|----------|
| 0x10 | 加速度 | 12 字节 | 3 个 int32 小端，单位 m/s² × 10^6 |
| 0x20 | 角速度 | 12 字节 | 3 个 int32 小端，单位 °/s × 10^6（需转 rad/s） |
| 0x40 | 欧拉角 | 12 字节 | 3 个 int32 小端 (pitch, roll, yaw)，单位 ° × 10^6（需转 rad） |
| 0x41 | 四元数 | 16 字节 | 4 个 int32 小端 (q0,q1,q2,q3)=(w,x,y,z) × 10^6 |
| 0x51 | 采样时间戳 | 4 字节 | uint32 小端，单位 μs |

### 3.4 数据转换

```python
# 加速度：int32 × 1e-6 = m/s²
accel_mps2 = tuple(v * 1e-6 for v in struct.unpack("<iii", data[:12]))

# 角速度：int32 × 1e-6 = °/s → 转 rad/s
gyro_rps = tuple(math.radians(v * 1e-6) for v in struct.unpack("<iii", data[:12]))

# 欧拉角：int32 × 1e-6 = ° → 转 rad
pitch, roll, yaw = (math.radians(v * 1e-6) for v in struct.unpack("<iii", data[:12]))

# 四元数：Yesense 顺序 (w,x,y,z) → ROS 顺序 (x,y,z,w)
q0, q1, q2, q3 = struct.unpack("<iiii", data)
ros_quat = (q1 * 1e-6, q2 * 1e-6, q3 * 1e-6, q0 * 1e-6)  # x,y,z,w
```

### 3.5 串口读取逻辑

```python
class H30ImuAdapter:
    def open(self):
        self._serial = serial.Serial(
            port="/dev/ttyUSB0",
            baudrate=460800,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.01  # 10ms 超时，非阻塞读取
        )

    def read_sample(self):
        waiting = self._serial.in_waiting or 1
        data = self._serial.read(waiting)  # 读取缓冲区所有可用字节
        samples = self.parser.feed(data)   # 喂给帧解析器
        return samples[-1] if samples else None  # 取最新一帧
```

### 3.6 ROS2 发布

发布标准 `sensor_msgs/Imu` 消息：
```python
msg = Imu()
msg.header.stamp = ros_clock.now()
msg.header.frame_id = "imu_link"
msg.orientation.x, .y, .z, .w = quaternion  # 来自 0x41 或欧拉角转换
msg.angular_velocity.x, .y, .z = gyro_rps   # rad/s
msg.linear_acceleration.x, .y, .z = accel_mps2  # m/s²
# 协方差矩阵（对角线）
msg.orientation_covariance[0,4,8] = [0.05, 0.05, 0.10]
msg.angular_velocity_covariance[0,4,8] = [0.02, 0.02, 0.02]
msg.linear_acceleration_covariance[0,4,8] = [0.05, 0.05, 0.08]
```

### 3.7 配置文件

`src/wheelchair_bringup/config/h30_imu.yaml`:
```yaml
imu_adapter_node:
  ros__parameters:
    mode: real
    frame_id: imu_link
    publish_rate_hz: 100.0
    serial_port: /dev/ttyUSB0
    baud_rate: 460800
    serial_timeout_sec: 0.01
    orientation_covariance: [0.05, 0.05, 0.10]
    angular_velocity_covariance: [0.02, 0.02, 0.02]
    linear_acceleration_covariance: [0.05, 0.05, 0.08]
```

---

## 四、FD07-34 超声波传感器接入

### 4.1 硬件连接

- 接口：RS485 转 USB（Modbus RTU 协议）
- 默认串口：`/dev/ttyUSB1`（Linux）
- 波特率：9600 baud
- 数据格式：8N1
- 每个传感器有独立 Modbus 地址（出厂默认地址 1，需要逐个修改为不同地址）
- 当前启用 2 个（地址 1 和 2），预留 6 路

### 4.2 Modbus RTU 协议

读取距离值使用功能码 0x03（读保持寄存器）：

**请求帧：**
```
[地址(1B)] [功能码0x03(1B)] [寄存器起始地址(2B大端)] [寄存器数量(2B大端)] [CRC16(2B小端)]
```

示例：读取地址 1 的传感器，寄存器 0x0001，数量 1：
```
01 03 00 01 00 01 [CRC_L] [CRC_H]
```

**响应帧：**
```
[地址(1B)] [功能码0x03(1B)] [字节数(1B)] [数据(N字节)] [CRC16(2B小端)]
```

示例响应（距离 500mm）：
```
01 03 02 01 F4 [CRC_L] [CRC_H]
```
数据 `01 F4` = 500（大端 uint16），单位 mm。

### 4.3 CRC16 计算

```python
def modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

# CRC 附加到帧尾（小端序）
frame = request_without_crc + struct.pack("<H", modbus_crc16(request_without_crc))
```

### 4.4 轮询逻辑

```python
class UltrasonicArrayAdapter:
    def read_ranges(self) -> Dict[int, float]:
        values = {}
        for sensor in self.sensors:
            if not sensor.enabled:
                continue
            # 构建 Modbus 读取命令
            request = build_read_holding_registers(sensor.address, register=0x0001, count=1)
            # 清空接收缓冲区
            self._serial.reset_input_buffer()
            # 发送请求
            self._serial.write(request)
            # 读取响应（固定 7 字节：地址1 + 功能码1 + 字节数1 + 数据2 + CRC2）
            response = self._serial.read(7)
            # 解析响应
            address, registers = parse_response(response, sensor.address)
            distance_mm = registers[0]
            values[sensor.index] = distance_mm / 1000.0  # mm → m
            time.sleep(0.01)  # 传感器间短暂延迟，避免总线冲突
        return values
```

### 4.5 ROS2 发布

每个传感器发布到独立 topic，使用标准 `sensor_msgs/Range` 消息：

```python
# 固定发布 6 路（未启用的发布 max_range，避免安全模块误判为离线）
for i in range(6):
    msg = Range()
    msg.header.stamp = now
    msg.header.frame_id = f"ultrasonic_{i}_link"
    msg.radiation_type = Range.ULTRASOUND
    msg.field_of_view = 0.45  # ~26°
    msg.min_range = 0.03      # 3cm
    msg.max_range = 3.0       # 300cm
    msg.range = values.get(i, msg.max_range)  # 无数据时发布最大量程
    publisher[i].publish(msg)
```

### 4.6 配置文件

`src/wheelchair_bringup/config/ultrasonic.yaml`:
```yaml
ultrasonic_adapter_node:
  ros__parameters:
    mode: real
    publish_rate_hz: 20.0
    serial_port: /dev/ttyUSB1
    baud_rate: 9600
    serial_timeout_sec: 0.08
    register: 1                    # 保持寄存器地址 0x0001
    sensor_addresses: [1, 2]       # Modbus 从站地址
    sensor_indices: [0, 1]         # 对应 topic 索引
    enabled_count: 2               # 实际启用数量
    min_range: 0.03
    max_range: 3.0
    field_of_view: 0.45
```

---

## 五、坐标系与 TF 关系

所有传感器数据发布时带有 `frame_id`，通过 URDF 定义的静态 TF 变换到 `base_link`：

```
base_link (X前, Y左, Z上)
├── laser_link    — 雷达安装位置 (x=0.45, y=0, z=0.65, pitch=-5°)
├── imu_link      — IMU 安装位置 (x=0, y=0, z=0.45)
├── ultrasonic_0_link — 右前超声波 (x=0.45, y=-0.25, z=0.25, yaw=-30°)
├── ultrasonic_1_link — 正前超声波 (x=0.50, y=0, z=0.25, yaw=0°)
├── ultrasonic_2_link — 左前超声波 (x=0.45, y=0.25, z=0.25, yaw=30°)
├── ultrasonic_3_link — 左后超声波
├── ultrasonic_4_link — 正后超声波
└── ultrasonic_5_link — 右后超声波
```

这些外参定义在 `src/wheelchair_description/urdf/wheelchair.urdf.xacro` 中，是初始估计值，实际部署时需要用卷尺测量并在 RViz 中验证。

---

## 六、数据流总结

```
┌─────────────────────────────────────────────────────────────────────┐
│ 硬件层                                                               │
│                                                                     │
│ XT-M60 (以太网 TCP+UDP)                                             │
│ H30 IMU (串口 460800 baud, Yesense 二进制协议)                       │
│ FD07-34 超声波 ×2~6 (RS485 9600 baud, Modbus RTU 功能码 0x03)       │
└───────────────────────────┬─────────────────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────────────┐
│ ROS2 Adapter Nodes                                                   │
│                                                                     │
│ xtm60_adapter_node:                                                  │
│   SDK 回调 → 线程安全缓存 → 10Hz timer → /xtm60/points (PointCloud2)│
│                                                                     │
│ imu_adapter_node:                                                    │
│   串口读取 → Yesense 帧解析 → 100Hz timer → /imu/data (Imu)         │
│                                                                     │
│ ultrasonic_adapter_node:                                             │
│   Modbus 轮询 → CRC 校验 → 20Hz timer → /ultrasonic/*/range (Range) │
└───────────────────────────┬─────────────────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────────────┐
│ 感知处理层                                                           │
│                                                                     │
│ pointcloud_to_laserscan_node:                                        │
│   /xtm60/points → TF(laser_link→base_link) → 高度过滤 → 角度分bin   │
│   → /scan (LaserScan)                                                │
│                                                                     │
│ /scan → slam_toolbox (建图) 或 Nav2 costmap (导航)                   │
│ /imu/data → robot_localization EKF (融合轮速里程计)                   │
│ /ultrasonic/*/range → safety_supervisor_node (近距离急停)             │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 七、启动方式

```bash
# 全部传感器（真实模式）
ros2 launch wheelchair_bringup sensors.launch.py mode:=real

# 全部传感器（模拟模式，无硬件调试）
ros2 launch wheelchair_bringup sensors.launch.py mode:=mock

# 单独启动某个传感器节点
ros2 run wheelchair_sensors xtm60_adapter_node --ros-args \
  -p mode:=real -p ip_address:=10.55.231.101

ros2 run wheelchair_sensors imu_adapter_node --ros-args \
  -p mode:=real -p serial_port:=/dev/ttyUSB0

ros2 run wheelchair_sensors ultrasonic_adapter_node --ros-args \
  -p mode:=real -p serial_port:=/dev/ttyUSB1
```

---

## 八、关键注意事项

1. **XT-M60 的 IP 地址**：Codex 版默认 192.168.0.101，你的实际雷达可能是 10.55.231.101（参考 Kiro 版配置）。修改 `xtm60_sdk.yaml` 中的 `ip_address` 即可。

2. **point_unit_scale**：必须用实物标尺验证 SDK 输出的点云单位。如果 SDK 输出毫米，设为 0.001；如果输出米，保持 1.0。

3. **串口设备名**：Linux 上 USB 串口设备名不固定（/dev/ttyUSB0, 1, 2...），建议用 udev 规则固定设备名。

4. **超声波地址冲突**：所有超声波传感器共用一条 RS485 总线，每个必须有唯一 Modbus 地址。出厂默认都是地址 1，需要逐个修改。

5. **IMU 坐标系**：H30 输出的加速度和角速度是 Body 系（X前Y左Z上），与 ROS 的 `imu_link` 约定一致，无需额外旋转。

6. **XT-M60 坐标系**：SDK 默认 Camera 模式（Z 前方），代码中应配置为 Car 模式（X 前方）或在 URDF 中通过 TF 旋转补偿。Codex 版假设 SDK 输出已经是 X 前 Y 左 Z 上（Car 模式），frame_id 设为 `laser_link`，通过 URDF 中的 `laser_fixed_joint` 定义其相对 `base_link` 的位姿。
