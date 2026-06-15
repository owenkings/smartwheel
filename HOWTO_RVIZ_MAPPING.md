# SmartWheel — RViz 手动建图操作指南

面向当前阶段（单左雷达 + 双前向摄像头 + 手动遥控建图）。一步步照做即可。

---

## 0. 前提
- 已通过 NoMachine 连到轮椅主机的桌面。
- 左 XT-M60 雷达通电、网线已接（`192.168.0.101`）。
- 两个前向 USB 摄像头已插好。
- **底盘动力电已上、急停已松开**（最常见的"轮椅不动"原因就是动力电没上/急停被按）。
  开始前务必确认电机驱动器能使能（见 0.1）。
- 代码已构建过一次（若没有，见最后“构建”一节）。

### 0.1 确认电机驱动器能使能（轮椅能动的前提）
ZLAC8030D 驱动器必须先通动力电、松开急停，写使能控制字才会真正进入使能态。
开始建图前跑一次自检（需要串口空闲，即此时不要运行 base/建图栈）：
```bash
cd ~/smartwheel/tools/hardware_bringup
python3 probe_enable.py
```
- 看到 `after_enable: ... enabled=True` → 驱动器正常，轮椅能动。
- 一直 `enabled=False`（`0x20A2=0x8080`）→ 驱动器没使能：检查**动力电源开关**、**物理急停按钮**、驱动器报警灯。
  这种情况下任何软件指令都无法让电机转（这不是 ROS/代码问题）。

> 经验：曾出现"软件完全正常但轮椅不动"，根因就是动力电/急停未就绪，驱动器拒绝使能。
> 恢复硬件后 `probe_enable.py` 显示 `enabled=True`，轮椅立即可动。

---

## 1. 打开 RViz（看传感器，不动电机）

### 方式 A：只看传感器（阶段 0）
在**桌面终端**（NoMachine 里的终端，不是 SSH）运行：
```bash
cd ~/smartwheel
bash scripts/run_rviz_sensors.sh
```
脚本会自动找到正确的显示器并打开 RViz，显示：左雷达点云、`/scan`、左右摄像头、超声波、TF。

### 方式 B：建图 + 遥控（阶段 1，推荐）
见下面第 2 节。

> 如果脚本报 “no usable X DISPLAY”，在桌面终端先执行 `echo $DISPLAY` 和 `echo $XAUTHORITY`，
> 然后用这两个值运行：
> `DISPLAY=<值> XAUTHORITY=<值> bash scripts/run_rviz_sensors.sh`

---

## 2. 手动建图（核心流程）

### 2.1 安全准备（必做）
- 把轮椅**周围 0.5 米以上清空**（最好 1.5 米）。否则安全系统检测到近距离障碍会一直 `SLOWDOWN/STOP`，轮椅不会前进——这是正常避障，不是故障。
- 第一次测试建议**先把轮椅架空（驱动轮离地）**，确认方向正确再落地。
- 备好物理急停。

### 2.2 先用“只读模式”确认链路（电机不动）
```bash
cd ~/smartwheel
bash scripts/run_rviz_manual_mapping_left.sh
```
- 这会启动：传感器 + EKF + 安全层 + 底盘(只读) + 点云融合 + RTAB-Map + RViz。
- **此模式电机不会动**，用于确认一切正常。
- 另开一个终端验证：
  ```bash
  cd ~/smartwheel
  bash scripts/check_rviz_mapping_left.sh
  ```
  看到 `STAGE1_OK` 表示建图链路健康。

### 2.3 开启电机，真正手动建图
确认 2.1 安全准备完成后：
```bash
cd ~/smartwheel
MOTION=true bash scripts/run_rviz_manual_mapping_left.sh
```
⚠️ **此模式电机会真转。**

### 2.4 在 RViz 里驾驶
- RViz 左下/侧边有一个 **Wheelchair Teleop 面板**（按钮：Forward / Back / Left / Right / Stop）。
- 也可以用键盘（先点一下面板让它获得焦点）：
  - `W` 前进，`S` 后退，`A` 左转，`D` 右转，`空格` 停。
  - 速度可在面板里用 `v`、`w` 调整（建图时建议很慢：线速 0.05–0.1 m/s）。
- **支持边走边转（弧线）**：同时按住 `W`+`A` = 前进同时左转；`W`+`D` = 前进右转。按钮也可同时按住组合。
- 按钮/按键**按住才动、松开即停**。
- **手动模式不做传感器避障**（manual_bypass）：因为是人工驾驶，由你判断安不安全。雷达/超声波都**不会**自动挡你前进/转弯。**仍然有效的安全**：软件急停（空格/STOP）、硬件急停、指令超时（松手即停）、限速。
- 后退已开启（盲退，靠你看）。
- 慢速绕实验室走一圈，**转弯要慢**（雷达只有 120° 视野，转太快会丢特征）。

### 2.5 观察建图
RViz 里看这几项随你移动而增长：
- `RTAB-Map 3D Cloud Map`（`/rtabmap/cloud_map`）：累积的 3D 点云地图。
- `RTAB-Map Projected Grid`（`/rtabmap/grid_map`）：2D 栅格地图。
- 单帧点云（`Merged Live Cloud` / `Left XT-M60 Raw Cloud`）按**距离**上色（近/远不同颜色）。

### 2.6 保存地图
走完一圈后（程序保持运行），另开终端：
```bash
cd ~/smartwheel
bash scripts/save_mapping_result.sh lab_map_01
```
结果保存在 `maps/lab_map_01/`：
- `lab_map_01.db`：RTAB-Map 数据库（可重新导出/续建）
- `lab_map_01.ply`：导出的 3D 点云（可用 CloudCompare/MeshLab 打开）

### 2.7 结束
回到运行建图的终端按 `Ctrl+C` 停止。脚本会自动收尾：先 INT 直接子进程，再调用
`scripts/stop_mapping.sh` 扫掉整个子树（处理 ros2 launch 偶发逃逸/被 systemd 收养的节点）。

> 若发现退出后仍有残留（`ps -ef | grep wheelchair_` 还有节点，或 `ros2 node list` 出现
> 重复节点名），手动兜底全清：
> ```bash
> bash scripts/stop_mapping.sh          # 优雅(INT)→超时→强杀(KILL)
> bash scripts/stop_mapping.sh --force  # 直接强杀
> ```
> 启动脚本现在每次启动前也会自动预清理上一轮残留，避免 `zlac8030_driver_node` 多份争串口。

### 2.8 查看建好的地图
保存后 `maps/<name>/` 里有三种文件，各用不同方式查看（都在 NoMachine 桌面里开）：

**2D 栅格地图（`<name>.pgm`）** — 最简单，看俯视轮廓：
```bash
eog ~/smartwheel/maps/lab_map_01/lab_map_01.pgm
```
黑=障碍/墙，白=可通行，灰=未知。能直接看出走过的轮廓。

**3D 点云（`<name>_cloud.ply`）** — 需要 3D 查看器：
- 已自带（推荐，无需安装）：用 RTAB-Map 数据库查看器看 db 里的 3D 点云 + 位姿图 + 回环：
  ```bash
  source /opt/ros/humble/setup.bash
  rtabmap-databaseViewer ~/smartwheel/maps/lab_map_01/lab_map_01.db
  ```
  打开后菜单 View → 勾选 3D views / Graph view 即可看点云和轨迹。
- 可选（需要你自己 sudo 安装，更通用的 .ply 查看器）：
  ```bash
  sudo apt install meshlab        # 或 cloudcompare
  meshlab ~/smartwheel/maps/lab_map_01/lab_map_01_cloud.ply
  ```

**RTAB-Map 数据库（`<name>.db`）** — 完整检查（位姿图、关键帧、回环、可重新导出点云）：
```bash
rtabmap-databaseViewer ~/smartwheel/maps/lab_map_01/lab_map_01.db
```

> 快速验收：先 `eog` 看 2D pgm 轮廓对不对，再 `rtabmap-databaseViewer` 看 3D 细节。

---

## 3. RViz 使用小贴士

### 调整点云点的大小 / 颜色（界面里实时改）
左侧 **Displays** 面板 → 展开某个点云项（如 `Merged Live Cloud`）：
- `Size (Pixels)`：点的像素大小（当前 2，想更明显可调大）。
- `Style`：`Points`（细点）/ `Flat Squares`（方块，更显眼）。
- `Color Transformer`：`AxisColor` + `Axis: X` = 按前方距离上色；`Intensity` = 按反射强度。

### 保存你的排版（重要！）
你拖动面板/改了显示设置后，**必须保存**，否则重启会还原：
- 菜单 **File → Save Config**，或快捷键 **`Ctrl+S`**。
- 它会存回当前加载的配置文件：
  `src/wheelchair_bringup/rviz/manual_mapping_left.rviz`（阶段1）
  或 `src/wheelchair_bringup/rviz/sensor_view.rviz`（阶段0）。
- 存完后下次启动就是你的新排版。
- 若 `Ctrl+S` 没反应，用 **File → Save Config As**，手动定位到上述文件覆盖保存。

---

## 4. 常见问题

| 现象 | 原因 / 解决 |
|---|---|
| RViz 打不开，报 no DISPLAY | 在桌面终端取 `echo $DISPLAY; echo $XAUTHORITY`，用它们带进命令前缀运行 |
| 点云/scan 不显示，只有相机 | Fixed Frame 设错。建图时用 `map`（RTAB-Map 提供）；只看传感器用 `odom` |
| **按住前进/后退轮椅完全不动** | 先查驱动器是否使能：`cd tools/hardware_bringup && python3 probe_enable.py`，若 `enabled=False` 则是动力电没上/急停被按，不是软件问题 |
| 轮椅按了前进不动（safety 挡） | 前方 60cm 内有真障碍被安全层挡（`STOP`），清空前方 |
| 后退不动 | 严格配置禁止倒车；手动建图配置已开启倒车。确认用的是 `safety_params_manual_mapping.yaml`（manual_mapping/teleop 默认就是它） |
| 指令在动/不动间抖动 | 命令行 `ros2 topic pub` 和 RViz 面板抢 `/cmd_vel_nav`。只用面板驾驶，别再手动 pub |
| 雷达点云突然没了几秒 | XT-M60 偶发瞬时断流，会自动重连恢复，等几秒 |
| 退出后还有节点残留 / `ros2 node list` 有重复节点名 | 跑 `bash scripts/stop_mapping.sh` 兜底全清；启动脚本已会自动预清理上一轮残留 |
| 排版重启又变回去 | 没保存，改完按 `Ctrl+S` |

---

## 5. 构建（仅首次或改了代码后）
```bash
cd ~/smartwheel
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 6. 安全红线
- 一切运动都经过安全层：`遥控 → /cmd_vel_nav → safety_supervisor → /cmd_vel_safe → 底盘`。
- 默认 `motion_control_enabled:=false`（只读）。只有 `MOTION=true` 才会真正驱动电机。
- 右雷达当前硬件损坏，本流程只用左雷达（单雷达建图，正常）。
