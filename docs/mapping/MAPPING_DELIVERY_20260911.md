# 2026-09-11 建图入口、自动录制与地图保存交付

## 结论与边界

Orin 重启后 SSH 已恢复。此次只修改 `/home/nvidia/smartwheel`，分支为 `local/formal-3d-hardening-20260908`，HEAD 为 `103b569c0156c90bfcb6647e5da15fc12035b993`。保留原有脏工作区，没有暂存、提交、推送或操作 `/home/nvidia/smartwheel-rtab-old`。GitHub origin 在线 heads 查询因 TLS 错误未刷新。

根据用户最新“FAST-LIO2 不强制、尽快推进建图”的要求，当前交付入口采用：实测轮速前向速度＋H30 偏航角速度 → 平面 EKF → 新鲜度/反馈健康检查 → 连续车辆 TF；原始右雷达三维 XYZI＋该位姿 → RTAB-Map 累积地图及空间 ICP 回环候选。FAST-LIO 默认不启动，代码保留，可显式启用隔离旁路；它不是当前地图或车辆位姿来源。

这交付了可运行、可记录、可导出和重载的软件流程，**没有完成新安装状态下的实车地图精度、回环、外参、坡道或导航验收**。严格 Stage 1 标定合同仍为 BLOCKED_CONFLICT，未修改门禁。新入口使用的是明确标注的 provisional 诊断布局，不是通过正式标定门禁的产品入口。

## 本轮具体改动

1. `scripts/run_wheel_imu_mapping.sh` 默认 `FASTLIO_SHADOW=false`、`MOTION=false`。启动硬件前自动录制原始右点云、时间/质量、IMU、实测轮速、反馈健康、安全速度、有效位姿、TF；每 300 秒分包，磁盘少于 2 GiB 时拒绝启动。磁盘阈值是启动检查，不是运行中的磁盘空间保证。
2. `right_lidar_diag_mapping.launch.py` 为旁路 LIO 与输入适配器增加开关；`wheel_primary_pipeline.py` 同步关闭无数据意义的一致性监视器。原 FAST-LIO 诊断入口没有被删除。
3. 退出时保留原先“只清理本入口拥有的进程、先电机驱动再雷达 SDK”的次序，随后结束录包；在硬件/建图进程退出后自动导出本次数据库。退出不能证明物理急停闭环；现场必须确认车轮实际停止。
4. 新增 `scripts/mapping/wheel_map_bundle.py`：只读打开源 SQLite、备份到新目录，在仅本机 ROS domain 93 的私有数据库上调用 RTAB 服务；生成 XYZI PCD/PLY、二维 PGM/YAML、优化轨迹 CSV、地图消息和数据库快照、带 SHA-256 校验的 manifest。已有输出目录禁止覆盖；失败目录保留 `.incomplete` 并拒绝重载。首次验证发现当前 RTAB 的二维服务是 `/rtabmap/get_map`，已修正，未保留错误的 `/rtabmap/get_grid_map` 调用。
5. 新增离线 RViz 地图预览，不启动传感器或电机。文件保存完整与地图精度通过是两个独立状态，manifest 明确将 hardware/accuracy/extrinsics/loop_closure 验证设为 false。

## 验证证据

- `bash -n scripts/run_wheel_imu_mapping.sh` 通过。
- `wheelchair_3d_mapping`、`wheelchair_bringup` 编译安装通过；最后 launch 修改后再次构建 bringup 通过。
- `python3 -m pytest src/wheelchair_3d_mapping/test -q`：**195 passed in 24.97s**。
- 使用已存在的历史录制回放数据库 `auto_test/wheel_primary_full_replay_20260910_delivery/wheel_primary_rtabmap.db`，不是本轮新实车数据。
- 导出目录：`auto_test/mapping_delivery_20260911/bundle_check_v2`。80 个扫描/轨迹节点，**221842 个 XYZI 点**，二维栅格 **421×421，0.05 m/格**。返回 `export_complete=true`。
- `preview --no-gui --seconds 3` 通过：从保存消息重新组装得到相同 221842 点，所有清单文件校验通过，`reload_verified=true`。这验证数据重载/发布代码，不等于已经人工检查 RViz 画面质量。
- 图中的 79 条 type=0 链接为邻接链接；该样本没有可用来宣称成功闭环的空间回环。源数据库导出前后 SHA-256 相同：`346670084b29d72b16a0c2c231e236e381a84cb458caa92b68596ad13df672eb`。
- 本轮没有启动真实设备、驱动电机、修改曝光或写入新的外参。原文件备份位于 `auto_test/mapping_delivery_20260911/`。
- 最后进程检查未发现 XT-M60/H30/ZLAC/FAST-LIO/RTAB 遗留进程；安装后的 launch 参数检查确认 `enable_fastlio_shadow` 默认 false。工作区仍非干净状态，未擅自清理；全树 `git diff --check` 报出既有 FAST-LIO 补丁文件内的尾随空格，未改写这份非本轮修改的补丁，不能宣称全树空白检查通过。

## 安装照片与外参：尚不能当作标定值

用户卷尺测量上下沿约 760/700 mm，其平均值 730 mm 只是外形中点，不是厂家确认的 SDK 光学原点。照片显示 H30 安装在蓝色集成外壳的顶盖上，不是直接贴住雷达金属壳；因此不能把蓝盒顶面、雷达外壳顶面和光学中心混为一谈。照片也不能精确测定相对旋转。此次没有将 730 mm 或照片估角写进正式 TF。

此前本地 STEP 的包围尺寸：XT-M60 整体约 65.0004×38.5×80.4792 mm（含后部连接结构）；H30 标准金属壳约 46×59.5×11.7 mm。CAD 建模原点尚未证明是 SDK 测量原点。用户的 4 mm/2 mm 边缘偏移仍需明确是相对哪个实体的边缘。

当前保留双雷达 z=0.735 m 的既有 provisional profile；公共 x=0.45 m、yaw、实际安装旋转仍未最终验证。H30 的旧 xyz 也没有被包装成此次新位置的测量结果。选定 EKF 只用轮速 vx 与变换后的 gyro yaw rate，不用 IMU 加速度积分或磁航向，故不需要先猜出新的 IMU 平移才能试运行平地建图；但安装轴向/旋转仍然影响偏航角速度，不能省略后续核验。

## 用户完整操作

先正常退出其他建图终端，然后在 **NoMachine 中的 Orin 图形桌面终端**执行，不是在 Windows PowerShell 中执行：

```bash
cd /home/nvidia/smartwheel
source /opt/ros/humble/setup.bash
source install/setup.bash
CAMERAS=true ULTRASONIC=true MOTION=false FASTLIO_SHADOW=false \
  bash scripts/run_wheel_imu_mapping.sh
```

此命令禁止电机写入，WASD 不会驱动车轮；不是故障。需要进行已获允许的现场人工驾驶时，确认无人乘坐、场地清空、物理急停可用且原有运动安全条件满足，再将 `MOTION=false` 改为 `MOTION=true`，由现场操作者控制。不要用本入口进行自主导航或载人测试。

每次自动新建 `maps/wheel_imu_时间_随机后缀/`，不覆盖旧地图。启动期间不要移动，等有效点云/位姿出现后再进行现场允许的低速操作。结束时先 STOP/空格确认实际停车，再在启动终端按 Ctrl+C，等待 `Map files exported:`。不要直接断电、杀进程或关闭终端。

输出目录包含：`raw/` 原始数据、`rtabmap.db`、`ros_logs/`、`recorder.log`、`export.log`、`products/`。复制终端打印的 `Preview:` 完整命令即可离线重开本次地图；例如本轮已有样本可执行：

```bash
cd /home/nvidia/smartwheel
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/mapping/wheel_map_bundle.py preview \
  --bundle auto_test/mapping_delivery_20260911/bundle_check_v2
```

若输出 `EXPORT FAILED`，不能称作保存成功；保留整个会话目录。不要删除原始包或通过调宽定位阈值掩盖错误。

## 建图阶段仍需关闭的项目

在当前安装状态做一轮真实短路线并保留自动录包：静止、已知直线距离、原路返回、转向及返回起点。对照实测距离/方向与轮速尺度、IMU 转角，再评估平面地面、地图重影与回环前后误差。无独立实测真值不能认定“已无漂移”。若要求坡道/完整六自由度车身姿态或恢复 FAST-LIO 为主，需要另外完成光学/IMU 原点、旋转和时间标定及独立 LIO 验证；本轮没有以平面约束冒充六自由度修复。
