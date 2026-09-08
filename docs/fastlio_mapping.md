# FAST-LIO2 LiDAR-惯性建图与验收记录

本文档记录 SmartWheel 将 3D 建图从「纯 EKF 位姿 + RTAB-Map 零配准」重构为
**FAST-LIO2(LiDAR-惯性里程计)** 的获取、依赖、编译、外参、启动与保存等可复现信息。

对应 spec:`.kiro/specs/fastlio-narrow-fov-mapping/`(需求 1.1 / 1.3 / 1.5)。

> **当前状态（2026-09-07；覆盖下文早期任务状态描述）**：FAST-LIO2 的软件加固、
> 时间戳传播和离线回放入口已完成代码级收口；这不等于真实传感器、动态运动或最终地图验收。
> - 权威 FAST-LIO2 补丁是 patches/fastlio_smartwheel_hardening.patch，脚本会锁定基线、补丁
>   及两个修改后源文件的 SHA-256，并拒绝旧的 R=100/负 age ZUPT。
> - XT-M60 在 SDK 设备时间未启用时仍使用 SDK 回调的主机接收时间代理；H30 使用串口读取边界之间的
>   主机侧名义频率插值。两者都不是真实设备采样时钟，也没有证明 LiDAR–IMU 固定延迟。
> - 点云→scan→scan merge/fusion 会保留最新的非零源时间戳；地图输出时间戳是后端生成时间，需按产品
>   语义解释，不能倒推传感器采样时间。
> - LiDAR/IMU/base 外参、H30 动态轴向与 yaw/xyz、双雷达时间同步和重叠区融合仍为临时状态；
>   right_lidar_stage1_calibration_contract.json 仍是 BLOCKED_CONFLICT。
> - H2 离线完整回放已通过带时间窗说明；FAST-LIO2 实机、累计三维建图、导航、地面运动、物理急停和
>   载人安全均未通过，硬件任务结束后必须停机并确认无进程/UDP 残留。
>
> 下文较早的任务编号、候选外参和 bag 数字是历史证据记录；当前计划、授权边界和最新结论以
> docs/PROJECT_HANDOFF_CURRENT.md 与 docs/PROJECT_MEMORY_CURRENT.md 为准。
---

## 1. 仓库选型与锁定 commit(req 1.1 / 1.3 / 1.5)

FAST-LIO2 上游主仓库(`hku-mars/FAST_LIO`)以 ROS 1 为主;为在 **ROS 2 Humble / aarch64(Jetson Orin)**
上运行,本项目采用社区维护、明确支持 ROS 2 Humble 的 fork:

| 项 | 值 |
|---|---|
| 仓库 URL | `https://github.com/Ericsii/FAST_LIO_ROS2.git` |
| 分支 | `ros2` |
| 锁定 commit(FAST_LIO_ROS2) | `2fffc570a25d0df172720bac034fbdb6a13d2162` |
| commit 日期 / 摘要 | `2025-11-17 13:58:05 +0800` — `fix: #65 Fix typo in add_action for config file` |
| 包名(package.xml `<name>`) | `fast_lio`(可执行:`fastlio_mapping`) |
| vendored 路径 | `src/third_party/FAST_LIO_ROS2/` |
| 子模块 ikd-Tree URL | `https://github.com/hku-mars/ikd-Tree.git`(branch `fast_lio`) |
| 子模块 ikd-Tree 锁定 commit | `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4` |
| ikd-Tree vendored 路径 | `src/third_party/FAST_LIO_ROS2/include/ikd-Tree`(`--recursive` 拉取) |

获取命令(已执行,task 1):

```bash
mkdir -p src/third_party
git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive \
    src/third_party/FAST_LIO_ROS2
```

复现/回滚到锁定 commit:

```bash
git -C src/third_party/FAST_LIO_ROS2 checkout 2fffc570a25d0df172720bac034fbdb6a13d2162
git -C src/third_party/FAST_LIO_ROS2 submodule update --init --recursive
# 校验:
git -C src/third_party/FAST_LIO_ROS2 rev-parse HEAD       # -> 2fffc57...
git -C src/third_party/FAST_LIO_ROS2 submodule status     # -> e2e3f4e... include/ikd-Tree
```

> 平台事实(已核实,无需重测):aarch64 / Jetson Orin;`/etc/nv_tegra_release` = R36(JetPack 6);
> ROS 2 `humble`;gcc 11.4;cmake 3.22.1。该 fork README 明确支持 ARM 平台(TX2/Khadas/RPi)。

---

## 2. `livox_ros_driver2` 硬依赖分析与处置决策(req 1.1 / 1.3)

### 2.1 现状(已核实)

XT-M60 **不是** Livox 传感器,本项目不希望背上 Livox 工具链。检查 vendored 源码后确认
`Ericsii/FAST_LIO_ROS2`(commit `2fffc57`)**确实对 `livox_ros_driver2` 存在硬依赖**:

- `package.xml`:`<depend>livox_ros_driver2</depend>`。
- `CMakeLists.txt`:`find_package(livox_ros_driver2 REQUIRED)`,并加入 `dependencies` 列表
  传给 `ament_target_dependencies(...)`。
- 源码 `src/preprocess.h` / `src/preprocess.cpp`:
  - `#include <livox_ros_driver2/msg/custom_msg.hpp>`;
  - `Preprocess::process(const livox_ros_driver2::msg::CustomMsg::UniquePtr&, ...)` 与
    `avia_handler(...)` 的入参类型是 `livox_ros_driver2::msg::CustomMsg`。
- `laserMapping.cpp` 订阅时按 `lidar_type` 选择 Livox CustomMsg 或标准 `PointCloud2` 回调。

关键判断:Livox 仅在 **`lidar_type == AVIA`(Livox CustomMsg)** 这条输入路径上被实际使用。
本项目走 **Velodyne / Ouster 标准 `sensor_msgs/PointCloud2` 路径**(`velodyne_handler` /
`oust64_handler`),该路径在运行期**不触碰** Livox 消息;但因 `package.xml` / `CMakeLists.txt` /
头文件 `#include` 的强引用,**编译期仍需要解析 `livox_ros_driver2` 这个包**(否则
`find_package(... REQUIRED)` 与 `#include` 会失败)。

### 2.2 处置决策(本任务记录,编译在任务 2 落地)

本任务**不编译**(留任务 2),仅记录决策方向。可选方案:

- **方案 A(推荐,Ericsii 官方建议):提供 `livox_ros_driver2` 的「standard-unit」消息包以满足编译,
  但运行期不使用 Livox。**
  README §1.3 指向作者维护的 `https://github.com/Ericsii/livox_ros_driver2`(分支
  `feature/use-standard-unit`)。它只提供 `CustomMsg` 等消息定义(满足 `find_package` 与
  `#include`),不强制连接真实 Livox 雷达。我们仍然只用标准 `PointCloud2` 输入路径喂 XT-M60,
  Livox 包仅作为「消息接口存在」以通过编译。**优点**:对上游源码零改动,锁 commit 干净,后续
  跟随上游更新成本低。**缺点**:多一个仅为编译存在的依赖包。

- **方案 B:在 vendored 源码中「绕过」Livox。**
  移除 `package.xml` / `CMakeLists.txt` 的 `livox_ros_driver2` 引用,并在 `preprocess.h/.cpp`
  里用条件编译宏剥离 `avia_handler` 与 `CustomMsg` 相关代码,仅保留 `velodyne_handler` /
  `oust64_handler`。**优点**:彻底去掉 Livox 依赖。**缺点**:改动 vendored 第三方源码,违背
  「锁 commit、零改动」初衷,后续跟随上游更新需重复打补丁,维护成本高。

**初步决策:优先采用方案 A**(引入仅提供消息定义的 `Ericsii/livox_ros_driver2`
standard-unit 分支,运行期不接 Livox 硬件)。该决策保持 vendored FAST-LIO 源码不被修改,
符合「锁 commit、可回滚」原则;XT-M60 仍走标准 `PointCloud2` 路径(deskew 关闭,详见 spec 需求 2)。
最终是否需要引入该消息包、以及具体 commit,将在 **任务 2(依赖安装 + colcon build)** 中确认并回填本文档。

> 注意:本阶段**不**安装真实 `livox_ros_driver2` 驱动,也**不**连接任何 Livox 硬件;
> 引入的仅是消息接口定义(若方案 A 落地)。

### 2.3 最终落地(任务 2 已确认,req 1.1 / 1.3)

任务 2 已按**方案 A**落地并验证可编译。要点:

| 项 | 值 |
|---|---|
| 仓库 URL | `https://github.com/Ericsii/livox_ros_driver2.git` |
| 分支 | `feature/use-standard-unit` |
| 锁定 commit | `378d1c7c9d33ec44d0683aef885b6ed0cce9612c` |
| commit 日期 / 摘要 | `2025-07-15 16:57:01 +0800` — `feat: Update for arm64` |
| 包名(package.xml `<name>`) | `livox_ros_driver2` |
| vendored 路径 | `src/third_party/livox_ros_driver2/` |

获取命令(已执行,task 2):

```bash
git clone --branch feature/use-standard-unit --recursive \
    https://github.com/Ericsii/livox_ros_driver2.git \
    src/third_party/livox_ros_driver2
```

**关键验证**:该分支虽然 `CMakeLists.txt` 仍编译完整 ROS2 驱动(含预编译 `livox_sdk` 静态库,
仓库已自带 `livox_sdk/lib/aarch64/liblivox_lidar_sdk_static.a`,**无需** 另装 Livox-SDK),
但其 `rosidl_generate_interfaces` 在 `livox_ros_driver2` 包名下生成消息头,实测产出:

```
install/livox_ros_driver2/include/livox_ros_driver2/livox_ros_driver2/msg/custom_msg.hpp
```

这正是 FAST-LIO 源码 `#include <livox_ros_driver2/msg/custom_msg.hpp>` 所需的头文件路径与命名空间,
因此 `find_package(livox_ros_driver2 REQUIRED)` 与 `#include` 编译期均得到满足。
运行期我们**不**启动该驱动节点(不连任何 Livox 硬件),XT-M60 仍走标准 `PointCloud2` 路径。

> 复现/回滚到锁定 commit:
> ```bash
> git -C src/third_party/livox_ros_driver2 checkout 378d1c7c9d33ec44d0683aef885b6ed0cce9612c
> git -C src/third_party/livox_ros_driver2 rev-parse HEAD   # -> 378d1c7...
> ```

> 备注(诚实标注):方案 A 的代价是把一个完整的 Livox 驱动包(及其预编译 SDK 静态库)纳入工作区
> 仅为满足编译;但其换来 vendored FAST-LIO **零源码改动**、锁 commit 可回滚,符合本特性「集成外部
> 算法、不打补丁」的初衷。`apr`(`libapr1`)在本机**未安装**,livox CMake 中 `pkg_check_modules(APR)`
> 为可选(`if(APR_FOUND)`),缺失不影响编译,故未安装。

---

## 3. 系统依赖(任务 2 已核实,req 1.2 / 1.4)

平台:aarch64 / Jetson Orin,ROS 2 Humble,gcc 11.4.0,cmake 3.22.1。

### 3.1 apt 系统依赖

设计列出的 4 个 apt 依赖**在本机均已安装**,任务 2 实测无需新装:

```bash
sudo apt update
sudo apt install -y libpcl-dev libeigen3-dev ros-humble-pcl-ros ros-humble-pcl-conversions
```

| 包 | 本机已装版本 | 说明 |
|---|---|---|
| `libpcl-dev` | `1.12.1+dfsg-3build1` | PCL(`find_package(PCL ... common io)`) |
| `libeigen3-dev` | `3.4.0-2ubuntu2` | Eigen3(`find_package(Eigen3)`) |
| `ros-humble-pcl-ros` | `2.4.5-2jammy` | `find_package(pcl_ros)` |
| `ros-humble-pcl-conversions` | `2.4.5-2jammy` | `find_package(pcl_conversions)` |

### 3.2 其余 `find_package` 依赖(随 ros-humble-desktop / Python 已满足)

FAST_LIO `CMakeLists.txt` 还 `find_package`:`rclcpp`、`rclcpp_components`、`geometry_msgs`、
`nav_msgs`、`sensor_msgs`、`std_msgs`、`std_srvs`、`visualization_msgs`、`rosidl_default_generators`
(均随 Humble 桌面版到位)、`OpenMP`(QUIET,可选)、`PythonLibs`(`libpython3-dev` / `python3-dev`
`3.10.6` 已装)。

- **matplotlibcpp**:`find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")`。该头文件**已 vendored**
  在 `src/third_party/FAST_LIO_ROS2/include/matplotlibcpp.h`,配合系统 `PythonLibs` 即可,**无需**额外
  安装 `python3-matplotlib` 开发头。
- **Sophus / TBB / Ceres**:本 fork **不依赖**,任务 2 实测无需安装(设计 §风险 5 的预备 apt 命令未触发)。

### 3.3 livox_ros_driver2 依赖

见上文 §2.3:克隆 `Ericsii/livox_ros_driver2`(`feature/use-standard-unit`,commit
`378d1c7c…`)到 `src/third_party/`,仅为满足编译期 `find_package` 与 `#include`,自带 aarch64
预编译 SDK,**无需**额外 apt。

### 3.4 rosdep

```bash
rosdep install --from-paths src --ignore-src -y
```

`rosdep` 对 `fast_lio` / `livox_ros_driver2` 的键均可解析。注意:工作区其它本地包会报
`Cannot locate rosdep definition for [ament_pytest]` / `[wheelchair_voice_agent]` 等——这些是
**项目内部包的 rosdep 键缺失**,与 FAST-LIO 编译无关,不影响 `--packages-up-to fast_lio`。

---

## 4. 编译(任务 2 已验证,`BUILD_EXIT=0`)

工作命令(在工作区根 `/home/nvidia/smartwheel` 执行):

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to fast_lio
```

实测结果(2026-06-22):

```
Starting >>> livox_ros_driver2
Finished <<< livox_ros_driver2 [0.91s]
Starting >>> fast_lio
Finished <<< fast_lio [2min 26s]
Summary: 2 packages finished [2min 28s]
  1 package had stderr output: fast_lio        # 仅告警, 见下
RC=0  (BUILD_EXIT=0)
```

产物(可执行):`install/fast_lio/lib/fast_lio/fastlio_mapping`(symlink-install)。

**stderr 仅为告警,非错误**(不影响 `BUILD_EXIT=0`):
- boost `bind.hpp` 全局占位符弃用 `#pragma message`(来自 PCL → boost,非本项目代码)。
- GCC `note: parameter passing for argument of type 'std::pair<double,double>' ... changed to match
  C++14`(IKFoM 模板实例化的 ABI 提示,非错误)。
- `CMP0074 / PCL_ROOT` policy 提示(开发者级警告)。

> 编译顺序提示:`--packages-up-to fast_lio` 会先编 `livox_ros_driver2`(约 31s 首次,之后增量
> < 1s)再编 `fast_lio`(约 2.5 min,aarch64 首次)。aarch64 上首次编译数分钟属正常。

---

## 5. 历史离线回放记录（证据，不代表当前硬件验收通过）

本节保留早期任务的回放统计，供追溯使用；原始 bag 的实际留存状态和可复现入口以当前交接文档为准。\n\n任务 4 已一次性录制离线 bag(左 XT-M60 + IMU + TF,含温和运动),作为**任务 5–9、13 的唯一离线回放源**,
后续离线调参/集成**无需再次上电雷达**(雷达过热硬约束)。

| 项 | 值 |
|---|---|
| **留存(唯一回放源)路径** | `/home/nvidia/smartwheel/auto_test/lio_canonical_bag` |
| 采集原始路径 | `/home/nvidia/smartwheel/auto_test/20260622_lio_bag_recapture/bag` |
| 采集报告 | `auto_test/20260622_lio_bag_recapture/report.md` |
| 录制话题 | `/xtm60/left/points` `/imu/data` `/tf` `/tf_static` |
| Duration | 18.54 s |
| `/xtm60/left/points` | 182 帧,~9.8 Hz |
| `/imu/data` | 3708 帧,**~200 Hz**(实测,高于标称 100 Hz — 配置 LIO 时以实测为准) |
| `/tf` / `/tf_static` | 556 / 1 |
| 运动 | **实测确认**:前进 0.26m + 慢 yaw 转 ~103° + 后退;`/wheel/odom` 净位移 0.44m / 偏航 103° |
| 雷达上电时长 | 25 s(< 30 s 硬上限) |

> **重要(重做记录)**:第一版 bag(`20260622_142516_lio_bag_capture`)发布了 cmd_vel 但**轮椅未实际移动**
> (odom 净位移 0.0005m/0.45°),根因为 `manual_teleop` 满负载启动竞态,非硬件/驱动缺陷
> (已由直接 Modbus 诊断 `20260622_motor_feedback_diag` 与 base-only 诊断 `20260622_base_only_motion` 证明
> 电机与 ROS 驱动均能正常驱动)。本 canonical bag 为**重做后、经 `/wheel/odom` 实时确认有真实运动**的版本;
> 旧静止 bag 归档为 `auto_test/lio_canonical_bag_stationary_old`,不再使用。

离线回放(任务 5–9、13 通用):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 bag play auto_test/lio_canonical_bag   # 雷达 OFF,纯离线
```

> 注意:`/imu/data` 实测 ~190 Hz(非文档标称 100 Hz);任务 5/7 配置 FAST-LIO 时不要硬编码 100 Hz,
> 以回放实测频率为准。

## 6. 外参、启动、地图保存、RTAB-Map 后端开关

### 6.1 LiDAR↔IMU 外参（当前仍未完成，历史候选不可用于产品）

FAST-LIO 的 extrinsic_T/R 约定是雷达在 IMU 坐标系中的位姿：
T_il = inv(base→imu) · (base→lidar)。早期任务曾从文档值和旧 bag 推导候选矩阵，
但那些数据没有同时满足最终安装姿态、时间语义和独立几何约束，**不能称为“任务 5 已落地”**，
也不能直接作为正式地图或导航标定。

当前合同保持 fail-closed：

- right_lidar_stage1_calibration_contract.json 为 BLOCKED_CONFLICT，没有批准的 runtime transform；
  左/右 LIO YAML 中的数值仅供诊断或离线兼容，均标为 provisional。
- 当前固定 TF 只用于方向/显示和受限诊断。用户已确认两光学中心横向间距约 0.60 m、相对 fore/aft
  偏移为 0，因此不得用虚构的非零相对 x 去补偿场景误差。
- 右侧临时旋转来自一次地面拟合，左侧旋转沿用历史可靠候选；两者的 yaw、绝对 x、H30 动态轴向、
  LiDAR–IMU 固定延迟和双雷达重叠区都尚未独立验证。
- 真实标定须使用带动作 marker 的静态/直行/转弯数据，并在解除合同阻塞后再写入产品配置；
  在此之前不要把 FAST-LIO 输出或地图当作正式产品。

相关实测证据和待办见 docs/hardware/FASTLIO_H2_REAL_EXTRINSIC_TIME_20260906.md
及 FASTLIO_HARDWARE_VALIDATION_PLAN.md。

### 6.2 `LASER_POINT_COV` 历史结论更正(2026-09-04)

2026-06 的偏航实验曾把 `LASER_POINT_COV` 从 `0.001` 逐步放大到 `100.0`。该实验只说明在当时的
短动态数据、未完成外参/时间同步且缺少更新门禁的条件下,减弱 LiDAR 权重可让输出偏航更接近 IMU;
它**不能证明 `R=100` 是通用根因修复**。后续静止实机数据证明该值会放大不可观方向的预测漂移并
造成灾难性假运动:修复前净假位移 `1749.10 m`,最大单帧 `75.27 m`。

当前正式实现如下:

| 项目 | 当前值/行为 | 目的 |
|---|---|---|
| `mapping.laser_point_cov` | 默认及右/左配置均为 **`0.001`** | 恢复上游量级;它现在是 ROS 参数,不再是必须改源码的宏 |
| LiDAR 更新门 | 特征数、残差、有限值、可观秩和状态增量联合检查 | 阻止坏匹配进入状态 |
| 回滚 | 更新超限时同时恢复状态和协方差,且不写入 ikd-tree | 避免拒绝后污染地图 |
| 退化 | 可观秩不少于 3 时把更新投影到可观子空间;更严重时才整帧拒绝 | 保留走廊/单平面中的有效约束 |
| 合法大修正 | 仅当特征/残差/可观性均合格而状态增量超限时,从同一快照用 `R=.004/.016/.064` 最多重算 3 次;最终仍须满足原增量门 | 用多个有界小修正逐步重捕获,避免长期整帧拒绝;绝不恢复全局 `R=100` |
| IMU 噪声 | 左/右配置恢复保守的上游量级 | 不再用极小 bias 协方差“锁死”状态 |

静止实机最终结果为最大单帧 `4.67 cm`,90 秒后活动半径约 `1.12 cm`。这证明灾难发散已被阻断,
但直行、转弯、原地转向、STOP 恢复和长输出空窗后的重新收敛仍须作为动态验收项目,不能由静止
结果外推。

### 6.3 启动、地图保存、RTAB-Map 后端开关

启动与授权边界见当前交接文档；右雷达入口只是受限诊断入口，不代表最终外参或产品建图已批准。完整会话地图保存必须在建图开始前启动
`map_products_node`，结束时再调用 `scripts/save_mapping_result.sh`；脚本会先调用
`/map_session/stop`，再调用 `/map_export/export`，只接受正式会话的原子产品包。
旧的 `lio_save_cloud.py` 仍可用于短时固定窗口诊断，但不再被视为完整地图保存器，
也不会被 `save_mapping_result.sh` 在结束时偷偷启动。

手动 FAST-LIO 的软件编排示例（仅说明会话合同，不构成真实右雷达或车辆测试授权）：

```bash
ros2 launch smartwheel_bringup manual_mapping_lio_left_session.launch.py \
  map_name:=manual_lio_left \
  output_root:=maps/versions \
  motion_control_enabled:=false

# 上面的包装入口会先启动保存节点，再启动左侧 FAST-LIO；建图结束后、
# 关闭 launch 前执行：
bash scripts/save_mapping_result.sh
```

若正式服务不可用，保存脚本会失败关闭，不会退回六秒末尾采集。右雷达合同仍为
`BLOCKED_CONFLICT`；任何真实传感器、电机或车辆操作必须另行经过硬件安全批准。

**RTAB-Map 可选回环后端**（`enable_loop_backend:=true`,默认 off）:
- 以 external-odom 模式消费 FAST-LIO 位姿（`odom_topic=/Odometry`、`points_topic=/cloud_registered`、
  `Reg/Strategy=0`），仅做回环/位姿图/导出,不抢前端里程计（FAST-LIO 仍是唯一实时位姿源）。
- **TF 对齐（Task 6）**:FAST-LIO 把位姿发布在 `camera_init → body → base_link` 链上
  （`camera_init` 是其世界原点,`/Odometry` 在 `camera_init` 系）。RTAB-Map external-odom
  期望 `odom → base_link`。顶层 `manual_mapping_lio_right.launch.py` 在
  `enable_loop_backend:=true` 时发布一个 **identity `odom → camera_init`** 静态 TF,
  使链路 `odom → camera_init → body → base_link` 完整。默认 off 时不发布,无 TF 冲突。
- 角色边界:**FAST-LIO2 = 前端实时位姿;RTAB-Map = 可选后端回环/导出**,无双真值源。



---

## 7. 当前部署与硬件诊断入口（2026-09-07）

当前工作台支持左右 XT-M60 的只读诊断，但两路都必须在任务结束后停止；右路曾有较稳定的
几何回波，左路的有效点比例/链路仍需一变量一项健康检查。任何临时 TF 都不能用于宣称
FAST-LIO2、累计地图或导航通过。真实硬件启动、电机写入和运动测试必须遵循交接文档中的
显式授权与停机门禁。

### 7.1 当前临时 TF 与外参状态

| 传感器 | 当前临时平移（m） | 当前临时 RPY（rad） | 状态 |
|---|---|---|---|
| RIGHT 192.168.1.101 | [0.45, -0.30, 0.735] | [1.7071165220, 0.0265268546, 1.5744345976] | 地面拟合得到的诊断候选；yaw、绝对 x、时间偏移未标定 |
| LEFT 192.168.0.101 | [0.45, 0.30, 0.735] | [1.5515153364, -0.0136154207, 1.5709313394] | 历史可靠方向候选；当前获取质量和最终外参仍待确认 |

两光学中心的用户实测相对横向间距约 0.60 m，相对 fore/aft 偏移为 0；上表的共同
绝对 x 是未验证先验，不能用来吸收时间或姿态误差。当前 body→base_link 和
base_link→xtm60_*_link 只服务于显示/诊断，正式标定前保持 provisional。

### 7.2 RIGHT 雷达只读启动（需显式授权）

```bash
# 只读/不运动入口；硬件任务结束后必须按交接要求干净停机：
bash scripts/run_rviz_manual_mapping_left.sh                 # 默认只读诊断
RADAR=right MOTION=false bash scripts/run_rviz_manual_mapping_left.sh

# 或直接 launch:
ros2 launch wheelchair_bringup manual_mapping_lio_right.launch.py motion_control_enabled:=false
```

文件:
- 配置 src/wheelchair_3d_mapping/config/xtm60_right_lio.yaml（仅含 provisional extrinsic_T/R，不能视为最终外参）。
- `fast_lio_mapping.launch.py` 支持 `radar:=right|left`,按雷达选 config + 输入话题 + `base_link→xtm60_<radar>_link` 静态 TF。
- 顶层 `manual_mapping_lio_right.launch.py`(EKF off,FAST-LIO 拥有位姿)。
- 历史离线右雷达冒烟曾产出 /Odometry、/cloud_registered、/path；这不是当前实机或最终外参验收。

### 7.3 LEFT 雷达(出盒后恢复)

左雷达路径保留:`xtm60_left_lio.yaml`、`manual_mapping_lio_left.launch.py`、`fast_lio_mapping.launch.py radar:=left`、
`RADAR=left bash scripts/run_rviz_manual_mapping_left.sh`。左雷达恢复后仍须先做只读链路/质量检查；不能仅凭“有话题”进入最终标定或运动测试。

---

## 8. FAST-LIO 嵌套仓库的持久化合同(必读)

`src/third_party/FAST_LIO_ROS2` 是独立 Git 仓库,并被主仓库 `.gitignore` 忽略。只提交主仓库不会
带走其中的 `laserMapping.cpp` / `IMU_Processing.hpp` 修复。当前合同把上游基线固定为
`2fffc570a25d0df172720bac034fbdb6a13d2162`,并在主仓库保存一份**完整、唯一**的可重放补丁:

```bash
bash patches/apply_fastlio_patches.sh
colcon build --packages-select fast_lio
```

- 权威补丁:`patches/fastlio_smartwheel_hardening.patch`。
- 脚本只接受上述精确基线和干净的嵌套工作树；先执行 `git apply --check`，应用后验证关键标记，
  并核对权威补丁、laserMapping.cpp、IMU_Processing.hpp 的 SHA-256。任何检查失败均 fail closed，
  不再用 `sed` 强改源码；脚本已明确拒绝 LASER_POINT_COV=100.0。
- 历史 `fastlio_laser_point_cov.patch` 与 `fastlio_zupt.patch` 已改为不可应用的退役墓碑,避免旧工具
  无声恢复 `R=100` 或负 age ZUPT。
- 发布时需要两个清晰的提交关系:先在 FAST-LIO 嵌套仓库提交/标记源码修复,再在主仓库提交权威
  patch、脚本、配置、驱动和文档。主仓库提交说明必须记录嵌套提交 SHA;不要把两棵树误当成一个提交。

---

## 8.1 ZUPT:时间对齐、反馈健康门与正规伪测量(2026-09-04)

**症状**:短暂 WASD 驾驶(`MOTION=true`,真实电机转动)之后,按 Space/松键停止操作,rviz 里
`/path`(青蓝色 "LIO Path")与 `/Odometry` 仍在不规则漂移变化,而轮椅本体已经物理静止。

旧实现存在两个缺陷:它选取“最新”轮速而不是 LiDAR 时刻之前的样本,未来样本会产生负 age 并
绕过陈旧检查;同时直接执行 `s.vel=0` 和速度协方差硬裁剪,不是统计一致的测量更新。

当前实现只在以下条件全部成立时执行:

1. 从历史队列选取时间戳 `<= LiDAR` 的轮速样本,并检查 age、连续零速保持时间。
2. 同一驱动周期的 Modbus 左/右反馈读取都成功,且 `/base/wheel_feedback_healthy` 未超时。
   `zlac8030_driver_node` 在已配置反馈寄存器却读取失败时发布 `healthy=false`,并停止发布该周期的
   `/wheel/odom`;缺失信息不再伪装成零速,因此只能让 ZUPT 因输入陈旧而暂停。
3. LiDAR 时刻之前的 IMU 窗口同时满足角速度和加速度模长静止阈值。

满足后使用观测 `v=0`、可配置 `zupt.velocity_measurement_variance` 计算 Kalman 增益,通过
`boxplus` 更新状态并用 Joseph 形式更新协方差。姿态、bias 和重力不会被硬写零,但可通过与速度的
交叉协方差得到统计修正。

**局限**:轮速健康不是独立地面真值,外力推动仍主要靠 IMU 静止门排除;阈值和交叉状态修正仍需
真实 STOP 恢复测试。缺少健康话题或任一输入陈旧时实现会 fail closed,即暂停 ZUPT。

配置开关位于 `src/wheelchair_3d_mapping/config/xtm60_right_lio.yaml` 的 `zupt:` 块。

## 8.2 已落地的软件加固与仍需实机闭环的边界

| 项目 | 已落地的软件行为 | 仍需实机完成 |
|---|---|---|
| 时间戳 | XT-M60（SDK 设备时间关闭）使用 SDK 回调主机接收时间代理；H30 使用串口读取边界之间的 200 Hz 主机侧插值；点云→scan→merge/fusion 保留最新非零源戳 | 设备采样时钟/硬件同步能力、固定延迟和动态对齐标定 |
| IMU 初始化 | 至少 400 样本、2 秒连续静止,并检查 gyro/accel 均值与标准差;检测到运动会重启窗口 | 上电静置和启动重复性验收 |
| 点云质量 | 单雷达基础配置保留 temporal hard reject;驾驶入口改为只报告相邻像素变化而不因运动直接丢帧;输入另有限距 | 动态区分合法场景变化、多径和坏帧 |
| 输出 | `/Odometry` 在填好本帧 covariance 后发布;`/path` 限长并降采样;大点云发布使用小队列 | 压力测试确认最长输出间隔及恢复跳变 |
| 单/双雷达 | 单雷达 phase realign 默认 0;仅双雷达工作台显式覆盖 30 秒 | 双雷达错峰收益及停测影响验证 |
| TF | `body→base_link` 使用当前已知 `base_link→imu_link` 的完整逆变换,不再只有 z 平移 | LiDAR/IMU/车体最终外参标定 |

分阶段实机步骤、通过阈值和只读监控命令见
[`FASTLIO_HARDWARE_VALIDATION_PLAN.md`](FASTLIO_HARDWARE_VALIDATION_PLAN.md)。软件验收完成不等于动态、时间同步或外参已经标定。

上述主机时间线（包括 H30 的插值）只是比延迟发布时刻更接近采样的代理，**不等于真实设备采样时间**。在供应商未提供
可用设备时间戳或同步接口前,文档和 UI 不得把它宣传为硬件同步。

---

## 9. Orin 主机供电 brownout(运行重命令必读)

见 `.kiro/steering/orin-host-ops.md`。要点:重负载命令(满核 colcon、fastlio+rviz+bag 全栈)会瞬时拉高电流
导致 PMIC 硬复位(不是软件/温度/内存)。**防护**:用 `taskset -c 0-3` / `--parallel-workers 2` 限核;
采集与离线处理不并行;大点云 RANSAC 前子采样到 ≤4 万点。
