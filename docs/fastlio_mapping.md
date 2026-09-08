# FAST-LIO2 单(左)雷达 LiDAR-惯性建图

本文档记录 SmartWheel 将 3D 建图从「纯 EKF 位姿 + RTAB-Map 零配准」重构为
**FAST-LIO2(LiDAR-惯性里程计)** 的获取、依赖、编译、外参、启动与保存等可复现信息。

对应 spec:`.kiro/specs/fastlio-narrow-fov-mapping/`(需求 1.1 / 1.3 / 1.5)。

> 状态:本文件随实现任务逐步补全。当前已完成 **任务 1(获取并 vendored FAST-LIO2,锁定 commit)**、
> **任务 2(安装依赖并在 aarch64/Humble 编译 FAST-LIO2,`BUILD_EXIT=0`)**
> 与 **任务 4(录制离线 bag,作为后续所有离线工作的唯一回放源)**。
> 外参核对(任务 5/7)、启动与保存(任务 11/13)等章节将在后续任务补充。

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

## 5. 离线回放源 — canonical bag(任务 4 已落地,req 5.5 / 6.2)

任务 4 已一次性录制离线 bag(左 XT-M60 + IMU + TF,含温和运动),作为**任务 5–9、13 的唯一离线回放源**,
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

### 6.1 LiDAR↔IMU 外参(任务 5 已落地,req 2.2/2.4/3.1)

FAST-LIO 的 `extrinsic_T/R` = **LiDAR 在 IMU 体系下的位姿**:`T_il = inv(base→imu) · (base→lidar)`。
任务 5 从 canonical bag 的 `/tf_static`(`base→imu_link`=[0,0,0.45])+ `static_transforms.yaml` 文档值
(`base→xtm60_left_link`=[0.45,0.24,0.65] rpy[0,-0.0873,0])解析得:

```
extrinsic_T: [0.45, 0.24, 0.20]            # LiDAR 在 IMU 上方 0.20m、前 0.45m、左 0.24m
extrinsic_R: [0.996192, 0, -0.087189,      # 绕 Y 的 -5°(0.0873rad)俯仰
              0, 1, 0,
              0.087189, 0, 0.996192]
```

写入 `src/wheelchair_3d_mapping/config/xtm60_left_lio.yaml`(lidar_type=2 标准 PointCloud2 路径、
deskew off、`extrinsic_est_en=false`、fov 120、blind 0.1)。

> **两个遗留点(交 Task 6/7)**:
> 1. canonical bag 缺 `base_link→xtm60_left_link` 边(原 calibrator 被移除)——新 LIO 链路须补发该静态 TF。
> 2. 俯仰用的是文档值 -5°,与早期左雷达实测 ~10° 不一致;**Task 7 用真实地面核对(地面 z≈0/墙竖直)并按需修正**。

### 6.2 关键调参:LASER_POINT_COV(任务 7 根因修复)

**症状**:FAST-LIO 跑得动但**忽略 IMU 旋转**——真实/IMU 实测偏航 ~98°,输出只有 ~4°,地图成漩涡/重影。

**根因**(逐层插桩定位,见 `auto_test/20260622_lio_rootcause_fix/report.md`):
`LASER_POINT_COV` 在 `laserMapping.cpp` 硬编码 `0.001`(每个 LiDAR 点测量协方差极小=极度信任 LiDAR)。
窄视场稀疏 flash ToF 的几何约束很弱却被极度信任,iEKF 每帧用扫描匹配把 IMU 预测的旋转抹掉,
还把真实转速塞进陀螺零偏(`bg_z` 涨到 ~0.098 rad/s ≈ 平均转速)。

**修复**(单一杠杆,逐步验证 0.001→1.0→100):

| 文件 | 改动 | 原因 |
|---|---|---|
| `src/third_party/FAST_LIO_ROS2/src/laserMapping.cpp` | `#define LASER_POINT_COV` `0.001` → **`100.0`** | 降低对弱 LiDAR 约束的信任,让 IMU 主导姿态 |
| `config/xtm60_left_lio.yaml` | `b_gyr_cov` 1e-4→**1e-7**、`gyr_cov` 0.1→**0.5**、`b_acc_cov`→1e-5、`acc_cov`→0.5 | 锁死陀螺零偏,防止真实转速被当零偏吸收 |

**效果**:偏航 0.001→4°,1.0→33°,**100→89°**(真实 ~98°)。这实现了需求要的「IMU 主导 + LiDAR 辅助」。

> 注:`LASER_POINT_COV` 是 `#define`(非 ROS 参数),只能改 vendored 源码;已加注释标注原值与原因。
> 后续可提为 ROS 参数以免改第三方源码。

**已知待精修(非阻断)**:① 地面 z 仍在 ~1m 不在 0(外参俯仰/高度按真实地面校正;Task 5 文档 -5° vs 左雷达实测 ~10°);
② 89° vs 98° 余 ~9°(可再调信任度/精修外参)。这些是外参精修,不是 IMU 问题。

### 6.3 启动、地图保存、RTAB-Map 后端开关

启动见 §7.2（右雷达,当前默认入口）。完整会话地图保存必须在建图开始前启动
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

## 7. 当前部署:RIGHT 雷达(左雷达已入盒遮挡)

> **硬件现状变更(20260623)**:LEFT 雷达被放入盒子、视场被遮挡,标定不可用(地面点 <~150,无法拟合)。
> **当前部署改为 RIGHT 雷达单雷达**。详见 `.kiro/steering/orin-host-ops.md`。

### 7.1 两个雷达的实测外参

| 雷达 | 话题 | 高度 | pitch | roll | 平面残差 | 数据来源 |
|---|---|---|---|---|---|---|
| RIGHT(右,192.168.1.101) | `/xtm60/right/points` | **51.0 cm** | **−1.04°** | **−9.1°** | 4.3 mm | `auto_test/20260623_dual_radar_calib`(地面点充足,可靠) |
| LEFT(左,192.168.0.101) | `/xtm60/left/points` | 54.5 cm | +1.1° | +0.78° | 8 mm | `auto_test/20260622_ground_calib_bag`(入盒前;现遮挡不可用) |

> 注:早期对左/右各报过几组互相矛盾的数字(同一窗口/约定 bug),已废弃。上表为最终采用值。

### 7.2 RIGHT 雷达启动(当前默认)

```bash
# 一键(默认 right;脚本据 RADAR 选择 launch):
bash scripts/run_rviz_manual_mapping_left.sh                 # RADAR 默认 right
RADAR=right MOTION=true bash scripts/run_rviz_manual_mapping_left.sh   # 含运动(需空旷+急停)

# 或直接 launch:
ros2 launch wheelchair_bringup manual_mapping_lio_right.launch.py motion_control_enabled:=false
```

文件:
- 配置 `src/wheelchair_3d_mapping/config/xtm60_right_lio.yaml`(extrinsic_T/R = 右雷达外参)。
- `fast_lio_mapping.launch.py` 支持 `radar:=right|left`,按雷达选 config + 输入话题 + `base_link→xtm60_<radar>_link` 静态 TF。
- 顶层 `manual_mapping_lio_right.launch.py`(EKF off,FAST-LIO 拥有位姿)。
- 离线右雷达冒烟已验证:`fast_lio` 在右雷达 bag 上产出 `/Odometry` `/cloud_registered` `/path`。

### 7.3 LEFT 雷达(出盒后恢复)

左雷达路径保留:`xtm60_left_lio.yaml`、`manual_mapping_lio_left.launch.py`、`fast_lio_mapping.launch.py radar:=left`、
`RADAR=left bash scripts/run_rviz_manual_mapping_left.sh`。左雷达出盒、视野见地面后可直接用。

---

## 8. ⚠️ 关键脆弱点:LASER_POINT_COV 在 vendored 源码里(必读)

Task 7 的根因修复 `LASER_POINT_COV` `0.001 → 100.0` 改在 **vendored 第三方源码**
`src/third_party/FAST_LIO_ROS2/src/laserMapping.cpp`(它是 `#define`,不是 ROS 参数)。
**一旦 re-clone 或 `git submodule update`,这个改动会被静默还原**,FAST-LIO 会重新忽略 IMU、
偏航不再跟踪、地图回到漩涡。

**防护**:已把改动存为 patch 并提供重放脚本:
```bash
bash patches/apply_fastlio_patches.sh        # 幂等:已是 100.0 则跳过;否则打 patch
colcon build --packages-select fast_lio
```
- patch:`patches/fastlio_laser_point_cov.patch`
- 若 re-clone 后建图又变漩涡/偏航不动,**第一件事就是跑这个脚本**。

---

## 8.1 ZUPT(零速度更新)缓解:WASD 驾驶后位姿持续漂移(20260903)

**症状**:短暂 WASD 驾驶(`MOTION=true`,真实电机转动)之后,按 Space/松键停止操作,rviz 里
`/path`(青蓝色 "LIO Path")与 `/Odometry` 仍在不规则漂移变化,而轮椅本体已经物理静止。

**根因**(诊断会话,未做真机复测前的静态代码排查,详见该次对话记录):
右雷达路线没有 robot_localization EKF、没有轮速融合,位姿完全来自 FAST-LIO2 自身的 IMU-LiDAR
紧耦合 iEKF。WASD 引入的真实动态加速度会扰动 iEKF 的速度/加速度计 bias 状态;
`xtm60_right_lio.yaml` 里 Task-7 遗留的注释记录过真实测过的 accel-bias 漂移(静止场景
0.3 mm/s → 234 mm/s,4.7 h),当前"修复"(`b_acc_cov` 恢复上游默认值)只在长时间静止场景验证过,
从未验证"运动后状态如何收敛"。同时 `laserMapping.cpp` 的 `LASER_POINT_COV=100` 是 Task 7
故意调大的(让 IMU 主导姿态、削弱 LiDAR 校正力),一旦速度状态被运动扰动,LiDAR 更新本身就弱到
拉不回来。IKFoM 库里唯一像样的跳变限幅函数 `check_safe_update()`
(`include/IKFoM_toolkit/esekfom/esekfom.hpp`)从未被任何调用点使用,是死代码——整条链路没有
任何机制在停止后主动把被扰动的状态拉回。

**缓解**(`patches/fastlio_zupt.patch`,已生效,默认关闭,只在右雷达路线开启):
用 `/wheel/odom`(`zlac8030_driver_node` 无条件发布的真实轮速反馈,配置了反馈寄存器时是编码器
实测值,否则是指令值的开环估计)作为"轮子自认为没转"的信号。一旦该信号连续
`zupt.hold_time_sec`(默认 0.3 s)保持在阈值以下,`laserMapping.cpp` 每帧把 iEKF 的速度状态
`s.vel` 清零,并把速度分量的协方差裁到 `zupt.velocity_cov_reset`(默认 1e-4)以内,防止残余
速度被二次积分成位置漂移。`/wheel/odom` 超过 `zupt.stale_timeout_sec`(默认 1.0 s)没更新则
判定失效,ZUPT 暂停(什么都不做),不会把"数据陈旧"误判成"静止"。

**局限,诚实说明**:
- `/wheel/odom` 是"驱动认为轮子没转",不是独立的地面真值——如果外部把静止的轮椅推动、或
  `invert_left`/`invert_right` 符号配错导致驱动误判零速,ZUPT 不会发现。
- 这只处理"速度状态"这一个自由度,不处理 accel-bias 本身的收敛速度,也不处理窄视场沿墙退化
  这个更根本的几何约束缺失问题(`.kiro/specs/fastlio-narrow-fov-mapping/design.md` §风险与已知限制)。
- **尚未用真实驾驶复测确认效果**。已验证的范围:(1) `colcon build --packages-select fast_lio`
  编译通过;(2) 独立启动 `fastlio_mapping`(无雷达/IMU 数据)确认新增的 `zupt.*` 参数被正确解析、
  `/wheel/odom` 订阅被创建;(3) 用 `ros2 topic pub` 向 `/wheel/odom` 灌入近零速度消息持续数秒,
  节点保持响应、无崩溃无死锁;(4) `patches/fastlio_zupt.patch` 在干净的上游 `2fffc57` checkout
  上应用 `apply_fastlio_patches.sh` 后,产物与当前树逐字节一致,幂等重跑验证过。**没有**做过真实
  WASD 驾驶 + 停止后观察漂移是否收敛的端到端复测——下次真机测试时请重点验证这一点。

配置开关:`src/wheelchair_3d_mapping/config/xtm60_right_lio.yaml` 的 `zupt:` 块。
左雷达 (`xtm60_left_lio.yaml`)、双雷达、mock 均未开启,行为不变。

---

## 9. Orin 主机供电 brownout(运行重命令必读)

见 `.kiro/steering/orin-host-ops.md`。要点:重负载命令(满核 colcon、fastlio+rviz+bag 全栈)会瞬时拉高电流
导致 PMIC 硬复位(不是软件/温度/内存)。**防护**:用 `taskset -c 0-3` / `--parallel-workers 2` 限核;
采集与离线处理不并行;大点云 RANSAC 前子采样到 ≤4 万点。
