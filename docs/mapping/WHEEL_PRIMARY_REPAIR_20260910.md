# 轮速 / IMU 主位姿修复与 FAST-LIO 隔离验证

日期：2026-09-10。代码主树：Orin `/home/nvidia/smartwheel`，分支 `local/formal-3d-hardening-20260908`，基准 HEAD `103b569c0156c90bfcb6647e5da15fc12035b993`。

## 结论与完成边界

用户已明确选择“轮速＋IMU 为主”。本轮据此调整诊断入口：电机实测前向速度和 H30 角速度生成连续车辆位姿；RTAB-Map 接收该位姿和原始右雷达点云，生成地图并执行空间 ICP 回环候选检测；FAST-LIO 单独运行并接受一致性监测，不拥有车辆 TF。

**这不是宣称 FAST-LIO 独立定位已经准确，也不是宣称完整 FAST-LIO 前端＋RTAB 后端已经完成。** 当前已记录的 FAST-LIO 仍存在失真，因此它暂不参与主位姿或主地图生成。将来要恢复其主地图贡献，必须先通过同一原始数据的定位精度、连续性及物理标定验证；不会因为某个短窗口一致性通过就自动切换。

本入口是室内平地的平面运动模型，但保存的点云仍为三维 XYZ＋intensity；没有把所有地面点强行改成 z=0。平面模型不估计实际坡道高度、车身俯仰/横滚，不得当作坡道、导航或载人安全验收。

## 1. 为什么以前的接线和计划不一致

| 路径 | 连续车辆位姿来源 | RTAB-Map | 不能据此推出的结论 |
|---|---|---|---|
| 原 `run_right_diag_mapping.sh` | FAST-LIO 完整三维位姿；轮速/IMU EKF 只是对照 | 没有启动 | 不能说该入口已有 RTAB 回环 |
| 正式 3D launch 的默认设计 | FAST-LIO 前端，RTAB 后端 | 有配置，但受标定门禁约束 | 不能把正式 launch 配置当成实际运行路径 |
| 旧树 `smartwheel-rtab-old` | 平面 wheel/IMU EKF 外部里程计 | 使用外部里程计建图 | z/roll/pitch 被约束为 0 不等于水平轨迹绝对准确 |
| 本轮新入口 `run_wheel_imu_mapping.sh` | wheel vx＋IMU yaw rate → EKF → 健康检查 | 原始右 XYZ / intensity＋外部里程计，空间 ICP 回环 | FAST-LIO 仍是隔离对照，不是已批准的融合辅助源 |

FAST-LIO 本身是紧耦合激光惯性里程计，会估计完整运动状态，不是只负责将点云画出来的模块。见 [FAST-LIO 官方仓库](https://github.com/hku-mars/FAST_LIO)。不能一边让 FAST-LIO 发布全三维车辆 TF，一边期待轮速位移和 IMU 旋转天然成为 RViz 的主来源。

旧树实际数据库 `maps/run_20260910_204847_wmuy/rtabmap.db` 中，审查时保留图有 11 个节点、20 条双向邻接记录，未发现 type=2 空间回环；保留节点 z/roll/pitch 全为 0。故这份旧运行“不上下乱飞”的直接解释是平面状态约束，而不是已有成功回环不断纠偏。旧树此次没有修改。

## 2. 有证据的代码缺陷与修复

### 2.1 停车 ZUPT 绕过最终位姿保护

原 `apply_zupt_if_stationary()` 在 LiDAR 5 cm 位姿修正检查之后执行全状态 `boxplus`，然后直接发布。原 bag 的 header 相对起点 81.525 / 87.515 / 101.101 秒，预测到最终输出的位置净修正分别为 0.4007 / 0.6313 / 0.5529 m；即使扣除 LiDAR 允许的 0.05 m，仍至少有 0.3507 / 0.5813 / 0.5029 m 来自其后的停车更新。

修复：停车更新变为有创新、NIS 和速度修正上限的 velocity-only Schmidt/Joseph 更新；不直接改位置、姿态、bias 或重力。发布前再做完整有限性、协方差半正定和累计位姿修正检查。限幅未因本轮失败而继续放宽。

### 2.2 拒绝激光帧后，状态与 IMU 时间游标不同步

原 `rollback_imu_timeline()` 实际不回退已前进的 IMU 游标，`restore_last_valid_lidar_state()` 却恢复上一已接受帧的旧状态。于是被拒绝帧对应时间段的运动积分丢失，下一帧从旧状态却用新时间继续积分；在转弯和重新配准时尤其危险。

修复：保留同一扫描末端的 IMU 预测状态及协方差，拒绝的是坏的 LiDAR 校正/地图插入/输出，而不是把状态时间拉回过去。LiDAR 后辅助更新失效时也只回退到同一时刻的有效状态。

### 2.3 Odometry 速度与协方差发布不完整

原 `/Odometry.twist` 一直为零，因为发布函数没有填写速度；这与内部速度是否为零无关。原位姿协方差还交换了状态中 position 与 rotation 的块。

修复：填写 child-frame 线速度、去 bias 的角速度及相应协方差；position 从状态第 0 块、rotation 从第 3 块读取，并转换姿态协方差轴约定。`mat_pre` 的欧拉列改为本帧预测，而非上一帧的后验姿态。新增逐帧 `state_update_audit.tsv`，可直接区分预测、LiDAR 和辅助更新贡献。

生产文件：`src/third_party/FAST_LIO_ROS2/src/laserMapping.cpp`、`include/state_aiding_safety.hpp`。嵌套仓库被主仓库忽略，因此同时更新 `patches/fastlio_smartwheel_hardening.patch`、重建脚本和哈希校验，避免修复只留在某台机器的临时嵌套树中。

## 3. 为什么不能把所有漂移归因于 ZUPT 或电机

已有仅改变 wheel / ZUPT 开关的同一 85 秒数据回放：均开启时约第 70 秒 z=+0.426 m，均关闭时 z=+0.476 m，两组都已漂移。因此纯右雷达＋H30 的当前 LIO 也有问题；关闭辅助不能称作修复完成。

原始 gyro 积分和 IMU quaternion 相对 yaw 相符；wheel 与 gyro 角速度同号比例 99.22%，相关系数约 0.991。但约 79.1 秒时，原始 IMU 倾角约 -0.156 / +0.044°，LIO 约 -8.486 / -2.818°。这支持 LIO 姿态、bias / 重力估计出现失真，而不是仅 RViz 画错。

1648 帧原始 / LIO 输入的 header 及筛选后 XYZI 一致，1531 帧原 TF 与 Odometry 一致。该录制中没有原始长空窗、时间倒退、SDK reset 或竞争 TF 的证据。不能继续把本轮问题笼统归因于此前已改的 30 秒雷达 stop/start，也不能无证据假设固定 100 ms 偏移。

还存在未定量隔离的候选因素：host-receive 时间与真实曝光时间、实际 LiDAR–IMU 外参、窄视场几何可观测性、噪声/偏置权重和配准质量。当前矩阵代数一致不等于物理标定准确。本轮没有猜测设备时钟、重新写雷达曝光、篡改安装高度或继续放宽配准阈值。

详细原始证据见同目录 `FASTLIO_VERTICAL_JUMP_FORENSICS_20260910.md`。

## 4. 新主链的契约

```text
实测 wheel vx ─┐
               ├─ 平面 EKF ─ 健康门 ─ odom → base_link（唯一发布者）
H30 yaw rate ──┘                    │
                                    ├─ 蓝色连续轨迹 /wheel_mapping/path
原始右雷达 XYZI ────────────────────┼─ 实时三维点云 /wheel_mapping/cloud_registered
                                    └─ RTAB-Map ─ map → odom（全局校正）
                                                  ├─ 三维 / 二维地图
                                                  └─ 保留 intensity 的优化地图
同一右雷达＋H30 ─ FAST-LIO shadow ─ 相对运动一致性报告（无 TF 权限）
```

- EKF 输入是实测 wheel 速度，不是 `/cmd_vel`，也不重复融合已由轮速积分的 x/y；位移由速度随时间积分得到，仍受轮径、滑移和编码器比例影响。
- 旋转使用经过安装 TF 旋转后的 IMU 角速度，不直接采用室内磁航向；不把未经动态验证的加速度积分当平移。当前 wheel angular.z 用作对照，未作为第二个重复航向观测。
- `wheel_pose_health_gate` 检查 wheel、IMU、反馈健康和 EKF 新鲜度；缺输入时停止发布主位姿与 TF，避免把纯预测伪装成有效定位。它不是电机制动器，不能替代急停或运动安全监督。
- RTAB 接收 sensor-frame 原始点云和健康门后的外部里程计，不接收已在世界系变换过的 LIO 点云再次套位姿。邻接 ICP 重配准关闭，空间 ICP 回环开启；没有 RGB 特征词袋回环，此处的回环是激光几何候选。
- 相对一致性监测先将 IMU 原点的 LIO 位姿换算到 base，再比较共同时间窗口的相对运动，不比较两个世界原点的绝对位置。无数据、陈旧、错误和分歧时均不能将 shadow 升为主位姿。H30 为两路共享输入，因此此对照不是独立真值。
- RViz Fixed Frame 为 `odom`：车的连续运动不随全局回环跳变，RTAB 的全局地图经 `map→odom` 展示。实时点云独立于低频累积地图，遮挡或移动无需等待新地图关键帧。
- 累积三维显示使用 `/rtabmap/optimized_cloud`，来自原始三维扫描和优化图位姿，保留 intensity。实测 RTAB 原生 `/rtabmap/cloud_map` 为 XYZRGB，因此不把它误称为幅值产品。
- 实时点云增加 10 帧 / 0.25 秒有界 TF 重试队列，每 20 ms 非阻塞检查；TF 晚到可重试，过期/健康失效丢弃，零时间戳被拒绝，绝不改用 latest TF。该措施防止调度先后差异造成不必要的丢帧，并不改变真实采样时间。
- 每次运行创建独立数据库目录；退出只清理该入口自己的进程，先请求电机驱动停机，再等待雷达 SDK 正常停测。没有自动杀掉旧树或其他终端。

## 5. 运行与验证边界

请先退出仍在运行的旧建图终端，再在 **NoMachine 中的 Orin 图形桌面终端**执行：

```bash
cd /home/nvidia/smartwheel
source /opt/ros/humble/setup.bash
source install/setup.bash
CAMERAS=true ULTRASONIC=true MOTION=false \
  bash scripts/run_wheel_imu_mapping.sh
```

`MOTION=false` 明确禁止电机运动，WASD 不驱动是预期行为。需要自行做电动测试时，先确认空旷区域、无乘员并有可用物理急停，再将这一项改为 `MOTION=true`。不要继续使用旧 `run_right_diag_mapping.sh` 作为本轮主入口。

Ctrl+C 正常退出，等待停机信息。数据库和 ROS 日志保留在命令打印的 `maps/wheel_imu_时间_随机后缀/`。本轮没有启动真实设备、运行 Windows 上位机或发送电机命令；离线回放不包含控制速度话题。

进程退出只能证明 ROS 会话结束，不能证明停止寄存器写入成功或轮子物理静止；电动测试后须实际确认停车，异常时立即使用物理急停。

## 6. 完整数据回放：必须保留的失败结果

原始输入为 `bags/diag/right_vertical_jump_20260910_210202`，172.100 秒，1648 帧右雷达、34052 帧 IMU、6522 条轮速。两次独立 FAST 回放均保持轮速辅助和 ZUPT 开启，使用对应修复前 / 后二进制及同一实际 layout。

| 指标 | 修复前 FAST | 修复后 FAST |
|---|---:|---:|
| Odometry 帧数 | 1599 | 1015 |
| 最后有效输出相对 bag 起点 | 171.829 s | 111.843 s |
| 尾部停止更新 | 0 s | 59.987 s |
| z 全程范围宽度 | 0.849 m | 4.217 m |
| 连续输出判定 | 失败：最大间隔 2.468 s | 失败：尾部约 60 s |

**修复后 FAST 独立轨迹没有改善，整体回归结果更差，不能部署为主定位。** 正确处理拒绝帧的状态时间，并不能修复错误的惯性预测、匹配和状态估计；此前恢复旧状态的做法在一定程度上压住了数值变化，却丢失真实时间段的运动。不能为得到好看的小数值而恢复这种旧逻辑。

修复后逐帧审计有 1626 行：1015 接受、611 拒绝；40 次 ZUPT、456 次轮速更新。所有已接受帧的 LiDAR→辅助位置和姿态增量严格为 0；预测→最终最大修正为 0.0490885 m / 1.995511°，在 0.05 m / 2°保护内，证明已堵住已知辅助更新绕过限幅的路径。它只证明更新保护成立，不证明估计正确。后期有效特征跌到 10、再到 0，而原始扫描仍有约 824–1076 点，说明当前失配/跟踪崩溃仍未解决。

证据目录：

- `/home/nvidia/smartwheel/auto_test/vertical_jump_fix_full_before_20260910/`
- `/home/nvidia/smartwheel/auto_test/vertical_jump_fix_full_after_20260910/`
- 修复后 `generated_fast_logs/state_update_audit.tsv` 与 `result.json`。

回放前备份并按哈希恢复 FAST 固定调试日志；原始 bag 不修改。最初若干联合回放因诊断工具与 Humble 的消息 API 差异提前退出，保留了失败日志；它们不是生产修复的通过证据。当前主链的完整回放结果与这些失败尝试分开记录。

## 7. 最终联合回放与交付结果

最终证据：`/home/nvidia/smartwheel/auto_test/wheel_primary_full_replay_20260910_delivery/`，主要文件为 `result.json`、`tf_audit.log`、`primary_odom_samples.json`、`launch_graph.log`、`wheel_primary_rtabmap.db`。

| 检查 | 实际结果 |
|---|---|
| 原始输入完整观测 | 1648 点云 / 34052 IMU / 6522 wheel / 6962 health，均与 bag 数量相同 |
| 主位姿 | 3921 帧，持续到 +172.061 s，最大间隔 0.300 s，无非有限值/时间倒退 |
| TF 对应关系 | 3921/3921 帧时间戳与位姿逐值完全一致，无重复时间戳 |
| 实际 TF 发布者 | C++ 读取消息 publisher GID：odom→base_link 仅 wheel_pose_health_gate，map→odom 仅 rtabmap；无 camera_init→body |
| 实时三维点云 | 1303 帧，各帧内容不同，末帧与最后原始雷达帧时间完全一致，最大间隔 0.530 s |
| 原始→实时 intensity | 1303/1303 逐点幅值字节一致，0 不匹配 |
| 累积三维幅值地图 | 60 次发布，末次 221842 点，字段 x/y/z/intensity，最后时间距末雷达帧约 0.331 s |
| 二维投影 | 80 次更新，421×421 @ 0.05 m/cell；覆盖最后实测运动时刻 |
| RTAB 数据库 | 124 个 Node 行，优化轨迹保留 80 节点，158 条双向 type=0 邻接记录，0 条空间回环 |
| 主链结果 | `primary_mapping_smoke_pass=true` |
| 包含 FAST shadow 连续性的全链结果 | `offline_integration_smoke_pass=false`，FAST 后段仍失去跟踪；未隐瞒失败 |

原始 wheel 数据直到 +41.127 s 才出现，所以此前没有主位姿/实时配准点云是健康门的预期等待，不是用 IMU 假造平移。最后有效轮速运动约 +140.990 s，二维图最后更新约 +141.639 s；之后车辆已停，二维关键帧地图不继续变化属于正常行为。实时点云和主位姿仍更新至录制结束，不把两类产品混为一谈。

同一公共时间窗中，主航向相对 H30 积分只差约 -0.01343°，说明旋转确实沿预期 IMU 链路进入主位姿。轮速独立角速度积分约 231.748°，H30 约 222.282°，相差约 9.466°；相关系数 0.9913 也不意味着两者精度相同。因此不能先验认定电机里程计几何、角度尺度和无滑移均正确。H30 为共享输入，上述比较不是独立定位真值。

主链 z/roll/pitch 全零由 `two_d_mode=true` 强制，不能算“精确量出车身倾角”；保留了全部三维点云，但尚不支持实际坡道六自由度轨迹。本轮没有地面行驶真值，不能保证最终坐标闭合或地图绝对精度。也没有启动 RViz/摄像头/超声/真实电机进行全硬件负载验收。

软件验证：3 包构建成功（FAST-LIO 仅有上游 Boost/GCC 提示）；最终 mapping 测试 **182 passed**；C++ 安全更新回归通过；canonical patch 从固定上游重建一致，apply 脚本哈希验证通过。TF 审计器独立 CMake `-Werror` 构建通过。

FAST-LIO 最终源码 SHA256：`ae9df41f67cb62ed6d9dcd27601e3a2aa06eddb0a663b0fbde94c3bee78d3b08`；canonical patch：`57c8352ba839d9a7f9421985f3f0538f15d272c43eb6540a944eb6c3c1b17a9b`；已安装二进制：`d09f8f6c4bb4dd605214d3c2d952f158577e03e79c55deec3a30135354d98f81`。

说明：Humble 中 `publish_tf=false` 的节点仍可能创建 /tf 发布端点，因此仅数端点会错误判断冲突。本轮用 C++ `rclcpp::MessageInfo` 实际逐消息 GID 验证；Python 数值比较补充验证变换内容。最终结果中的旧 `expected_endpoint_graph_pass=false` 是那个“只有两个端点”的错误假设留下的诊断字段，不表示存在额外 TF 写入；`cpp_single_writer_contract_pass=true` 和逐帧对应结果才是实证。后续脚本已移除这个误导字段，历史结果文件原样保留。

主树未暂存、未提交、未推送；用户原有修改保留。旧树仍为 `local/rtab-old-loopfix-20260910@e0ab5957c327`，未修改。原文件及旧二进制备份位于 `auto_test/repair_delivery_before_20260910_JrAbTl`，带 COLCON_IGNORE；不要删除原始证据来伪造干净状态。

## 8. 后续 FAST-LIO 工作入口，不是本轮已完成项

下一阶段应针对 **纯 LIO 的失配起点**，用固定原始记录逐项分离 IMU 权重/bias 演化、flash 点云模型、真实采样时间和物理外参；每项都要求完整动态回放而非仅首帧或静态输出。独立 LIO 在整段定位、倾角与路线精度达标前，不重新取得 TF 权限，不参与主地图插入。本轮没有找到足以解释全部 LIO 失真的唯一物理根因，不能笼统归咎于 FAST-LIO2 官方算法不可用。

可复现工具留在 `scripts/mapping/replay_wheel_aiding_offline.py`、`replay_wheel_primary_offline.py`，以及 `scripts/mapping/tf_audit/`。它们只回放白名单传感器数据，使用独立 ROS domain；TF 审计器需先由 CMake 显式构建，入口默认使用本次已构建的 `auto_test/tf_audit_build/cmake-build/smartwheel_tf_audit`。这些开发诊断工具不是操作者启动命令；操作者仅使用第 5 节的新完整入口。
