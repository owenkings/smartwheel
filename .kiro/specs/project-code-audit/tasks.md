# Implementation Plan

## Overview

**审查/文档工程,非运行时代码改动。** 以下所有任务都是"打开文件 → 按 12 维度检视 → 把发现记入
`docs/project_code_audit.md`"的审查动作,**不修改任何运行时代码**(`src/**`、`scripts/**`、
`calib/**`、`auto_test/**` 的源文件内容保持不变)。真正的代码修复(remediation)是后续独立 effort,
由本报告的"编号 + 严重度 + 修复方向"驱动。

本计划用 Bug Condition 方法论组织:
- **Property 1: Bug Condition** —— 逐文件穷尽、不封顶、全 6 字段记录(任务 1、3–15)。这些"探查"
  任务在旧状态(F = 仅 `docs/mapping_code_review_2026-06-17.md`)下"失败"——即范围内文件没有任何
  检视痕迹/条目;走完后报告对每个命中文件都有条目或"已检视无缺陷"痕迹。
- **Property 2: Preservation** —— 排除集不动、真无缺陷如实、已记录结论保留、已修项标已修(任务 16–17)。

## Tasks

- [x] 1. 初始化审查产物骨架 + 生成 in-scope 文件枚举(覆盖清单种子)
  - **Property 1: Bug Condition** - 覆盖账本与报告骨架就位
  - **IMPORTANT**: 这是后续所有 review batch 的前置;先建骨架与文件全集,才能逐文件回填检视结果
  - 创建 `docs/project_code_audit.md`,写入设计 §3 文档骨架:
    - `## 0. 元信息`(审查日期、范围 = bugfix.md「审查范围」、显式排除集、审查方法、严重度定义 阻断/高/中/低)
    - `## 1. 缺陷表`,表头固定 6 列:`编号 | 文件位置(相对路径+函数/类+行号区间) | 问题描述 | 影响(建图项单列建图质量影响) | 严重度 | 建议修复方向`
    - `## 2. 逐文件覆盖清单`,表头:`相对路径 | 类型 | 状态(已检视/未检视) | 缺陷数 | 备注(无缺陷/迁移自F/已修)`
    - `## 3. 跨文件一致性缺陷`(占位,B14 回填)
    - `## 4. 已知问题与修复状态`(占位,B15 回填)
    - `## 5. 备注与方法学说明`
  - 运行设计 §2 的确定性枚举命令生成 in-scope 文件全集(分类型多次执行避免 shell 转义):
    - `find src -type f -name '*.py'`
    - `find src -type f \( -name '*.cpp' -o -name '*.c' -o -name '*.h' -o -name '*.hpp' \)`
    - `find src -type f \( -name '*.yaml' -o -name '*.xacro' \)`
    - `find src -type f \( -name 'CMakeLists.txt' -o -name 'package.xml' \)`
    - `find src -type f -name '*.rviz'`(仅影响建图/可视化判断时纳入)
    - `find scripts -type f -name '*.sh'`
    - `find calib -type f -name '*.yaml'`
    - `find auto_test -maxdepth 1 -type f -name '*.py'`
  - 用枚举结果为覆盖清单播种:每个文件一行,初始状态「未检视」,缺陷数 0
  - 显式排除 `build/`、`__pycache__/`、`.git/`、`.pytest_cache/`、二进制图片、根目录临时 dotfiles(`.b.log`/`.test.py`/`.jog.py`/`.mid360_probe.*` 等),不进枚举(除非发现被正式链路引用)
  - **不修改任何运行时代码** —— 仅创建文档并运行只读 `find`
  - _Requirements: 2.1, 2.3, 3.1_

- [x] 2. (探查基线)确认旧审查 F 存在缺口 —— Bug Condition 复现
  - **Property 1: Bug Condition** - 旧状态下命中文件无检视痕迹
  - **IMPORTANT**: 在产出完整新报告前先 surface 反例,验证根因(范围不全 / 人为封顶 / 粒度不够)成立
  - **GOAL**: 证明 `isBugCondition(X)` 在大量范围内文件上为真 —— 它们在 `docs/mapping_code_review_2026-06-17.md` 中零条目、零检视痕迹
  - **Scoped 方法**: 用任务 1 的枚举对照 F 内容,列出「在范围内、F 未检视」的文件清单
  - 抽样打开 `wheelchair_navigation` / `wheelchair_safety` / `wheelchair_diagnostics` / `wheelchair_perception` / `wheelchair_sensors` 各若干文件,确认确有缺陷,且 F 无对应条目
  - **EXPECTED OUTCOME**: 这些文件在 F 中既无缺陷条目也无「已检视无缺陷」痕迹(确认缺口存在)
  - 记录反例(例:`wheelchair_safety/velocity_limiter_node.py` 限速逻辑未被任何条目检视)以指导后续批次
  - **不修改任何运行时代码**
  - _Requirements: 1.1, 1.2, 1.3, 1.5_

- [x] 3. B1 审查 `wheelchair_base`(底盘:运动学/驱动/Modbus)
  - **Property 1: Bug Condition** - 逐文件穷尽 + 6 字段记录
  - 按「库 → 节点 → 配置 → launch → 测试」顺序打开 **每个**文件:kinematics.py, modbus_rtu.py, zlac8030_driver_node.py, __init__.py, setup.py, test/test_kinematics.py, test/test_zlac_shutdown.py
  - 每个文件跑满 12 维度检视清单(算法/功能 bug/不稳健/健壮性/规范/架构/参数一致性/时间戳/TF/QoS/单位符号/测试覆盖)
  - 每发现一个缺陷,向 §1 缺陷表追加一行,**全局连续编号、6 字段齐全、不封顶**,severity ∈ {阻断,高,中,低}
  - 每检视完一个文件,在 §2 覆盖清单把该行置「已检视」并回填缺陷数 / 「无缺陷」备注
  - 参数一致性(量程/频率/外参)命中时,标记并汇入 B14 第 3 节
  - **不修改任何运行时代码** —— 仅检视与记录
  - _Requirements: 2.1, 2.2, 2.3, 2.4_

- [x] 4. B2 审查 `wheelchair_sensors`(传感器适配)
  - **Property 1: Bug Condition** - 逐文件穷尽 + 6 字段记录
  - 逐个打开:camera_adapter_node.py, imu_adapter_node.py, mock_sensor_node.py, ultrasonic_adapter_node.py, xtm60_adapter_node.py, setup.py, test/{test_h30_imu_adapter,test_ultrasonic_adapter,test_xtm60_adapter}.py
  - 每文件跑满 12 维度;重点关注 `use_sdk_timestamps` 时间基准、量程/FOV 参数(供 B14 比对 xtm60 range/height)
  - 发现缺陷追加 §1 缺陷表(全局编号、6 字段、不封顶);每文件回填 §2 覆盖清单状态
  - **不修改任何运行时代码**
  - _Requirements: 2.1, 2.2, 2.3, 2.4_

- [x] 5. B3 审查 `wheelchair_perception`(感知)
  - **Property 1: Bug Condition** - 逐文件穷尽 + 6 字段记录
  - 逐个打开:dynamic_obstacle_layer_node.py, obstacle_detector_node.py, passability_analyzer_node.py, pointcloud_to_laserscan_node.py, scan_merger_node.py 及 3 个 test
  - 每文件跑满 12 维度;关注 QoS 匹配、scan/cloud frame 与时间同步
  - 发现缺陷追加 §1 缺陷表(全局编号、6 字段、不封顶);每文件回填 §2 覆盖清单状态
  - **不修改任何运行时代码**
  - _Requirements: 2.1, 2.2, 2.3, 2.4_

- [x] 6. B4 审查 `wheelchair_3d_mapping`(建图核心:融合/栅格/KISS-ICP/RTAB-Map 配置)
  - **Property 1: Bug Condition** - 逐文件穷尽 + 6 字段记录(建图项须单列建图质量影响)
  - 逐个打开:cloud_to_occupancy_grid_node.py, cloud_utils.py, dual_lidar_cloud_fusion_node.py, kiss_icp_mapping_node.py, rgb_cloud_colorizer_node.py, wheel_livo_consistency_monitor.py, 7×launch, 6×config(含 rtabmap_params.yaml), 3×rviz(限影响建图/可视化), 2 tests
  - 每文件跑满 12 维度;重点:融合时间戳(墙钟 vs 采集)、Grid/RangeMax vs 融合 max_range、approx_sync、Force3DoF、voxel
  - **建图链路相关缺陷的「影响」列必须单独说明对建图质量的影响**
  - 发现缺陷追加 §1 缺陷表(全局编号、6 字段、不封顶);每文件回填 §2 覆盖清单状态
  - 参数/时间基准跨文件命中标记并汇入 B14
  - **不修改任何运行时代码**
  - _Requirements: 2.1, 2.2, 2.3, 2.4, 2.5_

- [x] 7. B5 审查 `wheelchair_mapping`(地图后处理脚本 + launch + CMake)
  - **Property 1: Bug Condition** - 逐文件穷尽 + 6 字段记录(建图项须单列建图质量影响)
  - 逐个打开:scripts/{map_postprocess,map_quality_check,vectorize_occupancy_map}.py, launch/{online_mapping,save_map}.launch.py, CMakeLists.txt
  - 每文件跑满 12 维度;关注地图后处理算法正确性与建图质量影响
  - 发现缺陷追加 §1 缺陷表(全局编号、6 字段、不封顶);每文件回填 §2 覆盖清单状态
  - **不修改任何运行时代码**
  - _Requirements: 2.1, 2.2, 2.3, 2.4_

- [x] 8. B6 审查 `wheelchair_navigation`(导航:探索/目标/语义/定位)
  - **Property 1: Bug Condition** - 逐文件穷尽 + 6 字段记录
  - 逐个打开 11 个节点/库(frontier_explorer, goal_manager, named_goal_store, navigation_status, reactive_explorer, semantic_keepout(_node), semantic_map_store, startup_localization(_node)), config/{named_goals,semantic_map}.yaml, 8 tests
  - 每文件跑满 12 维度;关注状态机/返回值、TF lookup 时间、健壮性(空值/边界)
  - 发现缺陷追加 §1 缺陷表(全局编号、6 字段、不封顶);每文件回填 §2 覆盖清单状态
  - **不修改任何运行时代码**
  - _Requirements: 2.1, 2.2, 2.3, 2.4_

- [x] 9. B7 审查 `wheelchair_safety`(安全:急停/监督/限速)
  - **Property 1: Bug Condition** - 逐文件穷尽 + 6 字段记录
  - 逐个打开:emergency_stop_node.py, safety_supervisor_node.py, velocity_limiter_node.py, test/test_safety_supervisor.py
  - 每文件跑满 12 维度;关注安全裕度、异常吞没、超时、failsafe 逻辑
  - 发现缺陷追加 §1 缺陷表(全局编号、6 字段、不封顶);每文件回填 §2 覆盖清单状态
  - **不修改任何运行时代码**
  - _Requirements: 2.1, 2.2, 2.3, 2.4_

- [x] 10. B8 审查 `wheelchair_diagnostics`(诊断:探针/自检/健康/看门狗)
  - **Property 1: Bug Condition** - 逐文件穷尽 + 6 字段记录
  - 逐个打开:hardware_probe.py, hardware_self_check_node.py, localization_health_node.py, policy.py, sensor_watchdog_node.py, test/test_policy.py
  - 每文件跑满 12 维度;关注健壮性、资源泄漏、看门狗超时与误报
  - 发现缺陷追加 §1 缺陷表(全局编号、6 字段、不封顶);每文件回填 §2 覆盖清单状态
  - **不修改任何运行时代码**
  - _Requirements: 2.1, 2.2, 2.3, 2.4_

- [x] 11. B9 审查 `wheelchair_bringup`(C/C++ 节点 + 大量 launch/config)
  - **Property 1: Bug Condition** - 逐文件穷尽 + 6 字段记录
  - 逐个打开:src/teleop_panel.cpp, src/xt_bindshim.c, include/wheelchair_bringup/teleop_panel.hpp, CMakeLists.txt, 16×launch, 24×config(EKF/nav2/safety/scan/sensor/xtm60/camera/ultrasonic/zlac…), 3×rviz
  - 每文件跑满 12 维度;C/C++ 关注资源泄漏/生命周期,config 关注频率/QoS/量程(供 B14 比对 EKF frequency 30 vs 轮速 50 vs IMU 200)
  - 发现缺陷追加 §1 缺陷表(全局编号、6 字段、不封顶);每文件回填 §2 覆盖清单状态
  - **不修改任何运行时代码**
  - _Requirements: 2.1, 2.2, 2.3, 2.4, 2.5_

- [x] 12. B10 审查 `wheelchair_description`(URDF/xacro + static tf + display)
  - **Property 1: Bug Condition** - 逐文件穷尽 + 6 字段记录(建图项须单列建图质量影响)
  - 逐个打开:urdf/wheelchair.urdf.xacro, config/static_transforms.yaml, launch/display.launch.py, CMakeLists.txt
  - 每文件跑满 12 维度;重点:雷达/IMU 安装位姿、frame 命名、单位/符号(供 B14 比对 URDF↔static_transforms↔calib 外参)
  - 发现缺陷追加 §1 缺陷表(全局编号、6 字段、不封顶);每文件回填 §2 覆盖清单状态
  - **不修改任何运行时代码**
  - _Requirements: 2.1, 2.2, 2.3, 2.4, 2.5_

- [x] 13. B11 审查 `scripts/`(8 个 shell 脚本)
  - **Property 1: Bug Condition** - 逐文件穷尽 + 6 字段记录
  - 逐个打开 8×.sh(check/run rviz, hardware_shutdown, save_mapping_result, setup_radar_network, stop_mapping 等)
  - 每文件跑满适用维度;关注健壮性(未引用变量/错误处理)、命令注入、网络/外参环境设置一致性
  - 发现缺陷追加 §1 缺陷表(全局编号、6 字段、不封顶);每文件回填 §2 覆盖清单状态
  - **不修改任何运行时代码**
  - _Requirements: 2.1, 2.2, 2.3, 2.4_

- [x] 14. B12 审查 `calib/` + B13 审查 `auto_test/`
  - **Property 1: Bug Condition** - 逐文件穷尽 + 6 字段记录
  - B12 逐个打开:calib/{left,left_unrotated,right,right_unrotated}.yaml,检视外参/内参一致性(供 B14 比对左右雷达外参,关注已知 #50 右雷达占位外参)
  - B13 逐个打开:auto_test/odom_calib.py, auto_test/yaw_compare.py(测试脚本,非报告 md),跑满 12 维度
  - 发现缺陷追加 §1 缺陷表(全局编号、6 字段、不封顶);每文件回填 §2 覆盖清单状态
  - **不修改任何运行时代码**
  - _Requirements: 2.1, 2.2, 2.3, 2.4, 2.5_

- [x] 15. B14 跨文件一致性专项(汇入报告 §3)
  - **Property 1: Bug Condition** - 跨文件不一致作为独立条目
  - 横向拉齐「同一物理量在多文件中的取值」,不一致作为独立条目记入 §3 并**列出所有相关文件位置**:
    - **量程 / voxel**:`xtm60_*` 适配器 range/height ↔ `dual_lidar_fusion.yaml` max_range/voxel ↔ `rtabmap_params.yaml` Grid/RangeMax ↔ `cloud_to_occupancy_grid.yaml`(核对已知 #30:Grid/RangeMax=8 vs 融合 12)
    - **坐标系 / 外参**:`wheelchair.urdf.xacro` ↔ `static_transforms.yaml` ↔ `calib/*.yaml` ↔ 融合节点 `_lookup` frame(核对已知 #50)
    - **频率 / QoS**:轮速 publish_rate ↔ IMU 频率 ↔ `robot_localization_ekf.yaml` frequency ↔ 各 pub/sub QoS(核对已知 #37)
    - **时间基准**:适配器 `use_sdk_timestamps` ↔ 融合 `header.stamp` ↔ EKF/RTAB-Map approx_sync(核对已知 #13/#15/#41:墙钟 vs 采集时刻)
  - 每条跨文件缺陷使用统一 6 字段格式,全局编号、不封顶,建图相关项单列建图质量影响
  - **不修改任何运行时代码**
  - _Requirements: 2.4, 2.5_

- [x] 16. B15 起点文档迁移 + 已知问题修复状态(汇入报告 §4)
  - **Property 2: Preservation** - 起点结论保留 + 已修项标已修
  - 把 `docs/mapping_code_review_2026-06-17.md` 的 50 条结论迁移/重编号进 §1 缺陷表或 §4,**保留技术实质**,按本报告严重度定义(阻断/高/中/低)重判(⛔→阻断或高;⚠️→中;🔸→低)
  - §4 逐条体现已知真实问题并标注修复状态:
    - **已修**:EKF 轮速 yaw 主导(→ IMU 主导)、雷达 URDF 高度、RTAB-Map yaml 从不加载 —— 标「已修」,**不**作为待修缺陷重复报告
    - **未修**:雷达盒子遮挡假点、融合时间戳用墙钟、纯激光窄 FOV 无回环(架构性)、IMU 仅姿态级无室内位移观测(硬件约束)
  - 迁移自 F 的文件在 §2 覆盖清单备注标「迁移自F」
  - **不修改任何运行时代码**
  - _Requirements: 2.6, 3.2, 3.4_

- [x] 17. 最终验证 —— Fix Checking + Preservation Checking
  - **Property 2: Preservation** - 全量属性断言
  - **IMPORTANT**: 对报告本身做属性式校验,不新写运行时测试;复用任务 1 的文件枚举作为真值
  - **Fix Checking**(对应 Property 1):
    - 断言 §2 覆盖清单条目集合 == 任务 1 文件枚举集合(差集为空,**无未检视的范围内文件遗留**)
    - 断言 §1 缺陷表每行 6 字段非空、编号全局连续无重复、severity ∈ {阻断,高,中,低}、location 指向**真实存在**的文件路径
    - 断言报告中**无任何「至少/至多 N 条」人为封顶措辞**;条目数仅由发现决定
    - 断言建图相关条目的「影响」列单独包含对建图质量的说明
  - **Preservation Checking**(对应 Property 2):
    - 断言显式排除集(`build/`、`__pycache__/`、`.git/`、`.pytest_cache/`、二进制图片、根目录临时 dotfiles)**不出现**在缺陷表/覆盖清单(除非被正式链路引用)
    - 断言 F 的 50 条技术结论均能在 §1/§4 找到对应(可重编号),无丢失
    - 抽样断言简单文件(`__init__.py`、纯数据 yaml)在覆盖清单标「无缺陷」而非被强行制造条目
    - 断言已修项(EKF 轮速 yaw / 雷达 URDF 高度 / RTAB-Map yaml 加载)在 §4 标「已修」,未出现在待修缺陷里
    - 断言无无文件依据的猜测性条目
  - **EXPECTED OUTCOME**: 上述断言全部通过;若任一失败,回到对应批次补检/补字段,不修改运行时代码
  - **不修改任何运行时代码**
  - _Requirements: 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 3.1, 3.2, 3.3, 3.4, 3.5_

- [x] 18. Checkpoint —— 确认报告闭环且所有验证通过
  - 确认 `find 枚举 → 覆盖清单 → 缺陷表` 三者闭环一致,逐文件穷尽得证
  - 确认报告为不封顶、6 字段、严重度合法的单一活文档 `docs/project_code_audit.md`
  - 如有疑问询问用户。审查阶段不改运行时代码;remediation 为后续独立 effort

## Task Dependency Graph

```json
{
  "waves": [
    { "wave": 1, "tasks": ["1"], "dependsOn": [] },
    { "wave": 2, "tasks": ["2"], "dependsOn": ["1"] },
    { "wave": 3, "tasks": ["3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14"], "dependsOn": ["2"] },
    { "wave": 4, "tasks": ["15"], "dependsOn": ["3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14"] },
    { "wave": 5, "tasks": ["16"], "dependsOn": ["15"] },
    { "wave": 6, "tasks": ["17"], "dependsOn": ["16"] },
    { "wave": 7, "tasks": ["18"], "dependsOn": ["17"] }
  ]
}
```

- 任务 1 是所有批次的前置(骨架 + 文件枚举)。
- 任务 2 确认 Bug Condition 复现后,批次 3–14(B1–B13)彼此独立,可并行/任意顺序检视。
- 任务 15(B14 跨文件)依赖 3–14 完成,因其横向比对各包内已标记的共享物理量。
- 任务 16(B15 迁移)在缺陷表编号体系稳定后进行。
- 任务 17(验证)在所有记录完成后执行;任务 18 收尾。

## Notes

- 全部任务均为**审查/文档动作**,产物仅为 `docs/project_code_audit.md`;**不修改任何运行时代码**。
  remediation(实际修复)是后续独立 effort,由本报告的「编号 + 严重度 + 修复方向」驱动。
- PBT 风格映射:任务 1、3–15 校验 **Property 1: Bug Condition**(逐文件穷尽、不封顶、6 字段);
  任务 16–17 校验 **Property 2: Preservation**(排除集不动、真无缺陷如实、已记录结论保留、已修标已修)。
- 缺陷条目格式固定 6 字段:编号 / 文件位置 / 问题描述 / 影响 / 严重度 / 建议修复方向;
  严重度 ∈ {阻断, 高, 中, 低};建图相关项的「影响」须单列对建图质量的影响。
- 覆盖清单(§2)必须与任务 1 的文件枚举一一对应,使「已检视无缺陷」与「未检视」可区分。
- 不设任何「至少/至多 N 条」人为封顶;条目数仅由实际发现决定。
- 显式排除集(`build/`、`__pycache__/`、`.git/`、`.pytest_cache/`、二进制图片、根目录临时 dotfiles)
  不进枚举、不制造条目,除非发现被正式链路引用。
