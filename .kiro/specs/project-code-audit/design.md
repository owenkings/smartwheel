# Project Code Audit Bugfix Design

## Overview

本设计针对的"缺陷"不是某段运行时代码,而是**现有审查工作的不完整性**。当前唯一审查产物
`docs/mapping_code_review_2026-06-17.md` 只覆盖建图链路、人为封顶 50 条、且未做逐文件穷尽。
本次"修复"(F')是用一套**系统化、逐文件、不封顶**的审查方法走完 SmartWheel 项目
(`/home/nvidia/smartwheel`)全部一手源文件,并产出一份结构化、可追踪的缺陷报告。

本设计是一个**审查/文档工程**,而非运行时代码改动。这里的"实现"= 按既定顺序逐个打开源文件、
按 12 个维度逐函数检视、把发现的每个缺陷按统一 6 字段格式记录成独立条目、并为每个文件留下
"已检视 / 缺陷数 / 无缺陷"的覆盖痕迹。最终输出一份**单一活文档** `docs/project_code_audit.md`。

审查方法的核心是"缺陷条件 C(X)"逐文件作用:对范围内每个源文件 X,只要存在偏离正确/健壮/规范
预期的实例、且未被现有审查以合规条目记录,即命中。整个项目被切成**按 ROS 包划分的审查批次**,
使后续 tasks 阶段可以把设计拆成"每包一个 review batch"的并行任务。

## Glossary

- **Bug_Condition (C)**: 审查缺口条件 —— 对某源文件 X,`inScope(X) AND hasDefect(X) AND NOT fullyRecordedByExistingReview(X)`。命中即说明该文件存在"未被合规记录的真实缺陷"。
- **Property (P)**: 修复后期望 —— 每个命中文件都被实际检视(`examined(X)=true`),其每个缺陷都产出含全部 6 字段的独立条目,且不因任何人为上限被截断(`noArtificialCap`)。
- **Preservation**: 对未命中输入(排除集文件 / 真无缺陷文件 / 已记录的真实结论与已修状态)保持不变 —— 不强行检视排除集、不捏造缺陷、保留起点文档的技术结论、已修项标记为已修。
- **F**: 原审查 = `docs/mapping_code_review_2026-06-17.md`(mapping-only,封顶 50 条,非逐文件)。
- **F'**: 新审查 = 本设计产出的 `docs/project_code_audit.md`(逐文件、穷尽、不封顶)。
- **In-scope source file**: bugfix.md "审查范围"小节列出的一手源文件(`*.py / *.cpp / *.c / *.h / *.hpp / *.launch.py / *.yaml / *.xacro / *.sh / CMakeLists.txt / package.xml / *.rviz(限影响建图/可视化时) / calib/*.yaml`)。
- **Review batch**: 一个 ROS 包(或一组同类目录,如 `scripts/`、`calib/`、`auto_test/`)构成的检视单元,对应 tasks 阶段一个任务。
- **Defect record**: 含 6 字段的独立缺陷条目(编号 / 文件位置 / 问题描述 / 影响 / 严重度 / 修复方向)。
- **Coverage checklist**: 文件清单,逐文件标注"已检视 / 缺陷数 / 无缺陷",是 Preservation 的可证明痕迹。
- **Severity taxonomy**: 阻断 / 高 / 中 / 低。

## Bug Details

### Bug Condition

缺陷条件作用在**每个范围内源文件**上。当某文件中存在代码/配置行为偏离正确、健壮、规范预期的
实例,而该实例未被现有审查(F)以"含全部 6 字段的合规条目"记录时,即命中缺陷条件。这包括三类
结构性缺口:范围不全(非建图文件被整体遗漏)、人为封顶(50 条之后停止记录)、粒度不够
(文件未被实际打开/未留检视痕迹)。

**Formal Specification:**
```
FUNCTION isBugCondition(X)
  INPUT: X 为项目源文件 (in-scope source file)
  OUTPUT: boolean

  RETURN inScope(X)
         AND hasDefect(X)
         AND NOT fullyRecordedByExistingReview(X)
END FUNCTION
```

其中:
- `inScope(X)` = X 属于 bugfix.md"审查范围"枚举的一手源文件类型,且不在显式排除集。
- `hasDefect(X)` = X 中存在至少一个偏离正确/健壮/规范预期的实例(覆盖 12 个审查维度之一)。
- `fullyRecordedByExistingReview(X)` = F 中已存在对应条目且该条目含全部 6 字段且文件已被逐函数检视。

### Examples

- `src/wheelchair_navigation/wheelchair_navigation/goal_manager_node.py` 含潜在缺陷,但 F 完全未覆盖导航包 → **命中**(范围不全)。
- `src/wheelchair_safety/wheelchair_safety/velocity_limiter_node.py` 安全限速逻辑未被任何条目检视,无"已检视无缺陷"痕迹 → **命中**(粒度不够)。
- 假设真实缺陷总数 > 50,则第 51 条起的真实缺陷在 F 中被人为上限截断 → **命中**(人为封顶)。
- `src/wheelchair_3d_mapping/wheelchair_3d_mapping/dual_lidar_cloud_fusion_node.py` 的时间戳问题已被 F 第 13 条以较完整描述记录 —— 若新审查重新编号并补齐 6 字段后视为已记录,该实例本身不再是"未记录缺陷",但所在文件仍需在 F' 中逐函数复检以确认无其他遗漏。
- `build/`、`__pycache__/`、根目录 `.test.py` 等临时 dotfile **不在范围**,`inScope` 返回 false → **不命中**(属于 Preservation 集)。

## Expected Behavior

### Preservation Requirements

**Unchanged Behaviors:**
- 显式排除集文件(`build/`、`__pycache__/`、`.git/`、二进制图片、`.pytest_cache/`、根目录临时 dotfiles 如 `.b.log`/`.test.py`/`.jog.py`/`.mid360_probe.*` 等)在未被正式链路引用时,继续排除在逐文件审查之外,不为其制造缺陷条目。
- 起点文档 `docs/mapping_code_review_2026-06-17.md` 中已记录的真实缺陷,其实质技术结论被保留(可重新编号/重组),不丢失。
- 经检视确实无缺陷的文件,如实标注为"无缺陷",不为凑数捏造问题。
- 已确认"已修复"的问题(EKF 轮速 yaw 主导 → 已改 IMU 主导;雷达 URDF 高度;RTAB-Map yaml 从不加载)被标注为"已修",不作为待修缺陷重复报告。
- 仅记录基于实际代码内容、可由文件位置佐证的缺陷,不引入无依据的猜测性条目。

**Scope:**
所有不满足 `isBugCondition` 的输入应完全不受本次审查的"制造缺陷"影响。这包括:
- 显式排除集文件(临时文件、构建产物、缓存、二进制资源)。
- 经逐函数检视确认无缺陷的源文件(它们只产生覆盖痕迹,不产生缺陷条目)。
- F 中已含完整技术结论的发现(被迁移/重编号,而非丢弃或捏造)。

**Note:** 命中文件的期望正确行为(逐文件检视 + 6 字段记录 + 不封顶)定义在下方 Correctness Properties(Property 1)。本节聚焦"什么必须不变"。

## Hypothesized Root Cause

现有审查不完整的根因分析:

1. **范围锚定偏差(Scope anchoring)**:F 的标题与目标被定义为"建图链路代码审查",导致检视者只
   遍历里程计/融合/RTAB-Map/EKF 相关文件,从结构上排除了导航(`wheelchair_navigation`)、感知
   (`wheelchair_perception`)、安全(`wheelchair_safety`)、诊断(`wheelchair_diagnostics`)、
   传感器适配(`wheelchair_sensors`)、UI/C++(`wheelchair_bringup/src`)、描述(URDF)等包。

2. **人为数量目标(Artificial cap)**:以"50 条"作为收尾信号,使审查在达到目标数量后停止,而非
   在"所有文件检视完毕"时停止。数量驱动而非覆盖驱动。

3. **缺乏文件清单机制(No coverage ledger)**:F 没有逐文件清单,无法区分"检视后无缺陷"与"根本
   没看过",因此无法证明逐文件覆盖,粒度天然不足。

4. **无跨文件一致性专项(No cross-file pass)**:F 按单链路、按文件分组记录,没有把"同一物理量在多
   个文件中的取值"横向拉齐检查(量程、坐标系、频率、外参、时间基准),架构级不一致被遗漏。

5. **缺乏统一条目模板(No record schema)**:F 的条目格式为自由文本(问题/位置/影响/建议),缺少
   全局唯一编号与强制严重度字段,导致条目不可稳定追踪。

## Correctness Properties

Property 1: Bug Condition - 逐文件穷尽、不封顶、全字段记录

_For any_ 命中缺陷条件的源文件 X(`isBugCondition(X)` 为真),新审查 F' SHALL 确保该文件被实际
逐函数检视(`examined(X)=true`),且该文件中每个被发现的缺陷在报告中都对应一条含全部 6 个字段
(唯一编号、文件位置、问题描述、影响、严重度、建议修复方向)的独立条目,条目总数由实际发现决定、
不设任何人为上限或下限;建图链路相关条目须单独说明对建图质量的影响。

**Validates: Requirements 2.1, 2.2, 2.4, 2.6**

Property 2: Preservation - 排除集不动、真无缺陷如实、已记录结论保留

_For any_ 不命中缺陷条件的输入 X(`isBugCondition(X)` 为假),新审查 F' SHALL 保留 F 中的真实技术
结论并维持原行为:显式排除集文件不被强行检视、不为其制造条目;经检视确实无缺陷的文件如实标注
"无缺陷"而非捏造问题;已确认"已修"的问题标注为已修而非重复报告为待修;不引入无文件依据的猜测性
条目。

**Validates: Requirements 2.3, 2.5, 3.1, 3.2, 3.3, 3.4, 3.5**

## Fix Implementation

### Changes Required

本次"实现"是产出与维护一份审查报告文档,而非改运行时代码。产物如下。

**File**: `docs/project_code_audit.md`(单一活文档,审查全程增量写入)

**Function**(逻辑环节,而非代码函数):逐文件检视 → 记录缺陷 → 维护覆盖清单 → 跨文件比对。

#### 1. 审查方法 / 遍历顺序(Audit Methodology)

按 ROS 包为单位、自底向上(底盘→传感器→感知→建图→导航→安全→诊断→集成)遍历,确保读后续包时
已理解其依赖的底层模块。遍历顺序与批次划分:

| 批次 | 范围 | 主要文件(节点/库/配置/测试) |
|------|------|------------------------------|
| B1 | `wheelchair_base` | kinematics.py, modbus_rtu.py, zlac8030_driver_node.py, __init__.py, setup.py, test/test_kinematics.py, test/test_zlac_shutdown.py |
| B2 | `wheelchair_sensors` | camera_adapter_node.py, imu_adapter_node.py, mock_sensor_node.py, ultrasonic_adapter_node.py, xtm60_adapter_node.py, setup.py, test/{test_h30_imu_adapter,test_ultrasonic_adapter,test_xtm60_adapter}.py |
| B3 | `wheelchair_perception` | dynamic_obstacle_layer_node.py, obstacle_detector_node.py, passability_analyzer_node.py, pointcloud_to_laserscan_node.py, scan_merger_node.py, +3 tests |
| B4 | `wheelchair_3d_mapping` | cloud_to_occupancy_grid_node.py, cloud_utils.py, dual_lidar_cloud_fusion_node.py, kiss_icp_mapping_node.py, rgb_cloud_colorizer_node.py, wheel_livo_consistency_monitor.py, 7×launch, 6×config(含 rtabmap_params.yaml), 3×rviz, 2 tests |
| B5 | `wheelchair_mapping` | scripts/{map_postprocess,map_quality_check,vectorize_occupancy_map}.py, launch/{online_mapping,save_map}.launch.py, CMakeLists.txt |
| B6 | `wheelchair_navigation` | 11×节点/库(frontier_explorer, goal_manager, named_goal_store, navigation_status, reactive_explorer, semantic_keepout(_node), semantic_map_store, startup_localization(_node)), config/{named_goals,semantic_map}.yaml, 8 tests |
| B7 | `wheelchair_safety` | emergency_stop_node.py, safety_supervisor_node.py, velocity_limiter_node.py, test/test_safety_supervisor.py |
| B8 | `wheelchair_diagnostics` | hardware_probe.py, hardware_self_check_node.py, localization_health_node.py, policy.py, sensor_watchdog_node.py, test/test_policy.py |
| B9 | `wheelchair_bringup` | src/teleop_panel.cpp, src/xt_bindshim.c, include/wheelchair_bringup/teleop_panel.hpp, CMakeLists.txt, 16×launch, 24×config(EKF/nav2/safety/scan/sensor/xtm60/camera/ultrasonic/zlac…), 3×rviz |
| B10 | `wheelchair_description` | urdf/wheelchair.urdf.xacro, config/static_transforms.yaml, launch/display.launch.py, CMakeLists.txt |
| B11 | `scripts/` | 8×.sh(check/run rviz, hardware_shutdown, save_mapping_result, setup_radar_network, stop_mapping) |
| B12 | `calib/` | left.yaml, left_unrotated.yaml, right.yaml, right_unrotated.yaml(外参/内参一致性) |
| B13 | `auto_test/` | odom_calib.py, yaw_compare.py(测试脚本,非报告 md) |
| B14 | 跨文件一致性专项 | 横向比对(见下方 Testing Strategy / Cross-file checks),不属于单一文件 |
| B15 | 已知背景核对 + 起点文档迁移 | 把 F 的 50 条结论迁移/重编号,标注修复状态 |

每个批次内,文件按"库 → 节点 → 配置 → launch → 测试"顺序检视。每检视完一个文件,立即在覆盖清单
登记一行,并把该文件发现的缺陷追加到缺陷表。

#### 2. 文件清单机制(File Inventory / Coverage Ledger)

用确定性枚举命令生成范围内文件全集,作为覆盖清单的初始行,逐文件回填检视结果:

```
# in-scope 枚举(分类型多次执行,避免 shell 转义问题)
find src -type f -name '*.py'                # 节点/库/测试
find src -type f \( -name '*.cpp' -o -name '*.c' -o -name '*.h' -o -name '*.hpp' \)
find src -type f \( -name '*.yaml' -o -name '*.xacro' \)
find src -type f \( -name 'CMakeLists.txt' -o -name 'package.xml' \)
find src -type f -name '*.rviz'              # 仅影响建图/可视化判断时纳入
find scripts -type f -name '*.sh'
find calib  -type f -name '*.yaml'
find auto_test -maxdepth 1 -type f -name '*.py'
```

覆盖清单表头:`相对路径 | 类型 | 状态(已检视/未检视) | 缺陷数 | 备注(无缺陷/迁移自F/已修)`。
清单必须覆盖枚举出的**每一个**范围内文件,使"已检视无缺陷"与"未检视"可区分,从而证明逐文件覆盖。

#### 3. 缺陷报告结构(`docs/project_code_audit.md` 文档骨架)

```
# SmartWheel 全项目代码审查报告
## 0. 元信息(审查日期、范围、排除集、方法、严重度定义)
## 1. 缺陷表(Defect Table)        # 全局连续编号、不封顶、6 字段/条
## 2. 逐文件覆盖清单(Coverage Ledger)
## 3. 跨文件一致性缺陷(Cross-file Inconsistencies)   # 对应 Req 2.5
## 4. 已知问题与修复状态(Known Issues & Fix Status)   # 对应 Req 2.6 / 3.2 / 3.4
## 5. 备注与方法学说明
```

缺陷表每行即一条 Defect record,字段顺序固定:

| 编号 | 文件位置(相对路径 + 函数/类 + 行号区间) | 问题描述 | 影响(建图项单列建图质量影响) | 严重度 | 建议修复方向 |

#### 4. 严重度分类(Severity Taxonomy)

- **阻断**:导致功能不可用、崩溃、数据损坏,或使建图/导航/安全完全失效。
- **高**:显著损害正确性/建图质量/安全裕度,常态触发但系统勉强运行。
- **中**:边界条件/隐患/健壮性缺陷,特定条件下触发。
- **低**:规范/可维护性问题(命名、重复、死代码、注释缺失),不影响运行时正确性。

(与起点文档 ⛔/⚠️/🔸 的映射:⛔→阻断或高;⚠️→中;🔸→低,迁移时按上述定义重判。)

#### 5. 每文件 12 维度检视清单(Per-Dimension Checklist)

每个文件至少跑一遍以下 12 维度,任何一维命中即生成缺陷条目:

1. 算法错误(公式/积分/坐标变换是否正确)
2. 功能性 bug(逻辑分支、状态机、返回值)
3. 不严谨/不稳健的方法(近似阶数不一致、量化偏置)
4. 健壮性(异常吞没、空值/边界、资源泄漏、超时)
5. 规范(命名/重复/死代码/魔法数)
6. 架构(职责混叠、生命周期管理、双重过滤)
7. 参数一致性(跨文件量程/坐标/频率/外参冲突 —— 标记后汇入第 3 节)
8. 时间戳与同步(墙钟 vs 采集时刻、approx_sync、queue)
9. TF / 坐标系正确性(lookup 时间、frame 命名、Force3DoF)
10. QoS 匹配(reliable/best_effort、depth、durability 在 pub/sub 两侧是否一致)
11. 单位/符号(rad vs deg、左右/正负、米 vs 毫米)
12. 测试覆盖(关键算法是否有回归测试、mock 是否掩盖真实问题)

#### 6. 跨文件一致性检查(Cross-file Consistency Checks,对应 Req 2.5)

横向拉齐"同一物理量在多文件中的取值",不一致作为独立条目记入第 3 节并列出所有相关文件:

- **量程 / voxel 限制**:`xtm60_*` 适配器 range/height ↔ `dual_lidar_fusion.yaml` max_range/voxel ↔ `rtabmap_params.yaml` Grid/RangeMax ↔ `cloud_to_occupancy_grid.yaml`(已知 #30:Grid/RangeMax=8 vs 融合 12)。
- **坐标系 / 外参**:`wheelchair.urdf.xacro` 雷达/IMU 安装位姿 ↔ `static_transforms.yaml` ↔ `calib/*.yaml` 左右雷达外参 ↔ 融合节点 `_lookup` 使用的 frame(已知 #50:右雷达占位外参)。
- **频率 / QoS**:轮速 publish_rate ↔ IMU 频率 ↔ `robot_localization_ekf.yaml` frequency ↔ 各 pub/sub QoS(已知 #37:EKF 30 vs 轮速 50 vs IMU 200)。
- **时间基准**:适配器 `use_sdk_timestamps` ↔ 融合 `header.stamp` ↔ EKF/RTAB-Map approx_sync(已知 #13/#15/#41:墙钟 vs 采集时刻)。

#### 7. 已知背景核对(对应 Req 2.6 / 3.2 / 3.4)

第 4 节须逐条体现已知真实问题并标注修复状态:EKF 轮速 yaw 主导(已修)、雷达 URDF 高度(已修)、
RTAB-Map yaml 从不加载(已修)、雷达盒子遮挡假点(未修)、融合时间戳用墙钟(未修)、纯激光窄
FOV 无回环(未修/架构性)、IMU 仅姿态级无室内位移观测(未修/硬件约束)。起点文档 50 条结论迁移
进缺陷表并重新编号,保留技术实质。

#### 8. 后续如何驱动修复计划(Feeding the remediation plan)

缺陷表的"编号 + 严重度 + 修复方向"三列是 tasks 阶段的输入:tasks 将按**严重度优先 + 同文件/同模块
聚合**把缺陷转成修复任务,每个修复任务引用一个或多个缺陷编号,从而保证可追踪。审查阶段本身不改
运行时代码,只产出可执行的修复线索。

## Testing Strategy

### Validation Approach

由于本"修复"是审查文档而非运行时代码,这里的"测试"= **对审查报告本身的验证**:先证明现有审查
确实存在缺口(命中 C(X) 的文件被遗漏/未记录),再验证新报告满足 Fix 属性(逐文件、全字段、不封顶)
与 Preservation 属性(排除集不动、真无缺陷如实、已记录结论保留)。

### Exploratory Bug Condition Checking

**Goal**: 在产出完整新报告前,先surface反例,证明现有审查 F 确实漏掉了命中文件,确认根因分析
(范围不全 / 人为封顶 / 粒度不够)成立;若反例不成立则需重新假设。

**Test Plan**: 用文件枚举对照 F 的内容,找出"在范围内、F 未检视"的文件清单;抽样打开其中若干文件
确认确有缺陷,以证明 `isBugCondition` 在这些文件上为真。在"未修复(= 旧报告)"状态下执行,观察其
失败(= 这些文件没有任何条目/痕迹)。

**Test Cases**:
1. **范围缺口反例**:枚举 `wheelchair_navigation` / `wheelchair_safety` / `wheelchair_diagnostics` / `wheelchair_perception` / `wheelchair_sensors` 文件,确认 F 中零条目覆盖(will fail on 旧报告)。
2. **粒度缺口反例**:对建图包内文件,确认 F 缺少"已检视无缺陷"痕迹,无法区分未看/看过(will fail on 旧报告)。
3. **封顶反例**:统计抽样若干包的真实缺陷估计,论证总数可能 > 50,F 的 50 条上限会截断(may fail on 旧报告)。
4. **跨文件缺口反例**:确认 F 未系统列出"量程 8 vs 12 / 时间基准"以外的跨文件不一致(will fail on 旧报告)。

**Expected Counterexamples**:
- 非建图包文件在 F 中完全无条目、无检视痕迹。
- 可能原因:范围锚定偏差、数量驱动收尾、无覆盖清单机制。

### Fix Checking

**Goal**: 验证对所有命中缺陷条件的文件,新审查 F' 都产出期望结果(已检视 + 全 6 字段 + 不封顶)。

**Pseudocode:**
```
FOR ALL X WHERE isBugCondition(X) DO
  records := F'(X)
  ASSERT examined(X) = true                          // 覆盖清单标"已检视"
  ASSERT FOR EACH d IN defectsOf(X):
           EXISTS r IN records WITH
             hasFields(r, {id, location, description, impact, severity, fixDirection})
  ASSERT noArtificialCap(records)                     // 不因 50 条上限截断
END FOR
```

**实现手段(对报告的检查)**:
- 解析缺陷表,断言每行 6 字段非空、编号全局连续无重复、severity ∈ {阻断,高,中,低}。
- 断言覆盖清单与文件枚举一一对应,无"未检视"的范围内文件遗留。
- 断言报告中无任何"至少/至多 N 条"措辞;条目数仅由发现决定。
- 建图相关条目的"影响"列必须单独包含对建图质量的说明。

### Preservation Checking

**Goal**: 验证对所有不命中缺陷条件的输入,F' 保留 F 的真实结论且行为不变(不捏造、不重复报告已修项、
不动排除集)。

**Pseudocode:**
```
FOR ALL X WHERE NOT isBugCondition(X) DO
  // 排除集文件不被强行检视;真无缺陷文件如实标注;
  // 已记录的真实结论与已修状态被保留
  ASSERT F'(X) 保留 F(X) 中真实技术结论 AND 不捏造新缺陷
END FOR
```

**Testing Approach**: 推荐属性式(穷举/抽样)检查,因为排除集与"真无缺陷"文件数量大,逐个人工核对
易漏;对全量文件清单做属性断言能覆盖整个输入域。

**Test Plan**: 先在"未修复"状态确认排除集与已修项的事实(哪些文件是临时/构建产物、哪三项已修),
再对新报告断言这些事实被如实保留。

**Test Cases**:
1. **排除集保留**:断言 `build/`、`__pycache__/`、`.git/`、`.pytest_cache/`、二进制图片、根目录临时 dotfiles 不出现在缺陷表中(除非被正式链路引用)。
2. **起点结论保留**:断言 F 的 50 条技术结论均能在 F' 缺陷表或第 4 节找到对应(可重编号)。
3. **真无缺陷如实**:抽样若干简单文件(如 `__init__.py`、纯数据 yaml),断言它们在覆盖清单标"无缺陷"而非被强行制造条目。
4. **已修项不重复报告**:断言 EKF 轮速 yaw / 雷达 URDF 高度 / RTAB-Map yaml 加载在第 4 节标"已修",未出现在待修缺陷里。

### Unit Tests

- 缺陷表行级校验:6 字段完整性、编号唯一连续、严重度取值合法。
- 覆盖清单完整性:清单条目集合 == 文件枚举集合(差集为空)。
- 排除集校验:缺陷表与覆盖清单不含排除集路径。

### Property-Based Tests

- 对文件枚举全集生成属性:`∀ 文件 f ∈ in-scope → f 出现在覆盖清单且状态为"已检视"`。
- 对缺陷条目生成属性:`∀ 条目 r → r 的 6 字段非空 ∧ severity 合法 ∧ location 指向真实存在的文件路径`(防止猜测性/捏造条目,对应 Req 3.5)。
- 对跨文件量值生成属性:`∀ 共享物理量(量程/频率/外参/时间基准)→ 多文件取值一致,否则存在对应跨文件条目`。

### Integration Tests

- 端到端覆盖核对:从 `find` 枚举 → 覆盖清单 → 缺陷表,验证三者闭环一致,证明逐文件穷尽。
- 起点文档迁移核对:逐条比对 F 的 50 条与 F' 第 4 节/缺陷表的对应关系,确认无技术结论丢失。
- 修复计划可追踪性核对:抽样验证每个修复方向可由文件位置佐证、可被 tasks 阶段引用编号转成修复任务。
