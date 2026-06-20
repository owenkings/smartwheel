# Bugfix Requirements Document

## Introduction

这是一个**缺陷审查(code audit)型 bugfix spec**。要被"修复"的对象不是某段运行时代码,而是
**现有审查工作本身的不完整性**。

当前唯一的审查产物是 `docs/mapping_code_review_2026-06-17.md`,它存在三个结构性缺陷:

1. **范围不全** — 只聚焦"建图链路",未覆盖 SmartWheel 项目(`/home/nvidia/smartwheel`)
   的全部源文件(导航、感知、安全、诊断、传感器适配、运动学/驱动、URDF、launch、
   配置 yaml、shell 脚本、C/C++ 节点、测试等)。
2. **人为封顶** — 列了恰好 50 条。用户明确反对任何"至少/至多 N 条"的封顶,要求
   **有多少缺陷就记录多少条,不设上限**。
3. **粒度不够** — 未做到**逐文件穷尽**:不能保证每个源文件都被实际打开、逐函数检视过,
   也未对"已检视但无缺陷"的文件留痕。

因此本次审查的"缺陷条件 C(X)"作用在**项目中的每个源文件 X** 上:对任意源文件,只要其中
存在"代码/配置行为偏离正确、健壮、规范预期"的实例,而该实例**未被现有审查以合规条目记录**,
即视为命中缺陷条件。修复(F')是产出一份**逐文件、穷尽、不封顶**的审查报告,使每个源文件
都被检视、每个被发现的缺陷都按统一格式记录成独立条目。

已知背景(作为输入,不得遗漏,且需在新审查中继续向外扩展,而非止步于此):

- 起点文档 `docs/mapping_code_review_2026-06-17.md` 的 50 条(聚焦建图),仅作为起点。
- 已确认的真实问题(部分已修):EKF 之前让轮速 yaw 主导(已修)、雷达高度 URDF 写错(已修)、
  RTAB-Map yaml 参数从不加载(已修)、雷达盒子遮挡产生固定假点、融合点云时间戳用墙钟、
  纯激光窄 FOV 无回环、IMU 为 WHEELTEC H30 / Yesense YIS106(姿态级,无室内位移观测)。
- 硬件:XT-M60 Flash ToF 雷达(160×60 点阵,120°×60° FOV,0.3–12 m),双雷达
  (左盒装 / 右裸装),ZLAC8030D 差速底盘,H30 IMU。建图栈为 RTAB-Map(非 slam_toolbox)。

### 审查范围(Scope)

被审查的"输入域"为 `/home/nvidia/smartwheel` 下的全部一手源文件,至少包含:

- Python 节点与库:`src/**/wheelchair_*/**/*.py`(运动学、里程计/驱动、Modbus、点云融合、
  cloud_utils、栅格投影、KISS-ICP、RGB 着色、一致性监控、感知、导航、安全、诊断、传感器适配、
  mock、各 `setup.py`)。
- 测试:`src/**/test/**/*.py` 与 `auto_test/*.py`。
- C / C++ 源:`src/wheelchair_bringup/src/*.cpp`、`*.c`、`include/**`、`CMakeLists.txt`。
- ROS launch:`src/**/launch/*.launch.py`。
- 配置 yaml:`src/**/config/*.yaml`(EKF、RTAB-Map、nav2、安全、诊断、传感器、scan、URDF static tf 等)。
- URDF / xacro:`src/wheelchair_description/urdf/*.xacro` 及 description 配置。
- Shell 脚本:`scripts/*.sh` 及 `src/**/scripts/*`。
- 地图后处理脚本:`src/wheelchair_mapping/scripts/*.py`。
- 标定文件:`calib/*.yaml`(外参/内参一致性)。
- RViz 配置 `*.rviz` 仅在影响建图/可视化正确性判断时检视。

**显式排除**:`build/`、`__pycache__/`、`.git/`、二进制图片、`.pytest_cache/`,以及
仓库根目录的临时 dotfiles(`.b.log`、`.test.py`、`.jog.py` 等)——除非审查中发现它们被
正式链路引用。

### 审查维度(Audit Dimensions)

每个文件至少按以下维度检视:算法错误、功能性 bug、不严谨/不稳健的方法、健壮性隐患
(异常吞没、空值/边界、资源泄漏)、规范问题(命名/重复/死代码)、架构缺陷(职责混叠、
生命周期管理)、参数一致性(跨文件量程/坐标/频率/外参冲突)、时间戳与同步问题、
TF/坐标系正确性、QoS 匹配、单位/符号错误。

### 缺陷条目记录格式(Per-Defect Record Format)

每条缺陷必须是独立条目,包含且仅包含以下字段:

1. **唯一编号**(全局连续,不复用旧文档编号)
2. **文件位置**:相对路径 + 函数名/类名 + 大致行号或行号区间
3. **问题描述**:具体说明偏离了什么预期
4. **影响**:对系统行为的影响,**建图链路相关项必须单独说明对建图质量的影响**
5. **严重度**:阻断 / 高 / 中 / 低 之一
6. **建议修复方向**:可执行的修复思路(非"重写一切"式空话)

### 完整性要求(Exhaustiveness)

- **逐文件覆盖**:范围内每个源文件都必须被实际检视;输出需附**文件清单**,标注每个文件
  "已检视 / 缺陷数 / 无缺陷"。
- **不封顶**:不得设置任何"至少 N 条""至多 N 条"的人为数量目标;条目数量由实际发现决定。
- **不丢已知项**:已知背景中的真实问题须在新报告中体现(标注"已修/未修")。

## Bug Analysis

### Current Behavior (Defect)

1.1 WHEN 审查范围被限定为"建图链路" THEN the system(现有审查产物)遗漏导航、感知、安全、
诊断、传感器适配、运动学/驱动、URDF、launch、配置 yaml、shell 脚本、C/C++ 节点、测试等
非建图文件中的缺陷

1.2 WHEN 审查以固定数量(50 条)收尾 THEN the system 人为封顶缺陷条目数,可能在达到 50 条后
停止记录,遗漏第 51 条及以后的真实缺陷

1.3 WHEN 项目中存在未被现有审查打开/检视的源文件 THEN the system 对这些文件不留任何检视痕迹
(既无缺陷条目,也无"已检视无缺陷"记录),无法证明逐文件覆盖

1.4 WHEN 某个源文件含有缺陷但缺少完整记录字段 THEN the system 产出的条目缺少唯一编号、
精确文件位置、严重度或修复方向之一,导致条目不可追踪、不可执行

1.5 WHEN 缺陷跨多个文件表现为参数/坐标/时间基准不一致 THEN the system(聚焦单链路的审查)
未系统性地做跨文件一致性比对,遗漏此类架构级缺陷

### Expected Behavior (Correct)

2.1 WHEN 执行审查 THEN the system SHALL 覆盖"审查范围"小节列出的全部一手源文件类型
(Python/C/C++/launch/yaml/urdf/shell/test/calib),不限于建图链路

2.2 WHEN 记录缺陷 THEN the system SHALL 按实际发现数量输出条目,不设任何数量上限或下限,
"有多少记多少"

2.3 WHEN 完成审查 THEN the system SHALL 输出一份文件清单,对范围内每个源文件标注
"已检视 / 缺陷数 / 无缺陷",以证明逐文件穷尽覆盖

2.4 WHEN 记录任一缺陷 THEN the system SHALL 为该条目提供全部六个字段(唯一编号、文件位置、
问题描述、影响、严重度、建议修复方向),且建图相关条目须单独说明对建图质量的影响

2.5 WHEN 缺陷涉及跨文件不一致(量程、坐标系、频率、外参、时间基准等)THEN the system SHALL
将其作为独立条目记录,并列出所有相关文件位置

2.6 WHEN 已知背景中列出真实问题 THEN the system SHALL 在报告中体现这些问题并标注其修复状态
(已修/未修),不得遗漏

### Unchanged Behavior (Regression Prevention)

3.1 WHEN 文件位于显式排除集合(`build/`、`__pycache__/`、`.git/`、二进制图片、
`.pytest_cache/`、根目录临时 dotfiles)且未被正式链路引用 THEN the system SHALL CONTINUE TO
将其排除在逐文件审查之外,不为其强行制造缺陷条目

3.2 WHEN 引用起点文档 `docs/mapping_code_review_2026-06-17.md` 中已记录的真实缺陷 THEN the
system SHALL CONTINUE TO 保留这些发现的实质内容(可重新编号/重组),不丢失其技术结论

3.3 WHEN 某文件经检视确实不含缺陷 THEN the system SHALL CONTINUE TO 如实记录其为"无缺陷",
不得为凑数而捏造问题

3.4 WHEN 已确认"已修复"的问题(EKF 轮速 yaw、雷达 URDF 高度、RTAB-Map yaml 加载)THEN the
system SHALL CONTINUE TO 将其标注为已修,而非作为待修缺陷重复报告

3.5 WHEN 审查输出新报告 THEN the system SHALL CONTINUE TO 仅记录基于实际代码内容的、
可由文件位置佐证的缺陷,不引入无依据的猜测性条目

## Bug Condition 形式化

**Bug Condition Function** — 识别命中审查缺口的输入:

```pascal
FUNCTION isBugCondition(X)
  INPUT: X 为项目源文件 (in-scope source file)
  OUTPUT: boolean

  // X 在范围内、含有偏离正确/健壮/规范预期的实例,
  // 且该实例未被现有审查以"含全部六字段的条目"记录
  RETURN inScope(X)
         AND hasDefect(X)
         AND NOT fullyRecordedByExistingReview(X)
END FUNCTION
```

**Property: Fix Checking** — 修复后对每个命中文件的期望:

```pascal
// F  = 现有审查(mapping-only, 封顶 50, 非逐文件)
// F' = 新的逐文件穷尽审查
FOR ALL X WHERE isBugCondition(X) DO
  records ← F'(X)
  ASSERT examined(X) = true
  ASSERT FOR EACH d IN defectsOf(X):
           exists record r IN records WITH
             hasFields(r, {id, location, description, impact, severity, fixDirection})
  ASSERT noArtificialCap(records)   // 不因 50 条上限而截断
END FOR
```

**Property: Preservation Checking** — 对未命中(排除集 / 已穷尽记录 / 真无缺陷)输入保持不变:

```pascal
FOR ALL X WHERE NOT isBugCondition(X) DO
  // 排除集文件不被强行检视;真无缺陷文件如实标注无缺陷;
  // 已记录的真实结论与已修状态被保留
  ASSERT F'(X) 保留 F(X) 中的真实技术结论 AND 不捏造新缺陷
END FOR
```
