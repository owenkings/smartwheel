# Wheelchair 常用命令与操作手册

> 本手册按当前源码中的命令、参数与存储规则编写。命令运行于已完成依赖安装和本机配置的 Linux / Bash 环境；带窗口的命令需要图形桌面。
> 先完成 [换机部署](../getting_started/deployment.md) 和 [存储配置](storage.md)。下文用 `PROJECT_ROOT` 表示实际源码根，用 `WC_DATA` 表示生效配置选定的数据根，均不绑定某台机器。
> 设备身份放在 Git 忽略的本机配置中。公开配置中的示例身份不能直接用于连接设备。录包、地图和报告不会随源码仓库发布。
> 本文覆盖日常操作、离线分析、标定与维护入口。测试源码是回归资产；测试产物和任务记录保存在数据根的 `dev_archive/`。

## 目录

- [录完后：融合地图并查看（直接复制）](#record-to-map)
- [1. 最常用命令速查与当前可用范围](#quick)
- [2. 终端、环境、路径与会话名称](#environment)
- [3. 录制一次数据：不录相机和超声波](#capture)
- [4. 录制参数、其他采集范围及档案结构](#capture-options)
- [5. 停止、保存、PARTIAL 与完成核验](#stop-save)
- [6. 打开实时点云和 RViz](#live-view)
- [7. 实时建图、手动驾驶与保存地图](#live-map)
- [8. 同一份录包反复融合、建图与算法对照](#compare)
- [8.7 零偏、轮速一致性与几何纠偏重建](#offline-refine)
- [9. 查看三维点云地图与二维栅格](#saved-map)
- [10. 左右点云选点、配准与网页叠加](#alignment)
- [10.0 幅度图辅助选点](#amplitude-alignment)
- [11. 采集新的静态配准场景](#static-pair)
- [12. 频率、raw/filtered、外参与坐标轴](#concepts)
- [13. IMU 零偏与参数文件](#calibration)
- [14. 状态、故障定位与常见报错](#diagnostics)
- [15. 研发用底层命令与 ROS 回放](#advanced)
- [16. 构建、测试、回退与文件整理](#maintenance)
- [17. 所有顶层脚本索引和帮助命令](#entrypoints)
- [18. 输入选择与核对依据](#examples)

<a id="quick"></a>
## 1. 最常用命令速查与当前可用范围

所有相对命令先完成[终端准备](#environment)，并进入实际工程根目录：

```bash
cd "$PROJECT_ROOT"
```

统一桌面面板可用 `bash scripts/panel` 打开；面板中的操作仍受各模块的设备身份、占用、输入完整性和停止收尾检查约束。

| 目的 | 入口 | 是否连接真实设备 | 输出/结果 |
|---|---|---|---|
| 统一桌面面板 | `bash scripts/panel` | 按用户启动的功能 | 采集、离线任务、地图和标定操作入口 |
| 录一次供后续研究的数据 | `python3 scripts/wc_phase1 capture ...` | 是，按 profile 选择 | 数据根 `data/experiments/<session>/` |
| 录制时看左右点云并手动驾驶 | 上述命令加 `--preview --manual-drive` | 是，同一批读取者 | 录制与显示分开，显示降频不抽掉录制帧 |
| 独立看左、右点云 | `bash scripts/view_lidars.sh` | 是 | RViz 实时预览，见第 6 章 |
| 主界面仅预览 | `python3 scripts/map left`、`right`、`all` | 是 | 默认不累计建图 |
| 实时累计建图 | `python3 scripts/map all --mapping true` | 是 | 工作会话在 `reports/maps/`，停止后询问保存 |
| 录包离线融合/比较 | `python3 scripts/wc_phase1 compare ...` | 否 | 指定的 数据根分析目录 |
| 离线融合同时生成地图 | `compare ... --native-map` | 否 | 分组轨迹、RTAB-Map 数据库、PLY/PGM/YAML |
| 看已保存三维/二维地图 | `python3 scripts/view_map 路径` | 否 | RViz，仅查看 |
| 左右点云拖动叠加/ICP | `bash scripts/align_lidar_clouds.sh --input ...` | 否 | 网页及离线外参候选 |
| 左右同名点人工选点 | `bash scripts/pick_lidar_points.sh --input ...` | 否 | 对应点及离线求解结果 |
| 复核录制档案 | `python3 scripts/wc_phase1 doctor --session-root ... --verify-archive` | 否 | 输出完整性与问题明细 |

<a id="record-to-map"></a>
### 1.1 录完后：融合地图并查看（直接复制）

**录制 `capture` 负责保存传感器数据；离线融合建图入口叫 `compare`。即使只生成一张地图，也使用 `compare --native-map`。** 不加 `--native-map` 只生成前端融合、轨迹和诊断文件，不生成可查看的原生三维地图。

先完成第 2 节准备，然后选择已经录完的会话。`SESSION` 必须是现有文件夹名；示例不会创建或改名原始录包。派生结果写入同一已配置数据根的新分析目录。

```bash
SESSION="实际会话名"
DATASET="$WC_DATA/data/experiments/$SESSION"
OUT="$WC_DATA/data/analysis/fused_${SESSION}_$(date +%Y%m%d_%H%M%S)"

python3 scripts/wc_phase1 compare \
  --dataset "$DATASET" --output "$OUT" \
  --estimators five_state --mechanical-initial \
  --cloud raw --filter on --input-rate-hz 5 \
  --native-map --native-rate-hz 5 --native-wall-interval-s 0.2
```

本例使用录包的机械安装初值；已有经确认外参时按输入配置或显式 `--hardware-setup` 选择。

这条命令使用**左右两只雷达、IMU和轮反馈**：按该录包的配置和现有机械关系转换坐标、处理运动，再累计生成三维点云和二维地图。它不是简单把两个原生坐标系的点直接拼在一起。先固定五状态EKF、raw分支和应用层过滤；不同时改变多个算法因素。

- 先等待融合命令结束，再查看；输出目录必须是新名字，**不要提前 `mkdir "$OUT"`**。
- 如果录制状态为 `PARTIAL`，先查看清单问题和实际留存范围；确认用于离线分析后才额外加 `--allow-partial`。它不修复档案、不改变原状态。

- `--mechanical-initial` 使用已有安装资料进行离线实验，不修改实时外参；生成地图不等于几何精度已经验收。
- 前端和建图采样均设5Hz；源数据保留。`--native-wall-interval-s 0.2` 只缩短离线执行时相邻发布之间的额外等待，每帧仍等待RTAB-Map确认，不改变记录时间戳或采样集合。默认等待为1秒，低性能环境可恢复为1。
- 没有设置 `--source-limit` 或 `--native-limit`，处理整份录包；包括准备和收尾阶段的有效数据。

**融合完成后，在同一个终端查看：**

```bash
MAP_EXPORT="$OUT/native/five_state_hz5_filter_on/export"
python3 scripts/view_map "$MAP_EXPORT" --check
python3 scripts/view_map "$MAP_EXPORT"
```

第一条查看命令只检查文件；第二条在图形桌面打开 RViz。另开终端时重新完成第 2 节准备，并把 `OUT` 设置为实际已完成的结果目录：

```bash
OUT="$WC_DATA/data/analysis/实际融合结果目录"
python3 scripts/view_map "$OUT/native/five_state_hz5_filter_on/export"
```

RViz默认使用可用的软件渲染回退；地图可能很大，首次加载需要等待。可以用 `--max-view-points 300000` 降低显示负载，不会删除原地图点。已有离线地图窗口占用时，先正常关闭那个窗口。完整融合参数在[第8章](#compare)，查看与RViz操作在[第9章](#saved-map)。


**操作范围：**

- `mapping_core` 选择左右雷达、IMU 和轮反馈，不启动相机和超声波。
- 实时双雷达运动建图需要生效配置具有有效的 `T_axle_lidar_left`、`T_axle_lidar_right`。使用 `python3 scripts/map all --mapping true --check-config` 查看本机状态；缺项会阻止相应建图。
- 离线 `compare --mechanical-initial` 使用录包安装资料做实验，不修改正式外参。
- 采集中的 `--manual-drive` 打开用户驾驶入口；运动行为由用户实际操作决定。
- 原始录包保留与停止后的派生地图保存是两项独立操作。软件流程完成不表示几何精度已经验收。

<a id="environment"></a>
## 2. 终端、环境、路径与会话名称

### 2.1 每个新终端的通用准备

`PROJECT_ROOT` 改为这台机器的实际源码目录。以下读取生效存储配置并检查目标；不会自动更换磁盘或创建备用数据根。配置尚未完成时先看第 2.4.1 节。

```bash
export PROJECT_ROOT="$HOME/wheelchair"
cd "$PROJECT_ROOT"
export PYTHONPATH="$PROJECT_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
WC_DATA="$(python3 - <<'PY'
from pathlib import Path
from wc_runtime.storage_policy import StoragePolicy
policy = StoragePolicy(Path.cwd())
policy.check()
print(policy.archive_root if policy.enabled else policy.project_root)
PY
)" && export WC_DATA
test -n "$WC_DATA" && test -d "$WC_DATA"
```

只有上述命令成功后才继续；失败时修复配置或挂载，不把空变量用于后续命令。`WC_DATA` 只是本终端引用数据位置的变量，不会覆盖程序的存储配置。

直接使用模块、ROS 命令或离线原生建图前，还需要载入 ROS 和本工程安装环境：

```bash
WC_ROS_SETUP="$(python3 -c 'from wc_runtime.project_paths import ros_setup_path; print(ros_setup_path())')"
source "$WC_ROS_SETUP"
source "$PROJECT_ROOT/install/main/setup.bash"
```

安装目录需先成功构建。包装入口会安排其所需环境；不要在同一终端混入其他工程的安装环境。

### 2.2 图形桌面与 SSH

- `--preview`、`--manual-drive`、RViz、浏览器需要有效图形会话；首选直接在 Orin 桌面打开终端。
- SSH 适合看日志、无窗口采集、离线比较、文件核验。仅设置一个猜测的 `DISPLAY` 不代表窗口就能正常打开。
- 网页的 `http://127.0.0.1:8767/` 指运行网页服务的那台机器。按本手册在 Orin 启动，就在 Orin 浏览器打开。
- Windows 源码副本与 Linux 运行工程是不同位置。不要把 Bash 命令原样放进 PowerShell。

### 2.3 逻辑路径与实际路径

| 写法 | 支持存储映射的项目入口 | `ls`、`cat`、`xdg-open` 等系统命令 |
|---|---|---|
| `data/experiments/会话名` | 解析到生效数据根 | 相对于当前 shell 目录 |
| `$PROJECT_ROOT/data/experiments/会话名` | 可作为本工程逻辑路径解析 | 访问该物理路径，不会自动转向数据根 |
| `$WC_DATA/data/experiments/会话名` | 实际数据路径 | 直接访问实际路径 |

系统命令与自写脚本优先用带引号的 `"$WC_DATA/..."`。历史档案中的原路径由相应加载器按来源解析，不通过改写原始证据来迁移。

```bash
ls "$WC_DATA/data/experiments"
xdg-open "$WC_DATA/data/experiments"
python3 scripts/wc_phase1 doctor \
  --session-root "$WC_DATA/data/experiments/实际会话名" --verify-archive
```

### 2.4 数据根与临时目录

| 真实位置 | 内容 |
|---|---|
| `$PROJECT_ROOT/src`、`config`、`scripts`、`tests` | 程序、配置、操作入口和回归测试源码 |
| `$PROJECT_ROOT/install/main` | Orin 已构建安装产物 |
| `$WC_DATA/data/experiments/<session>` | 独立 `capture` 的研究录包 |
| `$WC_DATA/data/bags/<session>` | 底层 `record` 命令的 ROS bag |
| `$WC_DATA/data/analysis/<实验名>` | 离线轨迹、融合、建图与对照结果 |
| `$WC_DATA/data/calibration/manual_points` | 人工点选结果 |
| `$WC_DATA/data/calibration/alignment_candidates` | 拖动/ICP 配准候选 |
| `$WC_DATA/reports/maps/<session>` | 主界面预览/建图的工作会话、日志及保存状态 |
| `$WC_DATA/reports/lidar_stability` | 新的四阶段静态雷达采集 |
| `$WC_DATA/reports/calibration` | 由静态采集准备的 `prepared.json` |
| `$WC_DATA/maps` | 用户保存地图的位置，以及地图管理模块的存储根 |
| `$WC_DATA/dev_archive/<任务>/` | 构建/测试产物、任务资料、开发报告与恢复材料 |

| `$PROJECT_ROOT/.phase1_runtime` | 设备锁、进程登记、控制 socket、源端临时证据 |
| `/dev/shm/wc_capture_<UID>/<session>` | 默认采集暂存区；收尾后核验转存到 数据根 |
| `/dev/shm/wc_compare_<UID>` | 仅显式 `--memory-work` 使用的离线临时区 |

`config/storage.local.json` 完整覆盖公开 `config/storage.json`。普通目录后端检查明确指定的目录；可移动存储后端还检查挂载点和 UUID。目标不可用就报错，不自动换到其他目录。内存暂存断电不持久，等待落盘与核验完成才算保存。

### 2.4.1 换机与数据目录

源码、配置、依赖与安装产物保留在工程，录包、地图、测试输出和日志写到配置的数据根。测试源码 `tests/` 继续保留。

公开默认配置使用普通目录 `~/wheelchair-data`。新机器选择普通目录时，在工程根执行：

```bash
python3 scripts/configure_storage --backend directory --archive-root "$HOME/wheelchair-data"
```

选择可移动存储时，填写这台机器已经核实的挂载点和 UUID：

```bash
MOUNT_POINT="/实际挂载点"
OUTPUT_UUID="实际文件系统UUID"
python3 scripts/configure_storage --backend removable \
  --archive-root "$MOUNT_POINT/wheelchair-data" \
  --mount-point "$MOUNT_POINT" --required-uuid "$OUTPUT_UUID"
```

这是明确修改本机存储配置的操作，不是每次采集前的步骤。配置完成后重新执行第 2.1 节。设备身份、SDK/ROS 依赖与编译见[换机部署](../getting_started/deployment.md)；不照搬其他机器的本机覆盖文件。

### 2.5 会话名称和命令复制

```bash
SESSION="core_$(date +%Y%m%d_%H%M%S)"
DATASET="$WC_DATA/data/experiments/$SESSION"
printf '本次会话：%s\n最终路径：%s\n' "$SESSION" "$DATASET"
```

- 名称使用英文字母、数字、`_`、`-`、`.`，总长最多 64 个字符，首字符为字母或数字。新录制不要复用已有会话名。
- `SESSION`、`DATASET`、`OUT` 等变量只在当前终端保留。另开终端或重启后要重新设置为实际名称；不要重新生成一个时间戳再拿它查旧数据。
- 多行 Bash 命令每行末尾的反斜杠 `\` 后面不能再有空格。容易复制出错时可合并成一行。
- “实际会话名”“实际文件名”均为占位说明，先选择真实输入再执行。不要改名历史录包来匹配示例。

<a id="capture"></a>
## 3. 录制一次数据：不录相机和超声波

### 3.1 日常推荐：最多 300 秒，手推/键盘并带预览

在 Orin 图形桌面终端执行：

```bash
cd "$PROJECT_ROOT"
SESSION="core_$(date +%Y%m%d_%H%M%S)"
DATASET="$WC_DATA/data/experiments/$SESSION"

python3 scripts/wc_phase1 capture \
  --profile mapping_core \
  --session "$SESSION" \
  --duration 300 \
  --manual-drive \
  --preview
```

这条命令记录：左右雷达各自的 SDK 主机滤波前/后点云、IMU 原始包与解码数据、左右轮反馈事务，以及时间、身份、参数、控制和故障证据。**不启动、不录制、不验证相机和超声波。**

- 不必先执行 `--dry-run`；容量预测仅作提示。真实空间、设备身份和占用保护仍自动工作。
- 等终端出现 `RECORDING` 再开始正式路线；初始化期间的数据也会保留。
- 300 秒是正式共同窗口的请求上限。改成 `240` 就是 240 秒，改成 `30` 就是 30 秒；整个进程还包含准备和收尾时间。
- 提前结束按 **一次 Ctrl+C**，然后等待 `STOPPING_SOURCES`、`TRANSFERRING`、`VERIFYING` 和最终结果。不要为了“结束得快”直接关闭终端。
- 使用 `--manual-drive` 后在人工控制窗口操作 WASD；仅鼠标聚焦与正常键盘操作即可，不额外点击运动授权。关闭这个窗口会结束本次采集。
- 关闭独立点云预览窗口仅关闭显示，录制继续。

### 3.2 静止采集，不打开驾驶界面

```bash
SESSION="static_$(date +%Y%m%d_%H%M%S)"
python3 scripts/wc_phase1 capture \
  --profile mapping_core --session "$SESSION" --duration 60 --preview
```

省略 `--manual-drive` 后不启动人工运动控制，但仍会查询轮反馈。再省略 `--preview` 就是不打开图形窗口的采集，适合 SSH：

```bash
SESSION="headless_$(date +%Y%m%d_%H%M%S)"
python3 scripts/wc_phase1 capture \
  --profile mapping_core --session "$SESSION" --duration 60
```

### 3.3 实际录到哪一段

程序等待所选来源全部就绪后才建立共同窗口；准备和停止阶段的额外数据也会保存。请求 300 秒不保证任何故障情况下都能录满。按当前实现，提前 Ctrl+C 即使正常转存，也可能因未覆盖请求的 300 秒而为 `PARTIAL`。

这不等于“没有录到”或“文件一定损坏”。读取实际窗口、源计数和问题列表，后续研究可用 `compare --allow-partial` 显式接受已有部分，见第 8 章。

<a id="capture-options"></a>
## 4. 录制参数、其他采集范围及档案结构

### 4.1 `capture` 完整公开参数

```bash
python3 scripts/wc_phase1 capture --help
```

| 参数 | 默认/可选值 | 作用与边界 |
|---|---|---|
| `--profile` | 必填；`mapping_core` / `mapping_cameras` / `all_sensors` | 明确选取本次来源，不能省略 |
| `--session` | 必填，唯一名称 | 运行会话身份；默认也是档案文件夹名 |
| `--folder-name NAME` | 默认沿用 session | 单独指定显示文件夹名；运行身份仍独立 |
| `--sides left/right/all` | `all` | 选择雷达来源；单侧采集属于诊断范围 |
| `--preview-layout separate/unified` | `separate` | 独立或统一预览布局 |
| `--duration` | 必填，整数 `0..3600` 秒 | 正式采集窗口；`0` 持续录制到用户正常停止，仍受资源与故障保护 |
| `--manual-drive` | 默认关闭 | 启用本会话用户键盘/手推入口；不启动 EKF 或 SLAM |
| `--preview` | 默认关闭 | 同源订阅预览，双雷达各用原生坐标显示；只预览 profile 包含的相机 |
| `--staging memory` | 默认 | 先录内存，停止来源后转存并核验 |
| `--staging disk` | 可选 | 直接写目标盘；exFAT 目标与 `--manual-drive` 组合被拒绝，带人工驾驶保留memory |
| `--output-root PATH` | 默认生效存储配置的 `data/experiments` | 选择已存在的实际父目录；程序检查目录或可移动存储身份，实际档案使用 session 或 folder-name |
| `--required-output-uuid UUID` | 可选，使用时必须同时给 output-root | 显式绑定可移动目标 UUID；未提供时用户目录解析器仍检查配置盘/实际挂载身份，不跳过验证 |
| `--recorder-queue-size N` | `256`，范围 `1..8192` | 有界写盘队列项数；调大不能提高底层盘的带宽 |
| `--dry-run` | 默认关闭 | 可选只读诊断；不启动采集、不创建正式录包；只打印配置不代表已经录制 |
| `--diagnostic` | 默认关闭 | 明确允许产生缺项的诊断档案，缺项仍算问题，不能改判完整 |

### 4.2 三种 profile

| profile | 双雷达 | IMU | 轮反馈 | 四相机 | 超声波地址 1–4 |
|---|---|---|---|---|---|
| `mapping_core` | 是 | 是 | 是 | 否 | 否 |
| `mapping_cameras` | 是 | 是 | 是 | 是 | 否 |
| `all_sensors` | 是 | 是 | 是 | 是 | 是 |

只有确实需要这些来源时才换 profile。例如：

```bash
SESSION="cameras_$(date +%Y%m%d_%H%M%S)"
python3 scripts/wc_phase1 capture \
  --profile mapping_cameras --session "$SESSION" --duration 60 \
  --manual-drive --preview
```

```bash
SESSION="all_$(date +%Y%m%d_%H%M%S)"
python3 scripts/wc_phase1 capture \
  --profile all_sensors --session "$SESSION" --duration 60 \
  --manual-drive --preview
```

`all_sensors` 缺超声波或相机时不会自动删减设备。默认相机 profile 为 `monitor_320`，320×240，保存成功捕获的 BGR8 帧，zlib 无损压缩；不是原始 UVC 字节，也不保证相机内部没有丢曝光。

### 4.3 保存到指定子目录

日常省略输出参数，使用生效配置。显式 `--output-root` 选择已经存在的实际父目录；内部目录使用目录保护，可移动存储使用配置的或实际唯一 UUID 验证，不会自动改为备用位置。

以下示例先检查数据根，再创建分类父目录：

```bash
python3 -c 'from pathlib import Path; from wc_runtime.storage_policy import StoragePolicy; StoragePolicy(Path.cwd()).check()' && \
  mkdir -p "$WC_DATA/data/experiments/experiment_group"
SESSION="core_$(date +%Y%m%d_%H%M%S)"
python3 scripts/wc_phase1 capture \
  --profile mapping_core --session "$SESSION" --duration 300 \
  --manual-drive --preview \
  --output-root "$WC_DATA/data/experiments/experiment_group"
```

最终路径是 `.../experiment_group/$SESSION/`；使用 `--folder-name` 时文件夹名取该参数。后续 `doctor` / `compare` 使用实际完整会话路径。

若要显式绑定已核实的可移动磁盘，再增加 `--required-output-uuid "$OUTPUT_UUID"`。该参数不能单独使用，不能填其他磁盘的身份。代码、配置、运行目录与文件系统根不能作为用户输出目录。

### 4.4 每路数据在哪里

以下均相对于一个完整会话根，例如 `$WC_DATA/data/experiments/实际会话名/`：

| 文件/目录 | 用途 |
|---|---|
| `capture_manifest.json`、`summary_zh.md` | 总结、采集范围、请求/实际窗口、状态、问题和哈希 |
| `progress.json` | 当前/最后阶段；内存暂存时先看 `/dev/shm/wc_capture_<UID>/<session>/progress.json` |
| `configuration/` | 冻结参数、来源原文、设备选择与能力说明 |
| `configuration/source_selection.json` | 明确哪些来源记录、哪些排除 |
| `configuration/capture_contract.json` | 本次设备身份、地址和检查策略 |
| `software/` | 源码与安装产物快照、依赖、Git 状态和哈希 |
| `bag/*.db3`、`bag/metadata.yaml` | 权威 ROS 消息档案 |
| `records.jsonl` | 序号、时间、消息内容哈希索引 |
| `sources/lidar/left/`、`sources/lidar/right/` | 各雷达身份、读回配置、SDK/队列统计、帧配对和 bag 索引 |
| `sources/imu/` | IMU 批次/原包及来源索引 |
| `sources/wheel_feedback.jsonl`、`sources/wheel_summary.json` | 原始轮事务、控制实际发送和读取结果 |
| `sources/wheel/left/`、`sources/wheel/right/` | 两个电机的独立索引 |
| `sources/cameras/<role>/` | 各相机身份、帧索引和无损分块；仅含相机的 profile 存在 |
| `sources/ultrasonic/` | 适配器身份、各地址请求/响应/CRC/超时；仅 all_sensors 存在 |
| `events/`、`logs/`、各模块 summary | 故障、停源、转存、显示等证据；具体文件看本次 manifest |

雷达的四个独立话题是：

```text
/wc_mapping/lidar_left/source_frame
/wc_mapping/lidar_left/source_frame_filtered
/wc_mapping/lidar_right/source_frame
/wc_mapping/lidar_right/source_frame_filtered
```

消息保留 `side`、`sensor_id`、`frame_sequence`、`stream_epoch` 以及设备/主机时间字段。不要仅靠一个合并 PLY 来判断“哪些点来自哪只雷达”；研究时保留原档案及这些索引。

<a id="stop-save"></a>
## 5. 停止、保存、PARTIAL 与完成核验

### 5.1 不同窗口的关闭含义

| 场景 | 推荐结束方式 | 结果 |
|---|---|---|
| `capture` 正在录制 | 在启动终端按一次 Ctrl+C | 停控/停源、排空、转存、核验；原始档案保留 |
| `capture` 人工驾驶窗口 | 关闭此窗口 | 触发本次采集收尾 |
| `capture --preview` 的预览窗口 | 关预览窗口 | 仅退出显示，采集继续 |
| `scripts/map ... --mapping true` | 关主窗口或 Ctrl+C | 先停设备，再询问是否保存本次地图 |
| `view_map` | 关 RViz 或 Ctrl+C | 结束离线查看，不修改原地图 |
| 配准/选点网页 | 在服务终端 Ctrl+C，或到 `--duration` 上限 | 结束服务；只关浏览器标签不等于停服务 |

`TRANSFERRING` 表示正在落盘，不等于“卡死”或“已完成”。可在另一个终端查看进度文件、目录大小和目标盘剩余空间；不要连续 Ctrl+C、拔盘或直接杀进程。

### 5.2 录制完成后的核验

如果还在原终端：

```bash
python3 scripts/wc_phase1 doctor --session-root "$DATASET" --verify-archive
python3 -m json.tool "$DATASET/capture_manifest.json"
xdg-open "$DATASET"
```

另开终端时明确指定旧会话：

```bash
cd "$PROJECT_ROOT"
DATASET="$WC_DATA/data/experiments/实际会话名"
python3 scripts/wc_phase1 doctor --session-root "$DATASET" --verify-archive
```

| 状态/字段 | 如何理解 |
|---|---|
| `COMPLETE` 且完整性检查通过 | 所选来源满足该会话的留存/窗口等检查；不代表地图精度通过 |
| `PARTIAL` | 至少有窗口、缺源、进程、记录链或其他未满足条件；看具体 issues |
| `USER_INTERRUPTED_BEFORE_REQUESTED_DURATION` | 用户提前停止，实际时长少于请求；已有数据仍需按自身完整性核验 |
| `CAPACITY_ESTIMATE_WARNING` | 预测可能录不满，允许启动；运行仍按实际余量保护 |
| `PREPARING` | 正在冻结配置、源码和准备录制器，尚未进入正式采集窗 |
| `WAITING_FOR_ALL_SOURCES` | 等待所选来源取得有效数据 |
| `RECORDING` | 已进入正式共同窗口 |
| `TRANSFERRING` / `VERIFYING` | 正在转存/核验，尚不能拔盘 |

### 5.3 原始档案与派生地图的保存选择

- `capture` 用于采研究数据，原始档案自动保留，不再询问“是否保留本次研究录包”。
- `scripts/map --mapping true` 停止后询问是否保存地图；拒绝则废弃本次工作会话的大数据，保留轻量诊断，不影响历史独立采集。
- `compare` 是显式离线计算，结果直接写入 `--output`，没有实时建图的保存询问。
- `experiment` 留存配置保留重放所需原始资料；`map_only` 会清理相应原始数据并标记不可重放。后续还要研究时保留默认 `experiment`。

<a id="live-view"></a>
## 6. 打开实时点云和 RViz

### 6.1 主界面预览入口

```bash
python3 scripts/map left
python3 scripts/map right
python3 scripts/map all
```

以上是三种替代选择，**逐条按需运行，不要同时复制启动三套硬件读取程序**。默认 `--mapping false`，只实时预览，不累计建图。主界面还按生效 `mapping_live.json` 中的相机、轮接口与交互配置启动附属组件；先查看本机 `cameras_enabled` 等字段，它与 `capture --profile mapping_core` 的明确无相机范围不同。

选择原始/滤波分支：

```bash
python3 scripts/map left --cloud raw
python3 scripts/map left --cloud filtered
```

缺少左右外参时不能把双雷达画在同一个未定义的坐标系里。如果只想看两路各自是否出点，优先使用第 3 章的 `capture --preview`，或下节的传感器专用预览入口。

### 6.2 纯雷达、现有会话的 RViz

只启动雷达和两扇独立 RViz，不启动录制、IMU、轮反馈或电机：

```bash
bash scripts/view_lidars.sh dual 900
```

单侧分别用 `bash scripts/view_lidars.sh single_left 300` 或 `single_right 300`。第1项是模式，默认 `dual`；第2项是启动后时限，默认900秒，范围1～3600。这里用**位置参数**，不能写成 `--mode dual --duration 900`。

默认显示 filtered，可在 RViz Displays 中取消 filtered、勾选 raw。按一次 Ctrl+C 或关闭任一 RViz 窗口触发整个查看会话收尾。日志在 `$WC_DATA/reports/sensor_views/lidar_view_<时间_随机串>/`，它不是研究录包。不能与另一套 capture / map 同时连接相同雷达。

| 雷达 | Fixed Frame | raw普通点云话题 | filtered普通点云话题 |
|---|---|---|---|
| 左 | `lidar_left` | `/wc_mapping/lidar_left/points_raw` | `/wc_mapping/lidar_left/points_filtered` |
| 右 | `lidar_right` | `/wc_mapping/lidar_right/points_raw` | `/wc_mapping/lidar_right/points_filtered` |

已有底层驱动会话时，查看指定原生坐标系：

```bash
python3 scripts/wc_phase1 rviz --view left --session "$SESSION"
```

右侧将 `left` 改为 `right`。此命令只开查看进程，不能凭空产生雷达数据；当前 `--session` 虽然必填，但不会按该ID自动筛选/切换消息来源。`--view 2d` / `3d` 使用相应建图视图，前提是匹配数据/TF 正在发布；缺话题时 RViz 窗口能打开也会是空白。

### 6.3 RViz 的基本查看方法

- 实时单侧原生点云的 Fixed Frame 选 `lidar_left` 或 `lidar_right`，与该侧对应。不要通过乱填 TF 让另一侧“看起来出现”。
- 保存地图使用 `scripts/view_map`，它负责地图话题和配套 RViz 配置；仅执行 `rviz2` 不会加载保存的 PLY/数据库。
- 点云检查话题、消息新鲜度、Fixed Frame、Displays 是否启用、相机视点和渲染日志。进程存在不等于画面有效。
- 系统渲染/软件渲染是显示性能选项，不能改变原始录包点数。Windows 本机不承担这里的建图/渲染任务。

<a id="live-map"></a>
## 7. 实时建图、手动驾驶与保存地图

### 7.1 当前配置边界与启动形式

先使用 `--check-config` 核对生效配置。缺少正式左右雷达到轮轴变换时，相应实时双雷达运动融合会被阻止；以下为入口用法，实际能否启动取决于本机配置与设备检查。

```bash
# 可选的只读配置检查，不打开真实设备
python3 scripts/map all --mapping true --check-config

# 配置能力满足后，双雷达累计建图
python3 scripts/map all --mapping true --cloud raw

# 与面板一致：先预览，在同一 RViz 窗口点击开始／停止并保存
python3 scripts/map all --mapping true --cloud raw --interactive
```

单雷达试验相应使用 `left` / `right`，也须满足该侧运动融合条件。仅希望现在用已有资料出结果时，使用第 8 章离线 `--mechanical-initial`，不用为了打开预览重新测量全部外参。

### 7.2 `scripts/map` 参数

| 参数 | 默认/可选值 | 含义 |
|---|---|---|
| 位置参数 `left/right/all` | 应明确选择 | 使用左、右或双雷达 |
| `--mode left/right/all` | 位置参数的另一种写法 | 二者同时给时必须一致 |
| `--mapping true/false` | `false` | `true` 才累计建图 |
| `--interactive` | 关闭；需 `--mapping true` | 持久窗口先预览；每次点击开始创建新地图，停止关库后恢复当前帧预览并保存旧地图 |
| `--cloud raw/filtered` | 默认配置值 `filtered` | 改变预览/建图所用点云分支 |
| `--name NAME` | 自动唯一名称 | 会话名称 |
| `--output PATH` | 默认 `reports/maps/<session>` | 临时工作会话；不是最终保存目录；支持映射到 数据根的工作路径 |
| `--config PATH` | `config/mapping_live.json` | 本次运行配置 |
| `--retention-profile experiment/map_only` | `experiment` | 保存后是否保留相应原始研究资料 |
| `--check-config` | 关闭 | 只读核对配置/能力 |
| `--duration N` | `0` | **兼容旧参数，当前不触发计时结束**。不要用它安排自动停止 |

没有 `--manual-drive`、`--input-rate-hz`、`--mechanical-initial` 这些 `map` 命令行参数；分别属于采集/离线对照入口或配置文件。实时建图输入频率在 `config/mapping_live.json` 的 `input_rate_hz` 中设置。

### 7.3 键盘与手推

当前混合交互策略支持按住 WASD 接管、正常松键后在新鲜零轮速反馈和协议回执条件满足时恢复手推。空格、失焦、断联、故障和退出走停止控制路径，不能一概视为正常松键后自动释放。

窗口软件状态不能代替实际制动、坡道和断线停车验收。同一时刻只保留一个底盘串口 owner，不同时开另一个旧工程 WASD/编码器读取器。

### 7.4 地图保存与恢复待决定会话

不带 `--interactive` 时，结束实时建图后先停止设备，再询问保存。交互模式点击“停止并保存”会关闭旧地图会话、启动新的当前帧预览，再询问旧地图保存；切换设备期间短暂无数据，界面明确显示“正在切换”。每次开始使用新目录及数据库，参数选择保持不变；预览阶段不累计地图。关闭整个交互窗口会停止设备并取消未完成保存，保留待处理数据；多个待保存会话在最终 `pending_sessions` 中分别列出，退出码为 3，真实收尾失败为 2。选择保存时填写 **数据根内新的目标目录**，例如：

```text
$WC_DATA/maps/新的地图名称
```

目标不能已经存在；不要把旧 `~/maps/...` 例子当成当前默认保存位置。关闭询问框、EOF 或中断决定不等于拒绝，可能保留为 `SAVE_PENDING`。

恢复一个已经停止、尚待决定的工作会话：

```bash
# WORKDIR 替换为终端当时打印的真实工作目录
WORKDIR="$WC_DATA/reports/maps/实际会话名"
python3 scripts/save_map "$WORKDIR"
```

显式保存到新的数据根目录：

```bash
DEST="$WC_DATA/maps/room_$(date +%Y%m%d_%H%M%S)"
python3 scripts/save_map "$WORKDIR" "$DEST"
```

`save_map` 只有 `WORKDIR` 和可选 `DEST` 两个位置参数，处理自己的临时会话，不启动硬件、不覆盖旧目的地。`wc_phase1 save` 是另一个地图版本管理入口，见第 15 章。

<a id="compare"></a>
## 8. 同一份录包反复融合、建图与算法对照

### 8.1 现有录包直接离线建图

选择现有录包；本节保留显式接受 `PARTIAL` 的用法，只有核对清单与实际留存范围后才使用该参数，完整录包可删去。机械初值仅用于离线实验。

```bash
cd "$PROJECT_ROOT"
source "$WC_ROS_SETUP"
source install/main/setup.bash
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONNOUSERSITE=1
export OPENBLAS_NUM_THREADS=1

DATASET="$WC_DATA/data/experiments/实际会话名"
OUT="$WC_DATA/data/analysis/research_$(date +%Y%m%d_%H%M%S)"

python3 scripts/wc_phase1 compare \
  --dataset "$DATASET" \
  --output "$OUT" \
  --allow-partial \
  --mechanical-initial \
  --estimators five_state \
  --cloud raw \
  --filter on \
  --input-rate-hz 5 \
  --native-map --native-rate-hz 5
```

输出到新的 `$OUT`，原始实验不修改。`--native-map` 才运行 RTAB-Map 并导出地图；不加它只做前端轨迹/融合对照，不会凭空出现三维地图导出。

完成后查看本组地图：

```bash
python3 scripts/view_map "$OUT/native/five_state_hz5_filter_on/export"
xdg-open "$OUT"
```

对一个新完整档案，把 `DATASET` 换为其路径；只有确实需要接受不完整档案时才加 `--allow-partial`。正式数据外参已齐备时可以去掉 `--mechanical-initial`，使用该档案/显式覆盖的外参。

### 8.2 比较两个 EKF

保持同一输入、外参、零偏、过滤和频率，仅比较估计器：

```bash
OUT="$WC_DATA/data/analysis/ekf_compare_$(date +%Y%m%d_%H%M%S)"
python3 scripts/wc_phase1 compare \
  --dataset "$DATASET" --output "$OUT" \
  --allow-partial --mechanical-initial \
  --estimators five_state robot_localization \
  --cloud raw --filter on --input-rate-hz 5 --native-map --native-rate-hz 5
```

两组地图分别在：

```text
OUT/native/five_state_hz5_filter_on/export
OUT/native/robot_localization_hz5_filter_on/export
```

当前默认实时估计器不会因为比较结果自动切换。没有独立真值时，报告的闭合偏差、墙厚、轨迹差异不等于绝对定位精度，也不能直接宣称两个 EKF 等效。

### 8.3 比较建图消费频率、过滤和 ICP

以下是三种独立实验，每次换新的输出目录，逐项运行：

```bash
# 5 Hz / 10 Hz：只改变点云消费频率
OUT="$WC_DATA/data/analysis/rate_$(date +%Y%m%d_%H%M%S)"
python3 scripts/wc_phase1 compare \
  --dataset "$DATASET" --output "$OUT" \
  --allow-partial --mechanical-initial --estimators five_state \
  --cloud raw --filter on --input-rate-hz 5 10 --native-map --native-rate-hz 5
```

```bash
# 不作点云消费限频：处理全部有效配对帧；不改变雷达设备发帧率
OUT="$WC_DATA/data/analysis/all_frames_$(date +%Y%m%d_%H%M%S)"
python3 scripts/wc_phase1 compare \
  --dataset "$DATASET" --output "$OUT" \
  --allow-partial --mechanical-initial --estimators five_state \
  --cloud raw --filter on --input-rate-hz 0 --native-map --native-rate-hz 10
```

```bash
# 应用层几何过滤 off / on；不是切换SDK raw/filtered
OUT="$WC_DATA/data/analysis/filter_$(date +%Y%m%d_%H%M%S)"
python3 scripts/wc_phase1 compare \
  --dataset "$DATASET" --output "$OUT" \
  --allow-partial --mechanical-initial --estimators five_state \
  --cloud raw --filter off on --input-rate-hz 5 --native-map --native-rate-hz 5
```

```bash
# 离线ICP约束诊断；shadow 不替换EKF、不把约束自动施加为纠正后的主轨迹
OUT="$WC_DATA/data/analysis/icp_shadow_$(date +%Y%m%d_%H%M%S)"
python3 scripts/wc_phase1 compare \
  --dataset "$DATASET" --output "$OUT" \
  --allow-partial --mechanical-initial --estimators five_state \
  --cloud raw --filter on --input-rate-hz 5 --icp-shadow point_to_plane
```

多个估计器、频率和过滤值同时指定会生成参数组合，消耗时间和空间也相应增加。先做单因素比较更容易解释结果。

**两级频率必须分清：**前端先按 `--input-rate-hz` 产生各组有效输入；原生地图再从所有组的共同有效时刻按 `--native-rate-hz` 取样。后者默认1Hz，以上例子已显式写出。

当前离线入口会把**本次独立RTAB-Map实例**的 `Rtabmap/DetectionRate` 设置为 `--native-rate-hz`，再读取实际值核验。这样不会再出现命令请求5Hz、后端仍为默认1Hz而在第一帧前失败的配置冲突。该覆盖记录在 `native/<组名>/result.json` 的 `native_detection_rate`，原录包配置和实时配置保持不变。

- 同一命令比较 `5 10` 且原生率为5时，两组最终地图仍使用共同样本，不是“一张真用5Hz、另一张真用10Hz”。它适合比较前端状态差异。
- 要看**不同地图消费频率**的完整影响，分两次使用新OUT运行：一组 `--input-rate-hz 5 --native-rate-hz 5`，另一组 `--input-rate-hz 10 --native-rate-hz 10`，其他参数保持一致。
- `--input-rate-hz 0` 只取消前端限频；原生地图仍按显式10Hz从有效时间中取样，不能称为原生地图端也完全不限频。提高这些值不会让旧录包补出新帧。

### 8.4 `compare` 完整公开参数

| 参数 | 默认/可选值 | 说明 |
|---|---|---|
| `--dataset PATH` | 必填 | capture 会话根目录，不是里面的某个 `.db3` |
| `--output PATH` | 必填 | 新的分析目录，不覆盖已有结果 |
| `--project-root PATH` | 脚本所在工程根 | 正常保留默认 |
| `--user-data-paths` | 关闭 | 显式使用用户选择的输入/输出目录，保留逐操作挂载检查；与 memory-work 互斥 |
| `--estimators ...` | `five_state robot_localization` | 一个或多个实现 |
| `--input-rate-hz ...` | `5`；支持 `0` 或 `(0,10]` | `0` 表示所有有效配对点云；轮/IMU事件不会按该值抽帧 |
| `--filter ...` | `on`；`off/on` | 应用层几何筛选，可多值对照 |
| `--cloud raw/filtered` | `raw` | 选择归档的SDK主机滤波前/后分支 |
| `--lidar left/right/all` | 录包原模式 | 选择点云侧；保留 IMU、轮反馈及原时间基准，见8.6节 |
| `--imu-time-mode auto/arrival/device_relative` | `auto` | 优先使用录包冻结的已验证 IMU 时序配置；旧录包保持到达时间语义 |
| `--sides all/left/right` | 沿用录包 | `--lidar` 同义参数；两者同时提供时须一致 |
| `--motion-candidate PATH` | 未设置 | 同录包已验证 IMU 零偏与轮偏航校正；在选定估计器间共享，不改实时标定 |
| `--hardware-setup PATH` | 优先归档配置 | 显式指定外参/机械参数覆盖，仍核验来源并冻结哈希 |
| `--gyro-bias PATH` | 按归档/运行配置 | 显式零偏覆盖；需匹配的 `.evidence.json` |
| `--allow-partial` | 关闭 | 接受已标记不完整的输入；不修复/更改原始完整性 |
| `--mechanical-initial` | 关闭 | 允许当前机械面初值的离线实验，不写实时TF |
| `--memory-work` | 关闭 | 必须同时把output设为 `/dev/shm/wc_compare_<UID>/<新目录>` 直接子目录；不能仍用数据根OUT。易失结果需另行核验转存，日常数据根流程不要加 |
| `--source-limit N` | `0` | 最多N条左右合计的所选raw或filtered雷达SourceFrame消息；0全量，不是成功配对数。此前IMU/轮事件仍处理 |
| `--truth-json PATH` | 无 | 独立真值文件，不能用估计轨迹冒充真值 |
| `--truth-min-common-samples N` | `1` | 真值与所有组共同有效时刻的最少样本数，正整数；不是精度阈值 |
| `--map-quality-policy PATH` | 归档runtime策略，无则内置实验默认 | 显式指定地图有效性实验判据并冻结 |
| `--icp-shadow off/point_to_plane` | `off` | 离线约束诊断，保存质量和拒绝原因 |
| `--native-map` | 关闭 | 启动无真实硬件的RTAB-Map回放、关闭和导出 |
| `--domain N` | `89`，范围1～232且不能为83 | 离线原生建图ROS域，避免与实际采集域冲突 |
| `--native-rate-hz N` | `1.0`，正数 | 原生地图共同输入时刻的取样频率；不能超过各组正频率最小值，全0组时上限10；不是墙钟运行速度 |
| `--native-wall-interval-s N` | `1.0`，有限数值，范围 `[0.05,10]` 秒 | 离线相邻帧发布的额外最小墙钟间隔；每帧仍等原时间戳Info确认。0.2可缩短等待；不改变采样频率和源时间戳 |
| `--native-limit N` | `0` | 原生地图阶段输入数量限制；0 不额外截断 |

### 8.5 配置来源与重放范围

默认读取**录制时冻结的配置**，不会因为今天修改 `config/mapping_live.json` 就自动用新参数重做旧实验。需要新外参/零偏时通过对应参数显式覆盖，并把输出另存新目录。

当前链路比较的是同一份 SDK 输出之后的融合/建图算法；它不是完整旧、新 SDK 原始网络包重放。完成一次录制即可反复运行这些算法，前提是保留源档案、配置、索引及依赖证据。

常用结果文件：根目录 `summary_zh.md`、`result.json`、`artifact_manifest.json`、`frozen_runtime_config.json`、`original_event_index.jsonl`；各组目录下 `trajectory.csv`、`runtime_config.json`、`input/pairs.jsonl`、`frontend/index.jsonl`；实际地图位于 `native/<组名>/`。`--source-limit` / `--native-limit` 截断的结果只代表前缀诊断。

### 8.6 从双雷达录包分别生成左、右、双雷达地图

`compare` 和 `refine` 均支持 `--lidar left|right|all`，省略时沿用录包模式；不需要重新录制或拆分原 bag。`--sides` 是同义入口，同时提供不一致值会被拒绝。

| 参数 | 地图点云来源 | 其他输入 |
|---|---|---|
| `--lidar left` | 左雷达 | 完整 IMU、轮反馈 |
| `--lidar right` | 右雷达 | 完整 IMU、轮反馈 |
| `--lidar all` | 双雷达 | 完整 IMU、轮反馈 |

先选择同一录包和属于它的已验证运动候选；没有候选时删去示例中的 `--motion-candidate` 一项，得到未修正基线：

```bash
DATASET="$WC_DATA/data/experiments/实际会话名"
CANDIDATE="$WC_DATA/data/analysis/该会话运动校正结果/motion_candidate.json"
SIDE=left
CLOUD=raw
OUT="$WC_DATA/data/analysis/single_${SIDE}_${CLOUD}_$(date +%Y%m%d_%H%M%S)"

python3 scripts/wc_phase1 compare \
  --dataset "$DATASET" --output "$OUT" --lidar "$SIDE" --cloud "$CLOUD" \
  --mechanical-initial --estimators robot_localization \
  --motion-candidate "$CANDIDATE" --input-rate-hz 5 --filter on \
  --native-map --native-rate-hz 5 --native-wall-interval-s 0.2 --domain 198
python3 scripts/view_map "$OUT/native/robot_localization_hz5_filter_on/export"
```

逐次执行，每次用新 `OUT`；同一 ROS domain 不同时运行两个原生建图任务。完整回放可能生成数 GB 派生缓存，提前检查空间。5 Hz 是消费上限，不改变原始录包频率。

`result.json` 的 `lidar_selection`、冻结配置的 `offline_lidar_selection` 记录原模式、选中侧、排除侧和身份；`original_event_index.jsonl`、各组 `input/pairs.jsonl` 可核对实际来源。单侧 pairs 只有一个来源。另一侧时间元数据用于维持原始公共时间原点，点云不进入地图；缺少所选侧时明确报错。

单侧仍使用各自到车体的外参。左单侧和双雷达以左雷达为参考，右单侧以右雷达为参考；对比时先对齐参考原点与有效时段，不能把固定平移当漂移。单侧分析能隔离拼接影响，不保证地图更好。

`refine --lidar left` / `right` 可继续比较五状态运动修正、连续噪声和几何约束；官方 EKF 加运动修正使用 `compare`，尚不接入几何反馈。

<a id="offline-refine"></a>
### 8.7 零偏、轮速一致性与几何纠偏重建

`compare` 保留原算法对照；新增 `refine` 用同一份录包生成经过独立窗口验证的运动校正，并可增加有拒绝门的平面点到面约束。全部在 Orin 运行，不连接传感器、不发驾驶指令、不改实时标定或默认估计器。

当前 `refine` 使用五状态平面 EKF；`robot_localization` 的原算法对照仍用 `compare`。不要把 `refine` 的会话校正结果与未用同样校正的官方 EKF 结果称为只改变估计器的对照。

`calibrated_motion` 保存运动校正结果，启用几何实验时另有 `geometric_motion`；地图位于 `native/<组名>/export`。完成后显式选择实际输出目录查看：

```bash
REFINE_OUT="$WC_DATA/data/analysis/实际已完成的校正结果"
python3 scripts/view_map "$REFINE_OUT/native/calibrated_motion/export" --max-view-points 300000
# 只有本次确实生成了几何地图，才在关闭上一窗口后运行：
python3 scripts/view_map "$REFINE_OUT/native/geometric_motion/export" --max-view-points 300000
```

末尾加 `--check` 可只检查文件。降低查看点数不修改完整地图。名称包含几何或校正，不表示结果一定更准确；比较采用同帧、同坐标、同尺度，并读取各自的通过/拒绝原因。

**在新目录计算运动校正与几何对照：**

```bash
cd "$PROJECT_ROOT"
source "$WC_ROS_SETUP"
source install/main/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
SESSION="实际会话名"
DATA="$WC_DATA"
OUT="$DATA/data/analysis/refined_${SESSION}_$(date +%Y%m%d_%H%M%S)"

python3 scripts/wc_phase1 refine \
  --dataset "$DATA/data/experiments/$SESSION" \
  --output "$OUT" \
  --allow-partial \
  --mechanical-initial \
  --input-rate-hz 5 \
  --geometry on \
  --native-map \
  --native-cells calibrated_motion geometric_motion
```

不要预先创建 `$OUT`。省略 `--motion-candidate` 时，程序从录包选择不重叠的预热、零偏训练、零偏验证、轮速训练、轮速验证窗口。录包应包含启动和结束静止段及双向转弯；窗口条件不满足时保留拒绝原因，不会假定零偏为零或强制给出拟合参数。每份录包必须通过自己的窗口与留出验证。

复用已验证候选时，为上述 `refine` 命令增加 `--motion-candidate "$CANDIDATE"`，其中 `CANDIDATE` 是属于同一录包的实际候选路径。候选绑定设备身份、原始数据、轴向和时钟，不用于其他录包，也不作为永久轮距或轮径标定。

| 参数 | 默认 | 用法与边界 |
|---|---|---|
| `--dataset PATH` | 必填 | 原始 capture 会话目录，通常在数据根 `data/experiments` |
| `--output PATH` | 必填 | 新的数据根分析目录；原始录包与已有地图不覆盖 |
| `--project-root PATH` | 当前源码根 | 明确选取执行工程 |
| `--user-data-paths` | 关闭 | 显式使用用户选择的数据目录，仍检查挂载与输入来源 |
| `--imu-time-mode auto/arrival/device_relative` | auto | 录包冻结时序或显式时序选择 |
| `--lidar left/right/all`、`--sides` | 沿用录包 | 单侧/双侧选择；同义参数同时设置时须一致 |
| `--motion-candidate PATH` | 自动估计 | 选择已通过验证且绑定当前录包的候选，保留完整内容及 SHA-256 |
| `--allow-partial` | 关闭 | 允许研究原状态为 PARTIAL 的录包；不会把录制改判完整 |
| `--mechanical-initial` | 关闭 | 离线使用已保存 安装初值；不写入正式实时标定 |
| `--input-rate-hz` | 5 | 点云消费上限，范围 `(0,10]`；IMU和轮反馈仍处理全部实际事件 |
| `--process-noise` | legacy | `legacy` 保留原分段常加速度模型；`white_acceleration` 是显式连续白加速度 PSD 单因素试验 |
| `--linear-acceleration-psd` | white 模式下 0.25 | 单位 m²/s³；有限正数，仅 white 模式可用；不是旧加速度方差的单位 |
| `--angular-acceleration-psd` | white 模式下 0.25 | 单位 rad²/s³；有限正数；未作独立动态标定，不能当作实测噪声 |
| `--cloud` | raw | 可选 raw/filtered；几何或算法对照应固定此项 |
| `--geometry` | off | 默认只做运动校正；`on` 才增加几何实验，当前没有稳定优于运动校正的实包证据 |
| `--native-map` | 关闭 | 加此项才运行 RTAB-Map、关闭数据库并导出可查看的地图 |
| `--native-cells` | calibrated_motion | 可选一项或两项；选择 geometric_motion 时同时加 `--geometry on` |
| `--domain` | 89 | 隔离的离线 ROS 域，不得使用生产域 83；已有同域任务时拒绝 |
| `--native-wall-interval-s` | 0.2 | 每帧确认后最短发送间隔，范围 0.02–60 秒；不改原始采样时间 |
| `--resume` | 关闭 | 继续已有输出，仅复用有完成证据且文件哈希一致的阶段；不跳过损坏或未收尾阶段 |

若几何阶段或原生地图阶段尚未开始，可以对原 `$OUT` 使用相同参数加 `--resume`。未完成阶段留下的半成品不会被静默覆盖；通常应换新 `OUT` 重算。代码、输入、候选或有效配置已变时使用新的输出目录。

日常只生成运动校正地图时，省略 `--geometry on` 和 `--native-cells ...` 即可。连续白加速度试验在新输出目录显式增加 `--process-noise white_acceleration`；这时先保持几何关闭，避免混合多个改变。两个过程模型的单位不同，程序会在会话快照和估计报告中分别记录，不把旧数字静默改成新含义。

**修正做了什么：**

1. IMU 在原生轴扣一次经独立静止窗口验证的三轴零偏，再做安装轴变换。
2. 轮角速度采用当前录包验证的 `w'=a*w+b*v`；完整测量协方差同步变换为 `J R Jᵀ`，包含交叉项。原始反馈保留，不把相同轮速积分位姿再重复融合。
3. 运动校正后，从原始左右 SourceFrame 重新执行融合与补偿，不只改最终轨迹文件。
4. 几何约束保留三维点，优化平面位姿，经过覆盖、残差、改变量及稳定平面支撑的退化检查。单墙或走廊缺乏完整平面位姿约束时回退运动先验；不按房间示意图强制矩形，不强制起终点闭合。
5. 几何输出的左右雷达补偿与地图使用同一个最终位姿提供者。几何协方差没有独立标定，明确记录额外模型不确定度，不宣称是准确的后验置信度。

**在哪里找原因与证据：**

| 文件 | 内容 |
|---|---|
| `motion_candidate.json`、`candidate_verification.json` | 应用的系数、独立窗口验证及来源核验 |
| `calibration/` | 自动估计时的窗口、运行快照、原始消息摘要；不通过时的 `rejection.json` |
| `frozen_runtime_config.json`、`refinement_software/` | 本次参数与实施代码快照 |
| `calibrated_motion/prior_initialization.json`、`frontend/index.jsonl` | 偏置应用、创新、接受/拒绝计数、每帧原来源及补偿位姿 |
| `geometric_motion/registration.jsonl` | 每帧匹配、退化、拒绝原因、候选/实际变换与分段 |
| `geometric_motion/frontend/index.jsonl` | 几何后的位姿、左右来源时刻、重新补偿的点位移；原 EKF 记录保存在 `calibrated_ekf_report` |
| `native/<组名>/` | 实际 ROS 发布与逐帧确认、原生日志、数据库和导出 |
| `refinement_result.json` | 全流程状态、原始录制状态、输入哈希和精度边界 |
| `refinement_artifacts.json`、`refinement_complete.json` | 输出文件清单、哈希和最终完成标记；只有运行中间状态不能视为完成 |
| `refinement_failure.json` | 本次失败原因，断盘时可能只能打印到终端 |

相同帧、相同高度切片和相同比例便于比较。起终点距离、朝向差是闭合一致性指标；没有精确复位或独立真值时，不能据此宣称绝对定位精度。

<a id="saved-map"></a>

## 9. 查看三维点云地图与二维栅格

### 9.1 打开已经保存的三维融合地图

```bash
python3 scripts/view_map "$OUT/native/five_state_hz5_filter_on/export"
python3 scripts/view_map "$WC_DATA/maps/实际保存目录"
```

选择其中一个实际存在的输入运行。查看器发布保存地图并打开 RViz，不连接雷达、电机或 IMU。

也支持可识别的 RTAB-Map `.db` 路径；需要导出时使用副本，不修改原数据库。不要把任意单帧 `.npz` 或 `prepared.json` 当成 `view_map` 输入。

仅有 RTAB-Map `.db` 时，查看器会先在临时目录导出副本；原生导出子进程最多等待 900 秒（15 分钟），复制、校验和显示时间另计。超时会报告日志末尾并退出，原库保留，临时副本清理后可重试。已有 PLY/YAML/PGM 导出文件时直接读取；`--check` 不执行导出。

### 9.2 查看参数

| 参数 | 默认/可选值 | 说明 |
|---|---|---|
| 位置参数 `PATH` | 必填 | 保存地图目录、export目录或可识别数据库；省略会报错，没有路径选择器 |
| `--check` | 关闭 | 只读检查，不开窗口、不导出副本 |
| `--renderer software/system` | `software` | 软件回退或使用系统OpenGL环境；system不保证一定用GPU |
| `--max-view-points N` | `1000000`，范围 `1000..2000000` | 只限制查看点数，不删除原地图点 |

```bash
python3 scripts/view_map "$OUT/native/five_state_hz5_filter_on/export" --check
python3 scripts/view_map "$OUT/native/five_state_hz5_filter_on/export" \
  --renderer system --max-view-points 1500000
```

若 system 出现 GLX/OpenGL 错误，回到 `--renderer software`。不同后端只用于显示性能对照，图像质量验收要看实际画面。

### 9.3 导出文件的含义

| 文件 | 含义 |
|---|---|
| `map_3d_cloud.ply` | 三维累计点云 |
| `map_3d.pgm` + `map_3d.yaml` | 二维占据栅格及分辨率/原点信息；名称含3d不改变PGM的二维属性 |
| `map_3d_poses.txt` | 导出节点位姿 |
| `native_export_copy.db` 或对应 `rtabmap.db` | 原生数据库/导出副本；以目录实际文件为准 |
| `export.log`、质量/比较报告 | 导出过程与结果检查 |

RViz 中选顶部视角，并按需关闭 `Saved 3D map`、保留 `Saved 2D map` 可看二维栅格。5 cm 栅格表示单元大小，不是5 cm定位精度。看单帧原始物体形状用第 6、10 章，查看整条路线的累计地图用本章。

一次只开一个本项目保存地图查看器，它使用独立domain84和查看锁；先关上一个窗口，再查看另一组。

<a id="alignment"></a>
## 10. 左右点云选点、配准与网页叠加

<a id="amplitude-alignment"></a>
### 10.0 单页双雷达配准工作台

现在在一个明亮主题网页内，依次完成：确认输入与选同名点 → 只计算人工初值 → 传入下方并调整 / ICP → 比较结果和明确保存。无需先保存才能进入 ICP，也不需要打开另一个页面。

```bash
cd "$PROJECT_ROOT"
PREPARED="$WC_DATA/reports/calibration/实际准备结果/prepared.json"
bash scripts/pick_lidar_points.sh \
  --input "$PREPARED" \
  --scene-index 1 --port 8769 --duration 7200 --no-browser
```

在 Orin 浏览器打开 `http://127.0.0.1:8769/`。如果本次服务已在运行，直接打开网址即可，不重复运行命令。`--duration` 为网页服务时长，不是录制时长。`--scene-index 1` 指输入中的第二组帧对；只有一组时使用 `0`。

1. 在两侧幅度图或三维点云选同一物理点，至少 3 对非共线；建议 6～10 对分布在不同深度、高度。
2. 点击“只计算，不保存”，查看逐点残差、中文可用性结论和初值叠加。
3. 点击“用此初值继续调整”，未保存矩阵也会以完整精度传入同页下方。微调后点“只计算 ICP”，比较前后和各视角。
4. 需要留存才点“保存本次 ICP 结果”：保存刚查看的那次计算，不重新跑 ICP。修改姿态或选点会使旧结果过期，需要重新计算。旧候选仅通过“恢复已保存候选”明确载入。

候选目录仍为 数据根 `data/calibration/manual_points/` 与 `data/calibration/alignment_candidates/`。计算不生成结果文件；保存不启用正式外参，实时建图和离线融合配置不变。原始数据、坐标、算法门限也不因 UI 改造改变。左右默认共用米制显示比例，可切各自适应；原生 Z 不是离地高度。

完整说明见 [幅度图辅助配准说明](amplitude_alignment.md)。旧 `/alignment` 地址现在也打开同一个工作台；较早已经运行的服务不会自动升级，使用本节新端口。

### 10.1 打开指定的离线配准输入

`PREPARED` 必须指向已经准备好的输入 JSON，场景号由其中的 `scenes` 决定，不能把某个编号固定理解为 raw 或 filtered。

```bash
PREPARED="$WC_DATA/reports/calibration/实际准备结果/prepared.json"
bash scripts/align_lidar_clouds.sh \
  --input "$PREPARED" --scene-index 0 --duration 7200 --no-browser
```

在服务所在机器浏览器打开 `http://127.0.0.1:8767/alignment`。需要第二个独立服务时显式选择未占用端口，例如 `--port 8768`；默认仅绑定本机回环地址。

这是冻结输入，不是实时画面。不传 `--input` 时会读取 `.phase1_runtime/state/point_picker_input.json` 中的用户选择；首次使用或切换实验时建议总是明确传入 `--input`。

### 10.2 网页怎样操作

1. 左云青色固定，右云橙色可调整。切换前视/顶视/侧视检查；旋转观察角度改变屏幕左右，不会改变点云坐标。
2. “仅调整视角”：拖动旋转，Shift+拖动平移视点，滚轮缩放。
3. “拖动右云”：在当前视图平面平移右云；旋转使用 Roll/Pitch/Yaw，单位度，平移单位米。
4. 修改数值后应用；保存手动候选，或从当前姿态运行ICP细化。查看匹配残差、退化和拒绝原因。
5. 新结果保存到新的候选目录，不自动写正式外参。页面仅看起来重叠不能代替独立几何验证。

变换约定：`p_left = R * p_right + t`，`R=Rz(yaw)Ry(pitch)Rx(roll)`。

在输入说明中确认是否已经应用机械初值。若已包含安装平移，不再手动重复加一次。通过“恢复已保存候选”明确选择与当前输入/场景匹配的候选；在来源信息中确认采用的是机械、人工还是已保存初值。

### 10.3 人工选择左右对应点

```bash
bash scripts/pick_lidar_points.sh \
  --input "$PREPARED" \
  --scene-index 0 --duration 7200 --no-browser
```

在 Orin 打开 `http://127.0.0.1:8766/`。先切换到“选点”模式，再依次选择左右云中同一实物角点；默认“浏览”模式点击不会添加对应点。至少3对不共线点，分散选择更多可靠角点可改善几何约束。不要因为两个屏幕位置相近就配成同名点。可撤销、清空、仅保存选点，或求解并保存初值。

### 10.4 选点/配准共用参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--input PATH` | 默认指针指向的 prepared | 使用已准备JSON，不能直接传任意PLY/PCD/NPZ |
| `--scene-index N` | `0` | 从0开始的场景编号，含义由该prepared决定 |
| `--port N` | 选点8766、配准8767 | 指定未占用端口；网址同步改端口 |
| `--duration SECONDS` | `3600`，范围1～43200 | 网页服务时长，不是录制时长 |
| `--no-browser` | 关闭 | 不自动打开浏览器，手动访问打印的网址 |
| `--alignment` | 配准脚本自动设置 | 直接用align脚本即可；底层共用模块的页面模式 |
| `--help` | — | 只看帮助 |

关闭浏览器标签页不停止服务；终端Ctrl+C停止。刷新页面会丢掉尚未保存的浏览器内调整。已保存结果仍在：

```text
$WC_DATA/data/calibration/manual_points/<UTC时间_随机串>/
  selected_pairs.json
  result.json / manifest.json    # 求解并保存后

$WC_DATA/data/calibration/alignment_candidates/<UTC时间_随机串>/
  manifest.json
  result.json
  correspondences.json
```

<a id="static-pair"></a>
## 11. 采集新的静态配准场景

### 11.1 一次采集并准备网页输入

```bash
cd "$PROJECT_ROOT"
bash scripts/record_lidar_points.sh room01
```

- `room01` 是场景名，1～20个英文字母/数字/下划线/短横线；省略默认 `manual`。
- 运行约3～4分钟：双开A、左单开、右单开、双开B四阶段。每阶段观察25秒、预热5秒，驱动有65秒上限；另有启动/停止时间。
- 保持轮椅和场景静止；这是静态对照，不适合边推边走。只采雷达，不启动电机、IMU或轮反馈。
- 设备使用 `preserve_current`，不自动把新的上位机参数写入设备。
- 包装脚本没有 `--duration` 参数；不要把capture的参数挪过来。
- 完成后打印prepared路径和下一步选点命令，并更新默认输入指针，不自动打开浏览器。后续仍建议复制打印的显式 `--input`。

| 产物 | 数据根路径 |
|---|---|
| 四阶段数据 | `reports/lidar_stability/room01_<UTC时间>/` |
| 每阶段点云/索引 | `dual_A/`、`left_single/`、`right_single/`、`dual_B/` 下的左右NPZ、frames JSON、capture审计 |
| 网页输入 | `reports/calibration/room01_<UTC时间>/prepared.json`；重复准备可能在新prepare子目录 |
| 操作日志 | `reports/point_picker_operations/<时间_随机串>/` |

中途按Ctrl+C会收尾，未完成四阶段验证时不会替换默认prepared指针，已取得数据保留。不要把不同时刻的四阶段云描述成硬件同步采集。

### 11.2 已有四阶段数据只重新准备，不再次采集

```bash
bash scripts/record_lidar_points.sh \
  --prepare-only reports/lidar_stability/room01_YYYYMMDDTHHMMSSZ
```

把路径替换为真实完成的study；必须含四阶段审计和正常停止证据。不能把任意 `capture` 会话、单个NPZ或柜子/椅子核验目录直接作为该study参数。

### 11.3 准备已有组织化点云输入

对于已经包含 `capture.json`、左右 NPZ 和帧元数据的组织化采集目录，可离线生成幅度图选点输入；不会启动设备：

```bash
bash scripts/prepare_lidar_alignment.sh \
  --capture-root "$WC_DATA/reports/实际组织化采集目录" \
  --output "$WC_DATA/reports/calibration/alignment_$(date +%Y%m%d_%H%M%S)" \
  --pair-count 3 --max-pair-dt-ms 100
```

`--pair-count` 为 1～16；也可用 `--left-frame-index N` 指定一个从 0 开始的左 raw 帧，配对最近右帧。`--max-pair-dt-ms` 是主机到达时间差上限，不是硬件同步精度。输出目录必须是新的；以打印的 `prepared.json` 路径启动网页。

方向检查报告和原始 bag 应一起保留。静态方向核对通过不表示左右雷达已经精确配准。

<a id="concepts"></a>
## 12. 频率、raw/filtered、外参与坐标轴

### 12.1 五种不同的“Hz”

| 位置 | 如何理解 | 修改位置 |
|---|---|---|
| 雷达实际输出 | 实际每秒接收帧数须看本次读回配置与源统计 | 设备参数，不能靠RViz或compare增加 |
| 录制接收/落盘 | 记录来源实际到达；raw和filtered各一份不代表双倍物理曝光 | source统计、records、bag对账 |
| 前端点云消费 | 默认对照5Hz，可0取消该级限频 | compare `--input-rate-hz`；实时map用配置 `input_rate_hz` |
| 原生地图输入 | compare默认1Hz，从共同有效时刻再取样 | `--native-rate-hz`，详见8.3 |
| 显示刷新 | capture预览约3Hz；独立RViz配置可能20FPS | 查看器/预览；不等于设备采样率 |

实际帧率和每帧点数是两个指标。例如 9600 点/帧、10 帧/秒与“显示每秒3次”可以同时成立。低消费频率可能减少参与建图的扫描次数，却不自动说明录包缺点；更高频率也不自动改善错误外参、时间或噪声。

### 12.2 三层处理

1. 设备成像/曝光/HDR/深度处理。
2. SDK投影成XYZ，再产生主机滤波前 `raw` 与主机滤波后 `filtered`。
3. 应用层的距离、邻域、高度等几何过滤，compare由 `--filter` 控制。

所以 `--cloud raw --filter on` 合法：选择SDK主机滤波前XYZ，再做应用几何筛选。`--filter off` 不会撤销设备处理，也不会将 `filtered` 还原成光学原始测量。录制保存的是这些接口给出的数据，不是原始UDP网络包。

### 12.3 坐标约定

- Orin发布采用FLU：X向前、Y向左、Z向上，单位米。
- 当前SDK CAR数据在驱动边界执行 `(X,Y,Z)_Orin=(X,-Y,Z)_SDK`，raw和filtered都只做一次。这是数据表示转换，不是盒子安装外参。
- 安装刚体外参必须保持正常旋转，不能通过加入镜像矩阵凑重合。
- `T_A_B` 表示把B中的点转换到A；网页 `T_left_right` 把右雷达点变到左雷达坐标系。
- 点云轴向由驱动契约与当前输入核对；单次静态方向检查不等于精确配准或动态精度验收。
- `--mechanical-initial` 是利用现有机械关系做离线实验，不表示传感器数据原点、姿态和动态精度已经标定。

<a id="calibration"></a>
## 13. IMU 零偏与参数文件

### 13.1 参数在哪里改

公开配置是可分发模板；现场身份和完整现场参数放在 Git 忽略的本机覆盖文件。以下文件不存在本机覆盖时才回到公开模板；本机文件存在但无效时直接报错，不悄悄忽略。

| 配置入口 | 内容 | 生效时机 |
|---|---|---|
| `config/storage.local.json` | 覆盖 `storage.json`，选择数据根及可移动存储身份 | 新命令，运行中继续检查目标 |
| `config/device_bindings.local.json` | IMU 等设备绑定，按专用加载器读取 | 下一次设备会话 |
| `config/local/project/mapping_live.json` | 运动模型、点云、过滤、输入率、零偏、相机与渲染 | 下一次会话 |
| `config/local/project/hardware_setup.json` | 外参、安装关系、轮几何及来源证据 | 下一次会话或显式离线覆盖 |
| `config/local/project/wheel_feedback_current.json` | 轮串口身份、反馈寄存器、交互策略 | 下一次轮接口会话 |
| `config/local/project/cameras.json` | 相机身份、角色、格式和预览 | 下一次相机会话 |
| `config/local/project/live_unvalidated.json`、`mapping_3d_diagnostic.json`、`comparison_20260917_handpush/mapping.json`、`wheel_manual_stop_evidence.template.json` | 对应完整现场配置；末项为人工停止证据模板 | 对应新会话/工具 |
| `config/local/lidar/` | 完整左右驱动 YAML 与四份 xtcfg；不与公开目录混搭 | 下一次驱动会话；原生程序改动仍需重建 |
| `config/ultrasonic_capture.json` | 适配器拓扑、地址和协议 | 下一次全源采集 |
| `config/local/imu_timing/` | 本机 IMU 时序配置及相邻证据，由专用加载器读取 | 下一次采集；旧录包沿用冻结配置 |
| `config/calibration/sources/` | 原始安装/测量资料，按原件与哈希引用 | 随会话冻结 |

八份 `config/local/project/` 文件是各自完整覆盖，不是零散字段合并。明确传入的外部配置、已冻结录包仍按其来源读取，不能用今天的本机身份重新解释旧证据。雷达本机目录包含 `left.yaml`、`right.yaml`、`left-2026-09-11.xtcfg`、`right-2026-09-11.xtcfg`、`left.xtcfg`、`right.xtcfg`。

不在运行中热换外参或估计器。源码与本机配置变化不改写已有档案；离线新参数通过显式覆盖和新输出目录记录。

### 13.2 零偏的两个入口

**旧候选确认入口**用于已有建图工作会话的候选：

```bash
python3 scripts/confirm_gyro_bias \
  "$WC_DATA/reports/maps/实际会话名" \
  --output "$WC_DATA/data/calibration/gyro_bias_new.json"
```

工具展示候选区间，只有实际确认对应时间物理静止才输入 `CONFIRM`。已有输出不覆盖；不要把程序判定零轮速直接当作用户观察到静止。

**独立录包三阶段入口**使用原生轴数据，预热/估计/独立验证默认各10秒；需要实际覆盖全30秒的IMU/轮反馈与用户静止证据。请求60秒静止采集通常便于留出启动余量：

```bash
python3 -m wc_runtime.mapping_bias_confirm \
  --capture-dataset "$DATASET" \
  --stationary-evidence "$WC_DATA/reports/实际静止证据.json" \
  --output "$WC_DATA/data/calibration/gyro_bias_new.json"
```

该命令前先执行第2.1节ROS/PYTHONPATH准备。静止证据至少包含 `physically_stationary`、`source`、`evidence_id`、`start_stamp_ns`、`end_stamp_ns`；时间必须来自录包对应事件时间并覆盖观察区间，不能填写执行命令的当前时间。具体提取方法见 `docs/operations/capture.md` 的零偏章节；输入路径使用已配置数据根中的实际文件。

成功得到零偏JSON和相邻证据文件后，离线compare可显式 `--gyro-bias 文件路径`；实时下一会话通过 `mapping_live.json` 的 `gyro_bias_config` 使用。检查生效配置该字段及证据；字段为空时没有应用确认零偏。

`scripts/confirm_gyro_bias` 也是同一模块的包装入口，可用其 `--capture-dataset` 代替上述 `python3 -m ...`。其完整公开参数如下：

| 参数 | 用途 |
|---|---|
| 可选位置参数 `SESSION_DIR` | 已有主建图候选模式，需要prior/status和runtime_config |
| `--capture-dataset PATH` | 原生录包三阶段模式，与calibration-jsonl互斥 |
| `--calibration-jsonl PATH` | 已按要求整理的输入事件，替代capture-dataset |
| `--stationary-evidence PATH` | 三阶段模式必需的物理静止声明 |
| `--output PATH` | 新零偏文件；父目录已存在，不覆盖；没有output的候选查询不保存 |
| `--confirm-stationary` | 候选模式非交互确认，只有已人工确认物理静止才使用 |
| `--candidate-token TOKEN` | 非交互确认时绑定刚查看的候选，不能自行编造 |

三阶段流程从输入首段开始，不会自动在一整圈移动数据中寻找静止段。默认各10秒，当前密度筛选至少40Hz，因此各阶段至少400样本，最大缺段0.1秒；不通过时不保存，不把失败当零偏0。这些是实验筛选条件，不是绝对精度认证。

<a id="diagnostics"></a>
## 14. 状态、故障定位与常见报错

### 14.1 可选只读检查

```bash
cd "$PROJECT_ROOT"
python3 scripts/wc_phase1 doctor --profile mapping_core
python3 scripts/map all --check-config
python3 scripts/wc_phase1 status
```

`doctor` 不带会话时检查环境/身份/配置，带 `--session-root` 看该会话健康，再加 `--verify-archive` 复核实际文件与消息。全局doctor默认profile为all_sensors；只用核心来源时明确写 `--profile mapping_core`，避免把不使用的超声波缺失误解为核心录制不可用。这些不是每次采集必须先手动执行的步骤。

doctor根层 `free_bytes` 当前检查工程所在内置盘；数据根余量看 `df` 或capture目标容量字段。`--verify-archive` 必须配合 `--session-root`。

### 14.2 空间、挂载与进度

```bash
df -h / "$WC_DATA" /dev/shm
findmnt --target "$WC_DATA"
du -h --max-depth=1 "$WC_DATA/data"
du -h --max-depth=1 "$WC_DATA/reports"

# 正在使用memory暂存的会话；SESSION须是实际运行名称
python3 -m json.tool "/dev/shm/wc_capture_$(id -u)/$SESSION/progress.json"

# 转存后的会话
python3 -m json.tool "$DATASET/progress.json"
python3 -m json.tool "$DATASET/capture_manifest.json"
```

如果文件尚不存在，先根据终端当前阶段判断是在启动前、内存暂存中还是最终目录；不要因此创建空progress/manifest来“修好”状态。运行保留约2GiB盘/tmpfs余量，内存还有独立保护；提高duration不会绕过资源边界。

### 14.3 现象到处理位置

| 现象 | 优先检查 | 处理方向 |
|---|---|---|
| 只打印JSON，没有界面 | 是否用了 `--dry-run`；是否缺 `--preview/--manual-drive` | dry-run不录制；需要窗口就用正式命令及相应选项 |
| 已PREPARING，暂未出窗口 | 源码快照/recorder准备、轮反馈是否就绪 | 看阶段和本会话logs，不重复启动第二份 |
| 长时间WAITING_FOR_WHEEL | 控制器电源、USB身份、事务超时和串口占用 | USB枚举不等于控制器有响应 |
| WAITING_FOR_ALL_SOURCES | selected_sources及各路新鲜度 | 定位缺的来源，不把超时填零或静默删源 |
| 点云录到但RViz空白 | frame/topic、source age、Displays、GLX/OpenGL日志 | 区分源没数据和显示失败，独立核对bag计数 |
| 画面不更新但程序仍运行 | FRESH/STALE、序号、队列和源退出 | 旧画面留存不证明持续采集 |
| 端口8766/8767已占用 | 旧网页服务是否仍运行 | 用原终端Ctrl+C，或显式换未占用端口；不盲杀未知进程 |
| 外参能力缺失 | 对应transform和配置来源 | 预览/采集可独立；现有数据可离线mechanical-initial实验 |
| compare拒绝PARTIAL | 原清单issues及实际记录范围 | 明确接受后加allow-partial，原状态保持 |
| compare没有PLY | 是否加native-map；native子目录状态 | 前端比较完成不代表原生地图阶段运行 |
| 提高input-rate地图变化小 | native-rate、共同时刻、原帧率 | 检查第8.3节的两级取样，不只改一个数字 |
| FileNotFound且路径含旧reports/data | 是项目入口还是普通shell | 系统命令改用数据根绝对路径 |
| exFAT + manual-drive + disk拒绝 | capture暂存选项 | 恢复默认memory；数据根仍是最终保存位置 |
| 输出目录已存在 | SESSION/OUT/DEST是否复用 | 使用新名字，避免覆盖旧证据 |
| 配准网页仍是旧场景 | input路径、scene-index、浏览器缓存姿态 | 明确prepared来源并刷新；网页不是实时流 |
| 改代码后运行行为没变 | src/实际import/安装二进制哈希 | 原生代码需构建，新会话才加载新参数 |

<a id="advanced"></a>
## 15. 研发用底层命令与 ROS 回放

以下入口用于明确的单模块调试，不能与正在运行的capture/map争用设备。普通研究录制仍使用第3章，普通离线融合仍使用第8章。

### 15.1 受管理后台会话：status / stop

```bash
python3 scripts/wc_phase1 status
python3 scripts/wc_phase1 status --session "$SESSION"
python3 scripts/wc_phase1 stop --session "$SESSION"
```

`status` 可省session，可能列出历史登记；`stop` 必填session，只停止身份匹配的该会话进程。它主要对应drivers/record/encoder/cameras/wheel/replay/底层map等后台入口，不保证前台capture也由同一登记机制管理。**capture仍使用它自己的Ctrl+C/人工窗口收尾。** `stop` 是进程管理，不是底盘急停指令。

### 15.2 只启动雷达源：drivers

```bash
SESSION="radar_probe_$(date +%Y%m%d_%H%M%S)"
python3 scripts/wc_phase1 drivers \
  --session "$SESSION" --mode dual --duration 30 --probe
```

这个probe会连接真实设备读取身份/配置，但不开始测量。它不同于纯只读软件诊断的 `capture --dry-run`。去掉probe则实际启动点云源；本命令不录bag。

| 参数 | 默认/范围 | 作用 |
|---|---|---|
| `--session ID` | 必填 | 新后台会话 |
| `--mode` | dual；single_left/single_right | 选雷达 |
| `--duration N` | 30；1～3600秒 | 驱动运行上限 |
| `--device-config-policy` | preserve_current / apply_xtcfg | 默认保留设备当前设置；apply_xtcfg明确写入支持的成像配置，日常不加 |
| `--probe` | 关闭 | 只连接读取身份/配置，不开始测量 |

### 15.3 底层雷达/IMU bag：record

```bash
SESSION="lidar_only_$(date +%Y%m%d_%H%M%S)"
python3 scripts/wc_phase1 record \
  --session "$SESSION" --mode dual --duration 30 --lidar-only
```

此例只录两雷达，写 `$WC_DATA/data/bags/$SESSION/`。完整参数：

| 参数 | 默认/范围 | 作用 |
|---|---|---|
| `--session`、`--mode`、`--duration`、`--device-config-policy` | 同drivers，duration默认30 | 来源与时长 |
| `--imu-poll-period-ms` | 10.0；可选2.5/5.0/10.0 | 主机串口轮询间隔，不是IMU物理采样率 |
| `--lidar-only` | 关闭，仅mode=dual允许 | 不启动H30、不录IMU话题 |

不带lidar-only默认有雷达和H30。它虽然订阅部分轮相关话题，**不会自己启动轮串口读取者**，也不含相机/超声波，所以不能代替mapping_core完整研究采集。SQLite分包大小上限536870912字节，保留整个bag目录。

### 15.4 轮反馈两个入口

真实FC03读取事务，不发运动命令：

```bash
SESSION="wheel_read_$(date +%Y%m%d_%H%M%S)"
python3 scripts/wc_phase1 encoder \
  --session "$SESSION" --duration 30 --rate-hz 10 --allow-read-queries
```

| encoder参数 | 默认/范围 |
|---|---|
| `--session` | 必填 |
| `--config` | config/wheel_feedback_current.json |
| `--duration` | 30秒；1～300 |
| `--rate-hz` | 10；10/20/50，目标请求率不代表实际成功率 |
| `--allow-read-queries` | 必须显式给出，允许固定读事务 |
| `--preview-history-calibration` | 可选，历史轮尺度生成预览，不表示实物标定通过 |

输出 `$WC_DATA/data/wheel_feedback/<ID>.transactions.jsonl`，占用唯一轮串口读取权。

`wheel` 只解码已有反馈话题，不开串口，也不发送查询：

```text
python3 scripts/wc_phase1 wheel --session ID
  [--config config/wheel_unvalidated.json]
  [--duration 0] [--use-sim-time]
```

duration为非负整数，0不设上限；use-sim-time使用回放clock。输出 `data/wheel_feedback/<ID>.jsonl`。没有上游来源时它不会自动获取底盘数据。

### 15.5 相机预览与发布

最简单的四路相机窗口：

```bash
bash scripts/view_cameras.sh monitor_320
```

只接受一个可选profile位置参数，默认monitor_320，最长3600秒；关RViz会停所属相机。**此脚本没有实现 `--help`，不要为查询帮助而给它传这个参数。** 它不归档全部图像；要录画面用capture的含相机profile。

仅启动四相机发布，不打开窗口：

```text
python3 scripts/wc_phase1 cameras --session ID
  [--config config/cameras.json]
  [--profile monitor_320|detail_640|detail_640_10fps]
  [--duration 3600]
```

duration范围1～43200秒，profile默认monitor_320。monitor_320请求320×240/30fps，detail_640请求640×480/30fps，detail_640_10fps请求640×480/10fps，预览发布均约8Hz；实际规格和成功率看设备读回。

仅开已有来源的RViz：

```text
python3 scripts/wc_phase1 rviz --session ID [--view 2d|3d|left|right|cameras]
```

默认3d，不启动来源，session不自动路由历史数据，见第6章。

### 15.6 旧式bag回放：replay

```bash
python3 scripts/wc_phase1 replay \
  --session "replay_$(date +%Y%m%d_%H%M%S)" \
  --bag "$WC_DATA/data/bags/实际底层录制会话名"
```

两个参数都必填，没有包装层 `--rate` / `--loop`。只接受 `data/bags/` 根下的bag，播放允许的测量话题和clock，排除控制、历史TF和部分派生话题，不连接硬件。播完结束，也可用status/stop管理。

新capture的 `data/experiments/<session>/bag` 不属于该旧式入口允许根；反复融合使用compare，不要搬包绕过入口契约。回放不要与真实采集共享活跃来源话题，不使用未经审查的整包 `ros2 bag play -a`。

### 15.7 底层处理器：wc_phase1 map

```text
python3 scripts/wc_phase1 map --session-config PATH
  [--duration 0] [--with-synthetic-source] [--icp-only]
```

`--session-config`必填，要求该处理链规定的完整会话配置，不是随意指向 `mapping_live.json`。仅接受dual；真实来源还要求calibration和quality_profile状态均为VALIDATED，不能换这个入口绕过缺失外参。duration非负，默认0；with-synthetic-source用于合成源；icp-only只做诊断处理不运行完整图后端。它与 `python3 scripts/map all --mapping true` 不同，普通实时入口用后者。

### 15.8 单雷达ICP实验链

```text
bash scripts/map_single_lidar.sh start --session ID
  [--side left|right] [--duration 300] [--port 8770]
bash scripts/map_single_lidar.sh inspect --session ID
bash scripts/map_single_lidar.sh stop --session ID
bash scripts/map_single_lidar.sh preview --session ID [--port 8770]
bash scripts/map_single_lidar.sh export --session ID
```

start默认右侧、300秒、8770端口；时长40～1800秒，端口1024～65535。启动所选雷达、ICP、RTAB-Map、bag和网页，不启动轮/IMU/相机/运动控制；输出 `reports/single_mapping/<ID>/`，网页 `http://127.0.0.1:8770/`。

这是独立的历史实验链，存在跟踪/退出约束，不是默认轮速+IMU建图。`STARTED_NOT_YET_VERIFIED`只是启动，不代表地图有效。inspect查状态、stop收尾、export要求成功停止及可核验数据。preview只看已停止会话的监视快照，会返回独立viewer_session；必要时用wc_phase1 stop停止该viewer，单关浏览器不等于退出。

### 15.9 版本地图包：save/load/verify/inspect/list

```text
python3 scripts/wc_phase1 save --map-id ID --version VERSION --snapshot DIR
python3 scripts/wc_phase1 load --map-id ID --version VERSION
python3 scripts/wc_phase1 verify --map-id ID --version VERSION
python3 scripts/wc_phase1 inspect --map-id ID --version VERSION
python3 scripts/wc_phase1 list --map-id ID
```

map-id/version为1～64位ASCII标识，已存版本不覆盖。snapshot必须是完整冻结图快照，含snapshot、observations、graph、trajectory、calibration、config等JSON和 `slam/rtabmap.db`；不能传单个PLY/DB或独立capture目录。snapshot用数据根绝对路径。输出 `$WC_DATA/maps/<ID>/<VERSION>/`。

load校验后返回 `LOADED_VIEW_ONLY` JSON，不打开RViz、不定位或导航；看图用view_map。verify和inspect当前使用相同完整核验；list列指定map-id版本，不是列所有地图。没有 `wc_phase1 version` 子命令。

### 15.10 离线目标点注释：goals

```text
python3 scripts/wc_phase1 goals --map-id ID --version VERSION list
python3 scripts/wc_phase1 goals --map-id ID --version VERSION set
  --name NAME --position X Y Z --orientation-xyzw QX QY QZ QW
python3 scripts/wc_phase1 goals --map-id ID --version VERSION review
  --name NAME --state UNVERIFIED|VERIFIED
python3 scripts/wc_phase1 goals --map-id ID --version NEW_VERSION migrate
  --from-version OLD_VERSION
```

名称最多128字符；位置是地图frame中的有限XYZ；姿态是单位四元数，顺序xyzw。set写同名新注释并设UNVERIFIED；review记录人工状态；migrate要求两个版本不同且frame_id相同，迁入后重置为UNVERIFIED。文件位于 `maps/.annotations/<ID>/<VERSION>/goals.json`，不改原地图。**这些是注释命令，不会下发导航或驾驶。**

### 15.11 通用离线标定：calibrate

| 完整子命令 | 作用 | 参数 |
|---|---|---|
| `wc_phase1 calibrate calibrate` | 多尺度点到面ICP | input/output-root/version必填；可选quality-profile、initial-result |
| `wc_phase1 calibrate validate` | 独立场景验证固定变换 | 三共同必填，另必填result，可选quality-profile |
| `wc_phase1 calibrate ground` | 地平面估计 | 三共同参数 |
| `wc_phase1 calibrate clock` | 到达时间关系拟合 | 三共同参数；不代表硬件同步 |
| `wc_phase1 calibrate prepare-files` | 明确配对的ASCII PCD/PLY整理 | 三共同参数，input是描述JSON |
| `wc_phase1 calibrate prepare-bag` | 从SourceFrame窗口准备 | 三共同参数 |
| `wc_phase1 calibrate manual-initial` | 对应点求右到左刚体初值 | 三共同参数 |
| `wc_phase1 calibrate imu-gravity` | 输入重力方向估计倾斜 | 三共同参数 |
| `wc_phase1 calibrate imu-gravity-bag` | bag IMU倾斜估计 | 三共同参数 |
| `wc_phase1 calibrate suggest-static` | 提示几何稳定窗口 | 三共同参数，不能独立证明静止 |
| `wc_phase1 calibrate report` | 读取已有结果 | `--result PATH` |
| `wc_phase1 calibrate list` | 列标定结果 | `--output-root DIR` |

“三共同参数”的完整写法是 `--input JSON --output-root DIR --version NAME`；version为1～80位ASCII标识，结果通常在 `DIR/NAME/result.json`，不覆盖。输入/输出用数据根绝对路径。`--quality-profile PATH`是判据文件，`--initial-result PATH`是初值结果。prepared/UNVALIDATED和返回码必须一起读，退出0不等于正式外参验证通过。

日常人工选点/拖动配准优先第10章；特殊PCD/PLY输入契约看 `docs/operations/offline_cloud_alignment.md`、`src/wc_calibration/cli.py` 对应子命令，不把任意PLY路径直接当input描述JSON。

<a id="maintenance"></a>
## 16. 构建、测试、回退与文件整理

### 16.1 构建与测试

先完成[部署依赖](../getting_started/deployment.md)，再按需构建或运行回归：

```bash
cd "$PROJECT_ROOT"
python3 scripts/wc_phase1 build
python3 scripts/wc_phase1 test
```

构建修改 `build/main` 与 `install/main`，日志写入 `$WC_DATA/dev_archive/<UTC任务名>/validation/colcon/`。测试入口将产物保存在它打印的独立任务目录中；两者都不是每次录制前的必需步骤。软件回归通过不表示真实硬件或动态精度已验收。

| 专项入口 | 用途/输出 |
|---|---|
| `bash scripts/build_verify.sh` | 构建、原生测试及 Python 回归；本次任务 `validation/` 下保存构建日志 |
| `bash scripts/build_tools_and_driver.sh` | 驱动/测试工具专项构建，含独立 `install/test_tools` |
| `python3 scripts/run_software_tests.py -- -q` | 软件回归；`--` 后传递 pytest 参数，生成独立任务收据 |
| `bash scripts/prepare_opencv45.sh` | Ubuntu 22.04 固定 OpenCV 开发/运行包下载并解包到 `SDKs/ubuntu_opencv45`，不安装系统包；不负责取得 XT SDK 或 legacy TBB |
| `python3 scripts/collect_delivery_evidence.py` | 在新任务 `reports/` 中生成交付清单；扫描证据，不停止进程 |
| `python3 scripts/check_layout.py` | 核对目录规则与维护文档 |
| `bash scripts/discover_remote.sh` | 打印本机设备和软件环境；输出可能含现场身份，仅保存在私有任务资料中 |

上述无参 Bash 构建/依赖脚本不能用 `--help` 当无操作探测。一次只运行一个重型构建；原生驱动或界面修改后需要重建并在新会话加载。可选 CAD 几何依赖缺失导致的 skip 不等于该项几何验证通过。

需要新增任务容器时使用 `python3 scripts/dev_archive 短任务名`，按输出目录内的导航放置快照、报告与验证记录。

### 16.2 精确回退

`DEPLOYMENT_REPORT` 先设为本次部署工具生成的实际报告目录（通常在私有任务归档中），不要把任意运行报告当作回退清单。

```text
python3 scripts/restore_deployment.py --report "$DEPLOYMENT_REPORT"
python3 scripts/restore_deployment.py --report "$DEPLOYMENT_REPORT" --apply
```

第一条只预览；确需恢复且所有相关程序已正常停止后才用第二条。报告必须含部署文件清单及baseline备份；检测到部署后其他修改会拒绝覆盖，不用git reset整树恢复。新增源码及回退过程日志会留在内部 `.phase1_runtime/rollback_retained/`，原始实验不删除。`SOURCE_RESTORED_REBUILD_REQUIRED`表示还需重新构建。

### 16.3 数据查看和整理

```bash
xdg-open "$WC_DATA"
du -h --max-depth=1 "$WC_DATA"
find "$WC_DATA/data/experiments" -mindepth 1 -maxdepth 1 -type d
find "$WC_DATA/data/analysis" -mindepth 1 -maxdepth 1 -type d
```

每个实验单独一个目录，便于文件管理器按会话整理。移动/删除前先结束使用它的采集、比较、查看和网页服务；保留需要重算的原始档案及配置。部分地图只保存了对原始档案的哈希引用，只有PLY在不代表还能重放。`PARTIAL`不自动等于垃圾，不按目录名批量删除。

可移动存储若使用 exFAT，Unix socket 等运行对象留在本机运行区。不要把运行锁或 socket 移到数据盘；开发归档也可能包含私有配置、完整恢复包或原始设备证据，不能直接发布。

### 16.4 测试与历史资料

维护中的测试源码按职责位于 `tests/`，由统一软件测试入口安排产物目录。需要真实硬件的专项脚本可能启动来源或控制接口，运行前按脚本说明核对范围，不对任意 shell 脚本盲加 `--help` 或采集参数。

历史任务、规划、开发报告、旧工作副本和恢复材料从 `$WC_DATA/dev_archive/README.md` 或 `index.jsonl` 查找。生产分析、标定输入、地图和录包仍按各自 manifest 与引用关系保留，不按文件夹年龄批量删除。

<a id="entrypoints"></a>
## 17. 所有顶层脚本索引和帮助命令

### 17.1 当前 `scripts/` 操作入口

| 脚本 | 类型/用途 |
|---|---|
| `wc_phase1` | 主 CLI：采集、比较、校正、诊断及底层子命令 |
| `capture_usb` | 兼容采集入口，使用同一存储策略，参数同 capture |
| `panel` | Bash 入口，启动统一桌面面板 |
| `map` | 预览、实时建图和人工操作 |
| `save_map` | 对已停止主会话重新决定保存 |
| `view_map` | 已存三维点云与二维地图查看 |
| `view_lidars.sh` | 原生 RViz 单/双雷达预览 |
| `view_cameras.sh` | 四相机预览，仅位置 profile 参数，无 help |
| `record_lidar_points.sh` | 四阶段静态采集与网页输入准备，或 prepare-only |
| `record_lidar_comparison.sh` | 四阶段底层采集，两个位置参数：场景名、场景静止说明；无 help |
| `prepare_lidar_alignment.sh` | 已有组织化点云的离线选点输入转换 |
| `pick_lidar_points.sh` | 同名点与统一配准网页 |
| `align_lidar_clouds.sh` | 整云拖动/ICP 离线候选 |
| `confirm_gyro_bias` | 候选确认和独立窗口验证 |
| `map_single_lidar.sh` | 单雷达 ICP 实验 |
| `configure_storage` | 明确配置普通目录或可移动存储 |
| `dev_archive` | 在生效数据根创建开发任务容器 |
| `build_verify.sh` | 工程构建及回归 |
| `build_tools_and_driver.sh` | 驱动/工具专项构建 |
| `prepare_opencv45.sh` | 固定 OpenCV 包下载与本地解包 |
| `run_software_tests.py` | 带产物隔离的软件回归 |
| `check_layout.py` | 工程布局检查 |
| `collect_delivery_evidence.py` | 交付证据扫描 |
| `discover_remote.sh` | 本机环境信息 |
| `restore_deployment.py` | 精确回退预览/执行 |

Python 入口使用 `python3 scripts/名称`，Bash 入口使用 `bash scripts/名称`；`panel` 虽无扩展名也是 Bash。不要求可执行位。

### 17.2 参数帮助

以下入口在解析帮助时退出，不运行相应采集或控制流程。依赖尚未安装时，入口的导入/环境检查仍可能报错：

```bash
python3 scripts/wc_phase1 --help
python3 scripts/wc_phase1 capture --help
python3 scripts/wc_phase1 compare --help
python3 scripts/wc_phase1 refine --help
python3 scripts/wc_phase1 doctor --help
python3 scripts/map --help
python3 scripts/view_map --help
python3 scripts/save_map --help
python3 scripts/confirm_gyro_bias --help
python3 scripts/configure_storage --help
python3 scripts/dev_archive --help
python3 scripts/run_software_tests.py --help
bash scripts/panel --help
bash scripts/view_lidars.sh --help
bash scripts/record_lidar_points.sh --help
bash scripts/prepare_lidar_alignment.sh --help
bash scripts/pick_lidar_points.sh --help
bash scripts/align_lidar_clouds.sh --help
bash scripts/map_single_lidar.sh --help
python3 scripts/wc_phase1 calibrate --help
python3 scripts/wc_phase1 calibrate calibrate --help
python3 scripts/wc_phase1 save --help
python3 scripts/wc_phase1 goals --help
```

顶层帮助列出 `capture`、`compare`、`refine`；各自参数仍以专用子命令帮助为准。

<a id="examples"></a>
## 18. 输入选择与核对依据

仓库不附带某台设备的录包或“最新结果”指针。使用自己的文件选择输入，并保持原始录包名称、原件内容与哈希不变：

| 需要的内容 | 选择位置 | 核对方式 |
|---|---|---|
| 录制档案 | `$WC_DATA/data/experiments/<session>` | capture manifest、状态及 doctor 完整性核验 |
| 算法比较 | `$WC_DATA/data/analysis/<result>` | 输入来源、冻结参数、结果状态与 artifact manifest |
| 原生地图 | `<result>/native/<group>/export` | `view_map --check` 后查看 |
| 选点输入 | `$WC_DATA/reports/calibration/<result>/prepared.json` | 原采集来源、场景编号和输入哈希 |
| 用户保存地图 | `$WC_DATA/maps/<name>` | 保存 manifest 与原始输入引用 |
| 历史开发报告/恢复材料 | `$WC_DATA/dev_archive/` | README、index 与任务清单；仅私有保留 |

本手册修订核对源码、入口和参数，未为文档审阅启动传感器或驾驶。软件测试、录包完整性、真实设备行为和几何精度是不同验收范围，分别读取本次产物与证据。

主要实现依据：`src/wc_runtime/cli.py`、`capture.py`、`capture_destination.py`、`mapping_app.py`、`mapping_compare.py`、`mapping_refine.py`、`compare_native.py`、`map_viewer.py`、`map_save_app.py`、`storage_policy.py`、`project_config.py`、`mapping_bias_confirm.py`、`prepare_picker_input.py`、`calibration_picker.py`、`prepare_organized_picker.py`、`single_mapping.py`，以及 `src/wc_calibration/cli.py`、`src/wc_maps/__main__.py` 和对应 scripts。

进一步说明：[存储](storage.md)、[采集](capture.md)、[录制契约](../reference/recording_contract.md)、[离线配准](offline_cloud_alignment.md)、[接口契约](../reference/interface_contract.md)、[部署](../getting_started/deployment.md)。
