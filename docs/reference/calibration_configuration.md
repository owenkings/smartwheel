# 外参与设备安装配置

本文说明参数在哪里、改哪一项会影响什么、怎样确认修改被使用。现有资料已经写入工程，不需要因为换机或整理目录重新测一遍。本文不修改现有数值，也不要求补测才能进行原始预览和采集。

## 1. 文件组织与已保存内容

以下路径均相对于实际 clone 的代码根目录，不要求固定用户名、主机名或目录。表内项目 JSON 是公开模板；实际设备优先使用 `config/local/project/` 下同名的完整私有副本。面板参数页显示实际选中的路径，私有副本的初始化见[根 README](../../README.md#5-配置本机设备保留公开模板)。来源快照保留原始文件名及哈希。

| 文件 | 用途 | 是否是当前运行入口 |
|---|---|---|
| `config/hardware_setup.json` | schema 2 主配置：装配版本、机械点、数据坐标变换、轮参数、来源索引 | 是，`mapping_live.json` 的 `hardware_setup_config` 指向它 |
| `config/calibration/sources/sensor_installation_20261002.md` | 最初安装资料原文 | 来源文件，保留原文及哈希 |
| `config/calibration/sources/measurement_record_20261005.md` | 后续实测记录原文 | 来源文件，保留原文及哈希 |
| `config/calibration/sources/v7_current_geometry_20261005.json` | 623 mm 跨度、当前水平 / 平行确认、更新后的机械点与轮轴关系 | 当前几何来源确认；主配置中的关联值要与它一致 |
| `config/calibration/legacy_hardware_setup_before_v7.json` | 旧安装历史配置 | `active=false`，不自动覆盖 |
| `config/mapping_live.json` | 当前平地模型、过滤、噪声、读取主硬件配置的路径 | 是，不应在这里重复填写一套机械外参 |
| `config/mapping_3d_diagnostic.json` | 三维诊断模型 | 独立诊断配置，不是平地默认 |
| `config/cameras.json` | 相机角色、USB 绑定、分辨率、预览旋转 | 相机操作配置；预览旋转不是物理外参 |
| `config/wheel_feedback_current.json` | 轮控制器身份及读取协议 | 设备连接配置，不等于轮径或轮距 |

当前主配置中已经保存：

| 内容 | 当前值 / 表达 |
|---|---|
| 左右滑盖前外顶角间距 | 623 mm |
| 轮半径 | `wheel_odometry.wheel_radius_m = 0.185` |
| 轮距 | `wheel_odometry.track_width_m = 0.610` |
| M 系到轮轴系 | `T_axle_M` 的平移 `[0.273, 0, 0.4582] m`，旋转为单位矩阵 |
| IMU 原生方向 | X 向前、Y 向左、Z 向上；当前 `R_M_imu` 为单位矩阵 |
| 左右雷达机械面参考点（M 系） | `[50, 228.75, 133.3] mm`、`[50, -228.75, 133.3] mm` |
| 其他机械点 | 四相机、四双孔位置、IMU 等 11 个主参考点，以及辅助点 |
| 来源核对 | 2026-10-07 只读检查通过：4 个来源文件哈希、11 个机械点及 21 项距离关系 |

这些是本项目当前装配的配置。换一台计算机不会改变机械参数；换雷达、轮椅底盘或安装位置时，应只修改发生变化的部分及相应来源。

## 2. 改哪里

| 想改的内容 | 主配置位置 | 联动及注意事项 |
|---|---|---|
| M 系相对轮轴的安装位置 / 角度 | `data_transforms` 中 `id=axle_from_mounting_M` | 平移 `translation.value_m`，旋转 `rotation.value_matrix`；当前来源确认内的 `current_transforms.T_axle_M` 必须一致 |
| 单雷达真实数据坐标到 M 的外参 | `id=mounting_M_from_lidar_left` 或 `mounting_M_from_lidar_right` | 填入平移、旋转及来源；不是改源点云的 X/Y 符号 |
| IMU 安装轴 | 对应 `child_frame=imu_native` 的变换条目中的旋转 | 原生零偏先扣除，再转到车体；只改变安装旋转不能修复设备身份错误 |
| 轮径 / 轮距 / 反馈比例 | `wheel_odometry` | `wheel_radius_m`、`track_width_m`、`register_to_wheel_rpm` 和左右符号；物理几何变化同步对应来源确认 |
| 左右盒子间距、机械布局 | `mechanical_reference` 及当前几何来源确认 | 两边位置、CAD→M、跨度和派生点要一起一致；只改“623”文字不会自动重算全部点 |
| 相机预览方向 | `config/cameras.json` 中相应角色的预览旋转 | 只影响预览显示，不能当作相机光学坐标外参 |
| 地面 / 障碍高度筛选 | `config/mapping_live.json` 的 `cloud_filter` / `map_profile` | 以当前轮轴及轮半径地面参考解释，不在这里修改雷达外参 |
| 离线会话零偏 / 轮反馈拟合 | `refine` 输出的 `motion_candidate.json` | 绑定特定会话和证据；不自动覆盖正式机械配置，不能直接用于其他录包 |

面板“设备参数”支持按组查看、解锁、校验和保存外参、轮参数、显示、算法及在线运动校正配置。改变机械布局时仍须同步其来源和派生关系；输入一个盒间距不会自动重算全部机械参数。

## 3. 变换方向与自动派生的边界

本工程约定 `T_A_B` 将 B 中的坐标变为 A 中的坐标。车体轴是前、左、上，运行时平移用米；机械资料中的点用毫米。旋转必须是行列式 +1 的正规旋转，左右镜像几何不能用反射矩阵冒充旋转。

变换图会按已填完整边自动求解，例如：

```text
T_axle_lidar_left = T_axle_M × T_M_lidar_left
T_lidar_left_lidar_right = inverse(T_axle_lidar_left) × T_axle_lidar_right
```

也支持直接提供传感器间或传感器到轮轴的关系，不必一定绕过 M；多条路径必须一致。机械 CAD 点到安装点的更新与这个数据坐标变换图是两件事：图可以组合变换，不会自行猜传感器内部测量中心，也不会只依据一个新版跨度自动改所有 CAD 点。

目前雷达真实数据坐标关系仍有 `null` 条目，因此正式能力报告会列出依赖项；现有离线实验通过显式 `--mechanical-initial` 使用已保存机械资料生成初值。它已经用了用户测量的数据，不能把报告中的未知数据原点误解成“之前尺寸全没保存、全没用”。IMU 平移未知也不妨碍当前原生零偏及 gyro-z 流程。

## 4. 修改流程

1. 保留原始测量 Markdown；复制一份当前硬件配置作为候选，并记录本次变化。候选及报告放在已配置数据根的 `reports/`，不要在 home 代码根不断创建测试目录。
2. 修改真正发生变化的条目。若更改机械布局，新增或更新当前结构化确认资料，并同步相关机械点、CAD 变换、轮轴关系与主配置。当前同一 CAD 原文只允许一份激活的当前布局覆盖，不要同时挂两份冲突版本。
3. 在 `source_documents` 中记录来源文件相对路径与 SHA-256；变换条目保留相应来源引用。不要为了通过校验只更新哈希而不检查实际数值一致性。
4. 用下面只读命令检查 JSON、单位、旋转、变换图、来源字节、机械派生一致性及能力依赖。检查可以在不连接传感器时运行。
5. 通过后将候选作为新的主配置，或在离线 `compare --hardware-setup` 中显式指定候选。启动一个新会话做短段验证，核对其 `configuration/` / 运行配置快照和哈希。保留上一版本用于回退。

候选 JSON 可以放在数据根的 `reports/`，但其中 `source_documents[].snapshot_path` **不是相对于候选文件所在目录**。校验及 `compare --hardware-setup` 都按 `--project-root` 的存储策略查找这些路径：`config/...` 指向代码根，`reports/...` 指向数据根。仅移动候选文件不应改变原有来源引用。新候选引用的来源文件也必须存在且哈希匹配。

正式采集目前冻结整个 `config/calibration/`，回放按这个布局寻找来源。将候选转为正式主配置时，新增的维护来源应收录到 `config/calibration/`，相应 `snapshot_path` 使用 `config/calibration/...`，并同步哈希和引用；不要让正式硬件配置永久引用某次测试 `reports/` 目录。这样复制录包到其他机器后，仍能在档案内部核验来源。

如果只是换电脑、仓库完整保留 `config/` 与来源文件，则不用重新填写原数值；设备端口 / 身份和系统部署另按部署文档配置。

## 5. 可直接使用的只读校验命令

先把 `PROJECT` 设为实际 clone 目录，然后在代码根执行。以下路径示例与 README 一致，可替换为包含空格的其他目录：

```bash
PROJECT="$HOME/projects/wheelchair"
cd "$PROJECT"

# 选择当前实际配置，再检查语法。
SETUP="$(PYTHONPATH=src python3 -c 'from pathlib import Path; from wc_runtime.project_config import selected_config_path; print(selected_config_path(Path.cwd(), "hardware_setup.json"))')"
python3 -m json.tool "$SETUP" >/dev/null

# 同时核验结构、来源哈希、派生关系并列出能力。
PYTHONPATH=src python3 -B -m wc_runtime.calibration_geometry \
  --setup "$SETUP" --project-root "$PWD"
```

上述命令不访问硬件、不修改配置。读取退出码及 `source_verification.status`；即使结构和来源检查通过，某个能力仍可能是 BLOCKED，不能据此宣称所有外参已齐备。

例如希望额外严格检查双雷达运动融合所需外参：

```bash
PYTHONPATH=src python3 -B -m wc_runtime.calibration_geometry \
  --setup "$SETUP" --project-root "$PWD" \
  --require wheel_imu_motion_all
```

这个附加检查会对缺少必要数据变换返回非零，并指出具体关系；它不影响原始采集。预览和采集能力也可分别指定 `native_preview`、`native_capture`。

查看来源文件哈希：

```bash
sha256sum config/calibration/sources/v7_current_geometry_20261005.json
```

若检查放在数据目录中的候选，先把逻辑路径解析成实际路径。`calibration_geometry --setup` 自身不会转换 `reports/...`：

```bash
CANDIDATE="$(PYTHONPATH=src python3 -B -m wc_runtime.storage_policy \
  --project-root "$PWD" reports/calibration_candidate/hardware_setup.json)"
PYTHONPATH=src python3 -B -m wc_runtime.calibration_geometry \
  --setup "$CANDIDATE" --project-root "$PWD"
```

上例要求候选已经存在；它只读取，不创建候选、不修改主配置。

检查主配置中所有活动数据变换和轮参数，可用以下纯读取命令快速定位，不输出完整机械资料：

```bash
PYTHONPATH=src python3 - <<'PY'
import json
from pathlib import Path
from wc_runtime.project_config import selected_config_path
s = json.loads(selected_config_path(Path.cwd(), 'hardware_setup.json').read_text(encoding='utf-8'))
for t in s['data_transforms']:
    print(t['id'], t['parent_frame'], '<-', t['child_frame'])
    print('  translation_m:', t['translation']['value_m'])
    print('  rotation:', t['rotation']['value_matrix'])
print('wheel_odometry:', s['wheel_odometry'])
PY
```

## 6. 会话与历史数据

修改主配置影响下一次会话，已经开始的会话采用冻结快照。历史录包中的配置是当时发生过什么的证据，不能用今天的值直接覆盖。

离线算法可以显式选择新候选，输出应放在新的分析目录，记录参数来源、输入哈希以及与原始实验的关联。`compare --hardware-setup /绝对路径/候选.json` 是当前已有外参覆盖入口；`refine` 当前没有同名覆盖参数，不能给它传不存在的选项。`--mechanical-initial` 是明确选择已有机械初值，不是保存永久新外参的编辑命令。

当前窗口查看的是哪个配置，应以该会话的快照和文件哈希为准；仅编辑源码目录中的 JSON 不会让已运行进程热更新。设备 SDK 的 `.xtcfg` 还涉及安装副本与读回参数，它不属于这一份机械外参 JSON，详见驱动配置说明。
