# SmartWheel · 传感器采集与建图

面向轮椅传感器的 ROS 2 工程，提供雷达、惯性数据与轮反馈采集，点云配对、离线融合、建图和结果浏览。程序通过统一面板和命令入口管理任务、停止、保存及完整性检查。

默认维护与发布分支为 `main`。获取源码时使用 `git clone --branch main https://github.com/owenkings/smartwheel.git`；后续功能开发以最新 `main` 为基线。

## 首次部署：从空目录到可运行工程

以下命令在 **Ubuntu 22.04 的 Bash 终端**执行。Orin 使用与型号匹配、提供 Ubuntu 22.04 的 NVIDIA 系统；普通电脑使用 amd64 Ubuntu 22.04。工程使用 ROS 2 Humble、系统 Python 3.10、Qt 5 和 RTAB-Map 0.23。Windows 仅用于资料和源码备份。换机器必须重新构建。

### 1. 准备系统和 ROS 软件源

先完成系统更新，确认 `locale` 输出使用 UTF-8。已经安装 Humble 的机器跳过软件源初始化；新系统按 [ROS Humble 官方安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)准备 Universe 和 ROS 软件源。官方页面不可访问时可阅读其[安装文档源码](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst)。下面给出软件源配置命令；下载失败时停止，不继续安装空文件：

```bash
(
  set -euo pipefail
  sudo apt update
  sudo apt install curl ca-certificates software-properties-common python3
  sudo add-apt-repository universe
  ROS_SOURCE_VERSION="$(curl --fail --silent --show-error --location \
    https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
  test -n "$ROS_SOURCE_VERSION"
  UBUNTU_SERIES="$(. /etc/os-release; printf '%s' "${UBUNTU_CODENAME:-$VERSION_CODENAME}")"
  test "$UBUNTU_SERIES" = jammy
  ROS_SOURCE_DEB="$(mktemp /tmp/smartwheel-ros2-apt-source.XXXXXX.deb)"
  curl --fail --location --output "$ROS_SOURCE_DEB" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_SOURCE_VERSION}/ros2-apt-source_${ROS_SOURCE_VERSION}.${UBUNTU_SERIES}_all.deb"
  sudo dpkg -i "$ROS_SOURCE_DEB"
)
```

安装编译、图形界面、采集、建图和软件测试依赖：

```bash
sudo apt update
sudo apt install \
  git curl ca-certificates build-essential cmake pkg-config \
  python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-scipy python3-yaml python3-serial \
  python3-opencv python3-matplotlib python3-psutil python3-pytest python3-pyqt5 \
  libboost-all-dev libssl-dev libeigen3-dev libpcl-dev libopencv-dev \
  libtbb2 qtbase5-dev psmisc v4l-utils \
  ros-humble-desktop ros-humble-rtabmap-ros \
  ros-humble-robot-localization ros-humble-rosbag2-py \
  ros-humble-rosbag2-storage-default-plugins
source /opt/ros/humble/setup.bash
```

使用系统 `/usr/bin/python3`，避免 Conda 环境混入 ROS 的 Python 包。厂商过滤库要求 `libtbb.so.2`；`libtbb.so.12` 不兼容，不能用软链接替代。软件源包版本变化时按实际错误核查兼容性，不能跳过工程中的 ABI、来源和哈希检查。

### 2. 克隆主分支，设置数据位置

`PROJECT` 是源码目录，可改成自己的位置；该目录必须尚不存在。后文默认在项目根目录运行。

```bash
PROJECT="$HOME/projects/wheelchair"
mkdir -p "$HOME/projects"
git clone --branch main https://github.com/owenkings/smartwheel.git "$PROJECT"
cd "$PROJECT"
git rev-parse HEAD
python3 scripts/configure_storage \
  --backend directory --archive-root "$HOME/wheelchair-data"
```

录包、地图、日志和测试结果进入独立数据根。数据位置写入忽略的 `config/storage.local.json`，不会随 Git 上传。使用外置盘时，先自行挂载并通过 `lsblk -f` 确认 UUID，再替换下面三个占位值：

```bash
python3 scripts/configure_storage --backend removable \
  --archive-root /实际挂载目录/wheelchair \
  --mount-point /实际挂载目录 \
  --required-uuid 实际文件系统UUID
```

该命令不格式化或挂载磁盘。已选外置盘不可用或身份变化时，程序拒绝写入，不回退到内部同名目录。锁、套接字等运行状态保留在本机 `.phase1_runtime/`。

### 3. 获取固定版本雷达 SDK 并应用补丁

源码依赖 [XT-Toffuture/xtsdk_ros](https://github.com/XT-Toffuture/xtsdk_ros) 的固定提交；完整 SDK 不随本仓库发布。下列命令仅在 SDK 目录不存在时执行，已有目录应先核查，不能重复打补丁：

```bash
(
  set -euo pipefail
  if test -e SDKs/xtsdk_ros || test -L SDKs/xtsdk_ros; then
    echo 'SDK 已存在，请核对提交与补丁后继续。' >&2
    exit 1
  fi
  mkdir -p SDKs
  test ! -L SDKs
  git clone --no-checkout https://github.com/XT-Toffuture/xtsdk_ros.git SDKs/xtsdk_ros
  git -C SDKs/xtsdk_ros checkout --detach 965d31ae726c44b47ad646666c3c5fa2a4d91bf4
  test "$(git -C SDKs/xtsdk_ros rev-parse HEAD)" = 965d31ae726c44b47ad646666c3c5fa2a4d91bf4
  printf '%s\n' '633832092c07a47e2e557d0023ddf1f882295a97e2b9bcf640e8f0a7ab9e8ecd  vendor_patches/xtsdk_ros_965d31a.patch' | sha256sum --check -
  PATCH_FILE="$PWD/vendor_patches/xtsdk_ros_965d31a.patch"
  git -C SDKs/xtsdk_ros apply --check "$PATCH_FILE"
  git -C SDKs/xtsdk_ros apply "$PATCH_FILE"
)
```

不需要运行厂商 `selros.sh`、安装脚本或示例。本项目负责构建补丁后的驱动。固定过滤库来自 [XT-Toffuture/xtsdk_cpp](https://github.com/XT-Toffuture/xtsdk_cpp) 的 `3d3db067ae9bdc0528202c3087bc10fd3b706638`，本仓库已附两种架构的二进制和 NOTICE；正常 clone 后无需再次下载完整 `xtsdk_cpp`。两份都参与来源检查：

```bash
printf '%s\n' \
  '3336a76590b5447efd7c037929e61c287523fcd79e8125589a3adb35eee83411  SDKs/xtsdk_filter_3d3db067/lib/linux/aarch64/libxtsdk_shared.so' \
  '69d80f0d64f1b7a65c1cf76aa57d7ba84fb0ff34a03014e6d04d52092e683821  SDKs/xtsdk_filter_3d3db067/lib/linux/x86_64/libxtsdk_shared.so' \
  | sha256sum --check -
```

若上述文件缺失，先确认源码 clone 完整，不从任意最新版 SDK 拷贝同名库。来源清单、闭源库边界和许可见 [vendor_patches](vendor_patches/README.md) 及 `SDKs/xtsdk_filter_3d3db067/NOTICE.md`。

### 4. 准备 OpenCV 并构建全部功能包

```bash
bash scripts/prepare_opencv45.sh
source "${WHEELCHAIR_ROS_SETUP:-/opt/ros/humble/setup.bash}"
python3 scripts/wc_phase1 build
source install/main/setup.bash
```

`prepare_opencv45.sh` 下载并提取 Ubuntu `4.5.4+dfsg-9ubuntu4` 到 `SDKs/ubuntu_opencv45/`，不安装这些包、不替换系统 OpenCV。首次执行需要网络和 Ubuntu 软件源。构建包含 `wc_interfaces`、`wc_xt_driver`、`wc_bringup`、`wc_slam`、`wc_estimation`、`wc_camera_panel` 六个包；安装结果位于 `install/main/`。不要将别的机器的 build/install 当作新机安装结果。

### 5. 配置本机设备，保留公开模板

仅浏览已有结果或处理旧录包时，不必接入真实传感器。接入设备前先创建私有副本，填写实际身份；下面只复制模板，不打开设备，也不覆盖已有本机文件：

```bash
(
  set -euo pipefail
  umask 077
  cp -n config/device_bindings.json config/device_bindings.local.json
  mkdir -p config/local/project/comparison_20260917_handpush config/local/lidar
  for name in cameras.json wheel_feedback_current.json live_unvalidated.json \
    mapping_live.json mapping_3d_diagnostic.json hardware_setup.json \
    wheel_manual_stop_evidence.template.json comparison_20260917_handpush/mapping.json; do
    cp -n "config/$name" "config/local/project/$name"
  done
  cp -n src/wc_xt_driver/config/*.yaml src/wc_xt_driver/config/*.xtcfg config/local/lidar/
)
```

| 私有配置位置 | 必须核对的内容 |
|---|---|
| `config/device_bindings.local.json` | IMU 稳定设备路径、by-id、硬件序列号、`H30-序列号`；可选雷达网卡绑定。 |
| `config/local/project/live_unvalidated.json`、`config/local/lidar/` | 左右雷达身份、设备/接收 IP、端口；JSON 与 YAML 中的侧别、序列号保持对应。 |
| `config/local/project/cameras.json` | 四路角色、稳定 by-path / udev ID_PATH、VID/PID 和序列号；不靠 video 编号猜角色。 |
| `config/local/project/wheel_feedback_current.json` | 轮反馈稳定路径、by-id、序列号、控制器身份与协议。 |
| `config/local/project/hardware_setup.json`、`mapping_live.json` | 安装关系、轮几何、IMU 身份、有效外参与证据引用。更换主机不等于改变机械参数。 |
| `config/imu_timing.local.json` 与 `config/local/imu_timing/` | 当前 IMU 的已核验时序配置和原始证据；保留原字节及哈希。不能将示例证据改成自己的序列号冒充验证。 |
| `config/storage.local.json` | 本机数据根及可选外置盘 UUID，由存储配置入口维护。 |

本机覆盖采用**完整文件替换**，不混合不同机器字段；已存在但损坏的覆盖会报错，不悄悄退回模板。未创建覆盖的项目配置仍使用基础文件，所以应在面板修改设备参数前完成私有副本初始化。`config/local/`、`*.local.json`、面板参数档案及现场生成的标定证据均被忽略，不随源码提交。复制源码到另一台机器时不要直接复制这些身份文件。

读取设备枚举与授权、相机 schema、雷达静态地址和 udev 规则的完整步骤见[连接真实设备](docs/getting_started/deployment.md#6-连接真实设备)。项目不自动修改设备固件或网络。IMU 时序必须按[时序说明](docs/reference/imu_timing.md)恢复本机证据；没有有效证据时先完成设备验证，不能跳过检查开始正式采集。

### 6. 验证安装

```bash
source /opt/ros/humble/setup.bash
source install/main/setup.bash
python3 scripts/check_layout.py
bash tests/run_target_tests.sh tests/usb_storage tests/panel_backend tests/panel_ui
python3 scripts/wc_phase1 doctor --profile mapping_core
```

软件测试使用隔离输入；`doctor` 只检查环境和配置，未连接真实设备时可能报告设备缺失。完整构建、原生测试和 Python 回归使用 `bash scripts/build_verify.sh`。结果自动存入数据根的 `dev_archive/`，查看最终退出码及报告，不只看窗口能否打开。实际采集、车辆交接和动态建图另做现场验证。

### 7. 日常启动和后续更新

首次部署完成后，在图形桌面终端进入工程，执行 `bash scripts/panel` 即可；不需要每次重装 SDK 或重新构建。主面板启动本身不会启动采集或驾驶。命令行工作的新终端先加载 ROS 和安装环境：

```bash
cd "$HOME/projects/wheelchair"   # 替换为自己的项目目录
source /opt/ros/humble/setup.bash
source install/main/setup.bash
bash scripts/panel
```

更新前正常结束所有采集、建图及处理任务，私下备份本机覆盖和数据，先查看 `git status --short`。工作区干净时更新默认分支并重建：

```bash
git switch main
git pull --ff-only origin main
python3 scripts/wc_phase1 build
```

有未提交修改或快进失败时，先保存并处理差异，不用强制重置覆盖自己的代码。跨设备部署和重装备份清单见[部署说明](docs/getting_started/deployment.md)。

## 开始使用

- [环境、依赖与部署](docs/getting_started/deployment.md)
- [面板使用](docs/operations/panel_user_guide.md)
- [命令参考](docs/operations/command_reference.md)
- [数据与存储](docs/operations/storage.md)
- [配置与标定](docs/reference/calibration_configuration.md)
- [功能测试](tests/README.md)

运行平台为 Ubuntu 22.04 / ROS 2 Humble，支持的 Linux 架构及厂商依赖见部署说明。Windows 工作区用于资料和源码备份，不能代替目标平台构建及验收。

### 打开面板

在 **Orin 图形桌面的终端**中执行：

```bash
cd /home/nvidia/wheelchair
bash scripts/panel
```

也可以从任意目录执行 `bash /home/nvidia/wheelchair/scripts/panel`。入口会自行定位工程；`scripts/panel` 是 Bash 脚本，应使用 Bash 启动。只查看命令帮助可执行 `bash /home/nvidia/wheelchair/scripts/panel --help`。

Windows 的 `source_backup` 用于阅读和备份。普通 SSH 会话通常没有图形显示环境；实际操作面板时，请使用 Orin 本机桌面或已有的远程桌面。已完成部署的机器无需每次启动面板前重新构建。

### 七个功能入口

| 面板按钮 | 用法 | 完成时应确认 |
|---|---|---|
| 实时建图 | 选择模式及参数后打开 RViz；建图模式先显示当前帧，再点击“开始建图”。 | 点击“停止并保存”，正常关闭本次地图后恢复实时预览并询问保存；可在同窗再次开始新地图。 |
| 数据录制 | 设置名称前缀、时间后缀、保存根目录和雷达，然后确认录制。 | 在 RViz 点击“结束录制”或正常关闭窗口，等待收尾和完整性检查，再确认“录制已完成”及数据目录。 |
| 离线融合 | 选择已有录包、输出目录及有效方案，确认任务列表后开始。 | 在队列中查看每项实际终态和结果位置；失败原因见日志。 |
| 结果对比 | 选择一组或多组已有结果，打开后切换地图、路线和三维点云。 | 缺失文件会明确提示；三维视角联动只同步视角，不代表结果已配准。 |
| 设备参数 | 查看安装关系和配置；需要修改时解锁对应参数组，核验后保存。点云配对也从这里进入。 | 参数保存成功后对下一次任务生效；点云配对只导出候选，不自动覆盖正式外参。 |
| 数据与存储 | 设置录制与融合结果的默认根目录，查看占用或清理允许删除的目录。 | 修改默认路径不会搬迁已有数据；删除需要确认，执行后不能通过面板撤销。 |
| 日志 | 选择任务，再选择总日志或实际阶段日志。 | 按具体错误检查设备、配置、输入或存储条件。 |

初次使用建议先查看“数据与存储”和“设备参数”，确认保存位置、设备身份及标定状态，再进入所需功能。查看已有结果、读取日志和处理已有录包可直接选择相应入口。

常规“数据录制”会启用人工驾驶通道，使用前需确认设备与现场条件。用于静态点云配对时，进入“设备参数 → 打开点云配对工具 → 录制新数据／从录包准备点云”，该录制入口关闭人工驾驶通道。

录制名称可勾选 `YYYY`、`MMDD`、`HHMMSS`，同名自动追加序号。窗口关闭不代表数据已完整保存，应等待最终结果。正常结束建图时，取消保存路径选择会返回保存选择流程；保存失败会保留临时结果。

同一用户只运行一个主面板。实时、录制或恢复任务尚未结束时关闭主面板，会最小化并继续管理任务。仅有离线队列运行时退出主面板，后台队列仍可继续；需要停止时使用离线窗口中的“取消剩余任务”：这会停止当前离线计算并取消后续任务，保留已生成的结果；应等待取消收尾完成。

### 功能状态与使用条件

面板的七个入口均已接入功能实现，软件回归覆盖界面、任务管理、配置、录制命名、停止与保存、结果读取等行为。软件验证与现场设备验证分别记录；软件测试通过不等于已验证全部传感器连接、车辆运动或动态建图效果。

- 实际采集与实时建图需要匹配的设备、可用存储及所选模式要求的有效外参。缺少跨雷达外参时，双雷达可分别预览原生点云，不能据此认定两侧已经对齐。
- “官方 EKF 几何反馈”尚未实现，保持禁用；连续白加速度 PSD 仅用于五状态 EKF 的相应模式。界面置灰的方案及原因应按实际说明处理。
- 在线运动修正需要匹配当前设备身份的校正声明、零偏确认及对应证据；离线录包生成的候选不会自动用于在线任务。
- 离线融合使用录包冻结的配置；修改当前设备参数不会改写历史录包。机械初值和不完整录包诊断需显式选择，其结果用途见操作手册。

更多参数、图形操作和保存流程见[完整面板使用说明](docs/operations/panel_user_guide.md)。

### 命令行入口与存储

在项目目录查看独立采集和存储配置命令：

```bash
python3 scripts/wc_phase1 --help
python3 scripts/configure_storage --help
```

数据根可使用普通目录或经明确配置的外置设备。本机覆盖优先于通用模板，既定目的地不可用时停止并报错。IMU 设备时间配置同样支持本机覆盖；离线处理始终读取录包冻结配置。

## 常用操作：采集、融合、查看和标定

所有命令在项目根目录运行；带图形窗口的命令使用本机或远程图形桌面。以下 `WC_DATA` 由当前存储配置解析，`SESSION` 替换为实际录包目录名，输出目录必须是新的，避免覆盖已有结果：

```bash
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONNOUSERSITE=1
WC_DATA="$(dirname -- "$(python3 -m wc_runtime.storage_policy --project-root "$PWD" data)")"
```

### 录制数据和核验完整性

`mapping_core` 录制所选雷达、IMU 和轮反馈；`mapping_cameras` 另外录制相机；`all_sensors` 再包括超声波。先运行不打开设备的检查，再根据实际需要执行录制命令：

```bash
SESSION="capture_$(date +%Y%m%d_%H%M%S)"
python3 scripts/wc_phase1 capture --profile mapping_core \
  --session "$SESSION" --duration 30 --dry-run
# 下面会打开真实传感器；仅在设备配置和现场条件确认后执行。
python3 scripts/wc_phase1 capture --profile mapping_core \
  --session "$SESSION" --duration 30 --preview
```

`--duration 0` 表示一直录制到正常停止，正数范围 1–3600 秒。`--sides left|right|all` 选择雷达，单侧属于明确诊断模式；不通过漏掉设备伪装完整采集。默认内存暂存，停止后转存并核验，所以可录时长同时受内存和磁盘限制。外置 exFAT 上启用 `--manual-drive` 时保留默认 `--staging memory`。

命令行只有显式加 `--manual-drive` 才启用本次人工驾驶；面板常规录制入口会启用该通道。停止优先使用 RViz 的“结束录制”、正常关闭窗口或终端一次 Ctrl+C，然后等待保存结束。需要按会话管理时：

```bash
python3 scripts/wc_phase1 status --session "$SESSION"
python3 scripts/wc_phase1 stop --session "$SESSION"
python3 scripts/wc_phase1 doctor \
  --session-root "$WC_DATA/data/experiments/$SESSION" --verify-archive
```

最终输出以程序返回的实际目录为准，面板自定义名称或同名避让会改变目录名。`PARTIAL`、文件存在或窗口关闭都不能代替完成确认。保留原始录包及其冻结配置；不要手工改哈希或删除缺失源记录让核验通过。

### 同一录包离线融合、建图和查看

离线功能读取录包冻结的设备身份、配置和时序证据，不打开设备。将 `SESSION` 改为实际录包目录；只计算轨迹时去掉 `--native-map`：

```bash
SESSION="实际录包目录名"
RESULT="$WC_DATA/data/analysis/fusion_$(date +%Y%m%d_%H%M%S)"
python3 scripts/wc_phase1 compare \
  --dataset "$WC_DATA/data/experiments/$SESSION" --output "$RESULT" \
  --estimators five_state robot_localization \
  --input-rate-hz 5 --cloud raw --filter on --native-map
```

`compare` 也用于只生成一套融合结果；并非只能做多方案比较。`--lidar left|right|all` 可从同一录包选择来源，省略时沿用录包模式，仍保留原始 IMU、轮反馈和共同时间起点。`--input-rate-hz 0` 消费全部有效配对；正数上限 10 Hz，仅限制离线点云消费，不删原录包的 IMU/轮反馈。`--imu-time-mode auto` 优先使用录包内核验过的设备时间，旧录包没有策略时沿用接收时间。

缺少已验证外参时命令会明确拒绝。确需用装配初值做离线实验时，显式追加 `--mechanical-initial`，其结果不作为正式外参或精度证明。不完整录包只有在理解缺失项后才使用 `--allow-partial`；不作为默认参数。

生成的各方案位于结果目录的 `native/` 等子目录，原生建图的 `export/` 含地图输出。用面板“结果对比”选中所需结果，或根据实际输出路径查看：

```bash
python3 scripts/view_map "$RESULT/native/five_state_hz5_filter_on/export" --check
python3 scripts/view_map "$RESULT/native/five_state_hz5_filter_on/export"
```

仅有 RTAB-Map `.db` 时，查看器会先在临时目录导出副本；原生导出子进程最多等待 900 秒（15 分钟），复制、校验和显示时间另计。超时会报告日志末尾并退出，原库保留，临时副本清理后可重试。已有 PLY/YAML/PGM 导出文件时直接读取；`--check` 不执行导出。

前端 `--input-rate-hz 5` 与原生建图采样是两个设置：`--native-rate-hz` 默认 1 Hz；需要原生地图按 5 Hz 消费时显式设置 `--native-rate-hz 5`，处理时间和资源占用会增加。分组名称会随估计器、频率和过滤组合变化，以日志和实际目录为准。未加 `--native-map` 时不会产生上述可视地图；单个方案失败时应检查该方案日志。两张图同步视角不代表坐标已经配准。

需要重估本录包的零偏、轮速一致性或做受约束的几何实验时使用独立入口：

```bash
python3 scripts/wc_phase1 refine \
  --dataset "$WC_DATA/data/experiments/$SESSION" \
  --output "$WC_DATA/data/analysis/refine_$(date +%Y%m%d_%H%M%S)" \
  --geometry off --native-map
```

几何实验、连续白加速度过程噪声及候选复用条件见[离线运动对照](docs/operations/offline_motion_compare.md)。同一输入、时基和校验条件满足时才能复用候选，不自动更新在线设备参数。

### 实时预览、建图和保存补救

下面的正常运行命令会连接设备；`--check-config` 只检查配置：

```bash
python3 scripts/map all --check-config
python3 scripts/map left                     # 左雷达实时预览
python3 scripts/map all --mapping true        # 双雷达实时建图
python3 scripts/map all --mapping true --interactive  # 同窗预览、开始、停止并保存
```

可选 `right`，以及 `--cloud raw|filtered`、`--estimator five_state|robot_localization`。预览默认不累计地图。正式双雷达建图要求相应外参与证据，不能将各自传感器坐标中的显示当作已对齐。实时人工操作、停止和保存请按[面板说明](docs/operations/panel_user_guide.md)进行。

面板建图与 `--interactive` 使用同窗按钮：初始和停止后只显示当前帧，建图期间显示本次累计三维云；每次开始都是新会话、新地图。切换时短暂停止并重新打开所属设备，界面显示“正在切换”。停止建图正常关库后先恢复真实预览，再询问旧地图是否保存。关闭整个交互窗口会停止设备、取消尚未完成的保存操作并保留待处理数据。只有单次保存返回 `SAVED` 才表示保存成功；存在待保存地图时窗口任务以 `SAVE_PENDING` 结束。

不带 `--interactive` 的旧命令仍在正常关闭 RViz 后停止设备并询问保存。`SAVE_PENDING` 表示原数据保留、等待选择；失败时检查日志并保留工作目录。对已停止且尚未完成保存的工作会话：

```bash
python3 scripts/save_map /实际会话工作目录 /新的地图保存目录
```

不用强制关闭进程或断电替代停止。默认实验保存保留原始数据引用；显式 `map_only` 会清理对应临时原始数据，之后不能完整重放。

### 点云配对、参数修改及辅助功能

在面板“设备参数”中打开点云配对工具，可以从已有录包准备点云，或通过专用静态入口录制新场景。该入口关闭人工驾驶通道。选定同名点或进行整云调整后导出候选，检查坐标约定和独立场景，再决定是否采用；候选不会自动写成正式外参。

已有 `prepared.json` 时可用独立网页工具：

```bash
bash scripts/pick_lidar_points.sh --input /实际路径/prepared.json
bash scripts/align_lidar_clouds.sh --input /实际路径/prepared.json
```

参数页按组解锁、校验、保存，成功后对新任务生效；活动任务仍使用创建时冻结的配置。恢复参数前核对档案来源和设备身份。日志页可查看总日志及各阶段日志；存储页更改默认位置不会移动旧数据，删除操作不可通过面板撤销。

| 其他需求 | 命令或说明 |
|---|---|
| 四相机实时预览 | `bash scripts/view_cameras.sh monitor_320`；会打开设备，此脚本不支持 `--help`。 |
| 独立雷达预览 | `bash scripts/view_lidars.sh --help` 查看侧别和显示参数。 |
| 静态配准录制与准备 | `bash scripts/record_lidar_points.sh --help`；实际录制参数见[命令手册](docs/operations/command_reference.md)。 |
| 幅度图辅助选点 | [幅度图操作](docs/operations/amplitude_alignment.md)。 |
| 轮反馈只读诊断、消息回放、地图包、目标点注释 | [完整命令手册](docs/operations/command_reference.md)；目标点注释不会执行导航。 |
| 定位、车头朝向与后续避障接入 | [定位与障碍感知](docs/architecture/navigation-and-obstacles.md)；说明超声波与激光雷达分工、地图位姿和 IMU 航向边界，当前未启用自动导航。 |
| 暂存、掉盘和输出路径 | [存储说明](docs/reference/storage.md)。 |

## 常见问题与恢复

| 现象 | 检查与处理 |
|---|---|
| `No module named PyQt5` 或 ROS Python 包缺失 | 确认使用系统 Python，安装上面的依赖并 source Humble/install 环境。 |
| 面板没有图形显示 | 在 Orin 本机或远程图形桌面终端启动；普通无显示 SSH 不具备 Qt 桌面环境。 |
| SDK/补丁/过滤库校验失败 | 核对固定提交、补丁及两架构库哈希，不直接替换为厂商最新库。 |
| 找不到 OpenCV 4.5 或 `libtbb.so.2` | 完成依赖和 `prepare_opencv45.sh`，重新构建；不用跨 ABI 软链接。 |
| 设备身份、串口权限或接收 IP 不匹配 | 核对私有配置、udev 属性、用户组和本机网卡；保留身份校验。 |
| 存储不可用、空间不足 | 核对选定数据根、真实挂载和 UUID；先正常结束任务，再处理空间。 |
| 录包 `PARTIAL`、融合拒绝或保存失败 | 读取最终状态与具体阶段日志，保留原录包和工作目录；不要先删数据重试。 |
| 新参数未影响正在运行的任务 | 任务冻结启动时的配置；结束后新建任务加载新参数。 |

重装或回退前私下保存 `config/*.local.json`、`config/local/`、现场参数档案、udev/网卡设置、源码提交与未提交差异、固定 SDK 和数据根。对于 `reports/` 下包含 `deployed_state.json` 和 `baseline/source_manifest.json`、`baseline/source.tar.gz` 的兼容部署报告，先用 `python3 scripts/restore_deployment.py --report /实际报告目录` 预览；仅在确认且任务全部结束后追加 `--apply`，并按结果重新构建。恢复方式和边界见[部署说明](docs/getting_started/deployment.md)。

## 功能与维护位置

| 功能 | 主要实现 | 对应验证 |
|---|---|---|
| 雷达与传感器输入 | [wc_xt_driver](src/wc_xt_driver/README.md)、[wc_sensors](src/wc_sensors/README.md)、[wc_imu](src/wc_imu/README.md) | [sensors](tests/sensors/README.md)、[imu](tests/imu/README.md)、[设备时间](tests/imu_timing_capture/README.md) |
| 相机 | [wc_cameras](src/wc_cameras/README.md)、[wc_camera_panel](src/wc_camera_panel/README.md) | [cameras](tests/cameras/README.md)、[camera_panel](tests/camera_panel/README.md) |
| 轮反馈与手动交互 | [wc_motion](src/wc_motion/README.md) | [motion](tests/motion/README.md)、[teleop_panel](tests/teleop_panel/README.md) |
| 标定与点云配对 | [wc_calibration](src/wc_calibration/README.md) | [calibration](tests/calibration/README.md) |
| 融合与建图 | [wc_fusion](src/wc_fusion/README.md)、[wc_estimation](src/wc_estimation/README.md)、[wc_slam](src/wc_slam/README.md)、[wc_maps](src/wc_maps/README.md) | [fusion](tests/fusion/README.md)、[slam](tests/slam/README.md)、[maps](tests/maps/README.md) |
| 任务、采集与回放 | [wc_runtime](src/wc_runtime/README.md) | [runtime](tests/runtime/README.md)、[operations](tests/operations/README.md) |
| 面板和可视化 | [wc_panel](src/wc_panel/README.md)、[wc_bringup](src/wc_bringup/README.md) | [panel_ui](tests/panel_ui/README.md)、[panel_backend](tests/panel_backend/README.md)、[panel_rviz](tests/panel_rviz/README.md) |

## 目录导航

| 位置 | 内容与添加规则 |
|---|---|
| [src](src/README.md) | 正式实现，按功能包扩展；可复用逻辑不放在测试目录。 |
| [scripts](scripts/README.md) | 用户命令与构建、验证、维护入口。 |
| [config](config/README.md) | 通用配置、模板与必要来源；机器覆盖保存在忽略的文件中。 |
| [tests](tests/README.md) | 按功能组织的测试及小型样本；正式功能不得反向依赖这里。 |
| [docs](docs/README.md) | 入门、操作、设计、接口与开发说明。 |
| [vendor_patches](vendor_patches/README.md) | 厂商补丁、来源清单与许可。 |
| `.phase1_runtime` | 本机生成的会话、锁和用户输入索引，不进入版本库。 |
| 配置的数据根 | 录包、地图、生产分析结果和 `dev_archive`，与源码分开。 |

新增功能先确定所属模块，同步添加相应测试和操作说明。确需新建目录时，必须同时编写 README 并更新父级导航；每份目录说明写清职责、内容、入口、依赖、新文件归属和验证方法。详见[开发规范](CONTRIBUTING.md)与[目录规则](docs/development/layout.md)。

## 构建与验证

依赖就绪后使用 `bash scripts/build_verify.sh` 完成目标平台构建与验证；此命令涉及现有完整测试集合。单独验证某个功能时使用：

```bash
bash tests/run_target_tests.sh tests/calibration
bash tests/run_target_tests.sh tests/panel_backend tests/panel_ui
```

验证结果自动进入数据根的独立开发任务目录。纯软件、合成输入、真实录包离线和实机验证分别记录；软件通过不代表物理运行已经验收。原有算法参数、坐标契约、设备身份和停止保存边界必须保持。

## 历史资料与开发归档

```bash
python3 scripts/dev_archive sensor_review
```

每个任务有独立目录及 tasks、prompts、handoffs、reports、validation、snapshots、backups 分类，均带 README。索引和迁移清单记录来源、对应提交与校验；同名任务不覆盖。历史过程资料不参与正常功能运行，也不随源码提交。

原始测量证据、旧录包和厂商原件保持原始内容；协议兼容记录可能保留历史标识。不可变快照仅在外层添加说明。build、install、缓存、本机状态及私有覆盖等生成目录不适用逐层维护 README 的要求。
