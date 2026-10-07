# 部署、换机与重置恢复

部署顺序是：**系统 / ROS → 源码 → 数据目录 → SDK / ABI 依赖 → 构建 → 离线验证 → 可选硬件接入**。`configure_storage` 只完成其中的数据目录设置。

## 1. 平台、目录与验证边界

| 项目 | 支持目标 / 限制 |
|---|---|
| Linux | Ubuntu 22.04，aarch64 或 x86_64 |
| ROS / Python | ROS 2 Humble，系统 Python 3.10 |
| 建图 / 官方 EKF | RTAB-Map 0.23 系列、robot_localization；历史实机分别为 0.23.7、3.5.4 |
| 工程目录 | 普通本地 Linux 文件系统，任意用户名 / 主机名，可含空格 |
| 数据目录 | 独立于代码，默认 `~/wheelchair-data` |
| Windows / WSL2 | 原生 Windows 不作为运行环境；WSL2 可用 Linux 离线流程，设备转发、网络接收、GUI 未作实机验收 |

历史完整传感器流程运行于 Jetson Orin、Ubuntu 22.04.5、Jetson Linux 36.4.4。通用目录和架构适配不等于已对每个 x86 主机或新 Orin 完成现场验收。换机需要重新构建，不能复制旧 `install/` 后当作兼容。

入口根据脚本 / 模块所在位置向上查找 `src/`、`scripts/`、`config/`。一般无需额外环境变量；特殊嵌入方式可设置绝对路径 `WHEELCHAIR_PROJECT_ROOT`。`WHEELCHAIR_ROS_SETUP` 可指定兼容 Humble 的绝对 `setup.bash` 路径，默认 `/opt/ros/humble/setup.bash`。

## 2. 基础系统与依赖

普通电脑使用 Ubuntu 22.04；Orin 使用适合其型号、提供 Ubuntu 22.04 的 NVIDIA 系统镜像。按 [ROS Humble 官方安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)准备系统和 ROS 软件源。官方说明对应 Jammy，并提供 amd64 / arm64 软件包；若网站不可访问，可查阅其 [官方文档源码](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst)。不要直接将本工程 ABI 配方用于其他 Ubuntu / ROS 版本。

在已经配置好 Humble 软件源的机器上：

```bash
sudo apt update
sudo apt install \
  git curl ca-certificates build-essential cmake pkg-config \
  python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-scipy python3-yaml python3-serial \
  python3-opencv python3-matplotlib python3-psutil python3-pytest \
  libboost-all-dev libssl-dev libeigen3-dev libpcl-dev libopencv-dev \
  libtbb2 qtbase5-dev psmisc v4l-utils \
  ros-humble-desktop ros-humble-rtabmap-ros \
  ros-humble-robot-localization ros-humble-rosbag2-py \
  ros-humble-rosbag2-storage-default-plugins
```

ABI 条件：厂商过滤库需要 legacy `libtbb.so.2`，`libtbb.so.12` 不能代替；RTAB-Map 构建使用匹配的 OpenCV 4.5.4。在部分 Jetson 系统中系统 OpenCV 为其他版本，因此第 5 节单独提取开发 / 运行文件，不替换系统 OpenCV。不要用不同主版本库的软链接绕过检查。

APT 修订号可能变化。若固定版本无法取得，应查看依赖错误并使用已备份的相同软件包或重新做兼容性适配，不更改哈希清单来掩盖差异。

## 3. 源码与数据位置

选择尚不存在的工程目录。下列变量可以改成自己的位置，所有路径均使用引号：

```bash
PROJECT="$HOME/projects/wheelchair"
mkdir -p "$HOME/projects"
git clone --branch v7/orin-usb-20261006 \
  https://github.com/owenkings/smartwheel.git "$PROJECT"
cd "$PROJECT"
git rev-parse HEAD

python3 scripts/configure_storage \
  --backend directory --archive-root "$HOME/wheelchair-data"
```

`v7/orin-usb-20261006` 是本次维护的发布分支。默认分支可能不是同一版本；复现实验时记录具体提交。

存储通用配置是 `config/storage.json`，本机完整覆盖是被 Git 忽略的 `config/storage.local.json`。数据根可选择内部 SSD 的普通目录，不依赖外置盘。日志、录包、地图和测试产物经存储策略进入数据根；代码、维护配置、测试源码留在工程。

## 4. 固定 SDK 与已附过滤库

SDK 主体来源 [`XT-Toffuture/xtsdk_ros`](https://github.com/XT-Toffuture/xtsdk_ros)，固定提交 `965d31ae726c44b47ad646666c3c5fa2a4d91bf4`。不执行厂商 `selros.sh`、安装脚本或样例；本项目 CMake 编译经补丁处理的源码。

在工程根、SDK 目录尚不存在时：

```bash
(
  set -euo pipefail
  if test -e SDKs/xtsdk_ros || test -L SDKs/xtsdk_ros; then
    echo 'SDK 已存在，保留并单独核查，不重复应用补丁。' >&2
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

CMake 检查源码清单、补丁、上游提交和反向补丁。已有 SDK 保留并核查，不自动删除或重置。

固定过滤库已经随仓库保存到 `SDKs/xtsdk_filter_3d3db067/`，来源为 `XT-Toffuture/xtsdk_cpp` 的 `3d3db067ae9bdc0528202c3087bc10fd3b706638`。两个架构文件都应保留，构建清单检查两份：

```bash
printf '%s\n' \
  '3336a76590b5447efd7c037929e61c287523fcd79e8125589a3adb35eee83411  SDKs/xtsdk_filter_3d3db067/lib/linux/aarch64/libxtsdk_shared.so' \
  '69d80f0d64f1b7a65c1cf76aa57d7ba84fb0ff34a03014e6d04d52092e683821  SDKs/xtsdk_filter_3d3db067/lib/linux/x86_64/libxtsdk_shared.so' \
  | sha256sum --check -
```

原 SDK 的旧库不覆盖。固定来源、补丁语义及闭源边界见 [vendor_patches](../vendor_patches/README.md) 和 [过滤库 NOTICE](../SDKs/xtsdk_filter_3d3db067/NOTICE.md)。

## 5. OpenCV ABI 与完整构建

以下脚本从 Ubuntu 软件源下载固定 `4.5.4+dfsg-9ubuntu4` 包，提取到工程 `SDKs/ubuntu_opencv45/`；不安装包、不运行包维护脚本、不替换系统 OpenCV：

```bash
bash scripts/prepare_opencv45.sh
source "${WHEELCHAIR_ROS_SETUP:-/opt/ros/humble/setup.bash}"
python3 scripts/wc_phase1 build
source install/main/setup.bash
```

完整构建入口覆盖六个维护包：

| 包 | 用途 |
|---|---|
| `wc_interfaces` | 消息与来源身份 |
| `wc_xt_driver` | 双雷达驱动及 SDK 校验 |
| `wc_bringup` | Python 模块、启动、预览、手动界面 |
| `wc_slam` | RTAB-Map 后端 |
| `wc_estimation` | 官方 EKF 库工作进程 |
| `wc_camera_panel` | RViz 相机面板 |

`build/` 和 `install/` 为本机产物；构建日志写数据根 `reports/builds/`。历史专项 `build_verify.sh` 不能代替当前完整构建。每次新终端手动运行 ROS 工具前，先进入工程根并 source 上述 ROS 与 install 环境。

## 6. 连接真实设备

**离线重放可以跳过本节。** 公共配置含示例身份；换机时填写真实枚举，保持身份检查。配置驱动绑定不等于修改设备固件、IP 或内部参数。

### 6.1 IMU 和可选网卡限制

复制完整默认结构为本机覆盖，填写真实值：

```bash
cp -n config/device_bindings.json config/device_bindings.local.json
```

| 字段 | 内容 |
|---|---|
| `imu.device` | 稳定设备别名或稳定设备路径，不能用会漂移的 tty 编号猜测 |
| `imu.expected_by_id` | 实际 `/dev/serial/by-id/` 链接 |
| `imu.hardware_serial` | 实际硬件序列号 |
| `imu.sensor_id` | `H30-` 加同一硬件序列号 |
| `network.interface` | 可填本机雷达网卡名；null 表示不固定接口名，仍核验配置的接收地址 |

本机文件完整替换通用文件，不逐字段混合，防止两套设备身份拼在一起。文件被 Git 忽略，重装前需私下备份。离线估计器使用录包身份，不使用当前机器的 IMU 绑定。

### 6.2 雷达接收地址

查看 `ip -br address` 和系统网络设置，确定连接雷达的专用接口。接口名称由机器决定，不要求固定名称。

雷达配置位于 `config/live_unvalidated.json` 与 `src/wc_xt_driver/config/`。按实际设备读回核对序列号、设备 IP、接收 IP 和端口；两个侧别的配置必须一致对应物理左右。SDK 配置安装到包内，修改后重新构建 / 安装并核对实际生效来源。

默认网络示例为左设备 `192.168.0.101` → 主机 `192.168.0.100:7687`，右设备 `192.168.1.101` → 主机 `192.168.1.100:7687`。仅当设备确实使用这些地址时，才在雷达专用接口配置相应静态地址；不设置无关默认网关，不覆盖现有上网 / SSH 连接。程序不自动修改网络。

### 6.3 轮反馈、相机、超声波

| 设备 | 配置入口 | 核对内容 |
|---|---|---|
| 轮反馈 | `config/wheel_feedback_current.json`，以及使用中的手动硬件配置 | 稳定设备别名、by-id、序列号、VID/PID、寄存器协议 |
| 四相机 | `config/cameras.json` | 每个物理角色的稳定 by-path 与实际 udev ID_PATH、VID/PID / 序列号；不能自动按 video 编号重分配 |
| 超声波 | `config/ultrasonic_capture.json` | 适配器身份和物理 USB 路径，地址 1–4；无回复不得填零距离 |
| 安装及轮几何 | `config/hardware_setup.json` 与来源资料 | 更换计算机不改变机械参数；改变装配才修改关联项 |

这些绑定由配置提供，不应为每台机器修改生产代码中的序列号常量。相同轮椅重装可以恢复原绑定；USB 拓扑变化时更新相应角色，不取消设备身份检查。`mapping_core` 不检查、启动或记录相机与超声波。

具体字段：轮反馈的 `device`、`expected_by_id`、`hardware_serial` 与 `device_id=ZLAC8030D-序列号` 必须指向同一设备，同时匹配 `usb_vid` / `usb_pid`。超声波使用 `device`、`expected_by_id`、`expected_usb_path`、`usb_vid` / `usb_pid`；没有唯一序列号的适配器尤其需要物理拓扑核对。

新机器相机配置使用 `schema_version: 2`，每个 `cameras[]` 条目填写 `device`（完整 `/dev/v4l/by-path/...-video-index0`）和 `expected_id_path`（对应 udev `ID_PATH`），保留 `role`、`port`、`rotate_deg`、`vid`、`pid`、`serial`。四个角色、路径和物理拓扑必须唯一；旧 schema 1 的固定拓扑仍可用于历史兼容，不能把其端口编号当作任意主机的自动发现结果。

读取实际枚举的命令：

```bash
ls -l /dev/serial/by-id/
ls -l /dev/v4l/by-path/
ip -br address
# 将占位节点替换为当前实际设备，只读取属性。
udevadm info --query=property --name /dev/REPLACE_WITH_ACTUAL_NODE
```

如使用 `/dev/smartwheel_*` 别名，需要按真实设备属性创建本机 udev 规则；规则是系统部署内容，不会由 Git clone 自动恢复。读取和操作权限按本机用户配置：

```bash
sudo usermod -aG dialout,video,render "$(id -un)"
# 编辑本机规则后，在没有录制 / 控制任务时加载，并重新插接设备。
sudo udevadm control --reload-rules
```

注销再登录使用户组生效。不要使用含占位序列号的规则，也不要在运行中重绑定设备。外参修改及只读校验见 [外参配置说明](calibration_configuration.md)。

## 7. 分级验收

1. 源码、SDK 与 ABI 检查通过，六包完整构建。
2. 在已 source 环境运行选定软件测试，例如 `bash tests/run_target_tests.sh tests/usb_storage tests/operations/test_software_test_runner.py`。产物进入数据根。
3. `python3 scripts/wc_phase1 doctor --profile mapping_core` 做只读环境 / 配置检查；缺硬件不等于离线流程不能用。
4. 使用另行保存的真实录包运行 `compare` 或 `refine`，检查帧身份、输出哈希和地图可读性；不连接设备。
5. 用户接好设备后进行 30 秒静态录制，正常收尾，执行 `doctor --session-root data/experiments/会话名 --verify-archive`。
6. 由现场操作者验证键盘 / 手推交接、启停与路线；另外评价地图几何，不能用完整录制替代动态精度验收。

原生程序启动、输出文件存在、旧机器曾通过测试，均不是新机实机验收结论。具体命令见 [README](../README.md)，算法结果和下一步见 [选型说明](algorithm_selection.md)。

## 8. 重置前另行备份

| 内容 | 原因 |
|---|---|
| Git 提交号、未提交维护文件 | GitHub 不会自动保存现场新修改 |
| `config/*.local.json` 和现场设备 / 外参配置 | 通用模板不包含本机身份与路径 |
| 测量原文、来源哈希 | 保留装配依据，不重复测量已保存尺寸 |
| SDK 固定源码、OpenCV 包缓存和依赖版本 | 上游下载临时不可用时恢复相同 ABI |
| 本机 udev 规则、相关网卡地址和角色说明 | 系统重置会移除这些配置 |
| 数据根的录包、地图、必要报告与哈希 | 原始实验不在公开 Git 仓库 |

版本检查可用 `git rev-parse HEAD`、`git status --short`、`dpkg-query -W 'ros-humble-*' libopencv-dev libtbb2`。Jetson 的 L4T 版本另外记录，普通 x86 不需要 NVIDIA 系统包。仅备份有关设备的配置，不上传口令、SSH 私钥或其他凭证。

## 9. 可选外置存储

需要移动数据时，先挂载外置盘并用 `lsblk -f` 核对真实 UUID，再执行：

```bash
python3 scripts/configure_storage --backend removable \
  --archive-root /实际挂载目录/wheelchair \
  --mount-point /实际挂载目录 \
  --required-uuid 实际文件系统UUID
```

命令不格式化、挂载或选择第一只磁盘。选择外置后，掉盘 / 身份变化停止写入，不回退同名内部目录。普通本机目录仍是默认部署路线。详见 [存储说明](storage_portability.md)。

`capture` 默认先在本机内存暂存，停止后转存到外置盘。当前 exFAT 上的 `--manual-drive` 必须使用默认 `--staging memory`，不能同时选择 `--staging disk`；可录时长受内存和目标盘共同限制。关闭终端、拔盘或重启不能替代正常停止与转存核验，参见 [暂存模式](storage_portability.md#4-运行状态暂存和测试产物)。
