# Orin 部署与重置恢复

本页针对新刷机或换一台 Orin 后的恢复。`configure_storage` 只选择数据保存位置；它不会安装 ROS、SDK、依赖、编译程序或恢复设备连接，因此不能单独完成部署。

以下命令是部署说明，本次文档核查没有重刷系统、安装软件、修改网络、写入设备或进行新的清空构建。当前机器已运行的结果，不等于每种全新系统组合均已验收。

## 1. 当前支持的平台与目录

2026-10-07 只读核对的工作环境：

| 项目 | 当前环境 / 要求 |
|---|---|
| CPU / 系统 | Orin，aarch64，Ubuntu 22.04.5 LTS |
| NVIDIA 系统基础 | `nvidia-l4t-core` 36.4.4；没有用未安装的 `nvidia-jetpack` 元包推断 JetPack 版本 |
| ROS / Python | ROS 2 Humble；系统 Python 3.10 |
| RTAB-Map | 0.23.7；项目 CMake 要求 0.23 系列及匹配的 OpenCV ABI |
| 官方 EKF | `robot_localization` 3.5.4 |
| 用户、主机名、代码目录 | `nvidia`、`ubuntu`、`/home/nvidia/wheelchair` |
| 数据目录 | 独立于代码目录，可为普通磁盘目录或配置过的移动盘 |

当前多个入口及设备操作范围校验仍使用固定代码目录。`wc_phase1` 的 `build`、`test`、普通 `doctor` 等分支在执行前也检查 `ubuntu/nvidia/aarch64`，并非只有真实采集才检查。`capture`、`compare`、`refine` 分别有其处理入口；这不代表它们已经支持任意用户名或路径。新机按上表部署；改变用户名、主机名、路径或设备型号属于另一次适配，不能仅删除检查让它通过。

代码仓库包括源码、配置、测试源码、部署文档。`SDKs/xtsdk_filter_3d3db067` 中经过固定哈希校验的两份过滤库随本次仓库发布。其他 SDK 源码和 OpenCV 提取目录、`build/`、`install/`、原始录包、地图、测试产物和本机 `config/storage.local.json` 不随源码仓库上传。当前这套轮椅的机械几何配置和来源文件随仓库保存。原始数据需要另外保存或带走原存储盘。

## 2. 基础系统和依赖

使用适合该 Orin 型号的 NVIDIA 系统镜像，恢复上述 Ubuntu 22.04 / L4T 环境。ROS 按 [Humble 官方 Ubuntu 安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)配置官方源并安装 `ros-humble-desktop`。不要在未确认兼容性的 Ubuntu 24.04 / ROS Jazzy 上直接套用本页。

在已配置 ROS Humble 软件源的目标机上，安装项目依赖：

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

这里有两个不能省略的 ABI 条件：

- `libtbb2` 提供旧版 `libtbb.so.2`，厂商过滤库需要其符号；`libtbb12` / `libtbb.so.12` 不能代替它。当前 Jammy universe 的已安装版本为 `2020.3-1ubuntu3`。
- 当前 JetPack C++ OpenCV 是 4.8，而 RTAB-Map 的 Ubuntu 库使用 OpenCV 4.5.4。`wc_slam` 需要第 5 节单独提取的 4.5.4 开发文件；不要通过替换系统 OpenCV 解决。雷达驱动和建图后端是不同进程，当前驱动使用系统 4.8，后端使用 4.5.4。

APT 仓库中的包修订号会更新，上表是实测版本而非永不变化的下载保证。若固定的 SDK、OpenCV 包或所需 ABI 无法取得，应保留错误并解决依赖；不能跳过哈希 / ABI 检查或把缺失库软链接到其他主版本。

## 3. 取回源码并选择存储

本次发布分支为 `v7/orin-usb-20261006`；也可以选择已确认的具体提交。不要误用尚未更新的默认分支。

```bash
cd /home/nvidia
# 目标不存在时执行；保留已有 wheelchair 目录，不覆盖。
git clone --branch v7/orin-usb-20261006 https://github.com/owenkings/smartwheel.git wheelchair
cd /home/nvidia/wheelchair
git rev-parse HEAD

# 没有 U 盘也可运行，生成物写在代码目录之外：
python3 scripts/configure_storage \
  --backend directory --archive-root "$HOME/wheelchair-data"
```

最后一条只写本机存储选择。通用配置为 `config/storage.json`，本机覆盖为被 Git 忽略的 `config/storage.local.json`。如选择 U 盘，请按[存储说明](usb_storage.md)查询该盘实际挂载路径和 UUID 后配置；不要复制另一台机器的 UUID 或假定盘符相同。运行期间已选外置盘掉线时不自动回退内部盘。

## 4. 恢复两个固定版本的雷达 SDK 依赖

项目不执行厂商安装脚本、示例程序、`selros.sh` 或厂商会修改源码目录的 CMake。下列恢复只下载文件；实际构建由本项目 CMake 完成。

### 4.1 固定 SDK 源码及本项目补丁

源码来源：[`XT-Toffuture/xtsdk_ros`](https://github.com/XT-Toffuture/xtsdk_ros)，提交 `965d31ae726c44b47ad646666c3c5fa2a4d91bf4`。补丁在仓库 `vendor_patches/` 中。

```bash
(
  set -euo pipefail
  cd /home/nvidia/wheelchair
  test "$(id -un)" = nvidia
  test "$(uname -m)" = aarch64
  if test -e SDKs/xtsdk_ros || test -L SDKs/xtsdk_ros; then
    echo 'SDK 已存在，保留并单独核查；不要重置或再次叠加补丁。' >&2
    exit 1
  fi
  mkdir -p SDKs
  test ! -L SDKs
  git clone --no-checkout https://github.com/XT-Toffuture/xtsdk_ros.git SDKs/xtsdk_ros
  git -C SDKs/xtsdk_ros checkout --detach 965d31ae726c44b47ad646666c3c5fa2a4d91bf4
  test "$(git -C SDKs/xtsdk_ros rev-parse HEAD)" = 965d31ae726c44b47ad646666c3c5fa2a4d91bf4
  printf '%s\n' '633832092c07a47e2e557d0023ddf1f882295a97e2b9bcf640e8f0a7ab9e8ecd  vendor_patches/xtsdk_ros_965d31a.patch' | sha256sum --check -
  git -C SDKs/xtsdk_ros apply --check /home/nvidia/wheelchair/vendor_patches/xtsdk_ros_965d31a.patch
  git -C SDKs/xtsdk_ros apply /home/nvidia/wheelchair/vendor_patches/xtsdk_ros_965d31a.patch
)
```

CMake 会检查完整文件清单、补丁、上游提交身份以及反向补丁检查；不能通过修改 manifest 来掩盖来源不一致。

### 4.2 检查仓库中已附的过滤库

本次发布已按用户要求随仓库保存下面两份过滤库，正常 clone 后无需再次下载。过滤库与已包含在补丁中的接口回移代码绑定。来源：[`XT-Toffuture/xtsdk_cpp`](https://github.com/XT-Toffuture/xtsdk_cpp)，提交 `3d3db067ae9bdc0528202c3087bc10fd3b706638`。

经上游该提交的文件树核实，两个二进制位于 `xtsdk/lib/linux/{aarch64,x86_64}/libxtsdk_shared.so`。manifest 检查两份文件，因此即使在 Orin 上也保留两份；原 SDK 内的旧 `.so` 不覆盖。

正常部署只做字节校验：

```bash
cd /home/nvidia/wheelchair
printf '%s\n' \
  '3336a76590b5447efd7c037929e61c287523fcd79e8125589a3adb35eee83411  SDKs/xtsdk_filter_3d3db067/lib/linux/aarch64/libxtsdk_shared.so' \
  '69d80f0d64f1b7a65c1cf76aa57d7ba84fb0ff34a03014e6d04d52092e683821  SDKs/xtsdk_filter_3d3db067/lib/linux/x86_64/libxtsdk_shared.so' \
  | sha256sum --check -
```

只有使用旧版源码包、上述目录尚不存在时，才需要下面的上游恢复命令。目录存在但校验失败时先保留并核查，不能覆盖异常文件掩盖来源问题。

```bash
(
  set -euo pipefail
  cd /home/nvidia/wheelchair
  DEST=SDKs/xtsdk_filter_3d3db067
  if test -e "$DEST" || test -L "$DEST"; then
    echo '过滤库目录已存在，保留并按 manifest 核查。' >&2
    exit 1
  fi
  mkdir -p "$DEST/lib/linux/aarch64" "$DEST/lib/linux/x86_64"
  BASE=https://raw.githubusercontent.com/XT-Toffuture/xtsdk_cpp/3d3db067ae9bdc0528202c3087bc10fd3b706638/xtsdk/lib/linux
  curl --fail --location "$BASE/aarch64/libxtsdk_shared.so" \
    --output "$DEST/lib/linux/aarch64/libxtsdk_shared.so"
  curl --fail --location "$BASE/x86_64/libxtsdk_shared.so" \
    --output "$DEST/lib/linux/x86_64/libxtsdk_shared.so"
  printf '%s\n' \
    '3336a76590b5447efd7c037929e61c287523fcd79e8125589a3adb35eee83411  SDKs/xtsdk_filter_3d3db067/lib/linux/aarch64/libxtsdk_shared.so' \
    '69d80f0d64f1b7a65c1cf76aa57d7ba84fb0ff34a03014e6d04d52092e683821  SDKs/xtsdk_filter_3d3db067/lib/linux/x86_64/libxtsdk_shared.so' \
    | sha256sum --check -
)
```

如果下载中断，已有目录会保留。对照哈希确认后再处理未完成文件，不要为了重试删除其他 SDK。上游闭源过滤实现不能由本项目源码审查代替；详细边界见 [SDK 说明](../vendor_patches/README.md)。

## 5. 提取 OpenCV 4.5.4 并构建全部包

先检查包仓库能取得脚本固定的 `4.5.4+dfsg-9ubuntu4`。脚本通过 `apt-get download` 和 `dpkg-deb -x` 提取到工程 `SDKs/ubuntu_opencv45`，不执行包维护脚本，不替换系统 OpenCV。

```bash
cd /home/nvidia/wheelchair
bash scripts/prepare_opencv45.sh
source /opt/ros/humble/setup.bash

# 全部六个维护中的 ROS 包：
python3 scripts/wc_phase1 build
source install/main/setup.bash
```

构建入口使用 `--base-paths src`，包含：

| 包 | 用途 |
|---|---|
| `wc_interfaces` | 自定义消息与来源身份 |
| `wc_xt_driver` | 双雷达驱动、SDK 校验、原始与过滤后点云 |
| `wc_bringup` | Python 模块、启动入口、预览与手动驾驶窗口 |
| `wc_slam` | RTAB-Map 建图后端 |
| `wc_estimation` | `robot_localization` 官方库对照工作进程 |
| `wc_camera_panel` | RViz 四相机面板 |

历史 `scripts/build_verify.sh` 的显式包列表曾只构建其中四个包，不能把该历史列表作为完整的新机部署命令。构建输出 `build/`、运行产物 `install/` 留在本地工程；构建日志通过当前存储策略写入数据根的 `reports/builds/`。`tests/` 保留测试源码，不应为了清理生成物而删除。

每次在新终端手动运行 ROS 工具前：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/wheelchair/install/main/setup.bash
```

## 6. 恢复设备连接（离线重放可先跳过本节）

设备连接不在 Git 提交中自动恢复。重置同一台 Orin，原设备序列号通常仍相同，但 udev 规则、用户组和网卡地址需要重新设置；换一台 Orin 的 USB 拓扑可能改变。序列号、IP 和物理左右关系应一起确认。

### 6.1 雷达专用网卡

部署示例使用同一雷达专用接口的两个接收地址；`eno1` 是现机接口名，序列号是公开占位值，须替换为真实读回：

| 侧别 | 雷达设备 | 主机接收 IP | UDP 端口 |
|---|---|---|---:|
| 左 | `192.168.0.101` / `XTM60B00000000000012` | `192.168.0.100/24` | 7687 |
| 右 | `192.168.1.101` / `XTM60B00000000000013` | `192.168.1.100/24` | 7687 |

先查看 `ip -br address` 和 `nmcli connection show`。在 NetworkManager 中给**连接雷达的专用接口**配置上述两个静态地址，不设默认网关；保留原互联网 / SSH 接口。不要把某台机器的接口名、连接名直接套到新机，亦不要运行厂商例子来改雷达 IP。本项目默认保留设备成像配置。

### 6.2 串口别名与权限

当前 `/etc/udev/rules.d/99-smartwheel-*.rules` 属于系统配置，不在 Git 中；重置后必须恢复别名。先检查：

```bash
ls -l /dev/serial/by-id/
ls -l /dev/v4l/by-path/
# 用实际枚举节点替换占位值；此命令只读取属性：
udevadm info --query=property --name /dev/REPLACE_WITH_ACTUAL_NODE
```

下列是**待填写模板**，不能原样使用尖括号中的占位值。控制器及 IMU 的序列号要和 `ID_SERIAL_SHORT` 一致；公开配置示例分别为 `0000000014` 和 `0000000015`，不是可直接使用的实机身份。超声波适配器没有独立序列号，需核实物理 USB 路径。

```udev
# /etc/udev/rules.d/99-wheelchair-local.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", ATTRS{serial}=="<轮控制器USB序列号>", GROUP="dialout", MODE="0660", SYMLINK+="smartwheel_zlac8030"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="<H30_USB序列号>", GROUP="dialout", MODE="0660", SYMLINK+="smartwheel_h30_imu"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ENV{ID_PATH}=="<超声波实际USB路径>", GROUP="dialout", MODE="0660", SYMLINK+="smartwheel_ultrasonic"
```

确认模板后通过 `sudoedit /etc/udev/rules.d/99-wheelchair-local.rules` 保存。在没有进行录制或控制时重新加载规则，随后重新插接设备：

```bash
sudo usermod -aG dialout,video,render nvidia
sudo udevadm control --reload-rules
# 注销并重新登录，使用户组生效；再检查别名对应的 by-id 身份。
```

本项目还执行自己的身份检查，不是有了别名就可以采集任意同类设备。当前需要一起核对的配置和源位置：

| 要变更的身份 | 必须同步核查 |
|---|---|
| 雷达身份、接收地址 | `config/live_unvalidated.json`、`src/wc_xt_driver/config/`、会话设备读回，以及 `src/wc_runtime/cli.py::device_preflight` 中当前固定的 `eno1` 接口检查 |
| 轮控制器 USB 身份 | `config/wheel_feedback_current.json`、手动驾驶硬件配置及 `src/wc_motion/feedback_transport.py` 的 `SERIAL` 验证 |
| IMU USB 身份 | `src/wc_runtime/cli.py` 的 `H30_BY_ID/H30_SERIAL`、IMU 启动参数及 `config/mapping_live.json` 的 sensor ID |
| 相机角色与 USB 拓扑 | `config/cameras.json`；当前代码 `src/wc_cameras/config.py`、`capture.py`、`src/wc_runtime/capture_contract.py` 使用 Orin 的 `platform-3610000...3.N` 路径 |
| 超声波身份 | `config/ultrasonic_capture.json` 的 by-id、USB 路径和 VID/PID；无回复不等于零距离 |

换设备或载板时，不能只改一处配置，也不能取消身份检查。按实际枚举同步修改以上所有生产消费者及回归测试后重新编译 / 安装；保持左右与物理角色一致。当前尚未实现任意设备的一键注册向导。仅使用 `mapping_core` 时不启动、检查或记录相机和超声波。

## 7. 部署验收顺序

1. **源码 / 依赖 / 构建**：六个包成功构建，SDK 清单及 ABI 检查通过。
2. **软件测试**：在已 source 的终端执行 `python3 scripts/wc_phase1 test`；日志保存在所选存储目录。通过不表示设备已经连接或车体可以驾驶。
3. **只读环境检查**：执行 `python3 scripts/wc_phase1 doctor --profile mapping_core`，查看缺项。此检查不启动硬件，不是要求每次录制都手工预检。
4. **已有录包离线重放**：先从另行保存的数据中选择一份录包，依照 README / 操作手册执行 `compare` 或 `refine`；不连接真实设备即可检验处理链。
5. **短时静态采集**：设备接好后由用户启动 `mapping_core` 录制 30 秒，正常收尾，再执行 `doctor --session-root <该会话> --verify-archive`。
6. **人工驾驶与动态精度**：由现场用户验收启停和闭合路线。录包完整与地图几何精度是两项结论，不能互相代替。

如果只想确认软件恢复，先完成前四步；不需要为了部署测试自动使能电机或驾驶轮椅。常用录制、停止、预览和融合命令见 [README](../README.md) 与[V7 操作参考](v7_operations.md)。

## 8. 重置前需要另外备份的内容

在仍可使用的原系统上，先确认 GitHub 中是所需提交，再把以下内容保存到已经核验的数据盘或其他备份介质。不要把完整个人主目录、SSH 私钥、浏览器资料或含密码的系统连接配置上传 GitHub。

| 备份项 | 用途 / 保存边界 |
|---|---|
| 当前源码提交号及未提交的维护文件 | 避免只保存远程旧提交；必要时保存完整差异和新增源码，排除生成物 |
| `config/` 原文快照，特别是 `storage.local.json`、外参和原始来源文件 | Git 中的通用模板不能代替本机存储和装配状态；本机覆盖私下备份 |
| `SDKs/xtsdk_ros`、`SDKs/xtsdk_filter_3d3db067`、`SDKs/ubuntu_opencv45/packages` | 作为上游下载不可用时的独立恢复来源；保留 manifest 和哈希。仓库已附固定过滤库，但不包含完整厂商 SDK 或 OpenCV 包缓存 |
| `/etc/udev/rules.d/99-smartwheel-zlac8030.rules`、`99-smartwheel-serial.rules`（若有本机新规则也单独保存） | 恢复串口别名；新机仍要核对真实身份 |
| ROS / 系统依赖版本清单 | 记录 `/opt/ros/humble` 对应的 Debian 包版本；记录 `nvidia-l4t-core`、OpenCV、TBB、Python 版本和 SDK provenance |
| 仅相关网卡的接口名、静态 IP / 掩码及设备角色说明 | 恢复雷达网段；不导出含 Wi-Fi 密码或其他凭证的连接文件 |
| 数据盘 `data/`、`maps/`、必要的 `reports/` 与哈希索引 | GitHub 不保存这些原始录包和研究结果 |

只读获取版本和网络事实的示例：

```bash
git rev-parse HEAD
git status --short
dpkg-query -W 'ros-humble-*' nvidia-l4t-core libopencv-dev \
  libopencv-core4.5d libtbb2 python3.10 python3-numpy python3-scipy
ip -br address
```

将输出保存到已配置数据根的报告目录。对源码和 SDK 的备份保留真实文件与哈希；不要把旧 `install/` 拷到新系统后直接当作已适配的构建。克隆源码并重建可执行文件，更容易定位新系统的 ABI 和缺依赖问题。

## 9. 恢复范围速查

| 操作 | 能恢复什么 | 不能替代什么 |
|---|---|---|
| clone 当前发布提交 | 项目源码、当前几何配置、文档、测试源码、固定过滤库 | 系统、其余依赖库、安装产物、数据 |
| `configure_storage` | 数据目的地及本机覆盖配置 | SDK、构建、网卡、设备权限 |
| SDK + 依赖 + build | 可执行程序和 ROS 包 | 当前设备身份、USB 拓扑、现场运行验证 |
| 带回原 U 盘 / 原数据目录 | 原录包、地图、实验结果 | 新机系统部署及当前装配校核 |

完整迁移应保留 Git 提交号、另存数据的校验文件，以及本机存储 / 网络 / 设备配置记录。当前仓库是可重建项目，尚不能声称任意 Orin 一条命令完成整机恢复。
