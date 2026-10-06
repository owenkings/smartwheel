# V7 双雷达轮椅采集与离线建图

本分支 `v7/orin-usb-20261006` 保存当前 V7 工程：双 XT-M60、H30 IMU、轮反馈采集，轮速/IMU 平面估计、双雷达运动补偿、RTAB-Map 建图，以及同一录包上的算法比较。它与仓库 `main` 的早期系统分别保存。

代码运行于 Orin；原始录包、地图和测试结果保存在挂载 USB 的 `wheelchair/` 目录。公开仓库只包含源码、配置示例、测量参数来源、使用说明与回归测试代码。**公开设备绑定均为示例值，首次部署先读 [PUBLICATION.md](PUBLICATION.md)。不要用公开模板覆盖现有 Orin 的现场配置。**

## 目录与存储

```text
/home/nvidia/wheelchair/       # Orin 代码、构建与设备入口
USB/wheelchair/
  data/experiments/<session>/  # 一轮原始录制及配置、身份、时间和完整性清单
  data/analysis/               # 离线比较和派生结果
  maps/                       # 保存的地图
  reports/                    # 检查、测试、迁移和历史研究报告
```

挂载点和文件系统 UUID 使用 Orin 的本地存储配置。更换 USB 时重新配置并核实身份；仅存在同名目录不代表 USB 已挂载。录制中的内存暂存不等于已经持久保存，必须等待转存和核验结束。

## 录制一轮数据

以下在已经完成本地设备绑定和 USB 配置的 Orin 图形桌面终端执行：

```bash
cd /home/nvidia/wheelchair
SESSION="v7_core_$(date +%Y%m%d_%H%M%S)"
python3 scripts/wc_phase1 capture \
  --profile mapping_core --session "$SESSION" --duration 300 \
  --manual-drive --preview
```

`mapping_core` 记录左右雷达独立 raw/filtered 点云、IMU 与轮反馈，不启动摄像头或超声波。若需要四相机及四地址超声波，显式选择 `all_sensors`；缺失设备不会被自动当作完整。

`--manual-drive` 提供操作者键盘/手推流程；采集本身不启动融合或 SLAM。`300` 为请求时长上限，提前结束按 **Ctrl+C 一次**，等待停止来源、转存、核验和最终结果。不要在 `TRANSFERRING` 阶段直接关机或拔 USB。手动提前结束不等于采满请求时长，原始状态会保留。

预检是可选诊断：同命令追加 `--dry-run` 不采集。正式启动仍保留设备身份、独占读取和实际剩余空间保护。完成后，用实际 USB 路径复核：

```bash
USB_ROOT="/media/nvidia/WHEELCHAIR_DATA/wheelchair"  # 换为本机挂载点
python3 scripts/wc_phase1 doctor \
  --session-root "$USB_ROOT/data/experiments/$SESSION" --verify-archive
```

每轮的 `configuration/`、`software/`、`bag/`、`sources/`、事件与根目录清单相互关联。左右雷达身份和帧索引分开；无效响应不作为零距离，显示抽帧不等于归档抽帧。

## 同一数据反复融合、比较

```bash
cd /home/nvidia/wheelchair
USB_ROOT="/media/nvidia/WHEELCHAIR_DATA/wheelchair"  # 换为本机挂载点
DATASET="$USB_ROOT/data/experiments/你的会话名"
RESULT="$USB_ROOT/data/analysis/compare_$(date +%Y%m%d_%H%M%S)"
python3 scripts/wc_phase1 compare \
  --dataset "$DATASET" --output "$RESULT" \
  --estimators five_state robot_localization \
  --cloud raw --filter on --input-rate-hz 0 \
  --mechanical-initial --native-map
```

`--input-rate-hz 0` 处理全部有效配对帧，改变的是离线消费上限，不是雷达设备频率。RTAB-Map 原生地图还有独立 `--native-rate-hz`，默认 1 Hz；比较时保持其他条件相同。后续可单独比较 `--filter off on` 或 `--input-rate-hz 5 10`，避免一次修改多个因素后混淆原因。

`--mechanical-initial` 明确使用已有 V7 机械安装初值，只适用于离线实验，不代表几何标定完成。只有接纳已知缺段/提前停止等限制的研究时才追加 `--allow-partial`；该参数不修改原始档案状态。默认五状态 EKF 与 `robot_localization` 对照不会自动更换现场默认估计器。无独立真值时不报告绝对定位精度。

## 查看 3D 地图

将 `EXPORT` 改为比较结果中某个 `native/<方案>/export` 目录：

```bash
cd /home/nvidia/wheelchair
EXPORT="/media/nvidia/WHEELCHAIR_DATA/wheelchair/data/analysis/你的结果/native/你的方案/export"
python3 scripts/view_map "$EXPORT" --check
python3 scripts/view_map "$EXPORT"
```

在 Orin 上打开 RViz。左键拖动旋转，中键拖动平移，滚轮缩放；取消勾选 `Saved 2D map` 可仅看三维。默认软件渲染是兼容回退，可按机器情况选择 `--renderer system`。查看器不更改原始地图。

## 查看左右点云是否重合

已有准备好的 `prepared.json` 时：

```bash
cd /home/nvidia/wheelchair
bash scripts/align_lidar_clouds.sh \
  --input /实际USB路径/wheelchair/reports/某次配准/prepared.json \
  --scene-index 0 --duration 7200 --no-browser
```

在同一台 Orin 的浏览器打开 `http://127.0.0.1:8767/alignment`。页面从指定输入读取左右原生点云与初始变换；切换前视、俯视和侧视检查共同墙面、桌沿的双层/错位。局部 ICP 与手动移动产生候选，不会自动替换正式外参。终端按 Ctrl+C 停止网页服务。准备输入和变换约定见 [离线点云配准说明](docs/offline_cloud_alignment.md)。

## 构建与验证

设备读写、UI 与点云运算在 Orin 执行。先按 [SDK 固定版本与补丁说明](vendor_patches/README.md) 准备依赖，再使用项目现有构建流程；不要自动替换正在使用的 SDK。环境与功能说明见 [V7 使用说明](docs/v7_operations.md)、[硬件配置说明](docs/operations/hardware_setup_zh.md)。

`tests/` 保留合成、单元及现场验证脚本；现场脚本不作为无硬件测试直接批量执行。验收应分别记录构建、合成测试、真实录包回放、现场静态和用户驾驶结果。地图文件可打开与录制完整，均不等于动态几何精度已经通过。
