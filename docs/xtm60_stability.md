# XT-M60 点云断流排查与稳定性

XT-M60 = **TCP 控制 + UDP 数据**双通道。点云走 UDP(无重传),断流通常是三层叠加:UDP 丢包/串流、SDK 自带重连(约 5–15s)、以及上层错误处理打断重连。

## 已做的代码修复(`xtm60_adapter_node.py`)
- **重连后自动重启测量**:`poll()` 在 `isconnect()==False` 时复位 `_measurement_started`,SDK 自连回来后(或事件 `cmdid=0xFE`)自动重新 `start()`。之前 `_measurement_started` 设一次就不复位,导致 SDK 重连后测量永不恢复 → 永久断流。
- **不销毁 SDK**:绝不在帧间隙 `shutdown()`/重建,避免打断 SDK 自身重连。
- **温和看门狗**:`isconnect()` 仍连着但 `frame_timeout_sec`(默认 8s)内无帧时,仅**重发 `start()`**(不销毁)来"踢"一下卡住的 UDP 流,有速率限制。
- 事件 `cmdid=0xFF`(断开)/`0xFE`(握手完成)分别复位/重启测量(对齐官方 `sdk_example_3d.py`)。

## 已做的系统层修复
- **UDP 接收缓冲**:`net.core.rmem_max=26214400`(原 208KB 太小,大点云帧会被丢弃)。已持久化到 `/etc/modprobe.d`?不——在 `/etc/sysctl.d/10-xtm60-rmem.conf`。
- (相机无关,见 calibration_checklist。)

## 两台雷达必做(避免 UDP 端口 7687 串流)
实机验证表明该版本 SDK 的本地接收 socket 固定绑定 `0.0.0.0:7687`。
两个进程会通过 `SO_REUSEPORT` 共享该端口,导致一路数据被分配到错误
进程,并产生某台雷达"有帧"的假象。

`wheelchair_bringup` 现在构建 `libxt_bindshim.so`,并由 `sensors.launch.py`
自动给左右 SDK 进程注入不同的本机绑定地址:
- 左: `192.168.0.100:7687`
- 右: `192.168.1.100:7687`

### 2026-06-06 端口对齐修复 + 右雷达硬件复核

之前右侧 `xtm60_right.yaml` 的 `udp_dest_port` 是 `7688`,而垫片把本地接收
socket 钉在 `7687`。设备被告知发往 7688、SDK 却在 7687 收 → 端口不匹配,
右雷达永远 0 帧。**已修复**:右侧 `udp_dest_port` 改为 `7687`,与垫片绑定端口
一致(左右两路靠不同网卡 IP `192.168.0.100` / `192.168.1.100` 隔离,端口同为
7687 不会串流)。

修复后实测:
- 左雷达 `192.168.0.101`:稳定 ~10Hz,`/xtm60/left/points` 正常。
- 右雷达 `192.168.1.101`:端口已对齐(`UDP dest set to 192.168.1.100:7687`),
  TCP 控制连得上、固件可读(`XTFW-RT-V2.34.1`)、`start()`/`setUdpDestIp()`
  均返回 True,但**仍 0 帧**。单独只启动右雷达(无双开、无垫片干扰)复现同样
  结果,设备信息为 `chip='0 0'`、状态停在 `Connected-Init`。

结论:右雷达不出数据**不是端口/双开/SDK 接收端的软件问题,而是该雷达硬件
故障**(传感器芯片未初始化,chip id 全 0)。复制 SDK + 改端口(方案二)也无法
解决,因为问题在设备侧不发包。需对右雷达**彻底断电重启**后重测;若 `chip`
仍为 `0 0`,需固件修复或返厂/更换。可把左雷达接到右子网做交叉验证,确认是
雷达坏还是右子网/线缆问题。不能将左侧误收的数据视为右侧点云。

## 硬件/网络检查单(官方文档要点)
- **供电**:M60 需 12V/3A 或 19V/3.42A,电压不足直接影响点云质量。
- **网线/交换机**:两台雷达直连同一千兆交换机,**不要经 WiFi 路由器**;换优质网线。
- **防火墙**:确保不拦截 UDP 7687(Linux 一般无 ufw;若开了 `sudo ufw allow 7687/udp`)。
- **隔离子网**:左 `192.168.0.101`、右 `192.168.1.101`,主机 `192.168.0.100/24`+`192.168.1.100/24`(见 `scripts/setup_radar_network.sh`)。

## 验收
```bash
ros2 launch wheelchair_bringup bringup_3d_slam.launch.py backend:=none use_colorizer:=false use_cloud_to_2d:=false
ros2 topic hz /xtm60/left/points --qos-reliability best_effort     # 应稳定 ~10Hz 不掉0
ros2 topic echo /points_merged/status --once --field data          # left_fresh:true, output_points>0
```
若仍间歇断:先确认单台稳定(拔掉另一台),再按上面开 UDP 端口分离 + 查供电/网线。
