# 正式 3D 地图：软件闭环、验收契约与人工收口清单

更新时间：2026-09-08

本文说明“正式 3D 地图”与当前 RViz 点云预览/离线实验地图的边界，并给出
可重复的启动、导出和验收方法。**任何一个必需门禁没有证据时，结果只能标记
为实验性或候选地图，不能标记为 `FINAL_PRODUCT`。**

## 已完成的软件工作

1. `formal_3d_mapping.launch.py` 是独立的 fail-closed 入口。它在构造 ROS
   节点前检查 canonical 双雷达 + IMU 校准合同、安装 epoch、三设备硬件身份、
   真实硬件 profile、双雷达、里程计模式、回环开关和单雷达回退。正式默认里程计模式是
   `contract_fastlio`；FAST-LIO 的 IMU/LiDAR 外参从同一合同计算，不再读取另一份
   临时 YAML 外参。当前默认合同仍为 `BLOCKED_CONFLICT`，因此默认启动会明确
   拒绝，而不会用 provisional 数值运行。
2. `rtabmap_3d_mapping.launch.py` 现在检查 topic/frame/队列/里程计模式，显式
   设置点云与里程计 QoS，关闭深度/RGB 隐式同步，并让 `rtabmap_viz` 不发布 TF。
   这解决了“两个节点争抢同一条 TF 边”和 LiDAR-only 等待相机数据的问题。
3. RTAB-Map 后端点云由 `rtabmap_optimized_cloud_node` 从同一个优化 pose graph
   快照重新组装，并同时发布 `/rtabmap/optimized_cloud` 与完整 6DoF
   `/rtabmap/optimized_path`。它支持压缩 cv::Mat 和原始 PointCloud2，保留真实
   XYZI intensity；XYZ-only 与 XYZI 混合的数据库会拒绝，不会以全零强度
   冒充反射率。
4. `map_products_node` 在正式入口中必须收到 `/rtabmap/optimized_cloud` 和
   同长度的 intensity，才允许导出几何/PointCloud+Amp 产品。输入点云与 odom
   的短暂回调乱序由有界 pending 队列处理，队列溢出会被记录而不是静默丢失。
5. 导出目录包含原子写入的几何 PCD/PLY、PointCloud+Amp PCD/PLY、完整 6DoF
   轨迹、二维 side product、硬件/算法 profile、质量报告、实际使用的
   `calibration_contract_used.json`、`formal_acceptance.json` 和 SHA-256 manifest。
   `hardware_validated=true` 时，writer 会对已经复制进 bundle 的 profile、合同和
   evidence 再做一次身份/epoch/合同 SHA 绑定校验，通过后才生成 manifest；失败
   保留 `.incomplete`。符号链接、非有限坐标/强度或哈希不一致也会被拒绝。
6. `validate_formal_3d_map.py` 与无 ROS 依赖的
   `smartwheel_map_products.formal_acceptance` 检查：后端来源、文件结构与点数、
   intensity 对齐、真实 profile、完整停止会话、三方硬件身份、设备时钟、外参合同、
   动态场景、实时性、精确 TF 边、双雷达、回环和重复性。任何降级 CLI 开关
   都会保留 `formal_ready=false`；退出码非 0 表示阻塞。运行证据在导出瞬间
   重新读取，并与固定的 profile/合同路径重新校验，避免把节点启动时的旧
   evidence 快照或校验后被替换的错误来源写成完整产品。
7. `scripts/save_mapping_result.sh` 现在按有界流程暂停 RTAB-Map、停止采集、等待
   stop 后的新 backend cloud/path 同快照、调用导出，并核对本次
   `/map_export/completed` 路径、目录、`.incomplete` 与 manifest；任一步失败均
   非零退出。

## 推荐的正式运行顺序（前置条件满足后）

仓库提供 `config/hardware_profile.template.yaml` 和
`config/formal_dual_lidar_imu_contract.template.json`。二者故意保留空 epoch、
空硬件 ID/外参与 `BLOCKED_CONFLICT`，不能直接用于正式启动；应复制到本次验收
目录，填入现场核对值和证据，再由独立复核者把合同改为 `APPROVED`。不要修改
模板本体来伪装已验收状态。

在 Orin 上：

```bash
cd /home/nvidia/smartwheel
source /opt/ros/humble/setup.bash
source install/setup.bash

# 先生成一个唯一数据库和输出目录；不要复用旧数据库。
DB="$HOME/.ros/formal_3d_$(date +%Y%m%d_%H%M%S).db"
OUT="$HOME/maps/formal"

ros2 launch wheelchair_3d_mapping formal_3d_mapping.launch.py \
  calibration_contract:=/absolute/path/to/approved_dual_lidar_imu_contract.json \
  hardware_profile:=/absolute/path/to/approved_dual_real_profile.yaml \
  points_topic:=/formal/points_merged \
  left_points_topic:=/xtm60/left/points \
  right_points_topic:=/xtm60/right/points \
  odom_mode:=contract_fastlio \
  formal_lio_radar:=right \
  odom_topic:=/Odometry \
  database_path:="$DB" \
  delete_db_on_start:=false \
  output_root:="$OUT" \
  map_name:=formal_dual_3d \
  hardware_validated:=true \
  evidence_path:=/absolute/path/to/formal_acceptance.json \
  bringup_sensors:=false
```

`database_path` 没有可复用默认值，必须显式给出绝对路径，而且该文件必须尚不
存在。正式入口固定 `delete_db_on_start:=false`；它不会替操作者删除或覆盖旧
RTAB-Map 数据库。`output_root` 也必须是绝对路径、可写目录或位于可写父目录下。

`bringup_sensors:=false` 表示左右 XT-M60 与 H30 适配器必须已在同一 ROS domain
中由经过预检的入口提供；上面的默认 `contract_fastlio`、严格双雷达融合、
RTAB-Map 和导出器仍由正式入口启动。缺少 `/xtm60/left/points`、
`/xtm60/right/points` 或 `/imu/data` 时不会产生有效地图。若明确要求入口自己
启动传感器，只有在合同已批准、profile 已核对且现场授权后才能设为 `true`；
内置适配器将被强制使用严格设备时间路径，XT-M60 时间不是 epoch 或 H30 缺少
设备时间 TLV 时会丢帧而不是回退到 ROS `now()`。

### 硬件身份三方绑定

正式 profile 与 calibration contract 必须各自包含完全相同、规范化后相等的
`hardware_identity`：`schema_version=1`、与文档顶层相同的
`installation_epoch`，以及恰好 `xtm60_left`、`xtm60_right`、`imu` 三个资产。
左右 XT-M60 使用厂家 serial 作为 `hardware_id`，当前已知绑定分别为
`XTM60B20250324000151` 和 `XTM60B20250324000134`；H30 不虚构厂家 serial，使用
操作者确认并持续维护的 `installation_asset_id`。每项还必须记录固定 role、model
和 frame；雷达另外记录 IP。三个 ID 和两个 IP 都不能重复。

`formal_acceptance.json.gates.hardware_validation` 除 `status=PASS` 外，还必须记录
同一 `installation_epoch`，并在 `observed_assets` 中为三个同名 role 分别提供：
实际观察到的 `hardware_id`、`observed_model`、`observed_identity_kind`、
`observed_frame_id`、非空 `identity_source` 和布尔值 `match=true`；两只雷达还
必须提供 `observed_ip`。IMU 可用 `operator_asset_tag` 作为来源。左右 serial
互换、H30 资产号缺失/改变、任何 epoch 不一致，或只有 PASS 字样而没有逐资产
观察值，启动器、exporter 构造/导出门禁和最终 validator 都会拒绝。evidence 的
extrinsics gate 还必须绑定当前合同 SHA-256；只改 PASS 字样或换一份同名合同都
不能通过。IP 只是连接配置的辅助一致性项，不能单独代替硬件身份；`/dev/tty*`
也不能作为 H30 身份，因为 USB 重连会改变它。

映射期间必须确认：

- `/Odometry` 实际连续发布；默认模式下由合同驱动的 FAST-LIO 提供；
- 唯一动态 TF 链与 evidence 都必须精确记录为：RTAB-Map
  `map -> camera_init`、FAST-LIO `camera_init -> body`、合同派生静态桥
  `body -> base_link`，每条边恰好一个 publisher；
- `/formal/points_merged` 是严格同步的双雷达 `base_link` 帧 XYZI 输入；缺任一路、
  时间戳为零、配对差超过 profile/20 ms 上限或缺 intensity 都必须拒绝；
- `/rtabmap/optimized_cloud` 已出现至少两个优化 keyframe，且字段为
  `x,y,z,intensity`；
- 不要在同一会话中删除/追加旧数据库；
- 正常停止使用 `scripts/save_mapping_result.sh`。它会先尝试暂停 RTAB-Map，再
  调用 `/map_session/stop`，等待 stop 后的新 optimized cloud/path 同快照，随后
  导出并核对 `/map_export/completed` 与 manifest。若需在停止后补写本次证据，
  应先 stop、原子替换 evidence，再单独调用 export；导出器会重读并重验该文件，
  writer 还会验证 bundle 内实际复制的三份来源后才发布 manifest。
- 若中途失败，不得手工删除 `.incomplete` 或把部分目录改名为正式地图。

标准停止/导出：

```bash
bash scripts/save_mapping_result.sh
```

导出后运行：

```bash
python3 scripts/hardware/validate_formal_3d_map.py \
  /absolute/path/to/exported_bundle \
  --calibration-contract /absolute/path/to/approved_dual_lidar_contract.json \
  --evidence /absolute/path/to/formal_acceptance.json \
  --json \
  --report-out /absolute/path/to/formal_validation_report.json
```

只有输出中的 `formal_ready: true` 且所有 `checks.*.passed` 都为 true，才可把
目录交付为正式地图。`--single-lidar`、`--no-loop-closure`、
`--no-repeatability`、`--no-intensity` 只适合生成诊断报告；当前验证器会显式写入
`formal_scope` blocker，因此这些调用不可能得到 `formal_ready: true`。

## 验收门禁（证据必须机器可读）

`formal_acceptance.json` 使用 schema 1。顶层还必须包含非空 `evidence_id`、
`generated_at_utc`、`source_artifacts` 和已确认的 `operator_attestation`；所有 gate
的 `status` 必须明确为 PASS，缺失值不会被推断为通过。验证器核对的是这些门禁
存在机器可读的 PASS 证据及其数值/合同绑定；它不会仅凭最终 PCD 反推出运行时
TF publisher、真实回环或重复路线。

| 门禁 | 默认要求 | 证据内容 |
|---|---|---|
| 硬件 | `hardware_validated=true` 且 gate PASS | 同 epoch 的逐资产 observed ID/model/kind/frame、雷达 IP、identity source、match=true、配置快照、启动/停止记录 |
| profile | real 双 XT-M60 + H30 | canonical hardware identity、两个不同 IP、`point_unit=m`、intensity 字段、非 host 时钟 |
| 双雷达 | 左右均 PASS | 每路有效点比例、频率、温度、同步/重叠统计；当前短测不能直接替代长期证据 |
| 时间 | 非主机接收/插值时钟、单调、最大偏差 ≤20 ms | 设备时钟行为、单位、回绕、固定偏移及置信区间 |
| 外参 | canonical 合同 `APPROVED` 且 transform 有限 | schema 1、`scope=dual_lidar_imu`、安装 epoch；`base_link` 到左右雷达/H30 的 xyz、roll/pitch/yaw，合同 SHA 与独立验证 |
| 动态 | straight、turn、in_place、stop_recovery 全 PASS | 每段的动作标记、轨迹/残差/拒帧/恢复结果 |
| 实时性 | 默认最长输出空窗 ≤0.5 s、丢帧率 ≤1%、持续 ≥60 s、≥100 输出样本 | A/B 负载测试原始报告；包括尾部静默 |
| TF | canonical 三边各恰好一个 publisher | `map->camera_init=1`、`camera_init->body=1`、`body->base_link=1`；证据列出 runtime graph artifact |
| 回环 | 至少 1 个实测闭环 | RTAB-Map Info/数据库 keyframe 与闭环 ID |
| 重复性 | 至少 3 次 | 相同路线的地图/回环误差及环境说明 |
| 产品 | 后端优化云、几何/强度点数一致 | PCD/PLY header、quality、manifest 哈希和重载结果 |

当前验证器还会拒绝：`mock_lio=true`、未停止会话、stop 前或 cloud/path 不同
backend 快照、混合 frame、超出时间窗的拒绝帧、缺少 sidecar、符号链接、
未列入 manifest 的文件、非有限/全零几何、非有限/负值/全零 intensity、
非完整 6DoF 或乱序轨迹、`host_receive`/`host_interpolated` 正式采样时钟、
profile/合同 installation epoch 或 canonical hardware identity 不一致、逐资产观察
证据缺失/调换，以及 `BLOCKED_CONFLICT` 合同。

## 当前可引用的证据与限制

- 2026-09-07 双 XT-M60 原始 SDK 并发 60 秒测试已完成并正常关机：左 512、右
  520 帧，回调丢帧为 0；SDK 时间约 10 Hz。按“有限 XYZ + 有限非负幅值”计算的
  平均有效像素约左 94.96%、右 94.18%，但右路低质量帧较多，且这是约 0 ms
  启动相位的原始测试，不等同于生产 phase grid 或动态验收。证据目录为
  `auto_test/dual_radar_validity_20260907_190000`，两路 UDP 已确认静默。
- 这次短测没有证明设备采样时钟、最终外参、动态 FAST-LIO2、RTAB-Map 回环、
  高负载实时性或地图几何精度；不能把它写入 `formal_acceptance.json` 的 PASS
  gate。
- 当前右雷达合同仍是 `BLOCKED_CONFLICT`；H30 生产时间线是主机侧插值估计，
  不是设备采样时刻；左路长期有效率/链路仍需单变量物理排查。
- 已有 H2 离线完整回放是 `H2_OFFLINE_FULL_REPLAY_PASS_WITH_TIME_WINDOW_NOTE`，
  它证明离线 2D/导出软件链可工作，不是 FAST-LIO2 或 RTAB-Map 3D 正式验收。

## 需要你最后人工配合的事项（按顺序）

这些步骤不能由代码安全地代替；在完成前保持合同为 BLOCKED、
`hardware_validated=false`，只生成候选/实验地图。

1. **确认 H30 安装资产号与三设备身份**：为当前 H30 指定并在设备/安装记录中
   固定一个稳定资产号，现场核对左右 XT-M60 铭牌 serial 与上述 role；生成一次
   带来源和安装 epoch 的只读身份快照。不要使用 `/dev/tty*` 充当资产号。
2. **左雷达硬件链单变量 A/B**：冷却并断电，检查/清洁窗口，再分别隔离供电、
   线缆/接口、设备本体；每次只改一个变量，记录左右有效率和温度。不要先改
   软件质量门或再次启用整套 SDK filter chain。
3. **真实采样时间与固定延迟**：用明确的慢速双向 yaw、前后静止和机器可读
   marker，验证 XT-M60/H30 的时钟来源、单位、重启/回绕和长期漂移；只有两种
   方向得到一致偏移，才填写合同/配置。
4. **最终外参**：测量 `base_link` 原点、两雷达光学中心和 H30 xyz；保留你已确认
   的左右约 0.60 m 横向间距、相对 fore/aft 为 0，不用地图外观反推未测的 x。
   用地面 + 两面不平行竖墙 + 独立验证求 roll/pitch/yaw；更新一份合同并同步
   URDF/FAST-LIO 配置，不能三处各存一套数值。
5. **动态 H3**：在无人、清场、急停可达和操作者在场条件下，依次采集直行、
   转弯、原地转向、STOP 恢复和一次受控短空窗；确认合法运动不会被整帧门禁
   长期拒绝，且 ZUPT 只在健康轮速/连续静止窗口触发。
6. **高负载 H4**：同一路线做“不录 bag / 最小 bag / 加 registered cloud”三轮，
   比较 Odometry/cloud 最大空窗、尾部静默、CPU/磁盘和单帧跳变；不能用放宽
   5 cm/2° 门限来掩盖实时性问题。
7. **RTAB-Map 回环与重复性**：完成至少一个小闭环并独立重复三次，检查墙面
   重影、地面倾斜、闭环误配和数据库/优化云重载；把原始日志、数据库、bag、
   TF 快照和最终验证报告一起归档。
8. **最终签字/放行**：由操作者确认所有 gate 的来源、三设备身份和安装 epoch 后，才将合同
   状态改为 `APPROVED`、生成正式 evidence，再运行验证器。任何人工直接把
   `hardware_validated` 或 gate 改成 PASS 而没有原始证据，都不算验收。

电机地面行驶、物理急停闭环和载人安全属于更高等级测试，不是本软件改动的
默认步骤；需要另行授权和安全方案。
