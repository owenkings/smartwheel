# RTAB-Map 3D mapping pipeline contract

更新时间：2026-09-08

本文定义项目中 RTAB-Map 3D 软件管线的输入、输出、TF 所有权和保存边界。
它是软件接口契约，不是 LiDAR/IMU 外参批准书，也不把任何 provisional
实机结果标记为正式产品地图。

## 处理链

```text
PointCloud2 (sensor frame, XYZI)
  -> strict dual_lidar_cloud_fusion_node in the formal entry
  -> points_topic (formal default /formal/points_merged, base_link)
  -> contract-derived FAST-LIO /Odometry (formal default)
     OR an explicitly pre-started external odom owner
  -> rtabmap_slam/rtabmap
  -> /rtabmap/cloud_map       (backend display/occupancy cloud)
  -> /rtabmap/mapData         (pose graph)
  -> rtabmap_optimized_cloud_node
  -> /rtabmap/optimized_cloud (global keyframe cloud, map frame)
  -> /rtabmap/optimized_path  (same snapshot, full 6DoF trajectory)
  -> map_products_node         (exported geometry/intensity products)
```

`/rtabmap/optimized_cloud` 与 `/rtabmap/optimized_path` 来自同一个 backend
snapshot，分别是正式导出的几何、轨迹来源。local odometry accumulator 不得
替代二者。
停止采集后必须收到一组更新且匹配的 cloud/path；快照早于停止边界、两者 stamp
不同或轨迹尾部落后于采集尾部时，导出失败关闭。

## TF ownership

通用 `rtabmap_3d_mapping.launch.py` 仍可用于 ICP/外部里程计实验；正式入口固定
核对以下实际链路，每条边必须恰好一个 publisher：

| 角色 | 正式边 | Owner |
|---|---|---|
| global correction | `map -> camera_init` | `rtabmap_slam/rtabmap` |
| local odometry | `camera_init -> body` | 合同驱动 FAST-LIO，或显式且等价的外部 owner |
| body bridge | `body -> base_link` | `formal_fast_lio.launch.py` 从同一校准合同计算的静态桥 |
| sensor mounts | `base_link -> xtm60_left_link / xtm60_right_link / imu_link` | `formal_3d_mapping.launch.py` 从同一 canonical 合同发布 |

`rtabmap_viz` 只负责可视化且 `publish_tf=false`。FAST-LIO、轮速里程计、EKF 与
RTAB-Map 不得同时发布同一边。正式启动器会拒绝非 canonical 边参数，导出 profile
也写入三条精确边；最终 PASS 仍必须由运行时 TF graph artifact 证明，而不能只看
launch 源码。

## Input and QoS rules

- `points_topic` and `odom_topic` must be absolute ROS topic names.
- 通用入口的 `odom_mode` 可为 `icp` 或 `external`。正式入口对 RTAB-Map 始终使用
  external 输入，但上层 `odom_mode` 可为默认 `contract_fastlio` 或显式
  `external`；无论哪种模式都必须产生 `/Odometry` 并符合上述 canonical TF 链。
- 正式 `database_path` 没有默认文件，必须是尚不存在的绝对路径；
  `delete_db_on_start` 必须为 `false`。已有数据库、空/相对路径一律在构造节点前
  拒绝，不执行覆盖或追加。`output_root` 必须是绝对且可写的目录/父目录。
- 正式入口无论是否自启传感器，都拥有一份专用严格双雷达 fusion。它要求左右
  输入、非零时间戳、XYZI、同步配对并禁止单雷达 fallback；不得信任不明来源的
  通用 `/points_merged`。
- 正式 hardware profile 与 calibration contract 必须包含规范化后完全相同的
  `hardware_identity`：schema 1、同一 installation epoch、恰好
  `xtm60_left/xtm60_right/imu` 三个 role、唯一 ID 和固定 frame。XT-M60 以厂家
  serial 为身份，IP 只作配置一致性；H30 以稳定 installation asset ID 为身份，
  不使用易变的 `/dev/tty*`。启动前 evidence 还必须逐 role 记录 observed
  ID/model/identity-kind/frame、identity source 和 `match=true`，雷达还必须记录
  observed IP；否则即使 gate 写为 PASS 也会拒绝。
- `bringup_sensors=true` 只在已授权的硬件会话使用，并强制 XT-M60 `sdk_epoch`
  与 H30 必需设备时间；时间字段不满足时丢帧，不能回退到 host receive/publish。
- Point clouds use sensor-data/BEST_EFFORT QoS (`qos_scan=2`), while odometry is
  reliable (`qos_odom=1`). The launch dictionary supplies these values explicitly
  because parameter-file loading differs between Humble installations.
- The launch keeps `subscribe_depth=false` and `subscribe_rgb=false` unless RGB
  is explicitly requested. Camera timing must not block the LiDAR geometry path.
- A LiDAR-only run must keep `subscribe_scan_cloud=true`; disabling it without
  an RGB input is rejected before any node starts.

## PointCloud+Amp preservation

RTAB-Map may return a keyframe scan as either a compressed cv::Mat or an embedded
`sensor_msgs/PointCloud2`. `rtabmap_optimized_cloud_node` supports both forms.
For XYZI/XYZINormal/XYZIT/XYZIRT formats it carries channel 4 through optimized
poses and deterministic voxel reduction. RGB and normal channels are never
interpreted as amplitude. A database mixing XYZI and XYZ-only keyframes fails
closed rather than padding missing amplitudes with zero. An XYZ-only result is
published without a fabricated intensity field and is reported in diagnostics.

The resulting `/rtabmap/optimized_cloud` must be checked for `x,y,z,intensity`
before claiming a PointCloud+Amp product. The exact XT-M60 amplitude LUT is not
inferred by this pipeline; values are retained as supplied by the sensor.

## Database and map products

Use a unique writable `database_path` for each mapping session. Keep the RTAB-Map
database with the bag, launch arguments, hardware profile, TF/calibration snapshot,
and algorithm version. The optimized-cloud assembler requests
`global_map=true`, `optimized=true`, and `graph_only=false` from
`/rtabmap/get_map_data`.

The custom exporter writes a versioned bundle and a manifest. 正式收口使用
`scripts/save_mapping_result.sh`：若 RTAB-Map 提供 `/rtabmap/pause`，先验证其
`std_srvs/srv/Empty` 类型并暂停；然后有界调用 session stop、等待 stop 后的新
backend cloud/path，同步导出，并核对 transient-local `/map_export/completed`
路径、`.incomplete` 和 manifest。导出器会在写 bundle 前重读 evidence 文件，
并重新校验固定的 profile/合同与现场身份。`hardware_validated=true` 时，writer
还会对 bundle 内已经复制的 profile、合同和 evidence 检查 epoch、逐资产身份与
合同 SHA；只有该快照通过才生成 manifest。失败只留下 `.incomplete`，不会发布
完整产品，因此停止后原子更新的本次证据不会被启动时旧快照替代，源文件替换
也不能绕过导出门禁。

Occupancy map saving must use the backend-specific tested saver;
`slam_toolbox`'s in-process Humble `SaveMap` callback is not interchangeable with
RTAB-Map. Reload is a separate acceptance step and must verify both the occupancy
files and the pose graph/database.

## Acceptance boundary

This contract makes the software path reproducible and fail-closed. Validator
会解析 ASCII PCD/PLY 数值、完整 6DoF 轨迹、profile、bundled calibration contract、
evidence provenance 和 manifest，并拒绝任何降级 CLI 调用成为 formal-ready。
它不会独立从最终点文件重建运行时 TF、真实回环或重复路线；这些仍需绑定原始
artifact 的机器可读证据。

`hardware_profile_used.yaml`、`calibration_contract_used.json` 和
`formal_acceptance.json` 都由 manifest 覆盖。validator 会将前两者的 canonical
hardware identity 精确比较，再将 evidence 中同 epoch 的三个 observed ID、
model/kind/frame 以及雷达 IP 与之逐项比较；左右 serial 调换、重复 ID、缺少
H30 asset ID、frame 或 identity source，以及 evidence 绑定的合同 SHA 与实际
复制合同不同，均 fail-closed。相同规则同时用于启动前、导出瞬间、writer 的
bundle 快照和最终 validator，避免各层策略漂移。

本契约不批准当前 `BLOCKED_CONFLICT` 外参、尚未证实的设备采样时间、动态
FAST-LIO2、双雷达最终外参、闭环重复性或地面/载人操作。上述证据全部通过前，
只能生成 candidate，不能标记为 `FINAL_PRODUCT`。
