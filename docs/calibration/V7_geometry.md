# V7 机械资料、数据外参与能力检查

本次升级将用户安装文档作为机械初值导入，不把包络面中心当成数据原点。默认仍可原生预览、采集和开展标定；轮速＋IMU运动融合、双雷达几何融合等依赖未知关系的能力会给出具体阻塞原因。

## 来源与已经完成的迁移

- 唯一运行入口：`config/hardware_setup.json`，现在是 schema 2。
- 原始资料逐字节快照：`config/calibration/sources/sensor_installation_20261002.md`。原文件为用户提供的 `传感器安装位置与距离.md`，SHA256 `b2b63b7b2c59c8dc0c48755b1cf4f17baee684b14f384961c09d827e136ec46e`。文档更新日期为 2026-10-02，不代表实物已经核对。
- 迁移前配置逐字节保留：`config/calibration/legacy_hardware_setup_before_v7.json`，SHA256 `5f3868a6827635933347949d9e7e7c4d1703ecd066eeefb825ae4f8bdc43715c`。它是旧安装候选的独立 profile，`active=false`；旧会话可读取 schema 1，V7 未知项不从它填值。
- 所有 11 个传感器参考点、外形尺寸/中心、名义朝向、两侧 CAD→M 转换、人工机械读数和 IMU 芯片封装中心均保留，带原文行号。导入器会重新检查 21 行距离算术。来源检查还比较 11 个点与两侧 CAD 转换，防止文档哈希未变却手动抄错配置。

## 三层关系与单位

`T_A_B` 始终把 B 坐标变换到 A：`p_A = R_A_B p_B + t_A_B`。

1. **机械资料**：`mechanical_reference` 描述 M 系，单位 mm，状态始终是 `CAD_NOMINAL`。雷达/摄像头/双孔模块的点是朝外包络面中心；IMU 点是底面中心。它们不是光学/测量原点。CAD→M 是正确旋转加平移，不能用负行列式反射矩阵给左传感器做坐标变换。
2. **数据外参**：`data_transforms` 描述真正数据坐标系，单位 m，方向只能为 `child_to_parent`。平移与旋转各有值、状态、原点/轴语义、证据及说明。缺值须保留 null。
3. **车体关系**：M 到轮轴中点的 `T_axle_M` 单独记录；M 不等于 axle。数据外参可通过 M 连接，也可直接实测/标定到 axle 或另一传感器，不必猜出外壳内部的测量中心。

例如完整关系可由 `T_axle_lidar_left = T_axle_M × T_M_lidar_left` 得到。双雷达可先直接标定 `T_lidar_left_lidar_right`；这不需要 axle 已经测好。相对标定通过不意味着轮轴杆臂、IMU方向或地面高度已经验证。

旋转以完整 3×3 矩阵存储，检查正交和 `det=+1`；不允许缩放、镜像、倾斜矩阵、字符串数字或布尔数字。若外部工具输出欧拉角，先按明确的 `Rz(yaw) Ry(pitch) Rx(roll)` 转成矩阵。多个已启用变换路径必须一致，否则拒绝含歧义的图。

## 状态与启用条件

| 状态 | 含义 | 可用于数据融合 |
|---|---|---|
| `UNKNOWN` | 未知，值必须 null | 否 |
| `CAD_NOMINAL` | 设计初值，不能替代数据原点/实测安装轴 | 否 |
| `LEGACY_CANDIDATE` | 历史候选，不认证当前装配 | 否 |
| `USER_MEASURED_EXPERIMENT` | 本装配下明确测量，来源为几何测量或标定记录 | 允许实验，但不认证精度 |
| `CALIBRATED` | 本装配下标定记录，来源类型必须为 `EXTRINSIC_CALIBRATION` | 允许实验，仍需独立验证 |

可用的来源必须匹配 `assembly.revision`。原机械文档的证据类型为 `CAD_AND_MANUAL_MEASUREMENTS`；不能仅将其坐标重标为 `USER_MEASURED_EXPERIMENT` 就当成数据外参。新测量/标定结果需要独立来源记录、项目内快照路径和 SHA256。源文件不存在、被改动、路径逃出工程或机械目录与原文不符时，来源检查失败，不回退到原路径/旧候选。

`assembly.confirmed` 表示实际版本与装配状态经核对；当前默认 false。它不自动证明标定精度。当前轮参数继续明确为 UNVALIDATED，与几何门控分开记录。

## 能力级门控

| 能力 | 几何依赖 |
|---|---|
| `native_preview` / `native_capture` | 不需要安装外参；硬件身份、占用和可用性另查 |
| `mechanical_layout` | 已解析的 M 系标称资料；不可附着为数据 TF |
| `camera_intrinsic_calibration` | 不需要车体外参；需固定原生图像处理方式 |
| `imu_native_bias_calibration` | 不需要平移或车体旋转；结果保留 IMU 原生轴 |
| `lidar_relative_calibration` | 允许先采集标定，不需要已知 axle 外参 |
| `dual_lidar_static_fusion` | 已测/标定的数据系 `T_lidar_left_lidar_right` |
| `wheel_imu_motion_left/right` | 所选 `T_axle_lidar`、`R_axle_imu_native`、实际装配确认 |
| `wheel_imu_motion_all` | 两侧完整 axle 外参、IMU 安装旋转、实际装配确认 |
| `ground_height_left/right/all` | 相应 axle 外参及装配确认；仍另外假设轮半径/平地参考 |
| `lidar_camera_geometry_*` | 相应光学系到雷达的真实数据变换 |
| `lidar_camera_projection` | 还需内参/畸变、图像轴定义和时间对应，本 schema 不认证这些条件 |
| `ultrasonic_spatial_projection` | 双孔模块型号/地址/物理角色、原点、轴和波束未绑定，当前阻止空间投影 |

AVAILABLE 只表示配置中的几何依赖完整，不代表设备健康、硬件授权、物理精度或时间同步通过。IMU 平移可继续未知，因为当前主线仅用陀螺与静止重力；未来加速度积分/杆臂模型必须重新添加依赖。

新版 configuration 只在建图启用时检查运动关系。缺外参的预览使用原生数据 frame，不创建假的共同 TF、地面或车体场景，同时关闭轮反馈/控制路径。`MappingInput` 仅用于已有完整几何的处理，不接受未知几何的原生预览；明确 offline 的 schema1 历史预览保留数学兼容，不作为当前 V7 外参。

五状态保持默认；`wheel_imu_estimator=robot_localization` 是明确选择的 planar 对照，并进入 `prior_template.estimator`。5 Hz 为实时入口上限；10 Hz 只允许 `offline_experiment=true` 的同数据比较，实时启动必须拒绝该标记。只读过程计划可构造历史 offline 配置，但必须标注不能启动；preflight/start 在任何硬件检查前拒绝 offline。

## 只读检查入口

在工程目录、已安装本工程 Python 依赖的环境中执行：

```sh
PYTHONPATH=src python3 -B -m wc_runtime.calibration_geometry \
  --setup config/hardware_setup.json --project-root .

PYTHONPATH=src python3 -B -m wc_runtime.calibration_geometry \
  --setup config/hardware_setup.json --project-root . \
  --require wheel_imu_motion_all

PYTHONPATH=src python3 -B -m wc_runtime.calibration_geometry \
  --inspect-document config/calibration/sources/sensor_installation_20261002.md \
  --expected-sha256 b2b63b7b2c59c8dc0c48755b1cf4f17baee684b14f384961c09d827e136ec46e
```

这些命令只读文件和输出 JSON，不启动 ROS/SDK/硬件、不生成文件。检查未知运动能力返回 2 和具体原因；不指定 `--require` 时可以查看全部能力。机械原文解析不生成数据外参。

Python API：

- `resolve_hardware_setup(setup, required_capabilities=())` 保留旧下游字段；schema2 的未知 `mounts` 不填值，未知 IMU旋转为 None。
- `require_geometry_capability(resolved_or_report, name)` 在依赖能力不可用时抛 ValueError。
- `motion_capability(mode)` 返回所选模式的门控名称。
- `verify_geometry_sources(setup, project_root)` 返回原文/标定资料快照检查结果。
- `inspect_installation_document(path, expected_sha256=...)` 重现机械表导入和算术检查。
- `fit_rigid_transform(parent_points_m, child_points_m)` 对真实对应点进行离线刚体拟合；至少三对非共线点。输出 `FIT_ONLY_NOT_VALIDATED`，不写配置；低拟合残差不能代替独立标定验证。

## 后续采集/标定待办与通过条件

1. 核对装配版本、止挡、滑盖关闭状态和是否移动过传感器；记录 IMU 实物 +X/+Y/+Z 标识。当前资料中“此前同轴”与“现装轴向待核查”冲突，以本装配的新记录为准。
2. 保存每侧原生 raw/filtered、身份、实际 SDK/设备参数和原时间信息；比较固定真实距离、多个平面/边缘，先分离单帧几何与显示/地图差异。采集动作另按硬件权限和独占规则安排，不由检查命令执行。
3. 求双雷达真实数据系外参，使用多个方向、多个深度的结构，保留独立验证片段；仅一个大平面或看起来重合不足以证明六自由度可观测。
4. 测/标定 M→axle 基准以及需要的各数据系关系。若直接测到 axle，可直接录该变换。分别核查旋转、平移、毫米/米和方向，不靠零值绕过未知项。
5. 标定 IMU 原生零偏及安装旋转。静止重力不能确定 yaw；当前不补未知 IMU 平移。必要时先做独立相机内参，光学外参和时间对应完成后再启用几何叠加。
6. 明确双孔模块与 FD07/通信地址的绑定和测距定义，再考虑空间投影；通信未响应不能单独证明探头损坏。2026-10-04 本轮诊断已收到四地址的有效协议响应，单位和物理位置仍待核实，详见升级验收报告。
7. 外参关系齐备后，五状态与 RL 使用同一份冻结数据、外参、零偏和轮参数开展对照。验证等级分别记录 STATIC_REVIEW/SYNTHETIC/REAL_BAG/LIVE_STATIC/LIVE_DYNAMIC，不把软件测试通过写成实物标定通过。

## 原文解释限制

- 435.7 mm 为面/窗中心距，不是数据原点基线。
- 两处间距差 Δ/L 约束两壁的相对角变化；公共轴定义下应区分角度差与按相向约定的角度和，不能据此分离单盒姿态或共同倾斜。
- 单处盒间距吻合不能独立证明两盒均到止挡。
- 对称分配出的每盒约 ±0.5 mm 不是独立置信区间；卷尺、打印/装配和共同基准误差仍存在。
- 设计点左右对称不代表实物传感器姿态/外参对称，且 IMU 仅左侧一个。
- 滤波后的点云可以配准/融合；需要检查过度平滑、边缘偏移、重叠不足和真实时间滞后，不能用“后处理不能融合”作为禁用依据。
