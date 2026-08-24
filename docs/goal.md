你现在位于一台 NVIDIA Jetson AGX Orin 64GB 上，系统预期为 Ubuntu 22.04、ROS 2 Humble。当前目录是 smartwheel 自动轮椅项目仓库。

本次任务不是在现有代码上修补，而是为“第一阶段室内建图系统”建立一套干净、可测试、可逐步接入真实硬件的 ROS 2 架构。可以阅读旧代码作为参考，但原则上不得沿用旧建图、传感器、launch、TF 和配置架构。只有已经证明可用的 WSAD 遥控逻辑、电机通信逻辑、编码器读取逻辑可以在审计后重构复用。

不要接入 FD07-34R 超声波传感器。本阶段完全排除超声波。W

====================
一、项目目标
====================

最终硬件包括：

1. NVIDIA Jetson AGX Orin 64GB；
2. 两台 XT-M60 固态/Flash 3D LiDAR；
3. 一台 H30 IMU；
4. 电动轮椅左右轮编码器和电机驱动；
5. 四台摄像头；
6. ROS 2 Humble；
7. 人工通过 WSAD 遥控轮椅，或在允许获取编码器数据的前提下人工推动轮椅。

最终目标是在室内低速绕行一圈后，产生：

1. 清晰、无明显多层重影的三维点云地图；
2. 可选择输出几何点云 PCD/PLY；
3. 可选输出由四台相机离线着色后的 XYZRGB PLY；
4. 可用于后续 Nav2 的二维占据栅格地图；
5. RTAB-Map 数据库或等价的可重定位地图数据库；
6. 完整轨迹、TF、质量报告和原始 rosbag。

算法主线确定为：

- FAST-LIO2 ROS 2：主 LiDAR + H30 IMU 的局部 LiDAR-Inertial Odometry；
- 轮速里程计：独立生成并校准，作为辅助约束和退化检测；
- RTAB-Map ROS 2：消费外部里程计和三维点云，负责关键帧、回环、全局位姿图和地图数据库；
- 双 LiDAR：第一版主 LiDAR进入FAST-LIO2，副LiDAR经外参转换后参与全局三维和二维建图；
- 相机：四路全部录包；第一版最多一台前向相机参与在线回环，其余相机用于离线点云着色；
- slam_toolbox：仅作为可切换的二维地图对照后端，不能与RTAB-Map同时发布map→odom；
- robot_localization：只作为可选状态选择或辅助融合层，不得把FAST-LIO2已经使用过的同一H30 IMU再次作为独立观测重复融合。

不得把FAST-LIVO2、LIO-SAM或自研多传感器滤波器作为第一版默认主线。

====================
二、执行边界
====================

任务分为两个明确阶段。

当前只执行“阶段A：无硬件开发”。

在阶段A中：

- 不连接、探测或控制任何真实设备；
- 不访问串口、CAN、RS485、USB摄像头或雷达网络；
- 不发送真实电机命令；
- 不使用sudo安装或修改系统；
- 可以检查当前系统环境；
- 可以生成依赖安装脚本，但不得自动执行需要sudo的命令；
- 不得根据猜测伪造XT-M60 SDK接口、寄存器、点云字段或时间戳；
- 所有尚不确定的硬件参数必须保留为显式TODO；
- 必须使用mock/synthetic数据完成整个软件链路测试。

阶段B接入硬件的代码框架和操作文档可以提前完成，但不得声称已经验证真实硬件。只有收到用户明确指令“开始阶段B”后，才能执行真实设备接入。

不要只输出计划。先给出实施计划和风险，再继续完成阶段A代码实现、编译、测试和文档。

====================
三、版本控制
====================

1. 检查当前Git状态，禁止覆盖用户未提交修改；
2. 从当前提交创建新分支：

   feature/mapping-v2-clean-architecture

3. 不删除旧代码；
4. 新代码放入清晰的新目录或ROS 2工作区结构；
5. 每完成一个独立模块做一次原子提交；
6. 提交信息应使用清晰的英文Conventional Commit格式；
7. 维护：
   - CHANGELOG.md
   - docs/IMPLEMENTATION_STATUS.md
   - docs/KNOWN_LIMITATIONS.md

====================
四、环境审计
====================

首先执行只读环境检查，并生成：

docs/ENVIRONMENT_REPORT.md

至少记录：

- uname -a；
- Ubuntu版本；
- ROS_DISTRO；
- Python版本；
- GCC/G++版本；
- CMake版本；
- colcon版本；
- PCL版本；
- Eigen版本；
- OpenCV版本；
- CUDA、JetPack信息；
- 可用磁盘空间；
- 当前仓库和分支；
- 已安装的RTAB-Map、slam_toolbox、robot_localization情况；
- 当前工作区中可能冲突的包；
- 是否存在XT-M60官方SDK目录，但阶段A不要加载SDK。

缺失依赖时，生成：

scripts/install_dependencies.sh

但不要自动运行需要sudo的部分。

====================
五、旧代码复用审计
====================

阅读旧仓库，生成：

docs/LEGACY_REUSE_AUDIT.md

分类列出：

1. 可直接重构复用：
   - WSAD键盘遥控；
   - 电机安全停机；
   - 已验证的电机协议；
   - 编码器读取和轮速计算。

2. 只能参考、不得直接复用：
   - 旧传感器节点；
   - 旧TF；
   - 旧launch；
   - 旧建图逻辑；
   - 旧双雷达合并；
   - 旧状态机。

3. 应当废弃：
   - 硬编码设备路径；
   - 硬编码外参；
   - 多个节点发布相同TF；
   - 无时间戳验证的点云拼接；
   - mock数据与真实数据混在同一节点；
   - 失败后静默发布伪正常值的逻辑。

只有完成审计后才能迁移WSAD和电机相关代码。

====================
六、第三方算法管理
====================

第三方算法不得复制粘贴核心代码进入项目。

建立：

third_party/README.md
third_party/dependencies.repos
docs/THIRD_PARTY_LICENSES.md

要求：

1. 评估并固定一个经过测试的FAST-LIO2 ROS 2版本；
2. 优先评估 Ericsii/FAST_LIO_ROS2，并与 hku-mars/FAST_LIO 上游核心保持对应关系；
3. 固定具体commit，不使用浮动latest；
4. RTAB-Map和rtabmap_ros优先使用ROS 2 Humble兼容版本；
5. slam_toolbox和robot_localization使用ROS 2官方发行版本；
6. 记录每项依赖：
   - 仓库；
   - commit或版本；
   - 许可证；
   - 修改内容；
   - 选择理由；
   - 已知风险。

不要修改FAST-LIO2或RTAB-Map数学核心。适配必须通过驱动、消息转换器、配置和launch完成。只有确认XT-M60消息无法被预处理模块接收时，才增加一个最小、隔离、带测试的preprocess adapter。

====================
七、建议ROS 2包结构
====================

创建或整理为以下模块。名称可小幅调整，但职责边界必须保留：

src/
├── smartwheel_interfaces
├── smartwheel_description
├── smartwheel_sensor_api
├── xtm60_ros2_driver
├── h30_imu_driver
├── wheel_odom_driver
├── camera_array_driver
├── dual_lidar_fusion
├── smartwheel_state_estimation
├── smartwheel_global_mapping
├── smartwheel_map_products
├── smartwheel_mapping_manager
├── smartwheel_teleop
├── smartwheel_bringup
├── smartwheel_sim
└── smartwheel_tests

职责：

smartwheel_interfaces：
- 自定义状态消息、诊断消息、地图任务服务；
- 不重新定义已有标准ROS消息。

smartwheel_description：
- URDF/Xacro；
- base_link、轮子、IMU、双雷达、四相机；
- 所有外参由YAML/Xacro参数输入；
- 禁止写死未经确认的尺寸。

smartwheel_sensor_api：
- 定义统一的LiDAR、IMU、编码器、相机抽象接口；
- 提供mock实现；
- 不包含具体厂商猜测。

xtm60_ros2_driver：
- 阶段A只实现mock backend和vendor SDK adapter接口；
- 阶段B再读取官方SDK；
- 输出标准sensor_msgs/PointCloud2和诊断信息。

h30_imu_driver：
- 阶段A只实现mock和解析单元测试；
- 阶段B接入真实协议。

wheel_odom_driver：
- 编码器计数转换；
- 差速运动学；
- 协方差；
- 方向、轮径、轮距、减速比全部参数化。

camera_array_driver：
- 四路相机统一命名；
- camera_info；
- mock图像；
- 时间戳和掉帧诊断。

dual_lidar_fusion：
- 双雷达TF变换；
- 时间门限检查；
- 单位和字段验证；
- 可选体素滤波；
- 不允许简单使用“最新两帧”无条件拼接；
- 第一版输出map-only merged cloud；
- 为后续dual-LIO模式预留接口。

smartwheel_state_estimation：
- lio_primary模式；
- wheel_imu_fallback模式；
- LIO和轮速残差监测；
- 禁止重复融合同一IMU。

smartwheel_global_mapping：
- FAST-LIO2适配；
- RTAB-Map外部里程计模式；
- slam_toolbox可选二维基线；
- 保证同一时刻只有一个map→odom发布者。

smartwheel_map_products：
- PCD/PLY导出；
- XYZRGB离线着色；
- 二维占据栅格；
- YAML/PGM/PNG；
- 轨迹导出；
- 地图质量报告。

smartwheel_mapping_manager：
- 建图任务状态机；
- rosbag录制；
- 离线回放；
- 地图版本管理；
- 失败原因。

smartwheel_teleop：
- WSAD；
- deadman；
-速度限制；
- 加速度斜坡；
- 命令超时停车；
- 硬件禁用模式绝不发送真实命令。

smartwheel_sim：
- 确定性室内场景；
- 模拟双Flash LiDAR、IMU、轮速和四相机；
- 噪声、延迟、丢帧、时间偏移可配置。

====================
八、坐标系和Topic契约
====================

定义并写入：

docs/FRAME_AND_TOPIC_CONTRACT.md

TF树：

map
└── odom
    └── base_link
        ├── imu_link
        ├── xtm60_left_link
        ├── xtm60_right_link
        ├── camera_front_link
        ├── camera_left_link
        ├── camera_right_link
        └── camera_rear_link

约定ROS REP-103：

- X前；
- Y左；
- Z上；
- 右手坐标系；
- 距离统一为米；
- 角度统一为弧度；
- 时间统一使用ROS时间戳。

主要Topic：

/lidar/left/points_raw
/lidar/right/points_raw
/lidar/primary/points_lio
/lidar/left/points_registered
/lidar/right/points_registered
/lidar/merged/points
/imu/data_raw
/wheel/odom
/lio/odom
/lio/path
/odom/fused
/camera/front/image_raw
/camera/left/image_raw
/camera/right/image_raw
/camera/rear/image_raw
/camera/*/camera_info
/map
/map_cloud
/mapping/status
/hardware/status
/diagnostics

所有frame_id、QoS、频率、超时和topic名称都应参数化。

====================
九、硬件配置模板
====================

创建：

config/hardware_profile.template.yaml

所有未知字段使用null、空字符串或显式TODO，不得编造。

至少包含：

lidar_left:
  model: XT-M60
  ip_address: null
  sdk_config_path: null
  frame_id: xtm60_left_link
  point_unit: null
  scan_rate_hz: null
  timestamp_source: null
  timestamp_semantics: null
  has_per_point_time: null
  point_time_field: null
  intensity_field: null
  range_min_m: null
  range_max_m: null
  horizontal_fov_deg: null
  vertical_fov_deg: null
  x_m: null
  y_m: null
  z_m: null
  roll_deg: null
  pitch_deg: null
  yaw_deg: null

lidar_right:
  同样字段。

dual_lidar:
  center_distance_m: null
  horizontal_overlap_deg: null
  vertical_overlap_deg: null
  max_pair_time_difference_ms: null
  primary_lidar: left
  integration_mode: map_only

imu:
  model: H30
  port: null
  baud_rate: null
  frame_id: imu_link
  rate_hz: null
  timestamp_source: null
  axis_convention: null
  orientation_available: null

wheel:
  wheel_radius_m: null
  track_width_m: null
  encoder_cpr: null
  gear_ratio: null
  left_sign: null
  right_sign: null
  velocity_unit: null
  register_definition_source: null

cameras:
  front/left/right/rear:
    device: null
    frame_id: null
    width: null
    height: null
    fps: null
    timestamp_source: null
    intrinsic_file: null
    distortion_model: null
    x/y/z/roll/pitch/yaw: null

同时创建：

config/hardware_profile.mock.yaml

用于无硬件测试，并使用明确标注的模拟数值。

====================
十、阶段A：确定性无硬件模拟
====================

实现一个不依赖Gazebo的轻量级确定性模拟器。

模拟环境至少包括：

- 一个矩形房间；
- 一段L形走廊；
- 门洞；
- 柱子；
- 桌腿或箱体；
- 高低不同的三维结构；
- 完整闭环路线。

生成固定随机种子的数据：

seed = 20260714

模拟：

1. 两台120°级窄视场Flash LiDAR；
2. 两雷达有可配置间距、俯仰、安装高度和重合角；
3. H30风格IMU；
4. 差速轮速里程计；
5. 四路相机图像；
6. 真实轨迹ground truth；
7. 时间偏移；
8. 高斯噪声；
9. 轮滑；
10. 掉帧；
11. 雷达遮挡；
12. 相机模糊占位测试。

模拟器必须能输出标准ROS 2消息，并能录制rosbag2。

必须提供：

ros2 launch smartwheel_bringup sim_mapping.launch.py

运行后至少可以看到：

- 两台模拟雷达点云；
- IMU；
- wheel odom；
- TF；
- 模拟轨迹；
- 合并点云；
- 一份三维地图；
- 一份二维占据栅格地图。

阶段A允许使用ground truth odom验证RTAB-Map和地图产品链路；同时提供mock-LIO接口，以便未来替换为真实FAST-LIO2。不得声称模拟结果等同于真实FAST-LIO2性能。

====================
十一、测试
====================

实现单元测试：

1. 差速轮式里程计公式；
2. 编码器溢出；
3. 左右轮方向；
4. 米/毫米转换；
5. 点云字段验证；
6. 时间戳单调性；
7. 双雷达时间配对；
8. 双雷达外参变换；
9. TF唯一发布者检查；
10. PointCloud2字段读取；
11. 体素滤波；
12. 二维栅格ray casting；
13. 地图保存；
14. teleop超时停车；
15. hardware_enabled=false时绝不发送电机命令。

实现集成测试：

- 模拟一圈闭环；
- 生成地图；
- 保存PCD/PLY；
- 保存PGM/PNG/YAML；
- 保存轨迹；
- 输出质量报告；
- 验证生成文件存在且非空；
- 验证地图坐标和ground truth大致一致；
- 验证没有TF冲突。

执行：

colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

colcon test
colcon test-result --verbose

修复所有由本次代码引入的编译和测试失败。

====================
十二、启动入口
====================

提供以下launch：

sim_mapping.launch.py
record_mapping.launch.py
single_lidar_mapping.launch.py
dual_lidar_mapping.launch.py
offline_mapping.launch.py
map_export.launch.py
teleop_safe.launch.py

参数：

mode:=mock|real
mapping_backend:=rtabmap|slam_toolbox
lidar_mode:=single|dual_map_only|dual_lio
state_mode:=lio_primary|wheel_imu_fallback
record_bag:=true|false
enable_cameras:=true|false
enable_online_visual_loop:=true|false
enable_offline_colorization:=true|false
hardware_enabled:=true|false

当mapping_backend=rtabmap时，不启动slam_toolbox的map TF。

当mapping_backend=slam_toolbox时，不启动RTAB-Map的map TF。

dual_lio模式在阶段A只能存在接口和明确的NOT_IMPLEMENTED诊断，不允许伪实现。

====================
十三、建图状态机
====================

实现：

IDLE
→ CHECKING
→ RECORDING
→ MAPPING
→ LOOP_CLOSING
→ OPTIMIZING
→ EXPORTING
→ QUALITY_CHECK
→ READY

失败进入FAILED，并记录具体原因。

检查内容：

- Topic是否存在；
- 数据频率；
- 时间戳是否倒退；
- LiDAR与IMU时间差；
- 双雷达时间差；
- TF完整性；
- 点云单位；
- 点云字段；
- 轮速是否更新；
- 磁盘空间；
- rosbag是否正常写入；
- 地图是否保存成功。

====================
十四、地图导出
====================

每次建图创建独立目录：

maps/versions/<map_name>_<timestamp>/

至少包含：

map_geometry.pcd
map_geometry.ply
map_colored.ply（有相机数据时）
map_2d.pgm
map_2d.png
map_2d.yaml
trajectory.tum
poses.csv
rtabmap.db（使用RTAB-Map时）
bag_path.txt
hardware_profile_used.yaml
algorithm_profile_used.yaml
quality_report.json
quality_report.md
manifest.json
logs/

二维地图初始分辨率：

0.05 m/cell

但必须参数化。

二维地图必须区分：

- occupied；
- free；
- unknown。

不得只把三维点简单垂直投影成黑点。需要使用传感器原点和ray casting更新自由空间。RTAB-Map生成的占据地图和自定义导出结果都需要保存。

====================
十五、四相机离线着色框架
====================

阶段A实现接口和mock测试。

离线着色流程：

1. 读取全局优化后的三维点云；
2. 读取四台相机图像和camera_info；
3. 读取每帧相机位姿；
4. 将点转换到相机坐标系；
5. 投影到图像平面；
6. 使用深度缓冲或可见性检查；
7. 排除相机背后的点；
8. 排除遮挡严重的点；
9. 多相机均可见时，优先选择：
   - 视线夹角更小；
   - 距离更近；
   - 图像更清晰；
   - 曝光更合理的视图；
10. 输出XYZRGB PLY。

阶段A使用合成图像验证投影，不使用真实相机。

====================
十六、安全要求
====================

任何真实电机命令都必须同时满足：

hardware_enabled=true
AND
deadman_active=true
AND
emergency_stop=false
AND
command_not_timed_out=true

默认：

hardware_enabled=false

WSAD要求：

- W前进；
- S后退；
- A左转；
- D右转；
- Space立即停止；
- 松开按键后超时停车；
- 限制最大线速度和角速度；
- 加速度和减速度斜坡；
- 程序退出时发送停车；
- 节点崩溃后由独立watchdog停车。

阶段A不得打开真实串口或发送真实命令。

人工推动模式必须单独标识：

motion_mode:=teleop|push

push模式不能默认假定编码器有效。必须在文档中说明：

- 电机断电后编码器是否仍供电；
- 人工推动时编码器是否仍返回；
- 驱动器是否允许被反拖；
- 若无轮速数据，wheel odom应明确标记不可用。

====================
十七、阶段B预留流程
====================

只写入 docs/HARDWARE_BRINGUP_RUNBOOK.md，当前不要执行。

阶段B应严格按顺序：

1. 检查XT-M60官方上位机配置、SDK、示例和数据结构；
2. 生成XT-M60_SDK_CAPABILITY_REPORT.md；
3. 明确：
   - 点云单位；
   - 坐标轴；
   - PointCloud字段；
   - 强度字段；
   - 一帧时间戳定义；
   - 是否逐点时间；
   - Flash曝光方式；
   - 帧率；
   - 网络协议；
   - SDK线程模型；
4. 先接一台XT-M60；
5. 仅发布原始点云和诊断；
6. 用卷尺验证距离和坐标；
7. 录制至少一个静态及低速移动bag；
8. 接H30 IMU；
9. 验证坐标轴、重力、角速度、时间戳；
10. 运行单LiDAR FAST-LIO2；
11. 验证静止、直线、缓慢旋转和小闭环；
12. 标定轮速里程计；
13. 加入第二台LiDAR的map-only模式；
14. 验证双雷达重叠区域；
15. 运行RTAB-Map全局闭环；
16. 加入一台前向相机；
17. 四相机只用于录制和离线着色；
18. 导出三维和二维地图；
19. 最后才评估dual_lio或FAST-LIVO2。

====================
十八、验收标准
====================

阶段A验收：

- 工作区可编译；
- 测试通过；
- 无硬件即可启动完整模拟；
- 可录制和回放rosbag；
- 可生成三维点云地图；
- 可生成二维占据栅格地图；
- 可导出所有地图文件；
- TF树正确；
- 无多个map→odom发布者；
- 所有硬件参数均来自YAML；
- 无硬编码IP、串口和外参；
- 真实电机默认不可用；
- 文档命令可以逐条执行；
- 明确列出所有未完成硬件TODO。

阶段B未来验收参考：

- 真实传感器连续运行30分钟无崩溃；
- 时间戳无倒退；
- 点云尺度正确；
- 小范围闭环无明显位姿跳变；
- 墙面没有严重双层重影；
- 二维墙体基本连续；
- 地图可重新加载；
- 同一路线重复三次结果具有基本一致性；
- 任何失败均有明确诊断，不发布伪正常数据。

====================
十九、最终交付
====================

完成阶段A后，输出：

1. 修改摘要；
2. 新目录树；
3. 每个包的职责；
4. 编译命令；
5. 测试命令；
6. 模拟启动命令；
7. rosbag录制和回放命令；
8. 地图导出命令；
9. 测试结果；
10. 已完成项；
11. 未完成项；
12. 阶段B所需用户信息；
13. 已知风险；
14. Git提交列表。

阶段B需要用户补充的信息至少包括：

- 两台XT-M60的IP；
- SDK实际路径；
- 官方配置文件；
- 两雷达中心间距；
- 每台雷达相对base_link的XYZ和roll/pitch/yaw；
- 雷达离地高度；
- 雷达视场重合情况；
- SDK点云字段和时间字段；
- H30安装位置和方向；
- 左右轮半径；
- 轮距；
- 编码器分辨率；
- 减速比；
- 电机和编码器方向；
- 四台相机设备标识；
- 相机分辨率、帧率和镜头信息；
- 四相机安装外参；
- 人工推动时编码器是否仍工作。

不得用猜测填充这些值。

现在开始：
1. 检查Git状态和系统环境；
2. 输出简洁实施计划；
3. 创建新分支；
4. 执行阶段A；
5. 编译和测试；
6. 提交代码；
7. 输出完整交付报告。

不要在阶段A连接真实设备，不要声称完成任何真实硬件验证。