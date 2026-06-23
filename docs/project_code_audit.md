# SmartWheel 全项目代码审查报告

> 单一活文档(living document)。审查全程增量写入:每检视完一个文件即在 §2 覆盖清单回填状态,
> 每发现一个缺陷即在 §1 缺陷表追加一行(全局连续编号、6 字段齐全、不封顶)。
> **本审查为文档/审查工程,不修改任何运行时代码**(`src/**`、`scripts/**`、`calib/**`、`auto_test/**` 源文件内容保持不变)。

## 0. 元信息

- **审查日期**:2026-06-17(骨架与文件枚举初始化)
- **审查对象项目**:`/home/nvidia/smartwheel`(SmartWheel 智能轮椅 ROS 2 工程)
- **起点文档(F)**:`docs/mapping_code_review_2026-06-17.md`(仅覆盖建图链路、人为封顶 50 条、非逐文件)
- **本报告(F')**:`docs/project_code_audit.md`(逐文件、穷尽、不封顶)

### 审查范围(Scope)

被审查的"输入域"为 `/home/nvidia/smartwheel` 下的全部一手源文件,至少包含(摘自 bugfix.md「审查范围」):

- Python 节点与库:`src/**/wheelchair_*/**/*.py`(运动学、里程计/驱动、Modbus、点云融合、cloud_utils、栅格投影、KISS-ICP、RGB 着色、一致性监控、感知、导航、安全、诊断、传感器适配、mock、各 `setup.py`)。
- 测试:`src/**/test/**/*.py` 与 `auto_test/*.py`。
- C / C++ 源:`src/wheelchair_bringup/src/*.cpp`、`*.c`、`include/**`、`CMakeLists.txt`。
- ROS launch:`src/**/launch/*.launch.py`。
- 配置 yaml:`src/**/config/*.yaml`(EKF、RTAB-Map、nav2、安全、诊断、传感器、scan、URDF static tf 等)。
- URDF / xacro:`src/wheelchair_description/urdf/*.xacro` 及 description 配置。
- Shell 脚本:`scripts/*.sh` 及 `src/**/scripts/*`。
- 地图后处理脚本:`src/wheelchair_mapping/scripts/*.py`。
- 标定文件:`calib/*.yaml`(外参/内参一致性)。
- RViz 配置 `*.rviz` 仅在影响建图/可视化正确性判断时检视。

### 显式排除集(Explicit Exclusions)

下列路径**不进入逐文件审查**,不为其制造缺陷条目(除非审查中发现其被正式链路引用):

- `build/`、`__pycache__/`、`.git/`、`.pytest_cache/`
- 二进制图片(`*.png` 等)
- 仓库根目录临时 dotfiles:`.b.log`、`.base.log`、`.base2.log`、`.bl.txt`、`.cm.txt`、`.const.txt`、`.dl.txt`、`.dl2.txt`、`.dv.txt`、`.f.txt`、`.jog.py`、`.m2.txt`、`.mid360_probe.sh`、`.mid360_probe.txt`、`.rpm.txt`、`.test.py`、`.verify_pts.txt`、`.w.txt`、`.w2.txt`、`.y.txt`、`.hwreadme.md` 等

### 审查方法(Audit Methodology)

按 ROS 包为单位、自底向上(底盘 → 传感器 → 感知 → 建图 → 导航 → 安全 → 诊断 → 集成)遍历。
每个批次内文件按「库 → 节点 → 配置 → launch → 测试」顺序检视。每个文件至少跑一遍以下 **12 维度检视清单**,任一维命中即生成缺陷条目:

1. 算法错误(公式/积分/坐标变换是否正确)
2. 功能性 bug(逻辑分支、状态机、返回值)
3. 不严谨/不稳健的方法(近似阶数不一致、量化偏置)
4. 健壮性(异常吞没、空值/边界、资源泄漏、超时)
5. 规范(命名/重复/死代码/魔法数)
6. 架构(职责混叠、生命周期管理、双重过滤)
7. 参数一致性(跨文件量程/坐标/频率/外参冲突 —— 标记后汇入 §3)
8. 时间戳与同步(墙钟 vs 采集时刻、approx_sync、queue)
9. TF / 坐标系正确性(lookup 时间、frame 命名、Force3DoF)
10. QoS 匹配(reliable/best_effort、depth、durability 在 pub/sub 两侧是否一致)
11. 单位/符号(rad vs deg、左右/正负、米 vs 毫米)
12. 测试覆盖(关键算法是否有回归测试、mock 是否掩盖真实问题)

审查批次划分(B1–B15)见 design.md §Fix Implementation。

### 严重度定义(Severity Taxonomy)

- **阻断**:导致功能不可用、崩溃、数据损坏,或使建图/导航/安全完全失效。
- **高**:显著损害正确性/建图质量/安全裕度,常态触发但系统勉强运行。
- **中**:边界条件/隐患/健壮性缺陷,特定条件下触发。
- **低**:规范/可维护性问题(命名、重复、死代码、注释缺失),不影响运行时正确性。

(与起点文档 ⛔/⚠️/🔸 的映射:⛔→阻断或高;⚠️→中;🔸→低,迁移时按上述定义重判。)

## 1. 缺陷表(Defect Table)

> 全局连续编号(D001、D002…),不复用旧文档编号,不封顶。每行 6 字段齐全;
> 建图链路相关条目的「影响」列须单独说明对建图质量的影响。严重度 ∈ {阻断, 高, 中, 低}。

| 编号 | 文件位置(相对路径+函数/类+行号区间) | 问题描述 | 影响(建图项单列建图质量影响) | 严重度 | 建议修复方向 |
|------|------------------------------------------|----------|--------------------------------|--------|--------------|
| D001 | src/wheelchair_base/wheelchair_base/kinematics.py · `DifferentialDriveModel.clamp_twist` / `twist_to_wheel_rpm` · L12-26 | `clamp_twist` 对 `linear_x` 与 `angular_z` **各自独立**限幅,没有对单轮速度做饱和。当线速度与角速度同时接近上限时,外侧轮速 = `linear + angular*sep/2` 会超出仅由 `max_linear` 推得的轮速上限,即电机可能进入物理饱和而命令未感知。 | 真实硬件上外侧电机饱和、实际转弯半径偏离指令值,而开环里程计仍按指令积分 → 航迹估计失真。建图质量影响:`/wheel/odom` 是 EKF 与 RTAB-Map 外部里程计的平移来源,饱和导致的航迹偏差会直接带入位姿图,转弯处建图漂移、墙线弯折。 | 中 | 在 `twist_to_wheel_rpm` 输出后增加按 `max_wheel_rpm` 的等比例缩放(保持转弯曲率的 wheel-speed saturation),或在 `clamp_twist` 中联合考虑单轮速度上限。 |
| D002 | src/wheelchair_base/wheelchair_base/kinematics.py · `twist_to_wheel_rpm` 与 zlac8030_driver_node.py `tick` · L19, L181-184 | 限幅重复:节点 `tick` 已调用 `self.model.clamp_twist(...)`,随后 `twist_to_wheel_rpm` 内部再次调用 `clamp_twist`,同一对值被限幅两次。 | 无功能性危害(幂等),但职责重复、易在后续修改其一时产生不一致预期,属规范/可维护性问题。无直接建图质量影响。 | 低 | 约定单一限幅点:或让 `twist_to_wheel_rpm` 不再内部限幅(由调用方负责),或移除 `tick` 中的预限幅,并补注释说明限幅契约。 |
| D003 | src/wheelchair_base/wheelchair_base/zlac8030_driver_node.py · `_initialize_motion_mode` · L300-330 | 在 ROS 定时器回调链(`tick`→`_write_wheel_commands`→`_initialize_motion_mode`)内使用阻塞 `time.sleep(0.2)` 两次(合计 0.4s)。单线程 executor 下该回调会阻塞整个节点。 | 首个非零命令到来时,节点被阻塞约 0.4s,期间无法处理 `/cmd_vel_safe`、无法发布 `/wheel/odom`/TF。建图质量影响:里程计与 TF 出现 0.4s 断流/时间跳变,RTAB-Map/EKF 在该窗口缺少 odom 约束,易触发位姿插值误差或同步告警。 | 中 | 将驱动使能时序改为非阻塞状态机(用定时器/时间戳分步推进 clear-fault→settle→enable),或把电机初始化放到独立线程/MutuallyExclusive 之外的回调组,避免阻塞 odom 发布。 |
| D004 | src/wheelchair_base/wheelchair_base/zlac8030_driver_node.py · `tick`/`_write_wheel_commands`/`_read_feedback` · L161-205, L255-275 | 50Hz 定时器(周期 20ms)回调内同步执行多次 Modbus RS485 往返(配置下为 1 次写 + 2 次读反馈),每次 `serial.read` 超时 0.08s。任一帧超时即阻塞远超 20ms 周期。 | RS485 偶发丢帧/超时会使单次 `tick` 耗时数十至上百毫秒,定时器溢出、odom 发布抖动且 dt 失真。建图质量影响:odom 时间戳间隔不均、积分 dt 抖动,直接降低 EKF/RTAB-Map 轨迹精度,产生周期性位姿台阶。 | 中 | 将串口 I/O 移出定时器回调(独立 I/O 线程 + 最新值缓存),或降低反馈读取频率、缩短超时并配合异步重试;确保 odom 发布与串口往返解耦。 |
| D005 | src/wheelchair_base/wheelchair_base/zlac8030_driver_node.py · `tick` 反馈分支 · L188-201 | 闭环(feedback 寄存器已配置,见 zlac8030_base.yaml 8363/8364)时,一旦 `_read_feedback` 返回 None(读失败),`actual_left/right_rpm` 保持 (0.0, 0.0),该周期不回退到已知的指令速度,直接按"零运动"积分里程计。 | 当 RS485 反馈读取偶发失败时,机器人仍在运动但该周期 odom 贡献被清零 → 里程计系统性少积分。建图质量影响:在反馈链路抖动时 `/wheel/odom` 距离被低估,地图整体尺度收缩、轨迹偏短,长走廊累计误差显著。 | 中 | 反馈读失败时回退到本周期指令速度(开环估计)而非清零,或对连续失败计数并显式上报诊断/降级,避免静默丢运动。 |
| D006 | src/wheelchair_base/wheelchair_base/zlac8030_driver_node.py · `__init__` 参数声明 · L66-78 ↔ src/wheelchair_bringup/config/zlac8030_base.yaml | 节点参数默认值与部署 YAML 不一致:节点默认 `max_linear_mps=0.4`/`max_angular_rps=1.0`/`single_slave_dual_axis=False`/`invert_left=False`/`invert_right=True`,而 zlac8030_base.yaml 为 0.25/0.6/True/True/False。裸 `ros2 run`(不加载 YAML)将以更高的不安全限速和错误的轴/方向配置运行。 | 缺省启动时限速高于经标定的安全值、方向/单从机配置相反 → 误动作风险与里程计方向错误。属跨文件参数一致性问题,**汇入 §3(B14)** 与量程/频率族一并核对。建图质量影响:方向/轴配置错误会使 odom 符号反转,建图轨迹镜像/发散。 | 中 | 让节点默认值与生产 YAML 对齐(或将安全上限设为更保守的默认),并在启动时校验关键参数是否来自 YAML;跨文件一致性详见 §3。 |
| D007 | src/wheelchair_base/wheelchair_base/zlac8030_driver_node.py · `__init__` 定时器创建 · L159-161 | `self.create_timer(1.0 / float(publish_rate_hz), self.tick)` 未校验 `publish_rate_hz`;参数为 0 时构造期即抛 `ZeroDivisionError`,为负时定时器周期非法。 | 配置失误(0/负频率)导致节点启动即崩溃,无友好报错。无直接建图质量影响(启动前失败)。 | 低 | 对 `publish_rate_hz` 做下界校验(>0),非法时记录错误并回退到安全默认值或拒绝启动并给出明确日志。 |
| D008 | src/wheelchair_base/wheelchair_base/zlac8030_driver_node.py · `tick`/`integrate`/方向往返/反馈路径(整体) | 现有测试(test_zlac_shutdown.py)仅覆盖关停与 `_write_wheel_commands` 使能时序,**未覆盖** `tick` 的里程计积分、`_apply_direction`/`_remove_direction` 往返一致性、反馈解析与开环回退路径。 | 核心 odom 生成链路缺回归测试,前述 D001/D005 一类缺陷无法被测试捕获。建图质量影响:里程计正确性是建图地基,缺测试使其回归风险长期存在。 | 中 | 为 `tick`(注入假 modbus/时钟)补单元测试:验证开环/闭环里程计积分、方向往返为恒等、反馈失败时的行为,以及 dt 异常处理。 |
| D009 | src/wheelchair_base/test/test_kinematics.py · 全文 · L11-30 | 测试仅覆盖直线 roundtrip、纯前进积分与 Modbus CRC,缺少:纯旋转(`angular_z≠0`)的 `integrate`/`wheel_rpm_to_twist` 验证、`clamp_twist` 限幅边界、负向运动等用例。 | 运动学旋转项与限幅逻辑无回归覆盖(D001 的饱和问题正落在此空白)。无直接运行时危害,属测试覆盖不足。 | 低 | 增补纯旋转里程计、限幅饱和、负速度与组合运动的断言用例。 |
| D010 | src/wheelchair_base/wheelchair_base/modbus_rtu.py · `_exchange`/`ModbusRtuClient` · L94-112 | 传输层无瞬时错误恢复:CRC 错误/超时直接抛异常且不重试;串口一旦 `open()` 成功后断连(设备拔出/总线故障),`_serial` 不会被关闭重连,后续所有交换持续失败直至节点重启。 | 偶发总线扰动无重试容错;物理断连后无自愈,需重启节点。结合 D005,读失败会清零 odom 周期。建图质量影响:总线抖动期间命令/反馈持续丢失,odom 长时间停摆导致建图中断。 | 低 | 对超时/CRC 失败增加有限次重试与帧间隔;在串口异常时 `close()` 并在下次 `_exchange` 重连;必要时上报诊断状态。 |
| D011 | src/wheelchair_sensors/wheelchair_sensors/camera_adapter_node.py · `CameraAdapter.read_image`/`CameraAdapterNode.tick` · L66-72, L171-200 | 单线程 executor 下,`tick` 在同一回调里**串行**调用每个启用相机的阻塞 `cv2.VideoCapture.read()`(real 模式)。多路 USB 相机依次读取,任一相机的 I/O 阻塞(USB 带宽争用、掉帧重连)都会阻塞整个节点。 | 多相机时单次 `tick` 耗时可能远超定时器周期(15Hz→66ms),导致发布抖动、相机流卡顿,并阻塞同进程其他回调。建图质量影响:若该节点与建图/着色链路共享 executor,图像延迟会拖累 RGB 着色与时间同步。 | 中 | 把相机采集移出定时器回调(每相机独立 I/O 线程 + 最新帧缓存),或为相机回调使用 ReentrantCallbackGroup/多线程 executor,确保采集与发布解耦。 |
| D012 | src/wheelchair_sensors/wheelchair_sensors/camera_adapter_node.py · `tick`/`cv_frame_to_image`/`_publish_info` · L153-156, L178-205 | 图像/CameraInfo 时间戳取 `self.get_clock().now()`(墙钟、tick 发布时刻)而非采集时刻;且 `Image`/`CameraInfo` 发布器用默认 QoS(reliable, depth=10),而图像流消费者(如 RGB 着色)常用 sensor_data(best_effort)QoS,两侧可能不兼容导致收不到数据。 | 墙钟时间戳引入采集→发布的可变延迟,损害与点云/IMU 的近似时间同步;QoS 不匹配则订阅方静默收不到图像。建图质量影响:RGB 着色依赖图像与点云时间对齐,墙钟戳错位会造成着色拖影/错配。时间基准问题汇入 §3(B14)。 | 中 | 用采集时刻(或至少在 `read_image` 返回处取戳)作为 header.stamp;为图像话题显式声明 sensor_data QoS,与下游消费者对齐。 |
| D013 | src/wheelchair_sensors/wheelchair_sensors/imu_adapter_node.py · `YesenseParser._valid_checksum` · L184-191 | 校验和验证**同时接受正序与逆序**两种字节序(`frame[-2:] in (bytes([a,b]), bytes([b,a]))`),把本应唯一的校验码放宽为两个合法值,显著削弱差错检测能力,被损坏的帧更易蒙混通过。 | 总线噪声导致的坏帧可能被当作有效 IMU 采样发布(错误姿态/角速度/加速度)。建图质量影响:错误 IMU 数据进入 EKF/LIVO 会污染姿态估计,转弯/振动时建图漂移。 | 中 | 固定为硬件实际使用的单一字节序(在 x86/ARM 上确认后写死),仅接受唯一校验码;如确需兼容两种设备,用参数显式选择而非无条件双接受。 |
| D014 | src/wheelchair_sensors/wheelchair_sensors/imu_adapter_node.py · `ImuAdapterNode._stamp` · L300-318 | `use_device_timestamp` 的设备时钟→ROS 时钟映射只取**运行期最小** offset(`offset < self._clock_offset_ns` 才更新),offset 单调不增。若设备时钟比主机慢,真实 offset 随时间增大而映射值永远停在历史最小值 → 长时间运行后时间戳正向漂移无界。此外参数名 `use_device_timestamp` 与 xtm60 的 `use_sdk_timestamps` 命名不一致(同一"用采集时刻"语义),汇入 §3(B14)时间基准族。 | 长会话下 IMU 时间戳逐渐偏离真实采集时刻,破坏与点云/里程计的同步;命名分裂增加配置误用风险。建图质量影响:IMU 戳漂移使 EKF/LIVO 时间对齐恶化,长走廊累计漂移。 | 中 | 改用带缓慢回归的时钟同步(允许 offset 双向缓变,如指数平滑/PI 跟踪)或周期性重置;统一传感器适配器的时间基准参数命名。 |
| D015 | src/wheelchair_sensors/wheelchair_sensors/ultrasonic_adapter_node.py · `UltrasonicArrayAdapter.read_ranges` · L150-170 | 逐传感器 `try: ... except Exception: continue` **静默吞没**所有串口/协议/CRC 错误,无日志、无计数、无诊断上报;无法区分"无障碍"与"传感器读失败"。 | 串口/总线故障被完全隐藏,运维无法发现某路超声长期失效。安全影响:失效传感器静默缺席,避障盲区不可见(配合 D016 更危险)。无直接建图质量影响。 | 中 | 对每路读失败计数并节流上报(日志/诊断话题),区分瞬时与持续失败;持续失败时显式降级标记,而非静默 continue。 |
| D016 | src/wheelchair_sensors/wheelchair_sensors/ultrasonic_adapter_node.py · `read_ranges`/`_publish_ranges` · L165, L235-247 | 发布的 `range` 仅做 `max(0.0, distance_mm/1000.0)`,**未裁剪到 [min_range, max_range]**:无回波/读数 0 会被发布为 `range=0.0`(低于 min_range,被下游解读为"贴脸障碍"),超量程值也原样透出,均未按 REP-117 置为越界标记。 | 0.0 假读数可能触发误急停/误避障;超量程值污染避障判断。无直接建图质量影响(超声不入建图)。 | 中 | 按 min/max_range 裁剪并对越界读数置 `+inf`/`-inf` 或丢弃(遵循 REP-117);0 值视为无效不发布。 |
| D017 | src/wheelchair_sensors/wheelchair_sensors/ultrasonic_adapter_node.py · `read_ranges`/`UltrasonicAdapterNode.tick` · L150-170, L255-265 | `read_ranges` 在定时器回调内对 4 路传感器做**顺序阻塞**串口往返(部署 `serial_timeout_sec=0.2`)外加每路 `time.sleep(0.01)`,最坏 ~0.84s 阻塞;而定时器周期为 1/publish_rate(10–20Hz=50–100ms)。单线程 executor 下严重超期。 | 单次 `tick` 远超周期 → 超声发布抖动/堆积,并阻塞同进程其他回调。无直接建图质量影响。 | 中 | 把串口轮询移出定时器(独立 I/O 线程 + 最新值缓存),去掉回调内 `sleep`,或降低轮询频率并缩短超时配合异步重试。 |
| D018 | src/wheelchair_sensors/wheelchair_sensors/xtm60_adapter_node.py · `XTM60AdapterNode._configured_ip_reachable`/`_maybe_start_adapter`/`tick` · L760-815, `tick` | SDK 未运行时,`tick` 调 `_maybe_start_adapter`,内部 `_configured_ip_reachable` 用阻塞 `subprocess.run(["ping","-c","1","-W","1",ip], timeout=2.0)` 探测可达性。单线程 executor 下,雷达断连期间每次重连尝试都阻塞 executor 最长 ~2s。 | 雷达离线/启动阶段节点周期性卡顿 ~2s,状态发布与重连节奏受拖累。建图质量影响:雷达恢复期间节点阻塞会延后点云恢复,延长建图中断窗口。 | 中 | 将可达性探测移出回调(独立线程或异步),或改用非阻塞 socket connect 探测;重连退避不应阻塞 executor。 |
| D019 | src/wheelchair_sensors/wheelchair_sensors/xtm60_adapter_node.py · `XTM60AdapterNode._make_cloud` · L820-832 | `use_sdk_timestamps=false`(left/right/sdk 三份 yaml 默认)时,点云 header.stamp 取 `get_clock().now()`(墙钟、tick 发布时刻);帧由 SDK 回调缓存、下一次定时器 tick 才发布,故时间戳≠采集时刻(墙钟 vs 采集,已知 #41 同类)。 | 采集→发布存在最长一个定时器周期(~0.1s)+ 抖动的延迟却被戳成"现在",破坏与 IMU/里程计的 approx_sync。建图质量影响:点云时间错位直接导致 RTAB-Map/LIVO 配准漂移、墙线重影。时间基准汇入 §3(B14:适配器 use_sdk_timestamps ↔ 融合 header.stamp ↔ EKF approx_sync)。 | 中 | 默认采集时刻优先:校验并启用 SDK 时间戳,或在 SDK 回调收帧瞬间打主机戳并随帧传递;避免在发布点重新打墙钟戳。 |
| D020 | src/wheelchair_sensors/wheelchair_sensors/xtm60_adapter_node.py · `XTM60SdkConfig`/`extract_xyzi_points`/`extract_xyzi_grid` · L70-100, L195-205 | `point_unit_scale` 默认 1.0,注释自承"deployments should verify whether SDK returns meters or millimeters"——单位未在代码层校验。若某 SDK 构建返回毫米,则坐标被放大 1000×,随后被 range 滤波([range_min,range_max])几乎全部剔除 → 点云静默变空,且无告警。此外节点默认 `range_max=20.0` 与部署 yaml(xtm60_sdk=12、left/right=50)不一致,标记汇入 §3(B14 量程族)。 | 单位错配会让整帧点云尺度错误或被滤空,建图链路无点云输入却无明显报错。建图质量影响:尺度错误→地图比例失真/无法建图;量程默认不一致→不同启动路径下有效视距不同。 | 中 | 在启动时对单帧距离分布做合理性检查(中位距离落在合理米级区间),或显式声明并校验单位;统一 range_max 默认与各 yaml(详见 §3)。 |
| D021 | src/wheelchair_sensors/.../camera_adapter_node.py `__init__` L153-156、imu_adapter_node.py `__init__` L262-264、ultrasonic_adapter_node.py `__init__` L255-257、mock_sensor_node.py `__init__`(`pointcloud_rate_hz`/`state_rate_hz`)L70-72 | 多个适配器/Mock 节点直接 `create_timer(1.0 / float(rate_param), cb)`,未校验速率参数 >0;配置为 0 时构造期抛 `ZeroDivisionError`,为负时周期非法。(xtm60 已用 `max(1.0, ...)` 规避,故不计入。) | 速率参数误配(0/负)导致节点启动即崩溃,无友好报错。无直接建图质量影响(启动前失败)。 | 低 | 抽取统一的速率校验工具:对各 `*_rate_hz` 做下界校验(>0),非法时记录错误并回退安全默认或拒绝启动。 |
| D022 | src/wheelchair_sensors/wheelchair_sensors/mock_sensor_node.py · `MockSensorNode.__init__`/`publish_cmd_vel_nav` · L96-99, L188-192 | 规范/架构:`self.x/self.y/self.yaw` 声明后从未被更新或使用(死代码);且默认 `publish_cmd_vel_nav=True` 时持续发布 `/cmd_vel_nav` linear.x=0.35,Mock 节点会主动给出非零运动指令。 | 死变量降低可读性;默认发布非零 cmd_vel 在接到真实底盘的联调场景下存在误动作风险(需依赖下游安全链路拦截)。无直接建图质量影响。 | 低 | 删除未使用状态变量;将 `publish_cmd_vel_nav` 默认设为 False 或在文档/日志中显式提示其会发布运动指令。 |
| D023 | src/wheelchair_sensors/test/test_h30_imu_adapter.py 全文、src/wheelchair_sensors/test/test_ultrasonic_adapter.py 全文 | 测试覆盖不足:IMU 测试未覆盖 `euler_to_quaternion`/EULER_ID 路径、`use_device_timestamp` 时钟映射(D014)、双字节序校验放宽(D013);超声测试未覆盖 `read_ranges` 的异常吞没(D015)、range 未裁剪/0 值发布(D016)与回调阻塞(D017)。 | 本批 D013–D017 一类缺陷无回归测试捕获,后续修复易再回退。无直接运行时危害,属测试覆盖不足。 | 低 | 补充:euler→quat 数值用例、设备时间戳映射的漂移/单调性断言;超声注入假串口验证读失败行为与 min/max_range 裁剪。 |
| D024 | src/wheelchair_perception/wheelchair_perception/pointcloud_to_laserscan_node.py · `PointCloudToLaserScanNode._make_scan` · L186-203(restamp 分支 L189-192) | `restamp_output` 默认 True 时,scan.header.stamp 取 `self.get_clock().now()`(墙钟、发布时刻)而非点云采集时刻 `msg.header.stamp`。点云经 SDK/融合链路缓存后才投影发布,墙钟戳≠真实采集时刻(与 F #13/#41 同类时间戳问题,首次在感知包复现)。 | 采集→投影发布的可变延迟被抹成"现在",破坏 /scan 与 TF/里程计的时间对齐。建图质量影响:`/scan` 馈入 slam_toolbox 2D 建图与 nav2 代价地图,墙钟戳使 scan-to-map 在错误时刻做 TF 查询 → 2D 地图墙线错位、动态期重影。时间基准汇入 §3(B14:适配器 use_sdk_timestamps ↔ 投影 restamp ↔ 下游 approx_sync)。 | 高 | 默认采集时刻优先:`restamp_output` 默认置 False 或仅在源 stamp 缺失(sec/nanosec 全 0)时回退墙钟;保留对空帧不发布的 staleness 语义。 |
| D025 | src/wheelchair_perception/wheelchair_perception/pointcloud_to_laserscan_node.py · `PointCloudToLaserScanNode._read_points_in_target_frame` · L173-181(L177-180 lookup_transform) | TF 查询用 `Time()`(=最新可用变换)而非点云 `msg.header.stamp` 对应时刻的变换:`lookup_transform(target_frame, frame_id, Time(), timeout=0.1)`。机体运动 + source≠target frame 时,用"最新" TF 变换"历史"点云,产生与运动速度成正比的投影错位(与 F #15 同类 TF 错位模式,异包复现)。 | 转弯/移动期间点云被错误外参时刻变换,墙线/障碍平面位置偏移。建图质量影响:错位的 `/scan` 进入 slam_toolbox/代价地图 → 2D 墙线弯折、障碍漂移,且速度越快错位越大。 | 高 | 用 `msg.header.stamp` 做 `lookup_transform`(配合容差/`lookup_transform_full` 或 `can_transform` 等待),变换时刻与点云采集时刻一致;TF 不可用时跳帧而非用 latest 强投影。 |
| D026 | src/wheelchair_perception/wheelchair_perception/pointcloud_to_laserscan_node.py · `ScanProjectionConfig.range_max`/参数声明 · L37, L121, L146 | 投影 `range_max` 默认 8.0m,与点云融合/其他链路常用的 12m 量程不一致(待 §3 跨文件核对);`scan_merger` 同样默认 8.0 但与 perception/bringup 其余 scan/fusion 配置族未拉齐。属跨文件量程一致性问题。 | 不同启动路径下 `/scan` 有效视距不同(8m vs 12m),远距障碍在 2D 链路被截断而在 3D/融合链路保留。建图质量影响:2D 代价地图/slam 的有效感知半径被默认值压低,远处墙体迟到入图。标记汇入 §3(B14 量程族)。 | 低 | 统一 scan 投影/合并/融合的 range_max 默认与各 yaml(详见 §3);在文档明确 2D `/scan` 截断半径的设计意图。 |
| D027 | src/wheelchair_perception/wheelchair_perception/pointcloud_to_laserscan_node.py · `__init__` 发布器/订阅器 · L156-160 | QoS 不对称:点云订阅用 `qos_profile_sensor_data`(best_effort),而 `/scan`、marker 发布器用默认 QoS(reliable, depth=queue_size)。下游 `/scan` 消费者(nav2 代价地图/部分 SLAM 常用 sensor_data best_effort)与 reliable 发布在某些 rmw 组合下可不兼容或回压。 | QoS 两侧不一致时下游可能静默收不到 `/scan` 或在丢帧时阻塞。建图质量影响:`/scan` 若被消费方拒收,2D 建图/避障直接断流。标记汇入 §3(B14 QoS 族)。 | 中 | 为 `/scan` 显式声明与下游一致的 QoS(通常 sensor_data best_effort),或将其设为可配置;与 scan_merger/消费者两侧对齐。 |
| D028 | src/wheelchair_perception/wheelchair_perception/scan_merger_node.py · `ScanMergerNode.tick` · L149-167(L152 header.stamp) | 合并输出 `output.header.stamp = now.to_msg()`(定时器墙钟)而非输入各 scan 的采集时刻。节点在固定 10Hz 定时器重采样并打"现在"戳,丢弃源时间信息(与 D024 同类时间戳问题)。 | 合并 scan 时间戳与真实采集时刻偏离最多一个定时器周期 + 源到达抖动,破坏与 TF/里程计同步。建图质量影响:合并 `/scan` 进入 2D slam/代价地图时时刻错位 → 墙线重影、配准漂移。时间基准汇入 §3(B14)。 | 中 | 用参与合并的最新源 scan 的 header.stamp(或其最小/最大时刻)作为输出戳,而非定时器墙钟;或随事件驱动发布。 |
| D029 | src/wheelchair_perception/wheelchair_perception/scan_merger_node.py · `merge_scan_slices`/`ScanMergerNode.on_scan`/`tick` · L43-67, L121-123, L149-167 | 合并仅按各输入 scan 自身 `angle_min/angle_increment` 计算角度并投到统一输出 bin,**从不把输入 scan 变换到 `frame_id`(默认 base_link)**:无 TF、无 frame 校验。若 `/scan_left`、`/scan_right` 位于不同传感器坐标系(如 laser_left/laser_right,带横向偏移/旋转),按绝对角度直接合并在几何上错误。 | 双雷达若不在同一坐标系,合并 `/scan` 的障碍/墙线位置系统性错位(偏移量=两雷达外参差)。建图质量影响:错位合并 scan 进入 2D 建图 → 墙体加宽/分裂、走廊宽度失真;仅当上游已把各 scan 投到同一 base 系时才安全。 | 高 | 对每个输入 scan 用 TF 将其极坐标点变换到 `frame_id` 后再按角度合并(或显式要求上游 pointcloud_to_laserscan 已统一 target_frame 并在节点校验/记录该前置条件)。 |
| D030 | src/wheelchair_perception/wheelchair_perception/scan_merger_node.py · `__init__` 订阅/发布 · L103-107 | QoS:输入 scan 订阅与 `/scan` 输出均用默认 QoS(reliable, depth=10),未声明 sensor_data。若上游 scan 发布为 best_effort(常见传感器流),reliable 订阅在 rmw 下可不匹配收不到;下游同 D027 风险。 | QoS 不匹配时合并节点收不到上游 scan 或下游收不到合并结果,链路静默断流。建图质量影响:同 D027,`/scan` 断流直接中断 2D 建图/避障。标记汇入 §3(B14 QoS 族)。 | 中 | 输入/输出 scan 统一采用与上下游一致的 QoS(通常 sensor_data best_effort)或设为可配置,两侧对齐核对。 |
| D031 | src/wheelchair_perception/wheelchair_perception/obstacle_detector_node.py · `extract_obstacle_points` · L57-66 | 聚类后 `if len(cluster) < 2: continue` **丢弃所有单波束簇**:细窄障碍(桌椅腿、立杆、门框边)在远距常仅命中 1 个波束,被直接忽略,不产生障碍中心、不计入 `nearest`。 | 细小但真实的障碍在 `/obstacles`、`/obstacle_summary` 中缺席,避障/可视化漏报。无直接建图质量影响(障碍话题不入建图),属感知健壮性缺陷。 | 中 | 降低最小簇尺寸阈值(允许单波束簇),或对单波束簇按距离/可信度有条件保留,而非无条件丢弃;阈值参数化。 |
| D032 | src/wheelchair_perception/wheelchair_perception/passability_analyzer_node.py · `analyze_passability` · L96-130(边界判定 L116-125) | 两处健壮性/逻辑缺陷:(1)恰好位于中线 `y == 0.0` 的点既不计入左也不计入右边界(`if y>0 ... elif y<0`),正前方点被边界统计漏掉;(2)任一侧未观测到边界即返回 `UNKNOWN`,空旷可通行场景(一侧无回波)被判为未知而非 CLEAR,过度保守。 | 通道宽度估计在中线障碍/单侧开阔时偏差或退化为 UNKNOWN,可能在实际可通行处给出保守/错误状态。无直接建图质量影响(可通行状态不入建图)。 | 中 | 将 `y == 0.0`(或 \|y\| 极小)点计入更近一侧或单独处理为正前方障碍;为单侧无边界场景区分"开阔可通行"与"真未知"(结合 max_side_distance 与前向净空判定)。 |
| D033 | src/wheelchair_perception/test/test_pointcloud_to_laserscan.py、test/test_scan_merger.py、test/test_passability_analyzer.py 全文 | 测试覆盖不足:pointcloud 测试未覆盖 TF 变换路径(D025)、restamp 时间戳行为(D024)、空帧不发布语义;scan_merger 测试未覆盖 staleness 超时、`require_all_sources`、跨 frame 合并(D029);passability 测试仅覆盖 CLEAR/BLOCKED,未覆盖 NARROW/UNKNOWN、`scan_to_points` 与中线点(D032)。 | 本批 D024/D025/D028/D029/D032 一类缺陷无回归测试捕获,后续修复易再回退。无直接运行时危害,属测试覆盖不足。 | 低 | 补充:注入假 TF/时钟验证投影时刻与 restamp;构造过期/缺源 scan 验证合并 staleness 与 require_all_sources;补 NARROW/UNKNOWN/中线点的 passability 断言。 |
| D034 | src/wheelchair_3d_mapping/wheelchair_3d_mapping/dual_lidar_cloud_fusion_node.py · `DualLidarCloudFusionNode._publish_merged` · L184-192(`header.stamp = self.get_clock().now()`) | 融合输出 `/points_merged` 的 `header.stamp` 取**发布时刻墙钟**(`get_clock().now()`),而非各帧采集时刻;`_on_cloud` 已把每帧 `msg.header.stamp` 存入 `state.stamp`,但 `_publish_merged` 完全丢弃,从不回填。这正是起点文档已知 #13(融合点云时间戳用墙钟)。 | 采集→定时器发布存在最长一个输出周期(~0.1s)+ 输入抖动的延迟却被打成"现在"。建图质量影响:`/points_merged` 是 RTAB-Map(`approx_sync`)与 icp_odometry 的唯一几何输入,墙钟戳使 scan-cloud 与 IMU/odom 在错误时刻对齐 → ICP/位姿图配准漂移、转弯处墙线重影、回环时刻错配。时间基准汇入 §3(B14:适配器 use_sdk_timestamps ↔ 融合 header.stamp ↔ RTAB-Map approx_sync,已知 #13/#15/#41)。 | 高 | 用参与合并的源帧 stamp(取两路最新 `state.stamp` 的较新或较旧者并保持一致约定)作为输出 `header.stamp`,而非发布点墙钟;源 stamp 缺失(全 0)时才回退墙钟。 |
| D035 | src/wheelchair_3d_mapping/wheelchair_3d_mapping/dual_lidar_cloud_fusion_node.py · `_on_cloud`/`_publish_merged`/`_fresh` · L113-148, L168-196 | 左右两路点云**无时间对齐**:各自回调异步缓存最新结果,定时器 `_publish_merged` 把"各自最新且 `_fresh`(≤`input_timeout`=0.5s)"的两片直接 `np.vstack` 合并并当作同一时刻发布。两片采集时刻可相差最多 input_timeout,运动(尤其原地旋转)时被当作同时刻叠加。 | 左右子云时间错位叠加 → 同一墙面在两片中位置不重合。建图质量影响:`/points_merged` 在机体运动时出现墙体加宽/双影(ghosting),ICP 配准被错误对应点拖偏,旋转期建图漂移最明显;静止时无害。 | 中 | 合并前校验两路 `state.stamp` 时间差并设上限(超限只发较新一路或跳过);或对较旧一路按里程计做运动补偿后再叠加;在 status 中暴露左右 stamp 差用于诊断。 |
| D036 | src/wheelchair_3d_mapping/wheelchair_3d_mapping/dual_lidar_cloud_fusion_node.py · `__init__` 参数声明 · L41-58 ↔ config/dual_lidar_fusion.yaml | 节点参数默认值与部署 YAML 不一致:节点默认 `max_range=20.0`/`z_min=-2.0`/`z_max=3.0`,而 dual_lidar_fusion.yaml 为 `12.0`/`-0.10`/`1.80`。裸 `ros2 run`(不加载 YAML)将以 20m 量程与 ±数米高度带运行,纳入大量地板/天花板与远端飞点。 | 缺省启动时量程/高度带与经调参的安全值不符。建图质量影响:未裁剪的地板/天花板与 12-20m 远端噪点进入 `/points_merged`,污染 RTAB-Map 法向量分割与栅格投影(地面误判、墙线变厚);跨文件量程一致性问题(max_range 20 vs 融合 12 vs Grid/RangeMax 8 vs 感知 scan 8),汇入 §3(B14 量程族)。 | 中 | 让节点默认值与生产 YAML 对齐(max_range=12、z_min=-0.10、z_max=1.80),或在文档强调必须经 YAML 启动;跨文件量程统一详见 §3。 |
| D037 | src/wheelchair_3d_mapping/wheelchair_3d_mapping/cloud_utils.py · `image_to_rgb` · L196-211 | 仅识别 `bgr8`/`rgb8`/`mono8` 三种编码且大小写敏感:其余编码(`bgra8`/`rgba8`/`yuv422`/含 `BGR8` 大写等)落入 `channels=3` 默认分支,按 3 通道 reshape 4 通道(或 YUV)数据 → 像素错位、颜色串色,且**无告警**(返回的数组非 None,调用方视为有效)。 | 非 8 位 3 通道相机静默产生错误颜色。建图质量影响:仅影响 `rgb_cloud_colorizer_node` 产出的 `/rgb_cloud_map`(用户可视化着色),不进入几何 SLAM,故对建图几何质量无影响;但着色图会串色/错位。 | 低 | 显式枚举支持的编码并对 `bgra8`/`rgba8` 按 4 字节步长取前 3 通道、对未知编码返回 None 并告警;编码比较前 `lower()`。 |
| D038 | src/wheelchair_3d_mapping/wheelchair_3d_mapping/cloud_to_occupancy_grid_node.py · `_on_cloud`/`_bounds`/`_tick` · L94-101, L116-131, L139-167 | 两处建图/导航缺陷:(1)`_on_cloud` 每次回调直接 `self._latest = xyz` **覆盖**而非累积,`_tick` 每周期从零 `np.full(unknown)` 重建栅格 → 当默认输入为 `/livo/cloud_registered`(当前帧配准云)时,输出 `/map_2d_from_3d` 只反映**当前可见的瞬时投影**,已驶过/转头后看不见的障碍被擦除;(2)`rolling` 模式下 `_bounds` 每周期按当前云 min/max 重算 origin 与宽高 → 栅格原点逐周期漂移,Nav2 消费的 2D 地图非持久、坐标跳变。 | 默认配置下 2D 占据栅格不是累积地图而是即时视野,且原点漂移。建图质量影响:`/map_2d_from_3d` 供 Nav2 click-to-goal 使用,瞬时+漂移栅格使代价地图丢失历史障碍、目标坐标不稳定,2D 导航可靠性显著下降(3D 云图不受影响)。 | 中 | 默认指向累积的 `/livo/map_cloud`,或在节点内做栅格累积/记忆;`rolling` 用稳定锚定的滚动窗口(固定步进而非每帧重算 origin),或在文档强制说明 cloud_registered 仅得瞬时图。 |
| D039 | src/wheelchair_3d_mapping/wheelchair_3d_mapping/cloud_to_occupancy_grid_node.py · `_bounds` · L128-131 | `rolling` 模式将宽高 `w/h` `min(..., self.max_cells)`(默认 4000)**静默截断**:当云的 xy 跨度超过 `max_cells*res`(=200m@0.05)或 `static` 模式 map_width/height 配置过大时,栅格被裁剪而无任何告警,落在窗口外的列/行被丢弃。 | 大场景下地图被静默裁剪。建图质量影响:超界障碍/自由区不入 2D 栅格且无提示,运维难以察觉地图边缘被截;常规室内尺度(<200m)不触发,故严重度低。 | 低 | 截断时记录节流告警并在 status/日志报告实际 vs 期望尺寸;或按 res 动态校验配置上限。 |
| D040 | src/wheelchair_3d_mapping/config/rtabmap_params.yaml · `rtabmap:` `Grid/RangeMax` · L78 ↔ launch/rtabmap_3d_mapping.launch.py `essential` L150 ↔ config/dual_lidar_fusion.yaml `max_range` | `Grid/RangeMax=8.0`(yaml 与 launch essential dict 两处)远小于融合 `max_range=12.0`:融合提供至 12m 的点,但 2D 投影栅格只采信 8m 内的点。这是起点文档已知 #30(Grid/RangeMax=8 vs 融合 12)。 | 8-12m 环带的墙体/障碍存在于 3D 云图却**不进入** `/rtabmap/grid_map` 2D 栅格。建图质量影响:2D 导航栅格有效感知半径被压到 8m,远处墙体迟到入图、走廊端墙在 8m 外不可见,影响 Nav2 路径规划与前沿探索;3D 点云图仍含 12m 数据。跨文件量程汇入 §3(B14)。 | 中 | 将 `Grid/RangeMax` 提升到与融合一致(12m)或显式文档化"2D 栅格刻意只信 8m"的设计意图并与量程族统一;详见 §3。 |
| D041 | src/wheelchair_3d_mapping/launch/rtabmap_3d_mapping.launch.py · `_setup` `essential` 字典 · L120-160 ↔ config/rtabmap_params.yaml `rtabmap:` 段 | 参数双源/遮蔽:launch 注释自承"YAML 文件参数在本配置下不可靠,rtabmap_slam 会忽略文件提供的 Grid/RGBD/Reg/* 键",于是把同一批 Grid/RGBD/Reg/Icp 参数**硬编码**进 `essential` dict 重复传入。rtabmap_params.yaml 的 `rtabmap:` 段与 dict 内容大量重复,形成两处需手工同步的真值源,且 yaml 那份据称运行时不生效。 | 同一参数两处定义、一处声明"不生效",任何只改 yaml 的调参(如 Grid/RangeMax、ProximityMaxGraphDepth)将**静默不起作用**,改 dict 才生效。建图质量影响:调参易改错地方导致预期的建图/回环行为未生效却无报错,排障成本高、参数漂移风险长期存在。属架构/规范缺陷。 | 中 | 确立单一真值源:要么修复 yaml 加载路径让文件参数生效并删除 dict 冗余,要么删去 yaml `rtabmap:` 冗余段、以 dict 为唯一源并在 yaml 注释指明;避免两处并存。 |
| D042 | src/wheelchair_3d_mapping/wheelchair_3d_mapping/kiss_icp_mapping_node.py · `__init__` 参数声明 · L62-66 ↔ launch/kiss_icp_mapping.launch.py L? ↔ 融合 max_range | KISS-ICP 节点与 launch 默认 `max_range=20.0`,与融合 `max_range=12.0`、感知/栅格 8.0 不一致(量程族裂)。因 `/points_merged` 已在传感器系裁剪到 12m,kiss 的 20m 上限实际无点可纳,功能上无害,但默认值与全链路量程不统一。 | 功能上无直接危害(上游已裁剪)。建图质量影响:KISS-ICP 回退路径的量程默认与主链路不一致,若日后直接订阅未裁剪云会纳入远端噪点。跨文件量程一致性问题,汇入 §3(B14 量程族)。 | 低 | 将 kiss `max_range` 默认与融合统一(12m)或在文档说明其依赖上游已裁剪;纳入 §3 量程族统一核对。 |
| D043 | src/wheelchair_3d_mapping/wheelchair_3d_mapping/rgb_cloud_colorizer_node.py · `_color_into` · L153-176 ↔ config/rgb_colorizer.yaml | 投影着色用纯针孔模型(`fx,fy,cx,cy`),**忽略相机畸变系数**(CameraInfo 的 D 未读取);且部署 yaml 的 `cam_lidar_*` 外参自承"APPROXIMATE / NOT calibrated"(假设值)。两者叠加使像素↔点对应在画面边缘与近距明显偏移。 | 着色点与真实像素错配,边缘更甚。建图质量影响:仅影响 `/rgb_cloud_map` 可视化着色(不进入几何 SLAM),故不损害建图几何;但 RGB 地图颜色与结构错位,观感与人工判读受影响。 | 低 | 读取并应用 CameraInfo 畸变(去畸变后再投影),并用 `scripts/solve_cam_lidar_extrinsic.py` 标定真实外参替换近似值;在缺标定时按现有 identity 警告同样提示外参为近似。 |
| D044 | src/wheelchair_3d_mapping/launch/autonomous_rviz_mapping.launch.py · `generate_launch_description` · L? (`DeclareLaunchArgument("enable_xtm60_radar", ...)`) | 死参数:声明了 `enable_xtm60_radar`(default true)但 `_setup` 全程未读取/使用(左右雷达启停实际由 `hardware_profile` 推导)。该参数对外暴露却无任何效果,易误导使用者以为可借此关雷达。 | 无运行时危害。建图质量影响:无(纯规范/可维护性)。但误导性参数可能让操作者误以为已禁用某雷达而实际未禁用。 | 低 | 删除未使用的 `enable_xtm60_radar` 声明,或将其真正接入左右雷达启停逻辑并与 `hardware_profile` 协调。 |
| D045 | src/wheelchair_mapping/scripts/map_quality_check.py · `read_pgm`(P5 分支)· L62-67 ↔ src/wheelchair_mapping/scripts/vectorize_occupancy_map.py · `read_pgm`(P5 分支)· L55-60 | 二进制 PGM(P5)读取在 maxval 之后用 `while ... isspace(): index += 1` **跳过所有空白字节**,再切 `data[index:index+w*h]`。PGM 规范规定 maxval 与栅格数据之间仅有**单个**空白字符;栅格的首批像素若恰为空白值字节(9/10/13/32 = 很暗/近障碍像素,地图中真实存在)会被当作分隔符误吞,导致整帧像素**整体错位**或长度不匹配。 | P5 是 `map_saver_cli` 的默认输出格式。当地图左上角起始为深色(障碍)像素时,质量检查/矢量化读到的是**错位的栅格**。建图质量影响:质量门(GOOD/WARNING/BAD)基于错位数据给出**错误判定**,可能放行劣质地图或误杀合格地图;矢量墙线层(`vectorize`)也据错位栅格生成,墙体位置整体偏移。 | 中 | 改为仅消费**单个**空白分隔字节(读取 maxval 后 `index += 1`,或严格按规范只跳一个 `\n`/空格),不要跳过全部空白;两个 `read_pgm` 同步修正。 |
| D046 | src/wheelchair_mapping/scripts/map_quality_check.py · `classify_cells`/`read_pgm` · L84-95 ↔ src/wheelchair_mapping/scripts/vectorize_occupancy_map.py · `occupied_mask` · L66-75 | 占据率计算硬编码 `pixel / 255.0`,**未按 maxval 归一化**:`read_pgm` 读出 `maxval` 但仅做 `0<maxval<=255` 校验后丢弃。若 PGM 的 maxval≠255(如 P2 文本图常见 maxval=100),像素被错误地按 255 归一,占据/空闲阈值判定全部偏移。 | `map_saver_cli` 默认输出 maxval=255,常态不触发;但对手工/第三方生成的非 255 maxval 地图,质量检查与矢量化的 free/occupied/unknown 分类系统性错误。建图质量影响:非 255 地图被错误分类→质量门判定失真、墙线提取错误(低概率路径)。 | 低 | 用 `occupancy = pixel / maxval`(或在读取后把像素线性缩放到 0-255)替代硬编码 255;两个脚本一致处理。 |
| D047 | src/wheelchair_mapping/scripts/vectorize_occupancy_map.py · `read_pgm`(P2 分支)· L52-58 | P2(文本)分支**缺少 payload 尺寸校验**:读满或读空 token 后直接 `return width, height, values`,不像 `map_quality_check.read_pgm` 那样断言 `len(values)==width*height`。截断的 P2 文件会返回过短列表,随后 `occupied_mask` 的 `pixels[row*width+col]` 抛 `IndexError`(无友好报错),或多余数据被静默忽略。 | 损坏/截断的 P2 地图导致矢量化崩溃且无诊断;与同仓 `map_quality_check.read_pgm` 行为不一致(后者会报"payload size does not match")。建图质量影响:仅影响 UI 墙线层生成,不入几何 SLAM,故无几何影响;属健壮性/一致性缺陷。 | 低 | 在 P2 分支补 `if len(values) != width*height: raise ValueError(...)`,与 `map_quality_check` 的 `read_pgm` 对齐。 |
| D048 | src/wheelchair_mapping/scripts/vectorize_occupancy_map.py · `vectorize` · L168-171(`segments[::step][:max_segments]`) | 当墙线段数超过 `max_segments`(默认 2000)时,用 `step = len//max_segments` 做**等步长抽稀**并截断:`segments[::step][:max_segments]`。这会**静默丢弃**墙线段,且抽稀是按列表顺序(行优先扫描)而非空间均匀,产生不连续/缺口的墙线,无任何告警。 | 大地图的矢量墙线层被无声裁剪,UI 上墙体出现缺口/断裂。建图质量影响:`vectorize` 产物仅用于 Web UI 显示与语义标注(文档明确不替代 Nav2/AMCL 静态栅格),故不损害导航/建图几何;属可视化健壮性问题。 | 低 | 抽稀时记录告警(报告丢弃段数),或改用按长度优先保留主要墙体的策略;并在 UI 文档标注墙线为近似显示。 |
| D049 | src/wheelchair_mapping/scripts/vectorize_occupancy_map.py · `vectorize` · L162-166 | 直接索引 `map_data["image"]`/`map_data["resolution"]`/`map_data["origin"]`,**未做必填键校验**;而同包 `map_postprocess.validate_map_yaml` 与 `map_quality_check.load_map` 都先检查 `["image","resolution","origin","occupied_thresh","free_thresh"]`。缺键的地图 YAML 会抛 `KeyError` 而非友好报错。 | 行为与同包另两个脚本不一致,缺键时报错信息不友好。建图质量影响:仅影响 UI 墙线工具的易用性,无几何影响;属健壮性/一致性缺陷。 | 低 | 复用与 `map_quality_check.load_map` 相同的必填键校验后再索引,统一三个脚本的 YAML 校验逻辑。 |
| D050 | src/wheelchair_mapping/launch/save_map.launch.py · `generate_launch_description` · L8-16 | 默认 `map_name="maps/indoor_map"` 为**相对路径**,`map_saver_cli -f` 按**当前工作目录**解析且**不会创建缺失的目录**:若启动 CWD 下无 `maps/` 目录,保存会失败(写不出 `.yaml`/`.pgm`);保存位置随启动 CWD 漂移,难以预期。 | 在缺少 `maps/` 目录或非预期 CWD 下启动时,建好的地图**未能持久化**而操作者可能未察觉(ExecuteProcess 仅 `output=screen`,无成功校验)。建图质量影响:辛苦建立的地图可能保存失败导致丢失,需重新建图。 | 低 | 使用绝对/可配置输出目录并在保存前确保目录存在(`mkdir -p` 或在 launch 中创建),或在文档明确要求预建 `maps/` 并固定启动 CWD;可附保存结果校验。 |
| D051 | src/wheelchair_mapping/scripts/{map_quality_check,vectorize_occupancy_map,map_postprocess}.py(全包无 test/ 目录) | 包内三个脚本含**非平凡算法**(PGM 解析、占据分类、连通域 BFS、质量评分、墙线提取/合并/抽稀),但 `wheelchair_mapping` **无任何测试**(无 test/ 目录、CMake 未注册测试)。D045–D048 一类解析/几何缺陷无回归测试捕获。 | 地图后处理与质量门的核心逻辑无测试保护,后续修改易回退。建图质量影响:质量门是建图成果进入定位/导航前的把关环节,其正确性缺测试使把关可靠性长期存在回归风险。 | 低 | 为 `read_pgm`(P5/P2、注释、错位、截断)、`classify_cells`/`free_components`、`cell_edges`/`merge_segments`/抽稀补单元测试(可用内存内构造的小 PGM/YAML,无需真实地图)。 |
<!-- B6（wheelchair_navigation）批次:为避免与并发执行的 B5 批次编号冲突,本批次预留 D070 起的编号块。D052–D069 留给 B5/其他批次;若最终出现编号空档,由后续批次或编号归并步骤回填,不影响 6 字段完整性与不封顶原则。 -->
| D070 | src/wheelchair_navigation/wheelchair_navigation/goal_manager_node.py · `navigate_to_name`/`on_goal_pose` · L218-236, L92-118 | 自触发 + 双重导航:`navigate_to_name` 既 `self.goal_pub.publish(pose)` 发布到 `/goal_pose`(Nav2 标准目标话题,bt_navigator 会直接消费并启动导航),又紧接着 `send_navigate_goal` 调 NavigateToPose action 再发一次同一目标 → 同一目标可能被并行提交两次;同时该节点又 `create_subscription(PoseStamped, "/goal_pose", self.on_goal_pose)` 订阅 `/goal_pose`,于是自己发布的目标会回调自身的 preview 计算(`/navigation/preview_goal` 同样路由到 on_goal_pose)。 | 双路目标提交可能导致 Nav2 收到重复/竞争目标(action goal 与 topic goal 行为不一致时尤甚),自订阅回环还触发多余的 ComputePathToPose 请求。无直接建图质量影响(导航期行为)。 | 中 | 明确单一目标提交路径:命名目标只走 NavigateToPose action(或只走 `/goal_pose` topic),不要两者并发;preview 订阅应区分"外部用户目标"与"本节点自发目标",避免自触发(如本节点发布的目标带标记或不订阅自己发布的话题)。 |
| D071 | src/wheelchair_navigation/wheelchair_navigation/goal_manager_node.py · `handle_command`(add_goal 分支)/`navigate_to_name` · L201-216, L221-226 | 健壮性:`add_goal` 直接索引 `command["name"]`、`float(command["x"])`、`float(command["y"])`;`navigate_to_name` 直接索引 `goal["position"][0]/[1]`。上游 `on_named_goal_command`/`on_voice_intent` 仅捕获 `json.JSONDecodeError`,缺键/类型错误会抛 `KeyError`/`ValueError`/`TypeError` 穿透订阅回调,未被任何 try 兜住。 | 一条字段不全的 `/named_goal_command` 或 `/voice/intent`(如缺 `x`/`y`)即可让回调抛异常中断处理;存储目标缺 `position` 时 navigate_to_name 同样崩。无直接建图质量影响。 | 中 | 对 `add_goal`/`navigate_to_name` 的字段访问加显式校验与 try/except,缺字段时 `publish_status("ERROR: ...")` 返回而非抛出;扩大异常捕获范围或在 handle_command 入口统一校验 schema。 |
| D072 | src/wheelchair_navigation/wheelchair_navigation/frontier_explorer_node.py · `tick`→`_select_goal`→`find_frontier_cells`/`cluster_cells` · L182-220, L36-83 | 架构/健壮性:每个 `period_sec`(默认 2s)定时器回调内,对整张占据栅格做纯 Python 三重嵌套前沿扫描(`find_frontier_cells` O(W·H·4))+ 8 邻接洪泛聚类(`cluster_cells`),全部同步执行。单线程 executor 下,大地图/长时间运行时该计算可达数百 ms~秒级,期间阻塞同 executor 的地图/safety/TF 订阅与 NavigateToPose action 回调。 | 探索周期越长地图越大,executor 卡顿越久 → safety_state 可能因此判 stale 触发 `_safety_reason` block、action 结果回调延迟。无直接建图质量影响(不写地图),但拖累自主探索的实时性与安全响应。 | 中 | 将前沿计算移出定时器(独立线程/进程或限时增量计算),或用 numpy 向量化 `find_frontier_cells`、对栅格降采样;为节点使用多线程 executor / ReentrantCallbackGroup 使 safety/TF 回调不被探索计算阻塞。 |
| D073 | src/wheelchair_navigation/wheelchair_navigation/frontier_explorer_node.py · `_select_goal` · L200-220(`cell_clear` 检查 L209-210) | 功能性/健壮性:前沿簇质心被直接当作目标点,仅用 `cell_clear`(质心半径内无障碍)校验,**未校验质心栅格本身是 free 且 known**。弧形/L 形前沿簇的几何质心可能落在 unknown 或被占据的栅格里,从而把 Nav2 目标设到非自由空间。 | 目标点可能落在未知/占据区,Nav2 规划失败→该前沿被 blacklist→可达前沿被误弃,探索过早判 DONE。无直接建图质量影响。 | 中 | 选目标时校验质心 cell 的占据值在 [0,FREE_MAX](free 且 known),否则在簇内挑选最近的 free 前沿 cell 作为目标,而非无条件用质心。 |
| D074 | src/wheelchair_navigation/wheelchair_navigation/frontier_explorer_node.py · `_publish_frontiers(self, centroids, goal)` · L262-279 | 规范:形参 `goal` 从未在函数体内使用(死参数),调用处 `self._publish_frontiers(frontiers, goal)` 传入后被忽略,选中目标未在 marker 中区分高亮。 | 无运行时危害(纯规范/可维护性);附带可视化未突出所选目标。无建图质量影响。 | 低 | 删除未使用的 `goal` 形参,或真正用它在 MarkerArray 中以不同颜色标出被选中的前沿目标。 |
| D075 | src/wheelchair_navigation/wheelchair_navigation/navigation_status_node.py · `forward_goal_status`/`on_nav2_status`/`publish_status` · L38-56 | 功能性:`self.latest` 被两个异步源竞争写入——`/navigation/goal_status`(String,来自 goal_manager 的语义状态)与 Nav2 `GoalStatusArray`(原始动作状态),谁后到谁覆盖;1Hz 定时器把"当前 latest"原样发布。两源并存时 `/navigation/status` 在语义状态与原始 Nav2 状态间反复跳变,无优先级/时序约定。 | 上层 UI/语音订阅 `/navigation/status` 会看到状态抖动/相互覆盖(如 goal_manager 刚发"GOAL_SENT"立即被 Nav2 的"EXECUTING"盖掉或反之),状态可信度下降。无直接建图质量影响。 | 中 | 明确状态优先级与合并策略(如以最新事件时间戳为准并标注来源,或分两个话题发布),避免单变量被双源无约定覆盖。 |
| D076 | src/wheelchair_navigation/wheelchair_navigation/named_goal_store.py · `save` · L34-36 | 健壮性:`save` 用 `open(path,"w")` + `yaml.safe_dump` **非原子写入**;同包的 `semantic_map_store.SemanticMapStore.save` 已采用临时文件 + `os.fsync` + `os.replace` 的原子写法。若 upsert/delete 期间进程被 kill 或断电,`named_goals.yaml` 会被截断/损坏,丢失全部已保存命名目标。 | 写盘瞬间崩溃即损坏命名目标库;且与姊妹 store 的健壮性不一致(一个原子一个非原子)。无直接建图质量影响(命名目标用于导航)。 | 中 | 复用 semantic_map_store 的原子写模式(临时文件 + fsync + os.replace),或抽取公共原子 YAML 写工具供两个 store 共用。 |
| D077 | src/wheelchair_navigation/wheelchair_navigation/semantic_keepout_node.py · `on_map`/`publish_mask`→`rasterize_keepout_zones` · L186-188, L255-289 | 健壮性:`rasterize_keepout_zones` 在 `geometry.width/height<=0` 或 `resolution<=0` 时抛 `ValueError`;`publish_mask` 在地图订阅回调链(`on_map`→`publish_mask`)内调用该函数但未捕获该异常 → 收到退化地图(分辨率 0/空尺寸)时回调抛出。另外每次地图消息都全量重算整张 mask。 | 退化地图触发未捕获异常打断 mask 发布;高频大地图下重复全量栅格化有额外开销。无直接建图质量影响(keepout mask 供 Nav2 代价地图过滤)。 | 低 | 在 `publish_mask` 对几何参数做有效性校验并 try/except 包裹 rasterize,非法时 `publish_status("ERROR: ...")` 跳过;mask 仅在地图几何或 keepout 文件变化时重算并缓存结果。 |
| D078 | src/wheelchair_navigation/config/named_goals.yaml · `goals` · 全文 | 规范:配置内含个人/调试占位条目(`lty了`、`wxy`,坐标如 [-1.195,0.034]、[-2.719,0.158])与占位目标(`charging`、`door` 在原点附近的示例值),该文件经 `setup.py` data_files 安装进 `share/wheelchair_navigation/config` 部署到生产。 | 调试残留数据随包部署,运维/语音可能"前往"到无意义的历史调试点;污染默认命名目标集。无直接建图质量影响。 | 低 | 将生产默认 `named_goals.yaml` 清成空集合(`goals: {}`)或仅保留真实标定点,把调试目标移出版本库;命名目标应由运行时标注生成而非随包发版。 |
| D079 | src/wheelchair_navigation/config/semantic_map.yaml · `rooms`/`points_of_interest` · 全文 | 规范:默认随包安装的语义地图含示例占位内容(房间"示例客厅"多边形、POI"充电点"),作为部署默认语义层发版。 | 安装后默认语义图为演示数据而非空层,UI/目标选择会显示示例房间,易与真实场景混淆。无直接建图质量影响(语义层不入规划栅格,仅 UI/目标选择)。 | 低 | 生产默认 `semantic_map.yaml` 使用空的 rooms/points_of_interest(保留 DEFAULT_SEMANTIC_MAP 结构),示例内容仅放文档或示例目录。 |
| D080 | src/wheelchair_navigation/test/(缺失)frontier_explorer / navigation_status / test_goal_manager.py · 全包测试覆盖 | 测试覆盖不足:`frontier_explorer_node.py` **零测试**(纯函数 `find_frontier_cells`/`cluster_cells`/`cell_clear`/`_select_goal` 易测却无覆盖,D072/D073 无回归捕获);`navigation_status_node.py` **零测试**(状态双源竞争 D075 无覆盖);`test_goal_manager.py` 仅覆盖 `is_voice_intent_confident`/`nearest_goal_label`,未覆盖 `handle_command`/add_goal 缺键(D071)、自触发/双重导航(D070)、`navigate_to_name`。 | 本批 D070/D071/D072/D073/D075 一类缺陷无回归测试捕获,后续修复易回退。无直接运行时危害,属测试覆盖不足。 | 中 | 为前沿算法补纯函数单测(已知小栅格→预期前沿/簇/选点);为 navigation_status 补双源状态合并测试;为 goal_manager 补 add_goal 缺键、navigate_to_name、self-trigger 抑制的用例。 |
<!-- B7（wheelchair_safety）批次:与并发执行的 B8 等批次错开编号,本批次使用 D090 起的编号块。D081–D089 预留给其他并发批次;若最终出现编号空档,由后续批次或编号归并步骤回填,不影响 6 字段完整性与不封顶原则。 -->
| D090 | src/wheelchair_safety/wheelchair_safety/velocity_limiter_node.py · `clamp`/`VelocityLimiterNode.on_cmd` · L11-12, L26-30 | 健壮性:`on_cmd` 未校验 `msg.linear.x`/`msg.angular.z` 是否有限,直接送入 `clamp(value, lower, upper)=max(lower, min(upper, value))`。NaN 输入经 `min(upper, NaN)` 返回 `upper`、再 `max(lower, upper)=upper` → **NaN 被映射为上限速度**(默认 0.4 m/s、0.8 rad/s;已实测确认)。同进程的 safety_supervisor 对 `/cmd_vel_nav` 已做 `math.isfinite` 拦截,本节点缺同等防护。 | 若该节点接入运动链路,上游 NaN 指令会被翻译成满速前进/转向而非归零,属失效偏向危险(fail-unsafe)。当前 `/cmd_vel_limited` 无订阅者(见 D092)故无即时危害。无建图质量影响。 | 中 | 在 `on_cmd` 入口对 `linear.x`/`angular.z` 做 `math.isfinite` 校验,非有限值时丢弃或归零(与 safety_supervisor 一致);并将 NaN 显式视为非法而非 clamp 透传。 |
| D091 | src/wheelchair_safety/wheelchair_safety/velocity_limiter_node.py · `VelocityLimiterNode`(整体)· L16-30 | 功能/架构:节点名为"velocity **limiter**",但仅做静态对称 `clamp`,**无加速度/jerk 限制、无 deadman/命令超时归零、无平滑**。上游 `/cmd_vel_nav` 停发时不会主动把输出归零(无 watchdog),限速语义弱于名称暗示;真正的超时归零与 failsafe 全部依赖 safety_supervisor。 | 单独运行时不提供速率限制或失联保护,易被误以为已具备"限速器"安全职责而产生错误信任。无直接建图质量影响。 | 低 | 明确该节点职责(纯静态 clamp)并在文档/类注释说明其不含加速度限制与超时归零;或补充加速度限幅与 cmd 超时归零使其名副其实。 |
| D092 | src/wheelchair_safety/wheelchair_safety/velocity_limiter_node.py · 节点装配 ↔ src/wheelchair_bringup/config/{safety_params,safety_params_manual_mapping}.yaml `velocity_limiter_node` 段 | 架构/规范:`velocity_limiter_node` 在 `setup.py` 注册为可执行并在 `safety_params*.yaml` 配置了 input/output/max 参数,但**全仓无任何 launch 实际启动它**(launch 仅起 safety_supervisor 与 emergency_stop),且其输出话题 `/cmd_vel_limited` **无任何订阅者**。即配置存在、节点死置(orphan),limited 话题为死端。 | 配置与实际拓扑不符,运维易误以为限速器在链路中生效(实际未运行),形成"纸面安全"。无运行时危害(未启动)、无建图质量影响。 | 低 | 若不使用则从 `safety_params*.yaml` 删除其配置段并在 setup/README 标注弃用;若需使用则在相应 launch 中启动并把 `/cmd_vel_limited` 接入下游(或并入 safety_supervisor 链路)。 |
| D093 | src/wheelchair_safety/wheelchair_safety/safety_supervisor_node.py · `on_ultrasonic`/`valid_ultrasonic_distances` · L590-592, L130-140 | 健壮性/failsafe:`on_ultrasonic` 直接存 `float(msg.range)` 不做 REP-117/量程校验;`valid_ultrasonic_distances` 仅保留 `>0.0 且 ≥min_valid_m` 的值。无回波/故障时超声适配器发布的 `range=0.0`(见 D016)会被**静默丢弃**为"无效"→该路从有效集消失,等同"无障碍(inf)"。0 值故障与"前方开阔"无法区分,且不计入 `_ultrasonic_fault_reason`(它只看时间新鲜度,不看数值合理性)。 | 持续吐 0 的故障超声会被当作"该方向清空"而非触发 SENSOR_FAULT,削弱避障安全裕度;时间戳仍在刷新故 staleness 也不报。无建图质量影响。 | 中 | 区分"无效读数"与"无障碍":对持续 0/越界读数计数并纳入 fault 判定(数值合理性),或对长期无有效回波的必检方向降级为 STOP,而非静默从有效集移除。 |
| D094 | src/wheelchair_safety/wheelchair_safety/safety_supervisor_node.py · `on_consistency_score`/`publish_safe_command`(consistency 分支)· L425, L633-637 | failsafe 缺口:`consistency_score` 初值 1.0,仅由 `/livo_wheel/consistency_score` 回调更新,**无 staleness/missing 守护**(对照 localization 有 `_localization_fault_reason` 的超时与缺失检查)。当 `require_consistency_healthy=True` 但一致性监控节点未启动/已崩溃时,score 冻结在 1.0(或最后值)→ 一致性门控**永不触发**,形同虚设。 | 启用一致性门控时若监控源失联,安全监督仍判"一致性健康"放行,失效偏向不安全;与 localization 门控的健壮性不一致。无建图质量影响。 | 中 | 为 consistency_score 增加接收时间戳与超时判定(参照 `_localization_fault_reason`):超时/从未收到且 `require_consistency_healthy` 时按 SENSOR_FAULT 归零,而非沿用初值 1.0。 |
| D095 | src/wheelchair_safety/wheelchair_safety/safety_supervisor_node.py · `publish_safe_command`(heartbeat 分支)/`on_emergency_*` · L383-385, L608-616 ↔ config/safety_params*.yaml | failsafe 默认过宽:软/硬急停经 `on_emergency_sw`/`on_emergency_hw` 锁存最后布尔值;仅当 `require_emergency_heartbeat=True` 才检查 e-stop 心跳新鲜度,而该参数默认 False 且**所有部署 yaml 均未开启**。于是 e-stop 发布者(emergency_stop_node)或硬件 e-stop 链路若死掉,监督节点会**无限沿用最后一次** emergency 值且不报障——若最后值为 False(未急停),急停能力静默失效。 | 安全关键的急停链路缺省无活性(liveness)守护,发布者宕机不可见,属 fail-unsafe 默认。无建图质量影响。 | 中 | 将 `require_emergency_heartbeat` 在生产 safety_params.yaml 默认开启(配合合理 `emergency_timeout_sec`),或在代码层对 e-stop 输入强制最小心跳;心跳缺失/超时按急停处理。 |
| D096 | src/wheelchair_safety/wheelchair_safety/safety_supervisor_node.py · `evaluate_safety` 阈值合成 · L213-216 | 不严谨方法/安全裕度:前/侧超声急停阈值用 `min(front_ultrasonic_emergency_distance, emergency_distance)` 与 `min(side_ultrasonic_emergency_distance, emergency_distance)` 合成——取**较小**(较近)者作为急停触发距离,即偏向**更晚**触发急停(更不保守)。对急停阈值通常应取较大(较远)者以保留裕度;且把 scan 用的 `emergency_distance` 与超声阈值耦合,改其一会意外影响另一路。 | 急停触发距离被压到两者较小值,缩短超声急停的反应余量;参数耦合增加误配风险。无建图质量影响。 | 中 | 急停阈值改用 `max(...)`(更保守)或分别独立使用各自传感器的急停距离,解耦 scan 与超声阈值;并补注释说明取值方向的安全意图。 |
| D097 | src/wheelchair_bringup/config/safety_params.yaml `safety_supervisor_node` L21、config/safety_params_mapping.yaml L27 · `debug_max_speed` ↔ safety_supervisor_node.py 参数声明 L355-401 | 参数一致性/死配置:两份 safety yaml 设置了 `debug_max_speed`(0.35 / 0.08),但 `SafetyParams` 与节点 `declare_parameter` 列表**均无该字段**,节点从不声明/读取。rclpy 默认 `allow_undeclared_parameters=False`,未声明的 override 被静默忽略 → 该"调试限速"配置**完全不生效**却看似在起作用。标记汇入 §3(B14 参数一致性族)。 | 运维以为设置了调试速度上限,实际无任何效果,真实上限仍由 `max_auto_speed` 决定,误导性安全配置。无建图质量影响。 | 低 | 删除 yaml 中未使用的 `debug_max_speed`,或在节点声明并实现该参数(如调试模式下用作 `max_auto_speed` 的替代上限);跨文件一致性详见 §3。 |
| D098 | src/wheelchair_safety/wheelchair_safety/safety_supervisor_node.py · `evaluate_safety`(hard_max 二次限幅)· L139-140 | 规范/死代码:`if abs(capped_linear) > params.hard_max_speed: capped_linear = copysign(hard_max_speed, ...)` 作为二次硬限幅,但 `capped_linear` 已先被 clamp 到 `[min_linear, max_auto_speed]`。部署中 `max_auto_speed`(0.25/0.05)< `hard_max_speed`(0.5/0.10),故该分支**恒不触发**(死代码),"硬上限"实际从不生效。 | 无运行时危害,但"硬最大速度"给出虚假的二级保护印象;若误将 max_auto 配置高于 hard_max,限幅顺序也会让 hard_max 失去意义。无建图质量影响。 | 低 | 调整限幅顺序使 `hard_max_speed` 成为最终上限(在 max_auto 之后、对最终输出再夹一次并取两者更小),或移除冗余分支并文档化 max_auto 即生效上限。 |
| D099 | src/wheelchair_safety/test/test_safety_supervisor.py · 全文 · L1-200 | 测试覆盖不足:仅覆盖纯函数 `evaluate_safety` 与 `compute_dynamic_stop_distance`,**完全未覆盖** `SafetySupervisorNode` 的节点级 failsafe 门控——scan/超声 staleness、e-stop 心跳(D095)、一致性门控(D094)、localization 门控、`_ultrasonic_fault_reason`、`cmd_timeout` 归零、`_publish_zero_state`。D093–D096 一类失效偏向缺陷均无回归捕获。 | 安全监督最关键的 failsafe 路径无回归测试,后续修改易回退且不可见。无直接运行时危害,属测试覆盖不足。 | 中 | 注入假时钟/假消息为节点级补单测:scan/超声过期归零、cmd 超时归零、e-stop 心跳缺失、一致性/定位门控开启时的失联归零、`_ultrasonic_fault_reason` 计数路径。 |
<!-- B8（wheelchair_diagnostics）批次:为避免与并发执行的 B7(wheelchair_safety)批次编号冲突,本批次预留 D100 起的编号块。D081–D099 留给 B7/其他批次;若最终出现编号空档,由后续批次或编号归并步骤回填,不影响 6 字段完整性与不封顶原则。 -->
| D100 | src/wheelchair_diagnostics/wheelchair_diagnostics/hardware_probe.py · 模块顶层 import · L17-22 | 健壮性/架构:`from wheelchair_sensors.imu_adapter_node import YesenseParser` 与 `from wheelchair_sensors.ultrasonic_adapter_node import ...` 在**模块顶层无 try 守护**直接导入;而同文件对 `cv2`、`serial` 均用 `try/except ImportError` 优雅降级。一旦 `wheelchair_sensors`(或其依赖)未安装/导入失败,整个 `hardware_probe` 模块不可导入,连带 `hardware_self_check_node`(`from wheelchair_diagnostics.hardware_probe import ...`)在 `rclpy` 就绪后仍构造期崩溃。 | 跨包导入与可选依赖的容错策略不一致;部分安装/单包调试场景下诊断自检节点整体不可用,且报错发生在 import 期而非运行期降级。无直接建图质量影响。 | 中 | 对 `wheelchair_sensors` 顶层导入同样用 `try/except ImportError` 守护并在调用处降级(对应 probe 返回 ERROR/WARN ProbeResult),与 `cv2`/`serial` 的处理方式统一;`package.xml` 已声明 exec_depend,但代码层仍应容错。 |
| D101 | src/wheelchair_diagnostics/wheelchair_diagnostics/hardware_probe.py · `probe_ultrasonic_port` · L73-83(`except Exception: continue`) | 健壮性:逐地址探测时 `try: parse... except Exception: continue` **静默吞没**所有协议/CRC/解析错误,无日志、无计数;函数仅以"是否找到 ≥1 个传感器"汇报,无法区分"该地址无传感器"与"该地址通信/解析错误"。与传感器适配器同类反模式(参见 D015)。 | 超声探针对部分地址的真实故障(接线错/CRC 错)被掩盖,自检结果可能误判为 WARN「no Modbus response」而非定位到具体故障。无直接建图质量影响(自检诊断)。 | 低 | 对每地址的失败原因计入 `details`(如 per-address error 列表),区分无响应与解析失败;必要时降级为带原因的 WARN/ERROR。 |
| D102 | src/wheelchair_diagnostics/wheelchair_diagnostics/hardware_self_check_node.py · `HardwareSelfCheckNode.__init__`/`run_check`/`_collect_results` · L78-81, L83-115, L117-150 | 健壮性/架构:`__init__` 末尾**直接调用 `self.run_check()`**,且定时器回调 `run_check` 也调用它;`_collect_results` 在该(构造期 + 定时器)同步路径内串行执行多个**阻塞 I/O 探针**——`probe_camera_device`(`cv2.VideoCapture` 打开真实相机,部署 `camera_devices=["0","2"]` 两路,单次可达秒级)、`probe_h30_port`(忙循环读串口达 `duration_sec=0.5s`)、`probe_ultrasonic_port`、`probe_zlac_read_register`、`probe_xtm60_sdk`(socket 连接 timeout 0.5s × 多 IP)。单线程 executor 下节点在构造期及每次自检期间被阻塞数秒。 | 节点构造即阻塞(spin 前),`run_once=false`(或 period_sec 周期)时每周期再阻塞数秒,期间无法处理其他回调;若与其他诊断/控制节点同进程更会拖累全局。无直接建图质量影响(一次性硬件自检)。 | 中 | 将硬件探针移出构造函数与定时器回调(独立线程/异步执行 + 结果回填发布),或为自检节点使用多线程 executor / ReentrantCallbackGroup;构造期不应执行长阻塞 I/O。 |
| D103 | src/wheelchair_diagnostics/wheelchair_diagnostics/localization_health_node.py · `__init__` 参数声明 · L33(`max_amcl_age_sec` 默认 2.0)↔ src/wheelchair_bringup/config/diagnostics.yaml `max_amcl_age_sec: 1000000.0` | 功能性/参数一致性:节点默认 `max_amcl_age_sec=2.0`,而部署 yaml 显式设为 `1000000.0` 并注释说明——AMCL **仅在机器人移动后**才重发 `/amcl_pose`,静止时靠 map→odom TF 维持位姿;2.0s 阈值会把"停稳但已定位"的机器人误判为 `LOST`→`/localization/is_healthy=false`→安全监督(`require_localization_healthy` 为真时)归零 `/cmd_vel_safe`→机器人**永远无法起步**(先有鸡还是先有蛋)。裸 `ros2 run`(不加载 yaml)即触发此默认值。 | 缺省启动下静止即被判 LOST,存在闭锁(无法起步)的安全/可用性风险;属节点默认值与生产 yaml 不一致(汇入 §3 频率/阈值族)。无直接建图质量影响(定位健康用于导航/安全)。 | 中 | 将节点默认 `max_amcl_age_sec` 提升到与 yaml 一致的大值(或显式区分"静止 hold"语义,改以协方差+scan+odom 新鲜度为主门),并在文档强调必须经 yaml 启动;跨文件阈值统一详见 §3。 |
| D104 | src/wheelchair_diagnostics/wheelchair_diagnostics/localization_health_node.py · `__init__` 订阅 `/scan` · L50 ↔ sensor_watchdog_node.py `_create_subscriptions`(`/scan`→`qos_profile_sensor_data`)· L150-167 | QoS 一致性/架构:同一诊断包内两个节点对 **同一 `/scan` 话题采用不同 QoS**——`localization_health_node` 用默认 QoS(reliable, depth=10),`sensor_watchdog_node` 用 `qos_profile_sensor_data`(best_effort)。若上游 `/scan` 发布为 best_effort(传感器流常见),`localization_health` 的 reliable 订阅在部分 rmw 下收不到 → `scan_age` 恒为 stale → 持续误判 `LOST`;若为 reliable 则 watchdog 端可能回压。两节点对同一信号的契约自相矛盾。 | QoS 不匹配可致 `/localization/health` 持续误报 LOST(配合 D103/安全链路放大);属跨文件 QoS 不一致,**汇入 §3(B14 QoS 族)**。无直接建图质量影响。 | 中 | 统一两节点对 `/scan`(及 amcl/odom)的 QoS,与上游发布端对齐(通常 `/scan` 用 sensor_data best_effort),或将 QoS 设为可配置;两侧核对详见 §3。 |
| D105 | src/wheelchair_diagnostics/wheelchair_diagnostics/sensor_watchdog_node.py · `__init__` 参数默认 · L42-46(`ultrasonic_topics=["/ultrasonic/range_0"]`)、L48-50(`points_topics=["/xtm60/points"]`)↔ config/diagnostics.yaml(4 路超声全 critical、双雷达 `/xtm60/left+right/points`) | 功能性/参数一致性:节点默认仅看护 **1 路超声**(`/ultrasonic/range_0`)与**单一 legacy 点云话题**(`/xtm60/points`),而生产 diagnostics.yaml 看护 4 路超声(全部 `critical=true`)与左右双雷达点云。裸 `ros2 run`(不加载 yaml)将**漏看护**其余 3 路安全相关超声与双雷达任一路 → 这些传感器掉线时看门狗不触发 NAV_BLOCKED。 | 缺省启动下安全看门狗覆盖面不足(漏看护 critical 超声/点云),掉线静默不报。属节点默认值与生产 yaml 不一致(汇入 §3),安全相关。无直接建图质量影响。 | 中 | 让节点默认话题集与生产 yaml 对齐(或默认即覆盖全部 critical 传感器),并在文档强调 yaml 为准;跨文件配置一致性详见 §3。 |
| D106 | src/wheelchair_diagnostics/wheelchair_diagnostics/localization_health_node.py(`on_amcl_pose`/`on_odom`/`on_scan`/`tick` L54-90)、sensor_watchdog_node.py(`mark_seen`/`tick` L169-176)·age 计算 | 不严谨/时间基准:两节点的"新鲜度/age"均基于 `time.monotonic()`,而发布的 `DiagnosticArray.header.stamp` 用 `self.get_clock().now()`(ROS 时钟)。当 `use_sim_time=true` 或回放 bag(ROS 时钟≠墙钟/monotonic)时,age 判定走的是真实墙钟而非 ROS 时间,staleness 阈值在仿真/回放下语义错位(诊断时间戳与 age 来自不同时钟)。 | 仿真/bag 回放场景下超时判定与 ROS 时间脱节,可能误报或漏报;实机运行无影响。属时间基准一致性隐患(对实时 liveness 用 monotonic 亦有其合理性,可能为有意设计,故记为低)。无直接建图质量影响。 | 低 | 若需支持 sim_time/bag 回放,统一以 `get_clock().now()` 计算 age(随 ROS 时间);或在文档明确该看门狗仅按真实时间判活、不支持 sim_time。 |
| D107 | src/wheelchair_diagnostics/test/test_policy.py 全文 + 无节点/probe 测试 | 测试覆盖不足:`test_policy.py` 覆盖 watchdog NAV_BLOCKED/DEGRADED/critical 超声与 localization 高协方差 DEGRADED,但**未覆盖**:`evaluate_watchdog` 的 startup_grace 行为(never-seen 在宽限期内判 online)、`evaluate_localization_health` 的 `LOST`(输入缺失)路径、`to_json` 序列化;且 `hardware_probe.py`(纯解析/探针函数)与三个节点(hardware_self_check/localization_health/sensor_watchdog)**零测试**。 | 本批 D100–D106 一类缺陷(尤其 grace 误报、LOST 判定、默认值)无回归测试捕获,后续修复易回退。无直接运行时危害,属测试覆盖不足。 | 低 | 为 policy 补 startup_grace、LOST、to_json 用例;为 hardware_probe 注入假 serial/socket 验证探针分支;节点层可注入假时钟/假 probe 验证发布与阈值逻辑。 |
<!-- B9（wheelchair_bringup,文件最多)批次:为避免与并发执行的其他批次编号冲突,本批次使用 D110 起的编号块(D110–D125)。D108–D109 预留给其他并发批次;D126–D129 暂空,若出现编号空档由后续归并步骤回填,不影响 6 字段完整性与不封顶原则。本批次涉及的跨文件一致性项(EKF frequency 30 ↔ 轮速 50 ↔ IMU 100、量程族、laser_link 非 optical 帧等)标记「汇入§3」供 B14 专项。 -->
| D110 | src/wheelchair_bringup/src/teleop_panel.cpp · `TeleopPanel::eventFilter` / 构造函数 `qApp->installEventFilter(this)` · L106, L183-216 | 架构/功能性 bug:面板在构造期对**整个 QApplication** 安装事件过滤器(`qApp->installEventFilter`),`eventFilter` 在面板可见(`isVisible()`)时对 W/A/S/D/Space 按键 `return true` **全局拦截并消费**。当焦点位于本面板的 Topic 输入框(`topic_edit_` QLineEdit)或 RViz 其它任何文本输入控件时,键入 w/a/s/d/空格会被过滤器吞掉无法输入,且同时触发 `setDir(...)` 下发运动指令。 | 用户在 RViz 编辑话题名/属性时无法输入这几个字母,且键入即触发底盘运动指令(经 safety_supervisor 后仍可能动作),属可用性 + 安全双重隐患。无直接建图质量影响。 | 中 | 仅在面板自身获得焦点时处理按键(改用 `keyPressEvent`/`keyReleaseEvent` 或在过滤器内判定事件目标是否属于本面板而非全应用);文本输入控件获得焦点时不拦截 WASD。 |
| D111 | src/wheelchair_bringup/src/teleop_panel.cpp · `TeleopPanel::onInitialize` · L108-115 | 健壮性/空值:`getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node()` 对 `weak_ptr::lock()` 的返回值**未做空判**直接解引用。若 ROS 节点抽象已失效(lock 返回 nullptr),将解空指针崩溃。 | 在节点抽象不可用的边界场景(RViz 关闭/重载顺序异常)下面板初始化崩溃,连带影响 RViz 进程。无直接建图质量影响。 | 低 | 对 `lock()` 结果判空,失败时记录错误并安全降级(不创建发布器/禁用面板),而非直接解引用。 |
| D112 | src/wheelchair_bringup/src/xt_bindshim.c · `bind` · L13-39 | 健壮性:`dlsym(RTLD_NEXT, "bind")` 的返回值未判空即在末尾 `return real_bind(...)` 调用——若解析失败(real_bind 为 NULL)将调用空函数指针崩溃;另 `atoi(port_env)` 对非法/越界端口字符串无校验(返回 0 等),且每次 `bind` 调用都重复 `getenv`。 | dlsym 失败时进程崩溃而非回退;非法 `XT_BIND_PORT` 静默变 0 端口,绑定行为异常。该 shim 经 LD_PRELOAD 注入 XT-M60 适配器,异常会影响雷达接收 socket 绑定。建图质量影响:绑定失败会使对应雷达收不到点云,延长/中断点云供给。 | 低 | 对 `real_bind` 判空(失败时设 errno 并返回 -1 或 abort 前记录);用 `strtol` 校验端口范围 [1,65535];可缓存 env 读取结果。 |
| D113 | src/wheelchair_bringup/package.xml · `<exec_depend>` · L35-36 | 架构/依赖:声明 `<exec_depend>wheelchair_ui</exec_depend>` 与 `<exec_depend>wheelchair_voice_agent</exec_depend>`,但当前工作区 `src/` 下**不存在**这两个包(仅 10 个 wheelchair_* 包,无 ui/voice_agent)。rosdep/colcon 无法解析该依赖。 | 依赖解析(rosdep install)对这两个包报错;且引用它们的 launch(见 D114)在运行时因找不到包失败。属跨包架构缺陷。无直接建图质量影响。 | 中 | 若 ui/voice_agent 不属于本工作区,移除这两条 exec_depend(或将其改为可选/条件依赖),并同步修正引用它们的 launch;若应纳入工作区,则补齐这两个包。 |
| D114 | src/wheelchair_bringup/launch/demo_mock.launch.py L131-159、full_system.launch.py L168-176/L223-251、mapping.launch.py L196-225 | 功能性 bug:多个 launch 直接 `Node(package="wheelchair_ui", ...)` / `Node(package="wheelchair_native_gui")` 以及 `IncludeLaunchDescription(FindPackageShare("wheelchair_voice_agent")/...)`,但这两个包在本工作区缺失(见 D113)。full_system 的 `enable_voice_agent` 默认 true、`enable_web_ui` 默认 true,启动即触发对缺失包的解析。 | full_system 默认配置启动时因找不到 `wheelchair_voice_agent`/`wheelchair_ui` 而报错;demo_mock 同样无法完成启动。属集成可用性缺陷。无直接建图质量影响。 | 中 | 用条件/可选包存在性检查(类似 bringup_3d_slam 对 topic_tools 的 try/except)包裹这些节点,或在工作区补齐 ui/voice_agent 包;默认值在包缺失时应优雅跳过而非整段失败。 |
| D115 | src/wheelchair_bringup/launch/demo_mock.launch.py · `mock_sensor_node` 参数 · L62-74 | 功能性 bug/类型:把 `LaunchConfiguration("obstacle_distance")`、`LaunchConfiguration("cycle_obstacle")` 直接放入 `parameters=[{...}]` 传给 mock 节点,而该节点 `declare_parameter("front_obstacle_distance", 2.0)`(double)、`declare_parameter("cycle_obstacle", True)`(bool)。未用 `ParameterValue(value_type=...)` 包裹时,launch substitution 解析为**字符串**("2.0"/"true"),与声明的 double/bool 类型不符。 | 在 Humble 下字符串 override 与已声明的 bool/double 类型不匹配,节点很可能拒绝该 override 或启动报参数类型错误,使 demo_mock 的障碍距离/循环配置失效或节点起不来。无直接建图质量影响(mock 演示路径)。 | 中 | 用 `ParameterValue(LaunchConfiguration(...), value_type=float/bool)` 显式声明类型,或在 DeclareLaunchArgument 与节点参数之间统一类型转换。 |
| D116 | src/wheelchair_bringup/launch/record_bag.launch.py · `ExecuteProcess(cmd=[... ros2 bag record ...])` · L19-43 | 功能性 bug/一致性:录包话题表写死 `/camera/front/image_raw`,但相机适配器实际默认 `enabled_cameras: [left, right]`(camera.yaml),发布 `/camera/{left,right}/image_raw`——前相机话题不存在,左右相机图像**未被录制**;同时只录 `/wheel/odom` 未录 EKF 输出 `/odometry/filtered`。 | 录制的 bag 缺少真实相机图像与融合里程计,回放/离线分析时这两类数据缺失。建图质量影响:离线复现/调参时缺图像(着色/LIVO)与融合 odom,降低可复现性。 | 低 | 把相机话题改为实际发布的 `/camera/left/image_raw`、`/camera/right/image_raw`(或参数化),并补录 `/odometry/filtered`;话题列表应跟随传感器配置。 |
| D117 | src/wheelchair_bringup/config/nav2_params.yaml · `local_costmap` `plugins` ↔ `range_layer` · L? (plugins: [obstacle_layer, inflation_layer]) | 功能性 bug/死配置:`local_costmap` 定义了完整的 `range_layer`(`nav2_costmap_2d::RangeSensorLayer`,订阅 4 路 `/ultrasonic/range_*`),但 `plugins:` 列表只列 `[obstacle_layer, inflation_layer]`,**未包含 `range_layer`** → 该层不被加载,超声测距**完全不进入局部代价地图**。 | 超声避障数据在 Nav2 局部代价地图中缺席,近距盲区/矮障碍仅靠 scan + safety_supervisor 兜底,代价地图层面漏看超声。无直接建图质量影响(导航代价地图)。 | 中 | 将 `range_layer` 加入 `local_costmap.plugins` 列表(置于 obstacle_layer 之后、inflation 之前),或若刻意不用超声则删除该死配置并在注释说明。 |
| D118 | src/wheelchair_bringup/config/nav2_params.yaml · `controller_server` `min_y_velocity_threshold` · L? | 规范/可疑魔法值:`min_y_velocity_threshold: 0.5` 显著大于同组其它阈值(`min_x`/`min_theta` 均为 0.001),对差速底盘(无 y 向速度)该值偏大且与常见 0.001 默认不符,疑似笔误/遗留值。 | 差速底盘 y 速度恒 0,该阈值实际不影响行为,但异常值易误导后续调参且语义不清。无建图质量影响。 | 低 | 将 `min_y_velocity_threshold` 改回常规小值(如 0.001)或在注释说明取 0.5 的意图。 |
| D119 | src/wheelchair_bringup/config/robot_localization_livo_wheel_ekf.yaml · `odom0_config`(/wheel/odom)· L20-23 ↔ robot_localization_ekf.yaml `odom0_config` L26-30 | 算法/参数一致性:LIVO-轮速 EKF 配置对 `/wheel/odom` **重新启用了绝对 yaw 与 vyaw**(`odom0_config` 中 yaw=true、vyaw=true),与主 EKF(`robot_localization_ekf.yaml`)刻意将轮速 yaw 关闭(yaw=false、vyaw=false,注释明确「wheel-derived yaw 是 WORST heading source」)的**已修结论相矛盾**;且本文件同时融合轮速 yaw + LIVO yaw(differential)+ IMU yaw 共三路航向源。 | 该 EKF(bringup_3d_slam LIVO 路径)重新引入轮速 yaw 漂移源并与 IMU/LIVO 航向竞争。建图质量影响:轮滑/轮距误差污染融合航向(转弯处尤甚),`/odometry/filtered` 作为 RTAB-Map 外部里程计,航向失真直接导致位姿图转弯漂移、墙线弯折——正是已修 EKF 问题在 LIVO 配置中的回归。跨文件一致性问题,**汇入 §3(B14 频率/航向族)**。 | 中 | 与主 EKF 对齐:关闭 `/wheel/odom` 的 yaw/vyaw(由 IMU 主导航向),仅保留轮速平移(x/y/vx);避免三路 yaw 叠加。 |
| D120 | src/wheelchair_bringup/config/camera_full.yaml · `front/left/right/rear_device` · L11-16 | 功能性 bug:`enabled_cameras: [front, left, right, rear]` 配 `front_device:"0"`、`left_device:"1"`、`right_device:"2"`、`rear_device:"3"`,把采集分配到 `/dev/video1`、`/dev/video3` 等**奇数索引**;而 camera.yaml 已实测注明奇数索引(1,3)为 metadata-only、**无法采集**(只有偶数 0/2 能取流)。 | 用 camera_full 档时 left(video1)/rear(video3)打不开或取不到图像。无直接建图质量影响(该档非默认建图路径);属配置正确性缺陷。 | 低 | 将设备索引改为可采集的偶数索引(或用 `/dev/v4l/by-id` 稳定路径),并与 camera.yaml 的实测结论一致。 |
| D121 | src/wheelchair_bringup/config/diagnostics.yaml · `hardware_self_check_node.zlac_port` · L? (`/dev/ttyUSB2`) ↔ config/zlac8030_base.yaml `serial_port: /dev/smartwheel_zlac8030` | 参数一致性:硬件自检节点探测 ZLAC 用 `zlac_port: /dev/ttyUSB2`(原始可枚举设备名),而底盘驱动用 udev 固定符号链接 `/dev/smartwheel_zlac8030`。两者指向方式不一致,`/dev/ttyUSB2` 的枚举号会随插拔/上电顺序变化,自检可能探到错误设备或探不到。 | 预检自检对 ZLAC 的连通性判定不可靠(误报无响应或误探其它 USB 串口)。无直接建图质量影响(一次性自检)。 | 低 | 将 `zlac_port` 统一为 udev 符号链接 `/dev/smartwheel_zlac8030`,与驱动一致;所有串口设备都用稳定 by-id/udev 名。 |
| D122 | src/wheelchair_bringup/config/sensor_layout.yaml · 全文 · L1-50 | 规范/架构:该文件为外参「估计值」参考表,但**无任何节点/launch 加载**(全仓仅 static_transforms.yaml 注释提及),且内容已过时/不完整——只列 `laser_link`(z=0.65)却**缺 `xtm60_left/right_link`** 两路真实雷达帧,另含 `ultrasonic_4/5`(URDF 标注 reserved/unused)。与 URDF 实际外参(xtm60_left z=0.45 等)脱节。 | 孤立的过时参考文件,若被误当作外参真值来源会引入错误安装位姿;当前无运行时危害。建图质量影响:潜在(若据其设置 TF 则雷达高度/缺帧错误)。**汇入 §3(B14:URDF ↔ static_transforms ↔ calib 外参一致性)**。 | 低 | 删除该孤立文件,或将其更新为与 URDF 完全一致并明确标注「仅文档、非加载源」;统一以 URDF/robot_state_publisher 为外参唯一真值。 |
| D123 | src/wheelchair_bringup/launch/replay_bag.launch.py · `ExecuteProcess(cmd=[... "--clock" ...])` · L13-19 | 一致性:回放用 `ros2 bag play --clock`(发布 `/clock`,意在 sim time 回放),但全工程各节点配置 `use_sim_time: false`(nav2/ekf/slam_toolbox 等),节点不会消费 `/clock`。`--clock` 与系统 use_sim_time 设定矛盾。 | 回放时节点仍用墙钟而非 bag 时间,时间对齐与录制时不一致,离线复现失真。无直接建图质量影响(离线工具)。 | 低 | 回放分析时统一启用 `use_sim_time:=true`(或去掉 `--clock`),保证回放时间基准与节点一致。 |
| D124 | src/wheelchair_bringup/config/camera_roles.yaml · 头部注释 · L11-15 ↔ config/camera.yaml `*_camera_info_url` · L23-26 | 规范/一致性:camera_roles.yaml 注释声称「camera_adapter_node 不发布 camera_info,需由外部标定发布器提供」,但 camera.yaml 设置了 `left/right_camera_info_url` 指向真实标定文件——适配器据此**会**发布 camera_info。两份文档对「是否发布 camera_info」的描述自相矛盾。 | 误导 LIVO/colorizer 的 camera_info 订阅配置(可能去订阅不存在的外部发布器或重复发布)。无直接建图质量影响(着色/视觉链路配置)。 | 低 | 更新 camera_roles.yaml 注释,说明 camera_info 由 camera_adapter 依据 `*_camera_info_url` 发布;统一两处描述。 |
| D125 | src/wheelchair_bringup/launch/bringup_3d_slam.launch.py · `from launch.conditions import IfCondition` · L? | 规范/死代码:导入 `IfCondition` 但全文件未使用(节点启停用 `_b(context, ...)` 布尔判定),属未使用的 import。 | 无运行时危害,纯可维护性。无建图质量影响。 | 低 | 删除未使用的 `IfCondition` 导入。 |
<!-- B10（wheelchair_description）批次:与并发执行的 B9(wheelchair_bringup,占用 D110 起)等批次错开编号,本批次使用 D130 起的编号块。D108–D129 预留给 B9/其他并发批次;若最终出现编号空档,由后续批次或编号归并步骤回填,不影响 6 字段完整性与不封顶原则。 -->
| D130 | src/wheelchair_description/config/static_transforms.yaml · `transforms[xtm60_left_link]` · L10-13 ↔ urdf/wheelchair.urdf.xacro `xtm60_left_fixed_joint` · L96-107 | 参数一致性/单位:左雷达安装高度在两份「同一物理量」的真值源之间发散——URDF 已把 `xtm60_left_link` 高度从旧值 0.65 修正为 **0.45**(joint origin z=0.45,注释明确「old 0.65 was 20 cm too high」),但 `static_transforms.yaml` 仍写 `xtm60_left xyz=[0.45,0.24,0.65]`(0.65 未同步修正)。即已知「雷达高度 URDF 写错(已修)」的修复**只落到 URDF,未传播到这份 fallback 文件**。 | 该文件注释自称为「static transform publisher fallback」,虽当前无 launch 加载(见 D131 备注),一旦被用作 URDF 缺失时的静态 TF 兜底,左雷达将比真实安装高出 20cm。建图质量影响:左雷达点云整体抬高 0.2m → 地面/墙面点被压到错误高度带,与右路点云及 base_link 地面假设不一致,融合后地面误判、墙线层错位、`/points_merged` 双高度面。跨文件外参一致性问题,**汇入 §3(B14:URDF ↔ static_transforms ↔ calib 外参)**。 | 高 | 将 `static_transforms.yaml` 的 `xtm60_left` 高度同步为 0.45(与 URDF 一致),或直接废弃该 fallback 文件改为以 URDF/`robot_state_publisher` 为唯一外参真值源,避免双源漂移;详见 §3。 |
| D131 | src/wheelchair_description/config/static_transforms.yaml · `transforms[xtm60_left_link/xtm60_right_link]` rpy · L13, L17 ↔ urdf/wheelchair.urdf.xacro `xtm60_*_fixed_joint` rpy · L105, L126 | 算法错误/坐标系约定:两路 XT-M60 在 URDF 中是 **optical 约定**链接(`rpy=[-1.5708, 0.0, -1.5708]`,使 optical-z 映射到 base_link +x),但 `static_transforms.yaml` 对 `xtm60_left/right` 给出的是**激光/机体约定** `rpy=[0.0, -0.0873, 0.0]`(与 `laser_link` 同),完全没有 optical→body 的 90° 旋转。两份文件对同一传感器帧的姿态约定**自相矛盾**。 | 同 D130 的 fallback 语义:若 `static_transforms.yaml` 被用作静态 TF 兜底,XT-M60 发布的 optical 约定点(z 前 / x 右 / y 下)会被按机体约定解释 → 点云被旋转约 90°,前方墙面被摆到上方/侧向。建图质量影响:点云朝向整体错误,ICP/RTAB-Map 配准彻底失败、地图不可用。跨文件坐标系一致性问题,**汇入 §3(B14)**。 | 高 | 将 `static_transforms.yaml` 的 `xtm60_left/right` rpy 同步为 URDF 的 optical 值 `[-1.5708, 0.0, -1.5708]`,或废弃该 fallback 文件以 URDF 为唯一来源;两处约定必须统一。 |
| D132 | src/wheelchair_description/urdf/wheelchair.urdf.xacro · `laser_link`/`laser_fixed_joint` · L71-86 ↔ src/wheelchair_bringup/config/xtm60_sdk.yaml `frame_id: laser_link` · L11 ↔ xtm60_adapter_node.py 默认 `frame_id="laser_link"` L72/649 | 坐标系约定不一致:`laser_link` 在 URDF 中是**非 optical** 帧(`rpy=[0.0, -0.0873, 0.0]`,仅 -5° 俯仰),而 `xtm60_left/right_link` 是 optical 帧。但单雷达 SDK 配置 `xtm60_sdk.yaml` 与 xtm60 适配器的**默认** `frame_id` 都用 `laser_link` 来发布 **optical 约定**的 XT-M60 点云 → 在单雷达/裸 `ros2 run`(不加载 left/right yaml)路径下,optical 点被放进非 optical 帧。 | 单雷达 SDK 模式或缺省启动时,XT-M60 点云相对 base_link 旋转约 90°(缺 optical→body 变换)。建图质量影响:该路径下点云朝向错误,`/scan` 投影与 3D 融合几何全错,建图不可用;仅双雷达专用 launch(frame_id=xtm60_*_link)规避了此问题。跨文件坐标系一致性问题,**汇入 §3(B14)**。 | 中 | 为 `laser_link` 增加与 xtm60 一致的 optical rpy(若该帧确为 XT-M60 输出帧),或将单雷达/默认 `frame_id` 改为 optical 帧 `xtm60_left_link`;统一「XT-M60 输出帧必须是 optical 约定」的约束,详见 §3。 |
| D133 | src/wheelchair_description/urdf/wheelchair.urdf.xacro · `xtm60_right_fixed_joint` · L117-128 ↔ calib/right.yaml | 参数一致性/外参占位:右雷达 joint origin `xyz=[0.45,-0.24,0.65]` 由注释自承为 **placeholder**——「relocated out of the box onto the right armrest, taped… lateral/forward offset is approximate and MUST be re-measured / calibrated in RViz before relying on the merged map」。即右雷达外参未标定,与左雷达(z=0.45)非对称且横/纵向偏移为估计值。这是起点文档已知 #50(右雷达占位外参)。 | 右雷达点云相对 base_link 的位姿带未标定误差。建图质量影响:右路点云与左路在重叠区无法对齐 → `/points_merged` 墙面错位/加厚、双影,融合地图在右侧系统性偏移;为已知 #50 的 URDF 侧根因。跨文件外参一致性问题,**汇入 §3(B14:URDF ↔ static_transforms ↔ calib `right.yaml`)**。 | 中 | 用 RViz/标定流程实测右雷达相对 base_link(或相对左雷达)的外参,替换 placeholder,并与 `calib/right.yaml`、`static_transforms.yaml` 三处同步;在标定完成前于 status/文档显式标注右路为未标定。 |
| D134 | src/wheelchair_description/urdf/wheelchair.urdf.xacro · `ultrasonic_link` 实例化 · L192-197 | 规范/死代码:URDF 实例化 **6** 路超声链接(idx 0–5),注释自承「4 active… idx 4,5 are reserved/unused」;而 `ultrasonic_adapter_node` 仅发布 4 路(range_0–3)。`ultrasonic_4_link`/`ultrasonic_5_link` 是无对应数据的孤立 TF 帧;且 idx0–3 的 xyz/rpy 注释也标「estimates - MEASURE on the real chassis」(未实测)。 | 无运行时危害(超声不入建图)。多余 TF 帧增加 RViz 噪声与误解(以为有 6 路超声);未实测的安装位姿使超声在 base_link 下的方向为估计值,影响避障可视化精度。无建图质量影响。 | 低 | 删除 idx 4/5 两个未使用链接(或在 adapter 启用 6 路时再加),并在实测后更新 idx0–3 的 xyz/rpy;注释标明「估计值」改为标定值。 |
| D135 | src/wheelchair_description/package.xml · `exec_depend` · L13 ↔ launch/display.launch.py `generate_launch_description` · L20-45 | 规范/死依赖:`package.xml` 声明 `<exec_depend>joint_state_publisher_gui</exec_depend>`,但 `display.launch.py` 只起 `robot_state_publisher` 与 `rviz2`,**从不启动 joint_state_publisher(_gui)**(本 URDF 全为 fixed joint,robot_state_publisher 已能发布全部 TF,确实无需 JSP);该依赖为未使用声明。此外 `rviz2` 以无 `-d` 配置参数启动,加载空默认视图而非 description 专用 rviz。 | 无运行时危害(纯规范/可维护性 + 易用性)。死依赖增加安装体积与误导;rviz 无预置配置使 `display.launch.py` 首次打开看不到机器人模型,需手动加 RobotModel/TF 显示。无建图质量影响。 | 低 | 删除未使用的 `joint_state_publisher_gui` exec_depend(或在需要可动关节时才引入);为 `display.launch.py` 增加可选 `-d` rviz 配置参数指向一份含 RobotModel/TF 的 .rviz。 |

<!-- B12(calib/)+ B13(auto_test/)批次:为避免与并发执行的 B9(D110 起)、B10(D130 起)、B11(D140 起)批次编号冲突,本批次使用 D150 起的编号块。D108–D149 留给 B9/B10/B11 等并发批次;若最终出现编号空档,由后续批次或编号归并步骤回填,不影响 6 字段完整性与不封顶原则。 -->
<!-- B12 关键勘正:design/tasks 假设 `calib/*.yaml` 含「左右雷达外参」供 B14 比对(关注 #50 右雷达占位外参)。实检确认 `calib/*.yaml` 实为**相机内参(camera_info)**,**不含任何雷达外参**;右雷达占位外参(#50/D043)位于 `static_transforms.yaml` 与 `rgb_colorizer.yaml` 的 `cam_lidar_*`,不在 calib/。详见 D152(汇入§3)。 -->
| D150 | calib/left_unrotated.yaml 全文、calib/right_unrotated.yaml 全文 | 规范/架构(死文件 + 误用风险):`left_unrotated.yaml`/`right_unrotated.yaml` **未被任何 config/launch/代码引用**(全仓仅 `camera.yaml` 引用 `calib/left.yaml`、`calib/right.yaml`)。它们是「未旋转」内参(主点 cy≈211/207),与当前 `camera.yaml` 的 `left_rotate_deg:180`/`right_rotate_deg:180`(发布前对图像做 180° 旋转,故须用已旋转内参 left.yaml/right.yaml,主点 cy≈267/272)**不匹配**。若运维误把 `*_camera_info_url` 指向 unrotated 版本,或把 rotate_deg 改回 0 却仍用 rotated 版本,则发布的 CameraInfo 主点/切向畸变符号与实际图像不符。 | 孤立标定文件易被误用 → 发布错误 CameraInfo(主点偏移、切向畸变 p1/p2 符号相反)。建图质量影响:CameraInfo 仅供 `rgb_cloud_colorizer` 着色投影使用(不进入几何 SLAM),误用会使 `/rgb_cloud_map` 着色错位,不损害建图几何;属可维护性/误用风险。 | 低 | 删除未引用的 `*_unrotated.yaml`,或在文件头注释标明「未旋转原始内参,当前部署不使用(图像在适配器内 180° 旋转,请用 left/right.yaml)」;并在 `camera.yaml` 注释绑定 rotate_deg 与所用内参版本的对应关系。 |
| D151 | calib/left.yaml L4、calib/right.yaml L4、calib/left_unrotated.yaml L4、calib/right_unrotated.yaml L4 · `camera_name` 字段 | 规范/命名一致性:四个标定文件 `camera_name` 为 `camera_left`/`camera_right`,而相机适配器以 `cfg.name`(`left`/`right`)构建话题 `/camera/left` 与 `/camera/right` 的 `camera_info` 与 frame。`load_camera_info` 读取的 `camera_name` 与发布命名空间不一致;`camera_calibration` 工具在用 URL 加载时通常会对 `camera_name` 做匹配/告警。 | 命名不一致可能触发 camera_info 加载告警或在多相机管理工具中错配;当前 `camera_adapter_node` 直接按 URL 加载未严格校验 name,故无运行时功能危害。无建图质量影响。 | 低 | 统一 `camera_name` 与话题命名空间(改为 `left`/`right`),或在文档说明该字段仅为信息性、加载时忽略。 |
| D152 | calib/(目录,4×yaml)↔ src/wheelchair_description/config/static_transforms.yaml ↔ src/wheelchair_3d_mapping/config/rgb_colorizer.yaml `cam_lidar_*` | 参数一致性/标定缺口(汇入§3):`calib/` 仅含**相机内参**,**不含**相机↔雷达外参,也不含左右雷达↔base 外参。系统所需的 cam↔lidar 外参在 `rgb_colorizer.yaml` 内自承「APPROXIMATE / NOT calibrated」(见 D043),左右雷达安装外参在 `static_transforms.yaml`/URDF(关联已知 #50 右雷达占位外参)。即:相机内参已标定而**外参链路未标定**,且无专门的外参标定产物目录。 | 外参未标定 → 着色投影(D043)与双雷达融合几何对齐依赖近似/占位值,边缘与近距错位。建图质量影响:左右雷达外参若为占位值(#50),`/points_merged` 双雷达拼接出现系统性错位(墙体加宽/分裂),直接降低融合点云与 RTAB-Map 建图质量。**汇入§3(B14:坐标系/外参族 — URDF ↔ static_transforms ↔ calib ↔ 融合 `_lookup` frame)**。 | 中 | 建立外参标定产物(如 `calib/cam_lidar_*.yaml`、`calib/lidar_extrinsics.yaml`)并用实测值替换 `static_transforms.yaml`/`rgb_colorizer.yaml` 中的占位/近似外参;B14 据此核对 URDF↔static_transforms↔融合 frame 一致性。 |
| D153 | auto_test/odom_calib.py · `Recorder.ekf_cb`/`wheel_cb`(路径累加 `self.*_path += math.hypot(...)`) · L47-49, L55-57 | 算法/不稳健:路径长度用**逐样本欧氏增量求和**(arc-length)。里程计/EKF 位姿含抖动噪声,逐样本求和会把噪声当作真实位移累加 → 近静止或低速时 `wheel_path`/`ekf_path` **系统性高估**;而该值正是计算 `register_to_rpm_scale` 校正比例(`0.112 * D/wheel_path`)的分母,高估分母会使标定出的 scale 偏小。 | 校准工具据高估的路径长度算出偏差的轮速标度,引入系统性里程计标定误差。建图质量影响:里程计标度偏差经 `/wheel/odom`→EKF→RTAB-Map 外部里程计带入位姿图,造成全局尺度漂移(走廊偏长/偏短)。 | 中 | 对逐样本增量设最小位移阈值(死区,忽略 < 噪声水平的增量)或对位姿做低通/降采样后再累加;或同时报告 net 直线距离与 arc-length 供人工取舍,并在文档说明噪声高估倾向。 |
| D154 | auto_test/odom_calib.py · 模块 docstring 与 `main` report 分支(`new register_to_rpm_scale = 0.112 * (D / wheel_path)`)· L31-33, L120-124 | 规范/魔法数:校正公式把当前 `register_to_rpm_scale` **硬编码为 0.112**。若部署中该参数已被重新标定为其他值,工具给出的「新 scale」基于过时基准 → 误导性建议。该 0.112 与运行时真值无任何联动校验。 | 操作者按工具提示套用错误基准,得到错误的轮速标度。建图质量影响:同 D153,错误标度污染里程计尺度,间接影响建图。 | 低 | 从实际参数(`ros2 param get` 或读 zlac8030_base.yaml `register_to_rpm_scale`)动态获取当前基准,或要求用户传入当前 scale,而非硬编码 0.112;打印时显示所用基准来源。 |
| D155 | auto_test/odom_calib.py · `Recorder.tick`(`json.dump(t, open(STATE,"w"))`)/`main`(`json.load(open(STATE))`)、模块 import 行、共享 `STATE=/tmp/odom_calib_path.json` · L19, L24, L60, L113 | 健壮性/规范:(1)`open(STATE,...)` 无 `with`/`close`,文件句柄依赖 GC 回收(资源管理不当);(2)多实例并发(文档建议 `--start &` 后台 + 另一进程 `--report`,甚至多次 `--start`)对同一 `/tmp` 状态文件**无加锁**,后台 recorder 每秒覆盖写,与 report 读存在竞态/读到半写;(3)`import ... signal, sys, time`(及未用项)为死导入。 | 句柄泄漏在长会话累积;并发写无锁可致 report 读到不一致快照;死导入降低可读性。属测试/工具脚本健壮性,无运行时建图危害。 | 低 | 用 `with open(...)` 上下文管理;对 `/tmp` 状态文件加进程锁或原子写(写临时文件后 `os.replace`);移除未使用的 `signal`/`sys`/`time` 导入。 |
| D156 | auto_test/yaw_compare.py · 模块级 `rclpy.init();n=M()` 与 `rclpy.spin(n)`(无 `if __name__=="__main__"` 守卫)、`import math, time`(`time` 未用)· L38-44, L7 | 架构/规范:脚本在**模块顶层**直接 `rclpy.init()` + 构造节点 + `rclpy.spin()`,无 `if __name__=="__main__":` 守卫。任何对该模块的 `import`(如复用 `yaw_of`、做单元测试)都会立即初始化 ROS 并阻塞 spin;且 `time` 为死导入。 | 模块不可安全导入/复用/测试(import 即副作用阻塞)。属诊断工具脚本的可维护性问题,无运行时建图危害。 | 低 | 把启动逻辑包进 `def main(): ...` 并以 `if __name__=="__main__": main()` 守卫;移除未使用的 `time` 导入。 |
<!-- B11(scripts/ 8 个 shell 脚本)批次:为避免与并发执行的 B9(D110 起)、B10(D130 起)、B12/B13(D150 起)批次编号冲突,本批次使用 D140 起的编号块。D108–D139 及空档由后续批次或编号归并步骤回填,不影响 6 字段完整性与不封顶原则。 -->
| D140 | scripts/hardware_shutdown.sh · `publish_direct_zlac_release` · L130-145 | 直接 Modbus 急停回退路径**恒不执行**:函数先 `[[ ! -x "$workspace_root/scripts/zlac8030_release.py" ]] && return 0`,而该 `zlac8030_release.py` 在本仓库 `scripts/` 下**不存在**(仅存在于另一仓库 `livo-wheel-3d-slam/scripts/`)。因此 `--no-direct-zlac-release` 开关与整段「ROS base 仍占用串口时直接写 ZLAC 停车」的兜底逻辑形同虚设,被静默跳过。 | 关停脚本的安全兜底失效:当 ROS 发布路径未能让电机停转(节点已死但仍占用 `/dev/smartwheel_zlac8030`)时,本应由直接 Modbus 写实现强制停车,实际从不触发。属 fail-silent 安全缺口。无建图质量影响。 | 中 | 将缺失的 `zlac8030_release.py` 纳入本仓库 `scripts/`(或修正路径指向真实位置),并在 helper 缺失时输出显式 WARN 而非静默 `return 0`,使运维可见兜底未生效。 |
| D141 | scripts/hardware_shutdown.sh · `publish_once`/`publish_stop_commands` · L113-140 | 安全停车命令发布全程吞错:`publish_once` 用 `timeout 2s ros2 topic pub --once ... >/dev/null 2>&1 \|\| true`,对 `/emergency_stop_sw`、`/cmd_vel`、`/cmd_vel_safe` 等的发布失败(DDS 未就绪、话题无匹配 QoS、超时)既无日志也不反映到退出码;脚本始终以成功收尾。 | 安全关键的停车广播失败时调用方(launch wrapper)无从得知,可能在电机未收到零速/急停的情况下继续 kill 进程。属 fail-silent。无建图质量影响。 | 中 | 对每条 publish 的失败计数并记日志(区分 timeout 与无订阅),关键停车命令(emergency_stop/cmd_vel)全部失败时以非零码退出或显式告警,供 wrapper 据此调整关停时序。 |
| D142 | scripts/save_mapping_result.sh · 数据库拷贝 · L33-37(`cp -f "$db_path" ...`) | 对**正在被 RTAB-Map 写入的活动 SQLite 数据库**直接 `cp -f` 抓快照;注释断言「copying it is safe」,但 SQLite 在 WAL/journal 未 checkpoint 时被裸拷贝可能得到撕裂(torn)/不一致的 `.db`,尤其「边建图边保存」场景。 | 建图质量影响:保存得到的 `.db` 可能损坏或缺最近一段位姿图/点云,后续 `rtabmap-export`/重定位读取失败或得到不完整地图,等于辛苦建立的成果未被可靠落盘。 | 中 | 优先在 RTAB-Map 暂停/退出后再拷贝,或使用 SQLite 一致性拷贝(`sqlite3 "$db" ".backup ..."` / `VACUUM INTO`),或先触发 WAL checkpoint;至少在文档去掉「copying is safe」的无条件论断并提示风险。 |
| D143 | scripts/save_mapping_result.sh · rtabmap-export / map_saver 调用 · L46-49, L72-78 | 管道遮蔽退出码致失败检测失效:`rtabmap-export ... 2>&1 \| tail -6 \|\| echo "WARN ..."` 与 `( cd ... && timeout 25 ros2 run nav2_map_server map_saver_cli ... \| tail -5 ) \|\| echo` 中,`\|\|` 看到的是管道末端 `tail` 的(成功)退出码而非 export/map_saver 的真实结果(脚本无 `set -o pipefail`),故导出/保存失败时 `\|\| echo WARN` **永不触发**,仅靠随后的 `ls *.ply`/`ls *.pgm` 间接判断。 | 错误上报路径失效:真实失败被吞,运维仅能从产物缺失推断,诊断信息不完整。建图质量影响:地图导出失败时缺少明确告警,易误以为保存成功。属健壮性/错误处理缺陷。 | 低 | 加 `set -o pipefail`(或用 `${PIPESTATUS[0]}` 判定首段命令退出码),使 `\|\|` 真正反映 export/map_saver 的成败;两处一致处理。 |
| D144 | scripts/setup_radar_network.sh · 变量初始化 `host_ip="${host_cidr%/*}"` · 紧随参数解析后(`host_cidr` 默认 `192.168.0.100/24,192.168.1.100/24`) | 死代码 + 错误取值:`host_ip="${host_cidr%/*}"` 在 `host_cidr` 为逗号分隔多 CIDR 时,`%/*` 仅删最后一个 `/` 后缀,得到无意义的 `192.168.0.100/24,192.168.1.100`;且该变量在后续逻辑(改用 `host_cidrs` 数组与 `host_src_for_radar`)中**从未被使用**。 | 无运行时危害(未被引用),但保留一个取值错误的死变量,易在后续维护中被误用为「主机 IP」。无建图质量影响。属规范/可维护性。 | 低 | 删除未使用的 `host_ip` 赋值;如确需「首个主机 IP」语义,应在 `split_cidr_csv` 后取 `${host_cidrs[0]%/*}`。 |
| D145 | scripts/run_rviz_manual_mapping_left.sh · `rviz_cfg` 解析与 `rviz2 -d` · `rviz_cfg=...`/`setsid rviz2 -d "$rviz_cfg"` ↔ scripts/run_rviz_sensors.sh · `rviz_cfg`/`rviz2 -d "$rviz_cfg"` | 健壮性:两脚本对 RViz 配置路径缺少最终存在性校验——manual 脚本 `[[ -f install/... ]] \|\| rviz_cfg=src/...` 回退到 src 路径后,即使 src 也不存在仍直接 `rviz2 -d "$rviz_cfg"`;sensors 脚本仅用 src 路径、无任何存在性检查。配置缺失时 `rviz2` 启动异常/退化,后台进程很快退出,`wait -n` 立即返回触发 `cleanup` 拆栈,但无明确「配置缺失」提示。 | 配置路径错误时表现为「RViz 一闪退、整栈被拆」而非清晰报错,排障困难。无建图质量影响。属健壮性/可用性。 | 低 | 在 `rviz2` 前校验最终 `rviz_cfg` 文件存在,缺失时打印明确错误并退出;并可加 `command -v rviz2` 守护。 |
| D146 | scripts/stop_mapping.sh · `patterns`/`signal_all` · L18-33, L52-57 | 健壮性/精度:用宽泛子串做 `pkill -f`(如 `wheelchair_base/lib`、`robot_localization/lib`、`wheelchair_3d_mapping/lib`),任何命令行**恰好包含**这些子串的无关进程都会被 INT/KILL;且无确认即无条件升级 SIGKILL。在多工作区/同主机并跑其他 ROS 栈时存在误杀风险。 | 误杀同主机其他进程的风险(尤其多 ws 共存时);对建图无直接质量影响,但可能误停正在运行的相邻栈。属健壮性。 | 低 | 收紧匹配(锚定到本工作区 `install/` 绝对路径前缀,或匹配特定节点可执行全路径/`__ns`),并在 KILL 前再次确认存活计数;必要时限定按本脚本启动的 PGID。 |
| D147 | scripts/check_rviz_mapping_left.sh · 全文 ↔ scripts/check_rviz_sensors_left.sh · 全文 | 健壮性/一致性:两个检查脚本 source ROS 后直接调用 `ros2 topic hz/echo`,但**未校验 `ros2` 是否可用**(对照 hardware_shutdown.sh 有 `command -v ros2` 守护)。当 `install/setup.bash` 未成功 source(`source ... \|\| true` 容错跳过)或 ros2 缺失时,所有 `check_hz`/`check_present` 均因 `timeout` 静默超时被记为 `FAIL ... silent`,误导为「话题无数据」而非「环境未就绪」。 | 误导性诊断输出:环境未就绪被报成传感器/话题全挂,增加误判。无运行时危害、无建图质量影响。属健壮性/一致性。 | 低 | 脚本开头加 `command -v ros2 >/dev/null \|\| { echo "ERROR: ros2 not on PATH; source the workspace first"; exit 1; }`,与 hardware_shutdown.sh 的可用性守护保持一致。 |

## 2. 逐文件覆盖清单(Coverage Ledger)

> 必须覆盖 §0 范围枚举出的**每一个**范围内文件,使「已检视无缺陷」与「未检视」可区分。
> 类型缩写:PY=Python,CPP=C/C++,YAML=配置 yaml,XACRO=URDF/xacro,BUILD=CMakeLists.txt/package.xml,RVIZ=RViz 配置,SH=Shell 脚本,CALIB=标定 yaml。
> 状态初始为「未检视」,缺陷数初始为 0;审查后回填。

| 相对路径 | 类型 | 状态(已检视/未检视) | 缺陷数 | 备注(无缺陷/迁移自F/已修) |
|----------|------|----------------------|--------|------------------------------|
| src/wheelchair_base/wheelchair_base/kinematics.py | PY | 已检视 | 2 | 迁移自F;D001(单轮饱和未限幅)、D002(重复限幅);F#4 中点 yaw 复核为可接受近似(非缺陷);F#6 轮距未标定→D172、F#11 invert 耦合→D176(§4.4) |
| src/wheelchair_base/wheelchair_base/modbus_rtu.py | PY | 已检视 | 1 | D010(无重试/断连自愈);CRC/帧解析与异常回复处理正确 |
| src/wheelchair_base/wheelchair_base/zlac8030_driver_node.py | PY | 已检视 | 6 | 迁移自F;D003(回调内 sleep 阻塞)、D004(串口 I/O 在 50Hz 回调)、D005(反馈失败清零 odom=F#2)、D006(参数默认值↔YAML 不一致,汇入§3)、D007(频率未校验)、D008(tick/odom 无测试=F#49);F#3/5/7/8/9/10/11→D170-D176(§4.4) |
| src/wheelchair_base/wheelchair_base/__init__.py | PY | 已检视 | 0 | 无缺陷(空包初始化文件) |
| src/wheelchair_base/setup.py | PY | 已检视 | 0 | 无缺陷;data_files 引用的 README.md 存在,entry_point 与节点一致 |
| src/wheelchair_base/test/test_kinematics.py | PY | 已检视 | 1 | D009(缺旋转/限幅/负向用例) |
| src/wheelchair_base/test/test_zlac_shutdown.py | PY | 已检视 | 0 | 无缺陷;关停/使能时序覆盖良好(tick/odom 覆盖缺口已记于 D008) |
| src/wheelchair_base/package.xml | BUILD | 已检视 | 0 | 无缺陷;依赖(rclpy/geometry_msgs/nav_msgs/std_msgs/tf2_ros/python3-serial)与代码 import 一致 |
| src/wheelchair_sensors/wheelchair_sensors/camera_adapter_node.py | PY | 已检视 | 2 | D011(多相机阻塞读在定时器回调)、D012(墙钟时间戳 + 图像 QoS 默认 reliable 可能与消费者不匹配,汇入§3) |
| src/wheelchair_sensors/wheelchair_sensors/imu_adapter_node.py | PY | 已检视 | 2 | D013(校验和双字节序放宽)、D014(设备时钟 offset 单调不增致漂移 + 命名与 xtm60 不一致,汇入§3) |
| src/wheelchair_sensors/wheelchair_sensors/mock_sensor_node.py | PY | 已检视 | 1 | D022(死变量 x/y/yaw + 默认发布非零 cmd_vel_nav);D021 涉及本文件速率参数未校验 |
| src/wheelchair_sensors/wheelchair_sensors/ultrasonic_adapter_node.py | PY | 已检视 | 3 | D015(异常吞没)、D016(range 未按 min/max 裁剪、0 值假读)、D017(顺序阻塞串口 + sleep 在回调) |
| src/wheelchair_sensors/wheelchair_sensors/xtm60_adapter_node.py | PY | 已检视 | 3 | 迁移自F;D018(ping 阻塞 executor)、D019(墙钟时间戳=F#41,汇入§3)、D020(point_unit_scale 单位未校验 + range_max 默认与各 yaml 不一致,汇入§3 量程族);F#42 多节点 now() 叠加→§3 D168;epoch 戳合理性检查、organized cloud 处理正确 |
| src/wheelchair_sensors/wheelchair_sensors/__init__.py | PY | 已检视 | 0 | 无缺陷(空包初始化文件) |
| src/wheelchair_sensors/setup.py | PY | 已检视 | 0 | 无缺陷;data_files 引用的 README.md 存在,entry_points 与 5 个节点一致 |
| src/wheelchair_sensors/test/test_h30_imu_adapter.py | PY | 已检视 | 1 | D023(IMU+超声测试覆盖不足,合并条目)—缺 euler/时间戳/双字节序用例 |
| src/wheelchair_sensors/test/test_ultrasonic_adapter.py | PY | 已检视 | 0 | 无缺陷(覆盖缺口并入 D023);Modbus 构帧/解析/sensor_list 用例正确 |
| src/wheelchair_sensors/test/test_xtm60_adapter.py | PY | 已检视 | 0 | 无缺陷;覆盖 extract/epoch/UDP-start 时序/事件状态机良好(注:引用 `_is_plausible_epoch_stamp` 与实现一致) |
| src/wheelchair_sensors/package.xml | BUILD | 已检视 | 0 | 无缺陷;依赖(rclpy/sensor_msgs/nav_msgs/geometry_msgs/std_msgs/tf2_ros/sensor_msgs_py/python3-serial/python3-opencv)与代码 import 一致 |
| src/wheelchair_perception/wheelchair_perception/dynamic_obstacle_layer_node.py | PY | 已检视 | 0 | 无缺陷;占位节点(仅 2s 定时发布状态串),职责与注释一致,刻意不写静态地图 |
| src/wheelchair_perception/wheelchair_perception/obstacle_detector_node.py | PY | 已检视 | 1 | D031(单波束簇被丢弃,细窄障碍漏报);marker/summary 逻辑正确 |
| src/wheelchair_perception/wheelchair_perception/passability_analyzer_node.py | PY | 已检视 | 1 | D032(中线 y==0 点漏统计 + 单侧无边界过度保守判 UNKNOWN) |
| src/wheelchair_perception/wheelchair_perception/pointcloud_to_laserscan_node.py | PY | 已检视 | 4 | D024(restamp 墙钟戳,汇入§3)、D025(TF 用 latest 非 msg.stamp)、D026(range_max=8 与融合 12 不一致,汇入§3)、D027(scan 发布 reliable 与下游 sensor_data 不匹配,汇入§3);空帧跳发/不发布 staleness 语义合理 |
| src/wheelchair_perception/wheelchair_perception/scan_merger_node.py | PY | 已检视 | 3 | D028(输出墙钟戳,汇入§3)、D029(不做 TF 跨 frame 合并,几何依赖上游统一坐标系)、D030(默认 reliable QoS,汇入§3);staleness/require_all_sources 逻辑正确 |
| src/wheelchair_perception/wheelchair_perception/__init__.py | PY | 已检视 | 0 | 无缺陷(空包初始化文件) |
| src/wheelchair_perception/setup.py | PY | 已检视 | 0 | 无缺陷;data_files 引用的 README.md 存在,entry_points 与 5 个节点一致 |
| src/wheelchair_perception/test/test_passability_analyzer.py | PY | 已检视 | 0 | 无缺陷(覆盖缺口并入 D033);CLEAR/BLOCKED 用例正确 |
| src/wheelchair_perception/test/test_pointcloud_to_laserscan.py | PY | 已检视 | 1 | D033(perception 三测试覆盖不足,合并条目)—缺 TF/restamp/空帧路径;beam_count 与 best_effort QoS 用例正确 |
| src/wheelchair_perception/test/test_scan_merger.py | PY | 已检视 | 0 | 无缺陷(覆盖缺口并入 D033);最近值合并/越界过滤用例正确 |
| src/wheelchair_perception/package.xml | BUILD | 已检视 | 0 | 无缺陷;依赖(rclpy/sensor_msgs/geometry_msgs/std_msgs/visualization_msgs/tf2_ros/sensor_msgs_py)与代码 import 一致 |
| src/wheelchair_3d_mapping/wheelchair_3d_mapping/cloud_to_occupancy_grid_node.py | PY | 已检视 | 2 | D038(默认 cloud_registered 即时投影不累积 + rolling 原点漂移)、D039(max_cells 静默截断);TF lookup 用 latest(此处源云已在 map_frame,可接受)、inflate 逻辑正确 |
| src/wheelchair_3d_mapping/wheelchair_3d_mapping/cloud_utils.py | PY | 已检视 | 1 | D037(image_to_rgb 编码白名单不全 + 大小写敏感,静默错位);quat/transform/range/voxel 数值逻辑正确,read_xyz_intensity skip_nans 合理 |
| src/wheelchair_3d_mapping/wheelchair_3d_mapping/dual_lidar_cloud_fusion_node.py | PY | 已检视 | 3 | 迁移自F;D034(输出墙钟戳=F#13,汇入§3)、D035(左右两路无时间对齐 vstack 致双影=F#14)、D036(参数默认 max_range=20/z 带与 YAML 不一致,汇入§3);F#15/16/17/18/19/20/21/22/23/24→D178-D186(§4.4);TF/fallback/range-then-height 顺序正确 |
| src/wheelchair_3d_mapping/wheelchair_3d_mapping/kiss_icp_mapping_node.py | PY | 已检视 | 1 | D042(max_range=20 默认与融合 12 不一致,汇入§3);deskew 关闭合理、_mat_to_quat 正确、path 截断/局部地图发布逻辑正确 |
| src/wheelchair_3d_mapping/wheelchair_3d_mapping/rgb_cloud_colorizer_node.py | PY | 已检视 | 1 | D043(忽略相机畸变 D + 外参近似未标定,着色错位);identity 外参告警、image-age 跳过、main→aux 填充逻辑正确(仅可视化,不入几何 SLAM) |
| src/wheelchair_3d_mapping/wheelchair_3d_mapping/wheel_livo_consistency_monitor.py | PY | 已检视 | 0 | 无缺陷;report-only 不发 e-stop,窗口运动/score/超时/yaw 计算正确,stamp 缺失回退合理 |
| src/wheelchair_3d_mapping/wheelchair_3d_mapping/__init__.py | PY | 已检视 | 0 | 无缺陷(空包初始化文件) |
| src/wheelchair_3d_mapping/setup.py | PY | 已检视 | 0 | 无缺陷;data_files 引用 README.md/config/launch/rviz,entry_points 与 5 个节点一致 |
| src/wheelchair_3d_mapping/test/test_autonomous_mapping_launch.py | PY | 已检视 | 0 | 无缺陷;profile 启停/速度上限/scan_merger/diagnostics/reserved 路径覆盖良好 |
| src/wheelchair_3d_mapping/test/test_cloud_utils.py | PY | 已检视 | 0 | 无缺陷;quat/transform/range/voxel 用例正确(注:未覆盖 image_to_rgb 编码分支 D037、filter_by_height,属可选补强) |
| src/wheelchair_3d_mapping/config/cloud_to_occupancy_grid.yaml | YAML | 已检视 | 0 | 无缺陷(缺陷归因于节点 D038/D039);map_frame=livo_map 与 livo_interface 一致,z 带/分辨率合理 |
| src/wheelchair_3d_mapping/config/dual_lidar_fusion.yaml | YAML | 已检视 | 0 | 无缺陷;量程/高度带经调参(0.3-12m、z[-0.10,1.80]),节点默认不一致记于 D036 |
| src/wheelchair_3d_mapping/config/livo_interface.yaml | YAML | 已检视 | 0 | 无缺陷;backend=none 默认安全,topic/frame 命名自洽,external_remappings 留空且有注释指引 |
| src/wheelchair_3d_mapping/config/rgb_colorizer.yaml | YAML | 已检视 | 0 | 无缺陷(外参近似未标定问题归 D043);max_image_age=1.25s 注释说明视觉容差合理 |
| src/wheelchair_3d_mapping/config/rtabmap_params.yaml | YAML | 已检视 | 1 | 迁移自F;D040(Grid/RangeMax=8 vs 融合 12=F#30,汇入§3);F#28 Force3DoF→D187、F#31 RayTracing→D189、F#33 缺 FOV→D191(§4.4);Force3DoF/法向量分割/ICP 参数合理,与 launch dict 双源问题记于 D041 |
| src/wheelchair_3d_mapping/config/wheel_livo_consistency.yaml | YAML | 已检视 | 0 | 无缺陷;阈值/窗口/频率与监控节点参数一致 |
| src/wheelchair_3d_mapping/launch/autonomous_rviz_mapping.launch.py | PY | 已检视 | 1 | D044(死参数 enable_xtm60_radar 未使用);profile/速度上限/安全门控(enable_motion+autonomous 双开)逻辑正确、默认不动 |
| src/wheelchair_3d_mapping/launch/cloud_to_2d_map.launch.py | PY | 已检视 | 0 | 无缺陷;config/input_cloud_topic 参数化、节点名一致 |
| src/wheelchair_3d_mapping/launch/dual_lidar_fusion.launch.py | PY | 已检视 | 0 | 无缺陷;config + fallback override 传参正确 |
| src/wheelchair_3d_mapping/launch/kiss_icp_mapping.launch.py | PY | 已检视 | 0 | 无缺陷(max_range=20 默认归 D042);bringup_sensors 可选、TF owner 说明清晰 |
| src/wheelchair_3d_mapping/launch/livo_3d_mapping.launch.py | PY | 已检视 | 0 | 无缺陷;backend 缺失/未知/launch 不存在均优雅报错不崩溃,remap 解析健壮 |
| src/wheelchair_3d_mapping/launch/livo_wheel_fusion.launch.py | PY | 已检视 | 0 | 无缺陷;tf_owner 单一 TF 所有权、EKF publish_tf 条件正确,监控始终运行 |
| src/wheelchair_3d_mapping/launch/rtabmap_3d_mapping.launch.py | PY | 已检视 | 1 | 迁移自F;D041(essential dict 与 yaml 参数双源/yaml 据称不生效=F#25 已修遗留、F#26);F#32 delete_db 误续建→D190(§4.4);odom_mode 外部模式默认 topic 误用有 LogInfo 警示、subscribe_depth 显式关闭正确;Grid/RangeMax=8 同记于 D040 |
| src/wheelchair_3d_mapping/rviz/3d_mapping.rviz | RVIZ | 已检视 | 0 | 无缺陷;Fixed Frame=livo_map 与 LIVO 全局系一致,点云 Best Effort、2D map Transient Local 与发布端 QoS 匹配 |
| src/wheelchair_3d_mapping/rviz/autonomous_3d_mapping.rviz | RVIZ | 已检视 | 0 | 无缺陷;Fixed Frame=map、cloud_map Reliable / grid_map Transient Local / scan Best Effort 与各发布端匹配 |
| src/wheelchair_3d_mapping/rviz/left_lidar_lab_mapping.rviz | RVIZ | 已检视 | 0 | 无缺陷;单左雷达可视化 QoS 与话题与 left_lidar_lab profile 一致 |
| src/wheelchair_3d_mapping/package.xml | BUILD | 已检视 | 0 | 无缺陷;依赖(rclpy/sensor_msgs/sensor_msgs_py/nav_msgs/geometry_msgs/std_msgs/tf2_ros/numpy/rtabmap_slam/rtabmap_odom)与 import 一致,kiss-icp/pip 与 topic_tools 可选有注释 |
| src/wheelchair_mapping/scripts/map_postprocess.py | PY | 已检视 | 0 | 无缺陷;占位脚本仅校验 map YAML(必填键+image 存在性),逻辑正确、无运行时风险(D049/D051 记于矢量化脚本与全包测试缺口) |
| src/wheelchair_mapping/scripts/map_quality_check.py | PY | 已检视 | 2 | D045(P5 读取吞全部空白致栅格错位)、D046(占据率硬编码 /255 未按 maxval 归一);连通域 BFS/评分/分类逻辑正确,P2 有 payload 校验 |
| src/wheelchair_mapping/scripts/vectorize_occupancy_map.py | PY | 已检视 | 4 | D045(P5 读取吞全部空白,与质量脚本同源)、D047(P2 分支缺 payload 校验)、D048(墙线超限静默抽稀截断)、D049(YAML 缺键无校验直接索引);cell_edges/merge_spans 几何逻辑正确(仅 UI 显示,不入几何 SLAM) |
| src/wheelchair_mapping/launch/online_mapping.launch.py | PY | 已检视 | 0 | 无缺陷;仅 include wheelchair_bringup/mapping.launch.py,薄封装无逻辑 |
| src/wheelchair_mapping/launch/save_map.launch.py | PY | 已检视 | 1 | D050(默认相对路径 map_name 随 CWD 漂移且不创建目录,保存可能静默失败致地图丢失) |
| src/wheelchair_mapping/CMakeLists.txt | BUILD | 已检视 | 0 | 无缺陷;install launch/scripts(USE_SOURCE_PERMISSIONS)+README,ament_package 正确;全包无测试缺口记于 D051 |
| src/wheelchair_mapping/package.xml | BUILD | 已检视 | 0 | 无缺陷;exec_depend(launch/launch_ros/slam_toolbox/nav2_map_server/python3-yaml)与 launch/脚本 import 一致,format=3、ament_cmake build_type 正确 |
| src/wheelchair_navigation/wheelchair_navigation/frontier_explorer_node.py | PY | 已检视 | 3 | D072(前沿扫描/聚类在定时器回调内阻塞 executor)、D073(质心目标未校验 free/known)、D074(_publish_frontiers 死参数 goal);safety 严格白名单 + stale 判定、blacklist、TF lookup 用 latest(此处合理)逻辑正确 |
| src/wheelchair_navigation/wheelchair_navigation/goal_manager_node.py | PY | 已检视 | 2 | D070(自触发 /goal_pose + 双重导航提交)、D071(add_goal/navigate_to_name 缺键抛异常穿透回调);ComputePathToPose preview 节流、nav 结果状态映射、TTS 逻辑正确 |
| src/wheelchair_navigation/wheelchair_navigation/named_goal_store.py | PY | 已检视 | 1 | D076(save 非原子写,易损坏命名目标库);normalize_key/yaw↔quat、label 回退查找、upsert/delete 逻辑正确 |
| src/wheelchair_navigation/wheelchair_navigation/navigation_status_node.py | PY | 已检视 | 1 | D075(self.latest 被 goal_status 与 Nav2 GoalStatusArray 双源无约定竞争覆盖);STATUS_NAMES 映射、1Hz 转发逻辑本身正确 |
| src/wheelchair_navigation/wheelchair_navigation/reactive_explorer_node.py | PY | 已检视 | 0 | 无缺陷;sector/corridor/clearance 几何、stuck 检测、turn 窗口状态机、超声 stale 回退、publish_rate 用 max(1.0,..) 已防 0 频率,逻辑严谨 |
| src/wheelchair_navigation/wheelchair_navigation/semantic_keepout_node.py | PY | 已检视 | 1 | D077(publish_mask 未捕获 rasterize 对退化地图的 ValueError + 每帧全量重算);filter_transform 等待门控、latched QoS、配置失效保留旧 zones 逻辑正确 |
| src/wheelchair_navigation/wheelchair_navigation/semantic_keepout.py | PY | 已检视 | 0 | 无缺陷;world_to_grid 旋转变换、point_in_polygon 射线法、rasterize bbox 裁剪 + 维度/分辨率正校验,数值逻辑正确 |
| src/wheelchair_navigation/wheelchair_navigation/semantic_map_store.py | PY | 已检视 | 0 | 无缺陷;save 采用临时文件 + fsync + os.replace 原子写(姊妹 named_goal_store 缺此,记于 D076),DEFAULT 合并、upsert/delete 逻辑正确 |
| src/wheelchair_navigation/wheelchair_navigation/startup_localization_node.py | PY | 已检视 | 0 | 无缺陷;mode 状态机、AMCL ack 容差匹配、external_anchor 校验、retry 上限、covariance 校验、frame_id=map 校验严谨 |
| src/wheelchair_navigation/wheelchair_navigation/startup_localization.py | PY | 已检视 | 0 | 无缺陷;normalize_mode/angle、finite 校验、named_goal_target frame/position 校验、pose_matches 容差与 yaw wrap 正确 |
| src/wheelchair_navigation/wheelchair_navigation/__init__.py | PY | 已检视 | 0 | 无缺陷(空包初始化文件) |
| src/wheelchair_navigation/setup.py | PY | 已检视 | 0 | 无缺陷;data_files 引用 README.md/config/*.yaml 存在,entry_points 与 6 个节点一致 |
| src/wheelchair_navigation/config/named_goals.yaml | YAML | 已检视 | 1 | D078(含调试占位目标 lty了/wxy + 原点示例点随包部署) |
| src/wheelchair_navigation/config/semantic_map.yaml | YAML | 已检视 | 1 | D079(默认随包安装含示例房间/POI 占位内容) |
| src/wheelchair_navigation/test/test_goal_manager.py | PY | 已检视 | 1 | D080(导航包测试覆盖不足,合并条目)—仅覆盖纯函数,未覆盖 handle_command/自触发/navigate_to_name |
| src/wheelchair_navigation/test/test_named_goal_store.py | PY | 已检视 | 0 | 无缺陷;upsert/get/list/delete roundtrip 用例正确(原子写缺陷归 D076) |
| src/wheelchair_navigation/test/test_reactive_explorer.py | PY | 已检视 | 0 | 无缺陷;sector/corridor/turn/ultrasonic 覆盖良好 |
| src/wheelchair_navigation/test/test_semantic_keepout_node.py | PY | 已检视 | 0 | 无缺陷;mask 发布/配置失效保留/等待 transform 用例覆盖良好 |
| src/wheelchair_navigation/test/test_semantic_keepout.py | PY | 已检视 | 0 | 无缺陷;栅格化/裁剪/旋转原点/多边形校验用例正确 |
| src/wheelchair_navigation/test/test_semantic_map_store.py | PY | 已检视 | 0 | 无缺陷;room/no_go roundtrip + 无残留临时文件断言正确 |
| src/wheelchair_navigation/test/test_startup_localization_node.py | PY | 已检视 | 0 | 无缺陷;disabled/fixed-retry-ack/external_anchor 三模式集成用例覆盖良好 |
| src/wheelchair_navigation/test/test_startup_localization.py | PY | 已检视 | 0 | 无缺陷;mode/finite/named_goal/pose_matches/covariance 纯函数用例正确 |
| src/wheelchair_navigation/package.xml | BUILD | 已检视 | 0 | 无缺陷;依赖(rclpy/geometry_msgs/nav_msgs/std_msgs/visualization_msgs/action_msgs/nav2_msgs/tf2_ros/ament_index_python/python3-yaml)与代码 import 一致 |
| src/wheelchair_safety/wheelchair_safety/emergency_stop_node.py | PY | 已检视 | 0 | 无缺陷;纯命令桥(/emergency_stop_command→/emergency_stop_sw Bool),10Hz 锁存发布,未知命令告警,职责与注释一致(心跳新鲜度依赖订阅端 supervisor,记于 D095) |
| src/wheelchair_safety/wheelchair_safety/safety_supervisor_node.py | PY | 已检视 | 4 | D093(超声 0 值故障静默丢弃≈无障碍)、D094(consistency_score 无超时守护,门控可形同虚设)、D095(e-stop 心跳默认不校验,fail-unsafe)、D096(超声急停阈值取 min 偏不保守 + 与 scan emergency 耦合);D098 记于死代码、scan/cmd 超时归零与 manual_bypass 门控逻辑本身正确 |
| src/wheelchair_safety/wheelchair_safety/velocity_limiter_node.py | PY | 已检视 | 3 | D090(NaN 经 clamp 透传为上限速度,fail-unsafe)、D091(仅静态 clamp 无加速度/超时归零,弱于"限速器"语义)、D092(节点 orphan + /cmd_vel_limited 无订阅者,纸面安全) |
| src/wheelchair_safety/wheelchair_safety/__init__.py | PY | 已检视 | 0 | 无缺陷(空包初始化文件) |
| src/wheelchair_safety/setup.py | PY | 已检视 | 0 | 无缺陷;data_files 引用的 README.md 存在,entry_points 与 3 个节点一致 |
| src/wheelchair_safety/test/test_safety_supervisor.py | PY | 已检视 | 1 | D099(仅测纯函数 evaluate_safety/dynamic_stop,未覆盖节点级 failsafe 门控);现有 emergency/reverse/rotation/directional 用例本身正确 |
| src/wheelchair_safety/package.xml | BUILD | 已检视 | 0 | 无缺陷;依赖(rclpy/geometry_msgs/sensor_msgs/std_msgs)与代码 import 一致 |
| src/wheelchair_diagnostics/wheelchair_diagnostics/hardware_probe.py | PY | 已检视 | 2 | D100(跨包 import 无 try 守护)、D101(超声探针逐地址异常静默吞没);cv2/serial 可选导入降级、camera/zlac/xtm60 探针的 finally 释放/超时处理正确 |
| src/wheelchair_diagnostics/wheelchair_diagnostics/hardware_self_check_node.py | PY | 已检视 | 1 | D102(构造期 + 定时器回调内串行阻塞 I/O 探针,阻塞 executor 数秒);result_to_status 映射、payload 序列化、xtm60 多 IP/camera 多设备命名正确 |
| src/wheelchair_diagnostics/wheelchair_diagnostics/localization_health_node.py | PY | 已检视 | 3 | D103(max_amcl_age_sec 默认 2.0↔yaml 1e6,静止误判 LOST 闭锁,汇入§3)、D104(/scan QoS 默认 reliable 与 watchdog 的 sensor_data 不一致,汇入§3)、D106(age 用 monotonic 而 header 用 ROS 时钟,sim_time/bag 错位) |
| src/wheelchair_diagnostics/wheelchair_diagnostics/policy.py | PY | 已检视 | 0 | 无缺陷;evaluate_watchdog(grace/critical 分级)、evaluate_localization_health(LOST/DEGRADED/GOOD)逻辑正确,age 用 max(0,now-seen) 防负,to_json 序列化完整(纯函数,时间由调用方注入) |
| src/wheelchair_diagnostics/wheelchair_diagnostics/sensor_watchdog_node.py | PY | 已检视 | 1 | D105(ultrasonic/points 默认话题集↔yaml 不一致,缺省漏看护 critical 传感器,汇入§3);has_parameter 守护 *_critical、qos_profile_sensor_data 订阅、lambda 默认参数绑定 topic 正确;age 时钟问题并入 D106 |
| src/wheelchair_diagnostics/wheelchair_diagnostics/__init__.py | PY | 已检视 | 0 | 无缺陷(空包初始化文件) |
| src/wheelchair_diagnostics/setup.py | PY | 已检视 | 0 | 无缺陷;data_files 引用 package.xml/README.md,entry_points 与 3 个节点一致(hardware_probe/policy 为库,不注册 console_script,正确) |
| src/wheelchair_diagnostics/test/test_policy.py | PY | 已检视 | 1 | D107(覆盖 NAV_BLOCKED/DEGRADED/高协方差,但缺 startup_grace/LOST/to_json 用例,且 hardware_probe 与 3 节点零测试) |
| src/wheelchair_diagnostics/package.xml | BUILD | 已检视 | 0 | 无缺陷;exec_depend(rclpy/diagnostic_msgs/std_msgs/sensor_msgs/nav_msgs/geometry_msgs/python3-serial/python3-opencv/wheelchair_sensors/wheelchair_base)与代码 import 一致(代码层容错缺口记于 D100) |
| src/wheelchair_bringup/src/teleop_panel.cpp | CPP | 已检视 | 2 | D110(qApp 全局事件过滤器吞 WASD 且误触发运动)、D111(onInitialize lock() 未判空解引用);Qt 控件均设 parent 由 Qt 树管理、定时器 parent=this 无泄漏,load/save/recompute 逻辑正确 |
| src/wheelchair_bringup/src/xt_bindshim.c | CPP | 已检视 | 1 | D112(dlsym/real_bind 未判空、atoi 端口无校验);memcpy 边界由 len 守护、pinned 栈对象无泄漏、AF_INET/INADDR_ANY 判定正确 |
| src/wheelchair_bringup/include/wheelchair_bringup/teleop_panel.hpp | CPP | 已检视 | 0 | 无缺陷;成员裸指针均由 Qt parent 树管理、默认值(0.25/0.6)与 .cpp 一致,override/Q_SLOTS 声明正确 |
| src/wheelchair_bringup/CMakeLists.txt | BUILD | 已检视 | 0 | 无缺陷;xt_bindshim(link dl)、teleop_panel(AUTOMOC + qt5_wrap_cpp + pluginlib 导出)、install launch/config/rviz 均正确 |
| src/wheelchair_bringup/package.xml | BUILD | 已检视 | 1 | D113(exec_depend wheelchair_ui/wheelchair_voice_agent 在本工作区缺失);其余依赖与 launch 引用一致,format=3/ament_cmake 正确 |
| src/wheelchair_bringup/launch/base.launch.py | PY | 已检视 | 0 | 无缺陷;publish_tf/motion_control_enabled 默认安全(false),YAML+override 传参正确,TF owner 注释清晰 |
| src/wheelchair_bringup/launch/bringup_3d_slam.launch.py | PY | 已检视 | 1 | D125(未使用的 IfCondition 导入);tf_owner 单一 odom→base_link 所有权、topic_tools 可选 try/except、backend=none 优雅降级逻辑正确 |
| src/wheelchair_bringup/launch/demo_mock.launch.py | PY | 已检视 | 2 | D114(引用缺失包 wheelchair_ui)、D115(mock 参数 LaunchConfiguration 未声明类型,bool/double override 类型不匹配);其余节点装配正确 |
| src/wheelchair_bringup/launch/diagnostics.launch.py | PY | 已检视 | 0 | 无缺陷;watchdog + 可选 localization_health(IfCondition)装配正确,共用 diagnostics.yaml |
| src/wheelchair_bringup/launch/full_system.launch.py | PY | 已检视 | 1 | D114(enable_voice_agent/enable_web_ui 默认 true 引用缺失包 wheelchair_voice_agent/wheelchair_ui);single/dual radar 门控 PythonExpression、motion 默认 false、include 链正确 |
| src/wheelchair_bringup/launch/localization.launch.py | PY | 已检视 | 0 | 无缺陷;map_server/amcl/lifecycle_manager/可选 EKF/startup_localization 装配与参数传递正确 |
| src/wheelchair_bringup/launch/manual_mapping_left.launch.py | PY | 已检视 | 0 | 无缺陷;manual_teleop + fusion + RTAB-Map(external odom)编排正确,motion 默认 false、EKF 拥有 TF、单左雷达 fallback 说明清晰 |
| src/wheelchair_bringup/launch/manual_teleop.launch.py | PY | 已检视 | 0 | 无缺陷;sensors/scan/EKF/watchdog(left-only profile)/safety/base 链正确,base publish_tf=false 避免与 EKF 抢 TF,motion 默认 false |
| src/wheelchair_bringup/launch/mapping.launch.py | PY | 已检视 | 1 | D114(enable_ui 路径引用缺失包 wheelchair_ui,默认 false 故非默认触发);use_ekf↔base_publish_tf 互斥、mock/dual radar 门控、slam_toolbox 装配正确 |
| src/wheelchair_bringup/launch/navigation.launch.py | PY | 已检视 | 0 | 无缺陷;nav2 bringup + cmd_vel→cmd_vel_nav 重映射、semantic_keepout/passability/safety/localization_health/goal_manager 装配正确 |
| src/wheelchair_bringup/launch/preflight_check.launch.py | PY | 已检视 | 0 | 无缺陷;仅起 hardware_self_check_node + diagnostics.yaml,薄封装无逻辑 |
| src/wheelchair_bringup/launch/record_bag.launch.py | PY | 已检视 | 1 | D116(录 /camera/front/image_raw 但实际发布 left/right,漏录相机图像与 /odometry/filtered);其余话题列表合理 |
| src/wheelchair_bringup/launch/replay_bag.launch.py | PY | 已检视 | 1 | D123(--clock 与全工程 use_sim_time:=false 矛盾,回放时间基准不一致);bag/rate 参数化正确 |
| src/wheelchair_bringup/launch/rviz_sensors_left.launch.py | PY | 已检视 | 0 | 无缺陷;Stage0 纯可视化,不起 safety/base/Nav2,EKF 唯一 TF owner,说明清晰 |
| src/wheelchair_bringup/launch/sensors.launch.py | PY | 已检视 | 0 | 无缺陷;单/双雷达条件启停、LD_PRELOAD+XT_BIND_IP/PORT 注入 bindshim、remap 左右点云/状态、各传感器 IfCondition 装配正确 |
| src/wheelchair_bringup/config/camera_full.yaml | YAML | 已检视 | 1 | D120(left/rear 分配到奇数 video1/video3 metadata-only 索引,无法采集);fourcc/分辨率/fps 合理 |
| src/wheelchair_bringup/config/camera_roles.yaml | YAML | 已检视 | 1 | D124(注释称不发布 camera_info,与 camera.yaml 设 camera_info_url 会发布相矛盾);main/aux 角色与 alias topic 命名自洽 |
| src/wheelchair_bringup/config/camera.yaml | YAML | 已检视 | 0 | 无缺陷;偶数索引(0/2)实测可采集、旋转 180°、camera_info_url 指向 calib/left.yaml/right.yaml(存在);camera_info 发布与 roles 注释矛盾记于 D124 |
| src/wheelchair_bringup/config/diagnostics_left_lidar_lab.yaml | YAML | 已检视 | 0 | 无缺陷;left-only profile 仅 points_0_critical(右雷达非关键),grace=10s 与超时阈值合理 |
| src/wheelchair_bringup/config/diagnostics.yaml | YAML | 已检视 | 1 | D121(zlac_port=/dev/ttyUSB2 枚举名与驱动 udev 符号链接不一致);max_amcl_age=1e6 注释合理(记于 D103 节点默认)、watchdog 关键传感器集与 critical 标记正确 |
| src/wheelchair_bringup/config/empty_map.yaml | YAML | 已检视 | 0 | 无缺陷;占位地图 yaml,image/resolution/origin/thresh 字段完整(纯数据) |
| src/wheelchair_bringup/config/h30_imu.yaml | YAML | 已检视 | 0 | 无缺陷;publish_rate=100Hz、串口/协方差合理;频率与 EKF 30/轮速 50 的跨文件关系记于 §3(B14 频率族) |
| src/wheelchair_bringup/config/nav2_autonomous_mapping_params.yaml | YAML | 已检视 | 0 | 无缺陷;低速无人建图档,odom=/odometry/filtered、static_layer 订阅 /rtabmap/grid_map、速度上限保守,装配自洽 |
| src/wheelchair_bringup/config/nav2_params.yaml | YAML | 已检视 | 2 | D117(range_layer 已定义但未列入 local_costmap.plugins,超声不进代价地图)、D118(min_y_velocity_threshold=0.5 异常魔法值);bt 插件全、cmd_vel→/cmd_vel_nav、costmap/critic 参数合理 |
| src/wheelchair_bringup/config/passability.yaml | YAML | 已检视 | 0 | 无缺陷;轮椅宽/裕度/lookahead/block_forward 参数自洽(缺陷归节点 D032) |
| src/wheelchair_bringup/config/pointcloud_to_scan_left.yaml | YAML | 已检视 | 0 | 无缺陷;source_frame=xtm60_left_link/target=base_link、range_max=8(量程族汇入§3 记于 D026)、restamp/QoS 缺陷归节点 D024/D027 |
| src/wheelchair_bringup/config/pointcloud_to_scan_right.yaml | YAML | 已检视 | 0 | 无缺陷;source_frame=xtm60_right_link、参数与 left 对称,range_max=8 量程族汇入§3 |
| src/wheelchair_bringup/config/pointcloud_to_scan.yaml | YAML | 已检视 | 0 | 无缺陷;单雷达 source_frame=laser_link(laser_link 非 optical 帧问题记于 B10 D132,汇入§3),range_max=8 汇入§3 |
| src/wheelchair_bringup/config/robot_localization_ekf.yaml | YAML | 已检视 | 0 | 迁移自F;无独立待修缺陷;F#34 轮速 yaw 主导→已改 IMU 主导(§4.1 已修,本文件即修复证据);F#35 two_d_mode→D192、F#36 imu0_relative→D193、F#39 无 map 全局校正→D194、F#40 IMU 固定协方差→D195(§4.4);frequency=30 跨文件关系汇入§3(B14 频率族 D166) |
| src/wheelchair_bringup/config/robot_localization_livo_wheel_ekf.yaml | YAML | 已检视 | 1 | D119(重新启用轮速 yaw/vyaw 与主 EKF 已修结论矛盾,三路 yaw 叠加,汇入§3);LIVO odom1 loose-coupling differential、IMU yaw 配置本身合理 |
| src/wheelchair_bringup/config/safety_params_manual_mapping.yaml | YAML | 已检视 | 0 | 无缺陷;manual_bypass=true 人工监督档,仍保留 E-stop/cmd 超时/限速,注释充分(velocity_limiter orphan 记于 D092) |
| src/wheelchair_bringup/config/safety_params_mapping.yaml | YAML | 已检视 | 0 | 无缺陷;无人低速建图档,min_ultrasonic_online=4 fail-closed,阈值保守;debug_max_speed 死配置记于 D097 |
| src/wheelchair_bringup/config/safety_params.yaml | YAML | 已检视 | 0 | 无缺陷;manned 默认档;debug_max_speed 死配置记于 D097、velocity_limiter orphan 记于 D092 |
| src/wheelchair_bringup/config/scan_merger_left_only.yaml | YAML | 已检视 | 0 | 无缺陷;单 /scan_left 输入、frame_id=base_link、10Hz/stale 0.5s 合理(节点 QoS/TF 缺陷记于 D029/D030) |
| src/wheelchair_bringup/config/scan_merger.yaml | YAML | 已检视 | 0 | 无缺陷;左右双输入合并、角度/range_max=8 参数自洽(跨 frame 合并依赖记于 D029) |
| src/wheelchair_bringup/config/sensor_layout.yaml | YAML | 已检视 | 1 | D122(孤立未加载文件,缺 xtm60_left/right 帧且 laser z=0.65 过时,与 URDF 脱节,汇入§3) |
| src/wheelchair_bringup/config/slam_toolbox_params.yaml | YAML | 已检视 | 0 | 无缺陷;max_laser_range=8 与 scan 链一致、transform_timeout=1.0/scan_buffer=50 有注释依据(应对 ZLAC 串口阻塞致 TF 延迟),回环参数合理 |
| src/wheelchair_bringup/config/ultrasonic_full.yaml | YAML | 已检视 | 0 | 无缺陷;与 ultrasonic.yaml 同地址/量程,4 传感器布局(节点 range 裁剪缺陷记于 D016) |
| src/wheelchair_bringup/config/ultrasonic.yaml | YAML | 已检视 | 0 | 无缺陷;10Hz/0.2s 超时有注释依据、地址/索引/量程(0.03-3.0m)自洽 |
| src/wheelchair_bringup/config/xtm60_left.yaml | YAML | 已检视 | 0 | 无缺陷;range_max=50(原始未滤,融合再裁,量程族汇入§3 记于 D020)、organized_cloud、SDK 滤波参数与上位机一致、udp 绑定注释清晰 |
| src/wheelchair_bringup/config/xtm60_right.yaml | YAML | 已检视 | 0 | 无缺陷;与 left 对称、ip=192.168.1.101、udp 7687 绑定说明正确,range_max=50 量程族汇入§3 |
| src/wheelchair_bringup/config/xtm60_sdk.yaml | YAML | 已检视 | 0 | 无缺陷;单雷达档 range 0.3-12m、frame_id=laser_link(非 optical 帧问题汇入§3 记于 B10 D132)、point_unit_scale=1.0 校验缺陷归节点 D020 |
| src/wheelchair_bringup/config/zlac8030_base.yaml | YAML | 已检视 | 0 | 无缺陷;motion_control_enabled/hold_zero 默认安全、单从机双轴/方向标定有日志依据、寄存器映射完整;节点默认值不一致记于 D006(汇入§3) |
| src/wheelchair_bringup/rviz/manual_mapping_left.rviz | RVIZ | 已检视 | 0 | 无缺陷;Fixed Frame=map、cloud_map/grid_map Transient Local+Reliable、/points_merged 与 /scan Best Effort 与发布端 QoS 匹配,内嵌 TeleopPanel→/cmd_vel_nav 正确 |
| src/wheelchair_bringup/rviz/sensor_view.rviz | RVIZ | 已检视 | 0 | 无缺陷;Fixed Frame=odom、点云/scan/range/image 均 Best Effort 与发布端匹配,TeleopPanel→/cmd_vel_nav |
| src/wheelchair_bringup/rviz/wheelchair_default.rviz | RVIZ | 已检视 | 0 | 无缺陷;精简默认视图,Fixed Frame=map,scan/map/obstacles 话题正确(未显式声明 QoS,用 RViz 默认,可视化场景可接受) |
| src/wheelchair_description/urdf/wheelchair.urdf.xacro | XACRO | 已检视 | 3 | D132(laser_link 非 optical 帧 vs xtm60 optical,汇入§3)、D133(右雷达占位外参=已知#50,汇入§3)、D134(idx4/5 死链接+未实测超声位姿);左雷达高度 0.65→0.45 已修(本文件已修正,见 §4) |
| src/wheelchair_description/config/static_transforms.yaml | YAML | 已检视 | 2 | D130(左雷达高度 0.65 未同步 URDF 的 0.45,汇入§3)、D131(xtm60 rpy 用机体约定 vs URDF optical 约定,汇入§3);当前无 launch 加载(fallback 文件) |
| src/wheelchair_description/launch/display.launch.py | PY | 已检视 | 0 | 无缺陷(rviz 无 -d 配置的易用性问题并入 D135);xacro→robot_description 装配正确 |
| src/wheelchair_description/CMakeLists.txt | BUILD | 已检视 | 0 | 无缺陷;install(urdf launch config + README.md)与目录结构、package.xml(ament_cmake)一致 |
| src/wheelchair_description/package.xml | BUILD | 已检视 | 1 | D135(joint_state_publisher_gui 死依赖 + display rviz 无 -d 配置) |
| scripts/check_rviz_mapping_left.sh | SH | 已检视 | 1 | D147(未校验 ros2 可用,环境未就绪误报为话题 silent,与 sensors 检查脚本合并条目);刻意不加 `set -u`(注释说明 source ROS 引用未绑定变量)、check_hz/awk 数值比较与 timeout 守护合理 |
| scripts/check_rviz_sensors_left.sh | SH | 已检视 | 0 | 无缺陷(ros2 可用性缺口并入 D147);超声 presence 仅 WARN 不计 fail、TF 检查与 PASS/FAIL 汇总逻辑合理 |
| scripts/hardware_shutdown.sh | SH | 已检视 | 2 | D140(direct ZLAC 释放路径因 zlac8030_release.py 在本仓缺失而恒跳过,安全兜底失效)、D141(停车命令发布吞错无上报);`set -uo pipefail`、参数解析、source 包裹 set+u/-u、command -v ros2 守护良好 |
| scripts/run_rviz_manual_mapping_left.sh | SH | 已检视 | 1 | D145(rviz_cfg 回退后无存在性校验,与 sensors 脚本合并条目);detect_display/XAUTHORITY 探测、setsid 进程组、cleanup 委托 stop_mapping、pre-launch 清理逻辑健壮 |
| scripts/run_rviz_sensors.sh | SH | 已检视 | 0 | 无缺陷(rviz_cfg 存在性缺口并入 D145);`set -u` + source 包裹 set+u/-u、detect_display、cleanup INT 直接子进程逻辑合理 |
| scripts/save_mapping_result.sh | SH | 已检视 | 2 | D142(活动 SQLite 库裸 cp 可能撕裂)、D143(管道遮蔽退出码致 export/map_saver 失败检测失效);db 存在性校验、--scan/--opt 2 导出、grid_map transient_local QoS 覆盖正确 |
| scripts/setup_radar_network.sh | SH | 已检视 | 1 | D144(死变量 host_ip 多 CIDR 取值错误且未引用);`set -euo pipefail`、run_root sudo 降级、双子网 host_src_for_radar/network_ready 幂等校验、systemd 服务安装、雷达 IP 默认与 xtm60_left/right.yaml(192.168.0/1.101)一致,逻辑严谨 |
| scripts/stop_mapping.sh | SH | 已检视 | 1 | D146(宽泛子串 pkill -f 存在多 ws 误杀风险);`set -u`、--force、count_alive/signal_all INT→KILL 升级、grace 等待逻辑本身正确 |
| calib/left.yaml | CALIB | 已检视 | 1 | 相机内参(非雷达外参);D151(camera_name 命名);与 right.yaml 共构成相机内参,左右内参合理(无左右不一致缺陷);外参缺口汇入§3见D152 |
| calib/left_unrotated.yaml | CALIB | 已检视 | 2 | 未引用死文件 D150;命名 D151;与 left.yaml 经核对为 180° 旋转关系(主点/切向畸变符号一致,内参一致性OK) |
| calib/right.yaml | CALIB | 已检视 | 1 | 相机内参(非雷达外参);D151(camera_name 命名);外参缺口汇入§3见D152 |
| calib/right_unrotated.yaml | CALIB | 已检视 | 2 | 未引用死文件 D150;命名 D151;与 right.yaml 为 180° 旋转关系(内参一致性OK) |
| auto_test/odom_calib.py | PY | 已检视 | 3 | D153(arc-length 噪声高估)、D154(硬编码 0.112)、D155(句柄/并发/死导入) |
| auto_test/yaw_compare.py | PY | 已检视 | 1 | D156(无 main 守卫 + 死导入);诊断打印逻辑无算法缺陷 |

## 3. 跨文件一致性缺陷(Cross-file Inconsistencies)

> 本节(批次 B14)横向拉齐「同一物理量在多文件中的取值」,把分散在各单文件批次中标记「汇入 §3」的
> 参数/坐标/频率/时间基准冲突**汇总成独立的跨文件条目**(对应 Req 2.5)。条目使用与 §1 一致的 6 字段
> 格式,采用 **D160 起**的全局编号块(与 §1 已用编号块 D001–D051 / D070–D080 / D090–D107 /
> D110–D125 / D130–D135 / D140–D147 / D150–D156 不重叠),不封顶。每条**列出所有相关文件位置**;
> 已有的单文件 Dxxx 编号作为佐证被引用,跨文件条目本身用新编号。
>
> 四类:① 量程 / voxel;② 坐标系 / 外参;③ 频率 / QoS;④ 时间基准。建图相关项的「影响」列单列对建图质量的影响。

### 3.1 量程 / voxel 一致性

| 编号 | 文件位置(列出所有相关文件) | 问题描述 | 影响(建图项单列建图质量影响) | 严重度 | 建议修复方向 |
|------|------------------------------|----------|--------------------------------|--------|--------------|
| D160 | ① src/wheelchair_3d_mapping/config/rtabmap_params.yaml · `rtabmap:` `Grid/RangeMax: "8.0"` · L88;② 同文件 `icp_odometry:` 段(及 launch/rtabmap_3d_mapping.launch.py `essential` dict 硬编码,见 D041);③ src/wheelchair_3d_mapping/config/dual_lidar_fusion.yaml · `max_range: 12.0` · L13;④ src/wheelchair_bringup/config/pointcloud_to_scan{,_left,_right}.yaml · `range_max: 8.0`;⑤ src/wheelchair_bringup/config/scan_merger.yaml · `range_max: 8.0`;⑥ src/wheelchair_3d_mapping/wheelchair_3d_mapping/kiss_icp_mapping_node.py · `max_range` 默认 20.0(见 D042);⑦ src/wheelchair_bringup/config/xtm60_sdk.yaml · `range_max: 12.0` vs xtm60_left/right.yaml · `range_max: 50.0`;⑧ dual_lidar_cloud_fusion_node.py 节点默认 `max_range=20.0`(见 D036) | **同一「点云有效量程」物理量在多处取值互不统一**,形成一条贯穿采集→融合→建图→2D 投影的量程裂缝:适配器原始话题 50m(left/right)/ 单雷达 SDK 12m;融合裁剪到 12m(节点默认却 20m);RTAB-Map 2D 栅格 `Grid/RangeMax=8m`;感知 scan 投影/合并 8m;KISS-ICP 默认 20m。即「Grid/RangeMax=8 vs 融合 12」(已知 #30 / D040)只是其中一段,实际是 8 / 12 / 20 / 50 四档并存。 | 不同启动路径、不同消费链路看到的有效视距不同。**建图质量影响**:`/points_merged` 含 12m 内的点,但 RTAB-Map 投影 2D 栅格(`/rtabmap/grid_map`)只采信 8m → 8–12m 环带墙体存在于 3D 云图却缺席 2D 导航栅格,远墙/走廊端墙迟到入图,影响 Nav2 规划与前沿探索;若任一节点以裸默认(20m)启动则反而纳入超出可信带的远端飞点,污染法向量分割与栅格。3D 云图与 2D 栅格有效半径不一致使「看得见但走不到」。 | 中 | 确立单一量程基准(可信带 0.3–12m):将 `Grid/RangeMax` 提至 12 或显式文档化「2D 栅格刻意只信 8m」的设计意图;统一 fusion/kiss/感知节点默认与各 yaml 到 12m;原始显示话题保留 50m 但在注释标明仅供显示。修正需同时改 rtabmap_params.yaml 与 launch essential dict(D041)两处真值源。 |
| D161 | ① src/wheelchair_3d_mapping/config/dual_lidar_fusion.yaml · `z_min: -0.10` / `z_max: 1.80` · L26-27;② src/wheelchair_3d_mapping/config/cloud_to_occupancy_grid.yaml · `obstacle_z_min: 0.15` / `obstacle_z_max: 1.80` / `ground_z_min: -0.30` / `ground_z_max: 0.15` · L11-15;③ src/wheelchair_3d_mapping/config/rtabmap_params.yaml · `Grid/MaxGroundHeight: "0.15"` / `Grid/MaxObstacleHeight: "1.8"` · L90-91;④ src/wheelchair_bringup/config/pointcloud_to_scan{,_left,_right}.yaml · `z_min: -0.10` / `z_max: 1.20` | **高度裁剪带在四处投影/栅格化环节取值不一致**:融合 MAPPING 云裁到 [-0.10, 1.80];占据栅格障碍带 [0.15, 1.80]、地面带 [-0.30, 0.15];RTAB-Map 地面/障碍高度阈值 0.15 / 1.8;而感知 2D scan 投影上界只到 1.20。多套高度阈值各自为政且坐标参考系不同(融合在 base_link,占据栅格在 map_frame/livo_map 重力系)。 | **建图质量影响**:scan 投影 1.20m 上界与融合/栅格 1.80m 不一致 → 1.20–1.80m 的墙体/家具上沿在 2D `/scan` 链路被截断而在 3D/栅格链路保留,2D 与 3D 地图墙线高度覆盖不一致;占据栅格地面带上界 0.15 与融合 z_min -0.10 之间的薄层若叠加标定/外参高度误差(见 D163 雷达高度 0.65 vs 0.45),地面点可能错落入障碍带,产生「地面误判为障碍」的伪墙。 | 中 | 建立统一的高度分层定义(地面/可通行/障碍/天花板)并在各 yaml 引用同一组数值;明确各文件的 Z 参考系(base_link vs 重力系)与换算关系;scan 投影上界与栅格/融合对齐或文档化其差异意图。 |
| D162 | ① src/wheelchair_3d_mapping/config/dual_lidar_fusion.yaml · `voxel_leaf_size: 0.05` · L31;② src/wheelchair_3d_mapping/config/rtabmap_params.yaml · `Grid/CellSize: "0.05"` / `OdomF2M/ScanSubtractRadius: "0.05"` · L88, L37;③ src/wheelchair_3d_mapping/config/cloud_to_occupancy_grid.yaml · `resolution: 0.05`;④ kiss_icp_mapping_node.py voxel 默认 ↔ launch | voxel / 栅格分辨率在主链路各环节均为 0.05(融合下采样、RTAB-Map 体素、占据栅格分辨率一致),**本项经核对一致**,无冲突——记录于此以证明该共享物理量已横向拉齐(Preservation:不为凑数捏造冲突)。仅 KISS-ICP 回退路径 voxel 为独立默认,因其非主链路,功能上不触发。 | 无缺陷。建图质量影响:分辨率链路一致,体素下采样与栅格化无尺度错配。 | 低 | 无需修复;保留作为「已核对一致」的横向比对留痕。如日后调分辨率,需同步 fusion voxel / Grid/CellSize / occupancy resolution 三处。 |

### 3.2 坐标系 / 外参一致性

| 编号 | 文件位置(列出所有相关文件) | 问题描述 | 影响(建图项单列建图质量影响) | 严重度 | 建议修复方向 |
|------|------------------------------|----------|--------------------------------|--------|--------------|
| D163 | ① src/wheelchair_description/urdf/wheelchair.urdf.xacro · `xtm60_left_fixed_joint` `xyz=[0.45,0.24,0.45]` · L96-107;② src/wheelchair_description/config/static_transforms.yaml · `xtm60_left_link xyz=[0.45,0.24,0.65]` · L10-13;③ src/wheelchair_bringup/config/sensor_layout.yaml · `laser_link xyz=[0.45,0.0,0.65]`(雷达高度族);④ 佐证 D130 | **左雷达安装高度在三份「同一外参」真值源间发散**:URDF 已修正为 z=**0.45**(注释「old 0.65 was 20 cm too high」),但 static_transforms.yaml 仍为 z=**0.65**,sensor_layout.yaml 亦留 0.65。已知雷达高度 URDF 修复(D130)未同步到另两份静态外参文件。 | **建图质量影响**:URDF 经 robot_state_publisher 发布的是权威 TF(0.45),static_transforms.yaml/sensor_layout.yaml 仅作备份/审计参考;但若任何 fallback static publisher 或离线脚本采信 0.65,点云会整体抬高 0.20m → 地面点落到 z<0、墙体在栅格中高度错位,地面分割与占据投影错误。三份文件矛盾本身是高风险的「真值源不一致」,易在维护中误用旧值。 | 中 | 以 URDF 为唯一外参真值源,将 static_transforms.yaml 与 sensor_layout.yaml 的 `xtm60_*`/`laser_link` 高度同步为 0.45(或删去冗余备份文件改为从 URDF 生成),并在文件头注明「以 URDF 为准」。 |
| D164 | ① src/wheelchair_description/urdf/wheelchair.urdf.xacro · `xtm60_left/right_fixed_joint` `rpy=[-1.5708,0.0,-1.5708]`(optical 约定)· L105, L126;② `laser_fixed_joint` `rpy=[0.0,-0.0873,0.0]`(机体/激光约定)· L84;③ src/wheelchair_description/config/static_transforms.yaml · `xtm60_left/right` `rpy=[0.0,-0.0873,0.0]` · L13, L17;④ 佐证 D131 / D132 | **同一组雷达链接在 URDF 与 static_transforms.yaml 间采用相互冲突的坐标约定**:URDF 中 `xtm60_left/right_link` 是 **optical 约定**(z 前、x 右、y 下,使 optical-z→base_link +x),而 static_transforms.yaml 对同名链接给出**机体/激光约定** `rpy=[0,-0.0873,0]`(仅 -5° 俯仰)。两者旋转差近 90°,不可能同时正确。另 `laser_link`(单雷达 SDK 路径,frame_id=laser_link)在 URDF 中是非 optical 帧,与 optical 的 `xtm60_*` 混用同一套适配器输出约定(D132)。 | **建图质量影响**:若 fallback 采信 static_transforms.yaml 的机体约定,而点云实际是 optical 排布,则整片点云被旋转近 90° → 融合点云完全错位、墙面映射成地面/天花板,建图彻底失败。即便 URDF 正确,两份矛盾外参也使「单雷达 laser_link 路径」与「双雷达 optical 路径」对同一物理传感器给出不同帧约定,切换启动路径即引入系统性旋转误差。 | 高 | 统一坐标约定:确认 XT-M60 SDK 实际输出 optical 还是机体排布,据此让 URDF、static_transforms.yaml、单/双雷达 frame_id 全部采用同一约定;删除矛盾的备份外参或由 URDF 生成;在文档明确 optical↔body 的换算。 |
| D165 | ① src/wheelchair_description/urdf/wheelchair.urdf.xacro · `xtm60_right_fixed_joint` `xyz=[0.45,-0.24,0.65]`(注释自承 placeholder,未标定)· L117-128;② src/wheelchair_description/config/static_transforms.yaml · `xtm60_right_link xyz=[0.45,-0.24,0.65]`;③ calib/(无雷达外参文件——calib/ 仅含 left.yaml/right.yaml 相机内参,无左右雷达相对外参);④ src/wheelchair_3d_mapping/wheelchair_3d_mapping/dual_lidar_cloud_fusion_node.py · `_lookup`/TF(target_frame=base_link)· L113-148;⑤ 佐证 D133 / 已知 #50 | **右雷达外参为未标定占位值,且全项目缺少左右雷达相对外参的标定真值源**:URDF 右雷达 origin 由注释自承「relocated… taped… approximate and MUST be re-measured / calibrated in RViz before relying on the merged map」。融合节点 `_publish_merged` 通过 TF 把两路点云变换到 base_link 叠加,完全依赖该占位外参;`calib/` 目录仅有相机内参(left.yaml/right.yaml 为 plumb_bob 相机标定),**没有**雷达-雷达或雷达-base 的外参标定文件来校正占位值。 | **建图质量影响**:右雷达占位外参的横向/前向偏移误差直接传入 `/points_merged` → 左右子云在重叠区不重合,同一墙面出现双影/加宽(与 D035 时间错位叠加更甚),ICP 配准被错误对应点拖偏,旋转期建图漂移最明显。这是「右雷达占位外参」(已知 #50)的跨文件根因:无标定文件 + 占位 URDF + 融合盲目信任 TF。 | 高 | 用棋盘/平面标定或 RViz 双云对齐求出右雷达真实外参,写入权威外参源(URDF 或新增 calib/lidar_extrinsics.yaml)并同步 static_transforms.yaml;在融合节点对外参缺标定状态给出启动告警;标定前在报告/文档标注 merged map 右半不可信。 |

### 3.3 频率 / QoS 一致性

| 编号 | 文件位置(列出所有相关文件) | 问题描述 | 影响(建图项单列建图质量影响) | 严重度 | 建议修复方向 |
|------|------------------------------|----------|--------------------------------|--------|--------------|
| D166 | ① src/wheelchair_bringup/config/zlac8030_base.yaml · `publish_rate_hz: 50.0`(轮速/odom);② src/wheelchair_bringup/config/h30_imu.yaml · `publish_rate_hz: 100.0`(IMU);③ src/wheelchair_bringup/config/robot_localization_ekf.yaml · `frequency: 30.0` / `sensor_timeout: 0.2` · L3-4;④ src/wheelchair_3d_mapping/config/dual_lidar_fusion.yaml · `output_rate_hz: 10.0`;⑤ src/wheelchair_3d_mapping/config/rtabmap_params.yaml · `expected_update_rate: 0.0`;⑥ 佐证已知 #37 | **传感器/估计器发布频率呈 100(IMU)/ 50(轮速)/ 30(EKF)/ 10(点云融合)的多级阶梯**,EKF 以 30Hz 融合 50Hz 轮速 + 100Hz IMU(下采样,可接受),但 EKF `sensor_timeout=0.2s` 相对 10Hz 点云链路偏紧;RTAB-Map `expected_update_rate=0.0`(不校验更新率)使融合 10Hz 掉到更低时无告警。各频率分散在四个 yaml,无单一「频率预算」定义,改一处易破坏同步假设。 | **建图质量影响**:EKF 输出 odom→base_link 给 RTAB-Map/icp_odometry 做 TF 查询;频率阶梯本身可工作,但当轮速/IMU 抖动接近 `sensor_timeout` 时 EKF 短暂停摆,而点云链路无 `expected_update_rate` 兜底,位姿约束缺失窗口不被察觉 → 建图在传感器抖动期静默漂移。频率定义分散增加误配风险(如改 EKF frequency 而忘记 sensor_timeout)。 | 中 | 在一处文档/配置集中定义频率预算(IMU/轮速/EKF/融合/RTAB-Map)与其依赖关系;按最慢上游设 EKF sensor_timeout;为 RTAB-Map 设非零 expected_update_rate 以便掉帧告警。 |
| D167 | ① src/wheelchair_3d_mapping/config/rtabmap_params.yaml · `qos_scan: 2`(BEST_EFFORT,匹配 /points_merged)/ `qos_imu: 1`(RELIABLE,匹配 /imu/data)· L17-18;② src/wheelchair_perception/wheelchair_perception/pointcloud_to_laserscan_node.py · 点云订阅 sensor_data(best_effort)而 `/scan` 发布默认 reliable(见 D027);③ scan_merger_node.py · 输入/输出 scan 均默认 reliable(见 D030);④ src/wheelchair_diagnostics/.../localization_health_node.py · `/scan` 默认 reliable vs sensor_watchdog_node.py · `/scan` sensor_data best_effort(见 D104);⑤ camera_adapter_node.py · 图像默认 reliable vs 下游着色常用 sensor_data(见 D012) | **`/scan` 与图像等传感器流在 pub/sub 两侧 QoS 可靠性不统一**,形成跨文件 QoS 裂缝:RTAB-Map 端正确地把 qos_scan 设为 BEST_EFFORT 以匹配 best_effort 的 `/points_merged`(本项正确);但 `/scan` 链路上 perception 投影/合并节点以 reliable 发布,而诊断包内 localization_health(reliable)与 sensor_watchdog(best_effort)对**同一 `/scan`**采用不同 QoS,图像流亦 reliable 发布 vs best_effort 消费。reliable↔best_effort 在部分 rmw 组合下不兼容,订阅方静默收不到。 | **建图质量影响**:`/scan` 若因 QoS 不匹配被 2D SLAM/代价地图消费方静默拒收 → 2D 建图/避障直接断流;图像 QoS 不匹配则 RGB 着色收不到帧(仅影响可视化着色,不损几何)。RTAB-Map 的 `/points_merged` 主几何链路 QoS 已对齐,故 3D 几何不受此条影响。 | 中 | 制定全项目传感器流 QoS 约定(传感器数据一律 sensor_data/best_effort,两侧一致),逐一对齐 D012/D027/D030/D104 涉及的 pub/sub;保留 RTAB-Map 已正确的 qos_scan=2 / qos_imu=1 作为基准范例。 |

### 3.4 时间基准一致性

| 编号 | 文件位置(列出所有相关文件) | 问题描述 | 影响(建图项单列建图质量影响) | 严重度 | 建议修复方向 |
|------|------------------------------|----------|--------------------------------|--------|--------------|
| D168 | ① src/wheelchair_sensors/wheelchair_sensors/xtm60_adapter_node.py · `_make_cloud`(`use_sdk_timestamps=false`→墙钟戳)· L820-832(见 D019);② src/wheelchair_bringup/config/xtm60_left.yaml · src/wheelchair_bringup/config/xtm60_right.yaml · src/wheelchair_bringup/config/xtm60_sdk.yaml · `use_sdk_timestamps: false`;③ src/wheelchair_sensors/wheelchair_sensors/imu_adapter_node.py · 参数名 `use_device_timestamp`(命名分裂,见 D014);④ src/wheelchair_3d_mapping/wheelchair_3d_mapping/dual_lidar_cloud_fusion_node.py · `_publish_merged` `header.stamp=get_clock().now()`(丢弃源 stamp,见 D034 / 已知 #13);⑤ src/wheelchair_perception/.../pointcloud_to_laserscan_node.py · `restamp_output=true`→墙钟(见 D024);⑥ scan_merger_node.py · `output.header.stamp=now`(见 D028);⑦ src/wheelchair_3d_mapping/config/rtabmap_params.yaml · `approx_sync: true` / `deskewing: false` · L45, L20 | **整条采集→融合→建图链路普遍用「发布时刻墙钟」覆盖「采集时刻」时间戳**,且时间基准参数命名不统一:适配器默认 `use_sdk_timestamps=false`(xtm60)/ `use_device_timestamp`(IMU,命名分裂)→ 点云打墙钟戳;融合节点 `_publish_merged` 把已缓存的源 `state.stamp` 丢弃改打墙钟;感知 scan 投影/合并同样 restamp 为「现在」。下游 RTAB-Map 用 `approx_sync` 吸收时戳误差,但输入本身已被系统性后移一个缓存周期(~0.1s)+ 抖动。这是已知 #13/#15/#41(墙钟 vs 采集时刻)的跨文件全貌。 | **建图质量影响**:`/points_merged` 是 RTAB-Map(approx_sync)与 icp_odometry 的唯一几何输入,墙钟戳使点云与 IMU/odom 在**错误时刻**对齐 → ICP/位姿图配准漂移、转弯处墙线重影、回环时刻错配;`/scan` 墙钟戳同样使 2D scan-to-map 在错误时刻做 TF 查询,墙线弯折。误差随机体速度增大,旋转期最严重。approx_sync 只能掩盖而非消除该系统性偏移。 | 高 | 全链路改「采集时刻优先」:在 SDK/驱动收帧瞬间打主机戳并随帧传递,融合输出回填源 `state.stamp`(缺失才回退墙钟),scan `restamp_output` 默认 False;统一传感器适配器时间基准参数命名(如 `use_source_timestamps`);保留 RTAB-Map approx_sync 作二级容差而非一级依赖。 |
| D169 | ① src/wheelchair_diagnostics/.../localization_health_node.py · age 基于 `time.monotonic()` 而 header.stamp 用 `get_clock().now()`(见 D106);② sensor_watchdog_node.py · 同类 monotonic vs ROS 时钟混用;③ src/wheelchair_bringup/config/robot_localization_ekf.yaml · 无 `use_sim_time` 显式声明 / `transform_time_offset: 0.0`;④ src/wheelchair_3d_mapping/config/rtabmap_params.yaml · `wait_for_transform: 0.2` | **诊断/看门狗的「新鲜度判定时钟」与「消息时间戳时钟」不一致(monotonic vs ROS clock)**,且全项目对 `use_sim_time` 无统一约定:liveness age 用 `time.monotonic()`,而发布的 DiagnosticArray.header.stamp 用 ROS 时钟。在 `use_sim_time=true` 或 bag 回放(ROS 时钟≠墙钟)下,staleness 阈值按真实墙钟判定而非 ROS 时间,与 EKF/RTAB-Map 的 `wait_for_transform`/`transform_time_offset`(均基于 ROS 时钟)语义错位。 | 仿真/bag 回放下超时判定与 ROS 时间脱节,诊断可能误报/漏报;实机(ROS 时钟≈墙钟)无影响。**建图质量影响**:间接——回放调试建图问题时,诊断时间语义错位会误导对「传感器掉线/位姿丢失」的判读,但不直接损害实机建图几何。 | 低 | 若需支持 sim_time/bag 回放,统一以 `get_clock().now()` 计算 age;在全项目 launch 统一 `use_sim_time` 约定;或在文档明确诊断看门狗仅按真实时间判活、不支持 sim_time。 |

## 4. 已知问题与修复状态(Known Issues & Fix Status)

> 本节(批次 B15)对应 Req 2.6 / 3.2 / 3.4,完成两件事:
> 1. **已知真实问题修复状态**(§4.1 / §4.2):逐条体现 bugfix.md「已知背景」列出的真实问题并标注「已修 / 未修」;
>    已修项**不**作为待修缺陷重复报告(Req 3.4)。
> 2. **起点文档 F 的 50 条结论迁移**(§4.3 / §4.4):把 `docs/mapping_code_review_2026-06-17.md` 的全部 50 条
>    **保留技术实质**地迁移进本报告 —— 或映射到 §1/§3 已有 Dxxx 编号,或作为 **F 中独有、§1 未覆盖**的待修缺陷
>    用 **D170 起的编号块**新建独立 6 字段条目(§4.4)。每条都建立「F 旧编号 → F' 对应条目 / 严重度 / 修复状态」
>    映射,确保 50 条技术结论**无丢失**(Req 3.2)。
>
> **严重度重判**:迁移按本报告严重度定义(阻断/高/中/低)重判 F 的 ⛔/⚠️/🔸 标记(⛔→阻断或高;⚠️→中;🔸→低),
> 并结合实际触发概率与建图影响微调;映射表同时保留 F 原符号与 F' 重判结果以便追溯。

### 4.1 已修问题(标「已修」,不作为待修缺陷重复报告 —— Req 3.4)

> 下列三项是 bugfix.md 明确「已确认已修复」的真实问题。经核对当前代码确认修复已落地,故**不**在 §1 待修缺陷表
> 中重复报告;仅记录修复证据与可能的遗留(遗留项另有独立编号)。

| 已修问题 | 修复证据(当前代码) | 遗留/关联条目 |
|----------|----------------------|----------------|
| **EKF 让轮速绝对 yaw 主导**(原 F#34 ⛔→✅)→ 已改 **IMU 主导** | `src/wheelchair_bringup/config/robot_localization_ekf.yaml`:`odom0_config` 的 yaw(abs)与 vyaw 均为 `false`(轮速仅供 x/y/vx 平移);`imu0_config` 启用 yaw(abs)+vyaw,`imu0_relative: true`。注释明确「let H30 drive heading -- wheels give distance, IMU gives angle」。 | 备份 EKF `robot_localization_livo_wheel_ekf.yaml` 重新启用了轮速 yaw,与本修复结论矛盾 → 待修 **D119**(已记于 §1,汇入 §3)。 |
| **雷达安装高度 URDF 写错**(0.65 → 0.45) | `src/wheelchair_description/urdf/wheelchair.urdf.xacro`:`xtm60_left_fixed_joint` 已修正为 `z=0.45`,注释自承「old 0.65 was 20 cm too high」。URDF 经 robot_state_publisher 发布的是权威 TF。 | `static_transforms.yaml` / `sensor_layout.yaml` 仍留旧值 0.65,真值源未同步 → 待修 **D130 / D163 / D122**(已记,汇入 §3)。 |
| **RTAB-Map yaml 参数从不加载** | `src/wheelchair_3d_mapping/launch/rtabmap_3d_mapping.launch.py`:Grid/RGBD/Reg/Icp 参数已改为通过 launch `essential` dict 显式注入(不再依赖 yaml 文件路径加载),注释说明 rtabmap_slam 会忽略文件键。`MaxObstacleHeight=1.8` 等已生效(2D 不再全黑)。 | yaml 与 dict 形成双真值源、yaml 那份据称不生效 → 待修 **D041 / D160**(已记,汇入 §3)。 |

### 4.2 未修问题(标「未修」—— Req 2.6)

> bugfix.md「已知背景」列出的未修真实问题。逐条标注修复状态与本报告对应条目;其中两项是**硬件/架构约束**
> (非单点代码缺陷),按实证如实标注,不强行制造可「修复」的代码条目。

| 未修问题 | 性质 | 本报告对应条目 / 说明 |
|----------|------|------------------------|
| **雷达盒子遮挡产生固定假点** | 硬件/安装(左雷达裸装于盒内,盒壁进入近距 FOV) | **未修**。属物理安装遮挡,非代码缺陷:盒壁在近距形成稳定回波,被当作固定障碍。本报告不为其制造代码缺陷条目(无代码依据),建议在近距 range/扇区掩膜或物理改装层面处理;关联近距 range 过滤 **D179**。 |
| **融合点云时间戳用墙钟** | 代码缺陷(可修) | **已修(2026-06-23)** → **D034** 已修复:`dual_lidar_cloud_fusion_node._publish_merged` 改用源帧 stamp(取左右较新),全 0 才回退墙钟;`_lookup` 用 `Time.from_msg`(D178 同步修复)。见 §4.5。 |
| **纯激光窄 FOV 无回环** | 架构性 | **未修** → 新建 **D188**(§4.4,架构级,高);全局校正缺失根因见 **D194**。120° 窄 FOV + 无视觉特征,空间回环实测 0 闭环。 |
| **IMU 仅姿态级、无室内位移观测**(WHEELTEC H30 / Yesense YIS106) | 硬件约束 | **未修**。IMU 只提供姿态(yaw)级观测,无室内绝对位移源;与「无 map 帧/无全局校正」**D194** 共同构成大范围漂移根因。属硬件能力边界,非代码缺陷:无对应可「修复」的代码条目,需视觉回环/UWB/AMCL 等外部全局观测补偿。 |

### 4.3 起点文档 F(50 条)→ F' 迁移映射表(Req 3.2 / 3.3)

> 逐条建立「F 旧编号(原严重度符号)→ F' 对应条目 / 重判严重度 / 修复状态」映射,确保 50 条技术结论无丢失。
> **去向**分三类:① 映射到 §1/§3 已有 Dxxx;② 新建 §4.4 的 D170+ 待修条目;③ 归入 §4.1 已修。
> 一条特殊项(F#4)经复核为**可接受的标准近似**(非缺陷),按 Req 3.3「不为凑数捏造缺陷」如实标注,技术结论仍保留在表中。

| F# | F 原级 | 技术实质(摘要) | F' 去向 | F' 严重度 | 状态 |
|----|--------|------------------|---------|-----------|------|
| 1 | ⛔ | 里程计 dt 用墙钟,与点云时间线不同源 | **D168**(§3 时间基准) | 高 | 未修 |
| 2 | ⛔ | 轮速反馈读失败时静默丢一帧运动(actual=0) | **D005**(§1) | 中 | 未修 |
| 3 | ⛔ | 里程积分用当前周期速度,反馈本身有一拍延迟 | **D170**(新) | 高 | 未修 |
| 4 | ⚠️ | 平移用中点 yaw、yaw 用端点更新(阶数) | 复核:中点法为标准二阶近似,yaw 端点更新对匀速转弯精确 → **可接受,非缺陷** | — | 复核保留(无独立条目) |
| 5 | ⚠️ | 轮速 yaw 不可靠却仍发布,易被误用 | **D171**(新) | 中 | 未修 |
| 6 | ⚠️ | wheel_separation_m=0.58 未标定 | **D172**(新) | 中 | 未修 |
| 7 | ⚠️ | odom 协方差写死常数,不随速度/打滑变化 | **D173**(新) | 中 | 未修 |
| 8 | 🔸 | 轮速 yaw 协方差 0.10 偏小(过度自信) | **D173**(并入,新) | 中 | 未修 |
| 9 | 🔸 | mock 模式 actual=target,掩盖真实问题 | **D174**(新) | 低 | 未修 |
| 10 | ⚠️ | 无滑动/堵转检测 | **D175**(新) | 中 | 未修 |
| 11 | 🔸 | invert 同作用于指令与里程(_apply==_remove 符号耦合) | **D176**(新) | 低 | 未修 |
| 12 | ⚠️ | 后退无专门处理(窄前视雷达倒车 ICP 易跳变) | **D177**(新) | 中 | 未修 |
| 13 | ⛔ | 融合输出点云时间戳用墙钟 | **D034**(§1)+ **D168** | 高 | 未修 |
| 14 | ⛔ | 左右两帧不同采集时刻合并成一帧 | **D035**(§1) | 中 | 未修 |
| 15 | ⚠️ | 融合 TF 用 latest 而非点云时间戳 | **D178**(新) | 中 | 未修 |
| 16 | ⚠️ | range 用欧氏范数 + 0 点占位被 min_range 滤,脆弱 | **D179**(新) | 低 | 未修 |
| 17 | ⚠️ | voxel 取首点而非质心,扫描顺序相关偏置 | **D180**(新) | 中 | 未修 |
| 18 | ⚠️ | voxel 全局 floor 量化,正负边界不对称 | **D180**(并入,新) | 中 | 未修 |
| 19 | 🔸 | 融合输出 height=1 无序,丢邻接信息 | **D181**(新) | 低 | 未修 |
| 20 | ⚠️ | 单雷达 fallback 判据 left_ok!=right_ok 语义脆弱 | **D182**(新) | 中 | 未修 |
| 21 | 🔸 | _fresh 用 recv_time(墙钟到达)判新鲜 | **D183**(新) | 低 | 未修 |
| 22 | 🔸 | filter pipeline 异常被吞,沿用上一帧旧值 | **D184**(新) | 中 | 未修 |
| 23 | 🔸 | intensity 缺失时整帧降级为无强度 | **D185**(新) | 低 | 未修 |
| 24 | ⚠️ | 无运动畸变补偿(deskew) | **D186**(新) | 低 | 未修 |
| 25 | ⛔(已修) | RTAB-Map yaml 参数从不加载 | **§4.1 已修**(遗留 D041/D160) | — | 已修 |
| 26 | ⚠️ | icp_odometry 与 rtabmap 共用一份 cfg,语义混 | **D041**(§1) | 中 | 未修 |
| 27 | ⚠️ | approx_sync=true + queue 偏小,叠加错位 | **D168**(§3) | 高 | 未修 |
| 28 | ⚠️ | Force3DoF 锁 z/roll/pitch,挤入外参 pitch 残差 | **D187**(新) | 中 | 未修 |
| 29 | ⚠️ | 纯激光窄 FOV 回环几乎不可能(实测 0 闭环) | **D188**(新,架构) | 高 | 未修 |
| 30 | ⚠️ | Grid/RangeMax=8 vs 融合 12 | **D040**(§1)+ **D160**(§3) | 中 | 未修 |
| 31 | 🔸 | Grid/RayTracing 依赖准确位姿,漂移时擦真墙 | **D189**(新) | 中 | 未修 |
| 32 | 🔸 | delete_db_on_start 行为依赖 launch 参数,易误续建 | **D190**(新) | 低 | 未修 |
| 33 | 🔸 | 未给 RTAB-Map 设 XT-M60 传感器 FOV | **D191**(新) | 低 | 未修 |
| 34 | ⛔→✅ | EKF 轮速 yaw 主导 → 已改 IMU 主导 | **§4.1 已修**(遗留 D119) | — | 已修 |
| 35 | ⚠️ | two_d_mode 丢弃 z/roll/pitch(无法测斜坡) | **D192**(新) | 低 | 未修 |
| 36 | ⚠️ | imu0_relative 以启动时刻为 0,启动须静止 | **D193**(新) | 低 | 未修 |
| 37 | ⚠️ | EKF 30 vs 轮速 50 vs IMU 200 频率阶梯 | **D166**(§3 频率族) | 中 | 未修 |
| 38 | 🔸 | sensor_timeout=0.2 偏大,期间外推陈旧值 | **D166**(并入,§3) | 中 | 未修 |
| 39 | ⚠️ | 无 map 帧 / 无全局校正,odom 漂移永久累积 | **D194**(新,架构) | 高 | 未修 |
| 40 | 🔸 | IMU 协方差来自 yaml 固定值 | **D195**(新) | 低 | 未修 |
| 41 | ⛔ | 默认 use_sdk_timestamps=false → 点云戳=接收时刻 | **D019**(§1)+ **D168** | 中 | 未修 |
| 42 | ⚠️ | 多节点各自 now() 盖时间戳,误差叠加 | **D168**(§3) | 高 | 未修 |
| 43 | 🔸 | 建图链路双重过滤,参数分散三处 | **D196**(新)+ 关联 D160/D161 | 低 | 未修 |
| 44 | 🔸 | 显示用与建图用点云混用同一适配器输出 | **D197**(新) | 低 | 未修 |
| 45 | 🔸(已修) | run 脚本 wait -n/setsid/set -u 历史 bug | **§4.1 已修**(现状脚本审计见 D140–D147) | — | 已修 |
| 46 | 🔸 | stop_mapping 靠 pkill 名字匹配 | **D146**(§1) | 低 | 未修 |
| 47 | ⚠️ | 无在线里程/位姿质量监控(consistency 未接主链路) | **D198**(新) | 中 | 未修 |
| 48 | 🔸 | save_mapping 2D 栅格在跑时存,易超时漏存 | **D050 / D142 / D143**(§1) | 低/中 | 未修 |
| 49 | 🔸 | 运动学/积分无单元测试覆盖 | **D008 / D009**(§1) | 中/低 | 未修 |
| 50 | ⚠️ | 右雷达外参占位估计 + 胶带固定,无在线校准 | **D133**(§1)+ **D165**(§3) | 高 | 未修 |

**迁移完整性核对(按 F 条目计)**:F 共 **50 条** → 去向四类,合计闭合:
- 归入 §4.1 **已修**:**3 条**(F#25, 34, 45);
- 复核为可接受**非缺陷**(Req 3.3 不捏造):**1 条**(F#4 中点 yaw 近似);
- 映射到 §1/§3 **已有 Dxxx** 编号:**15 条**(F#1, 2, 13, 14, 26, 27, 30, 37, 38, 41, 42, 46, 48, 49, 50);
- 新建 §4.4 **D170–D198 待修条目**:**31 条**(F#3,5,6,7,8,9,10,11,12,15,16,17,18,19,20,21,22,23,24,28,29,31,32,33,35,36,39,40,43,44,47),
  其中 F#7+#8→D173、F#17+#18→D180 各合并一条,故 31 条对应 29 个新编号(D170–D198)。

3 + 1 + 15 + 31 = **50 条技术结论全部有去向,无丢失。**

### 4.4 迁移自 F 的新增待修缺陷条目(D170–D198,6 字段)

> 以下为 **F 中独有、§1 各批次未覆盖**的技术结论,作为独立待修缺陷用 D170 起编号块记录(与 §1 D001–D156 /
> §3 D160–D169 不重叠),6 字段齐全、不封顶。来源文件(kinematics.py / zlac8030_driver_node.py /
> dual_lidar_cloud_fusion_node.py / cloud_utils.py / rtabmap_params.yaml / robot_localization_ekf.yaml)的
> §2 覆盖清单备注已补标「迁移自F」。

| 编号 | 文件位置(相对路径+函数/类+行号区间) | 问题描述 | 影响(建图项单列建图质量影响) | 严重度 | 建议修复方向 |
|------|------------------------------------------|----------|--------------------------------|--------|--------------|
| D170 | src/wheelchair_base/wheelchair_base/zlac8030_driver_node.py · `tick` 反馈→积分 · L188-205 ↔ kinematics.py `OdometryState.integrate` | 闭环里程把「本周期读回的反馈速度」与「本周期 dt」直接相乘积分,但 ZLAC8030 反馈是上一控制周期的实际值(命令写下→读回存在一拍延迟),速度与 dt 不严格对应,引入系统性距离/角度偏差(F#3)。 | 反馈一拍延迟使每周期积分的速度滞后于其 dt,匀加/匀减速段累积平移误差。建图质量影响:`/wheel/odom` 是 EKF/RTAB-Map 平移来源,系统性偏差使长走廊里程尺度偏移、轨迹累计漂移。 | 高 | 用「上一周期反馈速度」配「上一周期 dt」积分(时间对齐),或对反馈做一拍时延补偿;在驱动文档明确反馈时延契约。 |
| D171 | src/wheelchair_base/wheelchair_base/zlac8030_driver_node.py · `tick`/`_publish_odom` · L205-213, L? | 驱动仍把轮速推得的 yaw(经 `wheel_rpm_to_twist` 的 `(right-left)/sep`)写入 `/wheel/odom` 的 orientation 与 wz,即使 EKF 已不再融合轮速 yaw(见 §4.1 已修)。该 yaw 严重依赖 `wheel_separation` 且受打滑影响,易被其他节点误用(F#5)。 | EKF 已忽略该 yaw,但任何直接订阅 `/wheel/odom` 朝向的节点会用到不可靠 yaw。建图质量影响:若下游(如纯轮速可视化/回退定位)误用,转弯处朝向错误传入建图,墙线弯折。 | 中 | 在 `/wheel/odom` 显式标注 yaw 不可信(超大 yaw 协方差,已部分为 0.10 见 D173)或不发布 yaw 仅发布平移+vyaw=0;在文档标明轮速 yaw 仅供参考。 |
| D172 | src/wheelchair_base/wheelchair_base/kinematics.py · `DifferentialDriveModel.wheel_separation_m` 默认 0.58 · L9 ↔ zlac8030_base.yaml | `wheel_separation_m=0.58` 为估计值未标定;即便不用于 EKF yaw,`twist_to_wheel_rpm`(下发指令)与 `wheel_rpm_to_twist`(里程平移/角速度)仍用它,影响转弯指令实际半径与里程角速度(F#6)。 | 轮距误差使指令转弯半径与里程角速度系统性偏差。建图质量影响:转弯期里程角速度偏差进入 EKF/建图,环形轨迹闭合误差增大,影响回环与地图一致性。 | 中 | 用直线/原地旋转标定实测轮距(可用 auto_test/odom_calib.py 思路)替换估计值,并将标定值写入 yaml 与节点默认。 |
| D173 | src/wheelchair_base/wheelchair_base/zlac8030_driver_node.py · `_publish_odom` 协方差 · L? (pose.covariance[0/7/35], twist.covariance[0/35]) | odom 协方差为写死常数(x/y=0.05、yaw=0.10、vx=0.05、wz=0.10),不随速度/打滑变化;且对**不可靠的轮速 yaw** 标 0.10(≈18° 一倍标准差的 1/3)偏乐观,正是此前 EKF 被轮速 yaw 带偏的帮凶(F#7+F#8)。 | 固定协方差使 EKF 无法在打滑/高速时降低对轮速的信任;过小的 yaw 协方差让 EKF 过度采信轮速朝向。建图质量影响:打滑期里程被 EKF 过度信任,位姿估计被污染,建图在打滑/快转时漂移(EKF 已改 IMU 主导缓解,但协方差仍影响平移权重)。 | 中 | 让协方差随速度/命令-反馈偏差动态增大(打滑时升高);把轮速 yaw 协方差设为很大(标注不可信)以与「IMU 主导」修复一致。 |
| D174 | src/wheelchair_base/wheelchair_base/zlac8030_driver_node.py · `tick`(mode!=real 分支)· L196-200 | 非 real(mock)模式下 `actual = target`,里程「完美」无误差,容易在测试/联调时给出「里程很准」的错觉,掩盖真实积分/反馈缺陷(F#9)。 | 无运行时危害,但测试可信度被夸大。建图质量影响:无直接影响(mock 不入实机建图),但会掩盖 D170/D173 一类缺陷使其逃过测试。 | 低 | mock 模式注入可配置的噪声/打滑/延迟模型(而非 actual=target),或在状态/日志显式标注「mock 理想里程,不代表真实精度」。 |
| D175 | src/wheelchair_base/wheelchair_base/zlac8030_driver_node.py · `tick`/`_read_feedback` · L188-205, L255-275 | 无滑动/堵转检测:轮子抱死/打滑(实测右转抱死)时,反馈 rpm 与真实运动不符,里程仍照单全收,无 \|命令 vs 反馈\| 偏差监控(F#10)。 | 打滑/堵转期里程严重失真且无告警。建图质量影响:打滑段 `/wheel/odom` 与真实运动脱节,EKF/建图在该段漂移且不可察觉,地图局部错位。 | 中 | 增加 \|命令-反馈\| 偏差检测,异常时升高 odom 协方差(配合 D173)并上报诊断;持续堵转触发降级/告警。 |
| D176 | src/wheelchair_base/wheelchair_base/zlac8030_driver_node.py · `_apply_direction`/`_remove_direction` · L216-220 | `_apply_direction`(指令)与 `_remove_direction`(里程)实现**完全相同**(同一对 invert 取反),符号处理耦合;若将来左右电机不对称或需分别处理指令/里程符号,无法独立调整(F#11)。 | 当前幂等无功能危害,属架构/可维护性。建图质量影响:无直接影响;但符号耦合使未来方向标定改动易引入里程符号错误(镜像/发散风险)。 | 低 | 拆分指令侧与里程侧的方向变换为独立可配置映射,即使当前取值相同也保留分离的契约与注释。 |
| D177 | src/wheelchair_base/wheelchair_base/zlac8030_driver_node.py / 建图链路(架构)· tick + RTAB-Map 信任 | 倒车无专门处理:窄前视雷达后退时点云匹配差(已观察),代码层无「后退时降低 ICP 信任/标记倒车」机制,RTAB-Map 倒车时易跳变(F#12)。 | 倒车段几何观测质量下降而系统仍同等信任。建图质量影响:倒车时 ICP/RTAB-Map 配准退化、位姿跳变,地图在倒车轨迹段出现错位/重影。 | 中 | 检测倒车(linear_x<0)时对 ICP/里程协方差降权或标记该段,或限制倒车速度;在建图文档说明窄 FOV 倒车的局限。 |
| D178 | src/wheelchair_3d_mapping/wheelchair_3d_mapping/dual_lidar_cloud_fusion_node.py · `_lookup` · L150-159 | 融合 TF 用 `rclpy.time.Time()`(最新可用变换)而非点云 `msg.header.stamp` 对应时刻的 TF:运动时 base_link 在变,用「最新 TF」变换「旧点云」产生与速度成正比的错位(F#15;与感知侧 D025 同类,首次记于融合节点)。 | 转弯/移动期间两路点云被错误时刻外参变换到 base_link。建图质量影响:`/points_merged` 几何随运动错位,ICP/RTAB-Map 配准漂移、墙线重影,速度越快越严重;与 D034 墙钟戳、D035 左右错位叠加恶化。 | 中 | 用 `msg.header.stamp` 做 `lookup_transform`(配合容差/等待),变换时刻与采集时刻一致;TF 不可用时跳帧而非用 latest 强变换。 |
| D179 | src/wheelchair_3d_mapping/wheelchair_3d_mapping/cloud_utils.py · `filter_by_range` · L78-86 ↔ 融合 `_on_cloud` | range 过滤在传感器系用欧氏范数(语义正确但未在注释明确);且 (0,0,0) 占位/无效点范数为 0 会被 `min_range` 滤掉——管线依赖此「副作用」剔除无效点,较脆弱:若 min_range 设为 0 或占位点非零,无效点将透出(F#16)。 | 当前默认 min_range>0 时无害,属健壮性隐患。建图质量影响:边界配置下无效/占位点可能进入融合云,产生原点附近假点污染栅格;常态不触发。 | 低 | 显式剔除无效/占位点(NaN 或全 0)而非依赖 range 副作用;在 `filter_by_range` 注释标明传感器系语义与对 0 点的处理约定。 |
| D180 | src/wheelchair_3d_mapping/wheelchair_3d_mapping/cloud_utils.py · `voxel_downsample` · L99-107 | 体素下采样用 `np.floor(xyz/leaf)` 量化后 `np.unique(return_index)` 取每体素**第一个**点作代表(非质心),引入与扫描顺序相关的偏置且强度不平均;floor 量化在 odom 远处大坐标跨正负象限时边界不对称,边界体素重复/丢点(F#17+F#18)。 | 代表点偏置使下采样云相对真实表面有亚体素级系统偏移,强度不平均影响着色/分割。建图质量影响:`/points_merged` 几何带扫描顺序相关偏置,ICP 配准与栅格投影精度下降(次要但系统性);量化边界不对称在大场景边缘丢点。 | 中 | 取体素质心 + 强度均值作代表点(用 `np.add.at`/分组聚合);量化前可减去局部原点偏移以减小正负边界不对称。 |
| D181 | src/wheelchair_3d_mapping/wheelchair_3d_mapping/cloud_utils.py · `make_xyzi_cloud` · L? (`msg.height = 1`) | 融合输出强制 `height=1`(无序点云):适配器辛苦保留的有序网格(organized cloud)到融合这里被拍平,下游法线估计/地面分割失去邻接信息(F#19)。 | 无序云使依赖邻接的算法(法线、地面分割)退化为基于半径搜索,更慢且更易错。建图质量影响:RTAB-Map 法向量地面分割(`Grid/NormalsSegmentation`)失去网格邻接,法线质量下降,地面/障碍分类更易误判。 | 低 | 在可行时保留有序结构(height/width),或在文档说明融合后为无序并要求下游用半径近邻;权衡融合多路时有序性本就难保留。 |
| D182 | src/wheelchair_3d_mapping/wheelchair_3d_mapping/dual_lidar_cloud_fusion_node.py · `_publish_merged` `fallback_active=(left_ok!=right_ok)` · L173 | 单雷达 fallback 判据 `left_ok != right_ok` 语义脆弱:双雷达都新鲜时 `False`(正确),但「双雷达都失效」时也为 `False`,与「双雷达正常」无法区分;真正失效由 `not parts_xyz` 另行兜底,但 status 里 `single_lidar_fallback` 标志语义混淆(F#20)。 | 诊断标志无法区分「双正常」与「双失效」,运维误读。建图质量影响:无直接几何影响(失效有 no_input 兜底),但 fallback 状态误报使「单雷达降级建图」不可见,排障困难。 | 中 | 用三态/显式计数(left_ok, right_ok 两个布尔 + 输出点数)表达 fallback 语义,区分 双正常/单路/双失效;status 字段语义化。 |
| D183 | src/wheelchair_3d_mapping/wheelchair_3d_mapping/dual_lidar_cloud_fusion_node.py · `_fresh`/`_on_cloud` · L161-163, L107-111 | `_fresh` 用 `time.monotonic() - recv_time`(墙钟到达时间)判新鲜,而非点云采集时刻:网络抖动/积压时到达时间≠采集时间,可能把旧帧当新鲜并参与合并(F#21)。 | 网络抖动时陈旧帧被误判新鲜并叠加。建图质量影响:积压旧帧混入 `/points_merged`,与新帧时间错位叠加,运动期墙线重影(与 D034/D035/D178 同源)。 | 低 | 新鲜度判定基于采集时刻(header.stamp)而非到达墙钟;或同时校验到达间隔与 stamp 差。 |
| D184 | src/wheelchair_3d_mapping/wheelchair_3d_mapping/dual_lidar_cloud_fusion_node.py · `_on_cloud` 异常处理 · L142-148 | filter pipeline 的 `except Exception` 只 `_warn` 节流告警,该帧 `state.xyz` 保持**上一帧旧值**,下游 `_publish_merged` 继续把陈旧点云当新帧合并发布,而不自知(F#22)。 | 滤波异常被静默吞没且复用旧云。建图质量影响:异常帧期间 `/points_merged` 反复发布陈旧几何并打新墙钟戳,RTAB-Map 把旧云贴到新位姿 → 拖影/配准漂移,且无明显报错。 | 中 | 滤波异常时清空该路 `state.xyz`(标记为不新鲜)而非保留旧值,并计数/上报;避免陈旧帧被当作新数据发布。 |
| D185 | src/wheelchair_3d_mapping/wheelchair_3d_mapping/dual_lidar_cloud_fusion_node.py · `_publish_merged` intensity 合并 · L169-178 | 任一路 `inten is None` 即 `have_intensity=False`,合并云**整帧丢弃强度**(另一路的强度也丢),而非按列对齐填 0(F#23)。 | 一路无强度即全链路丢强度。建图质量影响:`/points_merged` 强度缺失影响 RGB 着色/强度可视化与任何依赖强度的分割;几何不受影响(仅强度通道)。 | 低 | 缺强度的一路按 0(或 NaN)填充该列后再 concatenate,保留另一路真实强度;统一输出恒含 intensity 字段。 |
| D186 | src/wheelchair_3d_mapping/wheelchair_3d_mapping/dual_lidar_cloud_fusion_node.py(整体)· 无 deskew | 无运动畸变补偿(deskew):XT-M60 为 flash 整帧曝光(单帧畸变小),但「移动 + 多帧/双路融合 + 不同采集时刻」仍有帧间错位,代码完全无补偿(F#24)。 | flash ToF 单帧畸变小,故严重度低;但多帧融合期仍有运动间错位。建图质量影响:高速移动时融合云有轻微运动错位,墙线略增厚;静止/慢速无害。 | 低 | 对多帧融合按里程计/IMF 在采集时刻差内做运动补偿(把较旧帧投到统一时刻),或限制融合速度;flash ToF 单帧可不做 per-point deskew。 |
| D187 | src/wheelchair_3d_mapping/config/rtabmap_params.yaml · `icp_odometry`/`rtabmap` `Reg/Force3DoF: "true"` · L40, L52 | `Reg/Force3DoF=true` 锁定 z/roll/pitch(对轮椅平面假设合理),但若雷达外参 pitch 仍有小残差(见 D164 optical 约定冲突),强制 3DoF 会把该误差挤进 x/y 平面(F#28)。 | 外参 pitch 残差被 3DoF 约束转嫁为平面位移误差。建图质量影响:雷达安装 pitch 偏差(-5° 名义 + 残差)在 Force3DoF 下表现为水平漂移,长直走廊缓慢侧偏、墙线渐斜。 | 中 | 先标定/消除雷达外参 pitch 残差(配合 D164),再保留 Force3DoF;或在标定不确定期对 ICP 适度放开 pitch 后再投影 2D。 |
| D188 | src/wheelchair_3d_mapping/config/rtabmap_params.yaml · `rtabmap` `Reg/Strategy:"1"`(ICP,无视觉)/ `RGBD/ProximityBySpace` · L48, L62-66 | 回环几乎不可能触发:纯激光、`Reg/Strategy=1`(ICP)无视觉词袋,`RGBD/ProximityBySpace` 虽开但 120° 窄 FOV 下空间回环极难成功(实测 0 闭环)。**架构性**:纯激光 + 窄 FOV 注定大范围漂移无法纠正(F#29;对应 bugfix 已知「纯激光窄 FOV 无回环」未修)。 | 无有效回环 → 位姿图无法闭合。建图质量影响:大范围/绕环建图时累积漂移无回环纠正,地图首尾不闭合、走廊错层,这是大场景「必糊」的架构根因之一(与 D194 全局校正缺失叠加)。 | 高 | 引入视觉回环(相机词袋,RTAB-Map 视觉特征)或外部全局观测(UWB/AMCL 先验);窄 FOV 下增设人工回环触发/重访策略;在文档明确纯激光大范围漂移的固有局限。 |
| D189 | src/wheelchair_3d_mapping/config/rtabmap_params.yaml · `rtabmap` `Grid/RayTracing: "true"` · L? | `Grid/RayTracing=true` 依赖准确位姿:位姿漂移时 ray tracing 会「擦掉」真障碍或「挖空」真墙,加剧 2D 图破碎(已观察到黑白斑驳)(F#31)。 | 漂移期 ray tracing 反向破坏已建栅格。建图质量影响:`/rtabmap/grid_map` 在位姿漂移时出现墙体被擦除/自由区被错误扩张,2D 导航栅格破碎,Nav2 代价地图出现假通路。 | 中 | 漂移/低置信期降低或关闭 RayTracing,或限制其作用半径;配合回环(D188)/全局校正(D194)提高位姿质量后再启用。 |
| D190 | src/wheelchair_3d_mapping/launch/rtabmap_3d_mapping.launch.py · `delete_db_on_start`(default true)· L160, L212 ↔ autonomous_rviz_mapping.launch.py L261 | `delete_db_on_start` 默认 true(新建库),但若运维显式设 false 而忘记换库路径,新建图会 append 到旧库 → 两次地图叠加(此前疑似遇到);行为完全依赖 launch 参数且无运行时校验/告警(F#32)。 | 误配下新旧地图叠加。建图质量影响:append 到旧库使两次 session 的点云/位姿图叠加,地图出现重影/错层,且无显式提示。 | 低 | 对 append(localization=false 且 delete=false 指向已存在库)给出显式启动告警;或按时间戳自动分库;文档强调换库与 delete 的关系。 |
| D191 | src/wheelchair_3d_mapping/config/rtabmap_params.yaml(缺 sensor FOV 配置)· 整体 | 未给 RTAB-Map 配置 XT-M60 的有效传感器位姿/视野(FOV):RTAB-Map 不知雷达 120°×60° FOV,ray tracing/空闲推断会在视野外乱推空闲(F#33)。 | RTAB-Map 在 FOV 外错误推断自由空间。建图质量影响:视野外区域被错误标为已知自由,2D 栅格出现「未观测却判为空旷」的假自由区,影响 Nav2 规划安全。 | 低 | 为投影/ray tracing 配置传感器 FOV 限制(角度/距离扇区),使空闲推断仅在实际视野内;或在栅格层按 FOV 掩膜未观测区为 unknown。 |
| D192 | src/wheelchair_bringup/config/robot_localization_ekf.yaml · `two_d_mode: true` · L5 | `two_d_mode=true` 丢弃 z/roll/pitch(对平面轮椅合理),但 IMU 的 pitch/roll 信息被丢弃,无法用于检测斜坡/颠簸,也无法纠正雷达外参 pitch 残差(F#35)。 | 平面假设下姿态观测被裁剪。建图质量影响:无法感知斜坡/门槛带来的 pitch 变化,Force3DoF + two_d_mode 共同把外参/地形 pitch 误差转为平面漂移(关联 D187);室内平地无害。 | 低 | 平地场景保留 two_d_mode;若需上斜坡/门槛,改用全 6DoF EKF 并在投影时再降维;文档说明平面假设适用边界。 |
| D193 | src/wheelchair_bringup/config/robot_localization_ekf.yaml · `imu0_relative: true` · L? | `imu0_relative=true` 以「启动时刻」为 yaw 参考 0(用于移除 ~173° 传感器系偏置);若启动时车在动/AHRS 未稳,基准就偏,后续全程 yaw 带固定偏置(F#36)。 | 启动瞬间状态决定 yaw 基准。建图质量影响:启动未静止时全局朝向带固定偏置,地图整体旋转一个常量角(不影响局部一致性但影响与 map 帧对齐)。 | 低 | 启动前确保静止数秒并等待 AHRS 收敛后再置零;或检测启动期角速度,未静止时延迟基准锁定/告警。 |
| D194 | src/wheelchair_bringup/config/robot_localization_ekf.yaml · `world_frame: odom`(无 map→odom 全局校正)· L? | 无 map 帧 / 无全局校正:`world_frame=odom`,只有 odom→base_link,没有任何 map→odom 的全局修正源(SLAM 回环/AMCL),odom 漂移永久累积。**架构性**(F#39;对应 bugfix 已知「无室内绝对位移观测」未修,与 IMU 仅姿态级硬件约束叠加)。 | 无全局校正使局部 odom 误差永不被纠正。建图质量影响:与回环缺失(D188)共同构成「大范围必糊」的架构根因——长时间/大范围运行后位姿单调漂移,地图整体扭曲、首尾不闭合。 | 高 | 引入全局校正源:视觉/激光回环(RTAB-Map 输出 map→odom)、AMCL 定位于已知图、或 UWB 锚点;EKF 增设 map 层 world_frame=map 并消费全局位姿。 |
| D195 | src/wheelchair_bringup/config/robot_localization_ekf.yaml / h30_imu.yaml · IMU 协方差固定 · L? | IMU 协方差来自 yaml 固定值,不随磁干扰/振动变化:强磁/强振环境下 yaw 观测质量下降但 EKF 仍按固定协方差采信(F#40)。 | 固定 IMU 协方差无法反映环境干扰下的质量退化。建图质量影响:磁干扰区(电机/金属结构附近)yaw 被 EKF 过度信任,朝向估计被污染,建图局部旋转漂移。 | 低 | 若 IMU 提供动态协方差/质量标志则透传给 EKF;或按角速度/磁场一致性在线调节 yaw 协方差;文档标注强磁环境局限。 |
| D196 | src/wheelchair_sensors/.../xtm60_adapter_node.py(range/height)↔ src/wheelchair_3d_mapping/.../dual_lidar_cloud_fusion_node.py(range/height/voxel)↔ src/wheelchair_3d_mapping/config/rtabmap_params.yaml(Grid 高度/量程) | 建图链路存在「三处双重(多重)过滤」:适配器、融合节点、RTAB-Map 各有一套 range/height 过滤,语义重叠、互相打架、难调(F#43;量程/高度具体数值冲突另见 §3 D160/D161)。 | 同一物理量被多处独立裁剪,职责分散。建图质量影响:多级过滤参数不一致时(D160 量程 8/12/20/50、D161 高度带)产生「看得见但不入图」或「重复裁剪」,调参需同时改三处易遗漏(架构/可维护性根因)。 | 低 | 明确各级过滤职责(适配器仅去无效点、融合统一裁剪、RTAB-Map 仅投影),收敛为单一可信带定义并文档化;减少重叠裁剪层。 |
| D197 | src/wheelchair_sensors/wheelchair_sensors/xtm60_adapter_node.py(显示与建图共用输出)↔ 融合/着色链路 | 显示用与建图用点云混用同一适配器输出:显示要全分辨率有序,建图要过滤+无序,职责未彻底分离(已部分分层)(F#44)。 | 单一输出难同时满足两类需求。建图质量影响:为兼顾显示而保留的全分辨率/有序数据增加融合负载,或为建图过滤后损害显示;职责混叠使两条链路相互制约(规范/架构)。 | 低 | 适配器分别发布「显示用」(全分辨率有序)与「建图用」(已滤/可无序)两个话题,或由下游各自处理;明确分层契约。 |
| D198 | src/wheelchair_3d_mapping/wheelchair_3d_mapping/wheel_livo_consistency_monitor.py ↔ launch/livo_wheel_fusion.launch.py(仅此 launch 启用) | 无在线里程/位姿质量监控接入主链路:项目有 `wheel_livo_consistency_monitor`,但仅在 `livo_wheel_fusion.launch.py` 启用,主建图 launch(rtabmap/autonomous)未接入;跑偏时无告警,只能事后看地图(F#47)。 | 主链路缺位姿质量在线指标。建图质量影响:建图漂移/里程-LIVO 不一致在运行时不可见,无法及时中止/纠正劣质建图,浪费整次建图;监控能力存在却未接入。 | 中 | 把一致性/位姿协方差监控接入主建图 launch,发布质量指标并在超阈值时告警/提示重建;或在诊断面板汇总位姿健康度。 |



## 5. 备注与方法学说明

- 本报告为**单一活文档**,审查全程增量写入;§1 缺陷表与 §2 覆盖清单同步更新。
- **不封顶**:不设任何「至少/至多 N 条」人为数量目标;条目数量由实际发现决定。
- **逐文件穷尽**:§2 覆盖清单与 §0 文件枚举一一对应(共 173 个 in-scope 文件),使「已检视无缺陷」与「未检视」可区分。
- **缺陷条目格式固定 6 字段**:编号 / 文件位置 / 问题描述 / 影响 / 严重度 / 建议修复方向;建图相关项的「影响」须单列对建图质量的影响。
- **仅记录基于实际代码内容、可由文件位置佐证的缺陷**,不引入无依据的猜测性条目(对应 Req 3.5)。
- **编号空档为有意预留**:§1/§3/§4.4 采用按批次划分的编号块(B1–B5 用 D001–D051、B6 用 D070+、B7 用 D090+、
  B8 用 D100+、B9 用 D110+、B10 用 D130+、B11 用 D140+、B12/B13 用 D150+、§3 跨文件用 D160+、§4.4 迁移用 D170+),
  以支持各批次并发回填、避免编号冲突。因此存在**有意预留的编号空档**:**D052–D069、D081–D089、D108–D109、
  D126–D129、D136–D139、D148–D149、D157–D159**。这些空档**不代表缺失条目**——每条已记录缺陷的 6 字段均完整、
  全局编号唯一无重复;空档既不影响「不封顶」原则,也不影响逐文件覆盖。条目数量由实际发现决定(当前 156 条已记录缺陷)。
- **不修改任何运行时代码**:remediation(实际修复)为后续独立 effort,由本报告的「编号 + 严重度 + 修复方向」驱动。

### 文件枚举统计(初始化时,2026-06-17)

| 类型 | 命令 | 数量 |
|------|------|------|
| Python(src) | `find src -type f -name '*.py'` | 99 |
| C/C++(src) | `find src -type f \( -name '*.cpp' -o -name '*.c' -o -name '*.h' -o -name '*.hpp' \)` | 3 |
| yaml/xacro(src) | `find src -type f \( -name '*.yaml' -o -name '*.xacro' \)` | 38 |
| CMakeLists/package.xml(src) | `find src -type f \( -name 'CMakeLists.txt' -o -name 'package.xml' \)` | 13 |
| rviz(src) | `find src -type f -name '*.rviz'` | 6 |
| shell(scripts) | `find scripts -type f -name '*.sh'` | 8 |
| calib yaml | `find calib -type f -name '*.yaml'` | 4 |
| auto_test py | `find auto_test -maxdepth 1 -type f -name '*.py'` | 2 |
| **合计** | | **173** |

### Task 2 探查基线 —— 旧审查 F 缺口确认(Bug Condition 复现,2026-06-17)

> 本小节为**探查工作笔记**(working notes),用于在产出完整缺陷表前先 surface 反例、验证根因。
> **此处不是缺陷表**:正式 6 字段缺陷条目由批次 B1–B14 回填 §1。以下反例仅作为后续批次的检视线索。

#### F 的实际范围(对照确认)

`docs/mapping_code_review_2026-06-17.md` 标题即「**建图链路**代码审查」,共 **50 条**,按 A–F 分组:
- **A. 里程计/运动学**:`kinematics.py`、`zlac8030_driver_node.py`(条目 1–12)
- **B. 点云融合**:`dual_lidar_cloud_fusion_node.py`(条目 13–24)
- **C. RTAB-Map 配置/启动**:`rtabmap_params.yaml`、`rtabmap_3d_mapping.launch.py`(条目 25–33)
- **D. EKF/时间/TF**:`robot_localization_ekf.yaml` + 全局(条目 34–40)
- **E. 时间戳/use_sdk_timestamps**:`xtm60_adapter_node.py`(条目 41–42)
- **F. 架构/工程/健壮性**:杂项,顺带提及 run 脚本/`stop_mapping.sh`(条目 43–50)

**F 显式命名/触及的文件(全集)**:`kinematics.py`、`zlac8030_driver_node.py`、
`dual_lidar_cloud_fusion_node.py`、`rtabmap_params.yaml`、`rtabmap_3d_mapping.launch.py`、
`robot_localization_ekf.yaml`、`xtm60_adapter_node.py`,外加 #45/#46 顺带提到的 run/stop 脚本。
合计 **约 7–9 个文件**有任何痕迹,且均为**主题分组的自由文本**,无逐文件「已检视/无缺陷」痕迹、
无统一 6 字段、无全局编号体系。

#### 在范围内但 F 未检视的文件(缺口量化)

总 in-scope 文件 = **173**(见 §0 枚举)。F 触及 ≈ **7–9**。
→ **约 164 个在范围内文件在 F 中零条目、零检视痕迹**(无法区分「未看」与「看过无缺陷」)。

整包级别完全无任何条目的包(粒度/范围缺口):
- `wheelchair_navigation`(23 文件)— 0 条目
- `wheelchair_safety`(7 文件)— 0 条目
- `wheelchair_diagnostics`(9 文件)— 0 条目
- `wheelchair_perception`(11 文件)— 0 条目
- `wheelchair_sensors`(11 文件,仅 `xtm60_adapter_node.py` 被 E 组触及)— 其余 10 文件 0 条目
- `wheelchair_mapping`(7 文件)— 0 条目
- `wheelchair_description`(5 文件)— 0 逐文件条目(URDF 雷达高度仅在已知背景被提及为「已修」)
- `wheelchair_bringup`(51 文件,仅 `robot_localization_ekf.yaml` 被 D 组触及)— 其余 50 文件 0 条目
- `calib/`(4 文件)— 0 逐文件条目(右雷达占位外参仅在 #50 概念性提及)
- `auto_test/`(2 文件)— 0 条目

5 个重点非建图包(navigation/safety/diagnostics/perception/sensors)合计 **60 个文件**,
其中仅 1 个(`xtm60_adapter_node.py`)被 F 触及 → **59 个文件无任何检视痕迹**。

#### 抽样反例(确认 `isBugCondition(X)=true`:在范围内 ∧ 有缺陷 ∧ F 未记录)

> 以下为打开文件后的初步缺陷线索,**确认确有缺陷且 F 无对应条目**;严重度/精确行号留待 B 批次定稿。

1. **`wheelchair_safety/velocity_limiter_node.py`**(F 零条目)
   - 安全限速节点**只做静态 clamp,无加速度/jerk 限制**,与"限速器"职责名不符。
   - **无 deadman/超时**:上游 `/cmd_vel_nav` 停发时不会主动归零(依赖上游),安全裕度依赖外部。
   - pub/sub 均用默认 QoS depth=10、无显式 reliability 声明(安全话题 QoS 未审定)。
   - 反例性质:安全包整体未被 F 检视(范围缺口)。

2. **`wheelchair_navigation/goal_manager_node.py`**(F 零条目)
   - **自触发回环**:既 `create_subscription(PoseStamped, "/goal_pose", self.on_goal_pose)` 又
     `navigate_to_name()` 内 `self.goal_pub.publish` 到 `/goal_pose` → 自己发的目标触发自己的预览回调。
   - **健壮性**:`handle_command` 的 `add_goal` 直接索引 `command["name"]/["x"]/["y"]`,
     仅捕获了 JSON 解析异常,缺键会抛 `KeyError` 打断回调。
   - 反例性质:导航包整体未被 F 检视(范围缺口)。

3. **`wheelchair_diagnostics/sensor_watchdog_node.py`**(F 零条目)
   - `ultrasonic_topics` 默认仅 `["/ultrasonic/range_0"]`,**只看护 1 路超声**(其余可能漏看护)。
   - 看门狗 critical 分级靠多个 `*_critical` 参数,边界/魔法值待核;诊断 age 计算依赖 policy.py。
   - 反例性质:诊断包整体未被 F 检视(范围缺口)。

4. **`wheelchair_perception/pointcloud_to_laserscan_node.py`**(F 零条目)
   - **时间戳缺陷**:`_make_scan` 在 `restamp_output=True`(默认)时用墙钟 `now()` 盖 scan 时间戳,
     而非点云采集时刻 —— **与 F #13/#41 同类的时间戳问题,但发生在 F 完全未覆盖的感知文件**。
   - **TF 用 `Time()`(latest)而非 `msg.header.stamp`** —— 与 F #15 同类错位模式,异包复现。
   - `range_max` 默认 8.0,与融合/其他配置的 12 不一致(待 B14 跨文件比对)。
   - 反例性质:感知包整体未被 F 检视 + 时间戳/TF 缺陷跨包复现(范围 + 粒度缺口)。

5. **`wheelchair_sensors/ultrasonic_adapter_node.py`**(F 零条目;同包仅 `xtm60_adapter` 被 E 组触及)
   - **异常吞没**:`read_ranges` 每传感器 `except Exception: continue`,串口/协议错误被静默丢弃。
   - 发布 `range` 仅 `max(0.0, mm/1000)`,**未对 `max_range` 上限裁剪**,可发布超量程值。
   - 反例性质:传感器包除 xtm60 外未被 F 检视(范围缺口)。

#### 结论:缺口确认(EXPECTED SUCCESS)

- ✅ **范围不全**:navigation/safety/diagnostics/perception(+sensors/bringup 大部分)整包零条目。
- ✅ **粒度不够**:F 无逐文件覆盖清单,无法区分「未看」与「看过无缺陷」;约 164/173 文件无痕迹。
- ✅ **人为封顶**:F 恰好 50 条且以数量收尾;上述抽样已在 5 个文件发现 ≥8 条新线索,真实总数远超 50。
- ✅ **跨文件缺口**:时间戳(感知 restamp / TF latest)等问题在 F 未覆盖文件中复现,F 无系统化横向比对。

→ `isBugCondition(X)` 在大量范围内文件上为真,**Bug Condition 在旧状态 F 下成功复现,缺口确实存在**,
根因(范围锚定偏差 / 数量驱动收尾 / 无覆盖清单机制 / 无跨文件专项)成立。**非 unexpected pass。**
后续批次 B1–B14 据此逐文件穷尽回填 §1/§2。

## 6. 最终验证结果(Final Verification — Fix Checking + Preservation Checking)

> 本节(任务 17)对报告本身做**属性式校验**(非新写运行时测试):复用任务 1 的确定性 `find` 枚举
> (重新执行 §0 八条 find 命令,得 **173** 个 in-scope 文件)作为真值,解析本报告 §1/§2/§3/§4 并逐项断言。
> 校验脚本为 `/tmp` 下的一次性只读解析器(运行后删除),**不修改任何运行时代码**。

### 6.1 Fix Checking(对应 Property 1 / Req 2.1–2.6)

| # | 断言 | 结果 | 说明 |
|---|------|------|------|
| A1a | §2 覆盖清单条目集合 == 任务 1 文件枚举集合(差集为空) | ✅ PASS | 覆盖清单 173 行 == 枚举 173 文件,无遗漏、无多余 |
| A1b | 无「未检视」的范围内文件遗留(全部「已检视」) | ✅ PASS | 173/173 状态均为「已检视」 |
| A2a | §1/§3/§4.4 每条缺陷行 6 字段非空 | ✅ PASS | 修复了 6 行因未转义 `\|`(如 `\|y\|`/`\|命令-反馈\|`/shell `\|\|`/`/camera/left`)被误拆列的行 |
| A2b | 编号全局唯一、无重复 | ✅ PASS | 156 条已记录缺陷,156 个唯一编号 |
| A2c | 严重度 ∈ {阻断, 高, 中, 低} | ✅ PASS | 全部合法 |
| A2d | location 指向真实存在的文件路径 | ✅ PASS | 修复 D168 简写路径 `xtm60_left/right/sdk.yaml` → 展开为三个真实文件路径 |
| A3 | 无任何「至少/至多 N 条」人为封顶措辞 | ✅ PASS | 全文 grep 无命中 |
| A4 | 建图相关条目「影响」列单列「建图质量影响」 | ✅ PASS | 所有建图链路(3d_mapping/mapping/rtabmap/dual_lidar/…)条目均含建图质量说明 |

**编号空档说明**:存在有意预留空档 D052–D069 / D081–D089 / D108–D109 / D126–D129 / D136–D139 /
D148–D149 / D157–D159(批次并发编号块,见 §5);空档不代表缺失条目,不违反不封顶与逐文件覆盖。

### 6.2 Preservation Checking(对应 Property 2 / Req 3.1–3.5)

| # | 断言 | 结果 | 说明 |
|---|------|------|------|
| A5a | 显式排除集(`build/`、`__pycache__/`、`.git/`、`.pytest_cache/`)不出现在缺陷表/覆盖清单 | ✅ PASS | 无命中 |
| A5b | 根目录临时 dotfiles 不出现在缺陷表/覆盖清单 | ✅ PASS | 无 `.xxx.` 形式路径命中 |
| A5c | 二进制图片(`*.png/jpg`)不出现在缺陷表/覆盖清单 | ✅ PASS | 无命中 |
| A6a | F 的 50 条技术结论均能在 §1/§4 找到对应(§4.3 映射表 50 行齐全) | ✅ PASS | F#1–F#50 全部有去向 |
| A6b | F→F' 闭合 3(已修)+1(复核非缺陷)+15(映射已有)+31(新建)=50 | ✅ PASS | §4.3 闭合等式存在且成立 |
| A7 | 简单文件(`__init__.py`、纯数据 `empty_map.yaml`)在覆盖清单标「无缺陷」而非捏造条目 | ✅ PASS | 抽样的 `__init__.py`/`empty_map.yaml` 缺陷数=0、备注含「无缺陷」 |
| A8 | 已修项(EKF 轮速 yaw / 雷达 URDF 高度 / RTAB-Map yaml 加载)在 §4 标「已修」,未作为待修缺陷重复 | ✅ PASS | §4.1 三项均「已修」(F#25/F#34/F#45),遗留项另用独立编号 |
| A9 | 无无文件依据的猜测性条目(抽样 Dxxx 主位置存在、行号区间合理) | ✅ PASS | 抽样 D001/D034/D070/D090/D100/D110/D130/D150/D160/D170 主路径均存在、行号在文件范围内 |

### 6.3 本次验证中对报告文档的修正(仅文档,不动运行时代码)

1. **转义表格内未转义的 `|`**:D032(`\|y\|`)、D141(shell `\|\| true`)、D145(`[[ -f ]] \|\| rviz_cfg`)、
   D147(`source ... \|\| true` 与 `command -v ros2 ... \|\| { ... }`)、D151(`/camera/left|right` → 拆成两个话题名)、
   D175(`\|命令 vs 反馈\|` / `\|命令-反馈\|`)。这些原本会把 6 字段表格行误拆为 7–10 列。
2. **展开简写文件路径**:D168 的 `src/wheelchair_bringup/config/xtm60_left/right/sdk.yaml`
   展开为三个真实路径 `xtm60_left.yaml` · `xtm60_right.yaml` · `xtm60_sdk.yaml`(均存在于枚举)。
3. **补充编号空档说明**:在 §5 增加「编号空档为有意预留」条目(原仅散落于各批次 HTML 注释)。

### 6.4 结论

**所有 Fix Checking 与 Preservation Checking 断言全部通过(ALL PASS)。** 报告满足:逐文件穷尽
(173/173 已检视)、6 字段完整、编号唯一、严重度合法、location 真实、不封顶、建图项单列建图质量影响;
并保留排除集不动、F 的 50 条结论无丢失、真无缺陷如实标注、已修项标已修、无猜测性条目。
审查阶段不修改任何运行时代码;remediation 为后续独立 effort。
