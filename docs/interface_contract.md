# 第一阶段实际接口契约 v2（2026-09-12）

执行环境：ubuntu / nvidia / aarch64，Ubuntu 22.04、ROS 2 Humble、已安装 RTAB-Map ROS 0.23.7。工程唯一根目录 `/home/nvidia/wheelchair`。源码以 `src/wc_interfaces/msg` 和 `srv` 为可编译类型定义，本文件解释它们的约束。

## 原始流与共同估计

左右 `SourceFrame` 分别发布到 `/wc_mapping/lidar_left/source_frame`、`/wc_mapping/lidar_right/source_frame`。消息身份是 `(session_id, sensor_id, stream_epoch, frame_sequence)`；时间戳不是唯一键。原始设备秒/纳秒、主机接收时间、主机单调时间与共同采样时间分开存储。没有证据时 `time_source=arrival_only`、`common_time_valid=false`、`uncertainty_valid=false`。厂商 CAR 坐标轴按手册和矩阵代码是传感器自身 FLU，距离单位米；这不代表安装外参已标定。

`FusionBundle` 保留左右原始键、各源时间、外参/时钟版本、原始观测原点、运动变换、贡献点数和逐点 source_codes。一次配对只使用各源一帧，每帧不复用。共同输入为 `/wc_mapping/fusion/points`，只有一个 `rtabmap_odom/icp_odometry` 估计运动。`bundle_id` 在会话内严格递增。桥接器使用精确整数纳秒联结 `Odometry` 与安装版本的 `OdomInfo`，同时通过质量门控后才发布 `AcceptedBundle` 到 `/wc_mapping/accepted`。

正式双源模式缺任一流、时间/身份跳变、超出误差预算、退化/无效跟踪均锁存暂停。原始记录可独立继续。恢复服务 `/wc_mapping/resume_frontend` 要求五个新鲜、不同、严格递增的合格原始帧、相同配置版本、位置连续性及明确确认，禁止自动单源降级。

## 图、坐标与地图

变换 `T_A_B` 把 B 中坐标变换到 A，矩阵行优先、米、弧度，ROS 四元数 xyzw。`rig_link` 定义在左雷达 FLU 原点，右雷达变换未知时为 null，不能默认单位变换。`odom -> rig_link` 始终来自原生 ICP 位姿：无轮预测模式由 ICP 发布 TF；轮预测模式关闭原生 TF，由融合节点唯一转发通过门控的原生位姿。`map -> odom` 只由图节点发布；地图显示节点不广播 TF。

## v2：配置滤波与编码器预测

用户已授权直接使用左右各自 xtcfg，不将滤波效果对比作为启用前提。原 `source_frame/points_raw` 保留新增主机滤波前的回波；新 `source_frame_filtered/points_filtered` 是主机处理后的表示。两分支 session/side/sensor/epoch/sequence 和全部采集时间一致，配置哈希按表示区分。滤波分支必须提供原配置哈希与原云 data 的 SHA256；有界联结器按身份、采集字段和内容引用配对。缺原始分支不得把滤波分支当原始射线。

ICP 的 source_codes、FusionBundle retained_count 和 TrackingAssessment solver_input_count 描述处理后求解输入。地图归档的 points、valid_indices、raw_count、valid_count 则描述滤波前回波和其有效索引，二者数量允许不同。各源原点及运动补偿仍分别保留。图优化后的占据射线不使用平滑、填补产生的虚拟回波。

当前全部 Filters 项及支持的成像设置有对应 API。`Setting.pclFilterOn` 的 Windows 处理器没有已核实 Linux 对应，实现显式记录 PARTIAL_XTCFG；不阻止采集，不声称完整上位机输出一致。`host_filter_and_dispatch_elapsed_ns` 仅描述接收后主机处理至回调的耗时，不是曝光到输出延迟，也不是同步误差。

编码器输入为 `/wc_mapping/wheel/feedback_raw` 的 `wc_wheel_feedback_v1` JSON，只解码已有被动反馈；ROS 节点不打开控制器、不发请求。实际协议、比例、轮径、轮距、方向、时间和方差未配置时保持 raw_only。有效输出 `/wc_mapping/wheel/odom` 是 wheel_odom 中的 axle_link 位姿，另附精确时间匹配的 diagnostics，不发布 TF。寄存器候选及暂定比例不能自动成为生产配置。

正式实机配置要求启用编码器预测。已核实 `T_rig_axle` 和共同时间后，轮姿态同时用于双帧时间补偿与原生 ICP 初值。原生 ROS 0.23.7-humble 使用 guess_frame_id=wheel_odom，所有 guess_min_* 为 0；轮预测只发布到私有 `/wc_mapping/icp_guess_tf` 的 wheel_odom→rig_link。ICP 的 /tf 与 /tf_static 重映射到私有话题，publish_tf=false。公共 odom→rig_link 由适配器转发原生 ICP 位姿，绝不转发轮姿态。实时观察原生 OdomInfo.guess 才能证明初值确实进入求解；这不是带协方差的轮速/ICP 联合残差优化。

轮预测与该帧一起冻结，先完成原始归档，再发送私有 TF 与同时间融合点云；原生结果按唯一在途时间匹配。轮反馈失效、重启、时间跳变或队列异常暂停。当前轮预测模式恢复要求新会话。发布者检查在隔离域中核对唯一原生节点及可用 GID；Humble 若不提供逐消息 GID，不声称具备该项鉴权证据。

H30 重力分析及人工对应点求初值均输出 CANDIDATE/UNVALIDATED，不发布未知 TF。IMU X 朝前只约束一个方向，仍缺绕 X 安装角及位置；左侧 IMU 单独不能测出右侧雷达的独立倾斜。

RTAB-Map 图节点直接使用已接受 `bundle_id` 作为显式 SensorData ID，不依赖浮点时间戳反查。每个节点保存原始左右键、`T_node_sensor`、原始归档 SHA256、odom epoch 与精确时间。`GraphSnapshot` 发布在 `/wc_mapping/graph/snapshot`，携带实际优化位姿与 RTAB-Map 约束类型。合成来源明确标为 synthetic，不能视作实机闭环。

正式图链路使用应用处理确认：最多保存 8 个不可变 `AcceptedBundle`，只发送队首一个，直到原生 `GraphSnapshot` 出现相同节点 ID、原始双源键和 odom epoch，且会话、来源、标定、时钟和 graph epoch 均一致。未确认时每 0.5 秒原样重发，5 秒未确认或容量用尽锁存失败，禁止跳过中间观测继续建图。DDS reliable 仅表示传输可靠，不能替代此确认。后端对完整消息保留固定大小指纹；相同 ID 和内容重发当前快照确认，不重复写图、推进 revision 或发布 TF；相同 ID 内容变化锁存失败。只有明确的 ICP-only 诊断入口关闭图确认，该模式不能作为双雷达建图交付。输入暂停后可继续确认先前已接受的队列，失败后的图传递要求新会话。

地图使用同一图 revision 的优化位姿和各观测真实原点重建。3D 为点云加稀疏占据/射线证据 JSON；2D 只有地面证据和完整碰撞高度层覆盖才置为空闲，未观测保持 -1。后台最多一个工作、一个可替换待处理 revision 和一个完整结果。持续输入时允许发布已完整重建的较早 revision，并明确显示 MAP_BEHIND_GRAPH；已发布地图 revision 不倒退，只有追上图 revision 才声明 consistent_snapshot。

`/wc_mapping/frontend/status` 是前端状态；`/wc_mapping/status` 是地图协调器状态。`graph_revision` 与 `completed_map_revision` 分开表达重建中状态。地图 cloud/grid/path/status 使用 reliable、transient_local、depth 1；原始点云使用 sensor_data QoS。内部融合点云与 ICP 结果使用 reliable、depth 8，安装版 ICP 的 qos=1。H30 发布 `/wc_mapping/imu/source_frame`、`data_raw`、`diagnostics`，不发布 TF、不输入电机控制。

融合原始归档由容量最多 8 的单线程 FIFO 写盘队列保存，只有文件和目录持久化确认成功后才能发布 ICP 输入。失败、积压或超龄都暂停接受。frontend_status.json 与 last_tracking.json 仅为原子更新的运行缓存，不是重启、位置连续性或地图恢复依据；两路径各保留一个待写最新状态，由独立有界写入线程处理，不进行周期性 fsync，不让临时状态写盘阻塞采集执行器。缓存失败及发布/提交各阶段耗时单独诊断，缓存成功从不授权 ICP 或地图接受。

归档待完成、归档已确认待 ICP、ICP 在途三阶段共用最多 8 个观测的容量限制。ICP 每次仅有一个在途输入，收到同时间戳的 Odometry 和 OdomInfo 并通过门控后才发布 FIFO 下一帧，避免 fsync 集中完成造成原生估计器内部繁忙丢帧。原有归档年龄和 ICP 结果超时不放宽；不向原生 ICP 重发同一观测，不跳过超龄帧。

## 保存、版本和运行边界

调用 `/wc_mapping/close_graph_snapshot` 暂停图接受、关闭唯一数据库写入者并生成 `graph/closed_snapshot.json`。离线组装器验证图、数据库、原始归档哈希和完整索引，再重建同 revision 的快照。快照、图、标定、原始观测中的来源/会话/epoch 必须一致，禁止只改顶层标签把合成图导入为真实图。地图版本只新建，原子无覆盖提交，SQLite 采用只读 backup。目标点只是绑定地图版本的人工注释，不发送导航动作。

`scripts/wc_phase1` 为入口。成功返回 0；运行组件失败、无效配置、未验证实机门控返回非零；停止仅作用于该入口创建并记录 PID/startticks 的进程。`SDKs/`、`tests/`、`reports/` 分开管理。真实传感器、重型构建、地图写入都有独立锁。ROS 使用 domain 83 与 localhost 限制，不能与已有图或控制栈合并。

工程不读取或运行 `~/smartwheel-rtab-old`。不修改系统网络/SSH/时钟/防火墙/固件，不发送任何电机或导航命令。现场安全、导航、驱动急停与载人安全不在本工程通过声明内。
