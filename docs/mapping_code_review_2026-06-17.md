# 建图链路代码审查 — 2026-06-17

聚焦"建图"相关代码(里程计/运动学、点云融合、时间同步、TF、RTAB-Map 配置、
节点架构)。按严重度分组。每条:**问题 / 位置 / 影响 / 建议**。

> 说明:有些条目是"真 bug",有些是"算法不严谨/隐患",有些是"工程缺陷"。
> 已用 ⛔(会明显损害建图) / ⚠️(隐患/边界条件) / 🔸(健壮性/规范) 标注。

---

## A. 里程计 / 运动学(kinematics.py, zlac8030_driver_node.py)

1. ⛔ **里程计 dt 用墙钟、与点云时间戳不同源**:`tick()` 用 `get_clock().now()` 算 dt
   并积分,而点云用 SDK/到达时间。两条时间线不对齐 → EKF/RTAB-Map 做时间同步时
   位姿和点云错配,运动时尤其明显。建议统一时间基准。

2. ⛔ **轮速反馈读失败时"静默丢一帧运动"**:`_read_feedback()` 失败返回 None,
   且 feedback_enabled=True 时 actual 保持 0 → 该周期里程当成"没动",积分丢失。
   高速移动时反馈偶发失败 = 里程计欠计。建议失败时用上一帧速度保持(zero-order hold)。

3. ⛔ **里程积分用"当前周期速度",反馈本身有一拍延迟**:命令写下去→读回反馈→积分,
   反馈是上一控制周期的实际值,和本周期 dt 不严格对应,引入系统性距离/角度偏差。

4. ⚠️ **积分用中点 yaw 近似(mid_yaw),但 yaw 更新用端点**:`integrate()` 平移用
   `mid_yaw`(二阶),但 `self.yaw` 用端点更新。匀速转弯时 x/y 与 yaw 的更新阶数
   不一致,长期转弯有累积偏差。建议统一用精确弧线模型(exact arc / Runge-Kutta)。

5. ⚠️ **轮速里程的 yaw 本质不可靠却仍发布 yaw**:`wheel_rpm_to_twist` 用
   `(right-left)/wheel_separation` 推 yaw,严重依赖 wheel_separation 且受打滑影响。
   (已在 EKF 层改为不融合轮速 yaw,但驱动仍发布它,易被其他节点误用。)

6. ⚠️ **wheel_separation_m 未标定**:0.58 是估计值。即便不用于 EKF yaw,
   twist_to_wheel_rpm(下发指令)仍用它,影响转弯指令的实际半径。

7. ⚠️ **odom 协方差是写死常数**:`_publish_odom` 里 yaw 协方差固定 0.10、x 固定 0.05,
   不随速度/打滑变化。EKF 据此加权,固定协方差让它无法在打滑时降低对轮速的信任。

8. 🔸 **轮速里程 yaw 协方差 0.10 偏小(过度自信)**:对一个不可靠的 yaw 源标 0.10
   (≈18°一倍标准差的 1/3)偏乐观,正是之前 EKF 被轮速 yaw 带偏的帮凶。

9. 🔸 **mock 模式把目标速度当实际速度**:`actual = target`,mock 里程"完美",
   容易掩盖真实问题,测试时给人"里程很准"的错觉。

10. ⚠️ **没有滑动/堵转检测**:轮子抱死/打滑(你实测右转抱死)时,反馈 rpm 与真实运动
    不符,里程仍照单全收。建议加 |命令 vs 反馈| 偏差检测,异常时降协方差。

11. 🔸 **invert 同时作用于指令和里程,符号耦合**:`_apply_direction` 与
    `_remove_direction` 完全相同;若将来左右电机不对称,无法分别处理指令/里程符号。

12. ⚠️ **后退(倒车)无专门处理**:窄视野前向雷达后退时点云匹配差(你已观察到),
    代码层无"后退时降低 ICP 信任/标记"机制,RTAB-Map 倒车时易跳变。

---

## B. 点云融合(dual_lidar_cloud_fusion_node.py)

13. ⛔ **输出点云时间戳用"当前墙钟"而非传感器采集时刻**:`_publish_merged` 里
    `header.stamp = now()`。但点云数据是几十毫秒前采的。RTAB-Map/EKF 按这个错误时间戳
    做同步 → 运动时点云被"贴"到错误的位姿上 → 地图拖影。**这是建图拖影的重要源头。**

14. ⛔ **左右两帧用不同采集时刻却合并成一帧**:左、右雷达异步到达,各自 stamp 不同,
    合并时只取一个 now()。两雷达若不同步,运动中合并会错位。建议按时间最近匹配 + 用
    较早/插值时间戳。

15. ⚠️ **TF 用 `rclpy.time.Time()`(最新可用)而非点云时间戳**:`_lookup` 用 latest TF,
    不是点云采集时刻的 TF。运动时 base_link 在变,用"最新 TF"变换"旧点云"=错位。
    应 `lookup_transform(target, source, msg.header.stamp)`。

16. ⚠️ **range 过滤在传感器系用欧氏范数,但传感器原点≠base_link**:range 在传感器系算
    没问题,但注释/语义需明确;且 0 点(0,0,0 占位/无效)范数为 0 会被 min_range 滤掉,
    依赖于此,脆弱。

17. ⚠️ **voxel_downsample 用 floor 量化 + 取"首个"代表点**:`np.unique(return_index)`
    取的是该体素里**第一个**点,不是质心,引入与扫描顺序相关的偏置;强度也不平均。
    建议取体素质心 + 强度均值。

18. ⚠️ **voxel 量化用全局坐标 floor,负坐标/大坐标精度不均**:`floor(xyz/leaf)` 在 odom
    远处坐标大时,int64 没问题,但跨象限(正负)边界量化不对称,边界处重复/丢点。

19. 🔸 **融合输出 height/width 退化为无序(height=1)**:适配器辛苦保留的有序网格,
    到融合这里被拍平成无序点云,下游(法线估计/地面分割)失去邻接信息。

20. ⚠️ **单雷达 fallback 判据 `left_ok != right_ok` 逻辑脆弱**:双雷达都新鲜时
    fallback_active=False 正确;但"都失效"时也=False,与"双雷达正常"无法区分,
    状态语义混淆。

21. 🔸 **`_fresh` 用 recv_time(墙钟到达时间)判新鲜**:网络抖动/积压时,到达时间≠采集
    时间,可能把旧帧当新鲜。

22. 🔸 **filter pipeline 异常被吞**:`except Exception` 只 warn,该帧 state.xyz 保持上一帧
    旧值,下游继续用陈旧点云而不自知。

23. 🔸 **intensity 缺失时整帧降级为无强度**:`have_intensity=False` 会让合并云丢掉另一路
    的强度。应按列对齐填 0 而非全丢。

24. ⚠️ **没有运动畸变补偿(deskew)**:虽然 XT-M60 是 flash 整帧曝光(畸变小),但移动 +
    多帧融合 + 不同采集时刻,仍有运动间错位,代码完全无补偿。

---

## C. RTAB-Map 配置与启动(rtabmap_params.yaml, rtabmap_3d_mapping.launch.py)

25. ⛔ **(已修)YAML 参数文件不被 rtabmap 节点加载**:之前所有 Grid/RGBD/Reg 调参都没生效,
    跑的是库默认值(MaxObstacleHeight=0 → 2D 全黑)。已改为 launch dict 显式注入。
    **遗留**:yaml 文件还在,容易让人误以为它生效,需删除或加显著注释。

26. ⚠️ **icp_odometry 与 EKF 二选一,但参数共用一个 cfg**:同一份 rtabmap_params.yaml
    既给 icp_odometry 又给 rtabmap,两个节点参数语义不同,混在一起难维护。

27. ⚠️ **approx_sync=true + 默认 queue 偏小**:`/points_merged`(~10Hz,墙钟时间戳)与
    `/odometry/filtered`(30Hz)近似同步,queue=10。时间戳本身就不准(见#13),
    approx_sync 进一步模糊匹配 → 运动时位姿-点云错配。

28. ⚠️ **Reg/Force3DoF=true 锁 z/roll/pitch,但点云有地面/天花板倾斜残差**:平面假设对
    轮椅合理,但若雷达外参 pitch 仍有小残差,强制 3DoF 会把误差挤进 x/y。

29. ⚠️ **回环几乎不可能触发(纯激光,0 words)**:`Reg/Strategy=1`(ICP),无视觉特征,
    `RGBD/ProximityBySpace` 虽开但窄 120° FOV 下空间回环极难成功(实测 0 闭环)。
    架构上,纯激光 + 窄 FOV 注定大范围漂移无法纠正。

30. ⚠️ **Grid/RangeMax=8 但融合 max_range=12**:两处量程不一致,栅格用 8m 投影、
    点云保留到 12m,边界行为不一致,易困惑。

31. 🔸 **Grid/RayTracing=true 依赖准确位姿**:位姿漂移时 ray tracing 反而会"擦掉"真障碍
    或"挖空"真墙,加剧 2D 图破碎(你看到的黑白斑驳)。

32. 🔸 **delete_db_on_start 行为依赖 launch 参数,易误续建**:若忘记,新建图会append 到旧库
    → 两次地图叠加(你之前疑似遇到过)。

33. 🔸 **没有为 XT-M60 设置点云的有效"传感器位姿/视野"给 RTAB-Map**:RTAB-Map 不知道
    雷达 FOV,ray tracing/空闲推断会在视野外乱推。

---

## D. EKF / 时间 / TF 链(robot_localization_ekf.yaml + 全局)

34. ⛔→✅ **(已修)EKF 让轮速绝对 yaw 主导、IMU 当配角** → 已改为 IMU 主导。本次最大修复。

35. ⚠️ **two_d_mode=true 丢弃 z/roll/pitch**:对平面 OK,但 IMU 的 pitch/roll 信息被
    丢弃,无法用于检测斜坡/颠簸,也无法纠正雷达外参残差。

36. ⚠️ **imu0_relative=true 以"启动时刻"为 0**:若启动时车在动/未稳,基准就偏。
    需保证启动时静止数秒。

37. ⚠️ **EKF frequency=30 但轮速 publish_rate=50、IMU 200**:重采样到 30 会丢高频 IMU
    信息,转弯快时欠采样。

38. 🔸 **sensor_timeout=0.2 偏大**:某传感器掉 0.2s 才超时,期间 EKF 用陈旧值外推。

39. ⚠️ **无 map 帧 / 无全局校正**:world_frame=odom,只有 odom→base_link。没有任何
    map→odom 的全局修正源(SLAM 回环/AMCL),所以 odom 漂移永久累积。这与 #29 共同
    构成"大范围必糊"的架构根因。

40. 🔸 **IMU 协方差来自 yaml 固定值**,不随磁干扰/振动变化。

---

## E. 时间戳 / use_sdk_timestamps(xtm60_adapter)

41. ⛔ **默认 use_sdk_timestamps=false → 点云时间戳=ROS 接收时刻**:雷达采集到 ROS 发布
    有可变延迟(网络/SDK 缓冲),时间戳带抖动。建图同步用它 → 运动拖影。建议用设备时间戳
    + 时钟对齐(代码里有该逻辑但默认关)。

42. ⚠️ **多个节点各自 now() 盖时间戳**:适配器、融合节点都用接收/发布时刻,误差层层叠加。

---

## F. 架构 / 工程 / 健壮性

43. 🔸 **建图链路有"双重过滤"且参数分散三处**:适配器(range/height)、融合(range/height/voxel)、
    RTAB-Map(Grid 高度/量程)各有一套,语义重叠、互相打架、难调。

44. 🔸 **显示用与建图用点云混用同一适配器输出**:显示要全分辨率有序,建图要过滤+无序,
    职责未分离(已部分分层,但不彻底)。

45. 🔸 **run 脚本用 wait -n + setsid 历史 bug、set -u 退出 bug**:已修,但反映启动编排脆弱,
    建议改用 ros2 launch 统一管理生命周期而非 shell 拼。

46. 🔸 **stop_mapping 靠 pkill 名字匹配**:健壮性差,改 launch 名/节点名就失效。

47. ⚠️ **没有任何里程计/位姿质量在线监控**:跑偏了也无告警,只能事后看地图。建议发布
    位姿协方差/一致性指标(项目里有 livo_wheel_consistency 但未接入主链路)。

48. 🔸 **save_mapping 的 2D 栅格依赖话题在跑时存,易超时漏存**(你已遇到 pgm 没生成)。
    建议改为从 db 离线重建栅格(rtabmap-export/reprocess)。

49. 🔸 **大量参数无单元测试覆盖运动学/积分**:kinematics 的积分、twist 转换无回归测试,
    符号/比例改动容易引入回归。

50. ⚠️ **right radar 外参为占位估计 + 胶带固定**:双雷达融合时右雷达位姿不准会直接污染
    /points_merged,当前无外参在线校准机制。

---

## 优先级总结(建图质量影响最大的前几条)

1. **时间戳一致性(#13/#15/#41)** — 运动拖影的最大工程性根因,且**可修**。
2. **回环 / 全局校正缺失(#29/#39)** — 大范围必糊的架构根因,需视觉回环。
3. **里程计反馈丢帧/延迟(#2/#3)** — 影响位姿连续性。
4. **voxel 取首点而非质心(#17)** — 影响点云质量(次要)。
5. EKF yaw 已修(#34),是本轮最大改善。

**下一步性价比最高:修时间戳一致性(让融合点云用传感器采集时刻 + 用对应时刻的 TF),
然后上视觉回环。** 其余多为健壮性/规范问题,可逐步清理。
