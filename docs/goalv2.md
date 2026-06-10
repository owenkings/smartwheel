你现在继续开发此项目，对应github的分支 feature/livo-wheel-3d-slam。

当前硬件临时状态发生变化：右侧 XT-M60 雷达返修，短期内不可用。因此现在必须先做一个“左雷达单雷达实验室自主建图 Demo”。后续右雷达返修回来后，仍然恢复双雷达方案（也就是说，目前整个项目要既能双雷达，也能单雷达使用）。

请不要推翻现有架构。当前主线仍然是：

- 左/右 XT-M60 点云 → /points_merged
- RTAB-Map 输出 /rtabmap/cloud_map 作为 3D 主地图
- /rtabmap/grid_map 作为 Nav2 / 探索用的 2D 投影
- H30 IMU + 轮速通过 robot_localization 输出 /odometry/filtered
- Nav2 / reactive explorer / frontier explorer 输出 /cmd_vel_nav
- safety_supervisor 裁剪为 /cmd_vel_safe
- base 只接收 /cmd_vel_safe

本轮目标不是完整产品化，也不是漂亮 3D 重建，而是尽快得到一个能在实验室演示的 demo：

“右雷达不可用，仅使用左 XT-M60 + H30 IMU + 轮速 + 4 个超声波 + 2 个摄像头，轮椅在极低速、有物理急停、有人员看护的条件下自主移动并生成 RTAB-Map 3D 点云地图，同时 RViz 显示 /rtabmap/cloud_map、/rtabmap/grid_map、/scan、/exploration/status、/cmd_vel_nav、/cmd_vel_safe、TF。”

一、先审查当前代码状态
====================

先检查这些文件：

src/wheelchair_3d_mapping/launch/autonomous_rviz_mapping.launch.py
src/wheelchair_3d_mapping/launch/rtabmap_3d_mapping.launch.py
src/wheelchair_3d_mapping/launch/dual_lidar_fusion.launch.py
src/wheelchair_3d_mapping/config/dual_lidar_fusion.yaml
src/wheelchair_3d_mapping/wheelchair_3d_mapping/dual_lidar_cloud_fusion_node.py
src/wheelchair_bringup/launch/sensors.launch.py
src/wheelchair_bringup/config/diagnostics.yaml
src/wheelchair_bringup/config/pointcloud_to_scan_left.yaml
src/wheelchair_bringup/config/pointcloud_to_scan_right.yaml
src/wheelchair_bringup/config/scan_merger.yaml
src/wheelchair_bringup/config/robot_localization_ekf.yaml
src/wheelchair_bringup/config/camera.yaml
src/wheelchair_navigation/wheelchair_navigation/reactive_explorer_node.py
src/wheelchair_navigation/wheelchair_navigation/frontier_explorer_node.py
scripts/check_autonomous_mapping_status.sh
scripts/run_autonomous_rviz_mapping.sh
scripts/stop_autonomous_mapping.sh

先在回复中总结当前阻塞点，尤其确认：

1. 当前 autonomous 模式是否在 enable_motion:=true 时禁止 single-lidar fallback；
2. sensor_watchdog 是否仍要求 /xtm60/right/points 为 critical；
3. check_autonomous_mapping_status.sh 是否仍硬要求 /xtm60/right/points 和 right_fresh=true；
4. scan_merger 是否可以在只有 /scan_left 时仍然发布 /scan；
5. reactive_explorer_node 是否可用于 first-pass autonomous mapping；
6. camera.yaml 是否仅作为记录/可视化，不作为 SLAM 的硬同步阻塞项。

二、新增硬件配置 profile：left_lidar_lab
====================

新增一个统一参数：

hardware_profile

可选值：

- dual_lidar，默认，原双雷达正式模式
- left_lidar_lab，当前右雷达返修期间的实验室 demo 模式
- right_lidar_lab，预留，不要求本轮完整实现

在 autonomous_rviz_mapping.launch.py 中加入 DeclareLaunchArgument：

hardware_profile，默认 dual_lidar。

当 hardware_profile:=left_lidar_lab 时：

1. sensors.launch.py 只启动左 XT-M60：
   enable_xtm60:=false
   enable_xtm60_left:=true
   enable_xtm60_right:=false
2. dual_lidar_fusion.launch.py 必须传：
   allow_single_lidar_fallback:=true
3. rtabmap_3d_mapping.launch.py 必须允许：
   allow_single_lidar_fallback:=true
4. 不允许把右雷达缺失作为 BLOCKED_TO_PLAN 或 BLOCKED_TO_ARM。
5. 必须在启动日志中明确打印：
   [left_lidar_lab] RIGHT XT-M60 is disabled / under repair. Single-left-lidar autonomous mapping demo mode.
6. max_linear_speed 默认降到 0.03 m/s 或 0.04 m/s。
7. max_angular_speed 默认不超过 0.18 rad/s。
8. 默认 exploration_mode 使用 reactive，而不是 frontier。
9. RViz 仍显示 /rtabmap/cloud_map、/rtabmap/grid_map、/scan_left、/scan、TF、/cmd_vel_nav、/cmd_vel_safe。

当 hardware_profile:=dual_lidar 时：
保持原行为，不破坏双雷达正式模式。

三、修复 rtabmap_3d_mapping.launch.py 的传感器启动参数
====================

当前 rtabmap_3d_mapping.launch.py 在 bringup_sensors=true 时，很可能硬编码启动左右雷达。

请新增参数：

enable_xtm60_left，默认 true
enable_xtm60_right，默认 true

并在 include sensors.launch.py 时使用这两个参数，而不是固定：
enable_xtm60_left=true
enable_xtm60_right=true

同时保留：
enable_imu=true
enable_ultrasonic=true
enable_camera 根据 subscribe_rgb/use_colorizer 决定

要求：

- left_lidar_lab 模式能只启动左雷达；
- dual_lidar 模式仍启动左右雷达；
- 不能启动已返修的右雷达节点导致 launch 等待或报错。

四、增加单左雷达扫描链路
====================

当前已有：
pointcloud_to_scan_left.yaml
pointcloud_to_scan_right.yaml
scan_merger.yaml

请新增或改造为 profile 化：

方案 A：新增配置文件，推荐：
src/wheelchair_bringup/config/scan_merger_left_only.yaml

内容：
input_topics:

  - /scan_left
    output_topic: /scan
    frame_id: base_link
    require_all_sources: false
    其他参数参考 scan_merger.yaml。

然后在 autonomous_rviz_mapping.launch.py 中：

- dual_lidar 模式启动 left + right pointcloud_to_laserscan，并使用 scan_merger.yaml；
- left_lidar_lab 模式只启动 pointcloud_to_laserscan_left_node，并使用 scan_merger_left_only.yaml；
- 不启动 pointcloud_to_laserscan_right_node。

要求：

1. 只有左雷达时，/scan 必须稳定发布；
2. /scan_left 和 /scan 都应在 RViz 可见；
3. 不允许因为 /scan_right 缺失导致 check 脚本失败。

五、增加单左雷达 diagnostics 配置
====================

新增：

src/wheelchair_bringup/config/diagnostics_left_lidar_lab.yaml

参考 diagnostics.yaml，但修改：

1. points_topics 只包含：
   /xtm60/left/points
2. points_0_critical: true
3. points_1_critical 不要要求，或者不要存在；
4. 四个超声波仍保持 critical：
   /ultrasonic/range_0..3
5. IMU critical；
6. wheel odom / base status critical；
7. 摄像头建议先不作为 hard critical，最多 WARN：
   当前 demo 的目标是自主建图，摄像头可用于记录/画面/上色，但不应阻塞 RTAB-Map LiDAR-primary 建图。
8. startup_grace_sec 可设为 8~12 秒，避免传感器刚启动误判。

在 autonomous_rviz_mapping.launch.py 中：

- hardware_profile=dual_lidar 使用 diagnostics.yaml；
- hardware_profile=left_lidar_lab 使用 diagnostics_left_lidar_lab.yaml。

同时 sensor_watchdog_node 的 OpaqueFunction 参数中不要再硬编码 points_1_critical=True；必须由 profile/config 决定。

六、改造 check 脚本，支持 --left-lidar-lab
====================

修改：

scripts/check_autonomous_mapping_status.sh

新增参数：
--left-lidar-lab

当传入 --left-lidar-lab 时：

READY_TO_PLAN 硬性检查：

- /xtm60/left/points
- /points_merged
- /points_merged/status
- /imu/data
- /ultrasonic/range_0
- /ultrasonic/range_1
- /ultrasonic/range_2
- /ultrasonic/range_3
- /wheel/odom
- /odometry/filtered
- /rtabmap/cloud_map
- /rtabmap/grid_map
- /scan_left
- /scan
- /safety_state
- /hardware/status
- /system_stop_required
- /base/status
- /cmd_vel_safe
- /exploration/status
- TF map->odom
- TF odom->base_link

不再硬性检查：

- /xtm60/right/points
- /scan_right
- right_fresh=true
- single_lidar_fallback=false

但需要检查 /points_merged/status 中：

- left_fresh=true
- right_fresh 可以是 false
- single_lidar_fallback 可以是 true
- output_points > 0

脚本输出应该区分：
READY_TO_PLAN_LEFT_LIDAR
READY_TO_ARM_LEFT_LIDAR
BLOCKED_TO_PLAN
BLOCKED_TO_ARM

如果用户没有传 --left-lidar-lab，则保持原双雷达检查逻辑。

七、新增一键运行脚本：run_left_lidar_lab_mapping_demo.sh
====================

新增：

scripts/run_left_lidar_lab_mapping_demo.sh

功能：

1. 进入仓库根目录；
2. source /opt/ros/humble/setup.bash 和 install/setup.bash；
3. 停止 smartwheel.service，避免重复节点；
4. 打印高风险警告：
   LEFT-LIDAR LAB AUTONOMOUS MAPPING DEMO
   RIGHT XT-M60 IS DISABLED / UNDER REPAIR
   MAX SPEED VERY LOW
   OFF-GROUND TEST FIRST
   PHYSICAL E-STOP REQUIRED
5. 要求用户输入确认字符串：
   I_UNDERSTAND_LEFT_LIDAR_LAB_MAPPING_RISK
6. 启动：

ros2 launch wheelchair_3d_mapping autonomous_rviz_mapping.launch.py \
  hardware_profile:=left_lidar_lab \
  enable_motion:=true \
  autonomous_exploration:=true \
  exploration_mode:=reactive \
  require_enable_signal:=true \
  max_linear_speed:=0.03 \
  max_angular_speed:=0.18 \
  turn_trigger_distance:=0.60 \
  stop_on_safety_warning:=false \
  use_colorizer:=false \
  rviz:=true

注意：

- use_colorizer 默认 false，避免摄像头不稳拖垮 demo；
- 摄像头可以启动，但不作为 mapping 主输入；
- reactive 模式默认不需要成熟地图，更适合 first-pass lab mapping；
- 仍然必须经过 safety_supervisor；
- 不允许直接发布到底盘绕过 /cmd_vel_safe。

八、新增预检脚本：check_left_lidar_lab_mapping_status.sh
====================

新增：

scripts/check_left_lidar_lab_mapping_status.sh

它可以直接调用：

scripts/check_autonomous_mapping_status.sh --left-lidar-lab

并额外输出当前建议动作：

如果 READY_TO_PLAN_LEFT_LIDAR 但未 READY_TO_ARM：
提示：mapping stack is up, but autonomous motion is not armed.

如果 READY_TO_ARM_LEFT_LIDAR：
提示：
Now publish:
ros2 topic pub --once /autonomy/enable std_msgs/msg/Bool "{data: true}"

如果 BLOCKED：
列出缺失话题。

九、修改 RViz 配置
====================

新增或修改：

src/wheelchair_3d_mapping/rviz/left_lidar_lab_mapping.rviz

显示：

1. TF
2. /xtm60/left/points
3. /points_merged
4. /rtabmap/cloud_map
5. /rtabmap/grid_map
6. /scan_left
7. /scan
8. /odometry/filtered
9. /cmd_vel_nav，若 RViz 可显示
10. /cmd_vel_safe，若 RViz 可显示
11. /exploration/status 或 Marker，如果有
12. /camera/left/image_raw 和 /camera/right/image_raw 可选显示，不作为核心

视角：

- Fixed Frame = map
- 默认 TopDownOrtho
- 点云颜色使用 AxisColor / Intensity
- 点大小 0.02~0.05

在 hardware_profile=left_lidar_lab 时，autonomous_rviz_mapping.launch.py 应加载这个 RViz 配置；dual_lidar 仍可用 autonomous_3d_mapping.rviz。

十、增加文档
====================

新增：

docs/left_lidar_lab_mapping_demo.md

内容必须包括：

1. 为什么有这个模式：
   右 XT-M60 返修，临时用左 XT-M60 做实验室自主建图 demo。

2. 它能做什么：
   - 左雷达生成 /points_merged
   - RTAB-Map 生成 /rtabmap/cloud_map 和 /rtabmap/grid_map
   - H30 + 轮速 EKF 提供 /odometry/filtered
   - reactive explorer 低速自主移动
   - 4 个超声波用于近距离补盲
   - RViz 可视化建图过程

3. 它不能承诺什么：
   - 不是最终双雷达正式能力；
   - 单 120° FOV 雷达覆盖不足，存在盲区；
   - 不适合狭窄复杂环境；
   - 不建议无人值守；
   - 不建议载人测试；
   - 摄像头当前不进入几何 SLAM 硬同步链路。

4. 运行步骤：
   colcon build --symlink-install
   source install/setup.bash
   bash scripts/run_left_lidar_lab_mapping_demo.sh
   bash scripts/check_left_lidar_lab_mapping_status.sh
   ros2 topic pub --once /autonomy/enable std_msgs/msg/Bool "{data: true}"

5. 验收标准：
   - /xtm60/left/points 有数据；
   - /points_merged 有数据；
   - /points_merged/status 显示 left_fresh=true；
   - /scan_left 和 /scan 有数据；
   - /odometry/filtered 有数据；
   - /rtabmap/cloud_map 增长；
   - /rtabmap/grid_map 增长；
   - /exploration/status 出现 FORWARD/TURN 类状态；
   - /cmd_vel_nav 有极低速；
   - /cmd_vel_safe 有极低速或被安全层合理裁剪；
   - RViz 中能看到实验室轮廓逐步生成。

6. 急停：
   bash scripts/stop_autonomous_mapping.sh
   物理急停
   Ctrl+C launch
   并确认 /cmd_vel_safe = 0。

7. 恢复双雷达：
   右雷达返修回来后，使用 hardware_profile:=dual_lidar，重新检查双雷达外参、/points_merged/status、scan_merger 和 watchdog。

十一、测试要求
====================

完成后必须执行：

colcon build --symlink-install

python3 -m py_compile $(git ls-files '*.py')

bash -n scripts/*.sh || true

ros2 launch wheelchair_3d_mapping autonomous_rviz_mapping.launch.py --show-args

ros2 launch wheelchair_3d_mapping autonomous_rviz_mapping.launch.py \
  hardware_profile:=left_lidar_lab \
  enable_motion:=false \
  autonomous_exploration:=false \
  rviz:=false \
  --show-args

ros2 launch wheelchair_3d_mapping autonomous_rviz_mapping.launch.py \
  hardware_profile:=left_lidar_lab \
  enable_motion:=true \
  autonomous_exploration:=true \
  exploration_mode:=reactive \
  rviz:=false \
  --show-args

如果新增任何 launch，也执行 --show-args。

不要提交：

- .bag
- .db
- .ply
- .pcd
- .log
- 大文件
- 临时地图
- token

十二、提交要求
====================

最终回复必须包含：

1. 当前右雷达缺失时，哪些地方原本会阻塞；
2. 新增了哪些文件；
3. 修改了哪些文件；
4. left_lidar_lab 模式如何启动；
5. 如何检查 READY_TO_PLAN_LEFT_LIDAR / READY_TO_ARM_LEFT_LIDAR；
6. 如何发布 /autonomy/enable；
7. RViz 中应该看到哪些话题；
8. 如何确认 /rtabmap/cloud_map 正在增长；
9. 如何确认 /cmd_vel_nav 与 /cmd_vel_safe 正常；
10. 如何急停；
11. 右雷达回来后如何恢复 dual_lidar 模式。
