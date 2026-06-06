# Codex Goal: SmartWheel 本地 first-release Web 2D 用户地图完善

在 Codex 中使用：

```text
/goal Follow docs/goal.md. Use only the local project files as the source of truth. Re-check the current branch and code before editing. Improve the existing SmartWheel first-release Web 2D user-map demo without bypassing safety.
```

## 0. 执行范围

只基于当前本地仓库 `/home/nvidia/smartwheel` 开发。不要把 GitHub 页面、issue、PR 描述或 README 当作主要事实来源；README 可以参考，但必须以本地 `src/`、`scripts/`、`docs/`、`launch/`、`config/`、`test/` 中的实际代码为准。

本阶段不是重写 SLAM，不是接入 3DGS/GMSL，不是做完整商品级 UI。目标是在当前分支已有架构上，把现有 Web UI/RosBridge/安全链路补齐成一个**初版可发布、可演示、可操作、可自检**的简陋版本。

必须优先复用现有代码：

1. Web 后端：`src/wheelchair_ui/wheelchair_ui/app.py`，当前已使用 FastAPI/uvicorn。
2. Web 静态资源：`src/wheelchair_ui/wheelchair_ui/static/index.html`、`app.js`、`style.css`、`gui_widgets.js`。
3. ROS 状态桥：`src/wheelchair_ui/wheelchair_ui/ros_bridge.py`。
4. 建图管理：`src/wheelchair_ui/wheelchair_ui/mapping_manager.py`。
5. Native GUI：`src/wheelchair_ui/wheelchair_ui/native_gui.py`。

不要新增第二套 Web 状态系统。除非有明确技术理由，不要新增 `src/wheelchair_ui/web/` 或 `web_user_map_server.py`，因为本地项目已经有可运行的 FastAPI Web UI。

## 1. 分支与本地事实门禁

开始任何代码修改前先执行：

```bash
pwd
git status --short
git branch --show-current
git log --oneline -5
```

当前目标分支必须是：

```text
feature/livo-wheel-3d-slam
```

如果不是该分支，停止并提示用户切换分支，不要继续实现。

然后审查本地代码，不依赖 README：

```bash
rg -n "rtabmap_3d_mapping|rgb_cloud_colorizer|enabled_cameras|ultrasonic_indices|sensor_addresses|range_0|range_1|range_2|range_3" src scripts
rg -n "goal_pose|named_goal_command|NavigateToPose|send_goal_pose|set_initial_pose" src
rg -n "semantic_map|semantic-map|upsert_room|no_go_zone|no-go-zones|NamedGoalStore" src
rg -n "cmd_vel_nav|cmd_vel_safe|/cmd_vel|safety_state|emergency_stop_sw" src scripts
rg -n "wheelchair_ui|FastAPI|uvicorn|static|8080|native_gui" src scripts docs
rg -n "autonomous_rviz_mapping|reactive_explorer|frontier_explorer" src scripts
```

先在回复里总结当前事实，至少包括：

1. 当前分支和最新 commit hash。
2. 当前 3D 建图输入 topic。
3. 当前 3D 主地图 topic。
4. 当前 2D 导航投影 topic。
5. 当前 Web UI 是否已经存在、入口是什么。
6. 当前 Native GUI / RosBridge 已经有哪些可复用能力。
7. 当前相机启用哪几路。
8. 当前 4 个超声波对应哪些 topic 和物理方位。
9. 当前导航/运动链路如何经过 Nav2 与 safety_supervisor。
10. 当前是否存在 Web/RosBridge 直接发布 `/cmd_vel` 或 `/cmd_vel_safe` 的路径。

本文件基于当前本地审查得到的基线事实：

1. 以下基线事实仅供参考，必须以本次运行时重新审查的本地代码为准。不要因为 commit hash、分支名或旧记录与当前不一致而停止执行: 4路超声波避障 + 轮询测试脚本`。
2. `rtabmap_3d_mapping.launch.py` 明确以 `/points_merged` 作为 LiDAR-primary 输入。
3. RTAB-Map 输出 `/rtabmap/cloud_map` 作为 3D 主地图，输出 `/rtabmap/grid_map` 作为 2D 导航投影。
4. `/rtabmap/grid_map` 与 `/map` 都已被 `RosBridge` 订阅，但当前地图优先级需要整理为“优先 `/rtabmap/grid_map`，fallback `/map`”，并暴露来源。
5. `wheelchair_ui.app` 已经是 FastAPI Web 后端，静态页面在 `wheelchair_ui/static/`，入口为 `ros2 run wheelchair_ui wheelchair_ui --host 0.0.0.0 --port 8080`。
6. `full_system.launch.py` 与 `mapping.launch.py` 已能按参数启动 Web UI；当前还缺一个“只启动 Web UI”的专用 launch/script。
7. `RosBridge` 已有 `status()`、`map_snapshot()`、POI、语义地图、`send_named_goal()`、`send_goal_pose()`、`set_initial_pose()`、`set_software_stop()` 等能力。
8. Web API 已有 `/api/status`、`/api/map`、`/api/goals`、`/api/navigate/{name}`、`/api/stop`、`/api/resume`、`/api/semantic-map`、`/api/rooms`、`/api/no-go-zones` 等；但还缺 `/api/navigate`、`/api/navigate_named`、`/api/initial_pose`、`/api/release_stop` 和下划线路由兼容。
9. 当前 `/api/map` 在没有 ROS 地图时返回一张假空白地图，这不适合验收；应改成明确的“地图未就绪”JSON 状态，前端显示提示而不是伪装成有地图。
10. 当前 RosBridge 创建了 `/cmd_vel` 与 `/cmd_vel_safe` publisher，并通过 `/api/hardware/zero`、`request_hardware_shutdown()`、软急停路径发布零速。这是必须审计和收敛的安全重点：普通 Web 操作不得直接发速度。
11. `camera.yaml` 当前启用 `enabled_cameras: [left, right]`，对应 `/camera/left/image_raw` 与 `/camera/right/image_raw`。
12. `ultrasonic.yaml` 当前 4 个 RS485 超声波为 `/ultrasonic/range_0..3`：`range_0=left-front`、`range_1=left`、`range_2=right-front`、`range_3=right`。
13. `autonomous_rviz_mapping.launch.py` 当前默认 `exploration_mode:=reactive`，frontier 作为实验模式保留。

如果实际代码与以上基线不一致，以重新审查得到的本地代码为准，并在修改前说明差异。

## 2. 安全边界

这些规则不可破坏：

1. Web 前端不得直接发布 `/cmd_vel`。
2. Web 后端普通导航不得直接发布 `/cmd_vel` 或 `/cmd_vel_safe`。
3. 不得绕过 `safety_supervisor`。
4. 所有真实导航运动必须保持：

```text
/goal_pose 或 /named_goal_command 或 NavigateToPose
  -> Nav2
  -> /cmd_vel_nav
  -> safety_supervisor
  -> /cmd_vel_safe
  -> base driver
```

5. 默认不允许真实电机写入。`base.launch.py` 已有 `motion_control_enabled:=false` 安全默认；若其他上层 launch（例如 `full_system.launch.py`）把默认改成 true，必须作为 P0 安全缺口处理，改成显式 opt-in。
6. Web UI 不提供普通用户“零速直发”按钮。软急停可以通过 `/emergency_stop_sw`、`/emergency_stop_command` 触发；若保留零速发布，只能绑定在软急停/硬件关闭的安全路径内，并在代码中明确隔离。
7. Web UI 不默认启用真实运动，不启动电机，不修改底盘默认安全策略。
8. 不删除 Native GUI、RViz、RTAB-Map、Nav2、安全层、底盘驱动。
9. 不把 3D 点云作为普通用户主界面；Web 端以 2D 地图为主，只显示 3D 建图状态。
10. 不提交 rosbag、`.db`、`.ply`、`.pcd`、`.log`、大文件、token、本地运行产物。

本阶段必须专门审计：

```bash
rg -n "create_publisher\\(Twist|/cmd_vel|cmd_vel_safe|publish_zero_velocity|hardware/zero|hardware/shutdown" src/wheelchair_ui src scripts
```

验收时应能说明：Web 端没有普通导航直接控制电机的代码路径。

## 3. 第一阶段目标：补齐现有 Web 2D 用户地图

本阶段不是“从零新增 Web”，而是完善现有 FastAPI Web UI。

最终 Web 页面面向普通用户/导师演示，应像简陋扫地机器人 App 的 2D 地图界面：

1. 显示 2D 房屋地图，优先 `/rtabmap/grid_map`，fallback `/map`。
2. 没有地图时明确显示“地图未就绪”，页面和 API 不崩溃。
3. 显示轮椅当前位置箭头，优先 map 坐标系定位；无 map 定位时标明来源和可信度。
4. 显示当前目标点。
5. 显示 Nav2 路径线，来自当前实际存在的 `/navigation/preview_path`、`/plan`、`/global_plan` 或 `/received_global_plan`。
6. 显示 POI 标签，例如 charging、door、客厅、卧室、卫生间、厨房。
7. 支持房间/区域手动 polygon 标注，第一版不做自动房间分割。
8. 支持禁行区/虚拟墙手动 polygon 标注，第一版只显示和保存，不强制接入 costmap。
9. 显示 `safety_state`、导航状态、定位状态、地图状态。
10. 显示传感器在线状态：左 XT-M60、右 XT-M60、`/points_merged`、H30 IMU、4 个超声波、left/right camera、`/rgb_cloud_map`、`/wheel/odom`、`/base/status`。
11. 支持点击地图设置临时目标、保存 POI、设置初始位姿、绘制房间 polygon、绘制禁行区 polygon。
12. 支持软急停与解除软急停。

当前已有页面可以保留建图向导、通行性、局部鸟瞰、摄像头状态等模块，但普通用户主路径必须清晰：看地图、选点、保存 POI、导航到 POI、急停。

## 4. RosBridge 最小增量要求

继续使用 `src/wheelchair_ui/wheelchair_ui/ros_bridge.py`，不要另写独立 ROS bridge。

必须保留并复用：

1. `status()`
2. `map_snapshot()`
3. `list_goals()`
4. `add_goal()`
5. `delete_goal()`
6. `send_named_goal()`
7. `send_goal_pose()`
8. `set_initial_pose()`
9. `set_software_stop()`
10. `semantic_map()`
11. `save_semantic_map()`
12. `upsert_room()`
13. `delete_room()`
14. `upsert_no_go_zone()`
15. `delete_no_go_zone()`

需要补齐或修正：

1. 地图优先级：`/rtabmap/grid_map` 优先，`/map` fallback。不要让晚到的 `/map` 覆盖更新鲜的 `/rtabmap/grid_map`，除非 `/rtabmap/grid_map` 超时。
2. `map_snapshot()` 返回中加入 `ok`、`source_topic`、`age_sec`、`frame_id`、`width`、`height`、`resolution`、`origin`、`data`。地图未就绪时返回 `None` 或由 API 包装成 `{"ok": false, "reason": "map not ready"}`。
3. 传感器状态拆分左/右 XT-M60，不要只用一个 `laser` 概念覆盖 `/xtm60/left/points` 和 `/xtm60/right/points`。
4. `mapping_3d` 保持包含 `/points_merged`、`/rtabmap/cloud_map`、`/rtabmap/grid_map`、`/rgb_cloud_map`、`/rtabmap/odom`。
5. `status()` 必须包含 `safety_state`、`navigation_status`、`pose`、`current_goal`、`route_path`、`sensor_status`、`sensors`、`mapping_3d`、`map_available`、`initial_pose`、`hardware_status`、`localization_health`、`passability_status`。
6. `set_initial_pose()` 已存在，Web API 必须暴露。
7. `send_goal_pose()` 已存在，Web API 必须暴露临时目标发送。
8. 软急停保留，但普通 Web API 不保留独立“直接零速”能力。

## 5. Web API 要求

继续在 `src/wheelchair_ui/wheelchair_ui/app.py` 内扩展现有 FastAPI app。不要换成 `http.server`，因为本地项目已经依赖并使用 FastAPI/uvicorn。

所有 API 必须返回 JSON。POST body 解析失败时，返回明确错误。地图未就绪、定位未就绪、Nav2 action server 未就绪时，API 不得崩溃。

保留现有路由以免破坏当前页面，同时新增兼容路由：

```text
GET    /api/status
GET    /api/map
GET    /api/goals
POST   /api/goals
DELETE /api/goals/{name}
POST   /api/navigate
POST   /api/navigate_named
POST   /api/navigate/{name}        # 现有命名目标路由可保留
POST   /api/initial_pose
POST   /api/stop
POST   /api/release_stop
POST   /api/resume                 # 现有解除急停路由可保留
GET    /api/semantic_map
POST   /api/semantic_map
GET    /api/semantic-map           # 现有路由可保留
PUT    /api/semantic-map           # 现有路由可保留
POST   /api/rooms
DELETE /api/rooms/{name}
POST   /api/no_go_zones
DELETE /api/no_go_zones/{name}
POST   /api/no-go-zones            # 现有路由可保留
DELETE /api/no-go-zones/{name}     # 现有路由可保留
```

`GET /api/map`：

1. 有 `/rtabmap/grid_map` 或 `/map` 时返回真实 OccupancyGrid snapshot。
2. 没有地图时返回：

```json
{"ok": false, "reason": "map not ready"}
```

3. 不再返回伪造的 80x60 空白地图。

`POST /api/navigate` body：

```json
{"x": 1.2, "y": -0.8, "yaw": 0.0}
```

调用 `RosBridge.node.send_goal_pose()`。失败时返回错误原因，例如定位未就绪、地图未就绪、Nav2 action server 未就绪、目标不安全。

`POST /api/navigate_named` body：

```json
{"name": "living_room"}
```

调用 `send_named_goal()`。

`POST /api/initial_pose` body：

```json
{"x": 0.0, "y": 0.0, "yaw": 0.0}
```

调用 `set_initial_pose()`。

`POST /api/stop`：

触发软急停。不得作为普通速度控制。

`POST /api/release_stop`：

解除软急停，等价兼容现有 `/api/resume`。

## 6. Web 前端要求

继续修改现有静态文件：

```text
src/wheelchair_ui/wheelchair_ui/static/index.html
src/wheelchair_ui/wheelchair_ui/static/app.js
src/wheelchair_ui/wheelchair_ui/static/style.css
src/wheelchair_ui/wheelchair_ui/static/gui_widgets.js  # 仅必要时
```

页面结构至少包含：

1. 顶部状态栏：`SmartWheel 室内导航`、安全状态、导航状态、定位状态、地图状态、当前模式。
2. 左侧主地图：Canvas 2D 地图、轮椅箭头、当前目标、路径线、POI 标签、房间 polygon、禁行区 polygon。
3. 右侧控制面板：传感器状态、操作按钮、POI 列表、语义编辑。

地图绘制规则：

1. unknown cell：浅灰或透明。
2. free cell：白色。
3. occupied cell：蓝色边界或深蓝。
4. path：绿色线。
5. goal：绿色圆点或旗帜。
6. wheelchair pose：带朝向箭头。
7. no-go zone：红色半透明。
8. room polygon：浅色半透明，并显示房间名。
9. safety CLEAR/SAFE：绿色。
10. safety WARNING/SLOWDOWN：黄色。
11. safety STOP/EMERGENCY：红色。

当前前端已有地图绘制、POI 保存、语义地图文本输入、传感器卡片、建图向导。需要补齐：

1. 地图未就绪时显示明确空状态，不把假地图当真地图。
2. 点击地图后能选择操作模式：临时目标、保存 POI、设置初始位姿、绘制房间、绘制禁行区。
3. 新增“发送点击目标”按钮，调用 `POST /api/navigate`。
4. 新增“设置点击点为初始位姿”按钮，调用 `POST /api/initial_pose`。
5. POI “导航”可以继续用 `/api/navigate/{name}`，也可切到 `/api/navigate_named`。
6. 房间/禁行区第一版必须支持点击绘制 polygon；可保留 textarea 作为工程输入。
7. 传感器列表要明确显示左 XT-M60、右 XT-M60、`/points_merged`、H30 IMU、U0..U3、left/right camera、`/rgb_cloud_map`、wheel odom、base。
8. 移除或隐藏普通用户“零速”按钮。软急停和解除急停保留。
9. “关闭硬件”这类工程按钮默认不放在普通用户主界面；如保留，必须清楚标识并确认不会绕过安全策略。

第一版不要求漂亮，但必须清晰、可用、不卡死，移动端/桌面端文本不能互相覆盖。

## 7. 坐标转换工具与测试

当前前端 JS 已有 `worldToCanvas()` 和 `canvasToMap()`，后端 `ros_bridge.py` 也有 `map_point_to_cell()`。为防止地图点击、路径绘制、边界判断出错，新增纯函数模块：

```text
src/wheelchair_ui/wheelchair_ui/map_canvas_utils.py
```

实现：

```python
map_to_canvas(x, y, map_info, canvas_width, canvas_height, scale, offset_x, offset_y)
canvas_to_map(px, py, map_info, canvas_width, canvas_height, scale, offset_x, offset_y)
map_point_in_bounds(x, y, map_info)
path_to_canvas_polyline(points, map_info, canvas_width, canvas_height, scale, offset_x, offset_y)
```

必须正确处理：

1. `OccupancyGrid.info.resolution`
2. `OccupancyGrid.info.width`
3. `OccupancyGrid.info.height`
4. `OccupancyGrid.info.origin.position.x`
5. `OccupancyGrid.info.origin.position.y`
6. ROS map 坐标 y 轴与 Canvas y 轴方向相反。
7. 点击点落在地图边界外时能判断 out-of-map。

新增测试放在本包现有测试目录：

```text
src/wheelchair_ui/test/test_map_canvas_utils.py
```

测试至少包括：

1. origin=(0,0), resolution=1 时，map(0,0) 与 canvas/cell 一致。
2. origin=(-5,-5), resolution=0.05 时，点转换后再反算误差小于半个 cell。
3. y 轴翻转正确。
4. 点击地图边界外能判断 out-of-map。
5. route path 多点能正确转换成 canvas polyline。

单元测试必须在无 ROS 硬件环境下运行。

## 8. Launch 与脚本

当前已有入口：

```bash
ros2 run wheelchair_ui wheelchair_ui --host 0.0.0.0 --port 8080
ros2 launch wheelchair_bringup full_system.launch.py enable_web_ui:=true
ros2 launch wheelchair_bringup mapping.launch.py enable_ui:=true
bash scripts/run_real_sensors.sh
```

本阶段新增一个只启动 Web UI 的专用 launch，包装现有 `wheelchair_ui` executable：

```text
src/wheelchair_ui/launch/web_user_map.launch.py
```

参数：

```text
host:=0.0.0.0
port:=8080
named_goals_path:=默认 named_goals.yaml
semantic_map_path:=默认 semantic_map.yaml
enabled_cameras:="left,right"
ultrasonic_indices:="0,1,2,3"
```

如果 `app.py` / `RosBridge` 当前不能接收 `enabled_cameras` 和 `ultrasonic_indices`，做最小增量扩展，不要通过独立状态绕开。

新增脚本：

```text
scripts/run_web_user_map.sh
```

功能：

1. cd 到仓库根目录。
2. source `/opt/ros/humble/setup.bash`。
3. source `install/setup.bash`。
4. 启动 `ros2 launch wheelchair_ui web_user_map.launch.py` 或直接 `ros2 run wheelchair_ui wheelchair_ui`。
5. 打印：

```text
http://localhost:8080
http://<orin-ip>:8080
This script starts the Web UI only. It does not enable motor motion.
```

新增或扩展自检脚本：

```text
scripts/check_first_release_ui.sh
```

检查：

1. `/rtabmap/grid_map` 或 `/map`
2. `/rtabmap/cloud_map`
3. `/points_merged`
4. `/rgb_cloud_map`，缺失 WARN，不 BLOCKED
5. `/scan`
6. `/imu/data`
7. `/ultrasonic/range_0..3`
8. `/camera/left/image_raw`
9. `/camera/right/image_raw`
10. `/camera/left/camera_info`，缺失 WARN
11. `/camera/right/camera_info`，缺失 WARN
12. `/wheel/odom`
13. `/base/status`
14. `/safety_state`
15. `/cmd_vel_safe`
16. `/plan` 或 `/global_plan` 或 `/navigation/preview_path`，缺失 WARN
17. `/navigate_to_pose` action server

输出分级：

```text
READY_FOR_WEB_UI
READY_FOR_NAV_DEMO
BLOCKED: <reason>
WARN: <reason>
```

规则：

1. 缺地图：Web UI 仍可启动，但地图区显示“地图未就绪”。
2. 缺 camera_info：只影响 RGB 点云准确性，不阻止 Web UI。
3. 缺 `/rgb_cloud_map`：只影响上色点云状态，不阻止 Web UI。
4. 缺 `/safety_state`：阻止导航演示。
5. 缺 `/navigate_to_pose`：阻止导航演示。
6. 缺 `/cmd_vel_safe`：阻止运动演示。
7. Web UI 脚本永远不主动启动电机。

## 9. 文档要求

新增或更新：

```text
docs/first_release_plan.md
docs/web_2d_user_map.md
docs/ui_architecture.md
```

必须写清楚：

1. 本阶段目标是“简陋可发布版本”，不是最终产品。
2. 后端继续使用 RTAB-Map 3D SLAM。
3. `/rtabmap/cloud_map` 是 3D 主地图，用于算法、调试、展示。
4. `/rtabmap/grid_map` 是 2D 导航投影，也是 Web 2D 用户地图基础。
5. Web 端是用户/导师演示主界面，基于现有 FastAPI `wheelchair_ui.app`。
6. Native GUI 是工程调试端，保留并共享 RosBridge/POI/semantic map。
7. RViz 是工程 3D 可视化端。
8. 当前只启用 left/right 两个前向 USB 摄像头。
9. 当前 4 个超声波全部用于近距离安全和状态显示。
10. 摄像头当前用于状态显示和 `/rgb_cloud_map` 上色，默认不进入 RTAB-Map 主几何闭环。
11. 3D 重建/3DGS/mesh 不放弃，但放到后续 P2/P3 离线展示层，不参与本阶段实时导航。
12. Web 前端不直接发速度。
13. 所有真实运动必须经过 Nav2 和 safety_supervisor。
14. 如何运行：只开 Web UI、开 3D 建图、开导航演示、开自动探索演示。
15. 如何自检：`check_first_release_ui.sh`、`ros2 topic list`、`ros2 topic echo`、浏览器测试项。

不要把普通用户引导到 RViz 黑色点云作为主体验。

## 10. 测试与自检

完成代码修改后必须运行能在当前环境运行的检查：

```bash
git status --short
git diff --stat
colcon build --symlink-install
python3 -m py_compile $(git ls-files '*.py')
bash -n scripts/*.sh
colcon test --event-handlers console_direct+ || true
ros2 launch wheelchair_ui web_user_map.launch.py --show-args
ros2 launch wheelchair_3d_mapping rtabmap_3d_mapping.launch.py --show-args
```

如果环境没有 ROS、没有硬件、没有依赖，必须明确说明哪些检查无法执行，不能假装通过。

新增坐标转换测试必须能在无硬件环境下跑。

## 11. 人工验收标准

Web UI 第一版验收：

1. 浏览器打开 `http://localhost:8080` 不报错。
2. 没有地图时显示“地图未就绪”，不是空白、假地图或崩溃。
3. 有 `/rtabmap/grid_map` 或 `/map` 时显示 2D 栅格地图。
4. 有定位时显示轮椅箭头，并标明定位来源。
5. 有路径时显示路径线。
6. 能显示当前 `safety_state`。
7. 能显示左/右雷达、`/points_merged`、IMU、4 个超声波、左/右摄像头、`/rgb_cloud_map`、轮速、底盘状态。
8. 能添加 POI。
9. 能删除 POI。
10. 刷新页面后 POI 仍存在。
11. 能点击地图发送临时目标。
12. 能点击 POI 发送命名目标。
13. 能设置初始位姿。
14. 能触发软急停。
15. 能解除软急停。
16. 能点击绘制并保存房间 polygon。
17. 能点击绘制并保存禁行区 polygon。
18. Web 普通导航没有任何直接发布 `/cmd_vel` 的代码路径。
19. Web 普通导航没有绕过 `/cmd_vel_nav -> safety_supervisor -> /cmd_vel_safe` 的路径。
20. 代码中没有提交 rosbag、PLY、PCD、DB、log、大文件。
21. 不修改默认电机安全策略为自动运动。

## 12. 允许修改文件范围

允许按需修改：

```text
src/wheelchair_ui/wheelchair_ui/app.py
src/wheelchair_ui/wheelchair_ui/ros_bridge.py
src/wheelchair_ui/wheelchair_ui/mapping_manager.py
src/wheelchair_ui/wheelchair_ui/map_canvas_utils.py
src/wheelchair_ui/wheelchair_ui/static/index.html
src/wheelchair_ui/wheelchair_ui/static/app.js
src/wheelchair_ui/wheelchair_ui/static/style.css
src/wheelchair_ui/wheelchair_ui/static/gui_widgets.js
src/wheelchair_ui/launch/web_user_map.launch.py
src/wheelchair_ui/setup.py
src/wheelchair_ui/package.xml
scripts/run_web_user_map.sh
scripts/check_first_release_ui.sh
docs/first_release_plan.md
docs/web_2d_user_map.md
docs/ui_architecture.md
src/wheelchair_ui/test/test_map_canvas_utils.py
必要的 launch/config 小修改
```

不要新增重复的 `src/wheelchair_ui/web/` 静态目录，除非先说明为什么现有 `wheelchair_ui/static/` 无法满足需求。

不要删除 Native GUI、RViz、RTAB-Map、Nav2、安全层、底盘驱动。

## 13. 提交要求

提交前执行：

```bash
git status --short
git diff --stat
```

不要提交：

```text
*.bag
*.db
*.ply
*.pcd
*.log
__pycache__
.ros/log
大文件
token
本地绝对路径下的运行产物
```

建议提交信息：

```text
feat(ui): harden first-release web 2D user map
```

如果用户没有要求提交，不要自行提交。

## 14. 最终回复要求

最终回复必须包含：

1. 当前分支和最新 commit hash。
2. 修改文件列表。
3. 每个文件的作用。
4. 如何运行 Web UI。
5. 如何运行自检脚本。
6. Web UI 显示哪些内容。
7. 如何验证坐标转换正确。
8. 如何验证点击地图发送目标。
9. 如何验证 POI 保存。
10. 如何验证软急停。
11. 明确说明 Web 端普通导航没有直接控制电机。
12. 明确说明摄像头当前是否进入主 SLAM；若没有，说明当前用途和后续升级路径。
13. 明确说明哪些功能仍未完成：自动房间分割、自动重定位、回充、3DGS/mesh 展示、GMSL 相机接入等。
14. 哪些内容必须在 AGX Orin 实机上验证。
