# 正式功能源码

按传感器、标定、采集、融合、地图、界面和任务生命周期归属现有功能包；只有独立职责确实不能容纳时才新增包。

## 内容与入口

- [wc_bringup](wc_bringup/README.md)：下一级职责说明。
- [wc_calibration](wc_calibration/README.md)：下一级职责说明。
- [wc_camera_panel](wc_camera_panel/README.md)：下一级职责说明。
- [wc_cameras](wc_cameras/README.md)：下一级职责说明。
- [wc_estimation](wc_estimation/README.md)：下一级职责说明。
- [wc_fusion](wc_fusion/README.md)：下一级职责说明。
- [wc_imu](wc_imu/README.md)：下一级职责说明。
- [wc_interfaces](wc_interfaces/README.md)：下一级职责说明。
- [wc_maps](wc_maps/README.md)：下一级职责说明。
- [wc_motion](wc_motion/README.md)：下一级职责说明。
- [wc_panel](wc_panel/README.md)：下一级职责说明。
- [wc_runtime](wc_runtime/README.md)：下一级职责说明。
- [wc_sensors](wc_sensors/README.md)：下一级职责说明。
- [wc_slam](wc_slam/README.md)：下一级职责说明。
- [wc_xt_driver](wc_xt_driver/README.md)：下一级职责说明。

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../README.md)；返回[上级目录](../README.md)。
