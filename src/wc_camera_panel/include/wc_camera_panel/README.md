# 原生相机显示

相机解码、原生显示控件与界面资源放这里；设备采集逻辑放 wc_cameras。

## 内容与入口

- [camera_panel.hpp](camera_panel.hpp)
- [decoder.hpp](decoder.hpp)
- [frame_store.hpp](frame_store.hpp)
- [image_canvas.hpp](image_canvas.hpp)

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../../../../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../../../../README.md)；返回[上级目录](../README.md)。

## 目录职责与新增内容

相机解码、原生显示控件与界面资源放这里；设备采集逻辑放 wc_cameras。

新增文件须同步说明用途，并补充所属功能测试。返回[上级目录](../README.md)。
