# 原生相机显示

相机解码、原生显示控件与界面资源放这里；设备采集逻辑放 wc_cameras。

## 内容与入口

- [include](include/README.md)：下一级职责说明。
- [src](src/README.md)：下一级职责说明。
- [CMakeLists.txt](CMakeLists.txt)
- [package.xml](package.xml)
- [plugins_description.xml](plugins_description.xml)

## 添加与验证

新增功能优先扩展现有模块；新增独立子目录时同时创建 README 并登记父级导航。运行状态、原始录包、临时任务和测试结果不得混入此目录。

修改后运行[对应功能测试](../../tests/README.md)；涉及入口、配置、资源或路径时同时检查安装后的调用与旧数据读取。

依赖与构建方法见[项目入口](../../README.md)；返回[上级目录](../README.md)。
