# 标定文档导航

本目录提供标定主题的入口，连接几何依据、配置契约与实际操作步骤。

- [几何依据](../reference/geometry.md)：坐标系、安装关系和变换约定。
- [标定配置](../reference/calibration_configuration.md)：配置字段、有效状态和来源证据。
- [点云配准](../operations/offline_cloud_alignment.md)：输入准备、选点和配准操作。
- [反射强度配准](../operations/amplitude_alignment.md)：对应输入与配准入口。
- [标定实现](../../src/wc_calibration/README.md)与[标定测试](../../tests/calibration/README.md)。

新增标定接口说明放入 reference，用户步骤放入 operations，并更新此处的主题导航；可复用实现放入 wc_calibration。测量原件保留在配置来源容器或数据根，任务过程资料进入开发归档，不在此复制原始录包或阶段报告。

本目录没有可执行程序。文档中的入口依赖目标平台和对应输入；修改后运行目录链接检查及上述标定测试，涉及几何时使用同一冻结输入核对变换和数值。返回[文档导航](../README.md)。
