# wheelchair_safety

安全监督包。

`safety_supervisor_node` 订阅 `/cmd_vel_nav`、`/scan`、`/ultrasonic/*/range`、`/emergency_stop_hw`、`/emergency_stop_sw`、`/system_stop_required`、`/passability/status` 和可选 `/localization/is_healthy`，发布 `/cmd_vel_safe` 和 `/safety_state`。

动态停车距离：

```text
d_safe = v * t_delay + v^2 / (2 * a_brake) + d_margin
```

状态至少包括 `CLEAR`、`WARNING`、`SLOWDOWN`、`STOP`、`EMERGENCY_STOP`、`SENSOR_FAULT`。默认自动导航最大速度为 `0.4 m/s`，默认禁止自动倒车。真实载人测试前必须重新标定制动距离、控制延迟、定位健康策略和超声波可靠性。
