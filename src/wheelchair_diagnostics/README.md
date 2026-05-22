# wheelchair_diagnostics

Diagnostics and preflight checks for the indoor autonomous wheelchair.

Nodes:

- `hardware_self_check_node`: active preflight probes for serial, camera, XT-M60 and ZLAC8030 links.
- `sensor_watchdog_node`: runtime topic watchdog and navigation-stop gate.
- `localization_health_node`: AMCL/odom/scan freshness and covariance monitor.

Runtime outputs:

- `/hardware/status`
- `/hardware/self_check`
- `/system_stop_required`
- `/system_stop_reason`
- `/localization/health`
- `/localization/is_healthy`
- `/diagnostics`
