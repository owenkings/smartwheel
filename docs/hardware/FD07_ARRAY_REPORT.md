# FD07-34R Four-Sensor Detection Report

Date: 2026-07-22
Branch: `feature/mapping-v2-rviz-workbench`
Audited HEAD: `b50c187fc9fb9d36aeef59e1ef0c618707db1597`
Decision: **PASS WITH LIMITS — FOUR-ADDRESS MODBUS AND 2 HZ ROS RANGE PATH**

## Safety and protocol scope

- Port: `/dev/smartwheel_ultrasonic -> /dev/ttyUSB0` (`1a86:7523`, CH340).
- Protocol: Modbus RTU, `9600 8N1`, function `0x03` only.
- Addresses: `1, 2, 3, 4`.
- Register: actual value `0x0001`.
- No address, baud-rate, mode, threshold, or other register was written.
- No motor, ZLAC, LiDAR, IMU, or camera node was active during raw testing.

The vendor document requires an interval strictly greater than 100 ms for actual-value
register `0x0001`. The raw diagnostic enforced `0.11 s` between request start times.

The production adapter now uses the same request-start scheduling rule. Response time
counts toward the 110 ms interval; it no longer adds an unconditional 110 ms sleep
after each approximately 94 ms response. The scheduler also preserves the interval
between the last request of one polling cycle and the first request of the next cycle.

## Raw 30-second result

The test completed 60 cycles and 240 valid transactions:

| Address | Success | Distance range | Median | Mean response latency |
| --- | --- | --- | --- | --- |
| 1 | `60/60` | `704–1556 mm` | `708 mm` | `94.16 ms` |
| 2 | `60/60` | `513–544 mm` | `514 mm` | `94.22 ms` |
| 3 | `60/60` | `376–385 mm` | `376 mm` | `94.16 ms` |
| 4 | `60/60` | `253–254 mm` | `254 mm` | `94.11 ms` |

All response addresses, function codes, byte counts, and CRC values were valid. All
samples were within the documented `30–3000 mm` measuring range. The minimum measured
request-start interval was `110.138 ms`; no request echo or protocol error occurred.

These distances describe only the objects/floor/structure visible during this stationary
test. In particular, address 1 intermittently returned a farther value up to `1556 mm`.
No controlled target was placed, so this is not a range-accuracy, cone-angle, crosstalk,
or enclosure-reflection calibration.

A 20-second post-change raw repeat completed 40 cycles / 160 transactions with no
failure. The minimum request-start interval was `110.143 ms`; all four addresses,
function codes, byte counts, CRCs, and `30–3000 mm` bounds passed.

## ROS adapter result

Only `ultrasonic_adapter_node` was started with `ultrasonic_full.yaml`. All four topics
published valid `sensor_msgs/Range` messages for 15 seconds:

```text
/ultrasonic/range_0 -> ultrasonic_0_link -> address 1
/ultrasonic/range_1 -> ultrasonic_1_link -> address 2
/ultrasonic/range_2 -> ultrasonic_2_link -> address 3
/ultrasonic/range_3 -> ultrasonic_3_link -> address 4
```

The original adapter produced 21 messages per topic at about `1.406 Hz`, because it
added 110 ms after each approximately 94 ms response. After request-start scheduling
was implemented, a 15-second ROS repeat produced 30–31 messages per topic. Measured
rates were `1.99969–1.99985 Hz`, header timestamps were strictly increasing, all values
stayed within the declared `0.03–3.0 m` bounds, and the node exited with status 0.

## Remaining limitations

- The ROS `field_of_view` remains the historical `0.45 rad`. The vendor document lists
  the FD07-34R as the large-angle RS485 variant, approximately `80 deg ±10 deg`, but the
  exact full-angle/half-angle convention is not confirmed. The current ROS value must
  not be treated as calibrated geometry.
- Sensor-to-physical-position mapping for addresses 1–4 has not been independently
  verified by covering or targeting one sensor at a time.
- No controlled distance targets, blind-zone target, no-target condition, moving target,
  enclosure A/B test, or simultaneous-acoustic crosstalk test was performed.
- Passing this gate does not make the array suitable for a passenger safety stop.
- No serial dropout or CRC failure suggested an immediate supply problem in the
  20-second repeat. Voltage, current, power margin, and operation under full system
  load were not measured; this is not a power-adapter certification.

## Evidence

- `docs/hardware/evidence/FD07_ARRAY_B5.json`
- `docs/hardware/evidence/FD07_ARRAY_B5_ROS.json`
- `docs/hardware/evidence/FD07_ARRAY_POST_PACING_RAW.json`
- `docs/hardware/evidence/FD07_ARRAY_POST_PACING_ROS.json`
- `scripts/hardware/fd07_array_diagnostic.py`
- `scripts/hardware/fd07_ros_diagnostic.py`

The electrical/Modbus/ROS publication path for all four addresses is validated. Physical
role mapping and safety behaviour remain a separate controlled test stage.
