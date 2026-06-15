#!/usr/bin/env python3
"""Enable the drive and write the SAME register values the ROS driver writes for
a 0.15 m/s forward command, then read back actual rpm. Determines whether the
driver's scaled register value actually turns the wheels. Off-ground only.

Driver math: wheel_rpm = linear/(2*pi*r)*60 ; reg = wheel_rpm * rpm_to_register_scale
with r=0.165, rpm_to_register_scale=8.9, invert_left=true, invert_right=false.
"""
import math
import time
from test_zlac8030d_motor import (
    Modbus, SLAVE_ID, REG_CONTROL_MODE, REG_CONTROL_WORD, REG_ASYNC_MODE,
    REG_TARGET_SPEED_LEFT, MODE_VELOCITY, CONTROL_CLEAR_FAULT, CONTROL_ENABLE,
    CONTROL_STOP, read_snapshot, print_snapshot,
)

linear = 0.15
r = 0.165
scale = 8.9
wheel_rpm = linear / (2 * math.pi * r) * 60.0
reg = int(round(wheel_rpm * scale))
# invert_left=true -> left negated; invert_right=false
left = -reg
right = reg
print(f"linear={linear} wheel_rpm={wheel_rpm:.2f} reg={reg} -> left={left} right={right}")

bus = Modbus("/dev/ttyACM0", 115200, 0.25)
try:
    bus.write_single(SLAVE_ID, REG_CONTROL_WORD, CONTROL_STOP)
    bus.write_single(SLAVE_ID, REG_CONTROL_MODE, MODE_VELOCITY)
    bus.write_single(SLAVE_ID, REG_ASYNC_MODE, 0)
    bus.write_single(SLAVE_ID, REG_CONTROL_WORD, CONTROL_CLEAR_FAULT)
    time.sleep(0.2)
    bus.write_single(SLAVE_ID, REG_CONTROL_WORD, CONTROL_ENABLE)
    time.sleep(0.2)
    print_snapshot("after_enable", read_snapshot(bus))
    bus.write_multiple(SLAVE_ID, REG_TARGET_SPEED_LEFT, [left, right])
    for i in range(5):
        time.sleep(0.3)
        print_snapshot(f"t{i}", read_snapshot(bus))
finally:
    bus.write_multiple(SLAVE_ID, REG_TARGET_SPEED_LEFT, [0, 0])
    bus.write_single(SLAVE_ID, REG_CONTROL_WORD, CONTROL_STOP)
    bus.close()
