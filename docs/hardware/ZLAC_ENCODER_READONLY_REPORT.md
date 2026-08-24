# ZLAC8030 Encoder Read-only Detection Report

Date: 2026-07-22
Branch: `feature/mapping-v2-rviz-workbench`
Audited HEAD: `b50c187fc9fb9d36aeef59e1ef0c618707db1597`
Decision: **NOT TESTABLE — USB BRIDGE PRESENT, NO CONTROLLER RESPONSE**

## Safety boundary

The user excluded motor testing. This diagnostic therefore did not start the normal
`zlac8030_driver_node` and did not issue drive enable, speed, stop, emergency-stop, or
any other Modbus write. The only transmitted function code was holding-register read
`0x03`.

The normal base node was deliberately avoided because its shutdown path can write the
configured control-word emergency-stop value even when `motion_control_enabled=false`.
That write may be appropriate during an actual motor session, but it is outside this
read-only detection scope.

## Enumerated transport

```text
/dev/smartwheel_zlac8030 -> /dev/ttyACM1
USB VID:PID: 1a86:55d3
USB serial: 5C66036979
configured RTU: 115200 baud, slave 1
feedback registers: left 0x20AB, right 0x20AC
```

The USB serial bridge is present and can be opened. No other process held the port.

## Result

An initial project-client probe produced deterministic CRC errors. Raw transaction
capture showed that the bytes were not a controller reply: the adapter returned the
exact eight-byte request frame.

The final echo-aware run used 50 attempts over 10 seconds. Every transaction returned
only this local echo:

```text
request:  01 03 20 AB 00 01 FE 2A
received: 01 03 20 AB 00 01 FE 2A
```

After removing the exact echo there were zero device-response bytes, zero valid left
feedback values, and zero valid right feedback values. No query reached the right
register because the left-register request never received a device response.

This is consistent with the motor/controller side being disconnected or unpowered, as
the user reported for the motor stage. It is not evidence that either encoder itself is
faulty. Encoder direction, resolution, scale, rate, timestamps, and odometry remain
unvalidated.

## Evidence and next condition

- `docs/hardware/evidence/ZLAC_ENCODER_READONLY.json`
- `scripts/hardware/zlac_feedback_readonly_diagnostic.py`

Repeat only when the controller and encoder feedback path are intentionally powered and
the user authorizes a controller session. The first repeat should remain function-0x03
read-only; movement, enable writes, and speed writes require a separate motor-stage gate.

## Powered follow-up — 2026-07-25

The user reported that the motors were connected and then connected and powered the
controller adapter. This follow-up remained strictly zero-motion and read-only. It did
not start `zlac8030_driver_node` and did not send control-word, speed, enable, stop, or
emergency-stop writes.

### Transport identity

The expected adapter enumerated successfully:

```text
/dev/smartwheel_zlac8030 -> /dev/ttyACM1
VID:PID: 1a86:55d3
USB serial: 5C66036979
USB path: platform-3610000.usb-usb-0:1.1:1.0
```

No process owned the port and `smartwheel.service` remained inactive. The udev rule was
present and matched the device, so the failure is not a missing Linux alias or a port
collision.

### Read-only result

A 10-second run at the historical, manufacturer-default settings (`115200` baud, slave
1, holding registers `0x20AB/0x20AC`) made 50 attempts. All 50 received only the exact
eight-byte request echo and no controller response. A second saved 5-second run made 25
attempts with the same result:

```text
request:  01 03 20 AB 00 01 FE 2A
received: 01 03 20 AB 00 01 FE 2A
```

An additional bounded function-`0x03` scan covered baud rates `9600`, `19200`, `38400`,
`57600`, `115200`, and `128000`, with slave IDs 1 through 4. All 24 combinations returned
only their exact local request echo; there were zero Modbus responses. The scan range is
consistent with the manufacturer's published ZLAC8030D RS485 range of 9600–128000 baud
and includes its published default of 115200 baud. The older project branches also used
115200 baud, slave 1, and these same dual-axis feedback registers.

Evidence:

- `docs/hardware/evidence/ZLAC_ENCODER_POWERED_READONLY_20260725.json`

### Current decision

**BLOCKED AT PHYSICAL RS485/CONTROLLER LINK — ADAPTER PRESENT, CONTROLLER SILENT.**

The evidence rules out the expected Linux device alias, port ownership, the historical
baud/slave combination, and the bounded common baud/address combinations. It does not
distinguish among controller logic power, RS485 A/B polarity, missing communication
common ground, use of the CAN connector instead of RS485, connector/contact failure, or
another physical interface fault. No conclusion about either encoder can be made until
the controller returns a valid frame.

Before any write or motion test, physically verify controller power/status LEDs, the
24–48 VDC input using qualified procedures, the exact ZLAC8030D RS485 connector pins,
A/B polarity, communication common ground, and the address switches. Keep both drive
wheels off the ground and the hardware emergency stop ready for the later motion gate.

## User-confirmed retry — 2026-07-25

After the user stated that the controller connection was correct, the adapter identity,
stable alias, and lack of port ownership were confirmed again. A new six-attempt test
used the historical `115200` baud/slave-1/register configuration at only 1 Hz and widened
the response timeout from 0.2 seconds to 1.0 second. Every attempt still received only
the exact eight-byte local request echo and zero controller bytes. This rules out the
short diagnostic timeout as the cause.

Evidence:

- `docs/hardware/evidence/ZLAC_ENCODER_POWERED_RETRY_LONG_TIMEOUT_20260725.json`

No archived raw ZLAC response frame or wheel-speed capture was found in the older Git
branches; only code, configuration comments, and historical calibration claims were
present. Those comments are useful background but are not a replacement for a valid
response in the current powered setup.

An adjacent live USB observation must be retained as system-level risk evidence. While
the ZLAC adapter itself remained enumerated, the shared USB2 hub logged an EMI disable
and re-enumerated the H30 adapter. Camera path `2-3.1` then disconnected, re-enumerated,
reported UVC `-110/-71` errors, and reset at SuperSpeed; at the final check its USB
device was visible but the video driver/path had not recovered. All four cameras must be
requalified after the controller/power/grounding work. These events do not prove that
the motor controller caused the USB faults, but they prohibit treating the full powered
system as stable.

## Expansion-hub retry — 2026-07-26

SSH and the complete USB topology were rechecked before testing. All four cameras were
again present at `5000M`. The ZLAC adapter had re-enumerated as
`/dev/smartwheel_zlac8030 -> /dev/ttyACM0`, but its stable USB identity was unchanged
(`1a86:55d3`, serial `5C66036979`). Its path was still
`platform-3610000.usb-usb-0:1.1:1.0`, under the external Genesys Logic hub. Therefore
this run was **not** a direct-Orin-port A/B test and cannot determine whether bypassing
the expansion hub changes the controller response.

A new 10-second read-only run used 115200 baud, slave 1, a 0.5-second timeout, and only
function `0x03`. All 20 attempts returned the exact request echo and no controller bytes;
there were zero valid `0x20AB/0x20AC` pairs. The kernel log contained no new USB/UVC
event during this run. No write or motion command was sent.

Evidence:

- `docs/hardware/evidence/ZLAC_ENCODER_HUB_RETRY_20260726.json`

The next discriminating test still requires moving this specific `1a86:55d3` adapter
off the Genesys expansion hub and onto a direct Orin USB port, then confirming that its
`ID_PATH` changed before repeating the same read-only query.

## Direct-Orin-port A/B test — 2026-07-26

The user removed the right-side and right-front cameras because the wide RS485 adapter
prevented use of the adjacent USB socket, then plugged the ZLAC adapter directly into
the freed Orin port. The topology changed as required:

```text
before: platform-3610000.usb-usb-0:1.1:1.0  (external Genesys hub)
after:  platform-3610000.usb-usb-0:4.3:1.0  (Orin direct-port USB2 companion path)
```

The stable device identity remained `1a86:55d3`, serial `5C66036979`, and the stable
alias remained `/dev/smartwheel_zlac8030 -> /dev/ttyACM0`. Camera paths `2-3.1` and
`2-3.2` remained online at `5000M`; the intentionally unplugged `2-3.3` and `2-3.4`
paths were absent.

The same 10-second, 20-attempt, 115200-baud, slave-1, function-`0x03` diagnostic was
then repeated with a 0.5-second timeout. Every transaction again contained only the
exact local request echo and zero controller-response bytes. There were no valid
`0x20AB/0x20AC` pairs and no new USB/UVC kernel event during the test. No Modbus write
or motion command was sent.

Evidence:

- `docs/hardware/evidence/ZLAC_ENCODER_DIRECT_USB_READONLY_20260726.json`

This A/B result rules out the external expansion hub as the sole cause of the silent
controller. The remaining fault domain is after the USB-RS485 adapter: controller
logic power/status, RS485 A/B polarity and communication ground, CAN-versus-RS485
connector selection, connector/contact continuity, controller address/settings, or a
controller-side interface fault. Do not issue control writes until a valid read frame
is observed.

## Communication recovery — 2026-07-26

After a further user-side physical adjustment, the four cameras were again present at
`5000M` and the ZLAC adapter was back on the external-hub path
`platform-3610000.usb-usb-0:1.1:1.0`. The user did not identify a single changed
electrical variable, so the recovery cannot be attributed to the hub, wiring, power,
or another specific component.

A 10-second, 1 Hz request-loop run with a 1.0-second timeout produced five complete
left/right pairs with no failures or echoes. Both feedback registers returned valid
Modbus frames, for example:

```text
request left:  01 03 20 AB 00 01 FE 2A
request right: 01 03 20 AC 00 01 4F EB
response:      01 03 02 00 00 B8 44
```

Both decoded stationary values were `0 raw` / `0.0 rpm`. A second 15-second stability
run used a 5 Hz request loop and 0.2-second timeout. All 38 complete pairs succeeded,
again with zero echoes, zero failures, and constant zero feedback on both axes. Its
achieved complete-pair rate was approximately 2.489 Hz because the diagnostic reads
the two registers sequentially. No new USB/UVC kernel event occurred during either
run, and no Modbus write or movement command was sent.

Evidence:

- `docs/hardware/evidence/ZLAC_ENCODER_CURRENT_RETRY_20260726.json`
- `docs/hardware/evidence/ZLAC_ENCODER_STATIONARY_CONFIRM_20260726.json`

**The read-only controller communication gate now passes.** This validates stationary
transport and the two feedback-register reads only. It does not yet validate encoder
direction, nonzero scaling, wheel geometry, motor polarity, watchdog behavior, normal
driver start/stop writes, or safe motion. Before any control write, the user must
explicitly confirm both drive wheels are off the ground, no passenger is present, the
area is clear, and the physical emergency stop is ready.

## Guarded motion preflight — 2026-07-26

The user explicitly confirmed that both drive wheels were off the ground, no passenger
was present, the area was clear, and the physical emergency stop was ready. The current
driver, guarded bring-up helper, historical `45b1163` implementation, and archived
hardware feedback were rechecked before issuing a control write. They agree on slave 1,
velocity mode register `0x200D`, control word `0x200E`, dual target registers
`0x2088/0x2089`, feedback registers `0x20AB/0x20AC`, and the clear/enable/stop/emergency
control values `6/8/7/5`.

The first write-only safety preflight sent a dual zero target followed by software
emergency stop. The controller acknowledged both operations and returned this snapshot:

```text
after_emergency_stop: mode=3 control=5 async=1 left_rpm=0 right_rpm=0 left_torque=0 right_torque=0
```

No wheel motion was requested in this preflight. The guarded motion initializer will
explicitly set asynchronous mode to `0` before enabling, matching the existing
known-good sequence.

### Left-wheel minimum-command trial

The guarded helper then initialized velocity mode, set asynchronous mode to `0`,
cleared faults, enabled the drive, and commanded only the left target register to `+5`
while the right target remained `0`. The command duration was limited to 0.75 seconds.
The helper's `finally` path wrote both targets to zero and latched software emergency
stop immediately afterward.

```text
after_enable:         mode=3 control=8 async=0 left_rpm=0 right_rpm=0 left_torque=0 right_torque=0
left_only:            target_left=5 target_right=0
left_sample:          mode=3 control=8 async=0 left_rpm=0 right_rpm=0 left_torque=3 right_torque=0
after_emergency_stop: mode=3 control=5 async=0 left_rpm=0 right_rpm=0 left_torque=3 right_torque=0
```

The controller acknowledged the enable, target, zero, and emergency-stop sequence.
Left torque changed from `0` to `3`, while both speed feedback registers remained `0`;
therefore this lowest target did not produce measurable encoder rotation. Physical
motion was not inferred from torque alone. A five-second post-test read-only check
passed 5/5 complete pairs with both axes fixed at zero, and no new USB/UVC kernel event
was logged.

Evidence:

- `docs/hardware/evidence/ZLAC_ENCODER_POST_LEFT5_READONLY_20260726.json`

Do not automatically increase the target. The next trial, if explicitly continued,
should remain left-only and use the next bounded target before attempting the right
axis.

### Left-wheel bounded `+20` trial

The user reported no visible change at target `+5` and explicitly requested a more
noticeable trial. With the same safety conditions still in force, the left-only target
was increased to raw `+20` for 1.0 second. This remained below the archived raw `30`
two-axis trial. The right target stayed zero and the same mandatory dual-zero plus
software-emergency-stop exit path was used.

```text
after_enable:         mode=3 control=8 async=0 left_rpm=0 right_rpm=0 left_torque=2 right_torque=0
left_only:            target_left=20 target_right=0
left_sample:          mode=3 control=8 async=0 left_rpm=1 right_rpm=0 left_torque=16 right_torque=0
after_emergency_stop: mode=3 control=5 async=0 left_rpm=0 right_rpm=0 left_torque=16 right_torque=0
```

This was the first current-session nonzero dynamic feedback: left speed reached raw
`+1` while left torque reached `16`; right speed and torque remained zero. The final
snapshot and a subsequent 5/5-pair read-only check both showed zero speed on both axes.
No new USB/UVC kernel event occurred. Physical rotation and its direction still require
the user's visual observation; the provisional `0.112 rpm/raw` scaling must not be
treated as dynamically calibrated from this one sample.

Evidence:

- `docs/hardware/evidence/ZLAC_LEFT20_GUARDED_MOTION_20260726.json`
- `docs/hardware/evidence/ZLAC_ENCODER_POST_LEFT20_READONLY_20260726.json`

### Left-wheel historical-boundary `+30` trial

The user still could not see a change and asked for a larger observation range. The
left-only target was increased to raw `+30` for 2.0 seconds, exactly matching the target
and duration already present in the archived hardware feedback test while keeping the
right target at zero. No larger historical boundary was used.

```text
after_enable:         mode=3 control=8 async=0 left_rpm=0 right_rpm=0 left_torque=15 right_torque=0
left_only:            target_left=30 target_right=0
left_sample:          mode=3 control=8 async=0 left_rpm=0 right_rpm=0 left_torque=26 right_torque=2
after_emergency_stop: mode=3 control=5 async=0 left_rpm=2 right_rpm=0 left_torque=26 right_torque=2
```

The controller acknowledged the complete sequence. The left torque rose to `26`, and
nonzero left speed raw `+2` was captured immediately after the target was zeroed and
software emergency stop was latched; this is consistent with a short sampling/inertia
window but does not establish physical direction. Right speed remained zero. A later
five-second read-only check passed 5/5 pairs with both speeds at zero, and no new
USB/UVC kernel event was logged.

Evidence:

- `docs/hardware/evidence/ZLAC_LEFT30_GUARDED_MOTION_20260726.json`
- `docs/hardware/evidence/ZLAC_ENCODER_POST_LEFT30_READONLY_20260726.json`

## RViz guarded manual-drive acceptance — 2026-07-26

After the user again confirmed both wheels were off the ground, no passenger was
present, the area was clear, and the physical emergency stop was available, the real
base was started behind the required command chain:

```text
/teleop/cmd_vel -> safety_supervisor -> /cmd_vel_safe -> zlac8030_driver_node
```

The user exercised the RViz W/A/S/D and STOP controls and explicitly confirmed that
all directions and buttons operated correctly. The bounded run was recorded at:

```text
/home/nvidia/smartwheel/bags/hardware/zlac_rviz_manual_20260726
```

The bag contains 254.201 seconds and 43,130 messages, including 5,181 input commands,
5,052 safety-clamped commands, 12,659 odometry messages, and 12,660 base-status
messages. The panel requested up to `0.15 m/s` and `0.25 rad/s`; the active safety
profile limited output to `0.08 m/s` and `0.20 rad/s`. Base feedback reached about
`0.09773 m/s` and `0.2102 rad/s`, with 767 base-status samples reporting nonzero wheel
speed. These observations prove that all GUI directions reached the real controller;
they do not calibrate wheel diameter, separation, direction, or odometric accuracy.

The same bag exposed a GUI input defect: held keys received through the remote desktop
appeared as repeated press/release pairs, producing isolated nonzero input samples
separated by zeros. The safety supervisor was not the source because fragmentation was
already present on `/teleop/cmd_vel`. The RViz panel now accepts repeat events and
defers release by 120 ms; a generation guard cancels an intermediate release when the
next repeat press arrives. STOP, application/window deactivation, hide/close, and the
independent safety timeout still publish zero immediately. A Qt regression test
reproduces the remote repeat sequence and verifies that only the final release clears
the held direction.

The test ended with RViz, recorder, base, safety supervisor, and emergency-stop test
units inactive and `/cmd_vel_safe` at zero. A final strictly read-only 3/3-pair query
found both speed registers at zero and sent no Modbus write.

Evidence:

- `docs/hardware/evidence/ZLAC_ENCODER_POST_RVIZ_READONLY_20260726.json`
