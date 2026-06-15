# Hardware bring-up — OFF-GROUND DEBUG ONLY

> ⚠️ The scripts here **command real ZLAC8030D motor motion**. Run them ONLY with
> the wheelchair **wheels off the ground** (jacked up / on blocks), no person
> seated, area clear, and an operator ready to cut power.

Moved out of `scripts/` so they are never run casually. They are NOT part of any
launch and are NOT triggered by `smartwheel.service` autostart.

- `jog_zlac8030d_motor.py` — manually jog a wheel motor (interactive).
- `test_zlac8030d_motor.py` — motor command/feedback test sequence.

Normal mapping/navigation keeps motors gated (`motion_control_enabled: false`,
ZLAC command registers `-1`). Nothing here is needed for RTAB-Map / KISS-ICP 3D
mapping. Read-only diagnostics (e.g. `scripts/read_zlac8030_readonly.py`) stay in
`scripts/` because they never move the motor.
