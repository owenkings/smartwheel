# Encoder feedback tests

`python -m pytest -p no:cacheprovider -q tests/motion`

All numerical examples use explicit synthetic geometry and an invented test
register map. They do not establish ZLAC8030D feedback scaling or real odometry.
Two Linux-only PTY tests exercise receive-only access, unchanged tty settings and
mutual exclusion; skipping these tests on Windows is not a pass.

The pytest regression tests do not open physical serial ports. The optional FC03 client uses an in-memory
fake transport and asserts that disabled queries, nonwhitelisted addresses and
all write function codes are rejected.

`wc_motion.replay` accepts a config, input JSONL and an exclusively created output:

```bash
PYTHONPATH=src python3 -m wc_motion.replay --config CONFIG.json --input FEEDBACK.jsonl --output NEW_RESULT.jsonl
```

Each odometry-mode input must carry a matched observed FC03 request/response,
unique physical device identity, epoch/sequence, common measurement timestamp,
explicit timestamp uncertainty and both wheel fields. The configuration must
contain evidenced units, conversion scales and geometry. In raw_only mode none
of these unknown values are invented and no wheel odometry is produced.

Explicit hardware integration is separate: `bash tests/motion/run_static_integration.sh monitor_320`
requires a stationary chair, free sensor devices and the Orin graphical desktop.
It starts four cameras, both lidars, H30, and fixed FC03 wheel feedback reads,
opens RViz, records a new bag, and stops its own bounded sessions. It sends no
motor control writes. Do not run it while another application owns these devices.
All outputs use new directories under `reports/encoder/`; the audited bag remains
under `data/bags/`. Camera slots still have unconfirmed physical directions.

`python3 -s tests/motion/check_recorded_feedback.py --bag BAG --output NEW.json`
is an offline, read-only audit requiring at least 10 raw and preview records plus
matching diagnostics. It verifies zero wheel feedback and zero preview pose for
static recordings only; it is not a dynamic odometry calibration test.
