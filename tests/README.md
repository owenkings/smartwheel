# Tests

All test scripts live here in owner-specific folders: fusion, sensors, calibration, maps and integration. Production modules contain no hidden test scripts. `tests/run_target_tests.sh` is the target-only dispatcher; it runs this directory and writes command/exit-code/source-hash/JUnit evidence to reports/test_runs.

Run only after deployment to the verified Orin. No synthetic test demonstrates real sensor operation, dynamic mapping, loop closure on real data, or RViz graphics acceptance.
