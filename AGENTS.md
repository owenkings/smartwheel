# V7 public source snapshot

Read PUBLICATION.md before adapting local hardware bindings. This branch contains code and configuration examples, not field data or Orin Git history. Do not start physical hardware or drive the chair during offline tests. Keep local configuration and measured evidence private and preserve source hashes.

## Code and generated artifacts

Keep maintained source, configuration templates, test source and current manuals in the project. Resolve new reports, diagnostic work folders, data, maps, build logs and test results through wc_runtime.storage_policy into the configured data root. Do not create ad-hoc work or report folders in the code root. The committed storage.json is portable; ignored storage.local.json holds this computer's explicit choice. A selected removable destination must not silently fall back after detachment.

Use tests/run_target_tests.sh for selected software tests. POSIX-only fixtures may use short unique system /tmp paths; archive their evidence to configured storage before removing only the owned scratch directory. Keep device locks, process registration and Unix sockets on a suitable local Linux filesystem. Keep tests/ source in Git and exclude outputs, local overrides, recordings and maps. State explicitly whether reviewed code has actually been pushed; a review candidate is not a GitHub update.
