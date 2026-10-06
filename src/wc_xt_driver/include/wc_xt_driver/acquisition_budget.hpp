#pragma once

namespace wc_xt_driver {
// Zero disables only total acquisition duration. Connection, source-health,
// owned-stop and cleanup deadlines remain independent.
inline bool valid_acquisition_runtime(int seconds) { return seconds >= 0; }
inline bool acquisition_runtime_expired(double elapsed_seconds, int seconds) {
  return seconds > 0 && elapsed_seconds > seconds;
}
// Continuous mapping disables silence shutdown without changing identity,
// connection/readback or explicit owned-stop behavior.
inline bool acquisition_source_silence_expired(double age_seconds, int seconds) {
  return seconds > 0 && age_seconds > seconds;
}
}  // namespace wc_xt_driver
