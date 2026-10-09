#pragma once
#include <atomic>
#include <exception>
#include <string>

namespace wc_xt_driver {
struct StopResult { bool acknowledged; std::string error; };

// Ordinary control-thread code, never a signal handler. "pending" records an
// owned stream whose stop has not been acknowledged; disabling publication is
// separate. A failed attempt remains retryable and permanently latches failure.
template<class Stop>
StopResult request_owned_stop(std::atomic<bool>& pending,
                             std::atomic<bool>& failed, Stop stop) {
  if (!pending.load()) return {true, {}};
  std::string error;
  try {
    if (stop()) {
      pending.store(false);
      return {true, {}}; // Never clear a previous failure, even after recovery.
    }
    error = "SDK stop was not acknowledged";
  } catch (const std::exception& exception) {
    error = std::string("SDK stop threw: ") + exception.what();
  } catch (...) {
    error = "SDK stop threw an unknown exception";
  }
  failed.store(true);
  return {false, error};
}
} // namespace wc_xt_driver
