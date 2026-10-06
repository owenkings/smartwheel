// Pure runtime-bound checks: no SDK, ROS, network, serial or hardware.
#include "wc_xt_driver/acquisition_budget.hpp"
#include <initializer_list>
#include <stdexcept>

int main() {
  using wc_xt_driver::valid_acquisition_runtime;
  using wc_xt_driver::acquisition_runtime_expired;
  if (valid_acquisition_runtime(-1) || !valid_acquisition_runtime(0) ||
      !valid_acquisition_runtime(30))
    throw std::runtime_error("runtime validation changed");
  for (double elapsed : {0., 30., 2415., 43201., 1000000000.}) {
    if (acquisition_runtime_expired(elapsed, 0))
      throw std::runtime_error("explicit until-stop acquisition expired");
    if (wc_xt_driver::acquisition_source_silence_expired(elapsed, 0))
      throw std::runtime_error("continuous source silence stopped acquisition");
  }
  if (acquisition_runtime_expired(29.9, 30) || acquisition_runtime_expired(30., 30) ||
      !acquisition_runtime_expired(30.1, 30))
    throw std::runtime_error("finite runtime no longer preserves its original boundary");
  if (wc_xt_driver::acquisition_source_silence_expired(3., 3) ||
      !wc_xt_driver::acquisition_source_silence_expired(3.01, 3))
    throw std::runtime_error("independent diagnostic source silence policy changed");
  return 0;
}
