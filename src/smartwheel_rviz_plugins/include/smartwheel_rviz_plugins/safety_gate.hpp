#ifndef SMARTWHEEL_RVIZ_PLUGINS__SAFETY_GATE_HPP_
#define SMARTWHEEL_RVIZ_PLUGINS__SAFETY_GATE_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <tuple>

namespace smartwheel_rviz_plugins
{

class SafetyGate
{
public:
  SafetyGate(double timeout_sec, double linear_max, double angular_max)
  : timeout_sec_(timeout_sec), linear_max_(linear_max), angular_max_(angular_max)
  {
    if (timeout_sec_ <= 0.0 || linear_max_ <= 0.0 || angular_max_ <= 0.0) {
      throw std::invalid_argument("safety timeout and limits must be positive");
    }
  }

  bool accept(double linear, double angular, double now_sec)
  {
    if (!std::isfinite(linear) || !std::isfinite(angular) || !std::isfinite(now_sec)) {
      linear_ = angular_ = 0.0;
      last_command_sec_ = -1.0;
      ++invalid_count_;
      return false;
    }
    linear_ = std::clamp(linear, -linear_max_, linear_max_);
    angular_ = std::clamp(angular, -angular_max_, angular_max_);
    last_command_sec_ = now_sec;
    return true;
  }

  std::tuple<double, double, std::string> output(double now_sec) const
  {
    if (emergency_stop_) {
      return {0.0, 0.0, "EMERGENCY_STOP"};
    }
    if (last_command_sec_ < 0.0 || now_sec - last_command_sec_ > timeout_sec_) {
      return {0.0, 0.0, "COMMAND_TIMEOUT"};
    }
    return {linear_, angular_, "ACTIVE"};
  }

  void setEmergencyStop(bool active) {emergency_stop_ = active;}
  bool emergencyStop() const {return emergency_stop_;}
  int invalidCount() const {return invalid_count_;}
  double timeout() const {return timeout_sec_;}

private:
  double timeout_sec_;
  double linear_max_;
  double angular_max_;
  double linear_{0.0};
  double angular_{0.0};
  double last_command_sec_{-1.0};
  bool emergency_stop_{false};
  int invalid_count_{0};
};

}  // namespace smartwheel_rviz_plugins

#endif  // SMARTWHEEL_RVIZ_PLUGINS__SAFETY_GATE_HPP_
