#include "smartwheel_rviz_plugins/diagnostic_model.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace smartwheel_rviz_plugins
{

UiStatus diagnosticLevelToStatus(uint8_t level, bool mock_mode, bool disabled)
{
  if (disabled) {
    return UiStatus::DISABLED;
  }
  if (level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
    return UiStatus::ERROR;
  }
  if (level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
    return UiStatus::WARNING;
  }
  return mock_mode ? UiStatus::MOCK : UiStatus::ONLINE;
}

std::string statusText(UiStatus status)
{
  switch (status) {
    case UiStatus::ONLINE: return "ONLINE";
    case UiStatus::WARNING: return "WARNING";
    case UiStatus::ERROR: return "ERROR";
    case UiStatus::DISABLED: return "DISABLED";
    case UiStatus::MOCK: return "MOCK";
  }
  return "ERROR";
}

}  // namespace smartwheel_rviz_plugins
