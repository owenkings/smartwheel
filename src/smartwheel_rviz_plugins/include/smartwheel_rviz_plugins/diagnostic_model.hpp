#ifndef SMARTWHEEL_RVIZ_PLUGINS__DIAGNOSTIC_MODEL_HPP_
#define SMARTWHEEL_RVIZ_PLUGINS__DIAGNOSTIC_MODEL_HPP_

#include <cstdint>
#include <string>

namespace smartwheel_rviz_plugins
{

enum class UiStatus {ONLINE, WARNING, ERROR, DISABLED, MOCK};

UiStatus diagnosticLevelToStatus(uint8_t level, bool mock_mode, bool disabled = false);
std::string statusText(UiStatus status);

}  // namespace smartwheel_rviz_plugins

#endif  // SMARTWHEEL_RVIZ_PLUGINS__DIAGNOSTIC_MODEL_HPP_
