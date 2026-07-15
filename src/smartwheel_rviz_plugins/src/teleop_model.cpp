#include "smartwheel_rviz_plugins/teleop_model.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <stdexcept>

namespace smartwheel_rviz_plugins
{

void TeleopModel::setSpeeds(double linear, double angular, double linear_max, double angular_max)
{
  if (linear < 0.0 || angular < 0.0 || linear_max <= 0.0 || angular_max <= 0.0) {
    throw std::invalid_argument("teleop speed values must be non-negative with positive maxima");
  }
  linear_max_ = linear_max;
  angular_max_ = angular_max;
  linear_ = std::min(linear, linear_max_);
  angular_ = std::min(angular, angular_max_);
}

void TeleopModel::setTimeout(double timeout_sec)
{
  if (timeout_sec <= 0.0) {
    throw std::invalid_argument("command timeout must be positive");
  }
  timeout_sec_ = timeout_sec;
}

void TeleopModel::setKey(char key, bool pressed, double now_sec)
{
  const char normalized = static_cast<char>(std::tolower(static_cast<unsigned char>(key)));
  if (normalized != 'w' && normalized != 'a' && normalized != 's' && normalized != 'd') {
    return;
  }
  if (pressed) {
    pressed_.insert(normalized);
  } else {
    pressed_.erase(normalized);
  }
  last_heartbeat_sec_ = now_sec;
  timed_out_ = false;
}

void TeleopModel::heartbeat(double now_sec)
{
  last_heartbeat_sec_ = now_sec;
  timed_out_ = false;
}

void TeleopModel::stop(double now_sec)
{
  pressed_.clear();
  last_heartbeat_sec_ = now_sec;
  timed_out_ = false;
}

void TeleopModel::focusLost(double now_sec)
{
  stop(now_sec);
}

std::pair<double, double> TeleopModel::command(double now_sec)
{
  if (!pressed_.empty() &&
    (last_heartbeat_sec_ < 0.0 || now_sec - last_heartbeat_sec_ > timeout_sec_))
  {
    pressed_.clear();
    timed_out_ = true;
    return {0.0, 0.0};
  }
  const double linear_direction = (pressed_.count('w') ? 1.0 : 0.0) -
    (pressed_.count('s') ? 1.0 : 0.0);
  const double angular_direction = (pressed_.count('a') ? 1.0 : 0.0) -
    (pressed_.count('d') ? 1.0 : 0.0);
  return {
    std::clamp(linear_direction * linear_, -linear_max_, linear_max_),
    std::clamp(angular_direction * angular_, -angular_max_, angular_max_)};
}

std::string TeleopModel::activeKeys() const
{
  std::ostringstream stream;
  for (const char key : pressed_) {
    if (stream.tellp() > 0) {
      stream << "+";
    }
    stream << static_cast<char>(std::toupper(static_cast<unsigned char>(key)));
  }
  return stream.str().empty() ? "NONE" : stream.str();
}

bool TeleopModel::deadman() const
{
  return !pressed_.empty();
}

bool TeleopModel::timedOut() const
{
  return timed_out_;
}

}  // namespace smartwheel_rviz_plugins

