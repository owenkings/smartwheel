#ifndef SMARTWHEEL_RVIZ_PLUGINS__TELEOP_MODEL_HPP_
#define SMARTWHEEL_RVIZ_PLUGINS__TELEOP_MODEL_HPP_

#include <set>
#include <string>
#include <utility>

namespace smartwheel_rviz_plugins
{

class TeleopModel
{
public:
  void setSpeeds(double linear, double angular, double linear_max, double angular_max);
  void setTimeout(double timeout_sec);
  void setKey(char key, bool pressed, double now_sec);
  void heartbeat(double now_sec);
  void stop(double now_sec);
  void focusLost(double now_sec);
  std::pair<double, double> command(double now_sec);
  std::string activeKeys() const;
  bool deadman() const;
  bool timedOut() const;

private:
  std::set<char> pressed_;
  double linear_{0.10};
  double angular_{0.25};
  double linear_max_{0.15};
  double angular_max_{0.25};
  double timeout_sec_{0.35};
  double last_heartbeat_sec_{-1.0};
  bool timed_out_{false};
};

}  // namespace smartwheel_rviz_plugins

#endif  // SMARTWHEEL_RVIZ_PLUGINS__TELEOP_MODEL_HPP_

