#ifndef SMARTWHEEL_RVIZ_PLUGINS__TELEOP_PANEL_HPP_
#define SMARTWHEEL_RVIZ_PLUGINS__TELEOP_PANEL_HPP_

#include "smartwheel_rviz_plugins/teleop_model.hpp"

#include <memory>
#include <set>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <smartwheel_interfaces/msg/hardware_status.hpp>
#include <std_msgs/msg/bool.hpp>

class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QTimer;

namespace smartwheel_rviz_plugins
{

class TeleopPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TeleopPanel(QWidget * parent = nullptr);
  ~TeleopPanel() override;
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

protected:
  bool eventFilter(QObject * object, QEvent * event) override;
  void closeEvent(QCloseEvent * event) override;
  void hideEvent(QHideEvent * event) override;

private Q_SLOTS:
  void publishTick();
  void stopNow();
  void updateParameters();
  void updatePublisher();

private:
  void setKeyboardDirection(char key, bool pressed);
  void setMouseDirection(char key, bool pressed);
  void applyDirectionState();
  void publishZero();
  double nowSeconds() const;
  double approach(double current, double target, double maximum_delta) const;
  void updateLabels(double linear, double angular);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<smartwheel_interfaces::msg::HardwareStatus>::SharedPtr hardware_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_subscription_;
  TeleopModel model_;
  bool hardware_enabled_{false};
  bool emergency_stop_{false};
  double current_linear_{0.0};
  double current_angular_{0.0};
  double last_tick_sec_{0.0};
  double acceleration_limit_{0.25};
  double angular_acceleration_limit_{0.5};
  QString topic_{"/teleop/cmd_vel"};
  std::set<char> keyboard_directions_;
  std::set<char> mouse_directions_;

  QLineEdit * topic_edit_{nullptr};
  QDoubleSpinBox * linear_speed_{nullptr};
  QDoubleSpinBox * angular_speed_{nullptr};
  QDoubleSpinBox * linear_max_{nullptr};
  QDoubleSpinBox * angular_max_{nullptr};
  QDoubleSpinBox * acceleration_{nullptr};
  QDoubleSpinBox * angular_acceleration_{nullptr};
  QSpinBox * command_rate_{nullptr};
  QSpinBox * command_timeout_{nullptr};
  QLabel * linear_label_{nullptr};
  QLabel * angular_label_{nullptr};
  QLabel * keys_label_{nullptr};
  QLabel * deadman_label_{nullptr};
  QLabel * hardware_label_{nullptr};
  QLabel * emergency_label_{nullptr};
  QLabel * source_label_{nullptr};
  QLabel * chain_label_{nullptr};
  QTimer * publish_timer_{nullptr};
};

}  // namespace smartwheel_rviz_plugins

#endif  // SMARTWHEEL_RVIZ_PLUGINS__TELEOP_PANEL_HPP_
