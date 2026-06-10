#ifndef WHEELCHAIR_BRINGUP_TELEOP_PANEL_HPP_
#define WHEELCHAIR_BRINGUP_TELEOP_PANEL_HPP_

#include <memory>

#include <QtWidgets>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace wheelchair_bringup
{

// An RViz dockable panel that drives the wheelchair with on-screen buttons and
// W/A/S/D keys. It publishes geometry_msgs/Twist to /cmd_vel_nav so motion still
// flows through safety_supervisor (/cmd_vel_nav -> /cmd_vel_safe -> base). It
// never publishes /cmd_vel_safe directly. A periodic timer repeats the current
// command; releasing a button (or pressing Stop/Space) sends zero velocity.
class TeleopPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TeleopPanel(QWidget * parent = nullptr);
  ~TeleopPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

protected Q_SLOTS:
  void setForward();
  void setBackward();
  void setLeft();
  void setRight();
  void stop();
  void updateSpeeds();

protected:
  bool eventFilter(QObject * object, QEvent * event) override;
  void publishCommand();
  void setTarget(double linear_scale, double angular_scale, const QString & label);

  QPushButton * forward_button_{nullptr};
  QPushButton * backward_button_{nullptr};
  QPushButton * left_button_{nullptr};
  QPushButton * right_button_{nullptr};
  QPushButton * stop_button_{nullptr};
  QDoubleSpinBox * linear_spin_{nullptr};
  QDoubleSpinBox * angular_spin_{nullptr};
  QLabel * state_label_{nullptr};
  QLineEdit * topic_edit_{nullptr};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  QTimer * publish_timer_{nullptr};

  double max_linear_{0.25};
  double max_angular_{0.6};
  double target_linear_{0.0};
  double target_angular_{0.0};
  QString topic_{"/cmd_vel_nav"};
};

}  // namespace wheelchair_bringup

#endif  // WHEELCHAIR_BRINGUP_TELEOP_PANEL_HPP_
