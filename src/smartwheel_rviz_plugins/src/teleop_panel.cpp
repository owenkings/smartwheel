#include "smartwheel_rviz_plugins/teleop_panel.hpp"

#include <QApplication>
#include <QCloseEvent>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFormLayout>
#include <QGridLayout>
#include <QHideEvent>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>

#include <rviz_common/display_context.hpp>

#include <algorithm>
#include <cmath>

namespace smartwheel_rviz_plugins
{
namespace
{

// Remote desktops can synthesize a held key as rapid press/release pairs. Keep a
// short release grace period so those pairs remain one continuous command while
// still stopping quickly after the final release.
constexpr int kKeyboardReleaseGraceMs = 120;

}  // namespace

TeleopPanel::TeleopPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * content = new QWidget;
  auto * layout = new QVBoxLayout(content);
  layout->setContentsMargins(4, 4, 4, 4);

  auto * topic_form = new QFormLayout;
  topic_edit_ = new QLineEdit(topic_);
  topic_form->addRow("Output", topic_edit_);
  layout->addLayout(topic_form);

  auto * directions = new QGridLayout;
  auto * forward = new QPushButton("W");
  auto * left = new QPushButton("A");
  auto * stop = new QPushButton("STOP");
  auto * right = new QPushButton("D");
  auto * reverse = new QPushButton("S");
  for (auto * button : {forward, left, right, reverse}) {
    button->setMinimumSize(54, 44);
  }
  stop->setMinimumSize(72, 48);
  stop->setStyleSheet("background:#b42318;color:white;font-weight:700;");
  directions->addWidget(forward, 0, 1);
  directions->addWidget(left, 1, 0);
  directions->addWidget(stop, 1, 1);
  directions->addWidget(right, 1, 2);
  directions->addWidget(reverse, 2, 1);
  layout->addLayout(directions);

  auto bind = [this](QPushButton * button, char key) {
      connect(button, &QPushButton::pressed, this, [this, key]() {
        setMouseDirection(key, true);
      });
      connect(button, &QPushButton::released, this, [this, key]() {
        setMouseDirection(key, false);
      });
    };
  bind(forward, 'w');
  bind(reverse, 's');
  bind(left, 'a');
  bind(right, 'd');
  connect(stop, &QPushButton::pressed, this, &TeleopPanel::stopNow);

  auto * parameters = new QFormLayout;
  linear_speed_ = new QDoubleSpinBox;
  linear_speed_->setRange(0.0, 0.15);
  linear_speed_->setValue(0.10);
  linear_speed_->setSuffix(" m/s");
  angular_speed_ = new QDoubleSpinBox;
  angular_speed_->setRange(0.0, 0.25);
  angular_speed_->setValue(0.25);
  angular_speed_->setSuffix(" rad/s");
  linear_max_ = new QDoubleSpinBox;
  linear_max_->setRange(0.01, 0.15);
  linear_max_->setValue(0.15);
  angular_max_ = new QDoubleSpinBox;
  angular_max_->setRange(0.01, 0.25);
  angular_max_->setValue(0.25);
  acceleration_ = new QDoubleSpinBox;
  acceleration_->setRange(0.01, 1.0);
  acceleration_->setValue(0.25);
  angular_acceleration_ = new QDoubleSpinBox;
  angular_acceleration_->setRange(0.01, 2.0);
  angular_acceleration_->setValue(0.5);
  command_rate_ = new QSpinBox;
  command_rate_->setRange(5, 50);
  command_rate_->setValue(20);
  command_rate_->setSuffix(" Hz");
  command_timeout_ = new QSpinBox;
  command_timeout_->setRange(100, 2000);
  command_timeout_->setValue(350);
  command_timeout_->setSuffix(" ms");
  parameters->addRow("Linear speed", linear_speed_);
  parameters->addRow("Angular speed", angular_speed_);
  parameters->addRow("Linear max", linear_max_);
  parameters->addRow("Angular max", angular_max_);
  parameters->addRow("Acceleration", acceleration_);
  parameters->addRow("Angular acceleration", angular_acceleration_);
  parameters->addRow("Command rate", command_rate_);
  parameters->addRow("Command timeout", command_timeout_);
  layout->addLayout(parameters);

  linear_label_ = new QLabel("linear.x 0.000 m/s");
  angular_label_ = new QLabel("angular.z 0.000 rad/s");
  keys_label_ = new QLabel("Keys NONE");
  keys_label_->setObjectName("teleopKeysLabel");
  deadman_label_ = new QLabel("Deadman RELEASED");
  hardware_label_ = new QLabel("hardware_enabled=false");
  emergency_label_ = new QLabel("emergency_stop=UNKNOWN");
  source_label_ = new QLabel("Source keyboard/mouse");
  chain_label_ = new QLabel("/teleop/cmd_vel -> safety_supervisor -> /cmd_vel_safe -> mock base");
  chain_label_->setWordWrap(true);
  hardware_label_->setStyleSheet("color:#b25d00;font-weight:600;");
  for (auto * label : {linear_label_, angular_label_, keys_label_, deadman_label_, hardware_label_, emergency_label_, source_label_, chain_label_}) {
    layout->addWidget(label);
  }

  auto * outer = new QVBoxLayout(this);
  auto * scroll = new QScrollArea;
  scroll->setWidgetResizable(true);
  scroll->setWidget(content);
  outer->addWidget(scroll);

  connect(topic_edit_, &QLineEdit::editingFinished, this, &TeleopPanel::updatePublisher);
  for (auto * spin : {linear_speed_, angular_speed_, linear_max_, angular_max_, acceleration_, angular_acceleration_}) {
    connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &TeleopPanel::updateParameters);
  }
  connect(command_rate_, QOverload<int>::of(&QSpinBox::valueChanged), this, &TeleopPanel::updateParameters);
  connect(command_timeout_, QOverload<int>::of(&QSpinBox::valueChanged), this, &TeleopPanel::updateParameters);
  qApp->installEventFilter(this);
  setFocusPolicy(Qt::StrongFocus);
  updateParameters();
}

TeleopPanel::~TeleopPanel()
{
  qApp->removeEventFilter(this);
  stopNow();
  publisher_.reset();
}

void TeleopPanel::onInitialize()
{
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    chain_label_->setText("FAILED: RViz ROS node unavailable");
    return;
  }
  node_ = abstraction->get_raw_node();
  updatePublisher();
  hardware_subscription_ = node_->create_subscription<smartwheel_interfaces::msg::HardwareStatus>(
    "/hardware/status", rclcpp::QoS(1).reliable().transient_local(),
    [this](const smartwheel_interfaces::msg::HardwareStatus::ConstSharedPtr message) {
      hardware_enabled_ = message->hardware_enabled;
      QMetaObject::invokeMethod(hardware_label_, "setText", Qt::QueuedConnection,
        Q_ARG(QString, QString("hardware_enabled=%1 (%2)").arg(message->hardware_enabled ? "true" : "false").arg(QString::fromStdString(message->mode))));
    });
  emergency_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", rclcpp::QoS(1).reliable().transient_local(),
    [this](const std_msgs::msg::Bool::ConstSharedPtr message) {
      emergency_stop_ = message->data;
      QMetaObject::invokeMethod(emergency_label_, "setText", Qt::QueuedConnection,
        Q_ARG(QString, QString("emergency_stop=%1").arg(message->data ? "ACTIVE" : "clear")));
    });
  publish_timer_ = new QTimer(this);
  connect(publish_timer_, &QTimer::timeout, this, &TeleopPanel::publishTick);
  publish_timer_->start(1000 / command_rate_->value());
  last_tick_sec_ = nowSeconds();
}

double TeleopPanel::nowSeconds() const
{
  return node_ ? node_->get_clock()->now().seconds() : 0.0;
}

double TeleopPanel::approach(double current, double target, double maximum_delta) const
{
  if (target > current) {return std::min(target, current + maximum_delta);}
  if (target < current) {return std::max(target, current - maximum_delta);}
  return target;
}

void TeleopPanel::setKeyboardDirection(char key, bool pressed)
{
  if (pressed) {
    ++keyboard_generations_[key];
    keyboard_directions_.insert(key);
  } else {
    keyboard_directions_.erase(key);
  }
  applyDirectionState();
}

void TeleopPanel::scheduleKeyboardRelease(char key)
{
  const auto generation = keyboard_generations_[key];
  QTimer::singleShot(kKeyboardReleaseGraceMs, this, [this, key, generation]() {
    const auto current = keyboard_generations_.find(key);
    if (current == keyboard_generations_.end() || current->second != generation ||
      keyboard_directions_.count(key) == 0)
    {
      return;
    }
    setKeyboardDirection(key, false);
  });
}

void TeleopPanel::setMouseDirection(char key, bool pressed)
{
  if (pressed) {
    mouse_directions_.insert(key);
  } else {
    mouse_directions_.erase(key);
  }
  applyDirectionState();
}

void TeleopPanel::applyDirectionState()
{
  const double now = nowSeconds();
  for (const char key : {'w', 'a', 's', 'd'}) {
    model_.setKey(
      key, keyboard_directions_.count(key) != 0 || mouse_directions_.count(key) != 0, now);
  }
  if (!model_.deadman()) {
    current_linear_ = 0.0;
    current_angular_ = 0.0;
    publishZero();
  }
  updateLabels(current_linear_, current_angular_);
}

void TeleopPanel::publishTick()
{
  if (!publisher_) {return;}
  const double now = nowSeconds();
  const double raw_elapsed = now - last_tick_sec_;
  const double elapsed = std::clamp(raw_elapsed, 0.0, 0.25);
  last_tick_sec_ = now;
  if (model_.deadman() &&
    raw_elapsed * 1000.0 > static_cast<double>(command_timeout_->value()))
  {
    stopNow();
    return;
  }
  if (!mouse_directions_.empty() && !(QApplication::mouseButtons() & Qt::LeftButton)) {
    mouse_directions_.clear();
    applyDirectionState();
  }
  if (model_.deadman()) {
    model_.heartbeat(now);
  }
  auto target = model_.command(now);
  if (emergency_stop_) {
    target = {0.0, 0.0};
  }
  if (!model_.deadman() || emergency_stop_) {
    current_linear_ = 0.0;
    current_angular_ = 0.0;
  } else {
    current_linear_ = approach(current_linear_, target.first, acceleration_limit_ * elapsed);
    current_angular_ = approach(current_angular_, target.second, angular_acceleration_limit_ * elapsed);
  }
  geometry_msgs::msg::Twist message;
  message.linear.x = current_linear_;
  message.angular.z = current_angular_;
  publisher_->publish(message);
  updateLabels(current_linear_, current_angular_);
}

void TeleopPanel::publishZero()
{
  if (publisher_) {
    publisher_->publish(geometry_msgs::msg::Twist());
  }
  updateLabels(0.0, 0.0);
}

void TeleopPanel::stopNow()
{
  for (const char key : {'w', 'a', 's', 'd'}) {
    ++keyboard_generations_[key];
  }
  keyboard_directions_.clear();
  mouse_directions_.clear();
  model_.stop(nowSeconds());
  current_linear_ = 0.0;
  current_angular_ = 0.0;
  publishZero();
}

void TeleopPanel::updateLabels(double linear, double angular)
{
  linear_label_->setText(QString("linear.x %1 m/s").arg(linear, 0, 'f', 3));
  angular_label_->setText(QString("angular.z %1 rad/s").arg(angular, 0, 'f', 3));
  keys_label_->setText(QString("Keys %1").arg(QString::fromStdString(model_.activeKeys())));
  deadman_label_->setText(model_.deadman() ? "Deadman ACTIVE" : "Deadman RELEASED");
  source_label_->setText("Source keyboard/mouse");
}

void TeleopPanel::updateParameters()
{
  linear_speed_->setMaximum(linear_max_->value());
  angular_speed_->setMaximum(angular_max_->value());
  model_.setSpeeds(linear_speed_->value(), angular_speed_->value(), linear_max_->value(), angular_max_->value());
  model_.setTimeout(static_cast<double>(command_timeout_->value()) / 1000.0);
  acceleration_limit_ = acceleration_->value();
  angular_acceleration_limit_ = angular_acceleration_->value();
  if (publish_timer_) {
    publish_timer_->setInterval(1000 / command_rate_->value());
  }
}

void TeleopPanel::updatePublisher()
{
  const QString topic = topic_edit_->text().trimmed();
  if (topic != "/teleop/cmd_vel") {
    topic_edit_->setText("/teleop/cmd_vel");
    chain_label_->setText("Output is locked to /teleop/cmd_vel for the safety chain");
  }
  topic_ = "/teleop/cmd_vel";
  if (node_) {
    publishZero();
    publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(topic_.toStdString(), 10);
  }
}

bool TeleopPanel::eventFilter(QObject * object, QEvent * event)
{
  if (event->type() == QEvent::ApplicationDeactivate || event->type() == QEvent::WindowDeactivate) {
    stopNow();
  }
  if (event->type() == QEvent::MouseButtonRelease && !mouse_directions_.empty()) {
    mouse_directions_.clear();
    applyDirectionState();
  }
  if ((event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease) && isVisible()) {
    auto * key_event = static_cast<QKeyEvent *>(event);
    const bool pressed = event->type() == QEvent::KeyPress;
    switch (key_event->key()) {
      case Qt::Key_W:
        pressed ? setKeyboardDirection('w', true) : scheduleKeyboardRelease('w');
        return true;
      case Qt::Key_A:
        pressed ? setKeyboardDirection('a', true) : scheduleKeyboardRelease('a');
        return true;
      case Qt::Key_S:
        pressed ? setKeyboardDirection('s', true) : scheduleKeyboardRelease('s');
        return true;
      case Qt::Key_D:
        pressed ? setKeyboardDirection('d', true) : scheduleKeyboardRelease('d');
        return true;
      case Qt::Key_Space: if (pressed) {stopNow();} return true;
      default: break;
    }
  }
  return rviz_common::Panel::eventFilter(object, event);
}

void TeleopPanel::closeEvent(QCloseEvent * event)
{
  stopNow();
  rviz_common::Panel::closeEvent(event);
}

void TeleopPanel::hideEvent(QHideEvent * event)
{
  stopNow();
  rviz_common::Panel::hideEvent(event);
}

void TeleopPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  float number = 0.0F;
  if (config.mapGetFloat("LinearSpeedMps", &number)) {linear_speed_->setValue(number);}
  if (config.mapGetFloat("AngularSpeedRadps", &number)) {angular_speed_->setValue(number);}
  if (config.mapGetFloat("LinearSpeedMaxMps", &number)) {linear_max_->setValue(number);}
  if (config.mapGetFloat("AngularSpeedMaxRadps", &number)) {angular_max_->setValue(number);}
  if (config.mapGetFloat("AccelerationLimitMps2", &number)) {acceleration_->setValue(number);}
  if (config.mapGetFloat("AngularAccelerationLimitRadps2", &number)) {angular_acceleration_->setValue(number);}
  int integer = 0;
  if (config.mapGetInt("CommandRateHz", &integer)) {command_rate_->setValue(integer);}
  if (config.mapGetInt("CommandTimeoutMs", &integer)) {command_timeout_->setValue(integer);}
  topic_edit_->setText("/teleop/cmd_vel");
  updateParameters();
}

void TeleopPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", "/teleop/cmd_vel");
  config.mapSetValue("LinearSpeedMps", linear_speed_->value());
  config.mapSetValue("AngularSpeedRadps", angular_speed_->value());
  config.mapSetValue("LinearSpeedMaxMps", linear_max_->value());
  config.mapSetValue("AngularSpeedMaxRadps", angular_max_->value());
  config.mapSetValue("AccelerationLimitMps2", acceleration_->value());
  config.mapSetValue("AngularAccelerationLimitRadps2", angular_acceleration_->value());
  config.mapSetValue("CommandRateHz", command_rate_->value());
  config.mapSetValue("CommandTimeoutMs", command_timeout_->value());
}

}  // namespace smartwheel_rviz_plugins
