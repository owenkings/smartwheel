#include "wheelchair_bringup/teleop_panel.hpp"

#include <rviz_common/display_context.hpp>

namespace wheelchair_bringup
{

TeleopPanel::TeleopPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * main_layout = new QVBoxLayout;

  // Topic + speed controls.
  auto * topic_layout = new QHBoxLayout;
  topic_layout->addWidget(new QLabel("Topic:"));
  topic_edit_ = new QLineEdit(topic_);
  topic_edit_->setToolTip(
    "Twist topic. Default /cmd_vel_nav routes through safety_supervisor.");
  topic_layout->addWidget(topic_edit_);
  main_layout->addLayout(topic_layout);

  auto * speed_layout = new QHBoxLayout;
  speed_layout->addWidget(new QLabel("v (m/s):"));
  linear_spin_ = new QDoubleSpinBox;
  linear_spin_->setRange(0.0, 1.0);
  linear_spin_->setSingleStep(0.05);
  linear_spin_->setValue(max_linear_);
  speed_layout->addWidget(linear_spin_);
  speed_layout->addWidget(new QLabel("w (rad/s):"));
  angular_spin_ = new QDoubleSpinBox;
  angular_spin_->setRange(0.0, 2.0);
  angular_spin_->setSingleStep(0.1);
  angular_spin_->setValue(max_angular_);
  speed_layout->addWidget(angular_spin_);
  main_layout->addLayout(speed_layout);

  // Direction buttons in a cross layout.
  auto * grid = new QGridLayout;
  forward_button_ = new QPushButton("Forward\n(W)");
  backward_button_ = new QPushButton("Back\n(S)");
  left_button_ = new QPushButton("Left\n(A)");
  right_button_ = new QPushButton("Right\n(D)");
  stop_button_ = new QPushButton("STOP\n(Space)");

  for (auto * b : {forward_button_, backward_button_, left_button_, right_button_}) {
    b->setMinimumHeight(48);
  }
  stop_button_->setMinimumHeight(48);
  stop_button_->setStyleSheet("background-color:#aa3030; color:white; font-weight:bold;");

  grid->addWidget(forward_button_, 0, 1);
  grid->addWidget(left_button_, 1, 0);
  grid->addWidget(stop_button_, 1, 1);
  grid->addWidget(right_button_, 1, 2);
  grid->addWidget(backward_button_, 2, 1);
  main_layout->addLayout(grid);

  state_label_ = new QLabel("STOPPED");
  state_label_->setAlignment(Qt::AlignCenter);
  state_label_->setStyleSheet("color:#80ff80; font-weight:bold; font-size:14px;");
  main_layout->addWidget(state_label_);

  // Explicit switching is opt-in for wheel/IMU mapping only. No auto-release
  // on key-up/STOP, and no command can drive through a pending/push mode.
  mode_widget_ = new QWidget;
  auto * mode_layout = new QVBoxLayout(mode_widget_);
  mode_layout->setContentsMargins(0, 0, 0, 0);
  auto * mode_buttons = new QHBoxLayout;
  push_mode_button_ = new QPushButton("手推建图");
  drive_mode_button_ = new QPushButton("WASD 建图");
  mode_buttons->addWidget(push_mode_button_);
  mode_buttons->addWidget(drive_mode_button_);
  mode_layout->addLayout(mode_buttons);
  mode_label_ = new QLabel("等待底盘模式状态");
  mode_label_->setWordWrap(true);
  mode_layout->addWidget(mode_label_);
  mode_widget_->hide();
  main_layout->addWidget(mode_widget_);
  connect(push_mode_button_, &QPushButton::clicked, this, [this]() {requestMappingMode(true);});
  connect(drive_mode_button_, &QPushButton::clicked, this, [this]() {requestMappingMode(false);});

  auto * note = new QLabel(
    "Motion flows through safety_supervisor.\n"
    "Motors move only if base runs with motion_control_enabled:=true.");
  note->setWordWrap(true);
  note->setStyleSheet("color:#aaaaaa; font-size:10px;");
  main_layout->addWidget(note);

  setLayout(main_layout);

  // Buttons set their direction flag while pressed and clear it on release.
  // Multiple held directions combine (e.g. Forward+Left = forward-left arc).
  connect(forward_button_, &QPushButton::pressed, this, [this]() {setDir(true, false, false, false, true);});
  connect(forward_button_, &QPushButton::released, this, [this]() {setDir(true, false, false, false, false);});
  connect(backward_button_, &QPushButton::pressed, this, [this]() {setDir(false, true, false, false, true);});
  connect(backward_button_, &QPushButton::released, this, [this]() {setDir(false, true, false, false, false);});
  connect(left_button_, &QPushButton::pressed, this, [this]() {setDir(false, false, true, false, true);});
  connect(left_button_, &QPushButton::released, this, [this]() {setDir(false, false, true, false, false);});
  connect(right_button_, &QPushButton::pressed, this, [this]() {setDir(false, false, false, true, true);});
  connect(right_button_, &QPushButton::released, this, [this]() {setDir(false, false, false, true, false);});
  connect(stop_button_, &QPushButton::clicked, this, &TeleopPanel::stop);
  connect(linear_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this, &TeleopPanel::updateSpeeds);
  connect(angular_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this, &TeleopPanel::updateSpeeds);
  connect(topic_edit_, &QLineEdit::editingFinished, this, [this]() {
      const QString t = topic_edit_->text().trimmed();
      if (!t.isEmpty() && t != topic_ && node_) {
        topic_ = t;
        publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(
          topic_.toStdString(), 10);
      }
    });

  // Capture W/A/S/D and Space when the panel (or its children) has focus.
  setFocusPolicy(Qt::StrongFocus);
  qApp->installEventFilter(this);
}

TeleopPanel::~TeleopPanel()
{
  qApp->removeEventFilter(this);
  if (publish_timer_) {publish_timer_->stop();}
  stop();
}

void TeleopPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    topic_.toStdString(), 10);
  mode_client_ = node_->create_client<std_srvs::srv::SetBool>("/base/set_mapping_push_mode");
  QPointer<TeleopPanel> panel(this);
  mode_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/base/mapping_drive_mode", 10,
    [panel](std_msgs::msg::String::ConstSharedPtr msg) {
      if (!panel) {return;}
      const QString value = QString::fromStdString(msg->data);
      QMetaObject::invokeMethod(panel.data(), [panel, value]() {
        if (!panel) {return;}
        panel->backend_mode_ = value;
        panel->mode_received_.restart();
        if (!panel->mappingDriveAllowed()) {panel->stop();}
        if (!panel->mode_pending_) {
          panel->mode_label_->setText("底盘模式: " + value + "\n" + panel->mode_result_);
        }
      }, Qt::QueuedConnection);
    });

  publish_timer_ = new QTimer(this);
  connect(publish_timer_, &QTimer::timeout, this, &TeleopPanel::publishCommand);
  publish_timer_->start(66);  // ~15 Hz
}

void TeleopPanel::updateSpeeds()
{
  max_linear_ = linear_spin_->value();
  max_angular_ = angular_spin_->value();
  recomputeTarget();
}

void TeleopPanel::setDir(bool forward, bool backward, bool left, bool right, bool held)
{
  if (held && !mappingDriveAllowed()) {stop(); return;}
  if (forward) {fwd_ = held;}
  if (backward) {back_ = held;}
  if (left) {left_ = held;}
  if (right) {right_ = held;}
  recomputeTarget();
}

void TeleopPanel::recomputeTarget()
{
  double lin = 0.0;
  if (fwd_) {lin += 1.0;}
  if (back_) {lin -= 1.0;}
  double ang = 0.0;
  if (left_) {ang += 1.0;}
  if (right_) {ang -= 1.0;}

  target_linear_ = lin * max_linear_;
  target_angular_ = ang * max_angular_;

  QString label;
  if (lin == 0.0 && ang == 0.0) {
    label = "STOPPED";
    state_label_->setStyleSheet("color:#80ff80; font-weight:bold; font-size:14px;");
  } else {
    QString lin_s = lin > 0 ? "FWD" : (lin < 0 ? "BACK" : "");
    QString ang_s = ang > 0 ? "LEFT" : (ang < 0 ? "RIGHT" : "");
    label = (lin_s + " " + ang_s).trimmed();
    state_label_->setStyleSheet("color:#ffd000; font-weight:bold; font-size:14px;");
  }
  state_label_->setText(label);
}

void TeleopPanel::stop()
{
  fwd_ = back_ = left_ = right_ = false;
  target_linear_ = 0.0;
  target_angular_ = 0.0;
  state_label_->setText("STOPPED");
  state_label_->setStyleSheet("color:#80ff80; font-weight:bold; font-size:14px;");
  if (publisher_) {publisher_->publish(geometry_msgs::msg::Twist());}
}

bool TeleopPanel::mappingDriveAllowed() const
{
  return !enable_push_mode_ || (drive_ack_ && !mode_pending_ &&
    // Normal driver enable has two existing 200 ms settling waits. Allow that
    // heartbeat gap; command release/STOP and the safety watchdog stay immediate.
    backend_mode_ == "drive" && mode_received_.isValid() && mode_received_.elapsed() <= 1000);
}

void TeleopPanel::requestMappingMode(bool push)
{
  if (!enable_push_mode_ || mode_pending_) {return;}
  stop();
  drive_ack_ = false;
  if (!mode_client_ || !mode_client_->service_is_ready()) {
    mode_result_ = "切换服务不可用；已停止发送运动指令";
    mode_label_->setText(mode_result_);
    return;
  }
  mode_pending_ = true;
  const unsigned int generation = ++mode_generation_;
  mode_label_->setText("正在切换；请保持轮椅静止");
  QPointer<TeleopPanel> panel(this);
  // Let the zero command traverse the existing safety chain before requesting
  // release. The driver independently checks fresh stationary encoder data.
  QTimer::singleShot(400, this, [panel, generation, push]() {
    if (!panel || generation != panel->mode_generation_) {return;}
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = push;
    try {
      panel->mode_client_->async_send_request(request,
      [panel, generation, push](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        if (!panel) {return;}
        std_srvs::srv::SetBool::Response::SharedPtr response;
        try {
          response = future.get();
        } catch (const std::exception & error) {
          response = std::make_shared<std_srvs::srv::SetBool::Response>();
          response->success = false;
          response->message = std::string("Mode service failed: ") + error.what();
        }
        QMetaObject::invokeMethod(panel.data(), [panel, generation, push, response]() {
          if (!panel || generation != panel->mode_generation_) {return;}
          panel->mode_pending_ = false;
          panel->drive_ack_ = response->success && !push;
          panel->stop();
          panel->mode_result_ = QString::fromStdString(response->message);
          panel->mode_label_->setText(panel->mode_result_);
          panel->mode_label_->setToolTip(QString::fromStdString(response->message));
        }, Qt::QueuedConnection);
      });
    } catch (const std::exception & error) {
      panel->mode_pending_ = false;
      panel->drive_ack_ = false;
      panel->mode_result_ = QString("Mode request failed: ") + error.what();
      panel->mode_label_->setText(panel->mode_result_);
      panel->stop();
    }
  });
  QTimer::singleShot(6000, this, [panel, generation]() {
    if (panel && panel->mode_pending_ && generation == panel->mode_generation_) {
      ++panel->mode_generation_;
      panel->mode_pending_ = false;
      panel->drive_ack_ = false;
      panel->stop();
      panel->mode_result_ = "切换超时，实际模式未知；保持静止并重新选择模式";
      panel->mode_label_->setText(panel->mode_result_);
    }
  });
}

void TeleopPanel::publishCommand()
{
  if (!publisher_) {
    return;
  }
  geometry_msgs::msg::Twist msg;
  if (!mappingDriveAllowed()) {
    stop();
    if (enable_push_mode_ && mode_received_.isValid() && mode_received_.elapsed() > 1000) {
      mode_label_->setText("底盘状态已超时；运动指令已停止");
    }
  }
  msg.linear.x = target_linear_;
  msg.angular.z = target_angular_;
  publisher_->publish(msg);
}

bool TeleopPanel::eventFilter(QObject * object, QEvent * event)
{
  if (event->type() == QEvent::ApplicationDeactivate ||
    (object == this && (event->type() == QEvent::Hide || event->type() == QEvent::Close)))
  {
    stop();
  }
  if (event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease) {
    if (!isVisible()) {
      return rviz_common::Panel::eventFilter(object, event);
    }
    auto * key_event = static_cast<QKeyEvent *>(event);
    if (key_event->isAutoRepeat()) {
      return false;
    }
    const bool pressed = (event->type() == QEvent::KeyPress);
    switch (key_event->key()) {
      case Qt::Key_W:
        setDir(true, false, false, false, pressed);
        return true;
      case Qt::Key_S:
        setDir(false, true, false, false, pressed);
        return true;
      case Qt::Key_A:
        setDir(false, false, true, false, pressed);
        return true;
      case Qt::Key_D:
        setDir(false, false, false, true, pressed);
        return true;
      case Qt::Key_Space:
        if (pressed) {
          stop();
        }
        return true;
      default:
        break;
    }
  }
  return rviz_common::Panel::eventFilter(object, event);
}

void TeleopPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  config.mapGetBool("EnablePushMode", &enable_push_mode_);
  mode_widget_->setVisible(enable_push_mode_);
  QString topic;
  if (config.mapGetString("Topic", &topic) && !topic.isEmpty()) {
    topic_ = topic;
    topic_edit_->setText(topic_);
  }
  float v;
  if (config.mapGetFloat("Linear", &v)) {
    max_linear_ = v;
    linear_spin_->setValue(v);
  }
  if (config.mapGetFloat("Angular", &v)) {
    max_angular_ = v;
    angular_spin_->setValue(v);
  }
}

void TeleopPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", topic_);
  config.mapSetValue("Linear", max_linear_);
  config.mapSetValue("Angular", max_angular_);
  config.mapSetValue("EnablePushMode", enable_push_mode_);
}

}  // namespace wheelchair_bringup

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(wheelchair_bringup::TeleopPanel, rviz_common::Panel)
