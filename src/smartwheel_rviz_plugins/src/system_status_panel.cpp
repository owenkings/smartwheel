#include "smartwheel_rviz_plugins/system_status_panel.hpp"

#include "smartwheel_rviz_plugins/diagnostic_model.hpp"

#include <QColor>
#include <QHeaderView>
#include <QLabel>
#include <QScrollArea>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

#include <rviz_common/display_context.hpp>

#include <algorithm>
#include <array>

namespace smartwheel_rviz_plugins
{
namespace
{

const std::array<const char *, 18> kComponents = {
  "Left LiDAR", "Right LiDAR", "H30 IMU", "Wheel Odometry",
  "Front Camera", "Left Camera", "Right Camera", "Rear Camera",
  "FAST-LIO2", "RTAB-Map", "slam_toolbox", "TF", "rosbag", "disk",
  "mapping manager", "safety supervisor", "motor driver", "emergency stop"};

QColor colorForState(const QString & state)
{
  if (state == "ONLINE") {return QColor(32, 138, 69);}
  if (state == "MOCK") {return QColor(0, 102, 204);}
  if (state == "WARNING") {return QColor(178, 93, 0);}
  if (state == "DISABLED") {return QColor(102, 112, 133);}
  return QColor(180, 35, 24);
}

QString normalized(QString text)
{
  return text.toLower();
}

}  // namespace

SystemStatusPanel::SystemStatusPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * content = new QWidget;
  auto * layout = new QVBoxLayout(content);
  layout->setContentsMargins(4, 4, 4, 4);
  table_ = new QTableWidget(static_cast<int>(kComponents.size()), 3);
  table_->setHorizontalHeaderLabels({"Component", "State", "Evidence"});
  table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
  table_->verticalHeader()->setVisible(false);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_->setSelectionMode(QAbstractItemView::NoSelection);
  for (int row = 0; row < static_cast<int>(kComponents.size()); ++row) {
    table_->setItem(row, 0, new QTableWidgetItem(kComponents[row]));
    table_->setItem(row, 1, new QTableWidgetItem("DISABLED"));
    table_->setItem(row, 2, new QTableWidgetItem("no authoritative status"));
  }
  table_->setMinimumHeight(300);
  layout->addWidget(table_, 1);
  topic_metrics_ = new QLabel("Topic rates/latency: awaiting diagnostics");
  timing_metrics_ = new QLabel("Timestamp/TF/dual LiDAR delta: awaiting diagnostics");
  resource_metrics_ = new QLabel("CPU/memory/disk/bag rate: awaiting diagnostics");
  profile_metrics_ = new QLabel("Git/hardware profile/algorithm profile: awaiting diagnostics");
  for (auto * label : {topic_metrics_, timing_metrics_, resource_metrics_, profile_metrics_}) {
    label->setWordWrap(true);
    layout->addWidget(label);
  }
  auto * outer = new QVBoxLayout(this);
  auto * scroll = new QScrollArea;
  scroll->setWidgetResizable(true);
  scroll->setWidget(content);
  outer->addWidget(scroll);
  connect(this, &SystemStatusPanel::componentReady, this, &SystemStatusPanel::applyComponent, Qt::QueuedConnection);
  connect(this, &SystemStatusPanel::metricReady, this, &SystemStatusPanel::applyMetric, Qt::QueuedConnection);
}

SystemStatusPanel::~SystemStatusPanel()
{
  diagnostics_subscription_.reset();
  hardware_subscription_.reset();
  mapping_subscription_.reset();
  workbench_subscription_.reset();
}

void SystemStatusPanel::onInitialize()
{
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    applyComponent("mapping manager", "ERROR", "RViz ROS node unavailable");
    return;
  }
  node_ = abstraction->get_raw_node();
  diagnostics_subscription_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    diagnostics_topic_.toStdString(), rclcpp::QoS(50),
    std::bind(&SystemStatusPanel::onDiagnostics, this, std::placeholders::_1));
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  hardware_subscription_ = node_->create_subscription<smartwheel_interfaces::msg::HardwareStatus>(
    "/hardware/status", latched, std::bind(&SystemStatusPanel::onHardware, this, std::placeholders::_1));
  mapping_subscription_ = node_->create_subscription<smartwheel_interfaces::msg::MappingStatus>(
    "/mapping/status", latched, std::bind(&SystemStatusPanel::onMapping, this, std::placeholders::_1));
  workbench_subscription_ = node_->create_subscription<smartwheel_interfaces::msg::WorkbenchStatus>(
    "/workbench/status", latched,
    [this](const smartwheel_interfaces::msg::WorkbenchStatus::ConstSharedPtr message) {
      Q_EMIT metricReady("mapping", QString("loops=%1 keyframes=%2 points=%3 state=%4")
        .arg(message->loop_closure_count).arg(message->keyframe_count).arg(message->map_point_count)
        .arg(QString::fromStdString(message->state)));
      const QString active = QString::fromStdString(message->mapping_backend);
      Q_EMIT componentReady("RTAB-Map", active == "rtabmap" ? "MOCK" : "DISABLED", active);
      Q_EMIT componentReady("slam_toolbox", active == "slam_toolbox" ? "MOCK" : "DISABLED", active);
    });
}

QString SystemStatusPanel::componentForDiagnostic(const QString & name)
{
  const QString text = normalized(name);
  if (text.contains("front") && text.contains("camera")) {return "Front Camera";}
  if (text.contains("left") && text.contains("camera")) {return "Left Camera";}
  if (text.contains("right") && text.contains("camera")) {return "Right Camera";}
  if (text.contains("rear") && text.contains("camera")) {return "Rear Camera";}
  if (text.contains("left") && text.contains("lidar")) {return "Left LiDAR";}
  if (text.contains("right") && text.contains("lidar")) {return "Right LiDAR";}
  if (text.contains("imu")) {return "H30 IMU";}
  if (text.contains("lio")) {return "FAST-LIO2";}
  if (text.contains("wheel")) {return "Wheel Odometry";}
  if (text.contains("tf")) {return "TF";}
  if (text.contains("bag")) {return "rosbag";}
  if (text.contains("disk")) {return "disk";}
  if (text.contains("safety")) {return "safety supervisor";}
  if (text.contains("motor")) {return "motor driver";}
  if (text.contains("emergency") || text.contains("estop")) {return "emergency stop";}
  if (text.contains("mapping") || text.contains("workbench")) {return "mapping manager";}
  return {};
}

void SystemStatusPanel::onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message)
{
  for (const auto & status : message->status) {
    const QString component = componentForDiagnostic(QString::fromStdString(status.name));
    if (!component.isEmpty()) {
      const QString hardware_id = QString::fromStdString(status.hardware_id).toUpper();
      const QString detail = QString::fromStdString(status.message);
      const bool disabled = hardware_id == "DISABLED" || detail.contains("DISABLED", Qt::CaseInsensitive);
      const bool mock = hardware_id.contains("MOCK");
      const auto mapped = diagnosticLevelToStatus(status.level, mock, disabled);
      Q_EMIT componentReady(component, QString::fromStdString(statusText(mapped)), QString::fromStdString(status.message));
    }
    for (const auto & pair : status.values) {
      Q_EMIT metricReady(QString::fromStdString(pair.key), QString::fromStdString(pair.value));
    }
  }
}

void SystemStatusPanel::onHardware(const smartwheel_interfaces::msg::HardwareStatus::ConstSharedPtr message)
{
  const QString active = message->mode == "mock" ? "MOCK" : "ONLINE";
  const QString disabled = "DISABLED";
  const QString evidence = QString::fromStdString(message->detail);
  Q_EMIT componentReady("Left LiDAR", message->lidar_left_available ? active : disabled, evidence);
  Q_EMIT componentReady("Right LiDAR", message->lidar_right_available ? active : disabled, evidence);
  Q_EMIT componentReady("H30 IMU", message->imu_available ? active : disabled, evidence);
  Q_EMIT componentReady("Wheel Odometry", message->wheel_available ? active : disabled, evidence);
  for (const QString camera : {"Front Camera", "Left Camera", "Right Camera", "Rear Camera"}) {
    Q_EMIT componentReady(camera, message->cameras_available ? active : disabled, evidence);
  }
  Q_EMIT componentReady("motor driver", message->hardware_enabled ? "WARNING" : "DISABLED", "hardware_enabled=" + QString(message->hardware_enabled ? "true" : "false"));
}

void SystemStatusPanel::onMapping(const smartwheel_interfaces::msg::MappingStatus::ConstSharedPtr message)
{
  const bool failed = message->state == "FAILED";
  Q_EMIT componentReady("mapping manager", failed ? "ERROR" : "MOCK",
    failed ? QString::fromStdString(message->failure_reason) : QString::fromStdString(message->state));
  bool static_tf = false;
  bool dynamic_tf = false;
  for (const auto & check : message->checks) {
    const QString value = QString::fromStdString(check);
    static_tf = static_tf || value == "tf_static_complete=PASS";
    dynamic_tf = dynamic_tf || value == "backend_tf_dynamic=PASS";
  }
  Q_EMIT componentReady(
    "TF", static_tf && dynamic_tf ? "MOCK" : "WARNING",
    QString("static=%1 map_to_base=%2; mapping manager TF buffer")
    .arg(static_tf ? "PASS" : "FAIL").arg(dynamic_tf ? "PASS" : "FAIL"));
}

void SystemStatusPanel::applyComponent(const QString & component, const QString & state, const QString & detail)
{
  for (int row = 0; row < table_->rowCount(); ++row) {
    if (table_->item(row, 0)->text() == component) {
      table_->item(row, 1)->setText(state);
      table_->item(row, 1)->setForeground(colorForState(state));
      table_->item(row, 2)->setText(detail);
      return;
    }
  }
}

void SystemStatusPanel::applyMetric(const QString & key, const QString & value)
{
  const QString name = normalized(key);
  if (name.contains("rate") || name.contains("frequency") || name.contains("latency")) {
    topic_metrics_->setText(key + "=" + value);
  } else if (name.contains("stamp") || name.contains("time") || name.contains("tf") || name.contains("unit")) {
    timing_metrics_->setText(key + "=" + value);
  } else if (name.contains("cpu") || name.contains("memory") || name.contains("disk") || name.contains("bag")) {
    resource_metrics_->setText(key + "=" + value);
  } else if (name.contains("git") || name.contains("profile") || name == "mapping") {
    profile_metrics_->setText(key + "=" + value);
  }
}

void SystemStatusPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  config.mapGetString("DiagnosticsTopic", &diagnostics_topic_);
  if (node_) {
    diagnostics_subscription_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_.toStdString(), rclcpp::QoS(50),
      std::bind(&SystemStatusPanel::onDiagnostics, this, std::placeholders::_1));
  }
}

void SystemStatusPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("DiagnosticsTopic", diagnostics_topic_);
}

}  // namespace smartwheel_rviz_plugins
