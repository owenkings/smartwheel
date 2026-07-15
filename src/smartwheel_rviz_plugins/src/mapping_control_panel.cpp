#include "smartwheel_rviz_plugins/mapping_control_panel.hpp"

#include <QFormLayout>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>

#include <rviz_common/display_context.hpp>

namespace smartwheel_rviz_plugins
{

MappingControlPanel::MappingControlPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * content = new QWidget;
  auto * layout = new QVBoxLayout(content);
  layout->setContentsMargins(4, 4, 4, 4);
  auto * name_form = new QFormLayout;
  map_name_edit_ = new QLineEdit("workbench_map");
  name_form->addRow("Map name", map_name_edit_);
  layout->addLayout(name_form);

  auto * commands = new QGridLayout;
  addCommandButton(commands, "Preflight Check", smartwheel_interfaces::srv::WorkbenchCommand::Request::PREFLIGHT);
  addCommandButton(commands, "Start Recording", smartwheel_interfaces::srv::WorkbenchCommand::Request::START_RECORDING);
  addCommandButton(commands, "Start Mapping", smartwheel_interfaces::srv::WorkbenchCommand::Request::START_MAPPING);
  addCommandButton(commands, "Pause Mapping", smartwheel_interfaces::srv::WorkbenchCommand::Request::PAUSE_MAPPING);
  addCommandButton(commands, "Resume Mapping", smartwheel_interfaces::srv::WorkbenchCommand::Request::RESUME_MAPPING);
  addCommandButton(commands, "Finish Mapping", smartwheel_interfaces::srv::WorkbenchCommand::Request::FINISH_MAPPING);
  addCommandButton(commands, "Optimize", smartwheel_interfaces::srv::WorkbenchCommand::Request::OPTIMIZE);
  addCommandButton(commands, "Save Map", smartwheel_interfaces::srv::WorkbenchCommand::Request::SAVE_MAP);
  addCommandButton(commands, "Export Products", smartwheel_interfaces::srv::WorkbenchCommand::Request::EXPORT_PRODUCTS);
  addCommandButton(commands, "Preview Saved Map", smartwheel_interfaces::srv::WorkbenchCommand::Request::PREVIEW_SAVED_MAP);
  addCommandButton(commands, "Cancel", smartwheel_interfaces::srv::WorkbenchCommand::Request::CANCEL, true);
  addCommandButton(commands, "Reset Session", smartwheel_interfaces::srv::WorkbenchCommand::Request::RESET_SESSION, true);
  layout->addLayout(commands);

  auto * status = new QFormLayout;
  state_label_ = new QLabel("IDLE");
  backend_label_ = new QLabel("--");
  bag_label_ = new QLabel("NOT RECORDING");
  version_label_ = new QLabel("--");
  elapsed_label_ = new QLabel("0.0 s");
  trajectory_label_ = new QLabel("0.00 m");
  keyframes_label_ = new QLabel("0");
  loops_label_ = new QLabel("0");
  points_label_ = new QLabel("0");
  save_label_ = new QLabel("NOT SAVED");
  quality_label_ = new QLabel("NOT RUN");
  failure_label_ = new QLabel("--");
  failure_label_->setWordWrap(true);
  status->addRow("State", state_label_);
  status->addRow("Backend", backend_label_);
  status->addRow("Bag", bag_label_);
  status->addRow("Version", version_label_);
  status->addRow("Elapsed", elapsed_label_);
  status->addRow("Trajectory", trajectory_label_);
  status->addRow("Keyframes", keyframes_label_);
  status->addRow("Loop closures", loops_label_);
  status->addRow("Map points", points_label_);
  status->addRow("Save", save_label_);
  status->addRow("Quality", quality_label_);
  status->addRow("Failure", failure_label_);
  layout->addLayout(status);

  auto * outer = new QVBoxLayout(this);
  auto * scroll = new QScrollArea;
  scroll->setWidgetResizable(true);
  scroll->setWidget(content);
  outer->addWidget(scroll);
  connect(this, &MappingControlPanel::commandCompleted, this, &MappingControlPanel::applyCommandResult, Qt::QueuedConnection);
  updateButtonStates("IDLE");
}

MappingControlPanel::~MappingControlPanel()
{
  status_subscription_.reset();
  client_.reset();
}

void MappingControlPanel::addCommandButton(QLayout * layout, const QString & text, uint8_t command, bool confirmation)
{
  auto * grid = qobject_cast<QGridLayout *>(layout);
  auto * button = new QPushButton(text);
  button->setMinimumHeight(30);
  const int index = static_cast<int>(buttons_.size());
  grid->addWidget(button, index / 2, index % 2);
  buttons_[command] = button;
  connect(button, &QPushButton::clicked, this, [this, command, confirmation]() {callCommand(command, confirmation);});
}

void MappingControlPanel::onInitialize()
{
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    applyCommandResult(false, "FAILED", "RViz ROS node unavailable", "");
    return;
  }
  node_ = abstraction->get_raw_node();
  resetRosInterfaces();
}

void MappingControlPanel::resetRosInterfaces()
{
  client_ = node_->create_client<smartwheel_interfaces::srv::WorkbenchCommand>(service_name_.toStdString());
  status_subscription_ = node_->create_subscription<smartwheel_interfaces::msg::WorkbenchStatus>(
    status_topic_.toStdString(), rclcpp::QoS(1).reliable().transient_local(),
    [this](const smartwheel_interfaces::msg::WorkbenchStatus::ConstSharedPtr message) {
      const auto status = *message;
      QMetaObject::invokeMethod(this, [this, status]() {applyStatus(status);}, Qt::QueuedConnection);
    });
}

void MappingControlPanel::callCommand(uint8_t command, bool confirmation)
{
  if (busy_) {return;}
  if (confirmation && QMessageBox::question(
      this, "Confirm operation", "This operation stops or clears the current mock session. Continue?",
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
  {
    return;
  }
  if (!client_ || !client_->service_is_ready()) {
    applyCommandResult(false, "FAILED", "workbench command service is unavailable", "");
    return;
  }
  auto request = std::make_shared<smartwheel_interfaces::srv::WorkbenchCommand::Request>();
  request->command = command;
  request->map_name = map_name_edit_->text().trimmed().toStdString();
  setBusy(true);
  client_->async_send_request(request,
    [this](rclcpp::Client<smartwheel_interfaces::srv::WorkbenchCommand>::SharedFuture future) {
      try {
        const auto response = future.get();
        Q_EMIT commandCompleted(
          response->accepted, QString::fromStdString(response->state),
          QString::fromStdString(response->reason), QString::fromStdString(response->version_directory));
      } catch (const std::exception & error) {
        Q_EMIT commandCompleted(false, "FAILED", QString::fromStdString(error.what()), "");
      }
    });
}

void MappingControlPanel::applyCommandResult(
  bool accepted, const QString & state, const QString & reason, const QString & directory)
{
  setBusy(false);
  state_label_->setText(state);
  failure_label_->setText(accepted ? reason : "REJECTED: " + reason);
  failure_label_->setStyleSheet(accepted ? "color:#208a45;" : "color:#b42318;font-weight:600;");
  if (!directory.isEmpty()) {version_label_->setText(directory);}
  updateButtonStates(state);
}

void MappingControlPanel::applyStatus(const smartwheel_interfaces::msg::WorkbenchStatus & status)
{
  const QString state = QString::fromStdString(status.state);
  state_label_->setText(status.paused ? state + " (PAUSED)" : state);
  map_name_edit_->setText(QString::fromStdString(status.map_name));
  backend_label_->setText(QString::fromStdString(status.mapping_backend));
  bag_label_->setText(status.recording ? QString::fromStdString(status.bag_path) : "NOT RECORDING");
  version_label_->setText(QString::fromStdString(status.version_directory));
  elapsed_label_->setText(QString("%1 s").arg(status.elapsed_sec, 0, 'f', 1));
  trajectory_label_->setText(QString("%1 m").arg(status.trajectory_length_m, 0, 'f', 2));
  keyframes_label_->setText(QString::number(status.keyframe_count));
  loops_label_->setText(QString::number(status.loop_closure_count));
  points_label_->setText(QString::number(status.map_point_count));
  save_label_->setText(status.map_saved ? "SAVED" : "NOT SAVED");
  quality_label_->setText(QString::fromStdString(status.quality_result));
  failure_label_->setText(status.failure_reason.empty() ? "--" : QString::fromStdString(status.failure_reason));
  updateButtonStates(state, status.paused);
}

void MappingControlPanel::setBusy(bool busy)
{
  busy_ = busy;
  for (auto & item : buttons_) {item.second->setEnabled(!busy);}
}

void MappingControlPanel::updateButtonStates(const QString & state, bool paused)
{
  if (busy_) {return;}
  for (auto & item : buttons_) {item.second->setEnabled(false);}
  auto enable = [this](uint8_t command) {buttons_.at(command)->setEnabled(true);};
  using Request = smartwheel_interfaces::srv::WorkbenchCommand::Request;
  if (state == "IDLE" || state == "FAILED") {
    enable(Request::PREFLIGHT); enable(Request::RESET_SESSION);
  } else if (state == "CHECKING") {
    enable(Request::START_RECORDING); enable(Request::START_MAPPING); enable(Request::CANCEL);
  } else if (state == "RECORDING") {
    enable(Request::START_MAPPING); enable(Request::CANCEL);
  } else if (state == "MAPPING") {
    enable(paused ? Request::RESUME_MAPPING : Request::PAUSE_MAPPING);
    enable(Request::FINISH_MAPPING); enable(Request::CANCEL);
  } else if (state == "LOOP_CLOSING") {
    enable(Request::OPTIMIZE); enable(Request::CANCEL);
  } else if (state == "OPTIMIZING") {
    enable(Request::SAVE_MAP); enable(Request::EXPORT_PRODUCTS); enable(Request::CANCEL);
  } else if (state == "READY") {
    enable(Request::PREVIEW_SAVED_MAP); enable(Request::RESET_SESSION); enable(Request::EXPORT_PRODUCTS);
  }
}

void MappingControlPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString map_name;
  if (config.mapGetString("MapName", &map_name) && !map_name.isEmpty()) {map_name_edit_->setText(map_name);}
  config.mapGetString("CommandService", &service_name_);
  config.mapGetString("StatusTopic", &status_topic_);
  if (node_) {
    resetRosInterfaces();
  }
}

void MappingControlPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("MapName", map_name_edit_->text());
  config.mapSetValue("CommandService", service_name_);
  config.mapSetValue("StatusTopic", status_topic_);
}

}  // namespace smartwheel_rviz_plugins
