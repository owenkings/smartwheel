#ifndef SMARTWHEEL_RVIZ_PLUGINS__MAPPING_CONTROL_PANEL_HPP_
#define SMARTWHEEL_RVIZ_PLUGINS__MAPPING_CONTROL_PANEL_HPP_

#include <memory>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <smartwheel_interfaces/msg/workbench_status.hpp>
#include <smartwheel_interfaces/srv/workbench_command.hpp>

class QLabel;
class QLineEdit;
class QPushButton;

namespace smartwheel_rviz_plugins
{

class MappingControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MappingControlPanel(QWidget * parent = nullptr);
  ~MappingControlPanel() override;
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

Q_SIGNALS:
  void commandCompleted(bool accepted, const QString & state, const QString & reason, const QString & directory);

private Q_SLOTS:
  void applyCommandResult(bool accepted, const QString & state, const QString & reason, const QString & directory);
  void applyStatus(const smartwheel_interfaces::msg::WorkbenchStatus & status);

private:
  void addCommandButton(QLayout * layout, const QString & text, uint8_t command, bool confirmation = false);
  void callCommand(uint8_t command, bool confirmation);
  void setBusy(bool busy);
  void resetRosInterfaces();
  void updateButtonStates(const QString & state, bool paused = false);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<smartwheel_interfaces::srv::WorkbenchCommand>::SharedPtr client_;
  rclcpp::Subscription<smartwheel_interfaces::msg::WorkbenchStatus>::SharedPtr status_subscription_;
  QString service_name_{"/workbench/command"};
  QString status_topic_{"/workbench/status"};
  QLineEdit * map_name_edit_{nullptr};
  QLabel * state_label_{nullptr};
  QLabel * backend_label_{nullptr};
  QLabel * bag_label_{nullptr};
  QLabel * version_label_{nullptr};
  QLabel * elapsed_label_{nullptr};
  QLabel * trajectory_label_{nullptr};
  QLabel * keyframes_label_{nullptr};
  QLabel * loops_label_{nullptr};
  QLabel * points_label_{nullptr};
  QLabel * save_label_{nullptr};
  QLabel * quality_label_{nullptr};
  QLabel * failure_label_{nullptr};
  std::unordered_map<uint8_t, QPushButton *> buttons_;
  bool busy_{false};
};

}  // namespace smartwheel_rviz_plugins

#endif  // SMARTWHEEL_RVIZ_PLUGINS__MAPPING_CONTROL_PANEL_HPP_
