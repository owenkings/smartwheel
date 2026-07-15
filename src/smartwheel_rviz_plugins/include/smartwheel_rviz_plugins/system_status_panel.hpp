#ifndef SMARTWHEEL_RVIZ_PLUGINS__SYSTEM_STATUS_PANEL_HPP_
#define SMARTWHEEL_RVIZ_PLUGINS__SYSTEM_STATUS_PANEL_HPP_

#include <memory>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <smartwheel_interfaces/msg/hardware_status.hpp>
#include <smartwheel_interfaces/msg/mapping_status.hpp>
#include <smartwheel_interfaces/msg/workbench_status.hpp>

class QLabel;
class QTableWidget;

namespace smartwheel_rviz_plugins
{

class SystemStatusPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SystemStatusPanel(QWidget * parent = nullptr);
  ~SystemStatusPanel() override;
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

Q_SIGNALS:
  void componentReady(const QString & component, const QString & state, const QString & detail);
  void metricReady(const QString & key, const QString & value);

private Q_SLOTS:
  void applyComponent(const QString & component, const QString & state, const QString & detail);
  void applyMetric(const QString & key, const QString & value);

private:
  void onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message);
  void onHardware(const smartwheel_interfaces::msg::HardwareStatus::ConstSharedPtr message);
  void onMapping(const smartwheel_interfaces::msg::MappingStatus::ConstSharedPtr message);
  static QString componentForDiagnostic(const QString & name);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_subscription_;
  rclcpp::Subscription<smartwheel_interfaces::msg::HardwareStatus>::SharedPtr hardware_subscription_;
  rclcpp::Subscription<smartwheel_interfaces::msg::MappingStatus>::SharedPtr mapping_subscription_;
  rclcpp::Subscription<smartwheel_interfaces::msg::WorkbenchStatus>::SharedPtr workbench_subscription_;
  QTableWidget * table_{nullptr};
  QLabel * topic_metrics_{nullptr};
  QLabel * timing_metrics_{nullptr};
  QLabel * resource_metrics_{nullptr};
  QLabel * profile_metrics_{nullptr};
  QString diagnostics_topic_{"/diagnostics"};
};

}  // namespace smartwheel_rviz_plugins

#endif  // SMARTWHEEL_RVIZ_PLUGINS__SYSTEM_STATUS_PANEL_HPP_

