#ifndef SMARTWHEEL_RVIZ_PLUGINS__MAP_PRODUCTS_PANEL_HPP_
#define SMARTWHEEL_RVIZ_PLUGINS__MAP_PRODUCTS_PANEL_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <smartwheel_interfaces/srv/map_product_task.hpp>

class QComboBox;
class QLabel;
class QListWidget;
class QPushButton;

namespace smartwheel_rviz_plugins
{

class MapProductsPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MapProductsPanel(QWidget * parent = nullptr);
  ~MapProductsPanel() override;
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

Q_SIGNALS:
  void taskCompleted(
    bool accepted, const QString & reason, const QString & selected,
    const QStringList & versions, const QStringList & files, const QString & quality);

private Q_SLOTS:
  void requestList();
  void selectCurrent();
  void republishCloud();
  void republishMap();
  void preview();
  void openDirectory();
  void applyResult(
    bool accepted, const QString & reason, const QString & selected,
    const QStringList & versions, const QStringList & files, const QString & quality);

private:
  void call(uint8_t command);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<smartwheel_interfaces::srv::MapProductTask>::SharedPtr client_;
  QString service_name_{"/map_products/task"};
  QString selected_version_;
  QComboBox * versions_{nullptr};
  QListWidget * files_{nullptr};
  QLabel * quality_{nullptr};
  QLabel * status_{nullptr};
  QPushButton * refresh_button_{nullptr};
};

}  // namespace smartwheel_rviz_plugins

#endif  // SMARTWHEEL_RVIZ_PLUGINS__MAP_PRODUCTS_PANEL_HPP_
