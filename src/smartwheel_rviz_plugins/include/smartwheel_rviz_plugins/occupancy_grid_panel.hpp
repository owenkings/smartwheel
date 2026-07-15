#ifndef SMARTWHEEL_RVIZ_PLUGINS__OCCUPANCY_GRID_PANEL_HPP_
#define SMARTWHEEL_RVIZ_PLUGINS__OCCUPANCY_GRID_PANEL_HPP_

#include <QImage>
#include <QPointF>
#include <QPolygonF>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <smartwheel_interfaces/msg/mapping_status.hpp>

class QCheckBox;
class QGraphicsEllipseItem;
class QGraphicsLineItem;
class QGraphicsPathItem;
class QGraphicsPixmapItem;
class QGraphicsScene;
class QGraphicsView;
class QLabel;
class QLineEdit;
class QTimer;

namespace smartwheel_rviz_plugins
{

class OccupancyGridPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit OccupancyGridPanel(QWidget * parent = nullptr);
  ~OccupancyGridPanel() override;
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

Q_SIGNALS:
  void mapReady(const QImage & image, int width, int height, double resolution, double stamp_sec);
  void mapFailed(const QString & reason);
  void robotReady(const QPointF & point, double yaw);
  void pathReady(const QPolygonF & points);
  void mappingStateReady(const QString & state);

private Q_SLOTS:
  void applyMap(const QImage & image, int width, int height, double resolution, double stamp_sec);
  void applyMapFailure(const QString & reason);
  void applyRobot(const QPointF & point, double yaw);
  void applyPath(const QPolygonF & points);
  void updateFreshness();
  void resubscribe();

private:
  void onMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message);
  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr message);
  void onPath(const nav_msgs::msg::Path::ConstSharedPtr message);
  QPointF worldToScene(double x, double y) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<smartwheel_interfaces::msg::MappingStatus>::SharedPtr status_subscription_;
  std::atomic_bool map_update_pending_{false};
  mutable std::mutex geometry_mutex_;
  nav_msgs::msg::MapMetaData map_info_;
  bool map_valid_{false};
  double last_map_wall_sec_{0.0};
  double stale_threshold_sec_{3.0};

  QString map_topic_{"/map"};
  QString odom_topic_{"/odom/fused"};
  QString path_topic_{"/mapping/optimized_path"};
  QString status_topic_{"/mapping/status"};
  QLineEdit * map_topic_edit_{nullptr};
  QLineEdit * odom_topic_edit_{nullptr};
  QLineEdit * path_topic_edit_{nullptr};
  QCheckBox * show_origin_{nullptr};
  QGraphicsView * view_{nullptr};
  QGraphicsScene * scene_{nullptr};
  QGraphicsPixmapItem * map_item_{nullptr};
  QGraphicsEllipseItem * robot_item_{nullptr};
  QGraphicsLineItem * heading_item_{nullptr};
  QGraphicsPathItem * path_item_{nullptr};
  QGraphicsLineItem * origin_x_item_{nullptr};
  QGraphicsLineItem * origin_y_item_{nullptr};
  QLabel * status_label_{nullptr};
  QLabel * dimensions_label_{nullptr};
  QLabel * zoom_label_{nullptr};
  QLabel * mapping_label_{nullptr};
  QTimer * freshness_timer_{nullptr};
};

}  // namespace smartwheel_rviz_plugins

#endif  // SMARTWHEEL_RVIZ_PLUGINS__OCCUPANCY_GRID_PANEL_HPP_
