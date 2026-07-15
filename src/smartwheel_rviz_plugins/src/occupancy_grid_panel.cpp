#include "smartwheel_rviz_plugins/occupancy_grid_panel.hpp"

#include "smartwheel_rviz_plugins/occupancy_grid_model.hpp"

#include <QCheckBox>
#include <QDateTime>
#include <QFormLayout>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsPathItem>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPainterPath>
#include <QScrollArea>
#include <QTimer>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <rviz_common/display_context.hpp>

#include <cmath>

namespace smartwheel_rviz_plugins
{
namespace
{

class MapGraphicsView : public QGraphicsView
{
public:
  explicit MapGraphicsView(QGraphicsScene * scene, QWidget * parent = nullptr)
  : QGraphicsView(scene, parent)
  {
    setDragMode(QGraphicsView::ScrollHandDrag);
    setRenderHint(QPainter::Antialiasing, true);
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  }

protected:
  void wheelEvent(QWheelEvent * event) override
  {
    const double factor = event->angleDelta().y() > 0 ? 1.15 : 1.0 / 1.15;
    scale(factor, factor);
    event->accept();
  }

  void mouseDoubleClickEvent(QMouseEvent * event) override
  {
    fitInView(scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
    event->accept();
  }
};

double wallSeconds()
{
  return static_cast<double>(QDateTime::currentMSecsSinceEpoch()) / 1000.0;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

OccupancyGridPanel::OccupancyGridPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * content = new QWidget;
  auto * layout = new QVBoxLayout(content);
  layout->setContentsMargins(4, 4, 4, 4);

  auto * topics = new QFormLayout;
  map_topic_edit_ = new QLineEdit(map_topic_);
  odom_topic_edit_ = new QLineEdit(odom_topic_);
  path_topic_edit_ = new QLineEdit(path_topic_);
  topics->addRow("Map", map_topic_edit_);
  topics->addRow("Robot", odom_topic_edit_);
  topics->addRow("Path", path_topic_edit_);
  layout->addLayout(topics);

  scene_ = new QGraphicsScene(this);
  view_ = new MapGraphicsView(scene_);
  view_->setMinimumSize(240, 200);
  layout->addWidget(view_, 1);
  map_item_ = scene_->addPixmap(QPixmap());
  path_item_ = scene_->addPath(QPainterPath(), QPen(QColor(0, 155, 255), 2.0));
  robot_item_ = scene_->addEllipse(-5, -5, 10, 10, QPen(Qt::red, 2), QBrush(QColor(255, 80, 80)));
  heading_item_ = scene_->addLine(0, 0, 12, 0, QPen(Qt::red, 3));
  origin_x_item_ = scene_->addLine(0, 0, 18, 0, QPen(Qt::red, 2));
  origin_y_item_ = scene_->addLine(0, 0, 0, -18, QPen(Qt::green, 2));
  robot_item_->setVisible(false);
  heading_item_->setVisible(false);

  show_origin_ = new QCheckBox("Show map origin");
  show_origin_->setChecked(false);
  origin_x_item_->setVisible(false);
  origin_y_item_->setVisible(false);
  layout->addWidget(show_origin_);
  status_label_ = new QLabel("NO MAP DATA");
  dimensions_label_ = new QLabel("0 x 0 | 0.000 m/cell");
  zoom_label_ = new QLabel("Zoom 100%");
  mapping_label_ = new QLabel("Mapping: UNKNOWN");
  layout->addWidget(status_label_);
  layout->addWidget(dimensions_label_);
  layout->addWidget(zoom_label_);
  layout->addWidget(mapping_label_);

  auto * outer = new QVBoxLayout(this);
  auto * scroll = new QScrollArea;
  scroll->setWidgetResizable(true);
  scroll->setWidget(content);
  outer->addWidget(scroll);

  connect(map_topic_edit_, &QLineEdit::editingFinished, this, &OccupancyGridPanel::resubscribe);
  connect(odom_topic_edit_, &QLineEdit::editingFinished, this, &OccupancyGridPanel::resubscribe);
  connect(path_topic_edit_, &QLineEdit::editingFinished, this, &OccupancyGridPanel::resubscribe);
  connect(show_origin_, &QCheckBox::toggled, this, [this](bool visible) {
    origin_x_item_->setVisible(visible);
    origin_y_item_->setVisible(visible);
  });
  connect(this, &OccupancyGridPanel::mapReady, this, &OccupancyGridPanel::applyMap, Qt::QueuedConnection);
  connect(this, &OccupancyGridPanel::mapFailed, this, &OccupancyGridPanel::applyMapFailure, Qt::QueuedConnection);
  connect(this, &OccupancyGridPanel::robotReady, this, &OccupancyGridPanel::applyRobot, Qt::QueuedConnection);
  connect(this, &OccupancyGridPanel::pathReady, this, &OccupancyGridPanel::applyPath, Qt::QueuedConnection);
  connect(this, &OccupancyGridPanel::mappingStateReady, mapping_label_, &QLabel::setText, Qt::QueuedConnection);

  freshness_timer_ = new QTimer(this);
  connect(freshness_timer_, &QTimer::timeout, this, &OccupancyGridPanel::updateFreshness);
  freshness_timer_->start(500);
}

OccupancyGridPanel::~OccupancyGridPanel()
{
  map_subscription_.reset();
  odom_subscription_.reset();
  path_subscription_.reset();
  status_subscription_.reset();
}

void OccupancyGridPanel::onInitialize()
{
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    applyMapFailure("RViz ROS node unavailable");
    return;
  }
  node_ = abstraction->get_raw_node();
  resubscribe();
}

void OccupancyGridPanel::resubscribe()
{
  map_topic_ = map_topic_edit_->text().trimmed();
  odom_topic_ = odom_topic_edit_->text().trimmed();
  path_topic_ = path_topic_edit_->text().trimmed();
  if (!node_ || map_topic_.isEmpty() || odom_topic_.isEmpty() || path_topic_.isEmpty()) {
    return;
  }
  auto map_qos = rclcpp::QoS(1).reliable().transient_local();
  map_subscription_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_.toStdString(), map_qos,
    std::bind(&OccupancyGridPanel::onMap, this, std::placeholders::_1));
  odom_subscription_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_.toStdString(), rclcpp::QoS(10),
    std::bind(&OccupancyGridPanel::onOdom, this, std::placeholders::_1));
  path_subscription_ = node_->create_subscription<nav_msgs::msg::Path>(
    path_topic_.toStdString(), rclcpp::QoS(5),
    std::bind(&OccupancyGridPanel::onPath, this, std::placeholders::_1));
  status_subscription_ = node_->create_subscription<smartwheel_interfaces::msg::MappingStatus>(
    status_topic_.toStdString(), map_qos,
    [this](const smartwheel_interfaces::msg::MappingStatus::ConstSharedPtr message) {
      Q_EMIT mappingStateReady(QString("Mapping: %1").arg(QString::fromStdString(message->state)));
    });
}

void OccupancyGridPanel::onMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message)
{
  if (map_update_pending_.exchange(true)) {
    return;
  }
  OccupancyGridModel model;
  std::string error;
  if (!model.setMap(*message, &error)) {
    map_update_pending_ = false;
    Q_EMIT mapFailed(QString::fromStdString(error));
    return;
  }
  {
    std::lock_guard<std::mutex> lock(geometry_mutex_);
    map_info_ = message->info;
    map_valid_ = true;
  }
  const double stamp = static_cast<double>(message->header.stamp.sec) +
    static_cast<double>(message->header.stamp.nanosec) * 1e-9;
  Q_EMIT mapReady(model.image(), static_cast<int>(model.width()), static_cast<int>(model.height()),
    model.resolution(), stamp);
}

QPointF OccupancyGridPanel::worldToScene(double x, double y) const
{
  std::lock_guard<std::mutex> lock(geometry_mutex_);
  if (!map_valid_) {
    return QPointF();
  }
  const auto & q = map_info_.origin.orientation;
  const double yaw = yawFromQuaternion(q);
  const double dx = x - map_info_.origin.position.x;
  const double dy = y - map_info_.origin.position.y;
  const double lx = std::cos(yaw) * dx + std::sin(yaw) * dy;
  const double ly = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  return QPointF(lx / map_info_.resolution, map_info_.height - ly / map_info_.resolution);
}

void OccupancyGridPanel::onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  {
    std::lock_guard<std::mutex> lock(geometry_mutex_);
    if (!map_valid_) {
      return;
    }
  }
  const auto point = worldToScene(message->pose.pose.position.x, message->pose.pose.position.y);
  Q_EMIT robotReady(point, yawFromQuaternion(message->pose.pose.orientation));
}

void OccupancyGridPanel::onPath(const nav_msgs::msg::Path::ConstSharedPtr message)
{
  QPolygonF polygon;
  polygon.reserve(static_cast<int>(message->poses.size()));
  for (const auto & pose : message->poses) {
    polygon.append(worldToScene(pose.pose.position.x, pose.pose.position.y));
  }
  Q_EMIT pathReady(polygon);
}

void OccupancyGridPanel::applyMap(
  const QImage & image, int width, int height, double resolution, double stamp_sec)
{
  map_item_->setPixmap(QPixmap::fromImage(image));
  scene_->setSceneRect(0, 0, width, height);
  dimensions_label_->setText(QString("%1 x %2 | %3 m/cell").arg(width).arg(height).arg(resolution, 0, 'f', 3));
  status_label_->setText("MAP ONLINE");
  status_label_->setStyleSheet("color:#208a45;font-weight:600;");
  last_map_wall_sec_ = wallSeconds();
  if (view_->transform().isIdentity()) {
    view_->fitInView(scene_->sceneRect(), Qt::KeepAspectRatio);
  }
  zoom_label_->setText(QString("Zoom %1% | stamp %2").arg(view_->transform().m11() * 100.0, 0, 'f', 0).arg(stamp_sec, 0, 'f', 3));
  map_update_pending_ = false;
}

void OccupancyGridPanel::applyMapFailure(const QString & reason)
{
  status_label_->setText("INVALID MAP: " + reason);
  status_label_->setStyleSheet("color:#b42318;font-weight:600;");
  map_update_pending_ = false;
}

void OccupancyGridPanel::applyRobot(const QPointF & point, double yaw)
{
  robot_item_->setPos(point);
  heading_item_->setLine(point.x(), point.y(), point.x() + 12.0 * std::cos(yaw), point.y() - 12.0 * std::sin(yaw));
  robot_item_->setVisible(true);
  heading_item_->setVisible(true);
}

void OccupancyGridPanel::applyPath(const QPolygonF & points)
{
  QPainterPath path;
  if (!points.isEmpty()) {
    path.moveTo(points.first());
    for (int index = 1; index < points.size(); ++index) {
      path.lineTo(points[index]);
    }
  }
  path_item_->setPath(path);
}

void OccupancyGridPanel::updateFreshness()
{
  if (last_map_wall_sec_ <= 0.0) {
    status_label_->setText("NO MAP DATA");
    status_label_->setStyleSheet("color:#b42318;font-weight:600;");
  } else if (wallSeconds() - last_map_wall_sec_ > stale_threshold_sec_) {
    status_label_->setText("MAP STALE");
    status_label_->setStyleSheet("color:#b25d00;font-weight:600;");
  }
  zoom_label_->setText(QString("Zoom %1%").arg(view_->transform().m11() * 100.0, 0, 'f', 0));
}

void OccupancyGridPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  config.mapGetString("MapTopic", &map_topic_);
  config.mapGetString("OdomTopic", &odom_topic_);
  config.mapGetString("PathTopic", &path_topic_);
  float stale = static_cast<float>(stale_threshold_sec_);
  if (config.mapGetFloat("StaleThresholdSec", &stale)) {
    stale_threshold_sec_ = stale;
  }
  bool origin = false;
  config.mapGetBool("ShowOrigin", &origin);
  map_topic_edit_->setText(map_topic_);
  odom_topic_edit_->setText(odom_topic_);
  path_topic_edit_->setText(path_topic_);
  show_origin_->setChecked(origin);
  resubscribe();
}

void OccupancyGridPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("MapTopic", map_topic_);
  config.mapSetValue("OdomTopic", odom_topic_);
  config.mapSetValue("PathTopic", path_topic_);
  config.mapSetValue("StaleThresholdSec", stale_threshold_sec_);
  config.mapSetValue("ShowOrigin", show_origin_->isChecked());
}

}  // namespace smartwheel_rviz_plugins
