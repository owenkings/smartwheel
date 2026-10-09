// Fit the dedicated 2D RViz viewport once, then only on an explicit user request.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <QElapsedTimer>
#include <QEvent>
#include <QMetaObject>
#include <QPointer>
#include <QPushButton>
#include <QVariant>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>
#include "wc_bringup/unified_rviz.hpp"

namespace wc_bringup {

class GridViewportFit final : public QObject {
public:
  // The viewport must already have completed initializeShown(). Its manager's
  // executor owns this subscription; all view changes run on the GUI thread.
  GridViewportFit(RvizViewport * viewport, const QString & topic,
    QPushButton * button, bool has_map)
  : QObject(viewport), viewport_(viewport), button_(button) {
    if (!viewport || !viewport->manager() || !button || !qApp ||
      QThread::currentThread() != qApp->thread()) {
      throw std::invalid_argument("GridViewportFit requires an initialized viewport and GUI thread");
    }
    setObjectName(viewport->objectName() + "_fit");
    setProperty("fit_count", 0);
    setProperty("bounds_valid", false);
    setProperty("first_fit_done", false);
    setProperty("topic", topic);
    button->setText(QStringLiteral("适配地图"));
    button->setEnabled(false);
    button->setToolTip(has_map ? QStringLiteral("等待有效地图及坐标变换") :
      QStringLiteral("预览模式不生成 2D 地图"));
    connect(button, &QPushButton::clicked, this, [this]() {fitNow();});
    retry_ = new QTimer(this);
    retry_->setInterval(200);
    geometry_quiet_.start();
    viewport->manager()->getRenderPanel()->getRenderWindow()->installEventFilter(this);
    connect(retry_, &QTimer::timeout, this, [this]() {refresh();});
    publishObservation();
    if (!has_map) {return;}
    if (topic.trimmed().isEmpty()) {throw std::invalid_argument("Empty map topic");}
    connect(viewport->manager()->getFrameManager(),
      &rviz_common::FrameManagerIface::fixedFrameChanged, this, [this]() {refresh();});
    subscription_ = viewport->node()->create_subscription<nav_msgs::msg::OccupancyGrid>(
      topic.toStdString(), rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
        // Queue with this QObject as context, so destruction cancels pending work.
        QMetaObject::invokeMethod(this, [this, message]() {receive(*message);},
          Qt::QueuedConnection);
      });
    viewport->beforeNodeRelease(this, [this]() {
      retry_->stop(); subscription_.reset(); viewport_ = nullptr;
    });
  }

  bool fitNow() {
    if (!refreshBounds()) {return false;}
    if (!viewport_ || !viewport_->manager()) {return false;}
    auto * manager = viewport_->manager();
    auto * view = manager->getViewManager()->getCurrent();
    if (!view || view->getClassId() != "rviz_default_plugins/TopDownOrtho") {
      reportError(QStringLiteral("2D 视口需要 TopDownOrtho 视角")); return false;
    }
    auto * x = dynamic_cast<rviz_common::properties::FloatProperty *>(view->subProp("X"));
    auto * y = dynamic_cast<rviz_common::properties::FloatProperty *>(view->subProp("Y"));
    auto * angle = dynamic_cast<rviz_common::properties::FloatProperty *>(view->subProp("Angle"));
    auto * scale = dynamic_cast<rviz_common::properties::FloatProperty *>(view->subProp("Scale"));
    auto * target = dynamic_cast<rviz_common::properties::TfFrameProperty *>(view->subProp("Target Frame"));
    auto * ogre = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(
      manager->getRenderPanel()->getRenderWindow());
    if (!x || !y || !angle || !scale || !target || !ogre ||
      ogre->getActualWidth() <= 0 || ogre->getActualHeight() <= 0) {
      reportError(QStringLiteral("等待 2D 视口完成布局")); return false;
    }
    // Humble TopDownOrtho uses physical viewport pixels / Scale as visible
    // metres. A 1.1 factor reserves 10% extra world extent around the bounds.
    const double span_x = std::max(max_x_ - min_x_, static_cast<double>(info_.resolution));
    const double span_y = std::max(max_y_ - min_y_, static_cast<double>(info_.resolution));
    const double pixels_per_metre = std::min(ogre->getActualWidth() / span_x,
      ogre->getActualHeight() / span_y) / 1.1;
    const double center_x = min_x_ + (max_x_ - min_x_) / 2.0;
    const double center_y = min_y_ + (max_y_ - min_y_) / 2.0;
    if (!asFloat(center_x) || !asFloat(center_y) || !asFloat(pixels_per_metre) ||
      static_cast<float>(pixels_per_metre) <= 0) {
      reportError(QStringLiteral("地图范围超出 2D 视口可表示范围")); return false;
    }
    // X/Y are relative to Target Frame; pin that frame before applying bounds
    // already transformed into the manager's Fixed Frame.
    target->setValue(rviz_common::properties::TfFrameProperty::FIXED_FRAME_STRING);
    x->setFloat(static_cast<float>(center_x)); y->setFloat(static_cast<float>(center_y));
    angle->setFloat(0); scale->setFloat(static_cast<float>(pixels_per_metre));
    manager->queueRender();
    first_fit_done_ = true;
    setProperty("first_fit_done", true);
    setProperty("fit_count", property("fit_count").toInt() + 1);
    setProperty("fit_center_x", center_x); setProperty("fit_center_y", center_y);
    setProperty("fit_scale", pixels_per_metre);
    setProperty("fit_viewport_width", ogre->getActualWidth());
    setProperty("fit_viewport_height", ogre->getActualHeight());
    setProperty("last_error", QString());
    retry_->stop();
    publishObservation();
    return true;
  }

private:
  bool eventFilter(QObject * watched, QEvent * event) override {
    if (!first_fit_done_ && event->type() == QEvent::Resize) {geometry_quiet_.restart();}
    return QObject::eventFilter(watched, event);
  }
  static bool asFloat(double value) {
    return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
  }
  void reportError(const QString & error) {
    setProperty("last_error", error);
    if (button_) {button_->setToolTip(error);}
    publishObservation();
  }
  void publishObservation() {
    if (!viewport_) {return;}
    QJsonObject observation{{"auto_fit_done", first_fit_done_},
      {"fit_count", property("fit_count").toInt()},
      {"bounds_valid", property("bounds_valid").toBool()},
      {"center_x", property("fit_center_x").toDouble()},
      {"center_y", property("fit_center_y").toDouble()},
      {"scale", property("fit_scale").toDouble()},
      {"fit_viewport_width", property("fit_viewport_width").toInt()},
      {"fit_viewport_height", property("fit_viewport_height").toInt()},
      {"fixed_frame", property("bounds_frame").toString()},
      {"last_error", property("last_error").toString()}};
    if (property("bounds_valid").toBool()) {
      observation["bounds"] = QJsonArray{min_x_, min_y_, max_x_, max_y_};
      observation["width_m"] = max_x_ - min_x_;
      observation["height_m"] = max_y_ - min_y_;
    }
    viewport_->setProperty("grid_fit", observation);
  }
  void receive(const nav_msgs::msg::OccupancyGrid & message) {
    const auto & info = message.info;
    const auto & p = info.origin.position;
    const auto & q = info.origin.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    const uint64_t cells = static_cast<uint64_t>(info.width) * info.height;
    metadata_valid_ = !message.header.frame_id.empty() && message.header.stamp.sec >= 0 &&
      message.header.stamp.nanosec < 1000000000u && info.width != 0 && info.height != 0 &&
      std::isfinite(info.resolution) && info.resolution > 0 && message.data.size() == cells &&
      asFloat(p.x) && asFloat(p.y) && asFloat(p.z) && std::isfinite(norm) &&
      std::abs(norm - 1.0) <= 1e-3 && asFloat(static_cast<double>(info.width) * info.resolution) &&
      asFloat(static_cast<double>(info.height) * info.resolution);
    if (!metadata_valid_) {
      setProperty("bounds_valid", false);
      if (button_) {button_->setEnabled(false);}
      reportError(QStringLiteral("地图尺寸、分辨率、原点或栅格数据无效"));
      retry_->stop(); return;
    }
    // Bounds need only metadata, not another retained copy of the whole grid.
    header_ = message.header; info_ = info;
    refresh();
  }
  bool refreshBounds() {
    setProperty("bounds_valid", false);
    if (button_) {button_->setEnabled(false);}
    if (!metadata_valid_ || !viewport_ || !viewport_->manager()) {return false;}
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    try {
      // Match this view's MapDisplay (Use Timestamp=false): try now, then
      // latest. Header-time-only fitting can disagree with the displayed map
      // when a transform moves or old transforms have left the TF cache.
      auto * manager = viewport_->manager();
      auto * frames = manager->getFrameManager();
      const auto clock = manager->getClock();
      if (!frames->transform(header_.frame_id, clock->now(), info_.origin, position, orientation) &&
        !frames->transform(header_.frame_id, rclcpp::Time(0, 0, clock->get_clock_type()),
          info_.origin, position, orientation)) {
        reportError(QStringLiteral("等待地图到 Fixed Frame 的坐标变换")); return false;
      }
    } catch (const std::exception & error) {
      reportError(QStringLiteral("地图坐标变换失败：") + QString::fromUtf8(error.what())); return false;
    }
    const double norm = orientation.w * static_cast<double>(orientation.w) +
      orientation.x * static_cast<double>(orientation.x) +
      orientation.y * static_cast<double>(orientation.y) +
      orientation.z * static_cast<double>(orientation.z);
    if (!asFloat(position.x) || !asFloat(position.y) || !asFloat(position.z) ||
      !std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3) {
      reportError(QStringLiteral("地图坐标变换包含无效数值")); return false;
    }
    const Ogre::Real width = static_cast<Ogre::Real>(info_.width * static_cast<double>(info_.resolution));
    const Ogre::Real height = static_cast<Ogre::Real>(info_.height * static_cast<double>(info_.resolution));
    min_x_ = min_y_ = std::numeric_limits<double>::infinity();
    max_x_ = max_y_ = -std::numeric_limits<double>::infinity();
    for (const auto & corner : {Ogre::Vector3(0, 0, 0), Ogre::Vector3(width, 0, 0),
      Ogre::Vector3(0, height, 0), Ogre::Vector3(width, height, 0)}) {
      const auto transformed = position + orientation * corner;
      if (!asFloat(transformed.x) || !asFloat(transformed.y) || !asFloat(transformed.z)) {
        reportError(QStringLiteral("地图四角超出可表示范围")); return false;
      }
      min_x_ = std::min(min_x_, static_cast<double>(transformed.x));
      max_x_ = std::max(max_x_, static_cast<double>(transformed.x));
      min_y_ = std::min(min_y_, static_cast<double>(transformed.y));
      max_y_ = std::max(max_y_, static_cast<double>(transformed.y));
    }
    setProperty("bounds", QVariantList{min_x_, min_y_, max_x_, max_y_});
    setProperty("bounds_frame", viewport_->manager()->getFixedFrame());
    setProperty("bounds_valid", true);
    setProperty("last_error", QString());
    if (button_) {
      button_->setEnabled(true);
      button_->setToolTip(QStringLiteral("按最新地图范围居中、缩放并复位旋转；仅首幅地图自动适配，后续更新保留手动视角"));
    }
    publishObservation();
    return true;
  }
  void refresh() {
    if (!metadata_valid_) {return;}
    const bool ready = refreshBounds();
    // Initial camera/status text and docks can resize the grid after the first
    // ROS message. Fit once only after its real native geometry has settled.
    // Later resize/map events never change an already fitted/manual view.
    if (first_fit_done_ || (ready && geometry_quiet_.elapsed() >= 500 && fitNow())) {retry_->stop();}
    else {retry_->start();}
  }

  QPointer<RvizViewport> viewport_;
  QPointer<QPushButton> button_;
  QTimer * retry_{};
  QElapsedTimer geometry_quiet_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  std_msgs::msg::Header header_;
  nav_msgs::msg::MapMetaData info_;
  bool metadata_valid_{false}, first_fit_done_{false};
  double min_x_{}, min_y_{}, max_x_{}, max_y_{};
};

}  // namespace wc_bringup
