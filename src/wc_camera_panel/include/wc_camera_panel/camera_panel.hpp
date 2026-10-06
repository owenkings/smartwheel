#pragma once

#include "wc_camera_panel/frame_store.hpp"
#include "wc_camera_panel/image_canvas.hpp"

#include <array>
#include <atomic>
#include <memory>
#include <thread>

#include <QLabel>
#include <QGroupBox>
#include <QTimer>
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace wc_camera_panel {

QString camera_title(const QString & direction, const std::string & frame_id, int confirmed_port = 0);

class CameraPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit CameraPanel(QWidget * parent = nullptr);
  ~CameraPanel() override;
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;
  QSize sizeHint() const override;
protected:
  void resizeEvent(QResizeEvent * event) override;
private:
  void applyCompactLayout(bool compact);
  void fitCompactLayout();
  void refresh();
  void refreshDiagnostics(TimePoint now);
  QString health_directory_;
  QString health_identity_;
  std::array<QString, 4> source_diagnostics_{};
  TimePoint diagnostics_time_{};
  std::shared_ptr<FrameStore> frames_;
  std::array<ImageCanvas *, 4> canvases_{};
  std::array<QGroupBox *, 4> boxes_{};
  std::array<QLabel *, 4> statuses_{};
  std::array<std::uint64_t, 4> displayed_{};
  std::array<std::uint64_t, 4> rate_counts_{};
  std::array<double, 4> rates_{};
  TimePoint rate_time_;
  QTimer * timer_{};
  QLabel * title_label_{};
  QLabel * mapping_label_{};
  QLabel * footer_label_{};
  QWidget * grid_widget_{};
  std::array<int, 4> confirmed_ports_{};
  bool compact_layout_{false};
  bool fitting_compact_layout_{false};
  bool initialized_{false};
  rclcpp::Node::SharedPtr node_;
  std::array<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr, 4> subscriptions_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<std::atomic_bool> stopping_;
  std::thread worker_;
};

}  // namespace wc_camera_panel
