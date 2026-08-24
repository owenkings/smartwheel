#ifndef SMARTWHEEL_RVIZ_PLUGINS__CAMERA_PANEL_HPP_
#define SMARTWHEEL_RVIZ_PLUGINS__CAMERA_PANEL_HPP_

#include <QImage>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <image_transport/subscriber.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QScrollArea;
class QTimer;
class QToolButton;
class QWidget;

namespace smartwheel_rviz_plugins
{

class CameraPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit CameraPanel(QWidget * parent = nullptr);
  ~CameraPanel() override;
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

  static bool isOffline(double now_sec, double last_frame_sec, double threshold_sec);
  static bool frameAllowed(double now_sec, double last_gui_sec, double max_fps);
  static std::string compressedTopic(const std::string & image_topic);

Q_SIGNALS:
  void frameReady(
    const QImage & image, int width, int height, double fps, double stamp_sec,
    double latency_ms, bool dropped);
  void decodeFailed(const QString & reason);

private Q_SLOTS:
  void applyFrame(
    const QImage & image, int width, int height, double fps, double stamp_sec,
    double latency_ms, bool dropped);
  void applyDecodeFailure(const QString & reason);
  void updateOfflineState();
  void updateSubscription();
  void updateDisplayOptions();

private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & message);
  void onCompressedImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & message);
  void resizeEvent(QResizeEvent * event) override;
  void refreshPixmap();

  rclcpp::Node::SharedPtr node_;
  image_transport::Subscriber image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_subscription_;
  std::atomic_bool frame_pending_{false};
  std::atomic_bool keep_aspect_{true};
  std::atomic_bool mirror_{false};
  std::atomic_int rotation_degrees_{0};
  std::atomic<double> max_display_fps_{12.0};
  std::atomic<double> offline_threshold_sec_{2.0};
  mutable std::mutex timing_mutex_;
  double last_receive_sec_{0.0};
  double previous_stamp_sec_{0.0};
  double smoothed_fps_{0.0};
  double last_gui_schedule_sec_{0.0};
  QImage latest_image_;

  QString image_topic_{"/camera/front/image_raw"};
  QString info_topic_{"/camera/front/camera_info"};
  QString physical_label_;
  // Four simultaneous 640x480 raw BGR topics overload the RViz/DDS display
  // path on the Orin. The camera adapter publishes JPEG transport explicitly;
  // make it the safe workbench default while retaining raw as an opt-in choice
  // for bounded calibration tasks.
  QString transport_hint_{"compressed"};
  QLineEdit * image_topic_edit_{nullptr};
  QLineEdit * info_topic_edit_{nullptr};
  QComboBox * transport_combo_{nullptr};
  QToolButton * details_toggle_{nullptr};
  QScrollArea * details_scroll_{nullptr};
  QWidget * details_widget_{nullptr};
  QLabel * physical_label_widget_{nullptr};
  QLabel * image_label_{nullptr};
  QLabel * topic_label_{nullptr};
  QLabel * resolution_label_{nullptr};
  QLabel * fps_label_{nullptr};
  QLabel * timestamp_label_{nullptr};
  QLabel * latency_label_{nullptr};
  QLabel * state_label_{nullptr};
  QLabel * dropped_label_{nullptr};
  QCheckBox * aspect_check_{nullptr};
  QCheckBox * timestamp_check_{nullptr};
  QCheckBox * fps_check_{nullptr};
  QCheckBox * mirror_check_{nullptr};
  QComboBox * rotation_combo_{nullptr};
  QDoubleSpinBox * max_fps_spin_{nullptr};
  QDoubleSpinBox * offline_spin_{nullptr};
  QTimer * offline_timer_{nullptr};
};

}  // namespace smartwheel_rviz_plugins

#endif  // SMARTWHEEL_RVIZ_PLUGINS__CAMERA_PANEL_HPP_
