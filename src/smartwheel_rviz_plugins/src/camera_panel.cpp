#include "smartwheel_rviz_plugins/camera_panel.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPixmap>
#include <QResizeEvent>
#include <QScrollArea>
#include <QTimer>
#include <QTransform>
#include <QVBoxLayout>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rmw/qos_profiles.h>
#include <rviz_common/display_context.hpp>

#include <algorithm>
#include <cmath>

namespace smartwheel_rviz_plugins
{
namespace
{

double steadySeconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

double messageSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

}  // namespace

CameraPanel::CameraPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * content = new QWidget;
  auto * layout = new QVBoxLayout(content);
  layout->setContentsMargins(4, 4, 4, 4);
  auto * topics = new QFormLayout;
  image_topic_edit_ = new QLineEdit(image_topic_);
  info_topic_edit_ = new QLineEdit(info_topic_);
  topics->addRow("Image", image_topic_edit_);
  topics->addRow("Camera info", info_topic_edit_);
  layout->addLayout(topics);

  image_label_ = new QLabel("NO IMAGE DATA");
  image_label_->setAlignment(Qt::AlignCenter);
  image_label_->setMinimumSize(220, 150);
  image_label_->setStyleSheet("background:#202428;color:#d0d5dd;border:1px solid #555;");
  layout->addWidget(image_label_, 1);

  topic_label_ = new QLabel(image_topic_);
  resolution_label_ = new QLabel("0 x 0");
  fps_label_ = new QLabel("FPS 0.0");
  timestamp_label_ = new QLabel("Frame --");
  latency_label_ = new QLabel("Latency -- ms");
  state_label_ = new QLabel("OFFLINE");
  dropped_label_ = new QLabel;
  layout->addWidget(topic_label_);
  layout->addWidget(resolution_label_);
  layout->addWidget(fps_label_);
  layout->addWidget(timestamp_label_);
  layout->addWidget(latency_label_);
  layout->addWidget(state_label_);
  layout->addWidget(dropped_label_);

  auto * options = new QFormLayout;
  aspect_check_ = new QCheckBox;
  aspect_check_->setChecked(true);
  mirror_check_ = new QCheckBox;
  timestamp_check_ = new QCheckBox;
  timestamp_check_->setChecked(true);
  fps_check_ = new QCheckBox;
  fps_check_->setChecked(true);
  rotation_combo_ = new QComboBox;
  rotation_combo_->addItems({"0", "90", "180", "270"});
  max_fps_spin_ = new QDoubleSpinBox;
  max_fps_spin_->setRange(1.0, 30.0);
  max_fps_spin_->setValue(12.0);
  offline_spin_ = new QDoubleSpinBox;
  offline_spin_->setRange(0.5, 30.0);
  offline_spin_->setValue(2.0);
  options->addRow("Keep aspect", aspect_check_);
  options->addRow("Mirror", mirror_check_);
  options->addRow("Rotation", rotation_combo_);
  options->addRow("Max display FPS", max_fps_spin_);
  options->addRow("Offline after (s)", offline_spin_);
  options->addRow("Show timestamp", timestamp_check_);
  options->addRow("Show FPS", fps_check_);
  layout->addLayout(options);

  auto * outer = new QVBoxLayout(this);
  auto * scroll = new QScrollArea;
  scroll->setWidgetResizable(true);
  scroll->setWidget(content);
  outer->addWidget(scroll);

  connect(image_topic_edit_, &QLineEdit::editingFinished, this, &CameraPanel::updateSubscription);
  connect(info_topic_edit_, &QLineEdit::editingFinished, this, &CameraPanel::updateSubscription);
  connect(aspect_check_, &QCheckBox::toggled, this, &CameraPanel::updateDisplayOptions);
  connect(mirror_check_, &QCheckBox::toggled, this, &CameraPanel::updateDisplayOptions);
  connect(timestamp_check_, &QCheckBox::toggled, this, &CameraPanel::updateDisplayOptions);
  connect(fps_check_, &QCheckBox::toggled, this, &CameraPanel::updateDisplayOptions);
  connect(rotation_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CameraPanel::updateDisplayOptions);
  connect(max_fps_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraPanel::updateDisplayOptions);
  connect(offline_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &CameraPanel::updateDisplayOptions);
  connect(this, &CameraPanel::frameReady, this, &CameraPanel::applyFrame, Qt::QueuedConnection);
  connect(this, &CameraPanel::decodeFailed, this, &CameraPanel::applyDecodeFailure, Qt::QueuedConnection);

  offline_timer_ = new QTimer(this);
  connect(offline_timer_, &QTimer::timeout, this, &CameraPanel::updateOfflineState);
  offline_timer_->start(250);
}

CameraPanel::~CameraPanel()
{
  image_subscription_.shutdown();
  info_subscription_.reset();
}

bool CameraPanel::isOffline(double now_sec, double last_frame_sec, double threshold_sec)
{
  return last_frame_sec <= 0.0 || now_sec - last_frame_sec > threshold_sec;
}

bool CameraPanel::frameAllowed(double now_sec, double last_gui_sec, double max_fps)
{
  return max_fps > 0.0 && (last_gui_sec <= 0.0 || now_sec - last_gui_sec >= 1.0 / max_fps);
}

void CameraPanel::onInitialize()
{
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    applyDecodeFailure("RViz ROS node unavailable");
    return;
  }
  node_ = abstraction->get_raw_node();
  updateSubscription();
}

void CameraPanel::updateSubscription()
{
  image_topic_ = image_topic_edit_->text().trimmed();
  info_topic_ = info_topic_edit_->text().trimmed();
  topic_label_->setText(image_topic_);
  image_subscription_.shutdown();
  info_subscription_.reset();
  if (!node_ || image_topic_.isEmpty() || info_topic_.isEmpty()) {
    return;
  }
  image_subscription_ = image_transport::create_subscription(
    node_.get(), image_topic_.toStdString(),
    std::bind(&CameraPanel::onImage, this, std::placeholders::_1), "raw",
    rmw_qos_profile_sensor_data);
  info_subscription_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
    info_topic_.toStdString(), rclcpp::QoS(1).reliable().transient_local(),
    [](const sensor_msgs::msg::CameraInfo::ConstSharedPtr) {});
}

void CameraPanel::updateDisplayOptions()
{
  keep_aspect_ = aspect_check_->isChecked();
  mirror_ = mirror_check_->isChecked();
  rotation_degrees_ = rotation_combo_->currentText().toInt();
  max_display_fps_ = max_fps_spin_->value();
  offline_threshold_sec_ = offline_spin_->value();
  timestamp_label_->setVisible(timestamp_check_->isChecked());
  fps_label_->setVisible(fps_check_->isChecked());
  refreshPixmap();
}

void CameraPanel::onImage(const sensor_msgs::msg::Image::ConstSharedPtr & message)
{
  const double now = steadySeconds();
  double fps = 0.0;
  bool dropped = false;
  {
    std::lock_guard<std::mutex> lock(timing_mutex_);
    const double stamp = messageSeconds(message->header.stamp);
    if (previous_stamp_sec_ > 0.0 && stamp > previous_stamp_sec_) {
      const double instantaneous = 1.0 / (stamp - previous_stamp_sec_);
      smoothed_fps_ = smoothed_fps_ <= 0.0 ? instantaneous : 0.85 * smoothed_fps_ + 0.15 * instantaneous;
      dropped = smoothed_fps_ > 0.0 && stamp - previous_stamp_sec_ > 1.8 / smoothed_fps_;
    }
    previous_stamp_sec_ = stamp;
    last_receive_sec_ = now;
    fps = smoothed_fps_;
    if (!frameAllowed(now, last_gui_schedule_sec_, max_display_fps_) || frame_pending_.exchange(true)) {
      return;
    }
    last_gui_schedule_sec_ = now;
  }

  try {
    const auto cv_image = cv_bridge::toCvShare(message, "rgb8");
    QImage image(
      cv_image->image.data, cv_image->image.cols, cv_image->image.rows,
      static_cast<int>(cv_image->image.step), QImage::Format_RGB888);
    image = image.copy();
    if (mirror_) {
      image = image.mirrored(true, false);
    }
    const int rotation = rotation_degrees_;
    if (rotation != 0) {
      image = image.transformed(QTransform().rotate(rotation));
    }
    const double stamp = messageSeconds(message->header.stamp);
    const double ros_now = node_->get_clock()->now().seconds();
    Q_EMIT frameReady(
      image, static_cast<int>(message->width), static_cast<int>(message->height), fps,
      stamp, std::max(0.0, (ros_now - stamp) * 1000.0), dropped);
  } catch (const cv_bridge::Exception & error) {
    frame_pending_ = false;
    Q_EMIT decodeFailed(QString::fromStdString(error.what()));
  }
}

void CameraPanel::applyFrame(
  const QImage & image, int width, int height, double fps, double stamp_sec,
  double latency_ms, bool dropped)
{
  latest_image_ = image;
  refreshPixmap();
  resolution_label_->setText(QString("%1 x %2").arg(width).arg(height));
  fps_label_->setText(QString("FPS %1").arg(fps, 0, 'f', 1));
  timestamp_label_->setText(QString("Frame %1").arg(stamp_sec, 0, 'f', 3));
  latency_label_->setText(QString("Latency %1 ms").arg(latency_ms, 0, 'f', 1));
  state_label_->setText("ONLINE");
  state_label_->setStyleSheet("color:#208a45;font-weight:600;");
  dropped_label_->setText(dropped ? "DROPPED FRAME" : "");
  dropped_label_->setStyleSheet("color:#b25d00;font-weight:600;");
  frame_pending_ = false;
}

void CameraPanel::applyDecodeFailure(const QString & reason)
{
  state_label_->setText("ERROR: " + reason);
  state_label_->setStyleSheet("color:#b42318;font-weight:600;");
  frame_pending_ = false;
}

void CameraPanel::updateOfflineState()
{
  double last = 0.0;
  {
    std::lock_guard<std::mutex> lock(timing_mutex_);
    last = last_receive_sec_;
  }
  if (isOffline(steadySeconds(), last, offline_threshold_sec_)) {
    state_label_->setText("OFFLINE");
    state_label_->setStyleSheet("color:#b42318;font-weight:600;");
  }
}

void CameraPanel::refreshPixmap()
{
  if (latest_image_.isNull()) {
    return;
  }
  const auto mode = keep_aspect_ ? Qt::KeepAspectRatio : Qt::IgnoreAspectRatio;
  image_label_->setPixmap(QPixmap::fromImage(latest_image_).scaled(image_label_->size(), mode, Qt::SmoothTransformation));
}

void CameraPanel::resizeEvent(QResizeEvent * event)
{
  rviz_common::Panel::resizeEvent(event);
  refreshPixmap();
}

void CameraPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  config.mapGetString("ImageTopic", &image_topic_);
  config.mapGetString("CameraInfoTopic", &info_topic_);
  bool value = true;
  if (config.mapGetBool("KeepAspectRatio", &value)) {aspect_check_->setChecked(value);}
  if (config.mapGetBool("ShowTimestamp", &value)) {timestamp_check_->setChecked(value);}
  if (config.mapGetBool("ShowFps", &value)) {fps_check_->setChecked(value);}
  if (config.mapGetBool("Mirror", &value)) {mirror_check_->setChecked(value);}
  int rotation = 0;
  if (config.mapGetInt("RotationDegrees", &rotation)) {rotation_combo_->setCurrentText(QString::number(rotation));}
  float number = 12.0F;
  if (config.mapGetFloat("MaxDisplayFps", &number)) {max_fps_spin_->setValue(number);}
  number = 2.0F;
  if (config.mapGetFloat("OfflineThresholdSec", &number)) {offline_spin_->setValue(number);}
  image_topic_edit_->setText(image_topic_);
  info_topic_edit_->setText(info_topic_);
  updateDisplayOptions();
  updateSubscription();
}

void CameraPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("ImageTopic", image_topic_);
  config.mapSetValue("CameraInfoTopic", info_topic_);
  config.mapSetValue("KeepAspectRatio", aspect_check_->isChecked());
  config.mapSetValue("ShowTimestamp", timestamp_check_->isChecked());
  config.mapSetValue("ShowFps", fps_check_->isChecked());
  config.mapSetValue("Mirror", mirror_check_->isChecked());
  config.mapSetValue("RotationDegrees", rotation_combo_->currentText().toInt());
  config.mapSetValue("MaxDisplayFps", max_fps_spin_->value());
  config.mapSetValue("OfflineThresholdSec", offline_spin_->value());
}

}  // namespace smartwheel_rviz_plugins
