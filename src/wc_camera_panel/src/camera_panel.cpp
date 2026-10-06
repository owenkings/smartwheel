#include "wc_camera_panel/camera_panel.hpp"

#include <algorithm>
#include <atomic>
#include <exception>
#include <set>
#include <utility>

#include <QGridLayout>
#include <QGroupBox>
#include <QEvent>
#include <QResizeEvent>
#include <QVBoxLayout>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/config.hpp>

namespace wc_camera_panel {
namespace {
const std::array<const char *, 4> roles{{"left_front", "right_front", "left_side", "right_side"}};
const std::array<const char *, 4> titles{{
  "左前", "右前", "左侧", "右侧"}};
std::atomic<unsigned> panel_sequence{0};

QString text(const char * value) { return QString::fromUtf8(value); }

QJsonObject read_report(const QString & path) {
  QFile file(path);
  if (QFileInfo(path).isSymLink() || !file.open(QIODevice::ReadOnly) || file.size() > 1000000) {return {};}
  return QJsonDocument::fromJson(file.read(1000000)).object();
}

class CameraStatusLabel final : public QLabel {
public:
  using QLabel::QLabel;

  void setCompactLayout(bool compact) {
    compact_ = compact;
    fitTextHeight();
  }

  void fitTextHeight() {
    // QLabel's default minimum hint permits a wrapped label to be compressed
    // below heightForWidth. Font fallback and display DPI change the actual
    // number/height of lines, so a fixed compact height can hide status text.
    const int required = compact_ ? std::max(32, heightForWidth(width())) : 38;
    if (minimumHeight() != required) { setMinimumHeight(required); }
  }

protected:
  void resizeEvent(QResizeEvent * event) override {
    QLabel::resizeEvent(event);
    fitTextHeight();
  }

  void changeEvent(QEvent * event) override {
    QLabel::changeEvent(event);
    if (event->type() == QEvent::FontChange || event->type() == QEvent::StyleChange) {
      fitTextHeight();
    }
  }

private:
  bool compact_{false};
};

QString camera_port(const std::string & frame_id) {
  const std::string prefix = "camera_usb3_";
  QString port = text("USB标识未知");
  if (frame_id.size() == prefix.size() + 1 && frame_id.compare(0, prefix.size(), prefix) == 0 &&
      frame_id.back() >= '1' && frame_id.back() <= '4') {
    port = QString("USB%1").arg(frame_id.back() - '0');
  }
  return port;
}

QString camera_binding_state(const std::string & frame_id, int confirmed_port) {
  QString state = text("方向待确认");
  if (confirmed_port >= 1 && confirmed_port <= 4) {
    state = frame_id == "camera_usb3_" + std::to_string(confirmed_port) ?
      text("方位已核对") : (frame_id.empty() ? text("等待绑定图像") : text("来源与绑定不符"));
  }
  return state;
}
}

QString camera_title(const QString & direction, const std::string & frame_id, int confirmed_port) {
  return direction + text("（") + camera_binding_state(frame_id, confirmed_port) +
    text(" · ") + camera_port(frame_id) + text("）");
}

CameraPanel::CameraPanel(QWidget * parent)
: rviz_common::Panel(parent), frames_(std::make_shared<FrameStore>()), rate_time_(Clock::now()) {
  setObjectName("wc_four_camera_panel");
  const auto arguments = QCoreApplication::arguments();
  QString view;
  for (int i = 1; i < arguments.size(); ++i) {
    if ((arguments[i] == "-d" || arguments[i] == "--display-config") && i + 1 < arguments.size()) {
      view = arguments[++i];
    } else if (arguments[i].startsWith("--display-config=")) {
      view = arguments[i].mid(QStringLiteral("--display-config=").size());
    }
  }
  const QFileInfo view_info(view);
  if (view_info.isAbsolute() && view_info.isFile() && !view_info.isSymLink()) {
    health_directory_ = view_info.absolutePath();
    health_identity_ = read_report(QDir(health_directory_).filePath("session.json")).value("session_id").toString();
  }
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);
  auto * title = new QLabel(text("四摄像头实时预览"), this);
  title_label_ = title;
  title->setTextFormat(Qt::PlainText);
  auto font = title->font();
  font.setPointSize(15);
  font.setBold(true);
  title->setFont(font);
  layout->addWidget(title);
  auto * mapping = new QLabel(text("方位暂按 USB 编号分配，方向与旋转均待现场核验；请通过画面或遮挡确认四路。"), this);
  mapping_label_ = mapping;
  mapping->setObjectName("camera_mapping_unconfirmed");
  mapping->setTextFormat(Qt::PlainText);
  mapping->setWordWrap(true);
  layout->addWidget(mapping);
  auto * grid_widget = new QWidget(this);
  grid_widget_ = grid_widget;
  grid_widget->setObjectName("camera_fixed_grid");
  grid_widget->setMinimumSize(960, 720);
  auto * grid = new QGridLayout(grid_widget);
  grid->setContentsMargins(0, 0, 0, 0);
  grid->setSpacing(8);
  for (std::size_t index = 0; index < roles.size(); ++index) {
    auto * box = new QGroupBox(camera_title(text(titles[index]), ""), grid_widget);
    boxes_[index] = box;
    box->setObjectName(QString("camera_") + roles[index]);
    auto * cell = new QVBoxLayout(box);
    cell->setContentsMargins(6, 10, 6, 6);
    canvases_[index] = new ImageCanvas(box);
    canvases_[index]->setObjectName(QString("image_") + roles[index]);
    canvases_[index]->setWarning(text("等待图像"));
    cell->addWidget(canvases_[index], 1);
    statuses_[index] = new CameraStatusLabel(text("尚未收到图像 · 接收帧率未知"), box);
    statuses_[index]->setObjectName(QString("status_") + roles[index]);
    statuses_[index]->setTextFormat(Qt::PlainText);
    statuses_[index]->setWordWrap(true);
    statuses_[index]->setMinimumHeight(38);
    cell->addWidget(statuses_[index]);
    const auto topic = QString("/wc_mapping/cameras/") + roles[index] + "/image_raw";
    box->setToolTip(topic);
    grid->addWidget(box, static_cast<int>(index / 2), static_cast<int>(index % 2));
  }
  grid->setColumnStretch(0, 1);
  grid->setColumnStretch(1, 1);
  grid->setRowStretch(0, 1);
  grid->setRowStretch(1, 1);
  layout->addWidget(grid_widget, 1);
  auto * footer = new QLabel(text("画面超过 1.5 秒未更新会标记过期；帧率按本机接收计算。面板可拖动边界放大。"), this);
  footer_label_ = footer;
  footer->setText(QStringLiteral("预览接收率与完整采集落盘分别统计；8 Hz 预览不代表保存全部相机帧。\n"
    "源端问题显示在各路状态；集中诊断提供证据与复测建议。"));
  footer->setTextFormat(Qt::PlainText);
  footer->setWordWrap(true);
  layout->addWidget(footer);
  setMinimumSize(980, 820);
  timer_ = new QTimer(this);
  timer_->setInterval(50);
  connect(timer_, &QTimer::timeout, this, &CameraPanel::refresh);
  timer_->start();
}

QSize CameraPanel::sizeHint() const {
  return compact_layout_ ? QSize(560, 460) : QSize(1000, 860);
}

void CameraPanel::resizeEvent(QResizeEvent * event) {
  rviz_common::Panel::resizeEvent(event);
  fitCompactLayout();
}

void CameraPanel::fitCompactLayout() {
  if (!compact_layout_ || fitting_compact_layout_) { return; }
  fitting_compact_layout_ = true;
  layout()->activate();  // Establish current widths, including a hidden loaded panel.
  // Explicit QWidget minimum sizes otherwise hide a larger child layout's
  // minimum hint. Reserve both rows, each group title and wrapped status, then
  // the panel title, mapping notice and footer using the target Qt font metrics.
  for (auto * box : boxes_) {
    auto * cell = box->layout();
    cell->invalidate();
    int required = cell->totalMinimumSize().height();
    if (cell->hasHeightForWidth()) {
      required = std::max(required, cell->totalHeightForWidth(box->width()));
    }
    box->setMinimumHeight(required);
  }
  auto * grid = grid_widget_->layout();
  grid->invalidate();
  grid_widget_->setMinimumHeight(std::max(280, grid->totalMinimumSize().height()));
  auto * outer = layout();
  outer->invalidate();
  const int required = std::max({400, outer->totalMinimumSize().height(),
    outer->totalHeightForWidth(std::max(480, width()))});
  setMinimumHeight(required);
  outer->activate();
  fitting_compact_layout_ = false;
}

void CameraPanel::applyCompactLayout(bool compact) {
  compact_layout_ = compact;
  auto * panel_layout = qobject_cast<QVBoxLayout *>(layout());
  const int margin = compact ? 4 : 8;
  panel_layout->setContentsMargins(margin, margin, margin, margin);
  panel_layout->setSpacing(compact ? 4 : -1);
  auto * grid = qobject_cast<QGridLayout *>(grid_widget_->layout());
  grid->setSpacing(compact ? 4 : 8);
  grid_widget_->setMinimumSize(compact ? QSize(460, 280) : QSize(960, 720));
  auto label_font = font();
  if (compact) { label_font.setPointSize(9); }
  mapping_label_->setFont(label_font);
  footer_label_->setFont(label_font);
  auto title_font = font();
  title_font.setPointSize(compact ? 11 : 15);
  title_font.setBold(true);
  title_label_->setFont(title_font);
  for (std::size_t index = 0; index < roles.size(); ++index) {
    boxes_[index]->setFont(label_font);
    boxes_[index]->setMinimumHeight(0);
    auto * cell = qobject_cast<QVBoxLayout *>(boxes_[index]->layout());
    if (compact) { cell->setContentsMargins(4, 6, 4, 4); }
    else { cell->setContentsMargins(6, 10, 6, 6); }
    cell->setSpacing(compact ? 2 : -1);
    // Only the viewport changes: ImageCanvas keeps the complete source image and
    // paints it with its existing aspect-preserving scaling and warning overlay.
    canvases_[index]->setMinimumSize(compact ? QSize(160, 80) : QSize(240, 160));
    static_cast<CameraStatusLabel *>(statuses_[index])->setCompactLayout(compact);
  }
  setMinimumSize(compact ? QSize(480, 400) : QSize(980, 820));
  updateGeometry();
}

void CameraPanel::load(const rviz_common::Config & config) {
  rviz_common::Panel::load(config);
  bool compact = false;  // Existing standalone camera configurations keep their layout.
  config.mapGetBool("Compact Layout", &compact);
  applyCompactLayout(compact);
  confirmed_ports_.fill(0);
  QString status;
  std::array<int, 4> ports{};
  bool valid = config.mapGetString("Mapping Status", &status) && status == "USER_CONFIRMED";
  for (std::size_t i = 0; i < roles.size(); ++i) {
    valid = config.mapGetInt(QString(roles[i]) + " USB Port", &ports[i]) && valid;
    valid = valid && ports[i] >= 1 && ports[i] <= 4;
  }
  valid = valid && std::set<int>(ports.begin(), ports.end()).size() == 4;
  if (valid) {
    confirmed_ports_ = ports;
    mapping_label_->setText(text("方位按用户现场核对绑定，USB编号来自实际图像；画面旋转保留当前配置。"));
  } else {
    mapping_label_->setText(text("方位绑定未确认或配置无效；请通过画面或遮挡核对四路。"));
  }
  refresh();
}

void CameraPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("Compact Layout", compact_layout_);
  config.mapSetValue("Mapping Status", confirmed_ports_[0] ? "USER_CONFIRMED" : "UNCONFIRMED");
  for (std::size_t i = 0; i < roles.size(); ++i) {
    config.mapSetValue(QString(roles[i]) + " USB Port", confirmed_ports_[i]);
  }
}

void CameraPanel::onInitialize() {
  if (initialized_) { return; }
  initialized_ = true;
  try {
    if (!rclcpp::ok()) { throw std::runtime_error("RViz ROS context is not initialized"); }
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);  // Never inherit RViz's __node remap.
    node_ = std::make_shared<rclcpp::Node>(
      "wc_camera_panel_" + std::to_string(++panel_sequence), "/wc_mapping/ui", options);
    rclcpp::SensorDataQoS qos;
    qos.keep_last(1);
    for (std::size_t index = 0; index < roles.size(); ++index) {
      const auto topic = std::string("/wc_mapping/cameras/") + roles[index] + "/image_raw";
      const auto frames = frames_;
      subscriptions_[index] = node_->create_subscription<sensor_msgs::msg::Image>(topic, qos,
        [frames, index](sensor_msgs::msg::Image::ConstSharedPtr message) {
          const auto received = Clock::now();
          try {
            ImageView source{message->width, message->height, message->step, message->encoding,
              message->is_bigendian, message->data.data(), message->data.size()};
            frames->accept(index, decode_image(source), received, message->header.frame_id);
          } catch (const std::exception & error) {
            frames->reject(index, error.what());
          }
        });
    }
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    const auto executor = executor_;
    const auto frames = frames_;
    stopping_ = std::make_shared<std::atomic_bool>(false);
    const auto stopping = stopping_;
    worker_ = std::thread([executor, frames, stopping]() {
      try {
        // Bounded spin also closes the cancel-before-spin-start race on unload.
        while (!stopping->load() && rclcpp::ok()) {
          executor->spin_some(std::chrono::milliseconds(20));
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      }
      catch (const std::exception & error) {
        for (std::size_t index = 0; index < 4; ++index) { frames->reject(index, error.what()); }
      }
    });
  } catch (const std::exception & error) {
    for (std::size_t index = 0; index < 4; ++index) { frames_->reject(index, error.what()); }
  }
}

CameraPanel::~CameraPanel() {
  timer_->stop();
  if (stopping_) { stopping_->store(true); }
  if (executor_) { executor_->cancel(); }
  if (worker_.joinable()) { worker_.join(); }
  subscriptions_.fill(nullptr);
  if (executor_ && node_) {
    try { executor_->remove_node(node_); } catch (const std::exception &) {}
  }
  executor_.reset();
  node_.reset();
  // RViz owns the shared ROS context; this panel must not call rclcpp::shutdown().
}

void CameraPanel::refreshDiagnostics(TimePoint now) {
  if (std::chrono::duration<double>(now - diagnostics_time_).count() < 1.0) {return;}
  diagnostics_time_ = now;
  source_diagnostics_.fill(QString());
  if (health_directory_.isEmpty() || health_identity_.isEmpty()) {return;}
  const auto report = read_report(QDir(health_directory_).filePath("health.json"));
  if (report.value("session_id").toString() != health_identity_) {return;}
  for (const auto & item : report.value("issues").toArray()) {
    const auto issue = item.toObject(); const auto component = issue.value("component").toString();
    for (std::size_t index = 0; index < roles.size(); ++index) {
      if (component == QString("camera/") + roles[index] || component == QString("camera_") + roles[index]) {
        source_diagnostics_[index] = issue.value("code").toString() + ": " + issue.value("observed").toString().left(400);
      }
    }
  }
}

void CameraPanel::refresh() {
  const auto now = Clock::now();
  refreshDiagnostics(now);
  const auto snapshots = frames_->snapshot();
  const auto interval = std::chrono::duration<double>(now - rate_time_).count();
  if (interval >= 1.0) {
    for (std::size_t index = 0; index < 4; ++index) {
      rates_[index] = static_cast<double>(snapshots[index].accepted - rate_counts_[index]) / interval;
      rate_counts_[index] = snapshots[index].accepted;
    }
    rate_time_ = now;
  }
  for (std::size_t index = 0; index < 4; ++index) {
    const auto & slot = snapshots[index];
    const auto full_title = camera_title(text(titles[index]), slot.frame_id, confirmed_ports_[index]);
    const auto title = compact_layout_ ?
      text(titles[index]) + text(" · ") + camera_port(slot.frame_id) : full_title;
    if (boxes_[index]->title() != title) { boxes_[index]->setTitle(title); }
    if (slot.frame && displayed_[index] != slot.accepted) {
      const auto & frame = *slot.frame;
      QImage image(frame.rgb.data(), static_cast<int>(frame.width), static_cast<int>(frame.height),
                   static_cast<int>(frame.width * 3), QImage::Format_RGB888);
      canvases_[index]->setFrame(image.copy());  // GUI thread owns this QImage's bytes.
      displayed_[index] = slot.accepted;
    }
    const auto state = preview_state(slot, now);
    const double age = slot.frame ? std::chrono::duration<double>(now - slot.last_valid).count() : 0.0;
    QString warning;
    if (state == PreviewState::waiting) { warning = text("等待图像"); }
    if (state == PreviewState::stale) { warning = text("图像已过期\n%1 秒未收到新有效帧").arg(age, 0, 'f', 1); }
    if (state == PreviewState::error) { warning = text("图像接收错误\n旧画面不可作为实时画面"); }
    if (slot.frame && confirmed_ports_[index] &&
        slot.frame_id != "camera_usb3_" + std::to_string(confirmed_ports_[index])) {
      warning = text("图像USB来源与已核对方位不符");
    }
    if (!source_diagnostics_[index].isEmpty()) {warning = source_diagnostics_[index];}
    canvases_[index]->setWarning(warning);
    QString status;
    if (slot.frame) {
      status = text("本机接收 %1 fps · %2×%3 · 有效 %4 / 拒绝 %5")
        .arg(rates_[index], 0, 'f', 1).arg(slot.frame->width).arg(slot.frame->height)
        .arg(static_cast<qulonglong>(slot.accepted)).arg(static_cast<qulonglong>(slot.rejected));
      if (state == PreviewState::stale) { status.prepend(text("已过期 · ")); }
    } else {
      status = text("尚无有效图像 · 接收帧率未知 · 拒绝 %1").arg(static_cast<qulonglong>(slot.rejected));
    }
    if (state == PreviewState::error) {
      status += "\n" + QString::fromUtf8(slot.error.data(), static_cast<int>(slot.error.size()));
    }
    if (!source_diagnostics_[index].isEmpty()) {status += "\n" + source_diagnostics_[index];}
    if (compact_layout_) {
      status.prepend(camera_binding_state(slot.frame_id, confirmed_ports_[index]) + "\n");
    }
    statuses_[index]->setText(status);
    statuses_[index]->setStyleSheet(state == PreviewState::live && source_diagnostics_[index].isEmpty() ? "color: #16825d;" : "color: #c9424a;");
    static_cast<CameraStatusLabel *>(statuses_[index])->fitTextHeight();
    statuses_[index]->setToolTip(text("收到的 frame_id（相机物理端口信息）：\n") +
      QString::fromUtf8(slot.frame_id.data(), static_cast<int>(slot.frame_id.size())) +
      (compact_layout_ ? "\n" + full_title : QString()));
  }
  fitCompactLayout();
}

}  // namespace wc_camera_panel

PLUGINLIB_EXPORT_CLASS(wc_camera_panel::CameraPanel, rviz_common::Panel)
