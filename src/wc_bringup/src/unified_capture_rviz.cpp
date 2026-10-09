// One subscriber-only recording window. Drivers and finalization remain owned
// by capture.py. This executable does not open any hardware or create a bag.
#include <csignal>
#include <QCloseEvent>
#include <QCommandLineParser>
#include <QElapsedTimer>
#include <QGroupBox>
#include <QMainWindow>
#include <QPushButton>
#include <QRegularExpression>
#include <QSplitter>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "wc_bringup/mapping_teleop.hpp"
#include "wc_bringup/unified_rviz.hpp"
// Ogre's GLX headers define Xlib Status/Bool; keep them after Qt headers.
#include <rviz_rendering/render_system.hpp>
#ifdef Status
#undef Status
#endif
#ifdef Bool
#undef Bool
#endif
#ifdef None
#undef None
#endif

namespace {
volatile std::sig_atomic_t stopped = 0;
void request_stop(int) {stopped = 1;}

class CaptureWindow final : public QMainWindow {
public:
  wc_bringup::MappingTeleopPanel * teleop{};
protected:
  void closeEvent(QCloseEvent * event) override {
    if (teleop) {teleop->stop_and_disarm("CAPTURE_UI_CLOSED");}
    QMainWindow::closeEvent(event);
  }
};

class LidarStatus final : public QGroupBox {
public:
  LidarStatus(const QString & side, const QString & directory,
    QWidget * parent) :
    QGroupBox(side == "left" ? "左雷达" : "右雷达", parent), side_(side), directory_(directory) {
    setObjectName("capture_status_" + side);
    auto * layout = new QVBoxLayout(this);
    label_ = new QLabel(this); label_->setTextFormat(Qt::PlainText); label_->setWordWrap(true);
    layout->addWidget(label_); layout->addStretch();
    const auto path = QDir(directory).filePath("configuration/lidar/" + side + ".yaml");
    if (QFileInfo(path).isFile() && !QFileInfo(path).isSymLink()) {
      QFile binding(path);
      if (binding.open(QIODevice::ReadOnly) && binding.size() <= 65536) {
        auto matches = QRegularExpression("^\\s*expected_serial:\\s*([A-Za-z0-9_]+)\\s*$",
          QRegularExpression::MultilineOption).globalMatch(QString::fromUtf8(binding.readAll()));
        if (matches.hasNext()) {
          serial_ = matches.next().captured(1);
          if (matches.hasNext()) {serial_.clear();}
        }
      }
    }
    elapsed_.start();
    auto * timer = new QTimer(this); timer->setInterval(500);
    connect(timer, &QTimer::timeout, this, [this]() {refresh();}); timer->start(); refresh();
  }
  void attach_viewport(wc_bringup::RvizViewport * viewport) {
    if (viewport) {
      auto * group = viewport->manager()->getRootDisplayGroup();
      for (int index = 0; index < group->numDisplays(); ++index) {
        auto * display = group->getDisplayAt(index);
        if (display->getClassId() != "rviz_default_plugins/PointCloud2" || !display->isEnabled()) {continue;}
        topic_ = display->subProp("Topic")->getValue().toString(); break;
      }
      if (!topic_.isEmpty()) {
        subscription_ = viewport->node()->create_subscription<sensor_msgs::msg::PointCloud2>(
          topic_.toStdString(), rclcpp::SensorDataQoS().keep_last(1),
          [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
            ++count_; points_ = static_cast<quint64>(cloud->width) * cloud->height;
            frame_ = QString::fromStdString(cloud->header.frame_id); age_.restart();
          });
        viewport->beforeNodeRelease(this, [this]() {subscription_.reset();});
      }
    }
    refresh();
  }
  QJsonObject observation() const {
    return {{"side", side_}, {"topic", topic_}, {"received", static_cast<double>(count_)},
      {"points", static_cast<double>(points_)}, {"frame_id", frame_}, {"text", label_->text()}};
  }
private:
  void refresh() {
    if (elapsed_.elapsed() >= 1000) {
      hz_ = (count_ - previous_) * 1000.0 / elapsed_.elapsed(); previous_ = count_; elapsed_.restart();
    }
    QString state = topic_.isEmpty() ? "本次未选择" : (count_ == 0 ? "等待预览数据" :
      (age_.elapsed() > 1500 || points_ == 0 ? "预览已过期 / 无有效点" : "预览接收正常"));
    const auto preview = wc_bringup::panel_read_json(QDir(directory_).filePath("capture_preview_status.json"));
    const auto source = preview.value("source_status").toObject().value("lidar_" + side_).toObject();
    label_->setText(QStringLiteral("%1\n配置绑定序列号：%2\n实际帧坐标：%3\n预览接收：%4 Hz\n当前帧：%5 点\n已接收：%6 帧\n来源状态：%7\n\n预览经过限频，不能代表设备采集频率。\n话题：%8")
      .arg(state, serial_.isEmpty() ? "未读取" : serial_, frame_.isEmpty() ? "等待" : frame_)
      .arg(hz_, 0, 'f', 1).arg(points_).arg(count_).arg(source.value("status").toString("等待报告"), topic_));
  }
  QString side_, directory_, topic_, serial_, frame_;
  QLabel * label_{};
  quint64 count_{}, previous_{}, points_{}; double hz_{};
  QElapsedTimer elapsed_, age_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

wc_bringup::TeleopSession manual_session(const QString & directory, const QString & identity,
  const QJsonObject & manifest) {
  const auto runtime = wc_bringup::panel_read_json(directory + "/configuration/manual_runtime.json");
  if (runtime.value("session_id").toString() != identity || runtime.value("source_mode").toString() != "real" ||
    runtime.value("status").toString() != "EXPERIMENT" ||
    !runtime.value("manual_controls").toObject().value("arm_allowed").toBool() ||
    runtime.value("manual_controls").toObject().value("interaction_policy").toString() != "hybrid_manual") {
    throw std::runtime_error("Capture manual-control snapshot does not match this live session");
  }
  wc_bringup::TeleopSession session;
  session.directory = wc_bringup::checked_manual_socket_directory(
    manifest.value("project_root").toString(), directory, identity,
    manifest.value("manual_socket_directory").toString());
  session.session_id = identity;
  if (!session.valid()) {throw std::runtime_error("Capture control socket identity invalid");}
  return session;
}
}

int main(int argc, char ** argv) {
  QApplication app(argc, argv);
  app.setApplicationName("统一数据录制预览");
  QCommandLineParser parser; parser.addHelpOption();
  for (const auto & option : {"left-config", "right-config", "session-root", "session-id"}) {
    parser.addOption({QString(option), QString(option), "value"});
  }
  parser.addOption({"lidars", "Selected sides: both, left, right", "sides", "both"});
  parser.addOption({"read-only", "Synthetic/subscriber-only viewing without a recording session"});
  parser.process(app);
  try {
    const QString lidars = parser.value("lidars");
    if (lidars != "both" && lidars != "left" && lidars != "right") {return 2;}
    const auto directory = QDir::cleanPath(parser.value("session-root"));
    const auto identity = parser.value("session-id");
    QJsonObject manifest;
    if (!parser.isSet("read-only")) {
      const QFileInfo info(directory);
      if (!info.isAbsolute() || !info.isDir() || info.isSymLink() ||
        !QRegularExpression("^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$").match(identity).hasMatch()) {return 2;}
      for (QString path = directory; ; ) {
        if (QFileInfo(path).isSymLink()) {return 2;}
        const QString parent = QFileInfo(path).dir().absolutePath();
        if (path == parent) {break;} path = parent;
      }
      manifest = wc_bringup::panel_read_json(directory + "/capture_manifest.json");
      if (manifest.value("session_id").toString() != identity || manifest.value("status").toString() != "RECORDING") {return 2;}
    }
    rclcpp::init(argc, argv);
    rviz_rendering::RenderSystem::get();
    CaptureWindow window; window.setWindowTitle("数据录制 — 左右雷达统一预览"); window.resize(1680, 920);
    auto * central = new QWidget(&window); auto * outer = new QVBoxLayout(central);
    auto * header = new QHBoxLayout;
    const auto storage = manifest.value("storage_staging").toObject();
    const QString final_directory = storage.value("final_directory").toString(
      manifest.value("output").toString());
    QString storage_text = QFileInfo(final_directory).isAbsolute() ?
      "最终保存位置：" + final_directory : "最终保存位置：录制程序尚未提供";
    if (storage.value("mode").toString() == "memory") {
      storage_text += "\n录制中临时缓存：" + directory + "（结束后转存）";
    }
    auto * title = new QLabel(parser.isSet("read-only") ? "只读预览 / 合成验证（未启动录制）" :
      "正在录制：" + identity + "\n" + storage_text, central);
    title->setObjectName("capture_destination_label");
    title->setTextFormat(Qt::PlainText); title->setWordWrap(true); header->addWidget(title, 1);
    auto * stop = new QPushButton("结束录制", central); stop->setObjectName("capture_stop_button");
    if (parser.isSet("read-only")) {stop->setText("关闭只读预览");}
    QObject::connect(stop, &QPushButton::clicked, &window, &QWidget::close); header->addWidget(stop);
    outer->addLayout(header);
    auto * splitter = new QSplitter(Qt::Horizontal, central); splitter->setObjectName("capture_three_columns");
    auto * statuses = new QWidget(splitter); statuses->setObjectName("capture_status_column");
    auto * status_layout = new QVBoxLayout(statuses); status_layout->setContentsMargins(0, 0, 0, 0);
    std::vector<LidarStatus *> observations;
    std::vector<wc_bringup::RvizViewport *> viewports;
    std::vector<std::pair<LidarStatus *, wc_bringup::RvizViewport *>> pending_views;
    for (const QString side : {QString("left"), QString("right")}) {
      auto * box = new QGroupBox(side == "left" ? "左雷达画面" : "右雷达画面", splitter);
      box->setObjectName("capture_" + side + "_column"); auto * layout = new QVBoxLayout(box);
      wc_bringup::RvizViewport * viewport = nullptr;
      if (lidars == "both" || lidars == side) {
        const auto config = wc_bringup::load_view_config(parser.value(side + "-config"));
        viewport = new wc_bringup::RvizViewport("capture_" + side + "_view", config, false, box);
        viewports.push_back(viewport);
        layout->addWidget(viewport, 1);
      } else {layout->addWidget(new QLabel("本次未选择该雷达", box));}
      auto * state = new LidarStatus(side, directory, statuses);
      pending_views.emplace_back(state, viewport);
      status_layout->addWidget(state, 1); observations.push_back(state); splitter->addWidget(box);
    }
    splitter->setSizes({300, 660, 660}); splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1); splitter->setStretchFactor(2, 1); outer->addWidget(splitter, 1);
    if (!parser.isSet("read-only") && manifest.value("manual_drive").toBool()) {
      window.teleop = new wc_bringup::MappingTeleopPanel(manual_session(directory, identity, manifest), &window, central);
      outer->addWidget(window.teleop); window.teleop->connect_to_server();
    }
    outer->addWidget(new QLabel("结束后由录制程序停止设备并完成数据落盘，请等待完成通知。预览不改变保存的数据。", central));
    window.setCentralWidget(central);
    window.show();
    // Every RenderPanel must have a real, final native parent before Ogre
    // creates its drawable. No event-loop pumping is needed: QWidget::show
    // synchronously delivers Show to the embedded window containers.
    for (const auto & item : pending_views) {
      auto * viewport = item.second;
      if (!viewport) {continue;}
      viewport->initializeShown();
      auto * box = viewport->parentWidget();
      static_cast<QVBoxLayout *>(box->layout())->insertWidget(0,
        wc_bringup::display_switches(viewport->manager(), box));
      item.first->attach_viewport(viewport);
    }
    wc_bringup::install_panel_diagnostics(&window, "capture", [observations, viewports]() {
      QJsonArray states; for (auto * state : observations) {states.append(state->observation());}
      QJsonArray displays; for (auto * viewport : viewports) {
        bool independent = true;
        for (auto * peer : viewports) {
          if (peer == viewport) {continue;}
          independent = independent && viewport->manager()->getSceneManager() != peer->manager()->getSceneManager()
            && viewport->manager()->getRenderPanel() != peer->manager()->getRenderPanel()
            && viewport->manager()->getViewManager() != peer->manager()->getViewManager();
        }
        displays.append(QJsonObject{{"view", viewport->objectName()},
          {"frames", static_cast<double>(viewport->manager()->getFrameCount())},
          {"independent_view", independent},
          {"color_materials_retained", viewport->property("rviz_default_colors_retained").toInt()},
          {"native_window", viewport->nativeObservation()},
          {"displays", wc_bringup::display_observation(viewport->manager())}});
      }
      return QJsonObject{{"lidars", states}, {"views", displays}};
    });
    std::signal(SIGINT, request_stop); std::signal(SIGTERM, request_stop);
    QTimer timer; QObject::connect(&timer, &QTimer::timeout, &window, [&]() {if (stopped) {window.close();}}); timer.start(50);
    const int result = app.exec();
    // Qt widgets/managers are destroyed at scope exit before rclcpp shutdown.
    return result;
  } catch (const std::exception & error) {
    qCritical("Unified capture view failed: %s", error.what()); return 2;
  }
}
