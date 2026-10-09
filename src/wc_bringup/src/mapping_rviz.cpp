// SPDX-License-Identifier: Apache-2.0
// Native RViz with a session-only close policy. Map saving belongs to mapping_app.
// Public APIs checked against ros2/rviz humble: visualizer_app.hpp,
// visualization_frame.hpp, visualizer_app.cpp and rviz2/src/main.cpp.
#include <algorithm>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include <QApplication>
#include <QEvent>
#include <QDockWidget>
#include <QElapsedTimer>
#include <QJsonArray>
#include <QLayout>
#include <QMainWindow>
#include <QObject>
#include <QPixmap>
#include <QPushButton>
#include <QScreen>
#include <QTimer>
#include <QWidget>
#include <QWindow>
#include <QVBoxLayout>
#include <QSplitter>
#include <QScrollArea>
#include <OgreCamera.h>
#include <OgreRenderTarget.h>
#include <OgreRenderSystem.h>
#include <OgreRenderSystemCapabilities.h>
#include <OgreRoot.h>
#include <OgreViewport.h>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/ros_integration/ros_client_abstraction.hpp"
#include "rviz_common/visualization_frame.hpp"
#include "rviz_common/visualizer_app.hpp"
#include "rviz_rendering/render_window.hpp"
#include "wc_bringup/mapping_render_diagnostics.hpp"
#include "wc_bringup/mapping_teleop.hpp"
#include "wc_bringup/mapping_health.hpp"
#include "wc_bringup/unified_rviz.hpp"
#include "wc_bringup/grid_view_fit.hpp"
#include "wc_bringup/mapping_session_view.hpp"
#include "wc_camera_panel/camera_panel.hpp"

namespace
{

void arrange_unified(rviz_common::VisualizationFrame & frame, QDockWidget & teleop,
  const QStringList & arguments)
{
  auto * cameras = frame.findChild<wc_camera_panel::CameraPanel *>();
  if (!cameras) {
    cameras = new wc_camera_panel::CameraPanel(&frame);
    cameras->onInitialize();  // Subscription-only: never starts a camera driver.
  }
  for (auto * dock : frame.findChildren<QDockWidget *>()) {
    if (dock != &teleop) {dock->hide();}
  }
  cameras->hide();
  auto * native_center = frame.takeCentralWidget();
  auto * splitter = new QSplitter(Qt::Horizontal, &frame);
  splitter->setObjectName("mapping_three_columns");
  auto * left = new QWidget(splitter); left->setObjectName("mapping_left_column");
  auto * left_layout = new QVBoxLayout(left); left_layout->setContentsMargins(3, 3, 3, 3);
  left_layout->addWidget(cameras->createSideView("left", left), 2);

  QString map_topic = "/wc_mapping/app/grid_map";
  auto * group = frame.getManager()->getRootDisplayGroup();
  bool has_map = false;
  for (int index = 0; index < group->numDisplays(); ++index) {
    auto * display = group->getDisplayAt(index);
    if (display->getClassId() == "rviz_default_plugins/Map") {
      map_topic = display->subProp("Topic")->getValue().toString();
      display->setEnabled(false); has_map = true;
    }
  }
  auto * map_label = new QLabel(has_map ? QStringLiteral("2D 栅格地图 · 拖动平移 / 滚轮缩放") :
    QStringLiteral("2D 栅格地图 · 预览模式不生成地图"), left);
  map_label->setObjectName("mapping_grid_label");
  map_label->setWordWrap(true); left_layout->addWidget(map_label);
  auto * fit_map = new QPushButton(QStringLiteral("适配地图"), left);
  fit_map->setObjectName("mapping_fit_grid_button");
  fit_map->setToolTip(QStringLiteral("按最新地图范围重新居中、缩放并复位朝向；自动适配只在第一帧执行。"));
  left_layout->addWidget(fit_map, 0, Qt::AlignRight);
  const QString map_yaml = QStringLiteral(
    "Global Options:\n  Fixed Frame: %1\n  Background Color: 24; 29; 36\n  Frame Rate: 10\n"
    "Displays:\n  - Class: rviz_default_plugins/Map\n    Name: 2D grid\n    Enabled: %2\n"
    "    Alpha: 1\n    Use Timestamp: false\n    Topic:\n      Value: %3\n      Reliability Policy: Reliable\n"
    "      Durability Policy: Transient Local\n      Depth: 1\n"
    "Tools:\n  - Class: rviz_default_plugins/MoveCamera\n"
    "Views:\n  Current:\n    Class: rviz_default_plugins/TopDownOrtho\n    Scale: 35\n"
    "    Angle: 0\n    X: 0\n    Y: 0\n").arg(frame.getManager()->getFixedFrame(),
      has_map ? "true" : "false", map_topic);
  rviz_common::YamlConfigReader reader; rviz_common::Config map_config;
  reader.readString(map_config, map_yaml);
  if (reader.error()) {throw std::runtime_error(reader.errorMessage().toStdString());}
  auto * grid_view = new wc_bringup::RvizViewport("mapping_grid_view", map_config, true, left);
  left_layout->addWidget(grid_view, 1);

  auto * center = new QWidget(splitter); center->setObjectName("mapping_center_column");
  auto * center_layout = new QVBoxLayout(center); center_layout->setContentsMargins(0, 0, 0, 0);
  auto * switches = wc_bringup::display_switches(frame.getManager(), center);
  switches->setObjectName("mapping_display_switches"); center_layout->addWidget(switches);
  center_layout->addWidget(native_center, 1);
  auto * right = new QWidget(splitter); right->setObjectName("mapping_right_column");
  auto * right_layout = new QVBoxLayout(right); right_layout->setContentsMargins(3, 3, 3, 3);
  right_layout->addWidget(cameras->createSideView("right", right), 2);
  auto * parameter_scroll = new QScrollArea(right);
  parameter_scroll->setWidgetResizable(true); parameter_scroll->setMinimumHeight(130);
  auto * parameters = new QWidget(parameter_scroll); parameters->setObjectName("mapping_parameters");
  auto * parameter_layout = new QVBoxLayout(parameters);
  QString view;
  for (int index = 1; index < arguments.size(); ++index) {
    if ((arguments[index] == "-d" || arguments[index] == "--display-config") && index + 1 < arguments.size()) {
      view = arguments[++index];
    }
  }
  const auto config = wc_bringup::panel_read_json(QFileInfo(view).dir().filePath("runtime_config.json"));
  if (auto * native_frame = wc_bringup::native_preview_frame_selector(frame.getManager(), config, center)) {
    center_layout->insertWidget(0, native_frame);
  }
  const auto estimator = config.value("wheel_imu_estimator").toString(config.value("estimator").toString("five_state"));
  auto * settings = new QLabel(QStringLiteral("当前任务参数\n模式：%1\n雷达：%2\n点云：%3\nEKF：%4\n运动修正：%5\n过程噪声：%6\n几何轨迹纠偏：%7\n参数修改对下一次任务生效。")
    .arg(config.value("mapping_enabled").toBool() ? "实时建图" : "预览",
      config.value("mode").toString("未知"), config.value("cloud_source").toString("参见本次配置"),
      estimator == "robot_localization" ? "官方 robot_localization" : "五状态",
      config.value("motion_correction").toBool() ? "已启用设备校正声明" : "关闭",
      config.value("process_noise").toString("legacy") == "white_acceleration" ? "连续白噪声 PSD" : "原模型",
      config.value("geometry").toBool() ? "启用（输出轨迹纠偏）" : "关闭"), parameters);
  settings->setTextFormat(Qt::PlainText); settings->setWordWrap(true); parameter_layout->addWidget(settings);
  settings->setObjectName("mapping_settings");
  if (auto * health = teleop.findChild<QWidget *>("mapping_health_panel")) {
    parameter_layout->addWidget(health);
  }
  parameter_scroll->setWidget(parameters); right_layout->addWidget(parameter_scroll, 1);
  splitter->addWidget(left); splitter->addWidget(center); splitter->addWidget(right);
  splitter->setStretchFactor(0, 0); splitter->setStretchFactor(1, 1); splitter->setStretchFactor(2, 0);
  splitter->setSizes({320, 980, 320});
  frame.setCentralWidget(splitter);
  frame.setWindowTitle(QStringLiteral("实时融合 — 3D / 2D / 四路相机"));
  frame.addDockWidget(Qt::BottomDockWidgetArea, &teleop); teleop.show();
  frame.resizeDocks({&teleop}, {170}, Qt::Vertical);
  // Install and show the final widget hierarchy before creating the auxiliary
  // Ogre drawable. Constructing it under Qt's temporary container parent can
  // leave a black GLX child even though ROS subscriptions report success.
  frame.show(); splitter->show();
  grid_view->initializeShown();
  new wc_bringup::GridViewportFit(grid_view, map_topic, fit_map, has_map);
}

// This filter belongs to this process and this one native frame. It does not
// dismiss arbitrary dialogs, write display files, or change another RViz window.
class SessionViewCloseFilter final : public QObject
{
public:
  explicit SessionViewCloseFilter(rviz_common::VisualizationFrame & frame)
  : frame_(frame)
  {
    frame_.installEventFilter(this);
  }

protected:
  bool eventFilter(QObject * watched, QEvent * event) override
  {
    if (watched == &frame_ && event->type() == QEvent::Close) {
      // Only unsaved view/layout edits are discarded. Returning false lets the
      // native closeEvent run its normal cleanup and application exit sequence.
      frame_.setWindowModified(false);
    }
    return false;
  }

private:
  rviz_common::VisualizationFrame & frame_;
};

QJsonArray diagnostic_rect(const QRect & value)
{return {value.x(), value.y(), value.width(), value.height()};}

QJsonObject diagnostic_geometry(QWidget * widget)
{
  if (widget == nullptr) {return {{"present", false}};}
  return {{"present", true}, {"class", widget->metaObject()->className()},
    {"name", widget->objectName()}, {"geometry", diagnostic_rect(widget->geometry())},
    {"frame_geometry", diagnostic_rect(widget->frameGeometry())},
    {"minimum_size", QJsonArray{widget->minimumWidth(), widget->minimumHeight()}},
    {"visible", widget->isVisible()}, {"active", widget->isActiveWindow()}};
}

// Opt-in observation only. These synchronous captures may stall the GUI and
// disk; record their duration rather than treating diagnostic runs as timing
// acceptance. Never renderNow(), reset, resize, hide/show, or synthesize input.
class RenderDiagnostics final : public QObject
{
public:
  RenderDiagnostics(rviz_common::VisualizationFrame & frame,
    const wc_bringup::TeleopSession & session, const rclcpp::Logger & logger,
    const QList<int> & schedule)
  : QObject(&frame), frame_(frame), output_(session), logger_(logger)
  {
    started_.start();
    QJsonArray scheduled_seconds;
    for (int seconds : schedule) {scheduled_seconds.append(seconds);}
    output_.write_json(QStringLiteral("identity.json"), {
      {"schema", "wc_mapping_render_diagnostics_v2"}, {"session_id", session.session_id},
      {"pid", static_cast<double>(QCoreApplication::applicationPid())},
      {"source_mode", "real"}, {"enabled_by", "WC_MAPPING_RENDER_DIAGNOSTICS=1"},
      {"scheduled_seconds", scheduled_seconds}, {"visual_review_status", "PENDING"},
      {"capture_order", QJsonArray{"qt_before_ogre", "ogre", "qt_after_ogre", "qt_later_1s"}},
      {"interference_note", "Ogre readback can select a viewport/context and synchronizes pixel reads. Synchronous Ogre/Qt capture and PNG/JSON writes can delay the GUI and disk IO. Capture success does not prove a visible map or unchanged rendering state/timing."},
      {"geometry_changed_by_diagnostics", false}});
    connect(qApp, &QCoreApplication::aboutToQuit, this, [this]() {stopped_ = true;});
    for (int seconds : schedule) {
      QTimer::singleShot(seconds * 1000, this, [this, seconds]() {
        if (!stopped_) {capture(seconds);}
      });
    }
    RCLCPP_WARN(logger_, "Optional render diagnostics enabled in %s; synchronous capture can affect timing",
      output_.path().toUtf8().constData());
  }

private:
  QJsonObject observe(int seconds) const
  {
    QJsonObject value{{"scheduled_seconds", seconds},
      {"elapsed_ms", static_cast<double>(started_.elapsed())},
      {"frame", diagnostic_geometry(&frame_)},
      {"central", diagnostic_geometry(frame_.centralWidget())}};
    auto * render = frame_.getRenderWindow();
    value["render_qwindow"] = render == nullptr ? QJsonObject{{"present", false}} :
      QJsonObject{{"present", true}, {"geometry", diagnostic_rect(render->geometry())},
      {"visible", render->isVisible()}, {"exposed", render->isExposed()},
      {"device_pixel_ratio", render->devicePixelRatio()},
      {"ogre_viewport_present", rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(render) != nullptr}};
    if (render != nullptr) {
      auto qwindow = value["render_qwindow"].toObject();
      // QWindow::winId creates a native surface when absent. Do not let a
      // diagnostic recreate a surface: read the ID only for an existing handle.
      qwindow["native_handle_present"] = render->handle() != nullptr;
      if (render->handle() != nullptr) {
        qwindow["xid"] = QStringLiteral("0x") + QString::number(static_cast<qulonglong>(render->winId()), 16);
      }
      value["render_qwindow"] = qwindow;
      auto * viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(render);
      if (viewport != nullptr && viewport->getTarget() != nullptr) {
        const auto background = viewport->getBackgroundColour();
        QJsonObject dimensions{
          {"relative_left_top_width_height", QJsonArray{
            viewport->getLeft(), viewport->getTop(), viewport->getWidth(), viewport->getHeight()}},
          {"actual_left_top_width_height", QJsonArray{
            viewport->getActualLeft(), viewport->getActualTop(),
            viewport->getActualWidth(), viewport->getActualHeight()}},
          {"dimensions_updated", viewport->_isUpdated()},
          {"clear_every_frame", viewport->getClearEveryFrame()},
          {"clear_buffers", static_cast<int>(viewport->getClearBuffers())},
          {"background_rgba", QJsonArray{background.r, background.g, background.b, background.a}}};
        const auto * camera = viewport->getCamera();
        if (camera != nullptr) {
          dimensions["camera_name"] = QString::fromStdString(camera->getName());
          dimensions["camera_aspect_ratio"] = static_cast<double>(camera->getAspectRatio());
          dimensions["camera_auto_aspect_ratio"] = camera->getAutoAspectRatio();
        }
        value["ogre_viewport"] = dimensions;
        auto * target = viewport->getTarget();
        QJsonObject ogre{{"name", QString::fromStdString(target->getName())},
          {"width", static_cast<int>(target->getWidth())}, {"height", static_cast<int>(target->getHeight())},
          {"active", target->isActive()}, {"auto_updated", target->isAutoUpdated()}};
        // Humble's Ogre 1.12.1 GLXWindow exposes WINDOW as an X11 Window
        // (unsigned long). It normally is a CHILD of the Qt render window,
        // because RViz passes parentWindowHandle; different IDs are expected.
        unsigned long native_window = 0;
        try {
          target->getCustomAttribute("WINDOW", &native_window);
          ogre["window_xid"] = QStringLiteral("0x") + QString::number(static_cast<qulonglong>(native_window), 16);
          ogre["window_xid_available"] = native_window != 0;
        } catch (const std::exception & error) {
          ogre["window_xid_available"] = false;
          ogre["window_xid_error"] = QString::fromUtf8(error.what());
        }
        value["ogre_target"] = ogre;
      }
    }
    QJsonArray docks;
    for (auto * dock : frame_.findChildren<QDockWidget *>()) {
      auto item = diagnostic_geometry(dock); item["title"] = dock->windowTitle();
      item["floating"] = dock->isFloating(); docks.append(item);
    }
    value["docks"] = docks;
    return value;
  }

  QJsonObject capture_qt(const QString & basename)
  {
    auto * handle = frame_.windowHandle();
    QScreen * screen = handle != nullptr ? handle->screen() : nullptr;
    if (screen == nullptr || handle->handle() == nullptr) {
      throw std::runtime_error("Owned frame native screen unavailable");
    }
    const WId client_id = handle->winId();
    const QString path = output_.new_file(basename);
    QElapsedTimer cost; cost.start();
    const QPixmap pixels = screen->grabWindow(client_id);
    const auto grab_ms = cost.elapsed();
    cost.restart();
    if (pixels.isNull() || !pixels.save(path, "PNG")) {
      throw std::runtime_error("Qt owned-window capture failed");
    }
    if (frame_.windowHandle() != handle || handle->handle() == nullptr || handle->winId() != client_id) {
      throw std::runtime_error("Owned Qt native client changed while captured");
    }
    return {{"file", path}, {"grab_ms", static_cast<double>(grab_ms)},
      {"png_write_ms", static_cast<double>(cost.elapsed())},
      {"pixel_width", pixels.width()}, {"pixel_height", pixels.height()},
      {"device_pixel_ratio", pixels.devicePixelRatio()},
      {"pid", static_cast<double>(QCoreApplication::applicationPid())},
      {"client_xid", QStringLiteral("0x") + QString::number(static_cast<qulonglong>(client_id), 16)},
      {"capture_method", "in_process_qscreen_grabWindow_owned_client"}};
  }

  void capture_later(int seconds, const QString & phase)
  {
    if (stopped_) {return;}
    QElapsedTimer total; total.start();
    QJsonObject value;
    try {
      value = observe(seconds);
      value["requested_delay_after_initial_capture_ms"] = 1000;
      value["qt_later"] = capture_qt(phase + QStringLiteral("_qt_later.png"));
      value["after"] = observe(seconds);
      value["capture_status"] = "CAPTURED_REQUIRES_VISUAL_REVIEW";
    } catch (const std::exception & error) {
      stopped_ = true; value["capture_status"] = "FAILED";
      value["error"] = QString::fromUtf8(error.what());
    } catch (...) {
      stopped_ = true; value["capture_status"] = "FAILED";
      value["error"] = "Unknown later screenshot exception";
    }
    value["elapsed_before_result_write_ms"] = static_cast<double>(total.elapsed());
    try {
      output_.write_json(phase + QStringLiteral("_later.json"), value);
      RCLCPP_WARN(logger_, "Render diagnostic %s later capture completed in %.3f ms including result write",
        phase.toUtf8().constData(), total.nsecsElapsed()/1.e6);
    } catch (const std::exception & error) {
      stopped_ = true;
      RCLCPP_WARN(logger_, "Cannot save later render diagnostic: %s", error.what());
    }
  }

  void capture(int seconds)
  {
    const QString phase = QStringLiteral("phase_%1").arg(seconds, 2, 10, QLatin1Char('0'));
    QJsonObject value;
    QElapsedTimer total; total.start();
    try {
      value = observe(seconds);
      output_.write_json(phase + QStringLiteral("_before.json"), value);
      value["before_json_write_ms"] = static_cast<double>(total.elapsed());
      auto * render = frame_.getRenderWindow();
      if (render == nullptr || rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(render) == nullptr) {
        throw std::runtime_error("Native render window or initialized Ogre viewport unavailable");
      }
      value["qt_before_ogre"] = capture_qt(phase + QStringLiteral("_qt_before_ogre.png"));
      value["before_ogre"] = observe(seconds);
      const QString ogre_path = output_.new_file(phase + QStringLiteral("_ogre.png"));
      QElapsedTimer cost; cost.start();
      render->captureScreenShot(ogre_path.toStdString());
      value["ogre_capture_ms"] = static_cast<double>(cost.elapsed());
      value["ogre_file"] = ogre_path;
      if (QFileInfo(ogre_path).size() <= 0) {throw std::runtime_error("Ogre screenshot is empty");}
      value["after_ogre"] = observe(seconds);
      value["qt_after_ogre"] = capture_qt(phase + QStringLiteral("_qt_after_ogre.png"));
      value["capture_status"] = "CAPTURED_REQUIRES_VISUAL_REVIEW";
      value["after"] = observe(seconds);
      // Return to the normal event loop before observing a later presented
      // frame. No processEvents(), renderNow(), forced update, or buffer swap.
      QTimer::singleShot(1000, this, [this, seconds, phase]() {capture_later(seconds, phase);});
    } catch (const std::exception & error) {
      stopped_ = true;
      value["capture_status"] = "FAILED";
      value["error"] = QString::fromUtf8(error.what());
      RCLCPP_WARN(logger_, "Render diagnostic %s failed: %s", phase.toUtf8().constData(), error.what());
    } catch (...) {
      stopped_ = true; value["capture_status"] = "FAILED";
      value["error"] = "Unknown screenshot exception";
      RCLCPP_WARN(logger_, "Render diagnostic %s failed with unknown exception", phase.toUtf8().constData());
    }
    value["elapsed_before_result_write_ms"] = static_cast<double>(total.elapsed());
    try {
      output_.write_json(phase + QStringLiteral(".json"), value);
      RCLCPP_WARN(logger_, "Render diagnostic %s completed in %.3f ms including result write",
        phase.toUtf8().constData(), total.nsecsElapsed()/1.e6);
    } catch (const std::exception & error) {
      stopped_ = true;
      RCLCPP_WARN(logger_, "Cannot save render diagnostic result: %s", error.what());
    }
  }

  rviz_common::VisualizationFrame & frame_;
  wc_bringup::render_diagnostics::OutputDirectory output_;
  rclcpp::Logger logger_;
  QElapsedTimer started_;
  bool stopped_ = false;
};

void maybe_start_render_diagnostics(rviz_common::VisualizationFrame & frame,
  const QStringList & arguments, const rclcpp::Logger & logger)
{
  if (!wc_bringup::render_diagnostics::enabled(qgetenv("WC_MAPPING_RENDER_DIAGNOSTICS"))) {return;}
  try {
    const auto schedule = wc_bringup::render_diagnostics::capture_schedule(
      qgetenv("WC_MAPPING_RENDER_DIAGNOSTICS_DELAY_S"));
    new RenderDiagnostics(frame, wc_bringup::teleop_session_from_arguments(arguments), logger, schedule);
  } catch (const std::exception & error) {
    RCLCPP_WARN(logger, "Optional render diagnostics refused: %s", error.what());
  }
}

void fit_initial_window(rviz_common::VisualizationFrame & frame, const rclcpp::Logger & logger)
{
  QScreen * screen = frame.windowHandle() != nullptr ? frame.windowHandle()->screen() : nullptr;
  if (screen == nullptr) {screen = QGuiApplication::primaryScreen();}
  if (screen == nullptr) {return;}
  // availableGeometry excludes desktop panels/taskbars. Account for the native
  // window decorations as well: QWidget::resize controls the client area only.
  const QRect available = screen->availableGeometry().adjusted(12, 12, -12, -12);
  const int border_width = std::max(0, frame.frameGeometry().width()-frame.width());
  const int border_height = std::max(0, frame.frameGeometry().height()-frame.height());
  const QSize target(std::max(1, std::min(1400, available.width()-border_width)),
    std::max(1, std::min(900, available.height()-border_height)));
  if (frame.layout() != nullptr) {frame.layout()->activate();}
  frame.resize(target);
  // For a top-level window QWidget::move places its frame (including titlebar).
  frame.move(available.left()+std::max(0, (available.width()-frame.frameGeometry().width())/2),
    available.top()+std::max(0, (available.height()-frame.frameGeometry().height())/2));
  const QSize central = frame.centralWidget() != nullptr ? frame.centralWidget()->size() : QSize();
  const QRect actual = frame.frameGeometry();
  RCLCPP_INFO(logger, "Initial native layout: available=%dx%d, client=%dx%d, frame=%dx%d, central=%dx%d",
    available.width(), available.height(), frame.width(), frame.height(), actual.width(), actual.height(),
    central.width(), central.height());
  if (!available.contains(actual)) {
    // Preserve any unknown user panel's minimum size; do not clip its controls
    // or silently replace the original RViz/OpenGL rendering widget.
    RCLCPP_WARN(logger, "Native panel minimum sizes exceed the available desktop; adjust optional panels in Panels menu");
  }
}

void arrange_initial_docks(rviz_common::VisualizationFrame & frame, QDockWidget & teleop)
{
  QDockWidget * displays = nullptr;
  QDockWidget * views = nullptr;
  QDockWidget * tools = nullptr;
  QDockWidget * cameras = nullptr;
  // Match only names declared by this application's RViz configuration. Do
  // not redock, hide, resize or rename additional user/plugin panels.
  for (auto * dock : frame.findChildren<QDockWidget *>()) {
    if (dock->windowTitle() == QStringLiteral("Displays")) {displays = dock;}
    else if (dock->windowTitle() == QStringLiteral("Views")) {views = dock;}
    else if (dock->windowTitle() == QStringLiteral("Tool Properties")) {tools = dock;}
    else if (dock->windowTitle() == QStringLiteral("四路实时摄像头")) {cameras = dock;}
  }
  frame.setDockOptions(frame.dockOptions() | QMainWindow::AllowTabbedDocks);
  if (displays != nullptr) {
    displays->setFloating(false);
    frame.addDockWidget(Qt::LeftDockWidgetArea, displays);
    for (auto * secondary : {views, tools}) {
      if (secondary != nullptr) {
        secondary->setFloating(false);
        frame.addDockWidget(Qt::LeftDockWidgetArea, secondary);
        frame.tabifyDockWidget(displays, secondary);
      }
    }
    displays->show(); displays->raise();
  }
  if (cameras != nullptr) {
    cameras->setFloating(false);
    frame.addDockWidget(Qt::RightDockWidgetArea, cameras);
    cameras->show();
  }
  teleop.setFloating(false);
  frame.addDockWidget(Qt::BottomDockWidgetArea, &teleop);
  frame.setCorner(Qt::BottomLeftCorner, Qt::BottomDockWidgetArea);
  frame.setCorner(Qt::BottomRightCorner, Qt::BottomDockWidgetArea);
  teleop.show();
  QList<QDockWidget *> side_docks;
  QList<int> widths;
  if (displays != nullptr) {side_docks.append(displays); widths.append(280);}
  if (cameras != nullptr) {side_docks.append(cameras); widths.append(500);}
  if (!side_docks.isEmpty()) {frame.resizeDocks(side_docks, widths, Qt::Horizontal);}
  frame.resizeDocks({&teleop}, {210}, Qt::Vertical);
}

void connect_logging(const rclcpp::Logger & logger)
{
  rviz_common::set_logging_handlers(
    [logger](const std::string & message, const std::string &, size_t) {
      RCLCPP_DEBUG(logger, "%s", message.c_str());
    },
    [logger](const std::string & message, const std::string &, size_t) {
      RCLCPP_INFO(logger, "%s", message.c_str());
    },
    [logger](const std::string & message, const std::string &, size_t) {
      RCLCPP_WARN(logger, "%s", message.c_str());
    },
    [logger](const std::string & message, const std::string &, size_t) {
      RCLCPP_ERROR(logger, "%s", message.c_str());
    });
}

void log_renderer_identity(const rclcpp::Logger & logger)
{
  // These public getters read Ogre's initialized capabilities cache. Do not
  // select a context, issue GL calls, or infer the device from environment flags.
  auto * root = Ogre::Root::getSingletonPtr();
  auto * system = root != nullptr ? root->getRenderSystem() : nullptr;
  const auto * capabilities = system != nullptr ? system->getCapabilities() : nullptr;
  if (capabilities == nullptr) {
    RCLCPP_WARN(logger, "Initialized Ogre renderer identity unavailable");
    return;
  }
  RCLCPP_INFO(logger, "Actual Ogre renderer: device=%s, vendor_category=%s, parsed_GL_version=%s "
    "(GL version parsed by Ogre; not the Mesa/NVIDIA package version)",
    capabilities->getDeviceName().c_str(),
    Ogre::RenderSystemCapabilities::vendorToString(capabilities->getVendor()).c_str(),
    capabilities->getDriverVersion().toString().c_str());
}

}  // namespace

int main(int argc, char ** argv)
{
  const auto logger = rclcpp::get_logger("mapping_rviz");
  try {
    // Qt consumes only its arguments; native RViz receives the original ROS
    // remappings as well as -d, -f and the other standard RViz options.
    auto qt_arguments = rclcpp::remove_ros_arguments(argc, argv);
    std::vector<char *> qt_argv;
    qt_argv.reserve(qt_arguments.size() + 1);
    for (auto & argument : qt_arguments) {
      qt_argv.push_back(&argument[0]);
    }
    int qt_argc = static_cast<int>(qt_argv.size());
    qt_argv.push_back(nullptr);
    QApplication application(qt_argc, qt_argv.data());
    connect_logging(logger);

    rviz_common::VisualizerApp native_rviz(
      std::make_unique<rviz_common::ros_integration::RosClientAbstraction>());
    native_rviz.setApp(&application);
    if (!native_rviz.init(argc, argv)) {
      return 1;
    }
    log_renderer_identity(logger);

    // VisualizerApp deliberately keeps its frame pointer private. Query the
    // public Qt widget tree instead of modifying RViz or relying on its layout.
    rviz_common::VisualizationFrame * frame = nullptr;
    for (QWidget * widget : QApplication::topLevelWidgets()) {
      if (auto * candidate = qobject_cast<rviz_common::VisualizationFrame *>(widget)) {
        if (frame != nullptr) {
          RCLCPP_ERROR(logger, "Multiple native RViz frames; close policy not installed");
          return 1;
        }
        frame = candidate;
      }
    }
    if (frame == nullptr) {
      RCLCPP_ERROR(logger, "Native RViz frame unavailable; close policy not installed");
      return 1;
    }

    // Stack lifetime removes the filter before VisualizerApp destroys its frame.
    SessionViewCloseFilter close_filter(*frame);
    QStringList original_qt_arguments;
    for (const auto & argument : qt_arguments) {
      original_qt_arguments.append(QString::fromStdString(argument));
    }
    auto * teleop_dock = new QDockWidget(QStringLiteral("WASD / 手推建图"), frame);
    teleop_dock->setObjectName(QStringLiteral("mapping_teleop_dock"));
    auto * bottom = new QWidget(teleop_dock);
    auto * bottom_layout = new QVBoxLayout(bottom); bottom_layout->setContentsMargins(0, 0, 0, 0);
    bottom_layout->addWidget(new wc_bringup::MappingHealthPanel(original_qt_arguments, bottom));
    bottom_layout->addWidget(new wc_bringup::MappingTeleopPanel(
      wc_bringup::teleop_session_from_arguments(original_qt_arguments), frame, bottom));
    teleop_dock->setWidget(bottom);
    frame->addDockWidget(Qt::BottomDockWidgetArea, teleop_dock);
    const QString interactive_directory = qEnvironmentVariable("WC_MAPPING_CONTROL_DIR");
    if (qEnvironmentVariable("WC_PANEL_LAYOUT") == "unified" || !interactive_directory.isEmpty()) {
      arrange_unified(*frame, *teleop_dock, original_qt_arguments);
    } else {arrange_initial_docks(*frame, *teleop_dock);}
    if (!interactive_directory.isEmpty()) {
      auto * binding = new wc_bringup::MappingSessionView(*frame, *teleop_dock);
      auto * controls = new wc_bringup::mapping_control::Panel(interactive_directory, frame,
        [binding](const QJsonObject & status) {binding->bind(status);},
        [binding]() {binding->clear();}, bottom);
      bottom_layout->insertWidget(0, controls);
    }
    // Let native dock/tab and font-metric layout requests settle, then perform
    // two bounded startup fits. There is no persistent resize policy that can
    // fight a user's later window size or panel changes.
    QTimer::singleShot(0, frame, [frame, logger]() {fit_initial_window(*frame, logger);});
    QTimer::singleShot(250, frame, [frame, logger]() {fit_initial_window(*frame, logger);});
    maybe_start_render_diagnostics(*frame, original_qt_arguments, logger);
    wc_bringup::install_panel_diagnostics(frame, "mapping", [frame]() {
      QJsonArray labels;
      for (auto * label : frame->findChildren<QLabel *>()) {
        if (label->objectName().startsWith("status_")) {
          labels.append(QJsonObject{{"name", label->objectName()}, {"text", label->text()}});
        }
      }
      QJsonArray grids;
      // RvizViewport intentionally has no Q_OBJECT. Inspect named widgets by
      // dynamic_cast instead of Qt's inherited metaobject cast.
      for (auto * widget : frame->findChildren<QWidget *>()) {
        if (auto * viewport = dynamic_cast<wc_bringup::RvizViewport *>(widget)) {
          grids.append(QJsonObject{{"name", viewport->objectName()},
            {"frames", static_cast<double>(viewport->manager()->getFrameCount())},
            {"independent_view", viewport->manager()->getSceneManager() != frame->getManager()->getSceneManager()
              && viewport->manager()->getRenderPanel() != frame->getManager()->getRenderPanel()
              && viewport->manager()->getViewManager() != frame->getManager()->getViewManager()},
            {"color_materials_retained", viewport->property("rviz_default_colors_retained").toInt()},
            {"native_window", viewport->nativeObservation()},
            {"grid_fit", viewport->property("grid_fit").toJsonObject()},
            {"displays", wc_bringup::display_observation(viewport->manager())}});
        }
      }
      QJsonObject interactive{{"session", frame->property("mapping_bound_session").toString()},
        {"view_generation", frame->property("mapping_bound_view_generation").toDouble()},
        {"bind_count", frame->property("mapping_bind_count").toInt()},
        {"clear_count", frame->property("mapping_clear_count").toInt()},
        {"tf_instance_replaced", frame->property("mapping_tf_instance_replaced").toBool()}};
      if (auto * controls = frame->findChild<QWidget *>("mapping_interactive_controls")) {
        interactive["state"] = controls->property("mapping_state").toString();
        interactive["pending"] = controls->property("command_pending").toBool();
        interactive["failure"] = controls->property("protocol_failure").toString();
      }
      return QJsonObject{{"camera_states", labels}, {"interactive", interactive},
        {"displays", wc_bringup::display_observation(frame->getManager())},
        {"frames", static_cast<double>(frame->getManager()->getFrameCount())}, {"secondary_views", grids}};
    });
    RCLCPP_INFO(logger, "Native RViz ready; closing discards temporary view edits. "
      "The mapping application owns map saving.");
    return application.exec();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger, "RViz initialization failed: %s", error.what());
    return 1;
  }
}
