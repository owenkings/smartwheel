// SPDX-License-Identifier: Apache-2.0
// Test-only fixture injected into a private copy of mapping_rviz.cpp.
// No ROS publisher, device access, control process, or production session parser.
#ifndef WC_TESTS__SYNTHETIC_MANUAL_RENDER_HPP_
#define WC_TESTS__SYNTHETIC_MANUAL_RENDER_HPP_

#include <algorithm>
#include <stdexcept>
#include <QApplication>
#include <QDir>
#include <QDockWidget>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLocalServer>
#include <QLocalSocket>
#include <QPixmap>
#include <QPushButton>
#include <QScreen>
#include <QTemporaryDir>
#include <QTimer>
#include <QWindow>
#include "rviz_common/visualization_frame.hpp"
#include "rviz_rendering/render_window.hpp"
#include "wc_bringup/mapping_teleop.hpp"

namespace synthetic_manual_render
{
inline QJsonArray rect(const QRect & value)
{return {value.x(), value.y(), value.width(), value.height()};}
inline QJsonArray size(const QSize & value)
{return {value.width(), value.height()};}

inline QJsonObject geometry(QWidget * widget)
{
  if (widget == nullptr) {return {{"present", false}};}
  QJsonObject result{{"present", true}, {"class", widget->metaObject()->className()},
    {"name", widget->objectName()}, {"geometry", rect(widget->geometry())},
    {"frame_geometry", rect(widget->frameGeometry())}, {"size_hint", size(widget->sizeHint())},
    {"minimum_size_hint", size(widget->minimumSizeHint())},
    {"minimum_size", size(widget->minimumSize())}, {"visible", widget->isVisible()}};
  if (auto * window = widget->windowHandle()) {
    result["qwindow"] = QJsonObject{{"geometry", rect(window->geometry())},
      {"visible", window->isVisible()}, {"exposed", window->isExposed()}};
  }
  return result;
}

class Fixture final : public QObject
{
public:
  Fixture(rviz_common::VisualizationFrame & frame, QApplication & application)
  : frame_(frame), application_(application), temporary_("/tmp/wc_synthetic_manual_XXXXXX")
  {
    if (qEnvironmentVariable("ROS_DOMAIN_ID") != "84" ||
      qEnvironmentVariable("ROS_LOCALHOST_ONLY") != "1" || !temporary_.isValid())
    {throw std::runtime_error("Synthetic fixture requires localhost domain 84 and a private tempdir");}
    output_ = qEnvironmentVariable("WC_SYNTHETIC_MANUAL_OUTPUT");
    QFileInfo info(output_);
    if (!info.isAbsolute() || info.exists() || info.isSymLink() || !info.dir().exists()) {
      throw std::runtime_error("Synthetic output must be a new absolute directory");
    }
    for (QDir ancestor = info.dir(); ; ) {
      if (QFileInfo(ancestor.absolutePath()).isSymLink()) {
        throw std::runtime_error("Synthetic output cannot traverse symlinks");
      }
      if (!ancestor.cdUp()) {break;}
    }
    if (!QDir().mkdir(output_)) {throw std::runtime_error("Cannot create synthetic output directory");}
    server_.setSocketOptions(QLocalServer::UserAccessOption);
    const QString socket = temporary_.filePath("manual.sock");
    if (!server_.listen(socket) || !QFile::setPermissions(socket, QFileDevice::ReadOwner | QFileDevice::WriteOwner)) {
      throw std::runtime_error("Cannot create private synthetic 0600 socket");
    }
    connect(&server_, &QLocalServer::newConnection, this, [this]() {
      if (peer_ != nullptr) {fail("Unexpected second synthetic socket connection"); return;}
      peer_ = server_.nextPendingConnection();
      peer_->setReadBufferSize(8192);
      connect(peer_, &QLocalSocket::readyRead, this, [this]() {receive();});
      connect(peer_, &QLocalSocket::disconnected, this, [this]() {
        if (!finishing_) {fail("Synthetic panel disconnected before normal window close");}
      });
    });
    connect(&application_, &QCoreApplication::aboutToQuit, this, [this]() {finishing_ = true;});
    write("identity.json", {{"source_mode", "synthetic"}, {"synthetic", true},
      {"session_id", identifier_}, {"socket_directory", temporary_.path()},
      {"hardware_started", false}, {"control_process_started", false}, {"arm_allowed", false},
      {"long_text_origin", "Current mapping_wheel.DISABLED_REASON, copied as text without importing its module"},
      {"long_arm_block_reason", QString::fromUtf8(WC_SYNTHETIC_BLOCK_REASON)}});
  }

  wc_bringup::TeleopSession session() const
  {
    wc_bringup::TeleopSession value;
    value.directory = temporary_.path(); value.session_id = identifier_;
    return value;
  }

  void start(QDockWidget & dock)
  {
    dock_ = &dock;
    started_.start(); last_tick_ns_ = 0;
    timer_.setInterval(100); timer_.setTimerType(Qt::PreciseTimer);
    connect(&timer_, &QTimer::timeout, this, [this]() {
      try {tick();} catch (const std::exception & error) {fail(QString::fromUtf8(error.what()));}
    });
    timer_.start();
  }

  int finish(int application_code)
  {
    finishing_ = true; timer_.stop();
    if (application_code != 0 && failure_.isEmpty()) {failure_ = "Application returned nonzero";}
    if (!complete_ && failure_.isEmpty()) {failure_ = "Window closed before both diagnostic phases completed";}
    if (peer_ != nullptr) {peer_->abort();}
    server_.close();
    auto result = summary(); result["application_exit_code"] = application_code;
    result["observations"] = observations_; result["client_messages"] = client_messages_;
    write("final.json", result);
    return failure_.isEmpty() ? 0 : 1;
  }

private:
  void write(const QString & name, const QJsonObject & object)
  {
    const QString path = QDir(output_).filePath(name);
    QFile file(path+".partial");
    const QByteArray bytes = QJsonDocument(object).toJson(QJsonDocument::Indented);
    if (!file.open(QIODevice::WriteOnly | QIODevice::NewOnly) || file.write(bytes) != bytes.size() || !file.flush()) {
      throw std::runtime_error("Cannot write exclusive synthetic evidence file");
    }
    file.close();
    // Publish complete JSON atomically; a reader must never see a partial row.
    if (!file.rename(path)) {throw std::runtime_error("Cannot finalize exclusive synthetic evidence file");}
  }

  void fail(const QString & reason)
  {
    if (failure_.isEmpty()) {failure_ = reason;}
    timer_.stop(); application_.exit(1);
  }

  void receive()
  {
    incoming_ += peer_->readAll();
    if (incoming_.size() > 8192) {fail("Synthetic client byte bound exceeded"); return;}
    int newline;
    while ((newline = incoming_.indexOf('\n')) >= 0) {
      const QByteArray line = incoming_.left(newline); incoming_.remove(0, newline+1);
      QJsonParseError error;
      const auto document = QJsonDocument::fromJson(line, &error);
      if (line.size() >= 4096 || error.error != QJsonParseError::NoError || !document.isObject()) {
        fail("Invalid synthetic client message"); return;
      }
      const auto message = document.object(); client_messages_.append(message);
      // No permissive arm/keys/disarm simulation: hello is the entire allowlist.
      if (message.value("type") != "hello" || message.value("session_id") != identifier_ || hello_count_ != 0) {
        fail("Forbidden client intent or synthetic identity mismatch"); return;
      }
      // write()/flush() only queue the reply; the client's readyRead and label
      // update run in a later event-loop turn. Do not start a visible phase yet.
      ++hello_count_; first_status_age_.start(); send_status();
    }
  }

  void send_status()
  {
    if (peer_ == nullptr || hello_count_ != 1 || peer_->state() != QLocalSocket::ConnectedState) {return;}
    const QJsonObject status{{"type", "status"}, {"session_id", identifier_},
      {"state", "DISARMED"}, {"reason", "STARTUP_READ_ONLY"}, {"arm_allowed", false},
      {"arm_generation", 0}, {"max_linear_m_s", .1}, {"max_angular_rad_s", .2},
      {"arm_block_reason", long_phase_ ? QString::fromUtf8(WC_SYNTHETIC_BLOCK_REASON) : QString("SYNTHETIC")}};
    const QByteArray bytes = QJsonDocument(status).toJson(QJsonDocument::Compact)+'\n';
    if (bytes.size() >= 4096 || peer_->bytesToWrite() > 4096 || peer_->write(bytes) != bytes.size()) {
      fail("Synthetic status socket write failed or exceeded byte bound"); return;
    }
    peer_->flush(); ++sent_;
    if (long_phase_) {++long_sent_;} else {++short_sent_;}
  }

  QJsonObject observe(const QString & phase)
  {
    auto * render = frame_.getRenderWindow();
    QJsonObject value{{"elapsed_ms", static_cast<double>(started_.elapsed())}, {"phase", phase},
      {"frame", geometry(&frame_)}, {"central", geometry(frame_.centralWidget())},
      {"manual_dock", geometry(dock_)}};
    if (render != nullptr) {
      value["render_qwindow"] = QJsonObject{{"geometry", rect(render->geometry())},
        {"visible", render->isVisible()}, {"exposed", render->isExposed()}};
    }
    for (const QString name : {QString("teleop_state"), QString("teleop_reason"), QString("teleop_limits")}) {
      if (auto * label = dock_->findChild<QLabel *>(name)) {
        auto item = geometry(label); item["text"] = label->text();
        item["word_wrap"] = label->wordWrap(); value[name] = item;
      }
    }
    return value;
  }

  void capture(const QString & phase)
  {
    QElapsedTimer cost; cost.start();
    auto value = observe(phase);
    auto * reason = dock_->findChild<QLabel *>("teleop_reason");
    const QString expected = long_phase_ ? QString::fromUtf8(WC_SYNTHETIC_BLOCK_REASON) : QString("SYNTHETIC");
    if (reason == nullptr || !reason->text().contains(expected)) {
      throw std::runtime_error("Expected synthetic phase text has not reached the visible panel");
    }
    auto * render = frame_.getRenderWindow();
    if (render == nullptr) {throw std::runtime_error("Native render window is unavailable");}
    const QString native_path = QDir(output_).filePath(phase+"_ogre.png");
    render->captureScreenShot(native_path.toStdString());
    const qint64 native_ms = cost.elapsed();
    QScreen * screen = frame_.windowHandle() != nullptr ? frame_.windowHandle()->screen() : nullptr;
    if (screen == nullptr) {throw std::runtime_error("Owned frame screen unavailable");}
    const QString client_path = QDir(output_).filePath(phase+"_qt_screen.png");
    if (!screen->grabWindow(frame_.winId()).save(client_path) || QFileInfo(native_path).size() <= 0) {
      throw std::runtime_error("Native/owned-window screenshot failed");
    }
    value["ogre_capture_ms"] = static_cast<double>(native_ms);
    value["both_captures_ms"] = static_cast<double>(cost.elapsed());
    value["ogre_file"] = native_path; value["qt_screen_file"] = client_path;
    // No renderNow/windowMovedOrResized call: observe the current rendering state.
    observations_.append(value);
    last_capture_ms_ = static_cast<double>(cost.elapsed());
  }

  QJsonObject summary() const
  {
    return {{"test", "SYNTHETIC_DISARMED_MANUAL_NATIVE_RENDER"},
      {"status", failure_.isEmpty() && complete_ ? "PASS" : "FAIL"}, {"failure", failure_},
      {"synthetic", true}, {"source_mode", "synthetic"}, {"session_id", identifier_},
      {"hardware_started", false}, {"control_process_started", false}, {"arm_allowed", false},
      {"hello_count", hello_count_}, {"status_messages", sent_},
      {"first_status_visible", first_status_visible_},
      {"first_status_wait_limit_ms", 1000}, {"first_status_wait_ticks", first_status_wait_ticks_},
      {"first_status_visible_after_hello_ms", first_status_visible_after_hello_ms_},
      {"short_status_messages", short_sent_}, {"long_status_messages", long_sent_},
      {"short_phase_minimum_ms", 3000}, {"long_phase_minimum_ms", 12000},
      {"timer_interval_ms", 100}, {"maximum_timer_gap_ms", max_tick_gap_ms_},
      {"last_capture_ms", last_capture_ms_}, {"capture_runs_on_status_event_thread", true},
      {"capture_interference_note", "Synchronous capture/PNG writes can delay the mock timer; inspect capture durations before attributing a disconnect to live behavior"},
      {"visual_review_status", "PENDING"},
      {"pass_meaning", "Both synthetic DISARMED phases completed without control intent; screenshots require visual review"}};
  }

  void tick()
  {
    const qint64 now = started_.nsecsElapsed();
    if (last_tick_ns_ != 0) {max_tick_gap_ms_ = std::max(max_tick_gap_ms_, (now-last_tick_ns_)/1.e6);}
    last_tick_ns_ = now;
    if (started_.elapsed() >= 35000) {fail("Synthetic window lifetime exceeded 35 seconds"); return;}
    if (hello_count_ == 0) {
      if (started_.elapsed() >= 10000) {fail("Synthetic hello was not received within 10 seconds");}
      return;
    }
    auto * arm = dock_->findChild<QPushButton *>("teleop_arm");
    auto * state = dock_->findChild<QLabel *>("teleop_state");
    auto * reason = dock_->findChild<QLabel *>("teleop_reason");
    if (arm == nullptr || arm->isEnabled() || state == nullptr || reason == nullptr) {
      observations_.append(observe("invalid_panel"));
      fail("Panel is not visibly DISARMED with activation disabled"); return;
    }
    const QString disarmed = QStringLiteral("手动控制：DISARMED | 会话：")+identifier_;
    if (!first_status_visible_) {
      const QString waiting = QStringLiteral("手动控制：WAITING_STATUS | 会话：")+identifier_;
      // Only this exact initial transport state gets a bounded wait. A fault,
      // disconnect, unexpected state or enabled Arm button is never retried.
      if (state->text() == waiting && first_status_age_.elapsed() < 1000) {
        ++first_status_wait_ticks_; observations_.append(observe("awaiting_first_status"));
        send_status(); return;
      }
      if (first_status_age_.elapsed() >= 1000 || state->text() != disarmed ||
        reason->text() != QStringLiteral("激活已禁用：SYNTHETIC | STARTUP_READ_ONLY"))
      {
        observations_.append(observe("first_status_rejected"));
        fail("Initial synthetic DISARMED status was not visibly confirmed within 1000 ms"); return;
      }
      first_status_visible_ = true;
      first_status_visible_after_hello_ms_ = static_cast<double>(first_status_age_.elapsed());
      phase_age_.start(); observations_.append(observe("first_status_confirmed"));
    } else if (state->text() != disarmed) {
      observations_.append(observe("established_status_rejected"));
      fail("Established synthetic panel left DISARMED; startup wait is not reusable"); return;
    }
    send_status();
    if (!failure_.isEmpty()) {return;}
    if (sent_ % 10 == 0) {observations_.append(observe(long_phase_ ? "long" : "short"));}
    if (!long_phase_ && phase_age_.elapsed() >= 3000) {
      capture("short"); long_phase_ = true; phase_age_.restart(); send_status();
    } else if (long_phase_ && !complete_ && phase_age_.elapsed() >= 12000) {
      capture("long"); complete_ = true;
      // The harness waits for this exclusive file, captures the external X11
      // window, then closes only its owned process. Keep status fresh meanwhile.
      write("phases_complete.json", summary());
    }
  }

  rviz_common::VisualizationFrame & frame_;
  QApplication & application_;
  QTemporaryDir temporary_;
  QLocalServer server_;
  QLocalSocket * peer_ = nullptr;
  QDockWidget * dock_ = nullptr;
  QTimer timer_;
  QElapsedTimer started_, phase_age_, first_status_age_;
  QString output_, failure_;
  const QString identifier_ = "synthetic_manual_dashboard_left_20260914_01";
  QByteArray incoming_;
  QJsonArray observations_, client_messages_;
  int hello_count_ = 0, sent_ = 0, short_sent_ = 0, long_sent_ = 0;
  int first_status_wait_ticks_ = 0;
  bool first_status_visible_ = false;
  bool long_phase_ = false, complete_ = false, finishing_ = false;
  qint64 last_tick_ns_ = 0;
  double max_tick_gap_ms_ = 0.;
  double last_capture_ms_ = 0.;
  double first_status_visible_after_hello_ms_ = -1.;
};
}  // namespace synthetic_manual_render
#endif  // WC_TESTS__SYNTHETIC_MANUAL_RENDER_HPP_
