// SPDX-License-Identifier: Apache-2.0
// SYNTHETIC: Qt events and a private mock QLocalServer. No ROS or device access.
#include <QApplication>
#include <QCloseEvent>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QLocalServer>
#include <QLocalSocket>
#include <QPushButton>
#include <QTemporaryDir>
#include <QTest>
#include <QTimer>
#include <QVBoxLayout>
#include "wc_bringup/mapping_teleop.hpp"
#include "wc_bringup/mapping_render_diagnostics.hpp"

class TeleopTest : public QObject
{
  Q_OBJECT
private:
  QTemporaryDir directory_;
  QLocalServer * server_ = nullptr;
  QLocalSocket * peer_ = nullptr;
  QWidget * window_ = nullptr;
  wc_bringup::MappingTeleopPanel * panel_ = nullptr;
  QWidget * control_ = nullptr;
  QLineEdit * text_ = nullptr;
  QTimer * status_timer_ = nullptr;
  QJsonObject status_;
  QList<QJsonObject> received_;
  int confirmation_calls_ = 0;
  bool suppress_status_ = false;

  wc_bringup::TeleopSession diagnostic_session(const QTemporaryDir & directory)
  {
    // SYNTHETIC metadata exercises the live-session parser; no ROS or devices.
    for (const QString name : {QString("view.rviz"), QString("runtime_config.json"), QString("session.json")}) {
      QFile file(directory.filePath(name));
      if (!file.open(QIODevice::WriteOnly | QIODevice::NewOnly)) {
        throw std::runtime_error("Cannot create synthetic diagnostic identity");
      }
      const QByteArray bytes = name == "view.rviz" ? QByteArray("# SYNTHETIC parser fixture\n") :
        QByteArray("{\"session_id\":\"SYNTHETIC_render_diagnostic\",\"source_mode\":\"real\",\"status\":\"EXPERIMENT\"}");
      if (file.write(bytes) != bytes.size()) {throw std::runtime_error("Cannot write synthetic identity");}
    }
    return wc_bringup::teleop_session_from_arguments({"mapping_rviz", "-d", directory.filePath("view.rviz")});
  }
  bool start_server()
  {
    if (!server_->listen(directory_.filePath("manual.sock"))) {return false;}
#ifdef Q_OS_UNIX
    return QFile::setPermissions(directory_.filePath("manual.sock"), QFileDevice::ReadOwner | QFileDevice::WriteOwner);
#else
    return true;
#endif
  }
  void status()
  {
    if (!suppress_status_ && peer_ != nullptr && peer_->state() == QLocalSocket::ConnectedState) {
      peer_->write(QJsonDocument(status_).toJson(QJsonDocument::Compact) + '\n'); peer_->flush();
    }
  }
  QList<QJsonObject> messages(const QString & kind) const
  {
    QList<QJsonObject> result;
    for (const auto & item : received_) {if (item.value("type").toString() == kind) {result.append(item);}}
    return result;
  }
  QJsonObject latest(const QString & kind) const
  {
    const auto items = messages(kind);
    return items.isEmpty() ? QJsonObject() : items.last();
  }
  void allow_keys()
  {
    status_["arm_allowed"] = true; status_["arm_block_reason"] = QJsonValue();
    status_["state"] = "READY"; status();
    QTRY_VERIFY(panel_->findChild<QLabel *>("teleop_state")->text().contains("READY"));
  }
  void finish_initialization()
  {
    status_["state"] = "ARMED"; status_["reason"] = "ARMED_MANUAL_KEYS"; status();
    QTRY_VERIFY(panel_->findChild<QLabel *>("teleop_state")->text().contains("ARMED |"));
  }
  void ready_after_stop()
  {
    QTRY_VERIFY(panel_->findChild<QLabel *>("teleop_state")->text().contains("READY"));
  }
private slots:
  void init()
  {
    QVERIFY(directory_.isValid()); received_.clear(); peer_ = nullptr; confirmation_calls_ = 0;
    suppress_status_ = false;
    status_ = {{"type", "status"}, {"session_id", "synthetic_teleop"}, {"state", "DISARMED"},
      {"reason", "READY_FEEDBACK_ONLY"}, {"arm_allowed", false}, {"arm_generation", 0},
      {"arm_block_reason", "USB/RS485 physical disconnect stop is unverified"},
      {"max_linear_m_s", .1}, {"max_angular_rad_s", .2}};
    server_ = new QLocalServer(this); server_->setSocketOptions(QLocalServer::UserAccessOption);
    const bool delayed_start = QString::fromLatin1(QTest::currentTestFunction()).startsWith("startup_");
    if (!delayed_start) {QVERIFY(start_server());}
    connect(server_, &QLocalServer::newConnection, this, [this]() {
        auto * accepted = server_->nextPendingConnection(); peer_ = accepted;
        connect(accepted, &QLocalSocket::readyRead, this, [this, accepted, incoming = QByteArray()]() mutable {
            incoming += accepted->readAll();
            int newline;
            while ((newline = incoming.indexOf('\n')) >= 0) {
              const auto item = QJsonDocument::fromJson(incoming.left(newline)).object();
              incoming.remove(0, newline + 1); received_.append(item);
              if (item.value("type") == "hello") {status();}
              if (item.value("type") == "keys" && item.value("arm_generation") == status_["arm_generation"]) {
                if (!item.value("keys").toArray().isEmpty() && status_["state"] == "READY") {
                  status_["state"] = "INITIALIZING"; status();
                } else if (item.value("keys").toArray().isEmpty() &&
                  (status_["state"] == "INITIALIZING" || status_["state"] == "ARMED"))
                {
                  status_["state"] = "READY";
                  status_["arm_generation"] = status_["arm_generation"].toInt() + 1; status();
                }
              }
              if (item.value("type") == "disarm") {
                status_["state"] = status_["arm_allowed"].toBool() ? "READY" : "DISARMED";
                status_["arm_generation"] = status_["arm_generation"].toInt() + 1; status();
              }
            }
          });
      });
    window_ = new QWidget(); auto * layout = new QVBoxLayout(window_);
    wc_bringup::TeleopSession session; session.directory = directory_.path(); session.session_id = "synthetic_teleop";
    panel_ = new wc_bringup::MappingTeleopPanel(session, window_, window_, [this]() {
        ++confirmation_calls_; return false;
      });
    text_ = new QLineEdit(window_); layout->addWidget(panel_); layout->addWidget(text_);
    control_ = panel_->findChild<QWidget *>("teleop_control_area"); QVERIFY(control_ != nullptr);
    window_->show(); QApplication::setActiveWindow(window_); text_->setFocus();
    status_timer_ = new QTimer(this); status_timer_->setInterval(100);
    connect(status_timer_, &QTimer::timeout, this, [this]() {status();}); status_timer_->start();
    if (!delayed_start) {
      QTRY_COMPARE(messages("hello").size(), 1);
      QTRY_VERIFY(panel_->findChild<QLabel *>("teleop_state")->text().contains("DISARMED"));
    }
  }
  void cleanup()
  {
    status_timer_->stop(); delete status_timer_; status_timer_ = nullptr;
    delete window_; window_ = nullptr; panel_ = nullptr;
    if (peer_ != nullptr) {peer_->abort();}
    server_->close(); delete server_; server_ = nullptr; peer_ = nullptr;
    QCoreApplication::processEvents();
  }
  void disabled_backend_never_sends_control()
  {
    QVERIFY(panel_->findChild<QPushButton *>("teleop_connect") == nullptr);
    QVERIFY(panel_->findChild<QPushButton *>("teleop_arm") == nullptr);
    QCOMPARE(panel_->findChild<QLabel *>("teleop_connection")->text(), QString::fromUtf8("已连接本会话"));
    QVERIFY(panel_->findChild<QLabel *>("teleop_state")->text().contains(QString::fromUtf8("已连接，仅只读")));
    QVERIFY(panel_->findChild<QLabel *>("teleop_control_guide")->text().contains(QString::fromUtf8("WASD 不会驱动车辆")));
    QVERIFY(panel_->findChild<QLabel *>("teleop_reason")->text().contains("USB/RS485"));
    QTest::keyPress(text_, Qt::Key_W); QTest::qWait(160); QTest::keyRelease(text_, Qt::Key_W);
    QCOMPARE(messages("hello").size(), 1); QCOMPARE(messages("arm").size(), 0); QCOMPARE(messages("keys").size(), 0);
  }
  void permission_display_follows_backend_without_driving()
  {
    allow_keys(); QTest::qWait(150);
    QVERIFY(!panel_->findChild<QLabel *>("teleop_state")->text().contains(QString::fromUtf8("仅只读")));
    QCOMPARE(messages("arm").size(), 0); QCOMPARE(messages("keys").size(), 0);
    status_["arm_allowed"] = false; status_["arm_block_reason"] = "SYNTHETIC permission revoked"; status();
    QTRY_VERIFY(panel_->findChild<QLabel *>("teleop_state")->text().contains(QString::fromUtf8("已连接，仅只读")));
    QVERIFY(panel_->findChild<QLabel *>("teleop_reason")->text().contains("permission revoked"));
    QTest::keyPress(text_, Qt::Key_D); QTest::qWait(100);
    QCOMPARE(messages("keys").size(), 0);
  }
  void first_key_initializes_and_stays_held_without_buttons_or_confirmation()
  {
    allow_keys(); QVERIFY(text_->hasFocus());
    QTest::keyPress(text_, Qt::Key_W);
    QTRY_COMPARE(latest("keys")["keys"].toArray(), QJsonArray({"w"}));
    QTRY_VERIFY(panel_->findChild<QLabel *>("teleop_state")->text().contains("INITIALIZING"));
    const int count = messages("keys").size(); QTest::qWait(150);
    QVERIFY(messages("keys").size() > count); finish_initialization(); QTest::qWait(100);
    QCOMPARE(latest("keys")["keys"].toArray(), QJsonArray({"w"}));
    QCOMPARE(latest("keys")["arm_generation"].toInt(), 0);
    QCOMPARE(messages("arm").size(), 0); QCOMPARE(confirmation_calls_, 0);
    QVERIFY(QApplication::activeModalWidget() == nullptr); QVERIFY(text_->text().isEmpty());
    QTest::keyRelease(text_, Qt::Key_W); ready_after_stop();
    QVERIFY(latest("keys")["keys"].toArray().isEmpty());
    int sequence = 0;
    for (const auto & item : messages("keys")) {QVERIFY(item["sequence"].toInt() > sequence); sequence = item["sequence"].toInt();}
  }
  void hand_push_requires_button_and_blocks_keys_until_explicit_exit()
  {
    allow_keys(); auto * push = panel_->findChild<QPushButton *>("teleop_push_mode"); QVERIFY(push != nullptr);
    QTest::qWait(200); QCOMPARE(messages("push_mode").size(), 0);
    QVERIFY(push->isEnabled()); QTest::mouseClick(push, Qt::LeftButton);
    QTRY_COMPARE(messages("push_mode").size(), 1);
    QCOMPARE(latest("push_mode")["enabled"].toBool(), true);
    status_["arm_generation"] = status_["arm_generation"].toInt()+1;
    status_["push_mode"] = true; status_["push_mode_requested"] = false; status();
    QTRY_COMPARE(push->text(), QString::fromUtf8("退出手推"));
    QTest::keyPress(text_, Qt::Key_W); QTest::qWait(120); QTest::keyRelease(text_, Qt::Key_W);
    QCOMPARE(messages("keys").size(), 0);
    QTest::mouseClick(push, Qt::LeftButton); QTRY_COMPARE(messages("push_mode").size(), 2);
    QCOMPARE(latest("push_mode")["enabled"].toBool(), false);
    status_["arm_generation"] = status_["arm_generation"].toInt()+1;
    status_["push_mode"] = false; status();
    QTRY_COMPARE(push->text(), QString::fromUtf8("进入手推"));
    QTest::qWait(120); QCOMPARE(messages("keys").size(), 0);
    QTest::keyPress(text_, Qt::Key_S); QTRY_COMPARE(latest("keys")["keys"].toArray(), QJsonArray({"s"}));
    QTest::keyRelease(text_, Qt::Key_S);
  }
  void hybrid_foreground_idle_heartbeats_need_no_allow_or_mode_button()
  {
    status_["interaction_policy"] = "hybrid_manual"; allow_keys();
    QTRY_VERIFY(messages("keys").size() >= 2);
    QCOMPARE(latest("keys")["foreground"].toBool(), true);
    QVERIFY(latest("keys")["keys"].toArray().isEmpty());
    QVERIFY(panel_->findChild<QPushButton *>("teleop_push_mode")->isHidden());
    QCOMPARE(messages("arm").size(), 0); QCOMPARE(messages("push_mode").size(), 0);
    QCOMPARE(confirmation_calls_, 0);
  }
  void hybrid_wasd_takes_over_push_and_immediate_repress_uses_new_generation()
  {
    status_["interaction_policy"] = "hybrid_manual";
    status_["push_mode"] = true; allow_keys();
    QTest::keyPress(text_, Qt::Key_W);
    QTRY_COMPARE(latest("keys")["keys"].toArray(), QJsonArray({"w"}));
    QVERIFY(latest("keys")["foreground"].toBool()); finish_initialization();
    QTest::keyRelease(text_, Qt::Key_W); QTest::keyPress(text_, Qt::Key_D);
    QTRY_COMPARE(latest("keys")["keys"].toArray(), QJsonArray({"d"}));
    QCOMPARE(latest("keys")["arm_generation"].toInt(), 1);
    QCOMPARE(messages("push_mode").size(), 0); QCOMPARE(messages("arm").size(), 0);
    QTest::keyRelease(text_, Qt::Key_D);
  }
  void hybrid_focus_loss_is_false_heartbeat_and_reentry_needs_new_key()
  {
    status_["interaction_policy"] = "hybrid_manual"; allow_keys();
    QTest::keyPress(text_, Qt::Key_W); finish_initialization();
    QEvent leave(QEvent::WindowDeactivate); QApplication::sendEvent(window_, &leave);
    QTRY_COMPARE(messages("disarm").size(), 1);
    QTRY_VERIFY(latest("keys").contains("foreground") && !latest("keys")["foreground"].toBool());
    QVERIFY(latest("keys")["keys"].toArray().isEmpty());
    const int after_leave = messages("keys").size();
    QEvent enter(QEvent::WindowActivate); QApplication::sendEvent(window_, &enter);
    QTRY_VERIFY(messages("keys").size() > after_leave && latest("keys")["foreground"].toBool());
    QVERIFY(latest("keys")["keys"].toArray().isEmpty());
    QKeyEvent repeat(QEvent::KeyPress, Qt::Key_W, Qt::NoModifier, "w", true, 1);
    QApplication::sendEvent(text_, &repeat); QTest::qWait(100);
    QVERIFY(latest("keys")["keys"].toArray().isEmpty());
    QTest::keyRelease(text_, Qt::Key_W); QTest::keyPress(text_, Qt::Key_D);
    QTRY_COMPARE(latest("keys")["keys"].toArray(), QJsonArray({"d"}));
    QCOMPARE(latest("keys")["arm_generation"].toInt(), 1);
    QCOMPARE(messages("push_mode").size(), 0);
    QTest::keyRelease(text_, Qt::Key_D);
  }
  void startup_waits_beyond_ten_seconds_and_does_not_replay_keys()
  {
    QTest::keyPress(text_, Qt::Key_W); QTest::qWait(10200);
    QCOMPARE(messages("hello").size(), 0);
    status_["arm_allowed"] = true; status_["state"] = "READY"; QVERIFY(start_server());
    QTRY_COMPARE(messages("hello").size(), 1); ready_after_stop(); QTest::qWait(150);
    QCOMPARE(messages("keys").size(), 0);
    QKeyEvent repeat(QEvent::KeyPress, Qt::Key_W, Qt::NoModifier, "w", true, 1);
    QApplication::sendEvent(text_, &repeat); QTest::qWait(100); QCOMPARE(messages("keys").size(), 0);
    QTest::keyRelease(text_, Qt::Key_W); QTest::keyPress(text_, Qt::Key_W);
    QTRY_COMPARE(latest("keys")["keys"].toArray(), QJsonArray({"w"}));
  }
  void startup_invalid_socket_never_retries_data()
  {
    QTest::addColumn<bool>("wrong_permissions");
    QTest::newRow("regular-file-is-not-socket") << false;
    QTest::newRow("socket-permissions-not-0600") << true;
  }
  void startup_invalid_socket_never_retries()
  {
#ifndef Q_OS_UNIX
    QSKIP("Session filesystem socket validation is Unix-only");
#endif
    QFETCH(bool, wrong_permissions); QTest::qWait(100);
    const QString path = directory_.filePath("manual.sock");
    if (wrong_permissions) {
      QVERIFY(start_server());
      QVERIFY(QFile::setPermissions(path, QFileDevice::ReadOwner | QFileDevice::WriteOwner | QFileDevice::ReadGroup));
    } else {
      QFile wrong_type(path); QVERIFY(wrong_type.open(QIODevice::WriteOnly)); wrong_type.close();
    }
    QTRY_VERIFY(panel_->findChild<QLabel *>("teleop_reason")->text().contains(QString::fromUtf8("不会自动重试")));
    if (wrong_permissions) {
      QVERIFY(QFile::setPermissions(path, QFileDevice::ReadOwner | QFileDevice::WriteOwner));
    } else {
      QVERIFY(QFile::remove(path)); QVERIFY(start_server());
    }
    QTest::qWait(1100); panel_->connect_to_server(); QTest::qWait(150);
    QCOMPARE(messages("hello").size(), 0); QCOMPARE(messages("keys").size(), 0);
  }
  void child_focus_changes_and_combined_keys_do_not_stop_control()
  {
    allow_keys(); QTest::keyPress(text_, Qt::Key_W); finish_initialization();
    control_->setFocus(); QCoreApplication::processEvents(); QTest::keyPress(control_, Qt::Key_A);
    QTRY_COMPARE(latest("keys")["keys"].toArray(), QJsonArray({"w", "a"}));
    text_->setFocus(); QCoreApplication::processEvents(); QTest::qWait(120);
    QCOMPARE(messages("disarm").size(), 0);
    QTest::keyRelease(text_, Qt::Key_W); QTRY_COMPARE(latest("keys")["keys"].toArray(), QJsonArray({"a"}));
    QTest::keyRelease(text_, Qt::Key_A); ready_after_stop();
    QVERIFY(latest("keys")["keys"].toArray().isEmpty());
  }
  void final_release_then_immediate_new_press_uses_new_generation()
  {
    allow_keys(); QTest::keyPress(text_, Qt::Key_W); finish_initialization();
    QTest::keyRelease(text_, Qt::Key_W); QTest::keyPress(text_, Qt::Key_D);
    QTRY_COMPARE(latest("keys")["keys"].toArray(), QJsonArray({"d"}));
    QCOMPARE(latest("keys")["arm_generation"].toInt(), 1); QCOMPARE(messages("arm").size(), 0);
  }
  void final_release_pending_ack_abnormal_stop_revokes_immediately_data()
  {
    QTest::addColumn<QString>("stop_event");
    for (const QString event : {QString("focus"), QString("space"), QString("close"),
      QString("hide"), QString("button")})
    {
      QTest::newRow(event.toLatin1().constData()) << event;
    }
  }
  void final_release_pending_ack_abnormal_stop_revokes_immediately()
  {
    QFETCH(QString, stop_event);
    status_["interaction_policy"] = "hybrid_manual";
    allow_keys(); QTest::keyPress(text_, Qt::Key_W); finish_initialization();
    status_timer_->stop(); suppress_status_ = true;  // No release/generation acknowledgement.
    QTest::keyRelease(text_, Qt::Key_W);
    QVERIFY(panel_->findChild<QLabel *>("teleop_state")->text().contains("STOPPING"));
    const int previous = messages("disarm").size();
    if (stop_event == "focus") {
      QEvent leave(QEvent::WindowDeactivate); QApplication::sendEvent(window_, &leave);
    } else if (stop_event == "space") {
      QTest::keyPress(text_, Qt::Key_Space);
    } else if (stop_event == "close") {
      QCloseEvent close; QApplication::sendEvent(window_, &close);
    } else if (stop_event == "hide") {
      panel_->hide();
    } else {
      panel_->findChild<QPushButton *>("teleop_stop")->click();
    }
    // The 500 ms stale-status watchdog must not be the source of this disarm.
    QTRY_VERIFY_WITH_TIMEOUT(messages("disarm").size() > previous, 150);
    QCOMPARE(latest("disarm")["session_id"].toString(), QString("synthetic_teleop"));
    QVERIFY(panel_->findChild<QLabel *>("teleop_keys")->text().endsWith(QString::fromUtf8("无")));
  }
  void opposite_keys_keep_physical_key_set_until_each_release()
  {
    allow_keys(); QTest::keyPress(text_, Qt::Key_W); finish_initialization();
    QTest::keyPress(text_, Qt::Key_S);
    QTRY_COMPARE(latest("keys")["keys"].toArray(), QJsonArray({"w", "s"}));
    QTest::qWait(150); QCOMPARE(messages("disarm").size(), 0);
    QTest::keyRelease(text_, Qt::Key_S);
    QTRY_COMPARE(latest("keys")["keys"].toArray(), QJsonArray({"w"}));
    QCOMPARE(latest("keys")["arm_generation"].toInt(), 0);
    QTest::keyRelease(text_, Qt::Key_W); ready_after_stop();
  }
  void key_pressed_and_released_while_stopping_does_not_latch_stopping()
  {
    allow_keys(); QTest::keyPress(text_, Qt::Key_W); finish_initialization();
    QTest::keyPress(text_, Qt::Key_Space);
    QTest::keyPress(text_, Qt::Key_A); QTest::keyRelease(text_, Qt::Key_A);
    ready_after_stop(); QTest::qWait(100);
    QVERIFY(panel_->findChild<QLabel *>("teleop_keys")->text().endsWith(QString::fromUtf8("无")));
    QTest::keyPress(text_, Qt::Key_D);
    QTRY_COMPARE(latest("keys")["keys"].toArray(), QJsonArray({"d"}));
    QCOMPARE(latest("keys")["arm_generation"].toInt(), 1);
  }
  void window_deactivation_stops_and_reentry_accepts_new_key()
  {
    allow_keys(); QTest::keyPress(text_, Qt::Key_D); finish_initialization();
    QEvent leave(QEvent::WindowDeactivate); QApplication::sendEvent(window_, &leave);
    QTRY_COMPARE(messages("disarm").size(), 1); ready_after_stop();
    QVERIFY(panel_->findChild<QLabel *>("teleop_keys")->text().endsWith(QString::fromUtf8("无")));
    const int count = messages("keys").size(); QTest::keyPress(text_, Qt::Key_W); QTest::qWait(100);
    QCOMPARE(messages("keys").size(), count);
    QEvent enter(QEvent::WindowActivate); QApplication::sendEvent(window_, &enter);
    QTest::keyRelease(text_, Qt::Key_D); QTest::keyPress(text_, Qt::Key_D);
    QTRY_COMPARE(latest("keys")["keys"].toArray(), QJsonArray({"d"}));
    QCOMPARE(messages("arm").size(), 0);
  }
  void hide_and_close_stop()
  {
    allow_keys(); QTest::keyPress(text_, Qt::Key_S); finish_initialization();
    panel_->hide(); QTRY_COMPARE(messages("disarm").size(), 1);
    QVERIFY(panel_->findChild<QLabel *>("teleop_keys")->text().endsWith(QString::fromUtf8("无")));
    window_->close(); QCoreApplication::processEvents();
  }
  void space_stops_and_new_key_resumes_without_replaying_old_key()
  {
    allow_keys(); QTest::keyPress(text_, Qt::Key_W); finish_initialization();
    QTest::keyPress(text_, Qt::Key_Space); QTest::keyPress(text_, Qt::Key_A);
    QTRY_COMPARE(messages("disarm").size(), 1);
    QTRY_COMPARE(latest("keys")["keys"].toArray(), QJsonArray({"a"}));
    QCOMPARE(latest("keys")["arm_generation"].toInt(), 1);
    QTest::keyRelease(text_, Qt::Key_A); ready_after_stop();
    const int count = messages("keys").size();
    QKeyEvent repeat(QEvent::KeyPress, Qt::Key_W, Qt::NoModifier, "w", true, 1);
    QApplication::sendEvent(text_, &repeat); QTest::qWait(150); QCOMPARE(messages("keys").size(), count);
  }
  void disconnected_socket_reconnects_without_replaying_keys()
  {
    allow_keys(); QTest::keyPress(text_, Qt::Key_W); finish_initialization();
    peer_->abort(); status_["state"] = "READY"; status_["arm_generation"] = 1;
    QTRY_COMPARE(messages("hello").size(), 2); ready_after_stop();
    const int count = messages("keys").size(); QTest::qWait(150); QCOMPARE(messages("keys").size(), count);
    QVERIFY(panel_->findChild<QLabel *>("teleop_keys")->text().endsWith(QString::fromUtf8("无")));
    QTest::keyRelease(text_, Qt::Key_W); QTest::keyPress(text_, Qt::Key_W);
    QTRY_COMPARE(latest("keys")["arm_generation"].toInt(), 1);
    QTRY_COMPARE(latest("keys")["keys"].toArray(), QJsonArray({"w"}));
  }
  void stale_backend_status_stops_then_recovers_without_buttons()
  {
    allow_keys(); QTest::keyPress(text_, Qt::Key_W); finish_initialization();
    status_timer_->stop(); status(); QTest::qWait(20);
    QElapsedTimer elapsed; elapsed.start(); QTest::qWait(350);
    QCOMPARE(messages("disarm").size(), 0);
    QTRY_COMPARE(messages("disarm").size(), 1); QVERIFY(elapsed.elapsed() < 1000);
    QTRY_COMPARE(messages("hello").size(), 2); status_timer_->start(); ready_after_stop();
    const int count = messages("keys").size(); QTest::qWait(150); QCOMPARE(messages("keys").size(), count);
    QTest::keyRelease(text_, Qt::Key_W); QTest::keyPress(text_, Qt::Key_S);
    QTRY_COMPARE(latest("keys")["keys"].toArray(), QJsonArray({"s"}));
  }
  void healthy_continuous_hold_keeps_refreshing_until_release()
  {
    allow_keys(); QTest::keyPress(text_, Qt::Key_W); finish_initialization();
    QTest::qWait(1100); QTest::keyPress(text_, Qt::Key_A); QTest::qWait(1200);
    QCOMPARE(messages("disarm").size(), 0);
    QCOMPARE(latest("keys")["keys"].toArray(), QJsonArray({"w", "a"}));
    const int count = messages("keys").size(); QTest::qWait(150); QVERIFY(messages("keys").size() > count);
    QTest::keyRelease(text_, Qt::Key_W); QTRY_COMPARE(latest("keys")["keys"].toArray(), QJsonArray({"a"}));
    QTest::keyRelease(text_, Qt::Key_A); ready_after_stop();
    QVERIFY(latest("keys")["keys"].toArray().isEmpty()); QCOMPARE(messages("disarm").size(), 0);
    QTest::qWait(900); QTest::keyPress(text_, Qt::Key_D);
    QTRY_COMPARE(latest("keys")["keys"].toArray(), QJsonArray({"d"}));
    QCOMPARE(messages("arm").size(), 0);
  }
  void backend_generation_change_clears_previous_held_key()
  {
    allow_keys(); QTest::keyPress(text_, Qt::Key_W); finish_initialization();
    status_["state"] = "READY"; status_["arm_generation"] = 1; status(); ready_after_stop();
    const int count = messages("keys").size(); QTest::qWait(150); QCOMPARE(messages("keys").size(), count);
    QVERIFY(panel_->findChild<QLabel *>("teleop_keys")->text().endsWith(QString::fromUtf8("无")));
    QTest::keyRelease(text_, Qt::Key_W); QTest::keyPress(text_, Qt::Key_W);
    QTRY_COMPARE(latest("keys")["arm_generation"].toInt(), 1);
  }
  void replacing_session_releases_old_keys_without_transferring_intent()
  {
    allow_keys(); QTest::keyPress(text_, Qt::Key_W); finish_initialization();
    delete panel_; panel_ = nullptr;
    QTRY_COMPARE(messages("disarm").size(), 1);
    QCOMPARE(latest("disarm")["session_id"].toString(), QString("synthetic_teleop"));
    status_["session_id"] = "synthetic_next"; status_["state"] = "READY";
    wc_bringup::TeleopSession next;
    next.directory = directory_.path(); next.session_id = "synthetic_next";
    panel_ = new wc_bringup::MappingTeleopPanel(next, window_, window_);
    window_->layout()->addWidget(panel_); panel_->show(); text_->setFocus();
    QTRY_COMPARE(messages("hello").size(), 2); ready_after_stop();
    QCOMPARE(latest("hello")["session_id"].toString(), QString("synthetic_next"));
    const int previous_keys = messages("keys").size();
    QKeyEvent repeat(QEvent::KeyPress, Qt::Key_W, Qt::NoModifier, "w", true, 1);
    QApplication::sendEvent(text_, &repeat); QTest::qWait(150);
    QCOMPARE(messages("keys").size(), previous_keys);
    QTest::keyRelease(text_, Qt::Key_W); QTest::qWait(50);
    QCOMPARE(messages("keys").size(), previous_keys);
    QTest::keyPress(text_, Qt::Key_D);
    QTRY_VERIFY(messages("keys").size() > previous_keys);
    QCOMPARE(latest("keys")["session_id"].toString(), QString("synthetic_next"));
    QCOMPARE(latest("keys")["keys"].toArray(), QJsonArray({"d"}));
    QCOMPARE(messages("disarm").size(), 1);
    QTest::keyRelease(text_, Qt::Key_D);
  }
  void mismatched_status_identity_disconnects_without_retry()
  {
    status_["session_id"] = "different_session"; status();
    QTRY_VERIFY(panel_->findChild<QLabel *>("teleop_state")->text().contains("DISCONNECTED"));
    QTest::qWait(1100); QCOMPARE(messages("hello").size(), 1);
    QTest::keyPress(text_, Qt::Key_W); QTest::qWait(100); QCOMPARE(messages("keys").size(), 0);
  }
  void offline_view_cannot_attach_to_control_socket()
  {
    delete panel_;
    wc_bringup::TeleopSession offline;
    offline.unavailable_reason = QString::fromUtf8("离线地图查看");
    panel_ = new wc_bringup::MappingTeleopPanel(offline, window_, window_);
    window_->layout()->addWidget(panel_); panel_->show();
    QTest::keyClicks(text_, "wasd"); QTest::qWait(600);
    QCOMPARE(text_->text(), QString("wasd"));
    QCOMPARE(messages("hello").size(), 1); QCOMPARE(messages("keys").size(), 0);
    QVERIFY(!wc_bringup::teleop_session_from_arguments({"mapping_rviz", "-d", directory_.filePath("saved.rviz")}).valid());
  }
  void render_diagnostics_require_exact_opt_in()
  {
    using wc_bringup::render_diagnostics::enabled;
    QVERIFY(enabled("1"));
    for (const QByteArray value : {QByteArray(), QByteArray("0"), QByteArray("true"), QByteArray(" 1"), QByteArray("01")}) {
      QVERIFY(!enabled(value));
    }
  }
  void render_diagnostics_exclusive_directory_and_artifacts()
  {
    QTemporaryDir fixture; QVERIFY(fixture.isValid());
    const auto session = diagnostic_session(fixture); QVERIFY(session.valid());
    using wc_bringup::render_diagnostics::OutputDirectory;
    OutputDirectory output(session);
    QCOMPARE(output.path(), fixture.filePath("render_diagnostics"));
#ifdef Q_OS_UNIX
    const auto permissions = QFileInfo(output.path()).permissions();
    QVERIFY(!(permissions & (QFileDevice::ReadGroup | QFileDevice::WriteGroup | QFileDevice::ExeGroup |
      QFileDevice::ReadOther | QFileDevice::WriteOther | QFileDevice::ExeOther)));
#endif
    output.write_json("phase_03.json", {{"synthetic", true}, {"sample", 3}});
    QFile original(output.path()+"/phase_03.json"); QVERIFY(original.open(QIODevice::ReadOnly));
    const QByteArray bytes = original.readAll(); original.close();
    QCOMPARE(QJsonDocument::fromJson(bytes).object().value("sample").toInt(), 3);
    QVERIFY_EXCEPTION_THROWN(output.write_json("phase_03.json", {{"sample", 99}}), std::runtime_error);
    QVERIFY_EXCEPTION_THROWN((void)OutputDirectory(session), std::runtime_error);
    QVERIFY_EXCEPTION_THROWN(output.new_file("../escape.png"), std::runtime_error);
    QVERIFY_EXCEPTION_THROWN(output.new_file("/tmp/escape.png"), std::runtime_error);
    QVERIFY(original.open(QIODevice::ReadOnly)); QCOMPARE(original.readAll(), bytes);
  }
  void render_diagnostics_delay_is_bounded_and_disables_early_readbacks()
  {
    using wc_bringup::render_diagnostics::capture_schedule;
    QCOMPARE(capture_schedule(QByteArray()), QList<int>({3, 15, 45}));
    QCOMPARE(capture_schedule("45"), QList<int>({45}));
    QCOMPARE(capture_schedule("1"), QList<int>({1}));
    QCOMPARE(capture_schedule("90"), QList<int>({90}));
    for (const QByteArray value : {QByteArray("0"), QByteArray("91"), QByteArray("900"),
      QByteArray("-1"), QByteArray("+1"), QByteArray("01"), QByteArray("1.0"),
      QByteArray(" 45"), QByteArray("45 "), QByteArray("45\n"), QByteArray("3,15,45")})
    {
      QVERIFY_EXCEPTION_THROWN(capture_schedule(value), std::runtime_error);
    }
  }
  void render_diagnostics_reject_unverified_and_changed_identity()
  {
    using wc_bringup::render_diagnostics::OutputDirectory;
    QVERIFY_EXCEPTION_THROWN((void)OutputDirectory(wc_bringup::TeleopSession()), std::runtime_error);
    QTemporaryDir fixture; QVERIFY(fixture.isValid());
    auto session = diagnostic_session(fixture); QVERIFY(session.valid());
    auto mismatch = session; mismatch.session_id = "different";
    QVERIFY_EXCEPTION_THROWN((void)OutputDirectory(mismatch), std::runtime_error);
    QVERIFY(!QFileInfo(fixture.filePath("render_diagnostics")).exists());
    OutputDirectory output(session);
    QFile identity(fixture.filePath("session.json")); QVERIFY(identity.open(QIODevice::WriteOnly | QIODevice::Truncate));
    identity.write("{\"session_id\":\"different\"}"); identity.close();
    QVERIFY_EXCEPTION_THROWN(output.write_json("phase_03.json", {}), std::runtime_error);
    QVERIFY(!QFileInfo(output.path()+"/phase_03.json").exists());
  }
  void render_diagnostics_preserve_preexisting_partial()
  {
    QTemporaryDir fixture; QVERIFY(fixture.isValid());
    wc_bringup::render_diagnostics::OutputDirectory output(diagnostic_session(fixture));
    QFile partial(output.path()+"/phase_03.json.partial"); QVERIFY(partial.open(QIODevice::WriteOnly | QIODevice::NewOnly));
    partial.write("SYNTHETIC existing evidence"); partial.close();
    QVERIFY_EXCEPTION_THROWN(output.write_json("phase_03.json", {}), std::runtime_error);
    QVERIFY(partial.open(QIODevice::ReadOnly)); QCOMPARE(partial.readAll(), QByteArray("SYNTHETIC existing evidence"));
    QVERIFY(!QFileInfo(output.path()+"/phase_03.json").exists());
  }
  void render_diagnostics_reject_symlink_directory_and_artifact()
  {
#ifdef Q_OS_UNIX
    using wc_bringup::render_diagnostics::OutputDirectory;
    QTemporaryDir fixture, elsewhere; QVERIFY(fixture.isValid()); QVERIFY(elsewhere.isValid());
    const auto session = diagnostic_session(fixture);
    QVERIFY(QFile::link(elsewhere.path(), fixture.filePath("render_diagnostics")));
    QVERIFY_EXCEPTION_THROWN((void)OutputDirectory(session), std::runtime_error);
    QVERIFY(QFile::remove(fixture.filePath("render_diagnostics")));
    OutputDirectory output(session);
    QVERIFY(QFile::link(elsewhere.filePath("missing.png"), output.path()+"/phase_03_ogre.png"));
    QVERIFY_EXCEPTION_THROWN(output.new_file("phase_03_ogre.png"), std::runtime_error);
#else
    QSKIP("POSIX symlink and ownership validation is tested on the target");
#endif
  }
};

QTEST_MAIN(TeleopTest)
#include "test_mapping_teleop.moc"
