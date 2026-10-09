// Coordinator files and button lifecycle only: no device, ROS or vehicle API.
#include <QTest>
#include <QTemporaryDir>
#include "wc_bringup/mapping_interactive_control.hpp"
namespace mc = wc_bringup::mapping_control;

struct Fixture {
  QTemporaryDir temp;
  QString control;
  QJsonObject owner, status;
  Fixture() {
    control = temp.path() + "/.phase1_runtime/mapping_windows/test_window";
    if (!QDir().mkpath(control)) {throw std::runtime_error("fixture directory");}
    ::chmod(QFile::encodeName(control).constData(), 0700);
    owner = {{"schema_version", 1}, {"project_root", temp.path()}, {"window_id", "test_window"},
      {"owner_pid", static_cast<double>(::getpid())}, {"owner_start_ticks", mc::process_ticks(::getpid())},
      {"control_directory", control}};
    status = owner;
    status["generation"] = 1; status["view_generation"] = 1;
    status["state"] = "PREVIEW"; status["message"] = "current frame";
    status["can_start"] = true; status["can_stop"] = false;
    status["command_token"] = QString(32, 'a'); status["last_sequence"] = 0;
    status["last_command"] = QJsonValue::Null;
    status["active_session"] = QJsonObject{{"session_id", "preview_1"}, {"mapping_enabled", false}};
    status["save"] = QJsonObject{{"state", "IDLE"}};
    write("owner.json", owner); publish();
  }
  void write(const QString & name, const QJsonObject & value) {
    QSaveFile file(control + '/' + name);
    if (!file.open(QIODevice::WriteOnly) || !file.setPermissions(QFile::ReadOwner | QFile::WriteOwner)) {
      throw std::runtime_error("fixture file");
    }
    file.write(QJsonDocument(value).toJson());
    if (!file.commit()) {throw std::runtime_error("fixture commit");}
  }
  void publish() {write("status.json", status);}
};

class InteractiveControlTest : public QObject {
  Q_OBJECT
private slots:
  void pending_request_never_replays_and_cancel_has_null_session() {
    Fixture f; QWidget window; window.show(); int binds = 0, clears = 0;
    mc::Panel panel(f.control, &window, [&](const QJsonObject &) {++binds;}, [&]() {++clears;});
    auto * button = panel.findChild<QPushButton *>("mapping_action_button");
    QCOMPARE(binds, 1); QVERIFY(button->isEnabled());
    button->click(); const auto start = mc::read_object(f.control + "/command.json");
    QCOMPARE(start["action"].toString(), QString("start"));
    QCOMPARE(start["active_session_id"].toString(), QString("preview_1"));
    QCOMPARE(start["sequence"].toInt(), 1); QVERIFY(!button->isEnabled());
    for (int i = 0; i < 4; ++i) {panel.poll(); button->click();}
    QCOMPARE(mc::read_object(f.control + "/command.json"), start);
    QCOMPARE(binds, 1); QCOMPARE(clears, 1);
    f.status["generation"] = 2; f.status["view_generation"] = 3;
    f.status["state"] = "STARTING"; f.status["active_session"] = QJsonValue::Null;
    f.status["can_start"] = false; f.status["can_stop"] = true;
    f.status["command_token"] = QString(32, 'b'); f.status["last_sequence"] = 1;
    f.status["last_command"] = QJsonObject{{"nonce", start["nonce"]}, {"sequence", 1}, {"accepted", true}};
    f.publish(); panel.poll();
    QVERIFY(button->isEnabled()); QCOMPARE(button->text(), QStringLiteral("取消开始"));
    button->click(); const auto stop = mc::read_object(f.control + "/command.json");
    QCOMPARE(stop["action"].toString(), QString("stop")); QVERIFY(stop["active_session_id"].isNull());
    QCOMPARE(stop["sequence"].toInt(), 2); QVERIFY(stop["nonce"] != start["nonce"]);
    struct stat info{}; QVERIFY(::stat(QFile::encodeName(f.control + "/command.json").constData(), &info) == 0);
    QCOMPARE(static_cast<int>(info.st_mode & 0777), 0600);
  }
  void repeated_generations_clear_old_sessions_and_allow_save_preview() {
    Fixture f; QWidget window; QStringList bindings; int clears = 0;
    mc::Panel panel(f.control, &window, [&](const QJsonObject & s) {
      bindings << s["active_session"].toObject()["session_id"].toString();
    }, [&]() {++clears;});
    int generation = 1, view = 1;
    for (int n = 1; n <= 3; ++n) {
      for (const auto & phase : {QString("STARTING"), QString("MAPPING"), QString("STOPPING"), QString("SAVING"), QString("PREVIEW")}) {
        f.status["generation"] = ++generation; f.status["state"] = phase;
        f.status["can_start"] = phase == "PREVIEW"; f.status["can_stop"] = phase == "MAPPING" || phase == "STARTING";
        if (phase == "MAPPING" || phase == "SAVING") {
          f.status["view_generation"] = ++view;
          f.status["active_session"] = QJsonObject{{"session_id", phase + QString::number(n)}, {"mapping_enabled", phase == "MAPPING"}};
        }
        f.status["save"] = QJsonObject{{"state", phase == "SAVING" ? "RUNNING" : "CANCELLED"}};
        f.publish(); panel.poll();
        QVERIFY2(panel.property("protocol_failure").toString().isEmpty(), qPrintable(panel.property("protocol_failure").toString()));
      }
    }
    QCOMPARE(bindings.size(), 7); QCOMPARE(clears, 6);
    QCOMPARE(bindings.last(), QString("SAVING3"));
    QVERIFY(panel.findChild<QPushButton *>("mapping_action_button")->isEnabled());
  }
  void same_generation_or_view_generation_cannot_change_binding() {
    for (bool bump_generation : {false, true}) {
      Fixture f; QWidget window;
      mc::Panel panel(f.control, &window, [](const QJsonObject &) {}, []() {});
      if (bump_generation) {f.status["generation"] = 2;}
      f.status["active_session"] = QJsonObject{{"session_id", "unexpected_other_session"}};
      f.publish(); panel.poll(); QVERIFY(!panel.property("protocol_failure").toString().isEmpty());
      QVERIFY(!panel.findChild<QPushButton *>("mapping_action_button")->isEnabled());
      QVERIFY(!QFileInfo::exists(f.control + "/command.json"));
    }
  }
  void owner_identity_change_disables_and_closes_only_owned_window() {
    Fixture f; QWidget window, other; window.show(); other.show();
    mc::Panel panel(f.control, &window, [](const QJsonObject &) {}, []() {});
    f.owner["owner_start_ticks"] = "0"; f.write("owner.json", f.owner); panel.poll();
    QTRY_VERIFY(!window.isVisible()); QVERIFY(other.isVisible());
    QVERIFY(!panel.findChild<QPushButton *>("mapping_action_button")->isEnabled());
  }
  void rejects_symlink_oversized_json_and_public_permissions() {
    for (int mode = 0; mode < 3; ++mode) {
      Fixture f; QWidget window;
      if (mode == 0) {
        QVERIFY(QFile::rename(f.control + "/status.json", f.control + "/saved.json"));
        QVERIFY(::symlink("saved.json", QFile::encodeName(f.control + "/status.json").constData()) == 0);
      } else if (mode == 1) {
        QFile file(f.control + "/status.json"); QVERIFY(file.open(QIODevice::WriteOnly));
        file.write(QByteArray(mc::max_bytes + 1, ' ')); file.close();
      } else {QVERIFY(::chmod(QFile::encodeName(f.control + "/status.json").constData(), 0644) == 0);}
      mc::Panel panel(f.control, &window, [](const QJsonObject &) {}, []() {});
      QVERIFY(!panel.property("protocol_failure").toString().isEmpty());
      QVERIFY(!panel.findChild<QPushButton *>("mapping_action_button")->isEnabled());
    }
  }
};
QTEST_MAIN(InteractiveControlTest)
#include "test_mapping_interactive_control.moc"
