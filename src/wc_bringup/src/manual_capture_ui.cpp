// User-operated capture UI. No ROS drivers, serial access or motion estimation.
#include <csignal>
#include <QApplication>
#include <QCommandLineParser>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QRegularExpression>
#include <QTimer>
#include <QVBoxLayout>
#include "wc_bringup/mapping_teleop.hpp"

namespace {
volatile std::sig_atomic_t stopping = 0;
void request_stop(int) {stopping = 1;}
QJsonObject read_object(const QString & path) {
  QFile f(path);
  if (QFileInfo(path).isSymLink() || !f.open(QIODevice::ReadOnly) || f.size() > 1000000) {return {};}
  QJsonParseError error;
  const auto doc = QJsonDocument::fromJson(f.readAll(), &error);
  return error.error == QJsonParseError::NoError && doc.isObject() ? doc.object() : QJsonObject();
}
}

int main(int argc, char ** argv) {
  QApplication app(argc, argv);
  app.setApplicationName(QStringLiteral("手动采集"));
  QCommandLineParser parser;
  parser.addHelpOption();
  parser.addOption({QStringLiteral("session-root"), QStringLiteral("Existing capture session"), QStringLiteral("path")});
  parser.addOption({QStringLiteral("session-id"), QStringLiteral("Exact capture identity"), QStringLiteral("id")});
  parser.process(app);
  const QString directory = QDir::cleanPath(parser.value(QStringLiteral("session-root")));
  const QString identity = parser.value(QStringLiteral("session-id"));
  const QFileInfo info(directory);
  if (!info.isAbsolute() || !info.isDir() || info.isSymLink() ||
    !QRegularExpression(QStringLiteral("^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")).match(identity).hasMatch()) {return 2;}
  for (QString p = directory; ; ) {
    if (QFileInfo(p).isSymLink()) {return 2;}
    const QString parent = QFileInfo(p).dir().absolutePath();
    if (p == parent) {break;}
    p = parent;
  }
  const auto runtime = read_object(directory + QStringLiteral("/configuration/manual_runtime.json"));
  const auto manifest = read_object(directory + QStringLiteral("/capture_manifest.json"));
  if (runtime.value("session_id").toString() != identity || runtime.value("source_mode").toString() != "real" ||
    runtime.value("status").toString() != "EXPERIMENT" || manifest.value("session_id").toString() != identity ||
    manifest.value("status").toString() != "RECORDING" || !manifest.value("manual_drive").toBool() ||
    !runtime.value("manual_controls").toObject().value("arm_allowed").toBool() ||
    runtime.value("manual_controls").toObject().value("interaction_policy").toString() != "hybrid_manual") {return 2;}
  QWidget window;
  window.setWindowTitle(QStringLiteral("采集与手动驾驶 — ") + identity);
  auto * layout = new QVBoxLayout(&window);
  auto * title = new QLabel(QStringLiteral("正在记录传感器数据。窗口前台可手推或按住 WASD；正常松键停稳后恢复手推。\n关闭此窗口会停止本次采集并完成记录收尾。"));
  title->setWordWrap(true);
  layout->addWidget(title);
  wc_bringup::TeleopSession session;
  session.directory = wc_bringup::checked_manual_socket_directory(
    manifest.value(QStringLiteral("project_root")).toString(), directory, identity,
    manifest.value(QStringLiteral("manual_socket_directory")).toString());
  session.session_id = identity;
  if (!session.valid()) {return 2;}
  auto * panel = new wc_bringup::MappingTeleopPanel(session, &window, &window);
  layout->addWidget(panel);
  window.resize(850, 420);
  std::signal(SIGINT, request_stop); std::signal(SIGTERM, request_stop);
  QTimer signal_timer;
  QObject::connect(&signal_timer, &QTimer::timeout, [&]() {if (stopping) {panel->stop_and_disarm("CAPTURE_STOP"); window.close();}});
  QObject::connect(&app, &QApplication::aboutToQuit, [&]() {panel->stop_and_disarm("CAPTURE_UI_CLOSED");});
  signal_timer.start(50);
  window.show();
  panel->connect_to_server();
  return app.exec();
}
