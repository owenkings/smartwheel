// SYNTHETIC metadata parser test; no window, device access or motor commands.
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QTemporaryDir>
#include "wc_bringup/mapping_teleop.hpp"

int main(int argc, char ** argv)
{
  QCoreApplication application(argc, argv);
  QTemporaryDir project;
  QTemporaryDir archive;
  if (!project.isValid() || !archive.isValid()) {return 1;}
  const QString identity = QStringLiteral("SYNTHETIC_usb_session");
  const QString key = QString::fromLatin1(QCryptographicHash::hash(identity.toLatin1(),
      QCryptographicHash::Sha256).toHex().left(24));
  const QString expected = project.path() + QStringLiteral("/.phase1_runtime/control/") + key;
  const auto write = [](const QString & path, const QByteArray & bytes) {
    QFile file(path);
    return file.open(QIODevice::WriteOnly | QIODevice::Truncate) && file.write(bytes) == bytes.size();
  };
  QJsonObject session{{"session_id", identity}, {"project_root", project.path()},
    {"manual_socket_directory", expected}};
  QJsonObject runtime{{"session_id", identity}, {"source_mode", "real"}, {"status", "EXPERIMENT"}};
  if (!write(archive.filePath("view.rviz"), "# SYNTHETIC\n") ||
    !write(archive.filePath("runtime_config.json"), QJsonDocument(runtime).toJson()) ||
    !write(archive.filePath("session.json"), QJsonDocument(session).toJson())) {return 2;}
  const auto parse = [&]() {
    return wc_bringup::teleop_session_from_arguments({"mapping_rviz", "-d", archive.filePath("view.rviz")});
  };
  auto result = parse();
  if (!result.valid() || result.directory != expected || result.session_id != identity) {return 3;}
  session["manual_socket_directory"] = project.filePath("other_socket");
  if (!write(archive.filePath("session.json"), QJsonDocument(session).toJson()) || parse().valid()) {return 4;}
  session.remove("manual_socket_directory");
  if (!write(archive.filePath("session.json"), QJsonDocument(session).toJson())) {return 5;}
  result = parse();
  if (!result.valid() || result.directory != archive.path()) {return 6;}
  return 0;
}
