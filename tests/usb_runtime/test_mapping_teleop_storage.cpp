// SYNTHETIC metadata parser test; no window, device access or motor commands.
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QTemporaryDir>
#include <unistd.h>
#include "wc_bringup/mapping_teleop.hpp"

int main(int argc, char ** argv)
{
  QCoreApplication application(argc, argv);
  QTemporaryDir project;
  QTemporaryDir archive;
  QTemporaryDir another_archive;
  if (!project.isValid() || !archive.isValid() || !another_archive.isValid()) {return 1;}
  const QString identity = QStringLiteral("SYNTHETIC_usb_session");
  const QString project_key = QString::fromLatin1(QCryptographicHash::hash(project.path().toUtf8(),
      QCryptographicHash::Sha256).toHex().left(12));
  const QString socket_root = QStringLiteral("/tmp/wc-sock-") + QString::number(::geteuid()) +
    QStringLiteral("-") + project_key;
  if (QFileInfo::exists(socket_root) || !QDir().mkdir(socket_root)) {return 2;}
  struct OwnedRoot {
    QString path;
    ~OwnedRoot() {QDir().rmdir(path);}
  } cleanup{socket_root};
  const auto private_mode = QFileDevice::ReadOwner | QFileDevice::WriteOwner | QFileDevice::ExeOwner;
  if (!QFile::setPermissions(socket_root, private_mode)) {return 3;}
  QByteArray input = archive.path().toUtf8(); input.append('\0'); input.append(identity.toUtf8());
  const QString key = QString::fromLatin1(QCryptographicHash::hash(input,
      QCryptographicHash::Sha256).toHex().left(24));
  const QString expected = socket_root + QStringLiteral("/") + key;
  const auto write = [](const QString & path, const QByteArray & bytes) {
    QFile file(path);
    return file.open(QIODevice::WriteOnly | QIODevice::Truncate) && file.write(bytes) == bytes.size();
  };
  QJsonObject session{{"session_id", identity}, {"project_root", project.path()},
    {"manual_socket_directory", expected}};
  QJsonObject runtime{{"session_id", identity}, {"source_mode", "real"}, {"status", "EXPERIMENT"}};
  if (!write(archive.filePath("view.rviz"), "# SYNTHETIC\n") ||
    !write(archive.filePath("runtime_config.json"), QJsonDocument(runtime).toJson()) ||
    !write(archive.filePath("session.json"), QJsonDocument(session).toJson())) {return 4;}
  const auto parse = [&]() {
    return wc_bringup::teleop_session_from_arguments({"mapping_rviz", "-d", archive.filePath("view.rviz")});
  };
  auto result = parse();
  if (!result.valid() || result.directory != expected || result.session_id != identity) {return 5;}
  // Reusing the ID in a different data directory must not attach to this socket.
  if (!wc_bringup::checked_manual_socket_directory(project.path(), another_archive.path(), identity, expected).isEmpty()) {return 6;}
  if (!QFile::setPermissions(socket_root, private_mode | QFileDevice::ReadOther) || parse().valid()) {return 7;}
  if (!QFile::setPermissions(socket_root, private_mode)) {return 8;}
  session["manual_socket_directory"] = project.filePath("other_socket");
  if (!write(archive.filePath("session.json"), QJsonDocument(session).toJson()) || parse().valid()) {return 9;}
  // Missing field retains read compatibility with the older adjacent-socket format.
  session.remove("manual_socket_directory");
  if (!write(archive.filePath("session.json"), QJsonDocument(session).toJson())) {return 10;}
  result = parse();
  if (!result.valid() || result.directory != archive.path()) {return 11;}
  return 0;
}
