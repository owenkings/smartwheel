// SPDX-License-Identifier: Apache-2.0
// Optional diagnostic evidence storage; no ROS, rendering, or control actions.
#ifndef WC_BRINGUP__MAPPING_RENDER_DIAGNOSTICS_HPP_
#define WC_BRINGUP__MAPPING_RENDER_DIAGNOSTICS_HPP_

#include <stdexcept>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QList>
#include <QRegularExpression>
#include "wc_bringup/mapping_teleop.hpp"

namespace wc_bringup
{
namespace render_diagnostics
{
inline bool enabled(const QByteArray & value) {return value == "1";}

// A nonempty delay requests exactly one diagnostic capture. No early Ogre
// readback then occurs before the external observer can establish a black frame.
inline QList<int> capture_schedule(const QByteArray & delay)
{
  if (delay.isEmpty()) {return {3, 15, 45};}
  const QString value = QString::fromLatin1(delay);
  if (!QRegularExpression(QStringLiteral("\\A[1-9][0-9]?\\z")).match(value).hasMatch()) {
    throw std::runtime_error("Render diagnostic delay must be an integer from 1 through 90 seconds");
  }
  const int seconds = value.toInt();
  if (seconds > 90) {
    throw std::runtime_error("Render diagnostic delay exceeds its 90 second bound");
  }
  return {seconds};
}

class OutputDirectory
{
public:
  explicit OutputDirectory(const TeleopSession & session) : session_(session)
  {
    check_session();
    path_ = QDir(session_.directory).filePath(QStringLiteral("render_diagnostics"));
    const QFileInfo info(path_);
    if (info.exists() || info.isSymLink() || !QDir().mkdir(path_)) {
      throw std::runtime_error("Render diagnostics require a new session/render_diagnostics directory");
    }
    if (!QFile::setPermissions(path_, QFileDevice::ReadOwner | QFileDevice::WriteOwner | QFileDevice::ExeOwner)) {
      throw std::runtime_error("Cannot make render diagnostic directory private");
    }
  }

  const QString & path() const {return path_;}

  QString new_file(const QString & name) const
  {
    check_session();
    const QFileInfo directory(path_);
    const QRegularExpression basename(QStringLiteral("^[a-z0-9_]+\\.(json|png)$"));
    if (!directory.isDir() || directory.isSymLink() ||
      directory.canonicalFilePath() != directory.absoluteFilePath() ||
      !basename.match(name).hasMatch())
    {
      throw std::runtime_error("Render diagnostic directory or basename is invalid");
    }
    const QString result = QDir(path_).filePath(name);
    if (QFileInfo(result).exists() || QFileInfo(result).isSymLink()) {
      throw std::runtime_error("Refusing an existing render diagnostic artifact");
    }
    return result;
  }

  void write_json(const QString & name, const QJsonObject & object) const
  {
    const QString destination = new_file(name);
    QFile file(destination + QStringLiteral(".partial"));
    const QByteArray bytes = QJsonDocument(object).toJson(QJsonDocument::Indented);
    if (!file.open(QIODevice::WriteOnly | QIODevice::NewOnly) ||
      file.write(bytes) != bytes.size() || !file.flush())
    {
      throw std::runtime_error("Cannot write exclusive render diagnostic JSON");
    }
    file.close();
    // QFile::rename refuses an existing destination. No replacement is allowed.
    if (!file.rename(destination)) {
      throw std::runtime_error("Cannot finalize exclusive render diagnostic JSON");
    }
  }

private:
  void check_session() const
  {
    if (!session_.valid()) {throw std::runtime_error("Render diagnostics require a verified live session");}
    const auto verified = teleop_session_from_arguments({QStringLiteral("mapping_rviz"),
      QStringLiteral("-d"), QDir(session_.directory).filePath(QStringLiteral("view.rviz"))});
    if (!verified.valid() || verified.directory != session_.directory ||
      verified.session_id != session_.session_id)
    {
      throw std::runtime_error("Render diagnostic session identity or path changed");
    }
  }

  TeleopSession session_;
  QString path_;
};
}  // namespace render_diagnostics
}  // namespace wc_bringup
#endif  // WC_BRINGUP__MAPPING_RENDER_DIAGNOSTICS_HPP_
