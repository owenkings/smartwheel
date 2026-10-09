// Qt-only command requests for one identity-bound mapping coordinator.
#pragma once
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPointer>
#include <QPushButton>
#include <QRegularExpression>
#include <QSaveFile>
#include <QTimer>
#include <QUuid>
#include <QWidget>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace wc_bringup { namespace mapping_control {
constexpr qint64 max_bytes = 256 * 1024;
inline void require(bool value, const char * message) {
  if (!value) {throw std::runtime_error(message);}
}
inline qint64 number(const QJsonValue & value) {
  const double n = value.toDouble(-1);
  require(value.isDouble() && std::isfinite(n) && n >= 0 &&
    n <= 9007199254740991. && std::floor(n) == n, "Invalid integer in mapping status");
  return static_cast<qint64>(n);
}
inline void ordinary_ancestors(const QString & path) {
  require(QFileInfo(path).isAbsolute() && QDir::cleanPath(path) == path,
    "Mapping control requires an absolute ordinary path");
  for (QString current = path;;) {
    struct stat info{};
    require(::lstat(QFile::encodeName(current).constData(), &info) == 0 &&
      S_ISDIR(info.st_mode), "Linked or missing mapping directory");
    const auto next = QFileInfo(current).dir().absolutePath();
    if (next == current) {break;}
    current = next;
  }
}
inline QByteArray read_bytes(const QString & path, bool private_file = true) {
  ordinary_ancestors(QFileInfo(path).absolutePath());
  const int fd = ::open(QFile::encodeName(path).constData(), O_RDONLY | O_NOFOLLOW | O_CLOEXEC | O_NONBLOCK);
  require(fd >= 0, "Cannot read mapping identity/status");
  QFile file;
  if (!file.open(fd, QIODevice::ReadOnly, QFileDevice::AutoCloseHandle)) {
    ::close(fd); throw std::runtime_error("Cannot own mapping status descriptor");
  }
  struct stat info{};
  require(::fstat(fd, &info) == 0 && S_ISREG(info.st_mode) &&
    (!private_file || (info.st_uid == ::geteuid() && (info.st_mode & 0777) == 0600)) &&
    info.st_size > 0 && info.st_size <= max_bytes, "Invalid mapping file type, owner, permissions or size");
  const auto bytes = file.read(max_bytes + 1);
  require(bytes.size() <= max_bytes, "Oversized mapping file");
  return bytes;
}
inline QJsonObject read_object(const QString & path, bool private_file = true) {
  const auto bytes = read_bytes(path, private_file);
  QJsonParseError error{};
  const auto doc = QJsonDocument::fromJson(bytes, &error);
  require(bytes.size() <= max_bytes && error.error == QJsonParseError::NoError && doc.isObject(),
    "Malformed mapping status");
  return doc.object();
}
inline QString process_ticks(qint64 pid) {
  const auto path = QStringLiteral("/proc/%1").arg(pid);
  struct stat info{};
  require(pid > 0 && ::stat(QFile::encodeName(path).constData(), &info) == 0 &&
    info.st_uid == ::geteuid(), "Mapping owner absent or wrong UID");
  QFile file(path + "/stat");
  require(file.open(QIODevice::ReadOnly), "Mapping owner stat unavailable");
  const auto bytes = file.read(8192);
  const auto fields = bytes.mid(bytes.lastIndexOf(')') + 2).simplified().split(' ');
  require(fields.size() > 19 && fields[0] != "Z" && fields[0] != "X",
    "Mapping owner is no longer alive");
  return QString::fromLatin1(fields[19]);
}

class Panel final : public QWidget {
public:
  using Bind = std::function<void(const QJsonObject &)>;
  Panel(const QString & directory, QWidget * window, Bind bind,
    std::function<void()> suspend, QWidget * parent = nullptr)
  : QWidget(parent), directory_(directory), window_(window), bind_(std::move(bind)), suspend_(std::move(suspend)) {
    setObjectName("mapping_interactive_controls");
    auto * layout = new QHBoxLayout(this);
    button_ = new QPushButton(QStringLiteral("开始建图"), this);
    button_->setObjectName("mapping_action_button"); button_->setEnabled(false);
    label_ = new QLabel(QStringLiteral("正在连接本窗口建图任务…"), this);
    label_->setObjectName("mapping_action_status"); label_->setWordWrap(true); label_->setTextFormat(Qt::PlainText);
    layout->addWidget(button_); layout->addWidget(label_, 1);
    timer_ = new QTimer(this); timer_->setInterval(100);
    connect(timer_, &QTimer::timeout, this, [this]() {poll();});
    connect(button_, &QPushButton::clicked, this, [this]() {request();});
    try {
      ordinary_ancestors(directory_);
      require(::lstat(QFile::encodeName(directory_).constData(), &directory_stat_) == 0 &&
        directory_stat_.st_uid == ::geteuid() && (directory_stat_.st_mode & 0777) == 0700,
        "Mapping control directory must be owned and private");
      owner_ = read_object(directory_ + "/owner.json");
      require(number(owner_["schema_version"]) == 1, "Unknown mapping owner schema");
      const auto project = owner_["project_root"].toString();
      const auto id = owner_["window_id"].toString();
      require(QRegularExpression("^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$").match(id).hasMatch() &&
        directory_ == project + "/.phase1_runtime/mapping_windows/" + id &&
        owner_["control_directory"].toString() == directory_, "Mapping owner path mismatch");
      ordinary_ancestors(project);
      require(owner_["owner_start_ticks"].isString() &&
        QRegularExpression("^[0-9]+$").match(owner_["owner_start_ticks"].toString()).hasMatch(),
        "Mapping owner start ticks invalid");
      poll(); if (!failed_) {timer_->start();}
    } catch (const std::exception & error) {fail(error.what());}
  }
  void poll() {
    if (failed_) {return;}
    try {
      validate_owner();
      const auto next = read_object(directory_ + "/status.json");
      for (const auto & key : {"schema_version", "project_root", "window_id", "owner_pid", "owner_start_ticks", "control_directory"}) {
        require(next[key] == owner_[key], "Mapping status belongs to another owner/window");
      }
      const auto generation = number(next["generation"]), view = number(next["view_generation"]);
      const auto sequence = number(next["last_sequence"]);
      require(generation >= generation_ && view >= view_generation_, "Mapping generation moved backwards");
      require(QRegularExpression("^[0-9a-f]{32}$").match(next["command_token"].toString()).hasMatch() &&
        next["can_start"].isBool() && next["can_stop"].isBool(), "Invalid mapping command capability");
      const auto state = next["state"].toString();
      require(QStringList{"OPENING", "PREVIEW", "STARTING", "MAPPING", "STOPPING", "SAVING", "CLOSING", "FAILED", "CLOSED"}.contains(state),
        "Unknown mapping lifecycle state");
      require(next["active_session"].isNull() || next["active_session"].isObject(), "Invalid mapping session binding");
      if (generation_ >= 0) {
        require(generation != generation_ || (next["active_session"] == status_["active_session"] &&
          view == view_generation_ && state == status_["state"].toString()), "Mapping binding/state changed within generation");
        require(view != view_generation_ || next["active_session"] == status_["active_session"],
          "Mapping session changed without a new view generation");
        require(sequence >= number(status_["last_sequence"]), "Mapping acknowledgement sequence regressed");
      }
      const auto ack = next["last_command"].toObject();
      if (!pending_nonce_.isEmpty() &&
        ((ack["nonce"].toString() == pending_nonce_ && number(ack["sequence"]) == local_sequence_) ||
        generation != generation_ || next["command_token"] != status_["command_token"] ||
        next["active_session"] != status_["active_session"])) {pending_nonce_.clear();}
      status_ = next; generation_ = generation; view_generation_ = view;
      const auto active = next["active_session"].toObject();
      const bool display_state = state == "PREVIEW" || state == "SAVING" || state == "MAPPING";
      const bool showing = display_state && pending_nonce_.isEmpty();
      require(!display_state || !active.isEmpty(), "Display state has no active session");
      if (!showing || active.isEmpty()) {suspend_once();}
      else if (bound_view_ != view || suspended_) {
        bind_(next); bound_view_ = view; suspended_ = false;
        setProperty("bound_view_generation", static_cast<double>(view));
        setProperty("bound_session_id", active["session_id"].toString());
      }
      label_->setText(next["message"].toString() + QStringLiteral("\n保存状态：") +
        next["save"].toObject()["state"].toString("IDLE"));
      if (!ack.isEmpty() && ack["accepted"].isBool() && !ack["accepted"].toBool()) {
        label_->setText(label_->text() + QStringLiteral("\n操作未接受：") + ack["reason"].toString());
      }
      action_ = state == "PREVIEW" && next["can_start"].toBool() ? "start" :
        ((state == "MAPPING" || state == "STARTING") && next["can_stop"].toBool() ? "stop" : "");
      button_->setText(state == "MAPPING" ? QStringLiteral("停止并保存") :
        state == "STARTING" && next["can_stop"].toBool() ? QStringLiteral("取消开始") : QStringLiteral("开始建图"));
      button_->setEnabled(!action_.isEmpty() && pending_nonce_.isEmpty());
      setProperty("mapping_state", state); setProperty("command_pending", !pending_nonce_.isEmpty());
      if (state == "CLOSED") {timer_->stop(); button_->setEnabled(false); close_owned_window();}
    } catch (const std::exception & error) {fail(error.what());}
  }
private:
  void validate_owner() {
    ordinary_ancestors(directory_);
    struct stat info{};
    require(::lstat(QFile::encodeName(directory_).constData(), &info) == 0 &&
      info.st_dev == directory_stat_.st_dev && info.st_ino == directory_stat_.st_ino &&
      info.st_uid == ::geteuid() && (info.st_mode & 0777) == 0700,
      "Mapping control directory identity changed");
    require(read_object(directory_ + "/owner.json") == owner_, "Mapping owner identity changed");
    require(process_ticks(number(owner_["owner_pid"])) == owner_["owner_start_ticks"].toString(),
      "Mapping owner PID identity changed");
  }
  void suspend_once() {
    if (!suspended_) {suspend_(); suspended_ = true;}
    setProperty("bound_session_id", QString());
  }
  void request() {
    poll();
    if (failed_ || action_.isEmpty() || !pending_nonce_.isEmpty()) {return;}
    try {
      validate_owner();
      require(std::max(local_sequence_, number(status_["last_sequence"])) < 9007199254740991LL,
        "Mapping command sequence exhausted");
      local_sequence_ = std::max(local_sequence_, number(status_["last_sequence"])) + 1;
      pending_nonce_ = QUuid::createUuid().toString(QUuid::Id128);
      QJsonObject command{{"schema_version", 1}, {"window_id", owner_["window_id"]},
        {"owner_pid", owner_["owner_pid"]}, {"owner_start_ticks", owner_["owner_start_ticks"]},
        {"generation", status_["generation"]}, {"active_session_id", status_["active_session"].toObject()["session_id"]},
        {"command_token", status_["command_token"]}, {"sequence", static_cast<double>(local_sequence_)},
        {"nonce", pending_nonce_}, {"action", action_}};
      if (status_["active_session"].isNull()) {command["active_session_id"] = QJsonValue::Null;}
      const QString path = directory_ + "/command.json";
      if (QFileInfo::exists(path) || QFileInfo(path).isSymLink()) {read_object(path);}
      QSaveFile file(path); file.setDirectWriteFallback(false);
      require(file.open(QIODevice::WriteOnly) && file.setPermissions(QFileDevice::ReadOwner | QFileDevice::WriteOwner),
        "Cannot create private atomic mapping command");
      const auto bytes = QJsonDocument(command).toJson(QJsonDocument::Compact);
      require(file.write(bytes) == bytes.size(), "Incomplete mapping command write");
      validate_owner();
      require(file.commit(), "Cannot commit mapping command");
      button_->setEnabled(false); setProperty("command_pending", true);
      label_->setText(QStringLiteral("请求已发送，等待本窗口任务确认…"));
      suspend_once();
    } catch (const std::exception & error) {fail(error.what());}
  }
  void close_owned_window() {
    if (window_) {QTimer::singleShot(0, window_, [window = window_]() {if (window) {window->close();}});}
  }
  void fail(const char * reason) {
    qWarning("Mapping window control refused: %s", reason);
    failed_ = true; timer_->stop(); button_->setEnabled(false);
    label_->setText(QStringLiteral("本窗口任务身份或状态无效，正在关闭：") + QString::fromUtf8(reason));
    setProperty("protocol_failure", QString::fromUtf8(reason));
    try {suspend_once();} catch (...) {}
    close_owned_window();
  }
  QString directory_, action_, pending_nonce_;
  QPointer<QWidget> window_;
  QJsonObject owner_, status_;
  struct stat directory_stat_{};
  qint64 generation_{-1}, view_generation_{-1}, bound_view_{-1}, local_sequence_{0};
  bool failed_{false}, suspended_{false};
  Bind bind_;
  std::function<void()> suspend_;
  QPushButton * button_{};
  QLabel * label_{};
  QTimer * timer_{};
};
}}  // namespace wc_bringup::mapping_control
