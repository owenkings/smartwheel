// SPDX-License-Identifier: Apache-2.0
// Qt-only UI transport. Device access, motor commands and limits belong solely
// to the session backend. No global keyboard hook, device API or ROS publisher.
#include "wc_bringup/mapping_teleop.hpp"

#include <cerrno>
#include <cmath>
#include <QApplication>
#include <QDir>
#include <QCryptographicHash>
#include <QEvent>
#include <QFile>
#include <QFileInfo>
#include <QFrame>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QKeyEvent>
#include <QLabel>
#include <QLocalSocket>
#include <QPushButton>
#include <QRegularExpression>
#include <QTimer>
#include <QVBoxLayout>

#ifdef Q_OS_UNIX
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace wc_bringup
{
namespace
{
constexpr int kLineBytes = 4096;
constexpr int kStatusTimeoutMs = 500;
constexpr int kReconnectRetryMs = 500;

bool checked_integer(const QJsonValue & value, qint64 & output)
{
  const double number = value.toDouble(-1.);
  if (!value.isDouble() || !std::isfinite(number) || number < 0. ||
    number > 9007199254740991. || std::floor(number) != number)
  {
    return false;
  }
  output = static_cast<qint64>(number);
  return true;
}

QJsonObject read_object(const QString & path)
{
  QFile file(path);
  if (QFileInfo(path).isSymLink() || !file.open(QIODevice::ReadOnly) || file.size() > 1000000) {
    return {};
  }
  const QByteArray bytes = file.read(1000001);
  QJsonParseError error;
  const auto document = QJsonDocument::fromJson(bytes, &error);
  return bytes.size() <= 1000000 && error.error == QJsonParseError::NoError && document.isObject() ?
         document.object() : QJsonObject();
}

QString key_name(int key)
{
  switch (key) {
    case Qt::Key_W: return QStringLiteral("w");
    case Qt::Key_A: return QStringLiteral("a");
    case Qt::Key_S: return QStringLiteral("s");
    case Qt::Key_D: return QStringLiteral("d");
    default: return QString();
  }
}
}  // namespace

TeleopSession teleop_session_from_arguments(const QStringList & arguments)
{
  TeleopSession result;
  QString view;
  for (int i = 1; i < arguments.size(); ++i) {
    if ((arguments[i] == QStringLiteral("-d") || arguments[i] == QStringLiteral("--display-config")) &&
      i + 1 < arguments.size())
    {
      view = arguments[++i];
    } else if (arguments[i].startsWith(QStringLiteral("--display-config="))) {
      view = arguments[i].mid(QStringLiteral("--display-config=").size());
    }
  }
  const QFileInfo info(view);
  if (view.isEmpty() || !info.isAbsolute() || !info.isFile() || info.isSymLink() ||
    info.fileName() != QStringLiteral("view.rviz"))
  {
    result.unavailable_reason = QStringLiteral("当前视图不是可操控的建图会话；地图查看仍可使用。");
    return result;
  }
  QString parent = info.absolutePath();
  while (!parent.isEmpty()) {
    const QFileInfo current(parent);
    if (current.isSymLink()) {
      result.unavailable_reason = QStringLiteral("会话路径经过符号链接，手动控制不可用。");
      return result;
    }
    const QString next = current.dir().absolutePath();
    if (next == parent) {break;}
    parent = next;
  }
  const auto runtime = read_object(info.dir().filePath(QStringLiteral("runtime_config.json")));
  const auto session = read_object(info.dir().filePath(QStringLiteral("session.json")));
  const QString identity = runtime.value(QStringLiteral("session_id")).toString();
  const QRegularExpression valid_identity(QStringLiteral("^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$"));
  if (!valid_identity.match(identity).hasMatch() ||
    session.value(QStringLiteral("session_id")).toString() != identity ||
    runtime.value(QStringLiteral("source_mode")).toString() != QStringLiteral("real") ||
    runtime.value(QStringLiteral("status")).toString() != QStringLiteral("EXPERIMENT"))
  {
    result.unavailable_reason = QStringLiteral("会话身份或配置缺失，手动控制不可用。");
    return result;
  }
  result.directory = info.absolutePath();
  const QString socket_directory = session.value(QStringLiteral("manual_socket_directory")).toString();
  if (!socket_directory.isEmpty() && socket_directory != result.directory) {
    // Only this session's deterministic internal control directory may replace
    // the legacy adjacent socket. Never accept an arbitrary socket from data.
    const QString project = session.value(QStringLiteral("project_root")).toString();
    const QString key = QString::fromLatin1(QCryptographicHash::hash(identity.toLatin1(),
      QCryptographicHash::Sha256).toHex().left(24));
    const QString expected = QDir(project).filePath(QStringLiteral(".phase1_runtime/control/") + key);
    if (!QFileInfo(project).isAbsolute() || QDir::cleanPath(project) != project ||
      socket_directory != expected)
    {
      result.directory.clear();
      result.unavailable_reason = QStringLiteral("Invalid local session control path");
      return result;
    }
    for (QString parent = socket_directory; ; ) {
      const QFileInfo current(parent);
      if (current.isSymLink()) {
        result.directory.clear();
        result.unavailable_reason = QStringLiteral("Linked local session control path");
        return result;
      }
      const QString next = current.dir().absolutePath();
      if (next == parent) {break;}
      parent = next;
    }
    result.directory = socket_directory;
  }
  result.session_id = identity;
  return result;
}

MappingTeleopPanel::MappingTeleopPanel(const TeleopSession & session, QWidget * lifecycle_window,
  QWidget * parent, Confirmation /* confirmation */)
: QWidget(parent), session_(session), lifecycle_window_(lifecycle_window)
{
  setObjectName(QStringLiteral("mapping_teleop_panel"));
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 5, 8, 5);
  auto * buttons = new QHBoxLayout();
  connection_label_ = new QLabel(this);
  connection_label_->setObjectName(QStringLiteral("teleop_connection"));
  stop_button_ = new QPushButton(QStringLiteral("停车（空格）"), this);
  stop_button_->setObjectName(QStringLiteral("teleop_stop"));
  push_button_ = new QPushButton(QStringLiteral("进入手推"), this);
  push_button_->setObjectName(QStringLiteral("teleop_push_mode"));
  buttons->addWidget(connection_label_); buttons->addStretch();
  buttons->addWidget(push_button_); buttons->addWidget(stop_button_);
  layout->addLayout(buttons);
  state_label_ = new QLabel(this); state_label_->setObjectName(QStringLiteral("teleop_state"));
  reason_label_ = new QLabel(this); reason_label_->setObjectName(QStringLiteral("teleop_reason"));
  reason_label_->setWordWrap(true); reason_label_->setTextFormat(Qt::PlainText);
  limits_label_ = new QLabel(QStringLiteral("限速：等待后端配置；运动与物理断线停车尚未验收"), this);
  limits_label_->setObjectName(QStringLiteral("teleop_limits")); limits_label_->setWordWrap(true);
  layout->addWidget(state_label_); layout->addWidget(reason_label_); layout->addWidget(limits_label_);
  auto * control = new QFrame(this);
  control_ = control; control_->setObjectName(QStringLiteral("teleop_control_area"));
  control->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
  control_->setFocusPolicy(Qt::StrongFocus); control_->setMinimumHeight(70);
  auto * area = new QVBoxLayout(control_);
  control_guide_ = new QLabel(control_);
  control_guide_->setObjectName(QStringLiteral("teleop_control_guide"));
  control_guide_->setWordWrap(true); control_guide_->setTextFormat(Qt::PlainText);
  control_guide_->setAttribute(Qt::WA_TransparentForMouseEvents);
  keys_label_ = new QLabel(QStringLiteral("当前按键：无"), control_);
  keys_label_->setObjectName(QStringLiteral("teleop_keys"));
  keys_label_->setAttribute(Qt::WA_TransparentForMouseEvents);
  area->addWidget(control_guide_); area->addWidget(keys_label_); layout->addWidget(control_);
  socket_ = new QLocalSocket(this); socket_->setReadBufferSize(kLineBytes * 2);
  timer_ = new QTimer(this); timer_->setInterval(50); timer_->setTimerType(Qt::PreciseTimer);
  connect(stop_button_, &QPushButton::clicked, this, [this]() {stop_and_disarm(QStringLiteral("用户停车"));});
  connect(socket_, &QLocalSocket::connected, this, [this]() {
      connection_age_.restart();
      state_ = QStringLiteral("WAITING_STATUS");
      reason_ = QStringLiteral("已连接，等待本会话状态。");
      send({{QStringLiteral("type"), QStringLiteral("hello")}, {QStringLiteral("session_id"), session_.session_id}});
      render();
    });
  connect(socket_, &QLocalSocket::readyRead, this, [this]() {receive_bytes();});
  connect(socket_, &QLocalSocket::disconnected, this, [this]() {
      if (!disconnecting_) {transport_failed(QStringLiteral("控制连接已断开；已清空按键，正在自动重连。"));}
    });
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
  connect(socket_, &QLocalSocket::errorOccurred, this, [this](QLocalSocket::LocalSocketError) {
#else
  connect(socket_, QOverload<QLocalSocket::LocalSocketError>::of(&QLocalSocket::error), this,
    [this](QLocalSocket::LocalSocketError) {
#endif
      if (!disconnecting_) {transport_failed(QStringLiteral("控制服务不可用：") + socket_->errorString());}
    });
  connect(timer_, &QTimer::timeout, this, [this]() {tick();});
  connect(push_button_, &QPushButton::clicked, this, [this]() {
      if (!have_status_ || !arm_allowed_ || waiting_for_stop_ || !status_age_.isValid() ||
        status_age_.elapsed() >= kStatusTimeoutMs || state_ == QStringLiteral("FAULT")) {return;}
      keys_.clear(); sent_keys_ = intent_started_ = false;
      const bool enabled = !push_mode_ && !push_requested_;
      if (send({{QStringLiteral("type"), QStringLiteral("push_mode")},
        {QStringLiteral("session_id"), session_.session_id},
        {QStringLiteral("sequence"), static_cast<double>(++sequence_)},
        {QStringLiteral("arm_generation"), static_cast<double>(generation_)},
        {QStringLiteral("enabled"), enabled}}))
      {
        waiting_for_stop_ = true; stop_revocation_sent_ = false; push_requested_ = enabled;
        reason_ = enabled ? QStringLiteral("已请求手推：等待零速度反馈后释放；不能据此确认坡道或载人安全。") :
          QStringLiteral("已请求退出手推；等待状态确认，新的 WASD 才会初始化驱动。");
        render();
      }
    });
  connect(qApp, &QCoreApplication::aboutToQuit, this, [this]() {stop_and_disarm(QStringLiteral("RViz 正在关闭"));});
  // Match the old RViz panel: capture application events, including child
  // widgets. This does not install an OS-wide keyboard hook.
  qApp->installEventFilter(this);
  reason_ = session_.valid() ? QStringLiteral("等待本会话控制服务；连接就绪后直接按 WASD。") : session_.unavailable_reason;
  render(); timer_->start();
  // Connecting only receives status. A new physical key press is the intent
  // that initializes the backend; old keys are never replayed after reconnect.
  if (session_.valid()) {
    retry_enabled_ = true;
    QTimer::singleShot(0, this, [this]() {connect_to_server();});
  }
}

MappingTeleopPanel::~MappingTeleopPanel()
{
  timer_->stop();
  qApp->removeEventFilter(this);
  stop_and_disarm(QStringLiteral("控制面板已关闭"));
  disconnecting_ = true;
  socket_->flush(); socket_->abort();
}

bool MappingTeleopPanel::active_intent() const
{
  return push_requested_ || intent_started_ || !keys_.isEmpty() || state_ == QStringLiteral("INITIALIZING") ||
         state_ == QStringLiteral("ARMED");
}

bool MappingTeleopPanel::application_active() const
{
  auto * active = QApplication::activeWindow();
  return !application_inactive_ && isVisible() && lifecycle_window_ != nullptr && active != nullptr &&
         (active == lifecycle_window_ || lifecycle_window_->isAncestorOf(active));
}

bool MappingTeleopPanel::keys_allowed() const
{
  return have_status_ && arm_allowed_ && (hybrid_manual_ || (!push_mode_ && !push_requested_)) && application_active() && status_age_.isValid() &&
         status_age_.elapsed() < kStatusTimeoutMs &&
         (state_ == QStringLiteral("READY") || state_ == QStringLiteral("INITIALIZING") ||
         state_ == QStringLiteral("ARMED") || waiting_for_stop_);
}

void MappingTeleopPanel::connect_to_server()
{
  if (!session_.valid() || !retry_enabled_ || socket_->state() != QLocalSocket::UnconnectedState) {return;}
  retry_age_.restart();
  keys_.clear(); sent_keys_ = intent_started_ = waiting_for_stop_ = arm_allowed_ = have_status_ = false;
  stop_revocation_sent_ = false;
  generation_ = sequence_ = 0; incoming_.clear(); status_age_.invalidate();
  const QString path = QDir(session_.directory).filePath(QStringLiteral("manual.sock"));
#ifdef Q_OS_UNIX
  struct stat info;
  const QByteArray encoded = QFile::encodeName(path);
  if (::lstat(encoded.constData(), &info) != 0) {
    const bool absent = errno == ENOENT;
    if (!absent) {retry_enabled_ = false;}
    state_ = QStringLiteral("DISCONNECTED");
    reason_ = absent ?
      QStringLiteral("等待本会话控制服务；服务就绪后自动连接，地图不受影响。") :
      QStringLiteral("本会话 manual.sock 不可访问；不会自动重试，请检查会话后重新打开视图。");
    render(); return;
  }
  if (!S_ISSOCK(info.st_mode) || info.st_uid != ::geteuid() || (info.st_mode & 0777) != 0600) {
    retry_enabled_ = false;
    state_ = QStringLiteral("DISCONNECTED");
    reason_ = QStringLiteral("本会话 manual.sock 类型、用户或 0600 权限不匹配；不会自动重试。");
    render(); return;
  }
#endif
  state_ = QStringLiteral("CONNECTING"); reason_ = QStringLiteral("正在自动连接本会话控制服务。");
  connection_age_.restart(); socket_->connectToServer(path); render();
}

bool MappingTeleopPanel::send(const QJsonObject & value)
{
  if (socket_->state() != QLocalSocket::ConnectedState) {return false;}
  const QByteArray bytes = QJsonDocument(value).toJson(QJsonDocument::Compact) + '\n';
  if (bytes.size() > kLineBytes || socket_->bytesToWrite() > kLineBytes || socket_->write(bytes) != bytes.size()) {
    transport_failed(QStringLiteral("控制消息写入失败或积压；已断开并清空按键。")); return false;
  }
  socket_->flush();  // Qt documents this as nonblocking; never wait in the GUI.
  return true;
}

bool MappingTeleopPanel::send_keys()
{
  if (hybrid_manual_) {
    if (!have_status_ || !arm_allowed_ || waiting_for_stop_ || !status_age_.isValid() ||
      status_age_.elapsed() >= kStatusTimeoutMs || state_ == QStringLiteral("FAULT") ||
      socket_->state() != QLocalSocket::ConnectedState) {return false;}
    if (!application_active()) {keys_.clear(); intent_started_ = false;}
  } else if (!keys_allowed() || waiting_for_stop_ || (keys_.isEmpty() && !sent_keys_)) {return false;}
  if (socket_->bytesToWrite() > 0) {
    transport_failed(QStringLiteral("控制消息未及时写出；按键意图已取消。")); return false;
  }
  QJsonArray pressed;
  for (int key : {Qt::Key_W, Qt::Key_A, Qt::Key_S, Qt::Key_D}) {
    if (keys_.contains(key)) {pressed.append(key_name(key));}
  }
  const bool sent = send({{QStringLiteral("type"), QStringLiteral("keys")}, {QStringLiteral("session_id"), session_.session_id},
    {QStringLiteral("sequence"), static_cast<double>(++sequence_)},
    {QStringLiteral("arm_generation"), static_cast<double>(generation_)}, {QStringLiteral("keys"), pressed},
    {QStringLiteral("foreground"), application_active()}});
  if (sent) {sent_keys_ = !keys_.isEmpty();}
  return sent;
}

void MappingTeleopPanel::stop_and_disarm(const QString & reason)
{
  const bool had_intent = active_intent() || waiting_for_stop_ ||
    (hybrid_manual_ && have_status_ && arm_allowed_);
  keys_.clear(); sent_keys_ = intent_started_ = false;
  // Waiting for a normal final-release acknowledgement does not revoke its
  // pending automatic hand-push transition. An abnormal stop must supersede
  // that intent immediately, without waiting for a generation/status reply.
  if (waiting_for_stop_ && stop_revocation_sent_) {render(); return;}
  if (had_intent && socket_->state() == QLocalSocket::ConnectedState) {
    waiting_for_stop_ = true;
    stop_revocation_sent_ = true;
    send({{QStringLiteral("type"), QStringLiteral("disarm")}, {QStringLiteral("session_id"), session_.session_id}});
    if (socket_->state() == QLocalSocket::ConnectedState) {state_ = QStringLiteral("STOPPING");}
  }
  if (had_intent) {reason_ = reason + QStringLiteral("；重新按方向键即可继续。");}
  render();
}

void MappingTeleopPanel::transport_failed(const QString & reason, bool retry)
{
  retry_enabled_ = retry && session_.valid(); retry_age_.restart();
  keys_.clear(); sent_keys_ = intent_started_ = waiting_for_stop_ = arm_allowed_ = have_status_ = false;
  stop_revocation_sent_ = false;
  generation_ = 0; incoming_.clear(); status_age_.invalidate();
  state_ = QStringLiteral("DISCONNECTED"); reason_ = reason;
  disconnecting_ = true; socket_->abort(); disconnecting_ = false;
  render();
}

void MappingTeleopPanel::receive_bytes()
{
  incoming_ += socket_->readAll();
  int newline;
  while ((newline = incoming_.indexOf('\n')) >= 0) {
    if (newline + 1 > kLineBytes) {transport_failed(QStringLiteral("状态消息超过长度限制。"), false); return;}
    const QByteArray line = incoming_.left(newline); incoming_.remove(0, newline + 1);
    QJsonParseError error;
    const auto message = QJsonDocument::fromJson(line, &error);
    if (error.error != QJsonParseError::NoError || !message.isObject()) {
      transport_failed(QStringLiteral("后端状态不是有效 JSON。"), false); return;
    }
    receive_status(message.object());
    if (socket_->state() != QLocalSocket::ConnectedState) {return;}
  }
  if (incoming_.size() >= kLineBytes) {transport_failed(QStringLiteral("状态行未结束且超过长度限制。"), false);}
}

void MappingTeleopPanel::receive_status(const QJsonObject & status)
{
  const auto policy = status.value(QStringLiteral("interaction_policy")).toString(QStringLiteral("explicit_push"));
  if (policy != QStringLiteral("explicit_push") && policy != QStringLiteral("hybrid_manual")) {
    transport_failed(QStringLiteral("后端交互策略无效；不会自动重试。"), false); return;
  }
  hybrid_manual_ = policy == QStringLiteral("hybrid_manual");
  push_mode_ = status.value(QStringLiteral("push_mode")).toBool(false);
  push_requested_ = status.value(QStringLiteral("push_mode_requested")).toBool(false);
  qint64 generation;
  const QString next = status.value(QStringLiteral("state")).toString();
  const QStringList states = {QStringLiteral("READY"), QStringLiteral("DISARMED"), QStringLiteral("INITIALIZING"),
    QStringLiteral("ARMED"), QStringLiteral("FAULT")};
  if (status.value(QStringLiteral("type")).toString() != QStringLiteral("status") ||
    status.value(QStringLiteral("session_id")).toString() != session_.session_id || !states.contains(next) ||
    !status.value(QStringLiteral("arm_allowed")).isBool() || !status.value(QStringLiteral("reason")).isString() ||
    !checked_integer(status.value(QStringLiteral("arm_generation")), generation))
  {
    transport_failed(QStringLiteral("后端状态身份、代际或字段不匹配；不会自动重试。"), false); return;
  }
  const double linear = status.value(QStringLiteral("max_linear_m_s")).toDouble(-1.);
  const double angular = status.value(QStringLiteral("max_angular_rad_s")).toDouble(-1.);
  if (!std::isfinite(linear) || !std::isfinite(angular) || linear <= 0 || angular <= 0) {
    transport_failed(QStringLiteral("后端没有提供有效限速配置。"), false); return;
  }
  if (generation < generation_) {transport_failed(QStringLiteral("后端控制代际倒退。"), false); return;}
  have_status_ = true; status_age_.restart();
  arm_allowed_ = status.value(QStringLiteral("arm_allowed")).toBool();
  reason_ = status.value(QStringLiteral("reason")).toString();
  const QString block_reason = status.value(QStringLiteral("arm_block_reason")).toString();
  if (!arm_allowed_ && !block_reason.isEmpty()) {
    reason_ = QStringLiteral("WASD 不可用：") + block_reason + QStringLiteral(" | ") + reason_;
  }
  limits_label_->setText(QStringLiteral("后端限速：%1 m/s，%2 rad/s（未验证）；")
    .arg(linear, 0, 'f', 2).arg(angular, 0, 'f', 2) + (arm_allowed_ ?
    (hybrid_manual_ ? QStringLiteral("前台自动手推；WASD 接管，正常松键停车后恢复手推。") :
    QStringLiteral("窗口内直接按住 WASD 执行，松键停止，可切换手推。")) :
    QStringLiteral("仅展示配置，本会话 WASD 禁用。")));
  // Initialization does not change generation: keep the actual held key and
  // its heartbeat through READY -> INITIALIZING -> ARMED. A stop or backend
  // watchdog changes generation and invalidates old intent.
  if (!arm_allowed_ || next == QStringLiteral("DISARMED") || next == QStringLiteral("FAULT")) {
    keys_.clear(); sent_keys_ = intent_started_ = waiting_for_stop_ = false;
    stop_revocation_sent_ = false;
  } else if (waiting_for_stop_) {
    if (generation == generation_) {render(); return;}
    waiting_for_stop_ = sent_keys_ = false;  // Keys pressed after the local stop are fresh.
    stop_revocation_sent_ = false;
  } else if (generation != generation_) {
    keys_.clear(); sent_keys_ = intent_started_ = false;
  }
  generation_ = generation; state_ = next; render();
}

void MappingTeleopPanel::tick()
{
  if (retry_enabled_) {
    if (socket_->state() == QLocalSocket::UnconnectedState &&
      (!retry_age_.isValid() || retry_age_.elapsed() >= kReconnectRetryMs))
    {
      connect_to_server();
    }
  }
  if (socket_->state() == QLocalSocket::ConnectingState && connection_age_.elapsed() > 2000) {
    transport_failed(QStringLiteral("连接超时；正在自动重连。")); return;
  }
  if (socket_->state() == QLocalSocket::ConnectedState &&
    ((!have_status_ && connection_age_.elapsed() > 2000) ||
    (have_status_ && status_age_.elapsed() >= kStatusTimeoutMs)))
  {
    stop_and_disarm(QStringLiteral("后端状态超过 500 ms 未更新"));
    transport_failed(QStringLiteral("后端状态超时；已清空按键，正在自动重连。")); return;
  }
  if (hybrid_manual_ && have_status_ && arm_allowed_) {
    // Empty foreground packets are the persisted user's hybrid intent. Explicit
    // foreground=false prevents focus loss from looking like normal key release.
    send_keys();
  } else if (active_intent()) {
    if (!application_active()) {stop_and_disarm(QStringLiteral("已切出 RViz 窗口"));}
    else if (!keys_.isEmpty()) {send_keys();}
  }
}

bool MappingTeleopPanel::eventFilter(QObject * watched, QEvent * event)
{
  if (event->type() == QEvent::ApplicationActivate || event->type() == QEvent::WindowActivate) {
    application_inactive_ = false;
  }
  if (event->type() == QEvent::ApplicationDeactivate ||
    (watched == lifecycle_window_ && event->type() == QEvent::WindowDeactivate))
  {
    application_inactive_ = true;
    stop_and_disarm(QStringLiteral("已切出 RViz 窗口"));
  }
  if ((watched == this || watched == lifecycle_window_) &&
    (event->type() == QEvent::Hide || event->type() == QEvent::Close))
  {
    stop_and_disarm(QStringLiteral("控制面板隐藏或窗口关闭"));
  }
  if (event->type() != QEvent::KeyPress && event->type() != QEvent::KeyRelease) {return false;}
  if (!session_.valid() || !isVisible()) {return false;}
  auto * key = static_cast<QKeyEvent *>(event);
  if (key->key() == Qt::Key_Space) {
    if (!key->isAutoRepeat() && event->type() == QEvent::KeyPress) {stop_and_disarm(QStringLiteral("空格停车"));}
    return true;
  }
  if (key_name(key->key()).isEmpty()) {return false;}
  if (key->isAutoRepeat()) {return true;}
  const bool had_keys = !keys_.isEmpty();
  if (event->type() == QEvent::KeyRelease) {
    keys_.remove(key->key());
  }
  else if (keys_allowed())
  {
    keys_.insert(key->key()); intent_started_ = true;
  }
  if (!keys_.isEmpty() || had_keys) {
    const bool sent = send_keys();
    if (sent && keys_.isEmpty() && had_keys && !waiting_for_stop_ && have_status_) {
      // The backend advances generation on the final release. Queue any new
      // key until its acknowledgement, without replaying the released keys.
      waiting_for_stop_ = true; intent_started_ = false;
      stop_revocation_sent_ = false;
      state_ = QStringLiteral("STOPPING");
    }
  }
  render(); return true;
}

void MappingTeleopPanel::render()
{
  const bool connected = socket_->state() == QLocalSocket::ConnectedState;
  const bool connecting = socket_->state() == QLocalSocket::ConnectingState;
  const bool read_only = connected && have_status_ && !arm_allowed_;
  connection_label_->setText(connected ? QStringLiteral("已连接本会话") :
    (retry_enabled_ ? QStringLiteral("自动连接中") : QStringLiteral("控制连接不可用")));
  stop_button_->setEnabled(active_intent() || (hybrid_manual_ && connected && have_status_ && arm_allowed_));
  push_button_->setVisible(!hybrid_manual_ && have_status_ && arm_allowed_);
  push_button_->setText(push_mode_ ? QStringLiteral("退出手推") :
    (push_requested_ ? QStringLiteral("取消进入手推") : QStringLiteral("进入手推")));
  push_button_->setEnabled(connected && have_status_ && arm_allowed_ && !waiting_for_stop_ &&
    state_ != QStringLiteral("FAULT"));
  const QString connection = connected ?
    (read_only ? QStringLiteral("已连接，仅只读") :
    (have_status_ ? QStringLiteral("已连接") : QStringLiteral("已连接，等待状态"))) :
    (connecting ? QStringLiteral("连接中") : QStringLiteral("未连接"));
  state_label_->setText(QStringLiteral("手动控制：") + state_ + QStringLiteral(" | ") +
    connection + QStringLiteral(" | 会话：") + session_.session_id);
  reason_label_->setText(reason_);
  if (read_only) {
    control_guide_->setText(QStringLiteral("当前仅只读轮反馈，WASD 不会驱动车辆。\n"
        "不可用原因见上方。"));
  } else if (!connected || !have_status_) {
    control_guide_->setText(QStringLiteral("等待本会话连接和后端状态；WASD 不可用。\n"
        "服务就绪后自动恢复；无需连接或激活按钮。"));
  } else {
    control_guide_->setText(QStringLiteral("RViz 窗口内直接按键：W 前进 / S 后退 / A 左转 / D 右转；无需点击此区域。\n"
        "按住持续执行，松键停车后可手推；空格或切出窗口停车，重新按方向键即可继续。"));
  }
  if (hybrid_manual_ && connected && have_status_ && arm_allowed_) {
    control_guide_->setText(push_mode_ ?
      QStringLiteral("当前已收到手推释放回执。直接按 WASD 自动接管，正常松键停车后恢复手推。\n"
        "断线、失焦或故障不会触发释放；已释放也不代表有机械制动保证。") :
      QStringLiteral("窗口前台：直接手推与 WASD 混合操作，无需启用或切换按钮。\n"
        "正常松键后先归零，连续新鲜零反馈确认后恢复手推；空格停车暂停自动释放。"));
  } else if (push_mode_) {
    control_guide_->setText(QStringLiteral("手推模式：WASD 已暂停。先退出手推，再以新的按键恢复。\n"
      "驱动已确认释放指令；机械制动、坡道和载人安全仍需独立验收。"));
  } else if (connected && have_status_ && arm_allowed_) {
    control_guide_->setText(QStringLiteral("W 前进 / S 后退 / A 左转 / D 右转。松键或空格停车。\n"
      "停车后保留驱动状态；只有点击“进入手推”才请求释放。"));
  }
  QStringList pressed;
  for (int key : {Qt::Key_W, Qt::Key_A, Qt::Key_S, Qt::Key_D}) {
    if (keys_.contains(key)) {pressed.append(key_name(key).toUpper());}
  }
  keys_label_->setText(QStringLiteral("当前按键：") + (pressed.isEmpty() ? QStringLiteral("无") : pressed.join(QStringLiteral(" + "))));
}
}  // namespace wc_bringup
