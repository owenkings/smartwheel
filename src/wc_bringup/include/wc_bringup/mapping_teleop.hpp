// SPDX-License-Identifier: Apache-2.0
#ifndef WC_BRINGUP__MAPPING_TELEOP_HPP_
#define WC_BRINGUP__MAPPING_TELEOP_HPP_

#include <functional>
#include <QByteArray>
#include <QElapsedTimer>
#include <QJsonObject>
#include <QSet>
#include <QStringList>
#include <QWidget>

class QLabel;
class QLocalSocket;
class QPushButton;
class QTimer;

namespace wc_bringup
{
struct TeleopSession
{
  QString directory;
  QString session_id;
  QString unavailable_reason;
  bool valid() const {return !directory.isEmpty() && !session_id.isEmpty() && unavailable_reason.isEmpty();}
};

// Accept only a live session's own view.rviz and matching archived identities.
// An offline saved-map view must never attach to a vehicle control socket.
TeleopSession teleop_session_from_arguments(const QStringList & arguments);

// Validate the deterministic short socket location against this exact project
// and data session. Does not create directories or connect to a socket.
QString checked_manual_socket_directory(const QString & project, const QString & directory,
  const QString & identity, const QString & claimed);


class MappingTeleopPanel final : public QWidget
{
public:
  // Retained for source compatibility; direct keyboard control has no dialog.
  using Confirmation = std::function<bool()>;
  MappingTeleopPanel(const TeleopSession & session, QWidget * lifecycle_window,
    QWidget * parent = nullptr, Confirmation confirmation = Confirmation());
  ~MappingTeleopPanel() override;

  void connect_to_server();
  void stop_and_disarm(const QString & reason);

protected:
  bool eventFilter(QObject * watched, QEvent * event) override;

private:
  void receive_bytes();
  void receive_status(const QJsonObject & status);
  bool send(const QJsonObject & value);
  bool send_keys();
  void tick();
  void transport_failed(const QString & reason, bool retry = true);
  void render();
  bool active_intent() const;
  bool application_active() const;
  bool keys_allowed() const;

  TeleopSession session_;
  QWidget * lifecycle_window_;
  QWidget * control_;
  QLabel * state_label_;
  QLabel * reason_label_;
  QLabel * limits_label_;
  QLabel * keys_label_;
  QLabel * control_guide_;
  QLabel * connection_label_;
  QPushButton * stop_button_;
  QPushButton * push_button_;
  QLocalSocket * socket_;
  QTimer * timer_;
  QByteArray incoming_;
  QSet<int> keys_;
  QElapsedTimer status_age_;
  QElapsedTimer connection_age_;
  QElapsedTimer retry_age_;
  QString state_ = QStringLiteral("DISCONNECTED");
  QString reason_;
  bool have_status_ = false;
  bool arm_allowed_ = false;
  bool push_mode_ = false;
  bool push_requested_ = false;
  bool hybrid_manual_ = false;
  bool intent_started_ = false;
  bool sent_keys_ = false;
  bool waiting_for_stop_ = false;
  bool stop_revocation_sent_ = false;
  bool application_inactive_ = false;
  bool disconnecting_ = false;
  // Temporary transport failures retry for the entire mapping session.
  // Untrusted paths, identities and invalid protocol messages stay blocked.
  bool retry_enabled_ = false;
  qint64 generation_ = 0;
  qint64 sequence_ = 0;
};
}  // namespace wc_bringup
#endif  // WC_BRINGUP__MAPPING_TELEOP_HPP_
