// SPDX-License-Identifier: Apache-2.0
#ifndef WC_BRINGUP__MAPPING_HEALTH_HPP_
#define WC_BRINGUP__MAPPING_HEALTH_HPP_
#include <QStringList>
#include <QWidget>
class QLabel;
class QPushButton;
class QTimer;
namespace wc_bringup {
// Read-only local reports; deliberately has no motor socket or device API.
class MappingHealthPanel final : public QWidget {
public:
  explicit MappingHealthPanel(const QStringList & arguments, QWidget * parent = nullptr);
private:
  void refresh();
  QString directory_;
  QString identity_;
  QString details_;
  QLabel * summary_;
  QPushButton * reasons_;
  QTimer * timer_;
};
}
#endif
