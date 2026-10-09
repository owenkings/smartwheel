#pragma once

#include <QImage>
#include <QString>
#include <QWidget>

namespace wc_camera_panel {

class ImageCanvas : public QWidget {
public:
  explicit ImageCanvas(QWidget * parent = nullptr);
  void setFrame(QImage image);
  void setWarning(const QString & warning);
protected:
  void paintEvent(QPaintEvent * event) override;
private:
  QImage image_;
  QString warning_;
};

}  // namespace wc_camera_panel
