#include "wc_camera_panel/image_canvas.hpp"

#include <QPainter>
#include <QPaintEvent>
#include <utility>

namespace wc_camera_panel {

ImageCanvas::ImageCanvas(QWidget * parent) : QWidget(parent) {
  setMinimumSize(240, 160);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void ImageCanvas::setFrame(QImage image) {
  image_ = std::move(image);
  update();
}

void ImageCanvas::setWarning(const QString & warning) {
  if (warning_ != warning) {
    warning_ = warning;
    update();
  }
}

void ImageCanvas::paintEvent(QPaintEvent *) {
  QPainter painter(this);
  painter.fillRect(rect(), QColor(17, 24, 32));
  if (!image_.isNull()) {
    const auto scaled = image_.size().scaled(size(), Qt::KeepAspectRatio);
    const QRect target((width() - scaled.width()) / 2, (height() - scaled.height()) / 2,
                       scaled.width(), scaled.height());
    painter.setRenderHint(QPainter::SmoothPixmapTransform);
    painter.setOpacity(warning_.isEmpty() ? 1.0 : 0.3);
    painter.drawImage(target, image_);
    painter.setOpacity(1.0);
  }
  if (!warning_.isEmpty()) {
    painter.fillRect(QRect(8, height() / 2 - 36, width() - 16, 72), QColor(80, 20, 25, 220));
    painter.setPen(QColor(255, 230, 230));
    auto font = painter.font();
    font.setBold(true);
    font.setPointSize(13);
    painter.setFont(font);
    painter.drawText(rect().adjusted(16, 8, -16, -8), Qt::AlignCenter | Qt::TextWordWrap, warning_);
  }
}

}  // namespace wc_camera_panel
