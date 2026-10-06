#include "wc_camera_panel/camera_panel.hpp"
#include "wc_camera_panel/image_canvas.hpp"

#include <QApplication>
#include <QFontMetrics>
#include <QGroupBox>
#include <QPainter>
#include <rviz_common/config.hpp>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool condition, const char * message) {
  if (!condition) { throw std::runtime_error(message); }
}
QImage render(QWidget & widget) {
  QImage image(widget.size(), QImage::Format_ARGB32);
  image.fill(Qt::transparent);
  QPainter painter(&image);
  widget.render(&painter);
  painter.end();
  return image;
}
}

int main(int argc, char ** argv) {
  QApplication app(argc, argv);
  try {
    // Constructor performs no ROS initialization, subscriptions or hardware I/O.
    wc_camera_panel::CameraPanel panel;
    rviz_common::Config defaults;
    panel.save(defaults);
    bool compact = true;
    check(defaults.mapGetBool("Compact Layout", &compact) && !compact, "compact layout enabled by default");
    check(panel.minimumSize() == QSize(980, 820) && panel.sizeHint() == QSize(1000, 860),
          "standalone camera panel default dimensions changed");
    panel.resize(1100, 920);
    panel.show();
    app.processEvents();
    const auto * grid = panel.findChild<QWidget *>("camera_fixed_grid");
    check(grid && grid->width() >= 960 && grid->height() >= 720, "four-tile grid smaller than 960x720");
    const auto * lf = panel.findChild<QGroupBox *>("camera_left_front");
    const auto * rf = panel.findChild<QGroupBox *>("camera_right_front");
    const auto * ls = panel.findChild<QGroupBox *>("camera_left_side");
    const auto * rs = panel.findChild<QGroupBox *>("camera_right_side");
    check(lf && rf && ls && rs, "missing fixed camera tile");
    check(lf->x() == ls->x() && rf->x() == rs->x() && lf->y() == rf->y() && ls->y() == rs->y(), "tiles do not form fixed rows/columns");
    check(lf->x() < rf->x() && lf->y() < ls->y(), "camera tile ordering changed");
    for (const auto * box : {lf, rf, ls, rs}) {
      check(box->title().contains(QString::fromUtf8("待确认")), "unconfirmed physical direction hidden");
      check(box->title().contains(QString::fromUtf8("USB标识未知")), "port inferred without a received frame");
    }
    const auto moved = wc_camera_panel::camera_title(QString::fromUtf8("左前"), "camera_usb3_4");
    check(moved.contains("USB4") && !moved.contains("USB1"), "title did not follow actual frame port after role remapping");
    for (const auto * frame_id : {"", "camera_usb3_40", "camera_usb3_0", "unverified_frame"}) {
      check(wc_camera_panel::camera_title(QString::fromUtf8("右前"), frame_id).contains(QString::fromUtf8("USB标识未知")),
            "invalid or absent frame port was inferred");
    }
    check(!render(panel).isNull(), "panel failed to render offscreen");
    rviz_common::Config mapping;
    mapping.mapSetValue("Mapping Status", "USER_CONFIRMED");
    mapping.mapSetValue("left_front USB Port", 1);
    mapping.mapSetValue("right_front USB Port", 4);
    mapping.mapSetValue("left_side USB Port", 2);
    mapping.mapSetValue("right_side USB Port", 3);
    panel.load(mapping);
    check(panel.minimumSize() == QSize(980, 820), "missing Compact Layout changed standalone dimensions");
    rviz_common::Config saved;
    panel.save(saved);
    int port = 0;
    check(saved.mapGetInt("right_front USB Port", &port) && port == 4, "user-confirmed right-front binding lost");
    check(wc_camera_panel::camera_title(QString::fromUtf8("右前"), "camera_usb3_4", 4).contains(QString::fromUtf8("方位已核对")), "matched binding still unconfirmed");
    check(wc_camera_panel::camera_title(QString::fromUtf8("右前"), "camera_usb3_2", 4).contains(QString::fromUtf8("来源与绑定不符")), "wrong USB source promoted to confirmed direction");

    mapping.mapSetValue("Compact Layout", true);
    panel.load(mapping);
    panel.resize(480, 400);
    app.processEvents();
    check(panel.minimumWidth() == 480 && panel.minimumHeight() >= 400 && panel.minimumHeight() <= 480 &&
          panel.sizeHint() == QSize(560, 460),
          "compact dimensions were not applied");
    check(panel.width() <= 560 && panel.height() <= 480, "compact panel cannot shrink beside the map");
    check(grid->width() < 960 && grid->height() < 720, "compact grid retained standalone minimum");
    check(panel.rect().contains(grid->geometry()), "compact grid is clipped outside the panel");
    check(lf->x() == ls->x() && rf->x() == rs->x() && lf->y() == rf->y() && ls->y() == rs->y(),
          "compact tiles no longer form fixed rows/columns");
    check(lf->geometry().right() < rf->x() && lf->geometry().bottom() < ls->y(),
          "compact camera tiles overlap");
    for (const auto * box : {lf, rf, ls, rs}) {
      const auto * status = box->findChild<QLabel *>();
      const auto * preview = box->findChild<QWidget *>(QString("image_") + box->objectName().mid(7));
      if (status && (!box->rect().contains(status->geometry()) ||
                    status->height() < status->heightForWidth(status->width()))) {
        std::cerr << "compact geometry: panel=" << panel.width() << 'x' << panel.height()
                  << " minimum=" << panel.minimumWidth() << 'x' << panel.minimumHeight()
                  << " tile=" << box->width() << 'x' << box->height()
                  << " status=" << status->x() << ',' << status->y() << ' '
                  << status->width() << 'x' << status->height()
                  << " required_text_height=" << status->heightForWidth(status->width()) << '\n';
      }
      check(box->title().contains(QString::fromUtf8("USB标识未知")), "compact title inferred a USB source");
      check(grid->rect().contains(box->geometry()), "compact camera tile is clipped outside the grid");
      check(box->fontMetrics().horizontalAdvance(box->title()) <= box->width() - 16, "compact title is too wide to read");
      check(status && status->text().contains(QString::fromUtf8("等待绑定图像")),
            "compact layout hid the binding confirmation state");
      check(status->wordWrap() && box->rect().contains(status->geometry()), "compact status is clipped outside its tile");
      check(status->height() >= status->heightForWidth(status->width()), "compact status text is vertically clipped");
      check(preview && preview->width() >= 160 && preview->height() >= 80, "compact image viewport collapsed");
      check(preview->geometry().bottom() < status->y(), "compact image overlaps the status");
    }
    check(!render(panel).isNull(), "compact panel failed to render offscreen");
    panel.save(saved);
    check(saved.mapGetBool("Compact Layout", &compact) && compact, "compact setting lost on save");
    check(saved.mapGetInt("right_front USB Port", &port) && port == 4, "compact save changed camera binding");
    wc_camera_panel::CameraPanel restored;
    restored.load(saved);
    rviz_common::Config restored_config;
    restored.save(restored_config);
    check(restored_config.mapGetBool("Compact Layout", &compact) && compact &&
          restored.minimumWidth() == 480 && restored.minimumHeight() >= 400 && restored.minimumHeight() <= 480,
          "compact layout did not survive save and load");

    mapping.mapSetValue("Compact Layout", false);
    panel.load(mapping);
    panel.resize(1100, 920);
    app.processEvents();
    panel.save(saved);
    check(saved.mapGetBool("Compact Layout", &compact) && !compact, "explicit false did not restore standalone layout");
    check(panel.minimumSize() == QSize(980, 820) && panel.sizeHint() == QSize(1000, 860) &&
          grid->width() >= 960 && grid->height() >= 720, "standalone dimensions not restored after compact load");
    for (const auto * box : {lf, rf, ls, rs}) {
      const auto * preview = box->findChild<QWidget *>(QString("image_") + box->objectName().mid(7));
      check(box->title().contains(QString::fromUtf8("等待绑定图像")), "standalone title not restored after compact load");
      check(preview && preview->minimumSize() == QSize(240, 160), "standalone canvas minimum not restored");
    }
    restored.load(rviz_common::Config());
    restored.save(restored_config);
    check(restored_config.mapGetBool("Compact Layout", &compact) && !compact &&
          restored.minimumSize() == QSize(980, 820), "missing compact key retained previous compact state");
    mapping.mapSetValue("right_front USB Port", 1);  // Duplicated physical source must lose confirmation.
    panel.load(mapping);
    panel.save(saved);
    QString state;
    check(saved.mapGetString("Mapping Status", &state) && state == "UNCONFIRMED", "invalid duplicated binding retained confirmation");

    wc_camera_panel::ImageCanvas canvas;
    canvas.resize(400, 400);
    QImage sample(4, 2, QImage::Format_RGB888);
    sample.fill(QColor(240, 30, 10));
    canvas.setFrame(sample);
    canvas.show();
    app.processEvents();
    const auto live = render(canvas);
    check(live.pixelColor(200, 200).red() > 200, "valid frame pixels not painted");
    check(live.pixelColor(200, 20).red() < 40, "image aspect ratio was stretched instead of letterboxed");
    canvas.setWarning(QString::fromUtf8("图像已过期"));
    const auto stale = render(canvas);
    check(stale.pixelColor(20, 110).red() < live.pixelColor(20, 110).red(), "stale frame not dimmed");
    check(stale != live, "stale warning did not affect visible rendering");
    std::cout << "PASS: offline Qt default/compact 2x2 layout and config round trip, binding labels, pixel rendering, aspect ratio and stale warning\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
