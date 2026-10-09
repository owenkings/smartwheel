// Real Qt/Ogre lifecycle and input regression. Publishes synthetic points only.
#include <QTest>
#include <QElapsedTimer>
#include <QImage>
#include <QMouseEvent>
#include <QWheelEvent>
#include <cstring>
#include <array>
#include "wc_bringup/unified_rviz.hpp"
#include "wc_bringup/grid_view_fit.hpp"
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSubEntity.h>
#include <rviz_rendering/render_system.hpp>
// Xlib is included by RenderSystem on Humble; do not let its macros affect Qt.
#ifdef Status
#undef Status
#endif
#ifdef Bool
#undef Bool
#endif
#ifdef None
#undef None
#endif

namespace {
using View = wc_bringup::RvizViewport;
const std::array<const char *, 8> color_names{{"RVIZ/Red", "RVIZ/Green", "RVIZ/Blue", "RVIZ/Cyan",
  "RVIZ/ShadedRed", "RVIZ/ShadedGreen", "RVIZ/ShadedBlue", "RVIZ/ShadedCyan"}};

rviz_common::Config view_config(bool clouds = false) {
  QString text = "Global Options:\n  Fixed Frame: lifecycle_frame\n  Background Color: 24; 29; 36\n"
    "  Frame Rate: 20\nTools:\n  - Class: rviz_default_plugins/MoveCamera\n"
    "Views:\n  Current:\n    Class: rviz_default_plugins/Orbit\n    Distance: 6\n"
    "    Pitch: 0.45\n    Yaw: 0.7\n    Focal Point:\n      X: 0\n      Y: 0\n      Z: 0\n";
  if (clouds) {
    text += "Displays:\n";
    for (const auto & name : {QString("scan_cloud"), QString("cloud_map")}) {
      text += QString("  - Class: rviz_default_plugins/PointCloud2\n    Name: %1\n"
        "    Enabled: true\n    Value: true\n    Style: Points\n    Size (Pixels): 6\n"
        "    Position Transformer: XYZ\n    Color Transformer: FlatColor\n    Color: %2\n"
        "    Topic:\n      Value: /wc_panel/lifecycle/%1\n      Depth: 1\n"
        "      Reliability Policy: Reliable\n      Durability Policy: Transient Local\n")
        .arg(name, name == "scan_cloud" ? "30; 120; 255" : "30; 230; 40");
    }
  }
  rviz_common::YamlConfigReader reader; rviz_common::Config result;
  reader.readString(result, text);
  if (reader.error()) {throw std::runtime_error(reader.errorMessage().toStdString());}
  return result;
}

bool frames(View & view, uint64_t count = 4) {
  const auto before = view.manager()->getFrameCount();
  QElapsedTimer timer; timer.start();
  while (view.manager()->getFrameCount() < before + count && timer.elapsed() < 4000) {QTest::qWait(40);}
  return view.manager()->getFrameCount() >= before + count;
}

Ogre::Entity * red_cube(View & view) {
  auto * scene = view.manager()->getSceneManager();
  auto * entity = scene->createEntity("lifecycle_red_cube", Ogre::SceneManager::PT_CUBE);
  entity->setMaterial(Ogre::MaterialManager::getSingleton().getByName("RVIZ/Red", "rviz_rendering"));
  auto * node = scene->getRootSceneNode()->createChildSceneNode();
  node->setScale(.015f, .015f, .015f); node->attachObject(entity);
  view.manager()->queueRender();
  return entity;
}

size_t material_count() {
  size_t count = 0;
  auto values = Ogre::MaterialManager::getSingleton().getResourceIterator();
  while (values.hasMoreElements()) {values.getNext(); ++count;}
  return count;
}

QJsonArray material_snapshot() {
  QJsonArray result;
  auto values = Ogre::MaterialManager::getSingleton().getResourceIterator();
  while (values.hasMoreElements()) {
    const auto resource = values.getNext();
    result.append(QJsonObject{{"name", QString::fromStdString(resource->getName())},
      {"group", QString::fromStdString(resource->getGroup())},
      {"handle", QString::number(resource->getHandle())},
      {"references", static_cast<double>(resource.use_count())}, {"loaded", resource->isLoaded()}});
  }
  return result;
}

QJsonArray pose(View & view) {
  auto * controller = view.manager()->getViewManager()->getCurrent();
  auto * camera = controller->getCamera();
  const auto position = camera->getDerivedPosition();
  const auto orientation = camera->getDerivedOrientation();
  return {position.x, position.y, position.z, orientation.w, orientation.x, orientation.y, orientation.z};
}

void drag(View & view, Qt::MouseButton button) {
  auto * target = view.manager()->getRenderPanel()->getRenderWindow();
  const QPointF start(target->width() / 2, target->height() / 2);
  QMouseEvent press(QEvent::MouseButtonPress, start, button, button, Qt::NoModifier);
  QCoreApplication::sendEvent(target, &press);
  QMouseEvent move(QEvent::MouseMove, start + QPointF(40, 20), Qt::NoButton, button, Qt::NoModifier);
  QCoreApplication::sendEvent(target, &move);
  QMouseEvent release(QEvent::MouseButtonRelease, start + QPointF(40, 20), button, Qt::NoButton, Qt::NoModifier);
  QCoreApplication::sendEvent(target, &release);
}

void zoom(View & view) {
  auto * target = view.manager()->getRenderPanel()->getRenderWindow();
  const QPointF point(target->width() / 2, target->height() / 2);
  QWheelEvent event(point, target->mapToGlobal(point.toPoint()), QPoint(), QPoint(0, 120),
    Qt::NoButton, Qt::NoModifier, Qt::NoScrollPhase, false);
  QCoreApplication::sendEvent(target, &event);
}

sensor_msgs::msg::PointCloud2 points(float x) {
  sensor_msgs::msg::PointCloud2 message;
  message.header.frame_id = "lifecycle_frame"; message.height = 1; message.width = 400;
  for (size_t i = 0; i < 3; ++i) {
    sensor_msgs::msg::PointField field;
    field.name = std::string(1, "xyz"[i]); field.offset = i * 4;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32; field.count = 1;
    message.fields.push_back(field);
  }
  message.point_step = 12; message.row_step = message.width * 12; message.is_dense = true;
  message.data.resize(message.row_step);
  for (uint32_t i = 0; i < message.width; ++i) {
    const float xyz[3] = {x, (static_cast<float>(i % 20) - 9.5f) * .045f,
      (static_cast<float>(i / 20) - 9.5f) * .045f};
    std::memcpy(message.data.data() + i * 12, xyz, 12);
  }
  return message;
}

bool displays_ready(View & view) {
  const auto displays = wc_bringup::display_observation(view.manager());
  if (displays.size() != 2) {return false;}
  for (const auto & item : displays) {
    const auto statuses = item.toObject()["status"].toArray();
    if (statuses.isEmpty()) {return false;}
    for (const auto & status : statuses) {if (status.toObject()["level"].toInt(-1) != 0) {return false;}}
  }
  return true;
}
}  // namespace

class RvizLifecycleTest : public QObject {
  Q_OBJECT
public:
  void writeReport(int result) {
    if (output_.isEmpty()) {return;}
    QFile file(output_ + "/result.json");
    if (!file.open(QIODevice::WriteOnly | QIODevice::NewOnly)) {throw std::runtime_error("Cannot create lifecycle result");}
    file.write(QJsonDocument(QJsonObject{{"status", result == 0 ? "PASS" : "FAIL"},
      {"level", "SYNTHETIC"}, {"hardware_started", false}, {"steps", observations_}}).toJson());
  }
private:
  QString output_;
  QJsonArray observations_;
  int screenshot_{};

  QImage capture(QWidget & window, const QString & phase) {
    auto * handle = window.windowHandle();
    if (!handle || !handle->screen()) {throw std::runtime_error("No owned window screen");}
    auto image = handle->screen()->grabWindow(window.winId()).toImage().scaled(window.size());
    const QString name = QString("%1_%2.png").arg(++screenshot_, 3, 10, QLatin1Char('0')).arg(phase);
    QFile file(output_ + "/" + name);
    if (image.isNull() || !file.open(QIODevice::WriteOnly | QIODevice::NewOnly) || !image.save(&file, "PNG")) {
      throw std::runtime_error("Owned window screenshot failed");
    }
    observations_.append(QJsonObject{{"phase", phase}, {"screenshot", name}});
    return image;
  }
  int color_pixels(const QImage & image, QWidget & window, View & view, char channel) {
    const QRect bounds(view.mapTo(&window, QPoint()), view.size());
    int count = 0;
    for (int y = std::max(0, bounds.top()); y < std::min(image.height(), bounds.bottom()); ++y) {
      for (int x = std::max(0, bounds.left()); x < std::min(image.width(), bounds.right()); ++x) {
        const auto pixel = image.pixelColor(x, y);
        if ((channel == 'r' && pixel.red() > 120 && pixel.green() < 90 && pixel.blue() < 90) ||
          (channel == 'g' && pixel.green() > 140 && pixel.red() < 90 && pixel.blue() < 90) ||
          (channel == 'b' && pixel.blue() > 150 && pixel.red() < 90 && pixel.green() < 160)) {++count;}
      }
    }
    observations_.append(QJsonObject{{"view", view.objectName()}, {"channel", QString(QChar(channel))}, {"pixels", count}});
    return count;
  }
private slots:
  void initTestCase() {
    const auto path = qEnvironmentVariable("WC_PANEL_LIFECYCLE_OUTPUT");
    const QFileInfo info(path);
    QVERIFY2(info.isAbsolute() && !info.exists() && !info.isSymLink() && info.dir().exists(),
      "Set WC_PANEL_LIFECYCLE_OUTPUT to a new absolute directory under the configured reports root");
    for (QString parent = info.dir().absolutePath(); ; ) {
      QVERIFY(!QFileInfo(parent).isSymLink());
      const auto next = QFileInfo(parent).dir().absolutePath();
      if (next == parent) {break;} parent = next;
    }
    QVERIFY(QDir().mkdir(path)); output_ = path;
  }

  void rendered_materials_survive_both_destruction_orders() {
    size_t baseline = 0;
    QJsonArray baseline_materials;
    for (int cycle = 0; cycle < 6; ++cycle) {
      QWidget window; window.resize(1100, 650); auto * layout = new QHBoxLayout(&window);
      std::unique_ptr<View> first(new View("lifecycle_first", view_config(), false, &window));
      layout->addWidget(first.get());
      window.show(); window.raise(); window.activateWindow();
      first->initializeShown();
      QPointer<rviz_common::ViewManager> first_controller_manager(first->manager()->getViewManager());
      auto * original_cube = red_cube(*first);
      std::array<std::weak_ptr<Ogre::Material>, 8> original;
      for (size_t i = 0; i < original.size(); ++i) {
        auto material = Ogre::MaterialManager::getSingleton().getByName(color_names[i], "rviz_rendering");
        QVERIFY(material); material->load(); original[i] = material;
      }
      QVERIFY(frames(*first));
      const auto before = capture(window, QString("cycle_%1_before_second").arg(cycle));
      QVERIFY2(color_pixels(before, window, *first, 'r') > 100, "First view must actually draw the old red material before second construction");

      std::unique_ptr<View> second(new View("lifecycle_second", view_config(), false, &window));
      layout->addWidget(second.get()); second->show(); second->initializeShown(); red_cube(*second);
      QPointer<rviz_common::ViewManager> second_controller_manager(second->manager()->getViewManager());
      QCOMPARE(second->property("rviz_default_colors_retained").toInt(), 8);
      QVERIFY(first->manager()->getSceneManager() != second->manager()->getSceneManager());
      QVERIFY(first->manager()->getRenderPanel() != second->manager()->getRenderPanel());
      QVERIFY(first->manager()->getViewManager() != second->manager()->getViewManager());
      for (size_t i = 0; i < original.size(); ++i) {
        QVERIFY(!original[i].expired());
        QVERIFY(original[i].lock()->isLoaded());
        QVERIFY(original[i].lock() != Ogre::MaterialManager::getSingleton().getByName(color_names[i], "rviz_rendering"));
      }
      QVERIFY(original_cube->getSubEntity(0)->getMaterial() == original[0].lock());
      QVERIFY(frames(*first)); QVERIFY(frames(*second));
      const auto both = capture(window, QString("cycle_%1_both").arg(cycle));
      QVERIFY(color_pixels(both, window, *first, 'r') > 100);
      QVERIFY(color_pixels(both, window, *second, 'r') > 100);

      const bool first_removed_first = cycle % 2 == 0;
      if (first_removed_first) {first.reset();} else {second.reset();}
      auto & survivor = first_removed_first ? *second : *first;
      window.resize(950, 570); QVERIFY(frames(survivor));
      const auto remaining = capture(window, QString("cycle_%1_survivor").arg(cycle));
      QVERIFY(color_pixels(remaining, window, survivor, 'r') > 100);
      first.reset(); second.reset();
      QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete); QTest::qWait(50);
      for (const auto & material : original) {QVERIFY2(material.expired(), "Retired materials must be released after both views are destroyed");}
      const auto resources = material_count();
      const auto materials = material_snapshot();
      if (cycle == 0) {baseline = resources; baseline_materials = materials;}
      else if (resources != baseline) {
        QStringList names;
        for (const auto & item : baseline_materials) {
          const auto value = item.toObject(); names.append(value["group"].toString() + "/" + value["name"].toString());
        }
        QJsonArray added;
        for (const auto & item : materials) {
          const auto value = item.toObject();
          if (!names.contains(value["group"].toString() + "/" + value["name"].toString())) {added.append(value);}
        }
        qInfo("Leaked material candidates: %s", QJsonDocument(added).toJson(QJsonDocument::Compact).constData());
        observations_.append(QJsonObject{{"material_baseline", baseline_materials}, {"materials_after_close", materials},
          {"added_materials", added}});
      }
      observations_.append(QJsonObject{{"cycle", cycle}, {"first_removed_first", first_removed_first},
        {"material_resources_after_close", static_cast<double>(resources)},
        {"first_view_manager_survived", !first_controller_manager.isNull()},
        {"second_view_manager_survived", !second_controller_manager.isNull()}});
      qInfo("Cycle %d after close: first ViewManager alive=%d second alive=%d resources=%zu", cycle,
        !first_controller_manager.isNull(), !second_controller_manager.isNull(), resources);
      QCOMPARE(resources, baseline);
      QVERIFY2(first_controller_manager.isNull(), "First ViewManager must be released with its viewport");
      QVERIFY2(second_controller_manager.isNull(), "Second ViewManager must be released with its viewport");
      window.close();
    }
  }

  void views_mouse_resize_and_cloud_toggles_remain_independent() {
    QWidget window; window.resize(1250, 720); auto * layout = new QHBoxLayout(&window);
    std::unique_ptr<View> left(new View("interaction_left", view_config(true), false, &window));
    std::unique_ptr<View> right(new View("interaction_right", view_config(true), false, &window));
    layout->addWidget(left.get()); layout->addWidget(right.get());
    window.show(); window.raise(); window.activateWindow();
    left->initializeShown(); right->initializeShown();
    // A sibling status widget can outlive both viewports in a real QSplitter.
    // Its subscription must release NodeBase before the last plugin factory.
    QObject status_owner;
    auto status_subscription = right->node()->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/wc_panel/lifecycle/scan_cloud", rclcpp::SensorDataQoS(),
      [](sensor_msgs::msg::PointCloud2::ConstSharedPtr) {});
    right->beforeNodeRelease(&status_owner, [&]() {status_subscription.reset();});
    auto * controls = wc_bringup::display_switches(left->manager(), left.get());
    static_cast<QVBoxLayout *>(left->layout())->insertWidget(0, controls);
    auto node = std::make_shared<rclcpp::Node>("wc_panel_lifecycle_synthetic_publisher");
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    auto live = node->create_publisher<sensor_msgs::msg::PointCloud2>("/wc_panel/lifecycle/scan_cloud", qos);
    auto map = node->create_publisher<sensor_msgs::msg::PointCloud2>("/wc_panel/lifecycle/cloud_map", qos);
    auto live_points = points(-.9f), map_points = points(.9f);
    QTimer publishing;
    connect(&publishing, &QTimer::timeout, [&]() {
      live_points.header.stamp = node->now(); map_points.header.stamp = live_points.header.stamp;
      live->publish(live_points); map->publish(map_points);
    });
    publishing.start(100); window.show(); window.raise(); window.activateWindow();
    QTRY_VERIFY_WITH_TIMEOUT(displays_ready(*left) && displays_ready(*right), 8000);
    QVERIFY(frames(*left)); QVERIFY(frames(*right));
    auto image = capture(window, "clouds_both_enabled");
    QVERIFY(color_pixels(image, window, *left, 'b') > 100);
    QVERIFY(color_pixels(image, window, *left, 'g') > 100);
    const auto right_before = pose(*right), left_before = pose(*left);
    drag(*left, Qt::LeftButton); QVERIFY(frames(*left));
    QVERIFY(pose(*left) != left_before); QCOMPARE(pose(*right), right_before);
    const auto left_rotated = pose(*left);
    drag(*right, Qt::MiddleButton); QVERIFY(frames(*right));
    QVERIFY(pose(*right) != right_before); QCOMPARE(pose(*left), left_rotated);
    const auto right_panned = pose(*right);
    zoom(*left); QVERIFY(frames(*left));
    QVERIFY(pose(*left) != left_rotated); QCOMPARE(pose(*right), right_panned);
    capture(window, "independent_rotate_pan_zoom");

    // Reset camera through the public view configuration so toggle comparisons
    // use the same framing. The input assertions above already checked routing.
    left->manager()->getViewManager()->getCurrent()->load(view_config().mapGetChild("Views").mapGetChild("Current"));
    auto * live_toggle = controls->findChild<QCheckBox *>("toggle_scan_cloud");
    auto * map_toggle = controls->findChild<QCheckBox *>("toggle_cloud_map");
    QVERIFY(live_toggle && map_toggle);
    QTest::mouseClick(live_toggle, Qt::LeftButton); QVERIFY(frames(*left));
    image = capture(window, "live_off_map_on");
    QVERIFY(color_pixels(image, window, *left, 'b') < 10);
    QVERIFY(color_pixels(image, window, *left, 'g') > 100);
    QVERIFY(right->manager()->getRootDisplayGroup()->getDisplayAt(0)->isEnabled());
    QTest::mouseClick(live_toggle, Qt::LeftButton); QTest::mouseClick(map_toggle, Qt::LeftButton);
    QTRY_VERIFY_WITH_TIMEOUT(displays_ready(*right), 4000); QVERIFY(frames(*left, 8));
    image = capture(window, "live_on_map_off");
    QVERIFY(color_pixels(image, window, *left, 'b') > 100);
    QVERIFY(color_pixels(image, window, *left, 'g') < 10);
    QVERIFY(right->manager()->getRootDisplayGroup()->getDisplayAt(1)->isEnabled());
    QTest::mouseClick(map_toggle, Qt::LeftButton);
    QTRY_VERIFY_WITH_TIMEOUT(displays_ready(*left), 4000);
    for (const QSize size : {QSize(900, 560), QSize(1400, 800), QSize(1100, 650)}) {
      window.resize(size); QVERIFY(frames(*left)); QVERIFY(frames(*right));
      image = capture(window, QString("resize_%1_%2").arg(size.width()).arg(size.height()));
      for (auto * view : {left.get(), right.get()}) {
        QVERIFY(view->width() >= 200 && view->height() >= 160);
        QVERIFY(color_pixels(image, window, *view, 'b') > 100);
        QVERIFY(color_pixels(image, window, *view, 'g') > 100);
      }
    }
    std::weak_ptr<rclcpp::Node> left_node = left->node(), right_node = right->node();
    QPointer<rviz_common::ViewManager> left_views(left->manager()->getViewManager());
    QPointer<rviz_common::ViewManager> right_views(right->manager()->getViewManager());
    publishing.stop(); left.reset(); right.reset();
    QVERIFY(left_node.expired()); QVERIFY(right_node.expired());
    QVERIFY(!status_subscription);
    QVERIFY(left_views.isNull()); QVERIFY(right_views.isNull());
    window.close();
  }

  void native_preview_switches_unrelated_lidar_frames_without_tf() {
    QWidget window; window.resize(950, 700); auto * layout = new QVBoxLayout(&window);
    auto config = view_config(true);
    config.mapGetChild("Global Options").mapSetValue("Fixed Frame", "lidar_left");
    std::unique_ptr<View> view(new View("native_frame_switch", config, false, &window));
    layout->addWidget(view.get()); window.show(); view->initializeShown();
    QJsonObject runtime{{"native_preview_only", true}, {"mapping_enabled", false}, {"mode", "all"}};
    auto * control = wc_bringup::native_preview_frame_selector(view->manager(), runtime, &window);
    QVERIFY(control); layout->insertWidget(0, control);
    auto * choice = control->findChild<QComboBox *>("native_preview_frame_choice");
    QVERIFY(choice); QCOMPARE(choice->currentData().toString(), QString("lidar_left"));
    for (const auto & blocked : {
      QJsonObject{{"native_preview_only", false}, {"mapping_enabled", false}, {"mode", "all"}},
      QJsonObject{{"native_preview_only", true}, {"mapping_enabled", true}, {"mode", "all"}},
      QJsonObject{{"native_preview_only", true}, {"mapping_enabled", false}, {"mode", "left"}}}) {
      QVERIFY(!wc_bringup::native_preview_frame_selector(view->manager(), blocked, &window));
    }
    auto node = std::make_shared<rclcpp::Node>("wc_native_frame_switch_synthetic_publisher");
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    auto left = node->create_publisher<sensor_msgs::msg::PointCloud2>("/wc_panel/lifecycle/scan_cloud", qos);
    auto right = node->create_publisher<sensor_msgs::msg::PointCloud2>("/wc_panel/lifecycle/cloud_map", qos);
    auto blue = points(-.9f), green = points(.9f);
    blue.header.frame_id = "lidar_left"; green.header.frame_id = "lidar_right";
    QTimer publishing;
    connect(&publishing, &QTimer::timeout, [&]() {
      blue.header.stamp = node->now(); green.header.stamp = blue.header.stamp;
      left->publish(blue); right->publish(green);
    });
    publishing.start(100); window.raise(); window.activateWindow();
    auto wait_for_colors = [&](char selected, char absent) {
      // QTRY re-evaluates its expression after the polling loop succeeds.
      // A fresh screen grab is non-monotonic, so a transient later blank frame
      // can produce a misleading "timeout too short" diagnostic. Use real
      // elapsed time, one image for both colors, and latch three successive
      // rendered frames that meet the unchanged pixel thresholds.
      QElapsedTimer elapsed; elapsed.start();
      uint64_t last_frame = view->manager()->getFrameCount();
      int stable = 0, selected_pixels = 0, absent_pixels = 0;
      while (elapsed.elapsed() < 8000) {
        QTest::qWait(50);
        if (elapsed.elapsed() >= 8000) {break;}
        const uint64_t frame = view->manager()->getFrameCount();
        if (frame == last_frame) {continue;}
        last_frame = frame;
        const auto image = window.windowHandle()->screen()->grabWindow(window.winId()).toImage();
        selected_pixels = color_pixels(image, window, *view, selected);
        absent_pixels = color_pixels(image, window, *view, absent);
        stable = selected_pixels > 100 && absent_pixels < 10 ? stable + 1 : 0;
        if (stable == 3) {
          qInfo("Native frame %s stable after %lld ms: selected=%d absent=%d",
            qPrintable(view->manager()->getFixedFrame()), static_cast<long long>(elapsed.elapsed()),
            selected_pixels, absent_pixels);
          return true;
        }
      }
      qWarning("Native color wait failed after %lld ms: frame=%llu stable=%d selected=%d absent=%d",
        static_cast<long long>(elapsed.elapsed()), static_cast<unsigned long long>(last_frame),
        stable, selected_pixels, absent_pixels);
      return false;
    };
    QVERIFY2(wait_for_colors('b', 'g'), "Left native cloud must render alone for three frames within 8 s");
    capture(window, "native_preview_left");
    QTest::keyClick(choice, Qt::Key_Down);
    QCOMPARE(view->manager()->getFixedFrame(), QString("lidar_right"));
    QVERIFY2(wait_for_colors('g', 'b'), "Right native cloud must render alone for three frames within 8 s");
    capture(window, "native_preview_right");
    QTest::keyClick(choice, Qt::Key_Up);
    QCOMPARE(view->manager()->getFixedFrame(), QString("lidar_left"));
    QVERIFY2(wait_for_colors('b', 'g'), "Returning left must render that cloud alone for three frames within 8 s");
    capture(window, "native_preview_left_again");
    observations_.append(QJsonObject{{"native_preview_frame_switch", "PASS"},
      {"frames", QJsonArray{"lidar_left", "lidar_right", "lidar_left"}}, {"tf_published", false}});
    publishing.stop(); view.reset(); window.close();
  }

  void first_grid_fits_once_and_button_restores_latest_bounds() {
    QWidget window; window.resize(800, 600); auto * layout = new QVBoxLayout(&window);
    auto config = view_config();
    auto camera = config.mapGetChild("Views").mapGetChild("Current");
    camera.mapSetValue("Class", "rviz_default_plugins/TopDownOrtho");
    camera.mapSetValue("Scale", 35.0); camera.mapSetValue("Angle", 0.0);
    camera.mapSetValue("X", 0.0); camera.mapSetValue("Y", 0.0);
    std::unique_ptr<View> view(new View("fit_grid", config, false, &window));
    auto * button = new QPushButton(&window); layout->addWidget(button); layout->addWidget(view.get());
    window.show(); view->initializeShown();
    const std::string topic = "/wc_panel/lifecycle/fit_grid";
    new wc_bringup::GridViewportFit(view.get(), QString::fromStdString(topic), button, true);
    QVERIFY(!button->isEnabled());
    auto node = std::make_shared<rclcpp::Node>("wc_panel_grid_fit_synthetic_publisher");
    auto publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>(topic,
      rclcpp::QoS(1).reliable().transient_local());
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "lifecycle_frame"; grid.header.stamp = node->now();
    grid.info.width = 160; grid.info.height = 100; grid.info.resolution = .05f;
    grid.info.origin.position.x = 20; grid.info.origin.position.y = -10;
    grid.info.origin.orientation.z = std::sqrt(.5); grid.info.origin.orientation.w = std::sqrt(.5);
    grid.data.assign(16000, 0); publisher->publish(grid);
    // Simulate delayed camera/dock layout changes after the first map arrives.
    QTimer::singleShot(100, &window, [&]() {window.resize(800, 450);});
    QTimer::singleShot(300, &window, [&]() {window.resize(800, 550);});
    auto state = [&]() {return view->property("grid_fit").toJsonObject();};
    QTRY_COMPARE_WITH_TIMEOUT(state()["fit_count"].toInt(), 1, 8000);
    QVERIFY(button->isEnabled()); QVERIFY(state()["auto_fit_done"].toBool());
    QVERIFY(std::abs(state()["center_x"].toDouble() - 17.5) < 1e-4);
    QVERIFY(std::abs(state()["center_y"].toDouble() + 6.0) < 1e-4);
    auto * current = view->manager()->getViewManager()->getCurrent();
    current->subProp("X")->setValue(123.0); current->subProp("Y")->setValue(-45.0);
    current->subProp("Angle")->setValue(.3); current->subProp("Scale")->setValue(7.0);
    grid.info.origin.position.x = 30; publisher->publish(grid);
    QTRY_VERIFY_WITH_TIMEOUT(std::abs(state()["bounds"].toArray()[0].toDouble() - 25.0) < 1e-4, 5000);
    QCOMPARE(state()["fit_count"].toInt(), 1);
    QCOMPARE(current->subProp("X")->getValue().toDouble(), 123.0);
    QCOMPARE(current->subProp("Y")->getValue().toDouble(), -45.0);
    QCOMPARE(current->subProp("Scale")->getValue().toDouble(), 7.0);
    QTest::mouseClick(button, Qt::LeftButton);
    QCOMPARE(state()["fit_count"].toInt(), 2);
    QVERIFY(std::abs(current->subProp("X")->getValue().toDouble() - 27.5) < 1e-4);
    QVERIFY(std::abs(current->subProp("Y")->getValue().toDouble() + 6.0) < 1e-4);
    QCOMPARE(current->subProp("Angle")->getValue().toDouble(), 0.0);
    QVERIFY(frames(*view));
    auto * viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(
      view->manager()->getRenderPanel()->getRenderWindow());
    const auto scale = current->subProp("Scale")->getValue().toDouble();
    QVERIFY(viewport->getActualWidth() / scale >= 5.0 * 1.09);
    QVERIFY(viewport->getActualHeight() / scale >= 8.0 * 1.09);
    observations_.append(QJsonObject{{"grid_fit", state()}, {"updated_map_preserved_manual_view", true}});
    capture(window, "grid_fit_after_explicit_reset");
    view.reset(); window.close();
  }
};

int main(int argc, char ** argv) {
  // These settings are internal to this test process, before ROS initialization.
  qputenv("ROS_LOCALHOST_ONLY", "1"); qputenv("ROS_DOMAIN_ID", "217");
  QApplication application(argc, argv); application.setQuitOnLastWindowClosed(false);
  rclcpp::init(0, nullptr);
  rviz_rendering::RenderSystem::get();
  RvizLifecycleTest test;
  const int result = QTest::qExec(&test, argc, argv);
  test.writeReport(result);
  rclcpp::shutdown();
  return result;
}
#include "test_rviz_lifecycle.moc"
