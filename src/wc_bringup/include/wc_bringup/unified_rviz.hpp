// Shared, subscription-only RViz viewport and opt-in synthetic inspection tools.
#pragma once
#include <algorithm>
#include <array>
#include <functional>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QBuffer>
#include <QHBoxLayout>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPointer>
#include <QRegularExpression>
#include <QScreen>
#include <QTimer>
#include <QThread>
#include <QVBoxLayout>
#include <QWindow>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/properties/status_list.hpp>
#include <rviz_common/properties/property_tree_model.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/yaml_config_reader.hpp>
#include <rviz_rendering/render_window.hpp>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreRenderTarget.h>
#include <OgreRoot.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>

namespace wc_bringup {
inline QJsonObject panel_read_json(const QString & path) {
  QFile file(path);
  if (QFileInfo(path).isSymLink() || !file.open(QIODevice::ReadOnly) || file.size() > 2000000) {return {};}
  return QJsonDocument::fromJson(file.readAll()).object();
}

// Each manager needs its own node/executor. Ignore the host's __node remap,
// while explicitly sharing only the mapping TF namespace when requested.
class ViewNode final : public rviz_common::ros_integration::RosNodeAbstractionIface {
public:
  ViewNode(const std::string & name, bool mapping) {
    rclcpp::NodeOptions options; options.use_global_arguments(false);
    if (mapping) {options.arguments({"--ros-args", "-r", "/tf:=/wc_mapping/app/tf",
      "-r", "/tf_static:=/wc_mapping/app/tf_static"});}
    node_ = std::make_shared<rclcpp::Node>(name, options);
  }
  std::string get_node_name() const override {return node_->get_name();}
  std::map<std::string, std::vector<std::string>> get_topic_names_and_types() const override {
    return node_->get_topic_names_and_types();
  }
  rclcpp::Node::SharedPtr get_raw_node() override {return node_;}
private:
  rclcpp::Node::SharedPtr node_;
};

// Humble creates the same eight color materials in each manager constructor.
// In its Ogre 1.12.1, declining a resource collision makes create() return null;
// RViz then dereferences that null material. Do not use a collision listener.
// Hold old instances alive, remove only their registrations by exact handle,
// and let RViz create the defaults normally. Existing scenes retain valid old
// objects; later name lookups resolve to the newly registered identical defaults.
class RvizColorMaterialRetention final {
public:
  RvizColorMaterialRetention() {
    if (!qApp || QThread::currentThread() != qApp->thread()) {
      throw std::runtime_error("RViz managers must be constructed on the GUI thread");
    }
    auto & materials = Ogre::MaterialManager::getSingleton();
    for (size_t i = 0; i < names_.size(); ++i) {
      existing_[i] = materials.getByName(names_[i], "rviz_rendering");
      if (existing_[i]) {
        loaded_[i] = existing_[i]->isLoaded();
        ++retained_;
      }
    }
    if (retained_ != 0 && retained_ != static_cast<int>(names_.size())) {
      throw std::runtime_error("Partial RViz default material registration");
    }
    for (size_t i = 0; i < existing_.size(); ++i) {
      const auto & material = existing_[i];
      if (!material) {continue;}
      // Ogre permits referenced resources to survive remove(), but its public
      // contract allows unloading. Restore any previously loaded material on
      // the GUI thread before rendering can resume. Never removeAll() or
      // destroy the shared rviz_rendering group.
      materials.remove(material->getHandle());
      if (loaded_[i] && !material->isLoaded()) {material->load();}
    }
  }
  RvizColorMaterialRetention(const RvizColorMaterialRetention &) = delete;
  RvizColorMaterialRetention & operator=(const RvizColorMaterialRetention &) = delete;
  int verifyPreserved() const {
    auto & materials = Ogre::MaterialManager::getSingleton();
    for (size_t i = 0; i < names_.size(); ++i) {
      const auto current = materials.getByName(names_[i], "rviz_rendering");
      if (!current || (existing_[i] &&
        (current == existing_[i] || (loaded_[i] && !existing_[i]->isLoaded())))) {
        throw std::runtime_error("RViz color material retention or registration failed");
      }
    }
    return retained_;
  }
private:
  const std::array<const char *, 8> names_{{"RVIZ/Red", "RVIZ/Green", "RVIZ/Blue", "RVIZ/Cyan",
    "RVIZ/ShadedRed", "RVIZ/ShadedGreen", "RVIZ/ShadedBlue", "RVIZ/ShadedCyan"}};
  std::array<Ogre::MaterialPtr, 8> existing_;
  std::array<bool, 8> loaded_{};
  int retained_{};
};

class RvizViewport final : public QWidget {
public:
  RvizViewport(const QString & name, const rviz_common::Config & config,
    bool mapping, QWidget * parent = nullptr) : QWidget(parent), config_(config) {
    setObjectName(name);
    auto * layout = new QVBoxLayout(this); layout->setContentsMargins(0, 0, 0, 0);
    render_ = new rviz_common::RenderPanel(this); layout->addWidget(render_, 1);
    node_ = std::make_shared<ViewNode>(name.toStdString(), mapping);
    setMinimumSize(200, 160);
  }
  // Call only after this widget is in its final layout and the top-level
  // QWidget has been shown. Qt 5's createWindowContainer initially parents
  // the QWindow to ContainerFakeParent; its Show event installs the real
  // native parent. Creating Ogre's external drawable in the constructor binds
  // it before that transition and can leave a black/invalid GLX drawable.
  void initializeShown() {
    if (manager_) {throw std::runtime_error("RViz viewport already initialized");}
    auto * native = render_->getRenderWindow();
    auto * parent = native->parent();
    if (!isVisible() || !window()->isVisible() || !native->isVisible() || !parent ||
      parent->objectName().endsWith("ContainerFakeParent") || !native->handle() ||
      !parent->handle() || native->width() <= 0 || native->height() <= 0) {
      throw std::runtime_error("RViz viewport must be shown in its final native container before initialization");
    }
    initial_window_id_ = native->winId(); initial_parent_id_ = parent->winId();
    // Expose events may already have initialized the RenderWindow. Humble's
    // initialize() is not idempotent, so never call it a second time.
    using Adapter = rviz_rendering::RenderWindowOgreAdapter;
    if (!Adapter::getOgreViewport(native)) {native->initialize();}
    if (!Adapter::getOgreViewport(native) || !Adapter::getSceneManager(native)) {
      throw std::runtime_error("RViz render window has no Ogre viewport/scene");
    }
    retained_colors_.reset(new RvizColorMaterialRetention());
    manager_.reset(new rviz_common::VisualizationManager(render_, node_, nullptr,
      node_->get_raw_node()->get_clock()));
    setProperty("rviz_default_colors_retained", retained_colors_->verifyPreserved());
    render_->initialize(manager_.get());
    const auto materials_before = resourceHandles(Ogre::MaterialManager::getSingleton());
    const auto textures_before = resourceHandles(Ogre::TextureManager::getSingleton());
    manager_->initialize();
    rememberSelectionResources(materials_before, textures_before);
    manager_->load(config_);
    manager_->startUpdate();
    qInfo("RViz viewport %s initialized after native Show: window=0x%llx parent=0x%llx size=%dx%d",
      qPrintable(objectName()), static_cast<unsigned long long>(initial_window_id_),
      static_cast<unsigned long long>(initial_parent_id_), native->width(), native->height());
  }
  ~RvizViewport() override {
    if (manager_) {manager_->stopUpdate();}
    // Widgets outside the viewport (capture status) and QObject children
    // (map fitting) can retain SubscriptionBase/NodeBase after node_.reset().
    // Release these borrowers while the complete manager/plugin set is alive,
    // independently of Qt's sibling/child destruction order.
    for (const auto & entry : node_release_callbacks_) {
      if (entry.first) {entry.second();}
    }
    node_release_callbacks_.clear();
    auto * scene = rviz_rendering::RenderWindowOgreAdapter::getSceneManager(render_->getRenderWindow());
    QPointer<rviz_common::ViewManager> views(manager_ ? manager_->getViewManager() : nullptr);
    if (views) {
      // Humble does not own/delete this raw ViewManager. Destroy controllers
      // while their context and scene are still valid, including Orbit's
      // focal Shape. Then detach all paths from the soon-to-be-deleted panel.
      render_->setViewController(nullptr);
      // Humble's adapter ignores a null camera; detach the actual Ogre
      // viewport explicitly before a controller destroys its camera.
      if (auto * viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(render_->getRenderWindow())) {
        viewport->setCamera(nullptr);
      }
      views->setRenderPanel(nullptr);
      views->getPropertyModel()->getRoot()->removeChildren();
    }
    manager_.reset();
    // CallbackGroup retains weak subscription control blocks even after the
    // displays themselves are gone. Their virtual destructors can live in
    // rviz_default_plugins. Keep ViewManager's plugin factory alive until
    // the ROS node has released those blocks; deleting the factory first
    // unloads the library and makes NodeBase/CallbackGroup teardown jump into
    // unmapped code (verified on Humble/aarch64 with PointCloud2 loaded).
    node_.reset();
    // QPointer also handles an upstream version that does delete its manager.
    // Its property tree is now empty, so no dead DisplayContext is accessed.
    if (views) {delete views.data();}
    delete render_; render_ = nullptr;
    // Humble RenderWindowImpl owns the RenderTarget but does not release its
    // independently created SceneManager. Release this viewport's scene only
    // after all RViz displays, controllers and the render target are gone.
    // No other viewport shares this scene (RenderPanel use_main_scene=false).
    if (scene) {Ogre::Root::getSingleton().destroySceneManager(scene);}
    auto & materials = Ogre::MaterialManager::getSingleton();
    for (const auto handle : selection_material_handles_) {
      if (materials.getByHandle(handle)) {materials.remove(handle);}
    }
    auto & textures = Ogre::TextureManager::getSingleton();
    for (const auto & identity : selection_texture_names_) {
      // SelectionTexture is recreated under its instance-unique name when a
      // pick target is resized. This name was recorded during this manager's
      // initialization, never inferred from other live views at close time.
      const auto texture = textures.getByName(identity.second, identity.first);
      if (texture) {textures.remove(texture->getHandle());}
    }
  }
  rviz_common::VisualizationManager * manager() const {return manager_.get();}
  void beforeNodeRelease(QObject * owner, std::function<void()> release) {
    node_release_callbacks_.emplace_back(QPointer<QObject>(owner), std::move(release));
  }
  rclcpp::Node::SharedPtr node() const {return node_->get_raw_node();}
  QJsonObject nativeObservation() const {
    const auto xid = [](WId id) {return QStringLiteral("0x") + QString::number(static_cast<qulonglong>(id), 16);};
    auto * native = render_->getRenderWindow();
    auto * parent = native->parent();
    const WId current_id = native->handle() ? native->winId() : 0;
    const WId parent_id = parent && parent->handle() ? parent->winId() : 0;
    QJsonObject result{{"initialized", manager_ != nullptr}, {"visible", native->isVisible()},
      {"exposed", native->isExposed()}, {"initial_window_xid", xid(initial_window_id_)},
      {"window_xid", xid(current_id)}, {"initial_parent_xid", xid(initial_parent_id_)},
      {"parent_xid", xid(parent_id)}, {"window_id_stable", initial_window_id_ != 0 && initial_window_id_ == current_id},
      {"parent_id_stable", initial_parent_id_ != 0 && initial_parent_id_ == parent_id},
      {"width", native->width()}, {"height", native->height()}};
    auto * viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(native);
    if (viewport) {
      auto * target = viewport->getTarget();
      result["ogre_width"] = static_cast<int>(target->getWidth());
      result["ogre_height"] = static_cast<int>(target->getHeight());
      unsigned long window = 0;
      try {target->getCustomAttribute("WINDOW", &window); result["ogre_window_xid"] = xid(window);}
      catch (const std::exception & error) {result["ogre_window_error"] = QString::fromUtf8(error.what());}
    }
    return result;
  }
private:
  static std::set<Ogre::ResourceHandle> resourceHandles(Ogre::ResourceManager & resources) {
    std::set<Ogre::ResourceHandle> result;
    auto values = resources.getResourceIterator();
    while (values.hasMoreElements()) {result.insert(values.getNext()->getHandle());}
    return result;
  }
  void rememberSelectionResources(const std::set<Ogre::ResourceHandle> & materials_before,
    const std::set<Ogre::ResourceHandle> & textures_before) {
    const QRegularExpression material_name("^SelectionRect[0-9]+$");
    auto materials = Ogre::MaterialManager::getSingleton().getResourceIterator();
    while (materials.hasMoreElements()) {
      const auto resource = materials.getNext();
      if (!materials_before.count(resource->getHandle()) && resource->getGroup() == "rviz_rendering" &&
        material_name.match(QString::fromStdString(resource->getName())).hasMatch()) {
        selection_material_handles_.push_back(resource->getHandle());
      }
    }
    const QRegularExpression highlight_name("^SelectionRect[0-9]+Texture$");
    const QRegularExpression picking_name("^SelectionTexture[0-9]+$");
    auto textures = Ogre::TextureManager::getSingleton().getResourceIterator();
    while (textures.hasMoreElements()) {
      const auto resource = textures.getNext();
      const auto name = QString::fromStdString(resource->getName());
      if (!textures_before.count(resource->getHandle()) &&
        ((resource->getGroup() == "rviz_rendering" && highlight_name.match(name).hasMatch()) ||
        (resource->getGroup() == Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME &&
        picking_name.match(name).hasMatch()))) {
        selection_texture_names_.emplace_back(resource->getGroup(), resource->getName());
      }
    }
  }
  std::vector<Ogre::ResourceHandle> selection_material_handles_;
  std::vector<std::pair<QPointer<QObject>, std::function<void()>>> node_release_callbacks_;
  std::vector<std::pair<std::string, std::string>> selection_texture_names_;
  rviz_common::Config config_;
  WId initial_window_id_{}, initial_parent_id_{};
  rviz_common::RenderPanel * render_{};
  std::shared_ptr<ViewNode> node_;
  std::unique_ptr<RvizColorMaterialRetention> retained_colors_;
  std::unique_ptr<rviz_common::VisualizationManager> manager_;
};

inline rviz_common::Config load_view_config(const QString & path) {
  if (!QFileInfo(path).isFile() || QFileInfo(path).isSymLink()) {
    throw std::runtime_error("Missing or linked RViz configuration");
  }
  rviz_common::Config config;
  rviz_common::YamlConfigReader reader; reader.readFile(config, path);
  if (reader.error()) {throw std::runtime_error(reader.errorMessage().toStdString());}
  return config.mapGetChild("Visualization Manager");
}

inline QWidget * display_switches(rviz_common::VisualizationManager * manager, QWidget * parent) {
  auto * widget = new QWidget(parent);
  auto * layout = new QHBoxLayout(widget); layout->setContentsMargins(2, 2, 2, 2);
  auto * root = manager->getRootDisplayGroup();
  for (int i = 0; i < root->numDisplays(); ++i) {
    auto * display = root->getDisplayAt(i);
    if (display->getClassId() != "rviz_default_plugins/PointCloud2") {continue;}
    const QString topic = display->subProp("Topic")->getValue().toString();
    QString caption = display->getName();
    if (topic.endsWith("/cloud_map")) {caption = "累计 3D";}
    else if (topic.endsWith("/scan_cloud")) {caption = "实时 3D";}
    else if (caption.size() > 20) {
      caption = (topic.contains("left") ? QString("左雷达 ") :
        topic.contains("right") ? QString("右雷达 ") : QString("点云 ")) +
        (topic.contains("raw") ? "raw" : topic.contains("filtered") ? "filtered" : "预览");
    }
    auto * button = new QCheckBox(caption, widget);
    button->setToolTip(display->getName() + "\n" + topic);
    button->setChecked(display->isEnabled());
    button->setObjectName("toggle_" + display->getName());
    QObject::connect(button, &QCheckBox::toggled, display, [display](bool enabled) {display->setEnabled(enabled);});
    layout->addWidget(button);
  }
  layout->addStretch(); return widget;
}

// Missing measured cross-lidar extrinsics permit independent native viewing,
// not an invented transform or fused scene. This control changes only RViz's
// target frame; it never writes configuration or publishes TF.
inline QWidget * native_preview_frame_selector(rviz_common::VisualizationManager * manager,
  const QJsonObject & config, QWidget * parent) {
  if (!config.value("native_preview_only").toBool() || config.value("mapping_enabled").toBool() ||
    config.value("mode").toString() != "all") {return nullptr;}
  auto * widget = new QWidget(parent); widget->setObjectName("native_preview_frame_control");
  auto * layout = new QHBoxLayout(widget); layout->setContentsMargins(2, 2, 2, 2);
  layout->addWidget(new QLabel(QStringLiteral("原生坐标预览："), widget));
  auto * choice = new QComboBox(widget); choice->setObjectName("native_preview_frame_choice");
  choice->addItem(QStringLiteral("左雷达"), QStringLiteral("lidar_left"));
  choice->addItem(QStringLiteral("右雷达"), QStringLiteral("lidar_right"));
  choice->setCurrentIndex(manager->getFixedFrame() == "lidar_right" ? 1 : 0);
  choice->setToolTip(QStringLiteral("选择当前查看的雷达原生坐标，不对两雷达点云做融合。"));
  layout->addWidget(choice);
  auto * note = new QLabel(QStringLiteral("缺跨雷达实测外参，分别预览"), widget);
  note->setWordWrap(true); layout->addWidget(note, 1);
  QObject::connect(choice, qOverload<int>(&QComboBox::currentIndexChanged), manager,
    [choice, manager](int) {manager->setFixedFrame(choice->currentData().toString());});
  return widget;
}

inline QJsonArray display_observation(rviz_common::VisualizationManager * manager) {
  QJsonArray result;
  auto * group = manager->getRootDisplayGroup();
  for (int i = 0; i < group->numDisplays(); ++i) {
    auto * display = group->getDisplayAt(i);
    QJsonArray status;
    for (int child = 0; child < display->numChildren(); ++child) {
      auto * property = display->childAt(child);
      // StatusList updates its displayed name to "Status: Ok/Error". Match
      // the type instead of the label, otherwise every real status is missed.
      if (!dynamic_cast<rviz_common::properties::StatusList *>(property)) {continue;}
      for (int item = 0; item < property->numChildren(); ++item) {
        auto * state = property->childAt(item);
        auto * typed = dynamic_cast<rviz_common::properties::StatusProperty *>(state);
        status.append(QJsonObject{{"name", state->getName()}, {"value", state->getValue().toString()},
          {"level", typed ? static_cast<int>(typed->getLevel()) : -1}});
      }
    }
    result.append(QJsonObject{{"name", display->getName()}, {"enabled", display->isEnabled()},
      {"class", display->getClassId()}, {"status", status}});
  }
  return result;
}

// Explicit opt-in diagnostic writes are exclusive, never touch devices, and
// export both the rendered window and actual widget/subscription state.
inline void install_panel_diagnostics(QWidget * window, const QString & kind,
  std::function<QJsonObject()> extra = {}) {
  const QString screenshot = qEnvironmentVariable("WC_PANEL_SCREENSHOT");
  const QString state_path = qEnvironmentVariable("WC_PANEL_STATE_JSON");
  int timeout = qEnvironmentVariableIntValue("WC_PANEL_TEST_EXIT_MS");
  if (screenshot.isEmpty() && state_path.isEmpty() && timeout <= 0) {return;}
  if (timeout <= 0) {timeout = 5000;}
  timeout = std::max(500, std::min(120000, timeout));
  QTimer::singleShot(timeout, window, [window, kind, screenshot, state_path, extra]() {
    QJsonObject result{{"schema", "wc_unified_panel_observation_v1"}, {"kind", kind},
      {"window_visible", window->isVisible()}};
    QJsonArray children;
    for (auto * child : window->findChildren<QWidget *>()) {
      if (child->objectName().isEmpty()) {continue;}
      const auto position = child->mapTo(window, QPoint());
      children.append(QJsonObject{{"name", child->objectName()}, {"visible", child->isVisible()},
        {"x", position.x()}, {"y", position.y()}, {"width", child->width()}, {"height", child->height()}});
    }
    result["widgets"] = children; if (extra) {result["runtime"] = extra();}
    auto create_output = [](const QString & path, const QByteArray & bytes) {
      if (path.isEmpty()) {return;}
      QFileInfo info(path);
      if (!info.isAbsolute() || !info.dir().exists() || info.exists() || info.isSymLink()) {
        throw std::runtime_error("Diagnostic output must be a new absolute file in an existing directory");
      }
      QFile file(path);
      if (!file.open(QIODevice::WriteOnly | QIODevice::NewOnly) || file.write(bytes) != bytes.size()) {
        throw std::runtime_error("Diagnostic output write failed");
      }
    };
    try {
      if (!screenshot.isEmpty()) {
        auto * handle = window->windowHandle();
        if (!handle || !handle->screen()) {throw std::runtime_error("No owned screen");}
        QByteArray png; QBuffer buffer(&png); buffer.open(QIODevice::WriteOnly);
        if (!handle->screen()->grabWindow(window->winId()).save(&buffer, "PNG")) {
          throw std::runtime_error("Owned RViz screenshot failed");
        }
        create_output(screenshot, png);
      }
      create_output(state_path, QJsonDocument(result).toJson());
    } catch (const std::exception & error) {
      qCritical("Panel observation failed: %s", error.what());
      qApp->exit(2); return;
    }
    if (qEnvironmentVariableIntValue("WC_PANEL_TEST_EXIT_MS") > 0) {window->close();}
  });
}
}  // namespace wc_bringup
