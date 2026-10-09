// Rebind the existing main drawable and replace the independent 2D scene.
#pragma once
#include <QDockWidget>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/transformation/transformation_manager.hpp>
#include "wc_bringup/mapping_interactive_control.hpp"
#include "wc_bringup/mapping_teleop.hpp"
#include "wc_bringup/mapping_health.hpp"
#include "wc_bringup/grid_view_fit.hpp"

namespace wc_bringup {
class MappingSessionView final : public QObject {
public:
  MappingSessionView(rviz_common::VisualizationFrame & frame, QDockWidget & teleop)
  : QObject(&frame), frame_(frame), teleop_(teleop) {}

  void clear() {
    // Destruction stops the old timer, clears held keys, revokes only the old
    // owned socket and removes its application event filter before any new UI.
    delete frame_.findChild<QWidget *>("mapping_teleop_panel");
    delete frame_.findChild<QWidget *>("mapping_health_panel");
    delete frame_.findChild<QWidget *>("mapping_display_switches");
    delete frame_.findChild<QWidget *>("native_preview_frame_control");
    delete frame_.findChild<QWidget *>("mapping_grid_view");
    auto * manager = frame_.getManager();
    manager->removeAllDisplays();
    // resetTime alone resets displays. A new transformer also destroys the old
    // TF listener/buffer; reusing the plugin class does not reuse its instance.
    auto * transforms = manager->getTransformationManager();
    const auto previous = transforms->getCurrentTransformer();
    transforms->setTransformer(transforms->getCurrentTransformerInfo());
    mapping_control::require(previous != transforms->getCurrentTransformer(), "TF transformer cache was not replaced");
    frame_.setProperty("mapping_tf_instance_replaced", true);
    manager->resetTime(); manager->queueRender();
    if (auto * label = frame_.findChild<QLabel *>("mapping_grid_label")) {
      label->setText(QStringLiteral("2D 地图已清空，等待当前任务"));
    }
    if (auto * button = frame_.findChild<QPushButton *>("mapping_fit_grid_button")) {button->setEnabled(false);}
    if (auto * settings = frame_.findChild<QLabel *>("mapping_settings")) {
      settings->setText(QStringLiteral("任务切换中；旧会话显示和控制已解除绑定。"));
    }
    frame_.setProperty("mapping_bound_session", QString());
    frame_.setProperty("mapping_clear_count", frame_.property("mapping_clear_count").toInt() + 1);
  }

  void bind(const QJsonObject & status) {
    using mapping_control::require;
    const auto active = status["active_session"].toObject();
    const auto view = active["view_path"].toString();
    const auto directory = active["directory"].toString();
    const auto identity = active["session_id"].toString();
    const auto project = status["project_root"].toString();
    require(view == directory + "/view.rviz", "Mapping view is outside its active session");
    const auto runtime = mapping_control::read_object(directory + "/runtime_config.json", false);
    const auto session = mapping_control::read_object(directory + "/session.json", false);
    const auto text = mapping_control::read_bytes(view, false);
    require(QRegularExpression("^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$").match(identity).hasMatch() &&
      runtime["session_id"].toString() == identity && session["session_id"].toString() == identity &&
      session["project_root"].toString() == project && runtime["source_mode"].toString() == "real" &&
      runtime["status"].toString() == "EXPERIMENT" && active["mapping_enabled"].isBool() &&
      runtime["mapping_enabled"] == active["mapping_enabled"] &&
      session["mapping_enabled"] == active["mapping_enabled"], "Active mapping session identity mismatch");
    require(active["runtime"].toString() == project + "/.phase1_runtime/sessions/" + identity + "/mapping_app",
      "Active mapping runtime is outside this project/session");
    mapping_control::ordinary_ancestors(active["runtime"].toString());
    require(active["mapping_enabled"].toBool() == (status["state"].toString() == "MAPPING"),
      "Preview/save state must not display an old cumulative map");
    const QStringList arguments{"mapping_rviz", "-d", view};
    const auto control = teleop_session_from_arguments(arguments);
    require(control.valid() && control.session_id == identity, "Session control binding is invalid");
    rviz_common::YamlConfigReader reader; rviz_common::Config document;
    reader.readString(document, QString::fromUtf8(text));
    require(!reader.error(), "Malformed session RViz view");
    const auto config = document.mapGetChild("Visualization Manager");
    QString fixed_frame;
    require(config.mapGetChild("Global Options").mapGetString("Fixed Frame", &fixed_frame) &&
      !fixed_frame.isEmpty(), "Session view has no fixed frame");

    clear();
    auto * manager = frame_.getManager();
    manager->load(config); manager->resetTime();
    auto * center = frame_.findChild<QWidget *>("mapping_center_column");
    auto * left = frame_.findChild<QWidget *>("mapping_left_column");
    auto * settings = frame_.findChild<QLabel *>("mapping_settings");
    auto * fit = frame_.findChild<QPushButton *>("mapping_fit_grid_button");
    auto * label = frame_.findChild<QLabel *>("mapping_grid_label");
    require(center && left && settings && fit && label, "Mapping viewport layout is incomplete");
    auto * center_layout = qobject_cast<QVBoxLayout *>(center->layout());
    auto * left_layout = qobject_cast<QVBoxLayout *>(left->layout());
    auto * parameters = qobject_cast<QVBoxLayout *>(settings->parentWidget()->layout());
    auto * controls = qobject_cast<QVBoxLayout *>(teleop_.widget()->layout());
    require(center_layout && left_layout && parameters && controls, "Mapping viewport layout type mismatch");
    auto * switches = display_switches(manager, center);
    switches->setObjectName("mapping_display_switches"); center_layout->insertWidget(0, switches);
    if (auto * selector = native_preview_frame_selector(manager, runtime, center)) {
      center_layout->insertWidget(0, selector);
    }

    bool has_map = false; QString topic = "/wc_mapping/app/grid_map";
    auto * displays = manager->getRootDisplayGroup();
    for (int i = 0; i < displays->numDisplays(); ++i) {
      auto * display = displays->getDisplayAt(i);
      if (display->getClassId() == "rviz_default_plugins/Map") {
        topic = display->subProp("Topic")->getValue().toString();
        display->setEnabled(false); has_map = true;
      }
    }
    require(!has_map || active["mapping_enabled"].toBool(), "Preview view unexpectedly contains a map display");
    const auto yaml = QStringLiteral(
      "Global Options:\n  Fixed Frame: %1\n  Background Color: 24; 29; 36\n  Frame Rate: 10\n"
      "Displays:\n  - Class: rviz_default_plugins/Map\n    Name: 2D grid\n    Enabled: %2\n"
      "    Alpha: 1\n    Use Timestamp: false\n    Topic:\n      Value: %3\n      Reliability Policy: Reliable\n"
      "      Durability Policy: Transient Local\n      Depth: 1\n"
      "Tools:\n  - Class: rviz_default_plugins/MoveCamera\nViews:\n  Current:\n"
      "    Class: rviz_default_plugins/TopDownOrtho\n    Scale: 35\n    Angle: 0\n    X: 0\n    Y: 0\n")
      .arg(manager->getFixedFrame(), has_map ? "true" : "false", topic);
    rviz_common::Config grid_config;
    reader.readString(grid_config, yaml); require(!reader.error(), "Cannot configure new 2D view");
    auto * grid = new RvizViewport("mapping_grid_view", grid_config, true, left);
    left_layout->addWidget(grid, 1); grid->show();
    left_layout->activate(); grid->initializeShown();
    new GridViewportFit(grid, topic, fit, has_map);
    label->setText(has_map ? QStringLiteral("2D 栅格地图 · 拖动平移 / 滚轮缩放") :
      QStringLiteral("2D 栅格地图 · 当前帧预览不生成地图"));
    const auto estimator = runtime.value("wheel_imu_estimator").toString(runtime.value("estimator").toString("five_state"));
    settings->setText(QStringLiteral("当前任务：%1\n模式：%2\n雷达：%3\n点云：%4\nEKF：%5\n运动修正：%6\n过程噪声：%7\n几何轨迹纠偏：%8\n参数修改对下一次任务生效。")
      .arg(identity, active["mapping_enabled"].toBool() ? "实时建图" : "当前帧预览",
        runtime["mode"].toString(), runtime["cloud_source"].toString(),
        estimator == "robot_localization" ? "官方 robot_localization" : "五状态",
        runtime["motion_correction"].toBool() ? "已启用设备校正声明" : "关闭",
        runtime["process_noise"].toString() == "white_acceleration" ? "连续白噪声 PSD" : "原模型",
        runtime["geometry"].toBool() ? "启用（输出轨迹纠偏）" : "关闭"));
    parameters->addWidget(new MappingHealthPanel(arguments, settings->parentWidget()));
    controls->addWidget(new MappingTeleopPanel(control, &frame_, teleop_.widget()));
    frame_.setProperty("mapping_bound_session", identity);
    frame_.setProperty("mapping_bound_view_generation", status["view_generation"].toDouble());
    frame_.setProperty("mapping_bind_count", frame_.property("mapping_bind_count").toInt() + 1);
    frame_.setWindowModified(false); manager->queueRender();
  }
private:
  rviz_common::VisualizationFrame & frame_;
  QDockWidget & teleop_;
};
}  // namespace wc_bringup
