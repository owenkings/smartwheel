#include "smartwheel_rviz_plugins/map_products_panel.hpp"

#include <QComboBox>
#include <QDesktopServices>
#include <QFormLayout>
#include <QGridLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QScrollArea>
#include <QTimer>
#include <QUrl>
#include <QVBoxLayout>

#include <rviz_common/display_context.hpp>

namespace smartwheel_rviz_plugins
{

MapProductsPanel::MapProductsPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * content = new QWidget;
  auto * layout = new QVBoxLayout(content);
  layout->setContentsMargins(4, 4, 4, 4);
  versions_ = new QComboBox;
  layout->addWidget(new QLabel("Map version"));
  layout->addWidget(versions_);
  files_ = new QListWidget;
  files_->setMinimumHeight(130);
  layout->addWidget(files_);
  quality_ = new QLabel("Quality report not loaded");
  quality_->setWordWrap(true);
  layout->addWidget(quality_);

  auto * buttons = new QGridLayout;
  refresh_button_ = new QPushButton("Refresh");
  auto * select = new QPushButton("Select");
  auto * cloud = new QPushButton("Republish 3D");
  auto * map = new QPushButton("Republish 2D");
  auto * preview_button = new QPushButton("Preview");
  auto * open = new QPushButton("Open Directory");
  buttons->addWidget(refresh_button_, 0, 0);
  buttons->addWidget(select, 0, 1);
  buttons->addWidget(cloud, 1, 0);
  buttons->addWidget(map, 1, 1);
  buttons->addWidget(preview_button, 2, 0);
  buttons->addWidget(open, 2, 1);
  layout->addLayout(buttons);
  status_ = new QLabel("NOT CONNECTED");
  status_->setWordWrap(true);
  layout->addWidget(status_);

  auto * outer = new QVBoxLayout(this);
  auto * scroll = new QScrollArea;
  scroll->setWidgetResizable(true);
  scroll->setWidget(content);
  outer->addWidget(scroll);

  connect(refresh_button_, &QPushButton::clicked, this, &MapProductsPanel::requestList);
  connect(select, &QPushButton::clicked, this, &MapProductsPanel::selectCurrent);
  connect(cloud, &QPushButton::clicked, this, &MapProductsPanel::republishCloud);
  connect(map, &QPushButton::clicked, this, &MapProductsPanel::republishMap);
  connect(preview_button, &QPushButton::clicked, this, &MapProductsPanel::preview);
  connect(open, &QPushButton::clicked, this, &MapProductsPanel::openDirectory);
  connect(this, &MapProductsPanel::taskCompleted, this, &MapProductsPanel::applyResult, Qt::QueuedConnection);
}

MapProductsPanel::~MapProductsPanel()
{
  client_.reset();
}

void MapProductsPanel::onInitialize()
{
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    status_->setText("FAILED: RViz ROS node unavailable");
    return;
  }
  node_ = abstraction->get_raw_node();
  client_ = node_->create_client<smartwheel_interfaces::srv::MapProductTask>(service_name_.toStdString());
  QTimer::singleShot(1000, this, &MapProductsPanel::requestList);
}

void MapProductsPanel::call(uint8_t command)
{
  if (!client_ || !client_->service_is_ready()) {
    status_->setText("FAILED: map product service unavailable");
    return;
  }
  auto request = std::make_shared<smartwheel_interfaces::srv::MapProductTask::Request>();
  request->command = command;
  request->version_directory = versions_->currentText().toStdString();
  client_->async_send_request(request,
    [this](rclcpp::Client<smartwheel_interfaces::srv::MapProductTask>::SharedFuture future) {
      try {
        const auto response = future.get();
        QStringList versions;
        for (const auto & item : response->versions) {versions.append(QString::fromStdString(item));}
        QStringList files;
        for (const auto & item : response->available_files) {files.append(QString::fromStdString(item));}
        Q_EMIT taskCompleted(
          response->accepted, QString::fromStdString(response->reason),
          QString::fromStdString(response->selected_version), versions, files,
          QString::fromStdString(response->quality_summary));
      } catch (const std::exception & error) {
        Q_EMIT taskCompleted(false, QString::fromStdString(error.what()), "", {}, {}, "");
      }
    });
}

void MapProductsPanel::requestList() {call(smartwheel_interfaces::srv::MapProductTask::Request::LIST);}
void MapProductsPanel::selectCurrent() {call(smartwheel_interfaces::srv::MapProductTask::Request::SELECT);}
void MapProductsPanel::republishCloud() {call(smartwheel_interfaces::srv::MapProductTask::Request::REPUBLISH_CLOUD);}
void MapProductsPanel::republishMap() {call(smartwheel_interfaces::srv::MapProductTask::Request::REPUBLISH_MAP);}
void MapProductsPanel::preview() {call(smartwheel_interfaces::srv::MapProductTask::Request::PREVIEW);}

void MapProductsPanel::openDirectory()
{
  const QString directory = selected_version_.isEmpty() ? versions_->currentText() : selected_version_;
  if (directory.isEmpty() || !QDesktopServices::openUrl(QUrl::fromLocalFile(directory))) {
    status_->setText("FAILED: no selected map directory or desktop open failed");
  }
}

void MapProductsPanel::applyResult(
  bool accepted, const QString & reason, const QString & selected,
  const QStringList & versions, const QStringList & files, const QString & quality)
{
  status_->setText((accepted ? "OK: " : "FAILED: ") + reason);
  status_->setStyleSheet(accepted ? "color:#208a45;" : "color:#b42318;font-weight:600;");
  if (!versions.isEmpty()) {
    const QString previous = versions_->currentText();
    versions_->clear();
    versions_->addItems(versions);
    const int index = versions_->findText(previous);
    if (index >= 0) {versions_->setCurrentIndex(index);}
  }
  if (!selected.isEmpty()) {
    selected_version_ = selected;
    const int index = versions_->findText(selected);
    if (index >= 0) {versions_->setCurrentIndex(index);}
  }
  if (!files.isEmpty()) {
    files_->clear();
    files_->addItems(files);
  }
  if (!quality.isEmpty()) {quality_->setText(quality);}
}

void MapProductsPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  config.mapGetString("TaskService", &service_name_);
  config.mapGetString("SelectedVersion", &selected_version_);
  if (node_) {
    client_ = node_->create_client<smartwheel_interfaces::srv::MapProductTask>(service_name_.toStdString());
  }
}

void MapProductsPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("TaskService", service_name_);
  config.mapSetValue("SelectedVersion", selected_version_.isEmpty() ? versions_->currentText() : selected_version_);
}

}  // namespace smartwheel_rviz_plugins
