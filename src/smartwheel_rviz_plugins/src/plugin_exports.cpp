#include "smartwheel_rviz_plugins/camera_panel.hpp"
#include "smartwheel_rviz_plugins/map_products_panel.hpp"
#include "smartwheel_rviz_plugins/mapping_control_panel.hpp"
#include "smartwheel_rviz_plugins/occupancy_grid_panel.hpp"
#include "smartwheel_rviz_plugins/system_status_panel.hpp"
#include "smartwheel_rviz_plugins/teleop_panel.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(smartwheel_rviz_plugins::OccupancyGridPanel, rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(smartwheel_rviz_plugins::CameraPanel, rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(smartwheel_rviz_plugins::TeleopPanel, rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(smartwheel_rviz_plugins::MappingControlPanel, rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(smartwheel_rviz_plugins::SystemStatusPanel, rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(smartwheel_rviz_plugins::MapProductsPanel, rviz_common::Panel)
