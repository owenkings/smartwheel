#ifndef SMARTWHEEL_RVIZ_PLUGINS__OCCUPANCY_GRID_MODEL_HPP_
#define SMARTWHEEL_RVIZ_PLUGINS__OCCUPANCY_GRID_MODEL_HPP_

#include <QColor>
#include <QImage>
#include <QPointF>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <string>

namespace smartwheel_rviz_plugins
{

class OccupancyGridModel
{
public:
  bool setMap(const nav_msgs::msg::OccupancyGrid & map, std::string * error = nullptr);
  QPointF worldToScene(double world_x, double world_y) const;
  QImage image() const;
  bool valid() const;
  static QColor colorForCell(int8_t value);

  uint32_t width() const {return map_.info.width;}
  uint32_t height() const {return map_.info.height;}
  double resolution() const {return map_.info.resolution;}

private:
  nav_msgs::msg::OccupancyGrid map_;
  QImage image_;
  bool valid_{false};
};

}  // namespace smartwheel_rviz_plugins

#endif  // SMARTWHEEL_RVIZ_PLUGINS__OCCUPANCY_GRID_MODEL_HPP_

