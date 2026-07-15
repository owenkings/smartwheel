#include "smartwheel_rviz_plugins/occupancy_grid_model.hpp"

#include <cmath>
#include <limits>

namespace smartwheel_rviz_plugins
{

QColor OccupancyGridModel::colorForCell(int8_t value)
{
  if (value < 0) {
    return QColor(128, 128, 128);
  }
  if (value >= 65) {
    return QColor(0, 0, 0);
  }
  return QColor(255, 255, 255);
}

bool OccupancyGridModel::setMap(const nav_msgs::msg::OccupancyGrid & map, std::string * error)
{
  const auto expected = static_cast<size_t>(map.info.width) * static_cast<size_t>(map.info.height);
  if (map.info.width == 0 || map.info.height == 0 || !std::isfinite(map.info.resolution) ||
    map.info.resolution <= std::numeric_limits<float>::epsilon() || map.data.size() != expected)
  {
    valid_ = false;
    image_ = QImage();
    if (error) {
      *error = "invalid dimensions, resolution, or row-major data length";
    }
    return false;
  }

  QImage next(static_cast<int>(map.info.width), static_cast<int>(map.info.height), QImage::Format_RGB888);
  for (uint32_t row = 0; row < map.info.height; ++row) {
    for (uint32_t column = 0; column < map.info.width; ++column) {
      const auto index = static_cast<size_t>(row) * map.info.width + column;
      next.setPixelColor(
        static_cast<int>(column), static_cast<int>(map.info.height - 1U - row),
        colorForCell(map.data[index]));
    }
  }
  map_ = map;
  image_ = next;
  valid_ = true;
  return true;
}

QPointF OccupancyGridModel::worldToScene(double world_x, double world_y) const
{
  if (!valid_) {
    return QPointF();
  }
  const auto & q = map_.info.origin.orientation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double dx = world_x - map_.info.origin.position.x;
  const double dy = world_y - map_.info.origin.position.y;
  const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
  const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  return QPointF(
    local_x / map_.info.resolution,
    static_cast<double>(map_.info.height) - local_y / map_.info.resolution);
}

QImage OccupancyGridModel::image() const
{
  return image_;
}

bool OccupancyGridModel::valid() const
{
  return valid_;
}

}  // namespace smartwheel_rviz_plugins
