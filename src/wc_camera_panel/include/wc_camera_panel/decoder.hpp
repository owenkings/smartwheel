#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace wc_camera_panel {

struct ImageView {
  std::uint32_t width{}, height{}, step{};
  std::string encoding;
  std::uint8_t is_bigendian{};
  const std::uint8_t * data{};
  std::size_t data_size{};
};

struct RgbFrame {
  std::uint32_t width{}, height{};
  std::vector<std::uint8_t> rgb;
};

class ImageError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

// Each channel is one byte; byte endianness does not change rgb8/bgr8/mono8.
// Limits apply before any output allocation, independently of ROS/Qt.
RgbFrame decode_image(const ImageView & source);

}  // namespace wc_camera_panel
