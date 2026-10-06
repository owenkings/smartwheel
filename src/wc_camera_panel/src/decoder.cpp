#include "wc_camera_panel/decoder.hpp"

#include <algorithm>

namespace wc_camera_panel {

RgbFrame decode_image(const ImageView & source) {
  if (source.width == 0 || source.height == 0 || source.width > 1920 || source.height > 1080) {
    throw ImageError("image dimensions must be within 1..1920 x 1..1080");
  }
  const bool mono = source.encoding == "mono8";
  const bool bgr = source.encoding == "bgr8";
  if (!mono && !bgr && source.encoding != "rgb8") {
    throw ImageError("supported encodings: bgr8, rgb8, mono8");
  }
  if (source.is_bigendian > 1) {
    throw ImageError("invalid is_bigendian field");
  }
  const std::size_t channels = mono ? 1 : 3;
  const auto row_bytes = static_cast<std::size_t>(source.width) * channels;
  const auto expected = static_cast<std::uint64_t>(source.step) * source.height;
  if (source.step < row_bytes || expected > 16U * 1024U * 1024U ||
      expected != source.data_size || source.data == nullptr) {
    throw ImageError("invalid step/data length or input exceeds 16 MiB");
  }
  RgbFrame result{source.width, source.height, {}};
  result.rgb.resize(static_cast<std::size_t>(source.width) * source.height * 3);
  for (std::size_t y = 0; y < source.height; ++y) {
    const auto * input = source.data + y * source.step;
    auto * output = result.rgb.data() + y * source.width * 3;
    if (!mono && !bgr) {
      std::copy_n(input, row_bytes, output);
      continue;
    }
    for (std::size_t x = 0; x < source.width; ++x) {
      if (mono) {
        output[3 * x] = output[3 * x + 1] = output[3 * x + 2] = input[x];
      } else {
        output[3 * x] = input[3 * x + 2];
        output[3 * x + 1] = input[3 * x + 1];
        output[3 * x + 2] = input[3 * x];
      }
    }
  }
  return result;
}

}  // namespace wc_camera_panel
