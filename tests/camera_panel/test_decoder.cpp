#include "wc_camera_panel/decoder.hpp"
#include "wc_camera_panel/frame_store.hpp"

#include <atomic>
#include <iostream>
#include <limits>
#include <thread>

using namespace wc_camera_panel;

namespace {
void check(bool condition, const char * message) {
  if (!condition) { throw std::runtime_error(message); }
}
void rejects(ImageView value, const char * reason) {
  try { (void)decode_image(value); }
  catch (const ImageError &) { return; }
  throw std::runtime_error(reason);
}
RgbFrame pixel(std::uint8_t value) { return RgbFrame{1, 1, {value, value, value}}; }
}

int main() {
  try {
    // Padding is not image content, including when a row is not RGB aligned.
    const std::vector<std::uint8_t> bgr{3, 2, 1, 6, 5, 4, 239, 238,
                                       9, 8, 7, 12, 11, 10, 237, 236};
    ImageView source{2, 2, 8, "bgr8", 0, bgr.data(), bgr.size()};
    check(decode_image(source).rgb == std::vector<std::uint8_t>({1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}),
          "BGR channel order or row padding corrupted");
    source.encoding = "rgb8";
    const auto rgb = decode_image(source).rgb;
    check(rgb == std::vector<std::uint8_t>({3, 2, 1, 6, 5, 4, 9, 8, 7, 12, 11, 10}), "RGB row stride corrupted");
    source.is_bigendian = 1;
    check(decode_image(source).rgb == rgb, "8-bit channels must not be byte-swapped");
    const std::vector<std::uint8_t> mono{0, 255, 71, 42, 128, 72};
    const ImageView gray{2, 2, 3, "mono8", 1, mono.data(), mono.size()};
    check(decode_image(gray).rgb == std::vector<std::uint8_t>({0, 0, 0, 255, 255, 255, 42, 42, 42, 128, 128, 128}),
          "Mono expansion or padding corrupted");
    auto invalid = source; invalid.data_size -= 1; rejects(invalid, "truncated frame accepted");
    invalid = source; invalid.data_size += 1; rejects(invalid, "extra frame bytes accepted");
    invalid = source; invalid.step = 5; rejects(invalid, "undersized row accepted");
    invalid = source; invalid.width = 0; rejects(invalid, "empty frame accepted");
    invalid = source; invalid.height = 1081; rejects(invalid, "oversized height accepted");
    invalid = source; invalid.width = 1921; rejects(invalid, "oversized width accepted");
    invalid = source; invalid.width = std::numeric_limits<std::uint32_t>::max(); rejects(invalid, "overflow width accepted");
    invalid = source; invalid.step = std::numeric_limits<std::uint32_t>::max(); rejects(invalid, "overflow stride accepted");
    invalid = source; invalid.is_bigendian = 2; rejects(invalid, "invalid endian flag accepted");
    invalid = source; invalid.encoding = "16UC1"; rejects(invalid, "unconverted multibyte pixels accepted");
    invalid = source; invalid.data = nullptr; rejects(invalid, "null image bytes accepted");

    FrameStore store;
    const auto start = Clock::now();
    check(preview_state(store.snapshot()[0], start) == PreviewState::waiting, "new tile must wait");
    store.accept(0, pixel(1), start, "physical_usb_port");
    const auto held = store.snapshot()[0];
    store.accept(0, pixel(2), start + std::chrono::milliseconds(20), "new_port");
    const auto latest = store.snapshot()[0];
    check(latest.accepted == 2 && latest.frame->rgb[0] == 2, "latest frame not replaced");
    check(held.frame->rgb[0] == 1, "GUI snapshot must retain immutable old frame while producer replaces it");
    check(preview_state(latest, start + std::chrono::seconds(1)) == PreviewState::live, "fresh frame is stale");
    check(preview_state(latest, start + std::chrono::seconds(2)) == PreviewState::stale, "frozen image not marked stale");
    store.reject(0, "bad image");
    const auto bad = store.snapshot()[0];
    check(bad.accepted == 2 && bad.rejected == 1 && bad.frame == latest.frame, "invalid frame altered last valid pixels/count");
    check(preview_state(bad, start) == PreviewState::error, "malformed frame not marked as error");
    store.accept(0, pixel(3), start, std::string(1000, 'x'));
    check(store.snapshot()[0].frame_id.size() == 160, "unbounded metadata retained");
    check(preview_state(store.snapshot()[0], start) == PreviewState::live, "valid frame did not clear error state");

    // Exercise producer/GUI snapshots with concurrent replacement; no queue grows.
    std::atomic_bool done{false};
    std::thread producer([&store, &done, start]() {
      for (int index = 0; index < 2000; ++index) {
        store.accept(1, pixel(static_cast<std::uint8_t>(index)), start, "USB2");
      }
      done.store(true);
    });
    bool coherent = true;
    while (!done.load()) {
      const auto slots = store.snapshot();
      if (slots[1].frame) {
        const auto & values = slots[1].frame->rgb;
        coherent = coherent && values.size() == 3 && values[0] == values[1] && values[1] == values[2];
      }
    }
    producer.join();
    check(coherent && store.snapshot()[1].accepted == 2000, "concurrent latest-frame snapshots corrupted");
    check(store.snapshot()[2].accepted == 0 && store.snapshot()[3].accepted == 0, "source tile isolation lost");
    std::cout << "PASS: image encodings, stride, endian, bounds, latest-frame ownership, stale/error and concurrency\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
