#include "wc_camera_panel/frame_store.hpp"

#include <utility>

namespace wc_camera_panel {

PreviewState preview_state(const SlotSnapshot & slot, TimePoint now) {
  if (!slot.last_event_valid && slot.rejected != 0) {
    return PreviewState::error;
  }
  if (!slot.frame) {
    return PreviewState::waiting;
  }
  return now - slot.last_valid > std::chrono::milliseconds(1500) ?
    PreviewState::stale : PreviewState::live;
}

void FrameStore::accept(std::size_t index, RgbFrame frame, TimePoint received, const std::string & frame_id) {
  auto owned = std::make_shared<const RgbFrame>(std::move(frame));
  std::lock_guard<std::mutex> lock(mutex_);
  auto & slot = slots_.at(index);
  slot.frame = std::move(owned);  // Exactly one latest frame retained per source.
  slot.last_valid = received;
  ++slot.accepted;
  slot.last_event_valid = true;
  slot.error.clear();
  slot.frame_id = frame_id.substr(0, 160);
}

void FrameStore::reject(std::size_t index, const std::string & error) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto & slot = slots_.at(index);
  ++slot.rejected;
  slot.last_event_valid = false;
  slot.error = error.substr(0, 160);
}

std::array<SlotSnapshot, 4> FrameStore::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return slots_;
}

}  // namespace wc_camera_panel
