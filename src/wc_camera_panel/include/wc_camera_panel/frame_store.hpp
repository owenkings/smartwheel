#pragma once

#include "wc_camera_panel/decoder.hpp"

#include <array>
#include <chrono>
#include <memory>
#include <mutex>

namespace wc_camera_panel {

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

struct SlotSnapshot {
  std::shared_ptr<const RgbFrame> frame;
  TimePoint last_valid{};
  std::uint64_t accepted{}, rejected{};
  bool last_event_valid{false};
  std::string error;
  std::string frame_id;
};

enum class PreviewState { waiting, live, stale, error };
PreviewState preview_state(const SlotSnapshot & slot, TimePoint now);

class FrameStore {
public:
  void accept(std::size_t index, RgbFrame frame, TimePoint received, const std::string & frame_id);
  void reject(std::size_t index, const std::string & error);
  std::array<SlotSnapshot, 4> snapshot() const;
private:
  mutable std::mutex mutex_;
  std::array<SlotSnapshot, 4> slots_;
};

}  // namespace wc_camera_panel
