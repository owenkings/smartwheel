#pragma once

#include <array>
#include <cstdint>

#include <rcl/allocator.h>
#include <wc_interfaces/msg/accepted_bundle.hpp>

namespace wc_slam {

// Only this fixed-size digest is retained per graph node, never its point cloud.
using AcceptedFingerprint = std::array<unsigned char, 32>;

// Covers every field using the installed ROS type support. CDR alignment bytes
// are zeroed before serialization; allocation/layout changes fail closed.
// The allocator argument also lets the native regression poison allocations.
AcceptedFingerprint accepted_fingerprint(
    const wc_interfaces::msg::AcceptedBundle &message,
    const rcl_allocator_t &allocator = rcl_get_default_allocator());

}  // namespace wc_slam
