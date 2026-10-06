// Native generated ROS messages/type support are required. Synthetic software
// identity test only: no graph, sensors, driver, or device calibration involved.
#include "wc_slam/accepted_fingerprint.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Accepted = wc_interfaces::msg::AcceptedBundle;
void expect(bool condition, const std::string &reason) {
  if (!condition) throw std::runtime_error(reason);
}
struct PoisonAllocator {
  rcl_allocator_t base = rcl_get_default_allocator();
  unsigned char pattern = 0;
  size_t allocations = 0;
  static void *allocate(size_t size, void *state) {
    auto &self = *static_cast<PoisonAllocator *>(state);
    void *value = self.base.allocate(size, self.base.state);
    if (value) std::memset(value, self.pattern, size);
    ++self.allocations;
    return value;
  }
  static void deallocate(void *value, void *state) {
    auto &self = *static_cast<PoisonAllocator *>(state);
    self.base.deallocate(value, self.base.state);
  }
  static void *reallocate(void *value, size_t size, void *state) {
    auto &self = *static_cast<PoisonAllocator *>(state);
    // Initial buffer allocation commonly enters via realloc(nullptr, size).
    if (!value) return allocate(size, state);
    ++self.allocations;
    return self.base.reallocate(value, size, self.base.state);
  }
  static void *zero_allocate(size_t count, size_t size, void *state) {
    auto &self = *static_cast<PoisonAllocator *>(state);
    ++self.allocations;
    return self.base.zero_allocate(count, size, self.base.state);
  }
  rcl_allocator_t allocator() {
    rcl_allocator_t value{};
    value.allocate = &allocate;
    value.deallocate = &deallocate;
    value.reallocate = &reallocate;
    value.zero_allocate = &zero_allocate;
    value.state = this;
    return value;
  }
};

Accepted fixture() {
  Accepted message;
  auto &b = message.bundle;
  auto &t = message.tracking;
  b.header.frame_id = "rig_link";
  b.header.stamp.sec = 100;
  b.header.stamp.nanosec = 123;
  b.session_id = "odd_sized_session";
  b.bundle_id = 91;
  b.source_mode = "synthetic";
  b.sensor_mode = "dual";
  b.calibration_id = "synthetic_calibration";
  b.left_clock_model_id = "left_clock";
  b.right_clock_model_id = "right_clock_odd";
  b.left_raw_key = "left:0091";
  b.right_raw_key = "right:0091";
  b.left_time_ns = 100000000123ll;
  b.right_time_ns = 100000000125ll;
  b.t_ref_left_sensor.rotation.w = 1.0;
  b.t_ref_right_sensor.rotation.w = 1.0;
  b.t_ref_rig_at_left_time.rotation.w = 1.0;
  b.t_ref_rig_at_right_time.rotation.w = 1.0;
  b.compensation_mode = "synthetic_truth";
  b.temporal_error_bound_valid = true;
  b.temporal_error_bound_m = 0.001;
  b.left_retained_count = 19;
  b.right_retained_count = 22;
  b.cloud.header = b.header;
  b.cloud.height = 1;
  b.cloud.width = 41;
  b.cloud.point_step = 12;
  b.cloud.row_step = 492;
  b.cloud.data.resize(492);
  for (size_t i = 0; i < b.cloud.data.size(); ++i)
    b.cloud.data[i] = static_cast<uint8_t>((i * 37) % 256);
  sensor_msgs::msg::PointField x;
  x.name = "x"; x.offset = 0; x.datatype = 7; x.count = 1;
  b.cloud.fields = {x};
  b.source_codes.resize(41, 3);
  b.calibration_valid = b.time_valid = b.dual_valid = true;
  // Nonempty arrays exercise both sequence length and per-string alignment.
  // This test fingerprints a message, it does not submit its flags to a graph.
  b.rejection_reasons = {"one", "different_string_length"};
  t.header = b.header;
  t.session_id = b.session_id;
  t.bundle_id = b.bundle_id;
  t.odom_epoch = "synthetic_odom";
  t.tracking_state = "MAPPING_DUAL";
  t.accepted = true;
  t.rejection_reasons = {"test_array"};
  t.left_solver_input_count = 19;
  t.right_solver_input_count = 22;
  t.inliers_available = t.residual_available = t.degeneracy_available = true;
  t.inliers = 31; t.residual = 0.01; t.degeneracy_metric = 0.125;
  t.odometry.header = b.header;
  t.odometry.header.frame_id = "odom";
  t.odometry.child_frame_id = "rig_link";
  t.odometry.pose.pose.orientation.w = 1.0;
  t.odometry.pose.pose.position.x = 0.125;
  for (size_t i = 0; i < 36; ++i) {
    t.odometry.pose.covariance[i] = 0.125 + i;
    t.odometry.twist.covariance[i] = 0.25 + i;
  }
  return message;
}
}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    static_assert(sizeof(wc_slam::AcceptedFingerprint) == 32,
                  "per-node retry identity must remain exactly one SHA256");
    const auto message = fixture();
    const auto baseline = wc_slam::accepted_fingerprint(message);
    for (unsigned int pattern = 0; pattern < 256; ++pattern) {
      PoisonAllocator poison;
      poison.pattern = static_cast<unsigned char>(pattern);
      std::vector<std::vector<unsigned char>> noise;
      for (size_t i = 1; i <= 13; ++i)
        noise.emplace_back(i * 131 + pattern, static_cast<unsigned char>(pattern ^ i));
      const Accepted copied = message;
      const auto digest = wc_slam::accepted_fingerprint(copied, poison.allocator());
      expect(poison.allocations > 0, "poison allocator was not exercised");
      expect(digest == baseline, "identical message hash depends on allocator/padding");
    }
    using Mutation = std::pair<std::string, std::function<void(Accepted &)>>;
    const std::vector<Mutation> changes{
      {"bundle.header.stamp", [](auto &m) { ++m.bundle.header.stamp.nanosec; }},
      {"bundle.header.frame_id", [](auto &m) { m.bundle.header.frame_id += "x"; }},
      {"bundle.session_id", [](auto &m) { m.bundle.session_id += "x"; }},
      {"bundle.bundle_id", [](auto &m) { ++m.bundle.bundle_id; }},
      {"bundle.source_mode", [](auto &m) { m.bundle.source_mode = "real"; }},
      {"bundle.sensor_mode", [](auto &m) { m.bundle.sensor_mode = "left"; }},
      {"bundle.calibration_id", [](auto &m) { m.bundle.calibration_id += "x"; }},
      {"bundle.left_clock", [](auto &m) { m.bundle.left_clock_model_id += "x"; }},
      {"bundle.right_clock", [](auto &m) { m.bundle.right_clock_model_id += "x"; }},
      {"bundle.left_key", [](auto &m) { m.bundle.left_raw_key += "x"; }},
      {"bundle.right_key", [](auto &m) { m.bundle.right_raw_key += "x"; }},
      {"bundle.left_time", [](auto &m) { ++m.bundle.left_time_ns; }},
      {"bundle.right_time", [](auto &m) { ++m.bundle.right_time_ns; }},
      {"bundle.left_origin", [](auto &m) { m.bundle.t_ref_left_sensor.translation.x = 1.0; }},
      {"bundle.right_origin", [](auto &m) { m.bundle.t_ref_right_sensor.rotation.x = 1.0; }},
      {"bundle.left_motion", [](auto &m) { m.bundle.t_ref_rig_at_left_time.translation.y = 1.0; }},
      {"bundle.right_motion", [](auto &m) { m.bundle.t_ref_rig_at_right_time.rotation.z = 1.0; }},
      {"bundle.compensation", [](auto &m) { m.bundle.compensation_mode += "x"; }},
      {"bundle.temporal_valid", [](auto &m) { m.bundle.temporal_error_bound_valid = false; }},
      {"bundle.temporal_one_bit", [](auto &m) { m.bundle.temporal_error_bound_m = std::nextafter(m.bundle.temporal_error_bound_m, 1.0); }},
      {"bundle.left_retained", [](auto &m) { ++m.bundle.left_retained_count; }},
      {"bundle.right_retained", [](auto &m) { ++m.bundle.right_retained_count; }},
      {"bundle.cloud.header", [](auto &m) { ++m.bundle.cloud.header.stamp.sec; }},
      {"bundle.cloud.height", [](auto &m) { ++m.bundle.cloud.height; }},
      {"bundle.cloud.width", [](auto &m) { ++m.bundle.cloud.width; }},
      {"bundle.cloud.field", [](auto &m) { ++m.bundle.cloud.fields[0].offset; }},
      {"bundle.cloud.endian", [](auto &m) { m.bundle.cloud.is_bigendian = true; }},
      {"bundle.cloud.point_step", [](auto &m) { ++m.bundle.cloud.point_step; }},
      {"bundle.cloud.row_step", [](auto &m) { ++m.bundle.cloud.row_step; }},
      {"bundle.cloud.data_bit", [](auto &m) { m.bundle.cloud.data[31] ^= 1; }},
      {"bundle.cloud.dense", [](auto &m) { m.bundle.cloud.is_dense = true; }},
      {"bundle.source_codes_byte", [](auto &m) { m.bundle.source_codes[2] = 1; }},
      {"bundle.cal_valid", [](auto &m) { m.bundle.calibration_valid = false; }},
      {"bundle.time_valid", [](auto &m) { m.bundle.time_valid = false; }},
      {"bundle.dual_valid", [](auto &m) { m.bundle.dual_valid = false; }},
      {"bundle.rejections", [](auto &m) { m.bundle.rejection_reasons[1] += "x"; }},
      {"tracking.header", [](auto &m) { ++m.tracking.header.stamp.sec; }},
      {"tracking.session", [](auto &m) { m.tracking.session_id += "x"; }},
      {"tracking.id", [](auto &m) { ++m.tracking.bundle_id; }},
      {"tracking.epoch", [](auto &m) { m.tracking.odom_epoch += "x"; }},
      {"tracking.state", [](auto &m) { m.tracking.tracking_state += "x"; }},
      {"tracking.accepted", [](auto &m) { m.tracking.accepted = false; }},
      {"tracking.rejections", [](auto &m) { m.tracking.rejection_reasons.push_back("x"); }},
      {"tracking.left_count", [](auto &m) { ++m.tracking.left_solver_input_count; }},
      {"tracking.right_count", [](auto &m) { ++m.tracking.right_solver_input_count; }},
      {"tracking.inliers_available", [](auto &m) { m.tracking.inliers_available = false; }},
      {"tracking.inliers", [](auto &m) { ++m.tracking.inliers; }},
      {"tracking.residual_available", [](auto &m) { m.tracking.residual_available = false; }},
      {"tracking.residual_one_bit", [](auto &m) { m.tracking.residual = std::nextafter(m.tracking.residual, 1.0); }},
      {"tracking.degeneracy_available", [](auto &m) { m.tracking.degeneracy_available = false; }},
      {"tracking.degeneracy_one_bit", [](auto &m) { m.tracking.degeneracy_metric = std::nextafter(m.tracking.degeneracy_metric, 1.0); }},
      {"odom.header", [](auto &m) { m.tracking.odometry.header.frame_id += "x"; }},
      {"odom.child", [](auto &m) { m.tracking.odometry.child_frame_id += "x"; }},
      {"odom.pose_one_bit", [](auto &m) { m.tracking.odometry.pose.pose.position.x = std::nextafter(m.tracking.odometry.pose.pose.position.x, 1.0); }},
      {"odom.covariance_one_bit", [](auto &m) { m.tracking.odometry.pose.covariance[35] = std::nextafter(m.tracking.odometry.pose.covariance[35], 100.0); }},
      {"odom.twist", [](auto &m) { m.tracking.odometry.twist.twist.angular.z = 0.01; }},
      {"odom.twist_covariance_one_bit", [](auto &m) { m.tracking.odometry.twist.covariance[35] = std::nextafter(m.tracking.odometry.twist.covariance[35], 100.0); }}
    };
    for (const auto &change : changes) {
      Accepted changed = message;
      change.second(changed);
      expect(wc_slam::accepted_fingerprint(changed) != baseline,
             "fingerprint missed field/bit change: " + change.first);
    }
    Accepted negative_zero = message;
    negative_zero.tracking.odometry.twist.twist.linear.x = -0.0;
    expect(wc_slam::accepted_fingerprint(negative_zero) != baseline,
           "fingerprint normalized exact floating-point sign bits");
    std::cout << "wc_slam_fingerprint: PASS (SYNTHETIC software identity; 256 allocator patterns, "
              << changes.size() << " field mutations and signed-zero bits)" << std::endl;
  } catch (const std::exception &error) {
    std::cerr << "wc_slam_fingerprint: FAIL: " << error.what() << std::endl;
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
