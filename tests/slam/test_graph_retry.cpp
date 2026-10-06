// Exercise the real graph subscription callback, serialization, RTAB-Map, TF,
// and snapshot I/O in a private synthetic session with remapped ROS topics.
// The production node stays in one translation unit; only its main is omitted.
#define WC_SLAM_GRAPH_NODE_NO_MAIN
#include "../../src/wc_slam/src/graph_node.cpp"

#include <tf2_msgs/msg/tf_message.hpp>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <thread>

namespace {
using Accepted = wc_interfaces::msg::AcceptedBundle;
using Snapshot = wc_interfaces::msg::GraphSnapshot;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

void expect(bool condition, const std::string &reason) {
  if (!condition) throw std::runtime_error(reason);
}

Accepted accepted_fixture(int id) {
  Accepted message;
  auto &b = message.bundle;
  auto &t = message.tracking;
  b.header.frame_id = "rig_link";
  b.header.stamp.sec = 100 + id;
  b.header.stamp.nanosec = 137;
  b.session_id = "synthetic_retry_session";
  b.bundle_id = id;
  b.source_mode = "synthetic";
  b.sensor_mode = "dual";
  b.calibration_id = "synthetic_calibration_only";
  b.left_clock_model_id = "synthetic_left_clock";
  b.right_clock_model_id = "synthetic_right_clock";
  b.left_raw_key = "synthetic:left:" + std::to_string(id);
  b.right_raw_key = "synthetic:right:" + std::to_string(id);
  b.left_time_ns = b.right_time_ns = static_cast<int64_t>(b.header.stamp.sec) * 1000000000ll + 137;
  b.t_ref_left_sensor.rotation.w = b.t_ref_right_sensor.rotation.w = 1.0;
  b.t_ref_left_sensor.translation.y = 0.20;
  b.t_ref_right_sensor.translation.y = -0.20;
  b.t_ref_rig_at_left_time.rotation.w = b.t_ref_rig_at_right_time.rotation.w = 1.0;
  b.compensation_mode = "synthetic_truth";
  b.temporal_error_bound_valid = true;
  b.temporal_error_bound_m = 0.001;
  b.calibration_valid = b.time_valid = b.dual_valid = true;
  auto &cloud = b.cloud;
  cloud.header = b.header;
  cloud.height = 1;
  cloud.width = 25 * 25 * 3;
  cloud.point_step = 12;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  for (uint32_t axis = 0; axis < 3; ++axis) {
    sensor_msgs::msg::PointField field;
    field.name = std::string(1, "xyz"[axis]); field.offset = axis * 4;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32; field.count = 1;
    cloud.fields.push_back(field);
  }
  cloud.data.resize(cloud.row_step);
  const float x = (id - 1) * 0.12f;
  size_t index = 0;
  auto append = [&](float px, float py, float pz) {
    const float point[]{px, py, pz};
    std::memcpy(cloud.data.data() + index * 12, point, sizeof(point));
    const uint8_t source = index % 2 ? 2 : 1;
    b.source_codes.push_back(source);
    if (source == 1) ++b.left_retained_count; else ++b.right_retained_count;
    ++index;
  };
  for (int row = 0; row < 25; ++row) {
    for (int column = 0; column < 25; ++column) {
      const float a = -2.0f + row / 6.0f;
      const float c = -1.5f + column / 8.0f;
      append(4.0f - x, a, c + 1.0f);
      append(a - x, 3.0f, c + 1.0f);
      append(a - x, c, -0.5f);
    }
  }
  t.header = b.header;
  t.session_id = b.session_id; t.bundle_id = b.bundle_id;
  t.odom_epoch = "synthetic_odom_epoch";
  t.tracking_state = "MAPPING_DUAL"; t.accepted = true;
  t.left_solver_input_count = b.left_retained_count;
  t.right_solver_input_count = b.right_retained_count;
  t.odometry.header = b.header; t.odometry.header.frame_id = "odom";
  t.odometry.child_frame_id = "rig_link";
  t.odometry.pose.pose.orientation.w = 1.0;
  t.odometry.pose.pose.position.x = x;
  // Explicit synthetic fixture noise, not a measured device covariance.
  for (size_t i = 0; i < 6; ++i) t.odometry.pose.covariance[i * 6 + i] = 0.01;
  return message;
}

using FileState = std::map<std::string, std::pair<std::string, std::filesystem::file_time_type>>;
FileState files(const std::filesystem::path &root) {
  FileState result;
  for (const auto &entry : std::filesystem::recursive_directory_iterator(root)) {
    if (entry.is_regular_file())
      result.emplace(std::filesystem::relative(entry.path(), root).generic_string(),
          std::make_pair(wc_slam::file_sha256(entry.path()), entry.last_write_time()));
  }
  return result;
}
}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  char template_path[] = "/tmp/wc_slam_retry_XXXXXX";
  char *created = ::mkdtemp(template_path);
  if (!created) { rclcpp::shutdown(); return 2; }
  const std::filesystem::path root(created);
  int result = 0;
  try {
    std::filesystem::create_directories(root / "session" / "raw_observations");
    for (int id = 1; id <= 3; ++id) {
      // This test only verifies graph archive identity/hashing. A minimal
      // synthetic file is not an assembled/reviewed map observation archive.
      wc_slam::atomic_json_file(root / "session" / "raw_observations" / (std::to_string(id) + ".json"),
          "{\"source_mode\":\"synthetic\",\"test_only\":true,\"bundle_id\":" + std::to_string(id) + "}\n", true);
    }
    const std::string prefix = "/wc_slam_retry_" + std::to_string(::getpid());
    const std::vector<std::string> original_topics{
      "/wc_mapping/accepted", "/wc_mapping/graph/snapshot", "/wc_mapping/graph/diagnostics",
      "/wc_mapping/pause_graph", "/wc_mapping/close_graph_snapshot", "/tf", "/tf_static"};
    std::vector<std::string> arguments{"--ros-args"};
    for (size_t i = 0; i < original_topics.size(); ++i) {
      arguments.push_back("-r");
      arguments.push_back(original_topics[i] + ":=" + prefix + "/t" + std::to_string(i));
    }
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.arguments(arguments);
    options.parameter_overrides({
      rclcpp::Parameter("session_root", (root / "session").string()),
      rclcpp::Parameter("run_root", (root / "run").string()),
      rclcpp::Parameter("session_id", "synthetic_retry_session"),
      rclcpp::Parameter("graph_epoch", "synthetic_retry_graph_epoch"),
      rclcpp::Parameter("source_mode", "synthetic"),
      rclcpp::Parameter("time_model_id", "synthetic_time_model"),
      rclcpp::Parameter("max_nodes", 2)});
    auto graph = std::make_shared<wc_slam::GraphNode>(options);
    auto observer = std::make_shared<rclcpp::Node>("wc_graph_retry_observer_" + std::to_string(::getpid()));
    auto publisher = observer->create_publisher<Accepted>(prefix + "/t0", rclcpp::QoS(8).reliable());
    size_t graph_count = 0, tf_count = 0;
    Snapshot latest;
    std::string graph_state;
    auto snapshot_subscription = observer->create_subscription<Snapshot>(prefix + "/t1",
        rclcpp::QoS(100).reliable().transient_local(), [&](const Snapshot::SharedPtr message) {
          latest = *message; ++graph_count;
        });
    auto diagnostics_subscription = observer->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(prefix + "/t2",
        rclcpp::QoS(100).reliable().transient_local(), [&](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
          for (const auto &status : message->status) for (const auto &value : status.values)
            if (value.key == "state") graph_state = value.value;
        });
    auto tf_subscription = observer->create_subscription<tf2_msgs::msg::TFMessage>(prefix + "/t5",
        rclcpp::QoS(100).reliable(), [&](const tf2_msgs::msg::TFMessage::SharedPtr message) {
          for (const auto &tf : message->transforms)
            if (tf.header.frame_id == "map" && tf.child_frame_id == "odom") ++tf_count;
        });
    auto close = observer->create_client<std_srvs::srv::Trigger>(prefix + "/t4");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(graph); executor.add_node(observer);
    auto pump = [&](std::chrono::milliseconds duration) {
      const auto end = Clock::now() + duration;
      do { executor.spin_some(); std::this_thread::sleep_for(2ms); } while (Clock::now() < end);
    };
    auto until = [&](const std::function<bool()> &predicate, const std::string &failure) {
      const auto deadline = Clock::now() + 20s;
      while (!predicate() && Clock::now() < deadline) pump(5ms);
      expect(predicate(), failure + "; latest diagnostic=" + graph_state);
    };
    until([&] { return publisher->get_subscription_count() == 1 &&
        snapshot_subscription->get_publisher_count() == 1 &&
        tf_subscription->get_publisher_count() == 1 && close->service_is_ready(); }, "private ROS endpoints did not discover");
    const auto first = accepted_fixture(1);
    const auto second = accepted_fixture(2);
    publisher->publish(first);
    until([&] { return latest.revision == 1 && tf_count == 1; }, "first bundle was not committed");
    const Snapshot revision_one = latest;
    const auto first_graph_files = files(root / "session" / "graph");
    size_t before = graph_count;
    publisher->publish(first);
    until([&] { return graph_count > before; }, "identical first retry was not acknowledged");
    pump(100ms);
    expect(latest == revision_one && tf_count == 1, "first retry changed graph or republished TF");
    expect(files(root / "session" / "graph") == first_graph_files, "first retry rewrote graph files");

    publisher->publish(second);
    until([&] { return latest.revision == 2 && tf_count == 2; }, "second bundle was not committed");
    expect(latest.nodes.size() == 2 && latest.nodes[0].bundle_id == 1 && latest.nodes[1].bundle_id == 2,
           "graph lost or duplicated an accepted bundle");
    expect(latest.source_mode == "synthetic" && latest.session_id == first.bundle.session_id && latest.raw_index_complete,
           "graph ACK provenance is incomplete");
    const Snapshot revision_two = latest;
    const auto second_graph_files = files(root / "session" / "graph");
    // Already at max_nodes=2: retries must not pass through engine resource
    // checks, duplicate-ID validation, optimization, or database insertion.
    for (int i = 0; i < 8; ++i) {
      before = graph_count;
      publisher->publish(i % 2 ? second : first);
      until([&] { return graph_count > before; }, "historical retry was not acknowledged at max_nodes");
      expect(latest == revision_two, "historical retry acknowledged an old graph revision/stamp");
    }
    pump(100ms);
    expect(tf_count == 2 && graph_state == "MAPPING_DUAL", "historical retry changed TF or latched a duplicate-ID error");
    expect(files(root / "session" / "graph") == second_graph_files, "historical retry wrote a graph revision/index");

    auto response = close->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    until([&] { return response.wait_for(0s) == std::future_status::ready; }, "close service timed out");
    expect(response.get()->success, "graph close barrier failed");
    pump(100ms);
    const auto closed_files = files(root / "session");
    expect(closed_files.count("slam/rtabmap.db") && closed_files.count("graph/closed_snapshot.json"),
           "closed database and immutable barrier are missing");
    before = graph_count;
    publisher->publish(first);
    until([&] { return graph_count > before; }, "closed graph did not ACK identical historical bundle");
    pump(100ms);
    expect(latest == revision_two && tf_count == 2, "closed retry mutated graph or TF");
    expect(files(root / "session") == closed_files, "closed retry changed DB/raw/revision/barrier bytes or mtimes");

    // Change only one point-cloud bit while retaining ID, timing, provenance,
    // all counts, and flags. This must latch even after a successful close.
    Accepted tampered = first;
    tampered.bundle.cloud.data[13] ^= 1;
    before = graph_count;
    publisher->publish(tampered);
    until([&] { return graph_state == "PAUSED_INVALID"; }, "different content for processed ID was not latched");
    publisher->publish(first);
    publisher->publish(accepted_fixture(3));
    pump(300ms);
    expect(graph_count == before && latest == revision_two && tf_count == 2,
           "latched graph acknowledged or processed a later bundle");
    expect(files(root / "session") == closed_files, "conflict/latch changed frozen DB or archive");
    executor.remove_node(graph); executor.remove_node(observer);
    graph.reset(); observer.reset();
    std::filesystem::remove_all(root);  // Only this executable's mkdtemp root.
    std::cout << "wc_slam_retry: PASS (SYNTHETIC ROS/native graph; exact ACK, old-ID latest revision, "
                 "max_nodes bound, two TF only, closed DB/archives unchanged, conflict latched)" << std::endl;
  } catch (const std::exception &error) {
    std::cerr << "wc_slam_retry: FAIL: " << error.what() << "; evidence retained at " << root << std::endl;
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
