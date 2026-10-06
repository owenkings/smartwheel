#include "wc_slam/graph_engine.hpp"
#include "wc_slam/io.hpp"
#include "wc_slam/accepted_fingerprint.hpp"

#include <rtabmap/core/Version.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <wc_interfaces/msg/accepted_bundle.hpp>
#include <wc_interfaces/msg/graph_snapshot.hpp>

#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>

namespace wc_slam {
namespace fs = std::filesystem;
using Accepted = wc_interfaces::msg::AcceptedBundle;
using SnapshotMessage = wc_interfaces::msg::GraphSnapshot;

namespace {
void require(bool condition, const std::string &reason) {
  if (!condition) throw std::runtime_error(reason);
}
int64_t nanos(const builtin_interfaces::msg::Time &stamp) {
  require(stamp.sec >= 0 && stamp.nanosec < 1000000000u, "noncanonical or negative ROS time");
  return static_cast<int64_t>(stamp.sec) * 1000000000ll + stamp.nanosec;
}
rtabmap::Transform transform(double x, double y, double z, double qx, double qy, double qz, double qw) {
  const std::array<double, 7> values{x, y, z, qx, qy, qz, qw};
  for (double value : values) require(std::isfinite(value), "nonfinite pose or sensor transform");
  require(std::abs(qx*qx + qy*qy + qz*qz + qw*qw - 1.0) <= 1e-5,
          "unnormalized transform quaternion");
  return rtabmap::Transform(x, y, z, qx, qy, qz, qw);
}
rtabmap::Transform transform(const geometry_msgs::msg::Transform &value) {
  return transform(value.translation.x, value.translation.y, value.translation.z,
                   value.rotation.x, value.rotation.y, value.rotation.z, value.rotation.w);
}
rtabmap::Transform transform(const geometry_msgs::msg::Pose &value) {
  return transform(value.position.x, value.position.y, value.position.z,
                   value.orientation.x, value.orientation.y, value.orientation.z, value.orientation.w);
}
geometry_msgs::msg::Pose pose_message(const rtabmap::Transform &value) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = value.x(); pose.position.y = value.y(); pose.position.z = value.z();
  const auto q = value.getQuaterniond();
  pose.orientation.x = q.x(); pose.orientation.y = q.y(); pose.orientation.z = q.z(); pose.orientation.w = q.w();
  return pose;
}
geometry_msgs::msg::Transform transform_message(const rtabmap::Transform &value) {
  geometry_msgs::msg::Transform result;
  const auto pose = pose_message(value);
  result.translation.x = pose.position.x; result.translation.y = pose.position.y; result.translation.z = pose.position.z;
  result.rotation = pose.orientation;
  return result;
}
std::string matrix_json(const rtabmap::Transform &value) {
  std::ostringstream output;
  output << std::setprecision(17) << '[';
  for (int row = 0; row < 4; ++row) {
    if (row) output << ',';
    output << '[';
    for (int column = 0; column < 4; ++column) {
      if (column) output << ',';
      output << (row == 3 ? (column == 3 ? 1.0 : 0.0) : static_cast<double>(value(row, column)));
    }
    output << ']';
  }
  output << ']';
  return output.str();
}
bool positive_covariance(const cv::Mat &matrix) {
  if (!cv::checkRange(matrix) || cv::norm(matrix - matrix.t()) > 1e-10) return false;
  cv::Mat eigenvalues;
  return cv::eigen(matrix, eigenvalues) && eigenvalues.at<double>(5) > 0;
}
}  // namespace

struct RawRecord {
  AcceptedFingerprint accepted_digest{};
  std::string odom_epoch;
  std::string left_key, right_key, relative_file, sha256, covariance_source;
  int64_t stamp_ns;
  rtabmap::Transform left_origin, right_origin, odometry;
};

class GraphNode : public rclcpp::Node {
 public:
  explicit GraphNode(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("wc_rtabmap_graph", node_options) {
    session_root_ = safe_absolute(declare_parameter<std::string>("session_root", ""));
    const auto run_root = safe_absolute(declare_parameter<std::string>("run_root", ""));
    session_id_ = declare_parameter<std::string>("session_id", "");
    graph_epoch_ = declare_parameter<std::string>("graph_epoch", "");
    source_mode_ = declare_parameter<std::string>("source_mode", "");
    time_model_id_ = declare_parameter<std::string>("time_model_id", "");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    rig_frame_ = declare_parameter<std::string>("rig_frame", "rig_link");
    require(!session_id_.empty() && !graph_epoch_.empty() && !time_model_id_.empty(),
            "session_id, graph_epoch and time_model_id must be explicit");
    require(source_mode_ == "real" || source_mode_ == "synthetic", "source_mode must be real or synthetic");
    require(!map_frame_.empty() && !odom_frame_.empty() && !rig_frame_.empty() &&
            map_frame_ != odom_frame_ && odom_frame_ != rig_frame_ && map_frame_ != rig_frame_, "invalid frame ownership configuration");
    max_points_ = declare_parameter<int>("max_scan_points", 200000);
    require(max_points_ >= 20, "max_scan_points must be at least 20");
    noise_model_ = declare_parameter<std::string>("graph_edge_noise_model", "none");
    noise_diagonal_ = declare_parameter<std::vector<double>>("graph_edge_noise_diagonal", std::vector<double>{});
    require(noise_model_ == "none" || noise_model_ == "diagonal_assumption", "unsupported graph edge noise model");
    if (noise_model_ == "diagonal_assumption") {
      require(noise_diagonal_.size() == 6, "graph noise model requires six explicitly supplied variances");
      for (double value : noise_diagonal_) require(std::isfinite(value) && value > 0, "graph noise variance must be positive and finite");
    }
    EngineOptions options;
    options.optimizer_strategy = declare_parameter<int>("optimizer_strategy", options.optimizer_strategy);
    options.stm_size = declare_parameter<int>("stm_size", options.stm_size);
    options.max_nodes = declare_parameter<int>("max_nodes", options.max_nodes);
    options.icp_voxel_size = declare_parameter<double>("icp_voxel_size", options.icp_voxel_size);
    options.icp_max_correspondence = declare_parameter<double>("icp_max_correspondence", options.icp_max_correspondence);
    options.icp_correspondence_ratio = declare_parameter<double>("icp_correspondence_ratio", options.icp_correspondence_ratio);
    options.icp_max_translation = declare_parameter<double>("icp_max_translation", options.icp_max_translation);
    options.icp_max_rotation = declare_parameter<double>("icp_max_rotation", options.icp_max_rotation);
    options.proximity_radius = declare_parameter<double>("proximity_radius", options.proximity_radius);
    options.proximity_angle_degrees = declare_parameter<double>("proximity_angle_degrees", options.proximity_angle_degrees);

    fs::create_directories(session_root_);
    ownership_ = std::make_unique<ProcessLock>(run_root / "rtabmap_graph.lock");
    database_ = safe_absolute(session_root_ / "slam" / "rtabmap.db");
    require(!fs::exists(database_), "database already exists; start a new session, never overwrite old maps");
    fs::create_directories(database_.parent_path());
    fs::create_directories(safe_absolute(session_root_ / "graph" / "revisions"));
    engine_ = std::make_unique<GraphEngine>(database_.string(), options);
    auto durable = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    graph_publisher_ = create_publisher<SnapshotMessage>("/wc_mapping/graph/snapshot", durable);
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/wc_mapping/graph/diagnostics", durable);
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    subscription_ = create_subscription<Accepted>("/wc_mapping/accepted", rclcpp::QoS(8).reliable(),
                         [this](const Accepted::SharedPtr message) { accepted(*message); });
    pause_service_ = create_service<std_srvs::srv::Trigger>("/wc_mapping/pause_graph",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
          std::lock_guard<std::mutex> guard(mutex_);
          paused_ = true;
          response->success = true;
          response->message = "Graph acceptance paused; this is not an actuator stop. Close snapshot or start a new session.";
          diagnostic("PAUSED", response->message, diagnostic_msgs::msg::DiagnosticStatus::WARN);
        });
    close_service_ = create_service<std_srvs::srv::Trigger>("/wc_mapping/close_graph_snapshot",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
          close_snapshot(*response);
        });
    diagnostic("READY", "New RTAB-Map graph opened; waiting for exact accepted dual bundles", diagnostic_msgs::msg::DiagnosticStatus::OK);
  }

 private:
  cv::Mat scan(const Accepted &message) {
    const auto &cloud = message.bundle.cloud;
    const uint64_t count = static_cast<uint64_t>(cloud.width) * cloud.height;
    require(!cloud.is_bigendian && count >= 20 && count <= static_cast<uint64_t>(max_points_), "invalid scan endian/count");
    require(cloud.point_step >= 12 && cloud.row_step >= cloud.point_step * static_cast<uint64_t>(cloud.width) &&
            cloud.data.size() == cloud.row_step * static_cast<uint64_t>(cloud.height), "invalid PointCloud2 byte layout");
    std::array<uint32_t, 3> offsets{};
    for (size_t index = 0; index < 3; ++index) {
      const std::string name(1, "xyz"[index]);
      auto found = std::find_if(cloud.fields.begin(), cloud.fields.end(), [&](const auto &field) { return field.name == name; });
      require(found != cloud.fields.end() && found->count == 1 && found->datatype == sensor_msgs::msg::PointField::FLOAT32 &&
              static_cast<uint64_t>(found->offset) + sizeof(float) <= cloud.point_step, "scan requires real FLOAT32 xyz fields");
      offsets[index] = found->offset;
    }
    require(message.bundle.source_codes.size() == count, "source provenance length differs from point count");
    uint64_t left = 0, right = 0;
    cv::Mat result(1, static_cast<int>(count), CV_32FC3);
    for (uint64_t index = 0; index < count; ++index) {
      const auto source = message.bundle.source_codes[index];
      require(source >= 1 && source <= 3, "invalid point source mask");
      left += (source & 1) != 0; right += (source & 2) != 0;
      const size_t offset = (index / cloud.width) * cloud.row_step + (index % cloud.width) * cloud.point_step;
      auto &point = result.at<cv::Vec3f>(0, static_cast<int>(index));
      for (size_t axis = 0; axis < 3; ++axis) {
        std::memcpy(&point[axis], cloud.data.data() + offset + offsets[axis], sizeof(float));
        require(std::isfinite(point[axis]), "accepted cloud contains invalid depth");
      }
    }
    require(left && right && message.tracking.left_solver_input_count && message.tracking.right_solver_input_count,
            "both sensors must actually participate in the common registration");
    return result;
  }

  void accepted(const Accepted &message) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (failed_) return;
    try {
      const auto &bundle = message.bundle;
      const auto &tracking = message.tracking;
      // Lookup does not narrow an unchecked uint64 ID. Exact retries remain
      // acknowledgeable after an intentional pause/close, with no DB access.
      if (bundle.bundle_id > 0 && bundle.bundle_id <= INT_MAX) {
        const auto prior = records_.find(static_cast<int>(bundle.bundle_id));
        if (prior != records_.end()) {
          require(accepted_fingerprint(message) == prior->second.accepted_digest,
                  "processed bundle ID reused with different AcceptedBundle content");
          builtin_interfaces::msg::Time latest_stamp;
          latest_stamp.sec = static_cast<int32_t>(last_stamp_ / 1000000000ll);
          latest_stamp.nanosec = static_cast<uint32_t>(last_stamp_ % 1000000000ll);
          graph_publisher_->publish(graph_message(latest_stamp));
          return;  // ACK only: never process, write a file/DB, or publish TF.
        }
      }
      if (paused_ || engine_->closed()) return;
      require(bundle.session_id == session_id_ && tracking.session_id == session_id_, "session mismatch");
      require(bundle.source_mode == source_mode_ && bundle.sensor_mode == "dual", "source mode is not the configured dual session");
      require(tracking.accepted && bundle.dual_valid && bundle.calibration_valid && bundle.time_valid &&
              bundle.rejection_reasons.empty() && tracking.rejection_reasons.empty(), "bundle/tracking acceptance flags disagree");
      require(bundle.temporal_error_bound_valid && std::isfinite(bundle.temporal_error_bound_m) && bundle.temporal_error_bound_m >= 0,
              "temporal error budget is unavailable");
      require(bundle.bundle_id > 0 && bundle.bundle_id <= INT_MAX && tracking.bundle_id == bundle.bundle_id,
              "bundle ID invalid, ambiguous or exceeds RTAB-Map int32 range");
      const int id = static_cast<int>(bundle.bundle_id);
      require(records_.empty() || id > records_.rbegin()->first, "duplicate/nonmonotonic bundle ID");
      const int64_t stamp = nanos(bundle.header.stamp);
      require(nanos(tracking.header.stamp) == stamp && nanos(tracking.odometry.header.stamp) == stamp &&
              nanos(bundle.cloud.header.stamp) == stamp, "exact bundle/assessment/odom/cloud timestamp mismatch");
      require(records_.empty() || stamp > last_stamp_, "timestamp jump or repeated reference time");
      require(bundle.header.frame_id == rig_frame_ && tracking.header.frame_id == rig_frame_ && bundle.cloud.header.frame_id == rig_frame_ &&
              tracking.odometry.header.frame_id == odom_frame_ && tracking.odometry.child_frame_id == rig_frame_, "coordinate frame ownership mismatch");
      require(!tracking.odom_epoch.empty() && (odom_epoch_.empty() || odom_epoch_ == tracking.odom_epoch),
              "odometry epoch changed; explicit new graph/continuity validation is required");
      require(!bundle.left_raw_key.empty() && !bundle.right_raw_key.empty() && bundle.left_raw_key != bundle.right_raw_key &&
              !used_raw_keys_.count(bundle.left_raw_key) && !used_raw_keys_.count(bundle.right_raw_key), "raw frame key missing/reused");
      require(bundle.left_retained_count > 0 && bundle.right_retained_count > 0, "one sensor has no retained valid observations");
      require(!bundle.calibration_id.empty() && !bundle.left_clock_model_id.empty() && !bundle.right_clock_model_id.empty(),
              "calibration/time model identity missing");
      if (!calibration_id_.empty())
        require(calibration_id_ == bundle.calibration_id && left_clock_ == bundle.left_clock_model_id && right_clock_ == bundle.right_clock_model_id,
                "calibration/time model changed within graph epoch");
      RawRecord record;
      record.odom_epoch = tracking.odom_epoch;
      record.left_key = bundle.left_raw_key; record.right_key = bundle.right_raw_key;
      record.left_origin = transform(bundle.t_ref_left_sensor); record.right_origin = transform(bundle.t_ref_right_sensor);
      record.odometry = transform(tracking.odometry.pose.pose);
      record.stamp_ns = stamp;
      record.relative_file = "raw_observations/" + std::to_string(id) + ".json";
      record.sha256 = file_sha256(session_root_ / record.relative_file);
      cv::Mat covariance(6, 6, CV_64FC1);
      std::copy(tracking.odometry.pose.covariance.begin(), tracking.odometry.pose.covariance.end(), covariance.ptr<double>());
      record.covariance_source = "upstream_odometry_covariance";
      if (!positive_covariance(covariance)) {
        require(noise_model_ == "diagonal_assumption", "upstream covariance unavailable; no explicit graph noise assumption configured");
        covariance = cv::Mat::zeros(6, 6, CV_64FC1);
        for (int index = 0; index < 6; ++index) covariance.at<double>(index, index) = noise_diagonal_[index];
        record.covariance_source = "configured_graph_edge_diagonal_assumption_not_measured_covariance";
      }
      const cv::Mat xyz = scan(message);
      record.accepted_digest = accepted_fingerprint(message);
      auto current = engine_->process(id, static_cast<double>(stamp) / 1e9, xyz, record.odometry, covariance);
      records_.emplace(id, record);
      used_raw_keys_.insert(record.left_key); used_raw_keys_.insert(record.right_key);
      odom_epoch_ = tracking.odom_epoch;
      calibration_id_ = bundle.calibration_id; left_clock_ = bundle.left_clock_model_id; right_clock_ = bundle.right_clock_model_id;
      last_stamp_ = stamp;
      ++revision_;
      last_engine_snapshot_ = std::move(current);
      auto graph = graph_message(bundle.header.stamp);
      std::ostringstream name;
      name << 'r' << std::setw(20) << std::setfill('0') << revision_ << ".json";
      last_revision_path_ = session_root_ / "graph" / "revisions" / name.str();
      atomic_json_file(last_revision_path_, graph_json(graph), true);
      // The immutable revision is durable before its ROS acknowledgement.
      // Live readers select that exact revision; final save requires the close
      // barrier. No mutable latest pointer participates in either operation.
      graph_publisher_->publish(graph);
      geometry_msgs::msg::TransformStamped correction;
      correction.header.stamp = bundle.header.stamp;
      correction.header.frame_id = map_frame_; correction.child_frame_id = odom_frame_;
      correction.transform = transform_message(last_engine_snapshot_.map_to_odom);
      broadcaster_->sendTransform(correction);
      diagnostic("MAPPING_DUAL", "RTAB-Map graph revision committed with complete raw-frame associations", diagnostic_msgs::msg::DiagnosticStatus::OK);
    } catch (const std::exception &error) {
      failed_ = paused_ = true;
      diagnostic("PAUSED_INVALID", error.what(), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
      RCLCPP_ERROR(get_logger(), "Graph acceptance latched: %s", error.what());
    }
  }

  std::string index_hash() const {
    std::ostringstream index;
    for (const auto &entry : records_) index << entry.first << '\t' << entry.second.sha256 << '\n';
    return text_sha256(index.str());
  }

  SnapshotMessage graph_message(const builtin_interfaces::msg::Time &stamp) const {
    require(last_engine_snapshot_.poses.size() == records_.size(), "graph/raw index is incomplete");
    SnapshotMessage graph;
    graph.header.frame_id = map_frame_; graph.header.stamp = stamp;
    graph.session_id = session_id_; graph.graph_epoch = graph_epoch_; graph.source_mode = source_mode_;
    graph.calibration_id = calibration_id_; graph.left_clock_model_id = left_clock_; graph.right_clock_model_id = right_clock_;
    graph.revision = revision_; graph.raw_index_complete = true; graph.raw_index_hash = index_hash();
    for (const auto &pose : last_engine_snapshot_.poses) {
      const auto &record = records_.at(pose.first);
      wc_interfaces::msg::GraphNode node;
      node.node_id = pose.first; node.bundle_id = static_cast<uint64_t>(pose.first); node.odom_epoch = record.odom_epoch;
      node.optimized_pose = pose_message(pose.second);
      node.raw_keys = {record.left_key, record.right_key};
      node.t_node_sensor = {transform_message(record.left_origin), transform_message(record.right_origin)};
      graph.nodes.push_back(node);
    }
    for (const auto &entry : last_engine_snapshot_.edges) {
      wc_interfaces::msg::GraphEdge edge;
      edge.from_id = entry.second.from(); edge.to_id = entry.second.to();
      edge.type = GraphEngine::edge_type(entry.second.type()); edge.synthetic = source_mode_ == "synthetic";
      graph.edges.push_back(edge);
    }
    return graph;
  }

  std::string graph_json(const SnapshotMessage &graph) const {
    std::ostringstream output;
    output << std::setprecision(17) << "{\"schema_version\":1,\"session_id\":" << json_string(session_id_)
           << ",\"source_mode\":" << json_string(source_mode_) << ",\"sensor_mode\":\"dual\",\"graph_epoch\":" << json_string(graph_epoch_)
           << ",\"revision\":" << revision_ << ",\"stamp_ns\":" << last_stamp_ << ",\"frame_id\":" << json_string(map_frame_)
           << ",\"calibration_id\":" << json_string(calibration_id_) << ",\"time_model_id\":" << json_string(time_model_id_)
           << ",\"left_clock_model_id\":" << json_string(left_clock_) << ",\"right_clock_model_id\":" << json_string(right_clock_)
           << ",\"raw_index_complete\":true,\"raw_index_hash\":" << json_string(graph.raw_index_hash)
           << ",\"rtabmap_version\":" << json_string(RTABMAP_VERSION) << ",\"database_closed\":false,\"nodes\":[";
    bool first = true;
    for (const auto &entry : records_) {
      if (!first) output << ','; first = false;
      const auto &record = entry.second;
      output << "{\"node_id\":" << entry.first << ",\"bundle_id\":" << entry.first << ",\"odom_epoch\":" << json_string(record.odom_epoch)
             << ",\"stamp_ns\":" << record.stamp_ns << ",\"T_map_node\":" << matrix_json(last_engine_snapshot_.poses.at(entry.first))
             << ",\"T_odom_node\":" << matrix_json(record.odometry) << ",\"raw_keys\":[" << json_string(record.left_key) << ',' << json_string(record.right_key)
             << "],\"t_node_sensor\":[" << matrix_json(record.left_origin) << ',' << matrix_json(record.right_origin)
             << "],\"raw_file\":" << json_string(record.relative_file) << ",\"raw_file_sha256\":" << json_string(record.sha256)
             << ",\"covariance_source\":" << json_string(record.covariance_source) << '}';
    }
    output << "],\"edges\":["; first = true;
    for (const auto &entry : last_engine_snapshot_.edges) {
      if (!first) output << ','; first = false;
      output << "{\"from_id\":" << entry.second.from() << ",\"to_id\":" << entry.second.to()
             << ",\"type\":" << json_string(GraphEngine::edge_type(entry.second.type()))
             << ",\"type_code\":" << static_cast<int>(entry.second.type())
             << ",\"synthetic\":" << (source_mode_ == "synthetic" ? "true" : "false") << '}';
    }
    output << "],\"parameters\":{"; first = true;
    for (const auto &parameter : engine_->parameters()) {
      if (!first) output << ','; first = false;
      output << json_string(parameter.first) << ':' << json_string(parameter.second);
    }
    output << "},\"graph_edge_noise_model\":" << json_string(noise_model_) << ",\"graph_edge_noise_diagonal\":[";
    for (size_t index = 0; index < noise_diagonal_.size(); ++index) { if (index) output << ','; output << noise_diagonal_[index]; }
    output << "],\"loop_closure_id\":" << last_engine_snapshot_.loop_closure_id
           << ",\"proximity_detection_id\":" << last_engine_snapshot_.proximity_detection_id << ",\"statistics\":{";
    first = true;
    for (const auto &entry : last_engine_snapshot_.statistics) {
      if (!first) output << ','; first = false;
      output << json_string(entry.first) << ':';
      if (std::isfinite(entry.second)) output << entry.second; else output << "null";
    }
    output << "}}\n";
    return output.str();
  }

  void close_snapshot(std_srvs::srv::Trigger::Response &response) {
    std::lock_guard<std::mutex> guard(mutex_);
    paused_ = true;
    try {
      require(!failed_ && revision_ > 0, "no complete graph revision, or graph is latched invalid");
      const auto barrier_path = session_root_ / "graph" / "closed_snapshot.json";
      require(!fs::exists(barrier_path), "graph was already closed; use its existing immutable snapshot barrier");
      engine_->close();
      for (const auto &record : records_)
        require(file_sha256(session_root_ / record.second.relative_file) == record.second.sha256,
                "raw observations changed since graph acceptance");
      const std::string relative_revision = fs::relative(last_revision_path_, session_root_).generic_string();
      std::ostringstream marker;
      marker << "{\"schema_version\":1,\"state\":\"CLOSED_FOR_SNAPSHOT\",\"session_id\":" << json_string(session_id_)
             << ",\"graph_epoch\":" << json_string(graph_epoch_) << ",\"revision\":" << revision_
             << ",\"graph_file\":" << json_string(relative_revision) << ",\"graph_sha256\":" << json_string(file_sha256(last_revision_path_))
             << ",\"database_file\":\"slam/rtabmap.db\",\"database_sha256\":" << json_string(file_sha256(database_))
             << ",\"raw_index_hash\":" << json_string(index_hash()) << "}\n";
      atomic_json_file(barrier_path, marker.str(), true);
      response.success = true; response.message = barrier_path.string();
      diagnostic("CLOSED_FOR_SNAPSHOT", "DB closed and hashes frozen; offline rebuilding can now create a complete map snapshot", diagnostic_msgs::msg::DiagnosticStatus::OK);
    } catch (const std::exception &error) {
      response.success = false; response.message = error.what();
      diagnostic("SNAPSHOT_BLOCKED", error.what(), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    }
  }

  void diagnostic(const std::string &state, const std::string &message, uint8_t level) {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level; status.name = "wc_slam/rtabmap_graph"; status.hardware_id = "software_graph_only"; status.message = message;
    const std::map<std::string, std::string> values{{"state", state}, {"session_id", session_id_}, {"source_mode", source_mode_},
      {"sensor_mode", "dual"}, {"graph_epoch", graph_epoch_}, {"graph_revision", std::to_string(revision_)},
      {"database", database_.string()}, {"raw_node_count", std::to_string(records_.size())},
      {"rtabmap_version", RTABMAP_VERSION}, {"navigation_validated", "false"}};
    for (const auto &entry : values) { diagnostic_msgs::msg::KeyValue value; value.key = entry.first; value.value = entry.second; status.values.push_back(value); }
    array.status.push_back(status); diagnostics_->publish(array);
  }

  std::mutex mutex_;
  std::unique_ptr<ProcessLock> ownership_;
  std::unique_ptr<GraphEngine> engine_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  fs::path session_root_, database_, last_revision_path_;
  std::string session_id_, graph_epoch_, source_mode_, time_model_id_, map_frame_, odom_frame_, rig_frame_;
  std::string odom_epoch_, calibration_id_, left_clock_, right_clock_, noise_model_;
  std::vector<double> noise_diagonal_;
  int max_points_ = 0;
  int64_t last_stamp_ = 0;
  uint64_t revision_ = 0;
  bool paused_ = false, failed_ = false;
  std::map<int, RawRecord> records_;
  std::set<std::string> used_raw_keys_;
  EngineSnapshot last_engine_snapshot_;
  rclcpp::Publisher<SnapshotMessage>::SharedPtr graph_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Subscription<Accepted>::SharedPtr subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_, close_service_;
};
}  // namespace wc_slam

#ifndef WC_SLAM_GRAPH_NODE_NO_MAIN
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<wc_slam::GraphNode>();
    rclcpp::spin(node);  // Single-threaded graph/database owner.
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("wc_rtabmap_graph"), "%s", error.what());
    result = 2;
  }
  rclcpp::shutdown();
  return result;
}
#endif
