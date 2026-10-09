// A test observer: no publisher, transform broadcaster, service call or device.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/message_info.hpp>
#include <rclcpp/node_interfaces/node_graph_interface.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rmw/types.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
namespace {
using Clock = std::chrono::steady_clock;
std::int64_t unix_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}
std::string json(const std::string &text) {
  std::ostringstream out;
  out << '"';
  for (const unsigned char value : text) {
    if (value == '"' || value == '\\') out << '\\' << value;
    else if (value < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(value);
    else out << value;
  }
  out << '"';
  return out.str();
}
std::string gid_hex(const std::uint8_t *data, std::size_t size) {
  std::ostringstream out;
  bool nonzero = false;
  for (std::size_t i = 0; i < size; ++i) {
    nonzero = nonzero || data[i] != 0;
    out << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(data[i]);
  }
  return nonzero ? out.str() : std::string();
}
fs::path safe_absolute(const fs::path &path) {
  if (!path.is_absolute()) throw std::runtime_error("output must be an absolute path");
  fs::path current;
  for (const auto &part : path) {
    if (part == "..") throw std::runtime_error("parent traversal refused");
    current /= part;
    std::error_code error;
    const auto state = fs::symlink_status(current, error);
    if (fs::is_symlink(state)) throw std::runtime_error("symbolic links refused");
    if (error && error != std::errc::no_such_file_or_directory)
      throw std::runtime_error("cannot inspect output path");
  }
  return path.lexically_normal();
}
class Descriptor {
 public:
  explicit Descriptor(int value) : value_(value) {}
  ~Descriptor() { if (value_ >= 0) ::close(value_); }
  Descriptor(const Descriptor &) = delete;
  Descriptor &operator=(const Descriptor &) = delete;
  int get() const { return value_; }
 private:
  int value_;
};
void fail_errno(const std::string &operation) {
  throw std::runtime_error(operation + ": " + std::strerror(errno));
}
void immutable_report(const fs::path &path, const std::string &report) {
  const auto checked = safe_absolute(path);
  if (fs::exists(checked)) throw std::runtime_error("existing evidence will not be overwritten");
  fs::create_directories(checked.parent_path());
  safe_absolute(checked.parent_path());
  const auto temporary = checked.parent_path() / ("." + checked.filename().string() + "." + std::to_string(::getpid()) + ".tmp");
  const int raw = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW, 0600);
  if (raw < 0) fail_errno("create owned evidence temporary");
  Descriptor descriptor(raw);
  try {
    std::size_t offset = 0;
    while (offset < report.size()) {
      const auto written = ::write(descriptor.get(), report.data() + offset, report.size() - offset);
      if (written < 0 && errno == EINTR) continue;
      if (written <= 0) fail_errno("write evidence");
      offset += static_cast<std::size_t>(written);
    }
    if (::fsync(descriptor.get()) != 0) fail_errno("sync evidence");
    safe_absolute(checked);
    if (::link(temporary.c_str(), checked.c_str()) != 0) fail_errno("publish immutable evidence");
    if (::unlink(temporary.c_str()) != 0) fail_errno("unlink committed temporary");
    Descriptor directory(::open(checked.parent_path().c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW));
    if (directory.get() < 0 || ::fsync(directory.get()) != 0) fail_errno("sync evidence directory");
  } catch (...) {
    ::unlink(temporary.c_str());  // This invocation successfully created it.
    throw;
  }
}

struct Observation {
  std::int64_t stamp_ns;
  std::int64_t received_unix_ns;
  std::string gid;
  std::array<double, 7> transform;
};
struct Endpoint {
  std::string gid;
  std::string node;
  std::int64_t first_seen_unix_ns;
  std::int64_t last_seen_unix_ns;
};

class Probe : public rclcpp::Node {
 public:
  Probe() : Node("wc_tf_ownership_probe_" + std::to_string(::getpid()),
                  rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
            started_(Clock::now()), start_unix_ns_(unix_ns()) {
    if (!has_parameter("output")) declare_parameter<std::string>("output", "");
    if (!has_parameter("session_id")) declare_parameter<std::string>("session_id", "");
    if (!has_parameter("duration_s")) declare_parameter("duration_s", 100.0);
    if (!has_parameter("wheel_private_mode")) declare_parameter("wheel_private_mode", false);
    wheel_private_mode_ = get_parameter("wheel_private_mode").as_bool();
    if (wheel_private_mode_) topic_ = "/wc_mapping/icp_guess_tf";
    output_ = safe_absolute(get_parameter("output").as_string());
    session_id_ = get_parameter("session_id").as_string();
    const auto duration = get_parameter("duration_s");
    duration_s_ = duration.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
        static_cast<double>(duration.as_int()) : duration.as_double();
    if (session_id_.empty() || session_id_.size() > 128 || !std::isfinite(duration_s_) || duration_s_ < 1 || duration_s_ > 600)
      throw std::runtime_error("explicit session_id and duration_s in [1,600] required");
    if (fs::exists(output_)) throw std::runtime_error("output exists; choose a new evidence path");
    if (wheel_private_mode_) {
      observations_["wheel_odom->rig_link"] = {};
    } else {
      observations_["map->odom"] = {};
      observations_["odom->rig_link"] = {};
    }
    subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(topic_, rclcpp::QoS(100).best_effort(),
        [this](const tf2_msgs::msg::TFMessage::ConstSharedPtr message, const rclcpp::MessageInfo &info) {
          receive(*message, info);
        });
    // DDS discovery only. A TF topic publisher does not by itself prove which
    // transform it sent; that attribution uses the actual MessageInfo GID.
    discover();
    timer_ = create_wall_timer(std::chrono::milliseconds(250), [this] { discover(); });
  }
  bool finished() const {
    return std::chrono::duration<double>(Clock::now() - started_).count() >= duration_s_;
  }
  const fs::path &output() const { return output_; }
  void record_error(const std::string &error) {
    if (errors_.size() < 100) errors_.insert(error);
  }
  void discover() {
    try {
      std::vector<Endpoint> current;
      for (const auto &info : get_publishers_info_by_topic(topic_)) {
        // Actual target Humble exposes TopicEndpointInfo in node_graph_interface
        // and names this accessor endpoint_gid(), with 24 RMW bytes.
        const auto &data = info.endpoint_gid();
        const auto identity = gid_hex(data.data(), data.size());
        const auto node = info.node_namespace() == "/" ? "/" + info.node_name() : info.node_namespace() + "/" + info.node_name();
        const auto now = unix_ns();
        Endpoint endpoint{identity, node, now, now};
        const auto key = std::make_pair(identity, node);
        auto stored = endpoints_.find(key);
        if (stored == endpoints_.end()) endpoints_.emplace(key, endpoint);
        else stored->second.last_seen_unix_ns = now;
        current.push_back(endpoint);
      }
      current_endpoints_ = std::move(current);
      last_discovery_unix_ns_ = unix_ns();
    } catch (const std::exception &error) {
      record_error(std::string("DDS endpoint discovery: ") + error.what());
    }
  }
  std::string report(bool interrupted, int &exit_code) {
    discover();
    const auto end = unix_ns();
    bool missing = false;
    for (const auto &edge : observations_) {
      std::set<std::string> gids;
      for (const auto &observation : edge.second) gids.insert(observation.gid);
      if (edge.second.empty()) missing = true;
      if (gids.count("")) record_error("missing actual message publisher GID on " + edge.first);
      if (gids.size() > 1) record_error("more than one observed publisher for " + edge.first);
      for (const auto &identity : gids) {
        std::set<std::string> nodes;
        // Fast DDS can expose an endpoint GID before ROS graph discovery has
        // resolved its name. Keep that observation in the raw history; it is
        // absence of a name, not a second node identity for the same GID.
        for (const auto &endpoint : endpoints_) if (endpoint.second.gid == identity &&
            endpoint.second.node != "_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_") nodes.insert(endpoint.second.node);
        if (nodes.empty()) missing = true;
        else if (nodes.size() != 1) record_error("GID resolves to conflicting discovered nodes on " + edge.first);
      }
    }
    const auto status = !errors_.empty() ? "FAIL" : missing || interrupted ? "BLOCKED" : "PASS";
    exit_code = std::string(status) == "PASS" ? 0 : 1;
    std::array<char, 256> hostname{};
    if (::gethostname(hostname.data(), hostname.size() - 1) != 0) throw std::runtime_error("cannot read hostname");
    const char *domain = std::getenv("ROS_DOMAIN_ID");
    std::ostringstream out;
    out << std::setprecision(17) << "{\"schema_version\":1,\"test\":\"native_tf_ownership_probe\",\"status\":" << json(status)
        << ",\"session_id\":" << json(session_id_) << ",\"session_binding\":\"operator_supplied_label; consumer must cross-check actual session messages and matching TF samples\""
        << ",\"hostname\":" << json(hostname.data()) << ",\"ros_domain_id\":" << json(domain ? domain : "0")
        << ",\"start_unix_ns\":" << start_unix_ns_ << ",\"end_unix_ns\":" << end
        << ",\"requested_duration_s\":" << duration_s_ << ",\"elapsed_s\":" << std::chrono::duration<double>(Clock::now() - started_).count()
        << ",\"interrupted\":" << (interrupted ? "true" : "false") << ",\"topic\":" << json(topic_)
        << (wheel_private_mode_ ? ",\"wheel_private_mode\":true" : "")
        << ",\"qos\":{\"depth\":100,\"reliability\":\"BEST_EFFORT\"}"
        << ",\"rmw_gid_storage_size\":" << RMW_GID_STORAGE_SIZE
        << ",\"messages_received\":" << messages_received_ << ",\"errors\":[";
    bool first = true;
    for (const auto &error : errors_) { if (!first) out << ','; first = false; out << json(error); }
    out << "],\"edges\":{";
    first = true;
    for (const auto &edge : observations_) {
      if (!first) out << ',';
      first = false;
      std::map<std::string, std::size_t> gids;
      std::int64_t min_stamp = std::numeric_limits<std::int64_t>::max(), max_stamp = 0;
      for (const auto &observation : edge.second) {
        ++gids[observation.gid]; min_stamp = std::min(min_stamp, observation.stamp_ns); max_stamp = std::max(max_stamp, observation.stamp_ns);
      }
      out << json(edge.first) << ":{\"transform_count\":" << edge.second.size() << ",\"publisher_gids\":{";
      bool first_gid = true;
      for (const auto &identity : gids) {
        if (!first_gid) out << ',';
        first_gid = false;
        out << json(identity.first) << ':' << identity.second;
      }
      out << "},\"min_stamp_ns\":";
      if (edge.second.empty()) out << "null"; else out << min_stamp;
      out << ",\"max_stamp_ns\":";
      if (edge.second.empty()) out << "null"; else out << max_stamp;
      out << ",\"observations\":[";
      bool first_observation = true;
      for (const auto &observation : edge.second) {
        if (!first_observation) out << ',';
        first_observation = false;
        out << "{\"stamp_ns\":" << observation.stamp_ns << ",\"received_unix_ns\":" << observation.received_unix_ns
            << ",\"publisher_gid\":" << json(observation.gid) << ",\"transform_xyz_xyzw\":[";
        for (std::size_t i = 0; i < observation.transform.size(); ++i) { if (i) out << ','; out << observation.transform[i]; }
        out << "]}";
      }
      out << "]}";
    }
    auto write_endpoint = [&out](const Endpoint &endpoint) {
      out << "{\"publisher_gid\":" << json(endpoint.gid) << ",\"node\":" << json(endpoint.node)
          << ",\"first_seen_unix_ns\":" << endpoint.first_seen_unix_ns
          << ",\"last_seen_unix_ns\":" << endpoint.last_seen_unix_ns << '}';
    };
    out << "},\"endpoint_history\":[";
    first = true;
    for (const auto &endpoint : endpoints_) { if (!first) out << ','; first = false; write_endpoint(endpoint.second); }
    out << "],\"last_discovery_unix_ns\":" << last_discovery_unix_ns_ << ",\"current_endpoints\":[";
    first = true;
    for (const auto &endpoint : current_endpoints_) { if (!first) out << ','; first = false; write_endpoint(endpoint); }
    out << "],\"scope\":" << json("Actual observed " + topic_ + " messages only; no claim about unobserved publishers or TF static topic.")
        << ",\"navigation_validated\":false}\n";
    return out.str();
  }

 private:
  void receive(const tf2_msgs::msg::TFMessage &message, const rclcpp::MessageInfo &info) {
    ++messages_received_;
    const auto &rmw_info = info.get_rmw_message_info();
    const auto identity = gid_hex(rmw_info.publisher_gid.data, RMW_GID_STORAGE_SIZE);
    for (const auto &transform : message.transforms) {
      const auto edge = transform.header.frame_id + "->" + transform.child_frame_id;
      auto found = observations_.find(edge);
      if (found == observations_.end()) {
        if (wheel_private_mode_) record_error("unexpected private TF edge observed: " + edge);
        continue;
      }
      if (found->second.size() >= 100000) {
        record_error("evidence record bound exceeded on " + edge);
        continue;
      }
      const auto &t = transform.transform.translation;
      const auto &q = transform.transform.rotation;
      std::array<double, 7> values{t.x, t.y, t.z, q.x, q.y, q.z, q.w};
      const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
      if (!std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); }) || std::abs(norm - 1.0) > 1e-4) {
        record_error("nonfinite or nonnormalized actual transform on " + edge);
        continue;
      }
      const auto timestamp = static_cast<std::int64_t>(transform.header.stamp.sec) * 1000000000LL + transform.header.stamp.nanosec;
      found->second.push_back(Observation{timestamp, unix_ns(), identity, values});
    }
  }
  fs::path output_;
  std::string session_id_;
  bool wheel_private_mode_ = false;
  std::string topic_ = "/tf";
  double duration_s_;
  Clock::time_point started_;
  std::int64_t start_unix_ns_;
  std::int64_t last_discovery_unix_ns_ = 0;
  std::size_t messages_received_ = 0;
  std::map<std::string, std::vector<Observation>> observations_;
  std::map<std::pair<std::string, std::string>, Endpoint> endpoints_;
  std::vector<Endpoint> current_endpoints_;
  std::set<std::string> errors_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto probe = std::make_shared<Probe>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(probe);
    std::cout << "TF_PROBE_READY output=" << probe->output() << std::endl;
    while (rclcpp::ok() && !probe->finished()) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    int exit_code = 1;
    const auto evidence = probe->report(!rclcpp::ok(), exit_code);
    immutable_report(probe->output(), evidence);
    std::cout << "TF_PROBE_COMPLETE output=" << probe->output() << " exit=" << exit_code << std::endl;
    executor.remove_node(probe);
    probe.reset();
    if (rclcpp::ok()) rclcpp::shutdown();
    return exit_code;
  } catch (const std::exception &error) {
    std::cerr << "tf_ownership_probe: " << error.what() << std::endl;
    if (rclcpp::ok()) rclcpp::shutdown();
    return 2;
  }
}
