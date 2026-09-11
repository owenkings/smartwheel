// Standalone, read-only /tf publisher-identity collector for ROS 2 Humble.
//
// This process creates one subscription and one graph-inspection timer.  It
// never creates a publisher, service, TF broadcaster, or hardware interface.

#include <array>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/message_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"
#include "tf2_msgs/msg/tf_message.hpp"

namespace
{

std::string gid_hex(const uint8_t * data, std::size_t size)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (std::size_t index = 0; index < size; ++index) {
    stream << std::setw(2) << static_cast<unsigned int>(data[index]);
  }
  return stream.str();
}

std::string gid_hex(const std::array<uint8_t, RMW_GID_STORAGE_SIZE> & gid)
{
  return gid_hex(gid.data(), gid.size());
}

std::string tsv_escape(const std::string & value)
{
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    switch (character) {
      case '\\':
        escaped += "\\\\";
        break;
      case '\t':
        escaped += "\\t";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\n':
        escaped += "\\n";
        break;
      default:
        escaped += character;
        break;
    }
  }
  return escaped;
}

std::string fully_qualified_node_name(
  const std::string & node_namespace, const std::string & node_name)
{
  if (node_namespace.empty() || node_namespace == "/") {
    return "/" + node_name;
  }
  if (node_namespace.back() == '/') {
    return node_namespace + node_name;
  }
  return node_namespace + "/" + node_name;
}

class TfAudit final : public rclcpp::Node
{
public:
  TfAudit()
  : Node("smartwheel_tf_audit")
  {
    // tf2_ros::DynamicBroadcasterQoS is reliable/volatile with depth 100 in
    // the installed Humble stack.  State it explicitly so the audit does not
    // silently miss a reliable /tf writer through an incompatible request.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(100));
    qos.reliable();
    qos.durability_volatile();
    subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", qos,
      [this](
        const tf2_msgs::msg::TFMessage::SharedPtr message,
        const rclcpp::MessageInfo & message_info)
      {
        on_tf(message, message_info);
      });
    endpoint_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {report_endpoints();});

    std::cout << "FORMAT\tSMARTWHEEL_TF_AUDIT_TSV_V1" << std::endl;
    std::cout << "FIELDS_TF\tparent\tchild\theader_sec\theader_nanosec\tpublisher_gid_hex"
              << std::endl;
    std::cout << "FIELDS_ENDPOINT\tpublisher_gid_hex\tfully_qualified_node_name"
              << std::endl;
  }

private:
  void on_tf(
    const tf2_msgs::msg::TFMessage::SharedPtr & message,
    const rclcpp::MessageInfo & message_info)
  {
    const auto & rmw_info = message_info.get_rmw_message_info();
    const std::string publisher_gid = gid_hex(
      rmw_info.publisher_gid.data, RMW_GID_STORAGE_SIZE);
    for (const auto & transform : message->transforms) {
      std::cout << "TF\t"
                << tsv_escape(transform.header.frame_id) << '\t'
                << tsv_escape(transform.child_frame_id) << '\t'
                << transform.header.stamp.sec << '\t'
                << transform.header.stamp.nanosec << '\t'
                << publisher_gid << std::endl;
    }
  }

  void report_endpoints()
  {
    const auto endpoints = get_publishers_info_by_topic("/tf");
    std::cout << "ENDPOINT_SNAPSHOT\t" << endpoints.size() << std::endl;
    for (const auto & endpoint : endpoints) {
      std::cout << "ENDPOINT\t"
                << gid_hex(endpoint.endpoint_gid()) << '\t'
                << tsv_escape(fully_qualified_node_name(
          endpoint.node_namespace(), endpoint.node_name()))
                << std::endl;
    }
  }

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr endpoint_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfAudit>());
  rclcpp::shutdown();
  return 0;
}
