#include "smartwheel_rviz_plugins/safety_gate.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

namespace smartwheel_rviz_plugins
{

class MockSafetySupervisor final : public rclcpp::Node
{
public:
  MockSafetySupervisor()
  : Node("mock_safety_supervisor"),
    started_(std::chrono::steady_clock::now()),
    gate_(
      declare_parameter("command_timeout_sec", 0.35),
      declare_parameter("linear_speed_max_mps", 0.15),
      declare_parameter("angular_speed_max_radps", 0.25))
  {
    const auto mode = declare_parameter<std::string>("mode", "mock");
    const bool hardware_enabled = declare_parameter("hardware_enabled", false);
    const double rate = declare_parameter("command_rate_hz", 20.0);
    if (mode != "mock") {
      throw std::runtime_error("mock safety supervisor permits mode:=mock only");
    }
    if (hardware_enabled) {
      throw std::runtime_error("mock safety supervisor refuses hardware_enabled:=true");
    }
    if (rate <= 0.0) {
      throw std::invalid_argument("command_rate_hz must be positive");
    }

    safe_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safe", 10);
    estop_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/emergency_stop", rclcpp::QoS(1).reliable().transient_local());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/teleop/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::ConstSharedPtr message) {
        ++input_count_;
        if (!gate_.accept(message->linear.x, message->angular.z, steadySeconds())) {
          RCLCPP_ERROR(get_logger(), "rejected non-finite /teleop/cmd_vel and stopped mock base");
        }
      });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/emergency_stop_request", 10,
      [this](const std_msgs::msg::Bool::ConstSharedPtr message) {
        gate_.setEmergencyStop(message->data);
        std_msgs::msg::Bool state;
        state.data = gate_.emergencyStop();
        estop_pub_->publish(state);
      });
    command_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), std::bind(&MockSafetySupervisor::tick, this));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&MockSafetySupervisor::publishDiagnostics, this));
    std_msgs::msg::Bool clear;
    clear.data = false;
    estop_pub_->publish(clear);
    publishZero();
  }

  void publishZero()
  {
    safe_pub_->publish(geometry_msgs::msg::Twist());
  }

private:
  double steadySeconds() const
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count();
  }

  void tick()
  {
    const auto [linear, angular, reason] = gate_.output(steadySeconds());
    geometry_msgs::msg::Twist command;
    command.linear.x = linear;
    command.angular.z = angular;
    safe_pub_->publish(command);
    last_reason_ = reason;
    ++output_count_;
  }

  void publishDiagnostics()
  {
    const double elapsed = std::max(steadySeconds(), 1e-6);
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = gate_.emergencyStop() ?
      diagnostic_msgs::msg::DiagnosticStatus::ERROR : diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.name = "smartwheel_workbench/safety_supervisor";
    status.message = "MOCK_ONLY " + last_reason_ + "; /cmd_vel_safe has no motor transport";
    status.hardware_id = "MOCK_ONLY";
    status.values = {
      diagnostic_msgs::msg::KeyValue().set__key("input_rate_hz").set__value(
        std::to_string(input_count_ / elapsed)),
      diagnostic_msgs::msg::KeyValue().set__key("output_rate_hz").set__value(
        std::to_string(output_count_ / elapsed)),
      diagnostic_msgs::msg::KeyValue().set__key("command_timeout_sec").set__value(
        std::to_string(gate_.timeout())),
      diagnostic_msgs::msg::KeyValue().set__key("invalid_command_count").set__value(
        std::to_string(gate_.invalidCount()))};
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  std::chrono::steady_clock::time_point started_;
  SafetyGate gate_;
  std::string last_reason_{"COMMAND_TIMEOUT"};
  std::size_t input_count_{0};
  std::size_t output_count_{0};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace smartwheel_rviz_plugins

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<smartwheel_rviz_plugins::MockSafetySupervisor>();
  rclcpp::spin(node);
  if (rclcpp::ok()) {
    node->publishZero();
  }
  node.reset();
  rclcpp::shutdown();
  return 0;
}
