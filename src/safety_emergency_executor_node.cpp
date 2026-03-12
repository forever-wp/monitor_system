// Copyright 2026 tokou
//
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav2_monitor/msg/safety_cmd.hpp>

namespace safety_emergency_executor
{

class SafetyEmergencyExecutorNode : public rclcpp::Node
{
public:
  SafetyEmergencyExecutorNode()
  : Node("safety_emergency_executor")
  {
    const auto safety_cmd_topic = this->declare_parameter<std::string>(
      "safety_cmd_topic",
      "/safety_system/cmd");
    const auto command_input_topic = this->declare_parameter<std::string>(
      "command_input_topic",
      "/command_safety");
    const auto command_output_topic = this->declare_parameter<std::string>(
      "command_output_topic",
      "/command");

    slow_down_percentage_ = this->declare_parameter<double>("slow_down_percentage", 50.0);
    brake_strong_speed_ = this->declare_parameter<double>("brake_strong_speed", -1.0);
    brake_medium_speed_ = this->declare_parameter<double>("brake_medium_speed", -0.5);
    brake_strong_repeat_ = this->declare_parameter<int>("brake_strong_repeat", 2);
    brake_medium_repeat_ = this->declare_parameter<int>("brake_medium_repeat", 2);
    brake_zero_repeat_ = this->declare_parameter<int>("brake_zero_repeat", 4);
    brake_interval_ms_ = this->declare_parameter<int>("brake_interval_ms", 20);

    speed_limit_active_.store(false);
    speed_limit_percentage_.store(true);
    speed_limit_value_.store(0.0);
    forwarding_enabled_.store(true);

    command_pub_ =
      this->create_publisher<std_msgs::msg::String>(command_output_topic, rclcpp::QoS(20));

    command_sub_ = this->create_subscription<std_msgs::msg::String>(
      command_input_topic, rclcpp::QoS(20),
      std::bind(&SafetyEmergencyExecutorNode::on_command_input, this, std::placeholders::_1));

    safety_cmd_sub_ = this->create_subscription<nav2_monitor::msg::SafetyCmd>(
      safety_cmd_topic, rclcpp::QoS(20),
      std::bind(&SafetyEmergencyExecutorNode::on_safety_cmd, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "safety_emergency_executor started: safety_cmd=%s, in=%s, out=%s",
      safety_cmd_topic.c_str(), command_input_topic.c_str(), command_output_topic.c_str());
  }

private:
  void on_safety_cmd(const nav2_monitor::msg::SafetyCmd::SharedPtr msg)
  {
    const auto action = msg->action;
    if (action == nav2_monitor::msg::SafetyCmd::RESUME) {
      forwarding_enabled_.store(true);
      speed_limit_percentage_.store(true);
      speed_limit_value_.store(0.0);
      speed_limit_active_.store(false);
      RCLCPP_WARN(get_logger(), "Apply RESUME: clear safety restrictions, reason=%s", msg->reason.c_str());
      return;
    }

    if (action == nav2_monitor::msg::SafetyCmd::SLOW_DOWN) {
      const double requested_percentage =
        msg->slow_down_percentage > 0.0F ? msg->slow_down_percentage : slow_down_percentage_;
      forwarding_enabled_.store(true);
      speed_limit_percentage_.store(true);
      speed_limit_value_.store(std::clamp(requested_percentage, 0.0, 100.0));
      speed_limit_active_.store(true);
      RCLCPP_WARN(
        get_logger(),
        "Apply SLOW_DOWN: %.1f%%, reason=%s",
        speed_limit_value_.load(), msg->reason.c_str());
      return;
    }

    if (action == nav2_monitor::msg::SafetyCmd::SOFT_STOP) {
      forwarding_enabled_.store(true);
      speed_limit_percentage_.store(true);
      speed_limit_value_.store(0.0);
      speed_limit_active_.store(true);
      RCLCPP_ERROR(
        get_logger(), "Apply SOFT_STOP: speed limit -> 0%%, reason=%s",
        msg->reason.c_str());
      return;
    }

    if (action == nav2_monitor::msg::SafetyCmd::EMERGENCY_STOP) {
      forwarding_enabled_.store(false);
      speed_limit_percentage_.store(true);
      speed_limit_value_.store(0.0);
      speed_limit_active_.store(true);
      RCLCPP_ERROR(get_logger(), "Apply EMERGENCY_STOP, reason=%s", msg->reason.c_str());
      publish_brake_sequence();
      return;
    }

    RCLCPP_WARN(get_logger(), "Unknown safety action=%u, reason=%s", action, msg->reason.c_str());
  }

  void on_command_input(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!forwarding_enabled_.load()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Forwarding disabled by EMERGENCY_STOP, dropping command_safety messages");
      return;
    }

    std_msgs::msg::String out = *msg;
    apply_limit_to_field(out.data, "speed");
    apply_limit_to_field(out.data, "angle");
    command_pub_->publish(out);
  }

  void apply_limit_to_field(std::string & json, const std::string & field_name)
  {
    const std::string key = "\"" + field_name + "\":";
    const auto pos = json.find(key);
    if (pos == std::string::npos) {
      return;
    }

    auto value_begin = pos + key.size();
    while (value_begin < json.size() &&
      std::isspace(static_cast<unsigned char>(json[value_begin])))
    {
      ++value_begin;
    }
    auto value_end = value_begin;
    while (
      value_end < json.size() &&
      (std::isdigit(static_cast<unsigned char>(json[value_end])) ||
      json[value_end] == '.' || json[value_end] == '-' || json[value_end] == '+'))
    {
      ++value_end;
    }

    if (value_end <= value_begin) {
      return;
    }

    double original = 0.0;
    try {
      original = std::stod(json.substr(value_begin, value_end - value_begin));
    } catch (...) {
      return;
    }

    double limited = original;
    if (speed_limit_active_.load()) {
      const auto is_percentage = speed_limit_percentage_.load();
      const auto limit_value = speed_limit_value_.load();
      if (is_percentage) {
        limited = original * (limit_value / 100.0);
      } else {
        limited = std::clamp(original, -limit_value, limit_value);
      }
    }

    if (std::fabs(limited - original) < 1e-6) {
      return;
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << limited;
    json = json.substr(0, value_begin) + oss.str() + json.substr(value_end);
  }

  void publish_brake_sequence()
  {
    auto publish_speed = [this](double speed_value) {
        std_msgs::msg::String msg;
        std::ostringstream ss;
        ss << "{\"speed\":" << std::fixed << std::setprecision(3) << speed_value
           << ",\"angle\":0.0,\"press\":1000,\"acc\":1000,\"place\":-1,\"ulock\":-1}";
        msg.data = ss.str();
        command_pub_->publish(msg);
      };

    const int interval = std::max(1, brake_interval_ms_);
    for (int i = 0; i < std::max(0, brake_strong_repeat_); ++i) {
      publish_speed(brake_strong_speed_);
      rclcpp::sleep_for(std::chrono::milliseconds(interval));
    }
    for (int i = 0; i < std::max(0, brake_medium_repeat_); ++i) {
      publish_speed(brake_medium_speed_);
      rclcpp::sleep_for(std::chrono::milliseconds(interval));
    }
    for (int i = 0; i < std::max(0, brake_zero_repeat_); ++i) {
      publish_speed(0.0);
      rclcpp::sleep_for(std::chrono::milliseconds(interval));
    }
  }

private:
  rclcpp::Subscription<nav2_monitor::msg::SafetyCmd>::SharedPtr safety_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;

  std::atomic<bool> forwarding_enabled_{true};
  std::atomic<bool> speed_limit_active_{false};
  std::atomic<bool> speed_limit_percentage_{true};
  std::atomic<double> speed_limit_value_{0.0};

  double slow_down_percentage_{50.0};
  double brake_strong_speed_{-1.0};
  double brake_medium_speed_{-0.5};
  int brake_strong_repeat_{2};
  int brake_medium_repeat_{2};
  int brake_zero_repeat_{4};
  int brake_interval_ms_{20};
};

}  // namespace safety_emergency_executor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<safety_emergency_executor::SafetyEmergencyExecutorNode>());
  rclcpp::shutdown();
  return 0;
}
