// Copyright 2026 tokou
//
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav2_monitor/msg/safety_cmd.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

#include "safety_emergency_executor/command_frame.hpp"
#include "safety_emergency_executor/pressure_adjuster.hpp"
#include "safety_emergency_executor/safety_policy_executor.hpp"
#include "safety_emergency_executor/velocity_converter.hpp"

namespace safety_emergency_executor
{

class SafetyEmergencyExecutorNode : public rclcpp::Node
{
public:
  SafetyEmergencyExecutorNode()
  : Node("safety_emergency_executor")
  {
    const auto safety_cmd_topic = this->declare_parameter<std::string>(
      "safety_cmd_topic", "/safety_system/cmd");
    const auto command_output_topic = this->declare_parameter<std::string>(
      "command_output_topic", "/command");
    const auto cmd_vel_topic = this->declare_parameter<std::string>(
      "cmd_vel_topic", "/cmd_vel");
    const auto pressure_update_topic = this->declare_parameter<std::string>(
      "pressure_update_topic", "/pressure_");
    const auto wheel_odom_topic = this->declare_parameter<std::string>(
      "wheel_odom_topic", "/odom_base");
    const auto loc_odom_topic = this->declare_parameter<std::string>(
      "loc_odom_topic", "/odom");
    const auto imu_topic = this->declare_parameter<std::string>(
      "imu_topic", "/livox/imu");

    velocity_converter_.configure(*this);
    pressure_adjuster_.configure(*this);
    safety_policy_.configure(*this);

    command_pub_ = this->create_publisher<std_msgs::msg::String>(
      command_output_topic, rclcpp::QoS(20));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, rclcpp::QoS(20),
      std::bind(&SafetyEmergencyExecutorNode::on_cmd_vel, this, std::placeholders::_1));
    pressure_sub_ = this->create_subscription<std_msgs::msg::String>(
      pressure_update_topic, rclcpp::QoS(10),
      std::bind(&SafetyEmergencyExecutorNode::on_pressure_update, this, std::placeholders::_1));
    wheel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      wheel_odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&SafetyEmergencyExecutorNode::on_wheel_odom, this, std::placeholders::_1));
    loc_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      loc_odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&SafetyEmergencyExecutorNode::on_loc_odom, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&SafetyEmergencyExecutorNode::on_imu, this, std::placeholders::_1));
    safety_cmd_sub_ = this->create_subscription<nav2_monitor::msg::SafetyCmd>(
      safety_cmd_topic, rclcpp::QoS(20),
      std::bind(&SafetyEmergencyExecutorNode::on_safety_cmd, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "safety_emergency_executor started: safety_cmd=%s, cmd_vel=%s, out=%s",
      safety_cmd_topic.c_str(), cmd_vel_topic.c_str(), command_output_topic.c_str());
  }

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Front gate: short-circuit as early as possible when safety forbids forwarding.
    if (!safety_policy_.allow_forward()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Forwarding disabled by safety policy, dropping cmd_vel messages");
      return;
    }

    CommandFrame frame = velocity_converter_.convert(*msg);
    pressure_adjuster_.apply(frame);

    // Final gate: re-check safety immediately before serializing and publishing.
    if (!safety_policy_.apply(frame)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Safety policy blocked command at publish stage");
      return;
    }

    publish_frame(frame);
  }

  void on_pressure_update(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string error;
    if (!velocity_converter_.update_params_from_json(msg->data, &error)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to parse pressure update: %s", error.c_str());
    }
  }

  void on_wheel_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    pressure_adjuster_.on_wheel_odom(msg);
  }

  void on_loc_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    pressure_adjuster_.on_loc_odom(msg);
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    pressure_adjuster_.on_imu(msg);
  }

  void on_safety_cmd(const nav2_monitor::msg::SafetyCmd::SharedPtr msg)
  {
    const auto emergency_sequence = safety_policy_.on_safety_cmd(
      *msg, velocity_converter_.template_frame());

    if (msg->action != nav2_monitor::msg::SafetyCmd::EMERGENCY_STOP) {
      return;
    }

    const auto interval_ms = safety_policy_.brake_interval_ms();
    for (const auto & frame : emergency_sequence) {
      publish_frame(frame);
      rclcpp::sleep_for(std::chrono::milliseconds(interval_ms));
    }
  }

  void publish_frame(const CommandFrame & frame)
  {
    std_msgs::msg::String out;
    out.data = velocity_converter_.to_json(frame);
    command_pub_->publish(out);
  }

  rclcpp::Subscription<nav2_monitor::msg::SafetyCmd>::SharedPtr safety_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pressure_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr loc_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;

  VelocityConverter velocity_converter_;
  PressureAdjuster pressure_adjuster_;
  SafetyPolicyExecutor safety_policy_;
};

}  // namespace safety_emergency_executor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<safety_emergency_executor::SafetyEmergencyExecutorNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
