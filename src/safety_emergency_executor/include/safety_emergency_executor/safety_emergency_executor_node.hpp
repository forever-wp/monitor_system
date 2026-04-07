#ifndef SAFETY_EMERGENCY_EXECUTOR__SAFETY_EMERGENCY_EXECUTOR_NODE_HPP_
#define SAFETY_EMERGENCY_EXECUTOR__SAFETY_EMERGENCY_EXECUTOR_NODE_HPP_

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_monitor/msg/safety_cmd.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>
#include <unordered_map>
#include <vector>

#include "safety_emergency_executor/command_frame.hpp"
#include "safety_emergency_executor/control_source_controller.hpp"
#include "safety_emergency_executor/pressure_adjuster.hpp"
#include "safety_emergency_executor/safety_policy_executor.hpp"
#include "safety_emergency_executor/velocity_converter.hpp"

namespace safety_emergency_executor
{

class SafetyEmergencyExecutorNode : public rclcpp::Node
{
public:
  explicit SafetyEmergencyExecutorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_cmd_vel(const std::string & source, const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_pressure_update(const std_msgs::msg::String::SharedPtr msg);
  void on_acc_update(const std_msgs::msg::Int32::SharedPtr msg);
  void on_wheel_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_loc_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void on_safety_cmd(const nav2_monitor::msg::SafetyCmd::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult on_parameter_update(
    const std::vector<rclcpp::Parameter> & parameters);
  void publish_frame(const CommandFrame & frame);
  void publish_control_source_state();

  rclcpp::Subscription<nav2_monitor::msg::SafetyCmd>::SharedPtr safety_cmd_sub_;
  std::unordered_map<std::string, rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr>
  cmd_vel_subscriptions_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pressure_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr acc_update_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr loc_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_source_state_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  VelocityConverter velocity_converter_;
  PressureAdjuster pressure_adjuster_;
  SafetyPolicyExecutor safety_policy_;
  ControlSourceController control_source_controller_{"navigation", false};
};

}  // namespace safety_emergency_executor

#endif  // SAFETY_EMERGENCY_EXECUTOR__SAFETY_EMERGENCY_EXECUTOR_NODE_HPP_
