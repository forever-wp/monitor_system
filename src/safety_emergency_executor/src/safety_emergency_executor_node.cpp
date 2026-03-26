#include "safety_emergency_executor/safety_emergency_executor_node.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace safety_emergency_executor
{

SafetyEmergencyExecutorNode::SafetyEmergencyExecutorNode(const rclcpp::NodeOptions & options)
: Node("safety_emergency_executor", options)
{
  const auto safety_cmd_topic = this->declare_parameter<std::string>(
    "safety_cmd_topic", "/safety_system/cmd");
  const auto command_output_topic = this->declare_parameter<std::string>(
    "command_output_topic", "/command");
  const auto cmd_vel_topic = this->declare_parameter<std::string>(
    "cmd_vel_topic", "/cmd_vel");
  const auto pressure_update_topic = this->declare_parameter<std::string>(
    "pressure_update_topic", "/pressure_");
  const auto acc_update_topic = this->declare_parameter<std::string>(
    "acc_update_topic", "/acc_");
  const auto wheel_odom_topic = this->declare_parameter<std::string>(
    "wheel_odom_topic", "/odom_base");
  const auto loc_odom_topic = this->declare_parameter<std::string>(
    "loc_odom_topic", "/odom");
  const auto imu_topic = this->declare_parameter<std::string>(
    "imu_topic", "/livox/imu");
  const auto manual_override_service = this->declare_parameter<std::string>(
    "manual_override_service", "/set_manual_override");
  const auto manual_override_state_topic = this->declare_parameter<std::string>(
    "manual_override_state_topic", "/manual_override_active");

  velocity_converter_.configure(*this);
  pressure_adjuster_.configure(*this);
  safety_policy_.configure(*this);

  command_pub_ = this->create_publisher<std_msgs::msg::String>(
    command_output_topic, rclcpp::QoS(20));
  manual_override_state_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    manual_override_state_topic, rclcpp::QoS(1).transient_local().reliable());

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, rclcpp::QoS(20),
    std::bind(&SafetyEmergencyExecutorNode::on_cmd_vel, this, std::placeholders::_1));
  pressure_sub_ = this->create_subscription<std_msgs::msg::String>(
    pressure_update_topic, rclcpp::QoS(10),
    std::bind(&SafetyEmergencyExecutorNode::on_pressure_update, this, std::placeholders::_1));
  acc_update_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    acc_update_topic, rclcpp::QoS(10),
    std::bind(&SafetyEmergencyExecutorNode::on_acc_update, this, std::placeholders::_1));
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
  manual_override_srv_ = this->create_service<std_srvs::srv::SetBool>(
    manual_override_service,
    std::bind(
      &SafetyEmergencyExecutorNode::on_manual_override_request,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  publish_manual_override_state();

  RCLCPP_INFO(
    get_logger(),
    "safety_emergency_executor started: safety_cmd=%s, cmd_vel=%s, manual_override_service=%s, out=%s",
    safety_cmd_topic.c_str(), cmd_vel_topic.c_str(), manual_override_service.c_str(),
    command_output_topic.c_str());
}

void SafetyEmergencyExecutorNode::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (external_override_.manual_override_active()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Manual override active, dropping automatic cmd_vel messages");
    return;
  }

  if (!safety_policy_.allow_forward()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Forwarding disabled by safety policy, dropping cmd_vel messages");
    return;
  }

  CommandFrame frame = velocity_converter_.convert(*msg);
  pressure_adjuster_.apply(frame);

  if (!safety_policy_.apply(frame)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Safety policy blocked command at publish stage");
    return;
  }

  publish_frame(frame);
}

void SafetyEmergencyExecutorNode::on_pressure_update(const std_msgs::msg::String::SharedPtr msg)
{
  std::string error;
  if (!velocity_converter_.update_params_from_json(msg->data, &error)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Failed to parse pressure update: %s", error.c_str());
  }
}

void SafetyEmergencyExecutorNode::on_acc_update(const std_msgs::msg::Int32::SharedPtr msg)
{
  velocity_converter_.update_acc_from_topic(msg->data);
}

void SafetyEmergencyExecutorNode::on_wheel_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  pressure_adjuster_.on_wheel_odom(msg);
}

void SafetyEmergencyExecutorNode::on_loc_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  pressure_adjuster_.on_loc_odom(msg);
}

void SafetyEmergencyExecutorNode::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  pressure_adjuster_.on_imu(msg);
}

void SafetyEmergencyExecutorNode::on_safety_cmd(const nav2_monitor::msg::SafetyCmd::SharedPtr msg)
{
  if (external_override_.manual_override_active()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Manual override active, ignoring safety commands");
    return;
  }

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

void SafetyEmergencyExecutorNode::on_manual_override_request(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  const bool activating_manual = request->data && !external_override_.manual_override_active();
  if (activating_manual) {
    auto stop_frame = velocity_converter_.template_frame();
    stop_frame.speed = 0.0;
    stop_frame.angle = 0.0;
    publish_frame(stop_frame);
  }

  const auto change = external_override_.set_manual_override(request->data);
  if (activating_manual) {
    safety_policy_.reset();
  }
  if (change.state_changed) {
    publish_manual_override_state();
  }

  response->success = true;
  response->message = change.message;
  RCLCPP_INFO(get_logger(), "%s", change.message.c_str());
}

void SafetyEmergencyExecutorNode::publish_frame(const CommandFrame & frame)
{
  std_msgs::msg::String out;
  out.data = velocity_converter_.to_json(frame);
  command_pub_->publish(out);
}

void SafetyEmergencyExecutorNode::publish_manual_override_state()
{
  std_msgs::msg::Bool state_msg;
  state_msg.data = external_override_.manual_override_active();
  manual_override_state_pub_->publish(state_msg);
}

}  // namespace safety_emergency_executor
