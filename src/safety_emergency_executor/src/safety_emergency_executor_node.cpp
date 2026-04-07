#include "safety_emergency_executor/safety_emergency_executor_node.hpp"

#include <chrono>
#include <functional>
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
  const auto cmd_vel_navigation_topic = this->declare_parameter<std::string>(
    "cmd_vel_navigation_topic", "/cmd_vel");
  const auto cmd_vel_miniapp_topic = this->declare_parameter<std::string>(
    "cmd_vel_miniapp_topic", "/cmd_vel_miniapp");
  const auto cmd_vel_remote_topic = this->declare_parameter<std::string>(
    "cmd_vel_remote_topic", "/cmd_vel_remote");
  const auto cmd_vel_other_topic = this->declare_parameter<std::string>(
    "cmd_vel_other_topic", "/cmd_vel_other");
  const auto control_source_state_topic = this->declare_parameter<std::string>(
    "control_source_state_topic", "/control_source_state");
  const auto active_control_source = this->declare_parameter<std::string>(
    "active_control_source", "navigation");
  const auto auto_preempt_enabled = this->declare_parameter<bool>(
    "control_source_auto_preempt_enabled", false);
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

  velocity_converter_.configure(*this);
  pressure_adjuster_.configure(*this);
  safety_policy_.configure(*this);
  control_source_controller_ = ControlSourceController(active_control_source, auto_preempt_enabled);

  command_pub_ = this->create_publisher<std_msgs::msg::String>(
    command_output_topic, rclcpp::QoS(20));
  control_source_state_pub_ = this->create_publisher<std_msgs::msg::String>(
    control_source_state_topic, rclcpp::QoS(1).transient_local().reliable());

  auto create_cmd_vel_subscription = [this](
    const std::string & source,
    const std::string & topic)
    {
      if (topic.empty()) {
        RCLCPP_WARN(
          get_logger(),
          "control source '%s' has no topic configured; no subscription created",
          source.c_str());
        return;
      }
      cmd_vel_subscriptions_[source] = this->create_subscription<geometry_msgs::msg::Twist>(
        topic, rclcpp::QoS(20),
        [this, source](const geometry_msgs::msg::Twist::SharedPtr msg) {
          on_cmd_vel(source, msg);
        });
    };
  create_cmd_vel_subscription("navigation", cmd_vel_navigation_topic);
  create_cmd_vel_subscription("miniapp", cmd_vel_miniapp_topic);
  create_cmd_vel_subscription("remote", cmd_vel_remote_topic);
  create_cmd_vel_subscription("other", cmd_vel_other_topic);
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
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&SafetyEmergencyExecutorNode::on_parameter_update, this, std::placeholders::_1));

  publish_control_source_state();

  RCLCPP_INFO(
    get_logger(),
    "safety_emergency_executor started: safety_cmd=%s, active_source=%s, nav=%s, miniapp=%s, remote=%s, other=%s, out=%s",
    safety_cmd_topic.c_str(), control_source_controller_.active_source().c_str(),
    cmd_vel_navigation_topic.c_str(), cmd_vel_miniapp_topic.c_str(),
    cmd_vel_remote_topic.c_str(), cmd_vel_other_topic.c_str(), command_output_topic.c_str());
}

void SafetyEmergencyExecutorNode::on_cmd_vel(
  const std::string & source,
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!control_source_controller_.accepts(source)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ignoring cmd_vel from inactive source '%s' while active source is '%s'",
      source.c_str(), control_source_controller_.active_source().c_str());
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

rcl_interfaces::msg::SetParametersResult SafetyEmergencyExecutorNode::on_parameter_update(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "ok";

  bool should_publish_state = false;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() != "active_control_source") {
      continue;
    }

    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      result.successful = false;
      result.reason = "active_control_source must be a string";
      return result;
    }

    const auto change = control_source_controller_.set_active_source(parameter.as_string());
    if (!change.success) {
      result.successful = false;
      result.reason = change.message;
      return result;
    }
    should_publish_state = should_publish_state || change.changed;
  }

  if (should_publish_state) {
    publish_control_source_state();
  }

  return result;
}

void SafetyEmergencyExecutorNode::publish_frame(const CommandFrame & frame)
{
  std_msgs::msg::String out;
  out.data = velocity_converter_.to_json(frame);
  command_pub_->publish(out);
}

void SafetyEmergencyExecutorNode::publish_control_source_state()
{
  std_msgs::msg::String state_msg;
  state_msg.data = control_source_controller_.active_source();
  control_source_state_pub_->publish(state_msg);
}

}  // namespace safety_emergency_executor
