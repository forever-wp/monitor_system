#include "safety_emergency_executor/safety_emergency_executor_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace safety_emergency_executor
{

namespace
{
constexpr const char * kNavigation = "navigation";
constexpr const char * kMiniapp = "miniapp";
constexpr const char * kRemote = "remote";
constexpr const char * kOther = "other";

constexpr const char * kNavigationSafetyEnabledParam = "control_source_navigation_safety_enabled";
constexpr const char * kMiniappSafetyEnabledParam = "control_source_miniapp_safety_enabled";
constexpr const char * kRemoteSafetyEnabledParam = "control_source_remote_safety_enabled";
constexpr const char * kOtherSafetyEnabledParam = "control_source_other_safety_enabled";

const char * bool_to_cstr(const bool value)
{
  return value ? "true" : "false";
}

const char * source_from_safety_param_name(const std::string & parameter_name)
{
  if (parameter_name == kNavigationSafetyEnabledParam) {
    return kNavigation;
  }
  if (parameter_name == kMiniappSafetyEnabledParam) {
    return kMiniapp;
  }
  if (parameter_name == kRemoteSafetyEnabledParam) {
    return kRemote;
  }
  if (parameter_name == kOtherSafetyEnabledParam) {
    return kOther;
  }
  return nullptr;
}
}  // namespace

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
  const auto control_source_command_topic = this->declare_parameter<std::string>(
    "control_source_command_topic", "/control_source_cmd");
  const auto control_source_state_publish_period_ms = this->declare_parameter<int64_t>(
    "control_source_state_publish_period_ms", 1000);
  const auto active_control_source = this->declare_parameter<std::string>(
    "active_control_source", "navigation");
  const auto auto_preempt_enabled = this->declare_parameter<bool>(
    "control_source_auto_preempt_enabled", false);
  control_source_safety_enabled_[kNavigation] = this->declare_parameter<bool>(
    kNavigationSafetyEnabledParam, true);
  control_source_safety_enabled_[kMiniapp] = this->declare_parameter<bool>(
    kMiniappSafetyEnabledParam, true);
  control_source_safety_enabled_[kRemote] = this->declare_parameter<bool>(
    kRemoteSafetyEnabledParam, true);
  control_source_safety_enabled_[kOther] = this->declare_parameter<bool>(
    kOtherSafetyEnabledParam, true);
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
  io_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  command_pub_ = this->create_publisher<std_msgs::msg::String>(
    command_output_topic, rclcpp::QoS(20));
  control_source_state_pub_ = this->create_publisher<std_msgs::msg::String>(
    control_source_state_topic, rclcpp::QoS(1).transient_local().reliable());
  const auto normalized_state_publish_period_ms =
    control_source_state_publish_period_ms > 0 ? control_source_state_publish_period_ms : 1000;
  if (control_source_state_publish_period_ms <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "control_source_state_publish_period_ms=%ld is invalid, fallback to 1000 ms",
      control_source_state_publish_period_ms);
  }
  control_source_state_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(normalized_state_publish_period_ms),
    std::bind(&SafetyEmergencyExecutorNode::publish_control_source_state, this),
    io_callback_group_);
  query_control_source_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/query_control_source",
    std::bind(
      &SafetyEmergencyExecutorNode::on_query_control_source, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    service_callback_group_);

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
      rclcpp::SubscriptionOptions subscription_options;
      subscription_options.callback_group = io_callback_group_;
      cmd_vel_subscriptions_[source] = this->create_subscription<geometry_msgs::msg::Twist>(
        topic, rclcpp::QoS(20),
        [this, source](const geometry_msgs::msg::Twist::SharedPtr msg) {
          on_cmd_vel(source, msg);
        },
        subscription_options);
    };
  create_cmd_vel_subscription("navigation", cmd_vel_navigation_topic);
  create_cmd_vel_subscription("miniapp", cmd_vel_miniapp_topic);
  create_cmd_vel_subscription("remote", cmd_vel_remote_topic);
  create_cmd_vel_subscription("other", cmd_vel_other_topic);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = io_callback_group_;
  pressure_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    pressure_update_topic, rclcpp::QoS(10),
    std::bind(&SafetyEmergencyExecutorNode::on_pressure_update, this, std::placeholders::_1),
    subscription_options);
  acc_update_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    acc_update_topic, rclcpp::QoS(10),
    std::bind(&SafetyEmergencyExecutorNode::on_acc_update, this, std::placeholders::_1),
    subscription_options);
  if (!control_source_command_topic.empty()) {
    control_source_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
      control_source_command_topic, rclcpp::QoS(10),
      std::bind(
        &SafetyEmergencyExecutorNode::on_control_source_command, this, std::placeholders::_1),
      subscription_options);
  } else {
    RCLCPP_WARN(
      get_logger(),
      "control_source_command_topic is empty; topic-based source switching disabled");
  }
  wheel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    wheel_odom_topic, rclcpp::SensorDataQoS(),
    std::bind(&SafetyEmergencyExecutorNode::on_wheel_odom, this, std::placeholders::_1),
    subscription_options);
  loc_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    loc_odom_topic, rclcpp::SensorDataQoS(),
    std::bind(&SafetyEmergencyExecutorNode::on_loc_odom, this, std::placeholders::_1),
    subscription_options);
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(),
    std::bind(&SafetyEmergencyExecutorNode::on_imu, this, std::placeholders::_1),
    subscription_options);
  safety_cmd_sub_ = this->create_subscription<nav2_monitor::msg::SafetyCmd>(
    safety_cmd_topic, rclcpp::QoS(20),
    std::bind(&SafetyEmergencyExecutorNode::on_safety_cmd, this, std::placeholders::_1),
    subscription_options);
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&SafetyEmergencyExecutorNode::on_parameter_update, this, std::placeholders::_1));

  publish_control_source_state();

  RCLCPP_INFO(
    get_logger(),
    "safety_emergency_executor started: safety_cmd=%s, active_source=%s, active_source_safety_enabled=%s, switch_cmd=%s, query_srv=%s, state=%s, state_period_ms=%ld, nav=%s, miniapp=%s, remote=%s, other=%s, out=%s",
    safety_cmd_topic.c_str(), control_source_controller_.active_source().c_str(),
    bool_to_cstr(is_active_source_safety_enabled()),
    control_source_command_topic.empty() ? "<disabled>" : control_source_command_topic.c_str(),
    "~/query_control_source",
    control_source_state_topic.c_str(),
    normalized_state_publish_period_ms,
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

  const bool safety_enabled = is_active_source_safety_enabled();
  if (safety_enabled && !safety_policy_.allow_forward()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Forwarding disabled by safety policy, dropping cmd_vel messages");
    return;
  }

  CommandFrame frame = velocity_converter_.convert(source, *msg);
  pressure_adjuster_.apply(frame);

  if (safety_enabled && !safety_policy_.apply(frame)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Safety policy blocked command at publish stage");
    return;
  }

  publish_frame(frame);
}

void SafetyEmergencyExecutorNode::on_pressure_update(const std_msgs::msg::Int32::SharedPtr msg)
{
  velocity_converter_.update_press_from_topic(msg->data);
  pressure_adjuster_.note_external_pressure_override(this->get_clock()->now());
}

void SafetyEmergencyExecutorNode::on_acc_update(const std_msgs::msg::Int32::SharedPtr msg)
{
  velocity_converter_.update_acc_from_topic(msg->data);
}

void SafetyEmergencyExecutorNode::on_control_source_command(
  const std_msgs::msg::String::SharedPtr msg)
{
  const auto result = this->set_parameter(
    rclcpp::Parameter("active_control_source", msg->data));
  if (!result.successful) {
    RCLCPP_WARN(
      get_logger(),
      "ignored control source command '%s': %s",
      msg->data.c_str(),
      result.reason.c_str());
  }
}

void SafetyEmergencyExecutorNode::on_query_control_source(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  response->success = true;
  response->message = control_source_controller_.active_source();
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

  if (!is_active_source_safety_enabled()) {
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
    const auto * safety_source = source_from_safety_param_name(parameter.get_name());
    if (safety_source != nullptr) {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = parameter.get_name() + " must be a bool";
        return result;
      }

      control_source_safety_enabled_[safety_source] = parameter.as_bool();
      if (control_source_controller_.active_source() == safety_source) {
        RCLCPP_INFO(
          get_logger(),
          "active control source safety updated: source=%s safety_enabled=%s",
          safety_source,
          bool_to_cstr(parameter.as_bool()));
      }
      continue;
    }

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
    log_control_source_change(change);
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

bool SafetyEmergencyExecutorNode::is_safety_enabled_for_source(const std::string & source) const
{
  const auto it = control_source_safety_enabled_.find(source);
  if (it == control_source_safety_enabled_.end()) {
    return true;
  }
  return it->second;
}

bool SafetyEmergencyExecutorNode::is_active_source_safety_enabled() const
{
  return is_safety_enabled_for_source(control_source_controller_.active_source());
}

void SafetyEmergencyExecutorNode::log_control_source_change(
  const ControlSourceChange & change) const
{
  const auto & source =
    change.active_source.empty() ? control_source_controller_.active_source() : change.active_source;
  RCLCPP_INFO(
    get_logger(),
    "%s, safety_enabled=%s",
    change.message.c_str(),
    bool_to_cstr(is_safety_enabled_for_source(source)));
}

}  // namespace safety_emergency_executor
