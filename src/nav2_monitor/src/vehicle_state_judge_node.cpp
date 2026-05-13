#include "nav2_monitor/vehicle_state_judge_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <sstream>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/serialized_message.hpp>

#include "nav2_monitor/config_profile_sync.hpp"

namespace nav2_monitor
{

namespace
{
std::string resolve_config_path(const std::string & config_file)
{
  if (config_file.empty()) {
    return config_file;
  }

  namespace fs = std::filesystem;
  const fs::path input(config_file);
  if (input.is_absolute() && fs::exists(input)) {
    return input.string();
  }
  if (fs::exists(input)) {
    return fs::absolute(input).lexically_normal().string();
  }

  try {
    const fs::path package_share =
      ament_index_cpp::get_package_share_directory("nav2_monitor");
    const std::array<fs::path, 2> candidates = {
      package_share / input,
      package_share / "config" / input
    };
    for (const auto & candidate : candidates) {
      if (fs::exists(candidate)) {
        return candidate.lexically_normal().string();
      }
    }
  } catch (const std::exception &) {
  }

  return config_file;
}
}  // namespace

VehicleStateJudgeNode::VehicleStateJudgeNode()
: Node("vehicle_state_judge"), fault_detector_(this)
{
  fault_config_path_ = declare_parameter<std::string>(
    "fault_config", "/opt/ry/config/Monitor/nav2_monitor/fault_detector_config.yaml");
  config_profile_topic_ = declare_parameter<std::string>(
    "config_profile_topic", "/monitor/config_profile");
  publish_topic_ = declare_parameter<std::string>("publish_topic", "/monitor/vehicle_state");
  human_intervention_topic_ = declare_parameter<std::string>(
    "human_intervention_topic", "/nav2_monitor/human_intervention");
  check_rate_hz_ = std::max(1.0, declare_parameter<double>("check_rate_hz", 10.0));

  load_configuration();

  state_pub_ = create_publisher<std_msgs::msg::String>(
    publish_topic_, rclcpp::QoS(1).reliable().transient_local());
  human_intervention_pub_ = create_publisher<std_msgs::msg::String>(
    human_intervention_topic_, rclcpp::QoS(50).reliable().durability_volatile());

  configure_profile_subscription();
  configure_subscriptions();

  const auto period = std::chrono::duration<double>(1.0 / check_rate_hz_);
  evaluate_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { evaluate_and_publish(); });
  moto_retry_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    [this]() { try_subscribe_moto_topic(); });

  RCLCPP_INFO(
    get_logger(),
    "vehicle_state_judge started: publish=%s human_intervention=%s command=%s moto=%s odom=%s imu=%s",
    publish_topic_.c_str(), human_intervention_topic_.c_str(), cfg_.command_topic.c_str(),
    cfg_.moto_topic.c_str(), cfg_.odom_topic.empty() ? "<disabled>" : cfg_.odom_topic.c_str(),
    cfg_.imu_topic.empty() ? "<disabled>" : cfg_.imu_topic.c_str());
}

void VehicleStateJudgeNode::load_configuration()
{
  const auto resolved = resolve_config_path(fault_config_path_);
  resolved_fault_config_path_ = resolved;
  fault_detector_.load_config(resolved);
  if (!fault_detector_.vehicle_state_judge_enabled()) {
    RCLCPP_WARN(
      get_logger(),
      "vehicle_state_judge config is disabled or missing in %s; node will publish UNKNOWN state",
      resolved.c_str());
  }
  cfg_ = fault_detector_.get_vehicle_state_judge_config();
  multi_value_cfg_ = fault_detector_.get_multi_value_judge_config();
  evaluator_.set_logger(get_logger());
  evaluator_.set_multi_value_config(multi_value_cfg_);
  evaluator_.reset();
  active_human_fault_keys_.clear();
}

void VehicleStateJudgeNode::configure_profile_subscription()
{
  config_profile_sub_ = create_subscription<std_msgs::msg::String>(
    config_profile_topic_,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&VehicleStateJudgeNode::on_config_profile, this, std::placeholders::_1));
}

void VehicleStateJudgeNode::on_config_profile(const std_msgs::msg::String::SharedPtr msg)
{
  ConfigProfileUpdate update;
  if (!parse_config_profile_update(msg->data, update)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignore invalid config profile message: %s", msg->data.c_str());
    return;
  }

  const auto resolved = resolve_config_path(update.fault_config);
  if (update.fault_config == fault_config_path_ && resolved == resolved_fault_config_path_) {
    return;
  }

  fault_config_path_ = update.fault_config;
  RCLCPP_WARN(
    get_logger(),
    "Reload vehicle_state_judge config profile: task=%s fault_config=%s",
    update.task_name.c_str(), fault_config_path_.c_str());
  reset_subscriptions();
  load_configuration();
  configure_subscriptions();
}

void VehicleStateJudgeNode::reset_subscriptions()
{
  command_sub_.reset();
  odom_sub_.reset();
  imu_sub_.reset();
  moto_sub_.reset();
  moto_topic_type_.clear();
  imu_last_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  imu_last_process_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  imu_time_initialized_ = false;
  imu_speed_estimate_ = 0.0;
  imu_acc_bias_ = 0.0;
  imu_bias_calibrated_ = false;
  imu_bias_samples_.clear();
}

void VehicleStateJudgeNode::configure_subscriptions()
{
  if (!cfg_.enabled) {
    return;
  }

  if (!cfg_.command_topic.empty()) {
    command_sub_ = create_subscription<std_msgs::msg::String>(
      cfg_.command_topic,
      rclcpp::QoS(20).reliable().durability_volatile(),
      std::bind(&VehicleStateJudgeNode::on_command, this, std::placeholders::_1));
  }
  if (!cfg_.odom_topic.empty()) {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      cfg_.odom_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&VehicleStateJudgeNode::on_odom, this, std::placeholders::_1));
  }
  if (!cfg_.imu_topic.empty()) {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      cfg_.imu_topic,
      rclcpp::SensorDataQoS().keep_last(5),
      std::bind(&VehicleStateJudgeNode::on_imu, this, std::placeholders::_1));
  }
  try_subscribe_moto_topic();
}

void VehicleStateJudgeNode::try_subscribe_moto_topic()
{
  if (!cfg_.enabled || cfg_.moto_topic.empty() || moto_sub_) {
    return;
  }

  const auto topics = get_topic_names_and_types();
  const auto it = topics.find(cfg_.moto_topic);
  if (it == topics.end() || it->second.empty()) {
    return;
  }

  moto_topic_type_ = it->second.front();
  moto_sub_ = create_generic_subscription(
    cfg_.moto_topic,
    moto_topic_type_,
    rclcpp::SensorDataQoS().keep_last(5),
    [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      on_moto_serialized(*msg);
    });
  RCLCPP_INFO(
    get_logger(), "Subscribed moto feedback: topic=%s type=%s",
    cfg_.moto_topic.c_str(), moto_topic_type_.c_str());
}

void VehicleStateJudgeNode::on_command(const std_msgs::msg::String::SharedPtr msg)
{
  data_store_.set_command_speed(parse_command_speed(msg->data), now());
}

void VehicleStateJudgeNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & v = msg->twist.twist.linear;
  const double speed = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  data_store_.set_odom_speed(speed, stamp_or_now(msg->header.stamp));
}

void VehicleStateJudgeNode::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  update_imu_motion(msg);
}

void VehicleStateJudgeNode::on_moto_serialized(const rclcpp::SerializedMessage & msg)
{
  double left_speed = 0.0;
  double right_speed = 0.0;
  const bool valid = decode_moto_info(msg, left_speed, right_speed);
  data_store_.set_moto_speed(left_speed, right_speed, valid, now());
  if (!valid) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "moto feedback decode failed: topic=%s type=%s bytes=%zu",
      cfg_.moto_topic.c_str(), moto_topic_type_.c_str(), msg.size());
  }
}

void VehicleStateJudgeNode::evaluate_and_publish()
{
  const auto now_time = now();
  if (!cfg_.enabled) {
    publish_state({}, data_store_.get_chassis_state(), now_time);
    return;
  }

  auto faults = evaluator_.evaluate(cfg_, data_store_, now_time);
  publish_state(faults, data_store_.get_chassis_state(), now_time);

  std::set<std::string> current_fault_keys;
  for (const auto & fault : faults) {
    current_fault_keys.insert(fault.fault_key);
    if (active_human_fault_keys_.count(fault.fault_key) == 0) {
      publish_human_intervention_request(fault, now_time);
    }
  }
  active_human_fault_keys_ = std::move(current_fault_keys);
}

void VehicleStateJudgeNode::publish_state(
  const std::vector<FaultInfo> & faults,
  const ChassisRuntimeState & chassis_state,
  const rclcpp::Time & now_time)
{
  const bool healthy = cfg_.enabled && faults.empty();
  std::string level = cfg_.enabled ? "OK" : "UNKNOWN";
  std::string summary = cfg_.enabled ? "vehicle state normal" : "vehicle_state_judge disabled";
  if (!faults.empty()) {
    level = fault_level_to_string(faults.front().level);
    summary = faults.front().reason;
  }

  auto fresh = [&](bool received, const rclcpp::Time & stamp) {
    return received && (now_time - stamp).seconds() <= cfg_.source_timeout_s;
  };
  const bool command_fresh = fresh(chassis_state.command_received, chassis_state.command_stamp);
  const bool moto_fresh = fresh(chassis_state.moto_received, chassis_state.moto_stamp);
  const bool odom_fresh = !cfg_.odom_topic.empty() && fresh(chassis_state.odom_received, chassis_state.odom_stamp);
  const bool imu_fresh = !cfg_.imu_topic.empty() && fresh(chassis_state.imu_received, chassis_state.imu_stamp);
  const bool command_has = command_fresh &&
    std::fabs(chassis_state.command_speed) >= cfg_.command_speed_threshold;
  const bool moto_has = moto_fresh && chassis_state.moto_valid &&
    std::max(std::fabs(chassis_state.left_speed_rad), std::fabs(chassis_state.right_speed_rad)) >=
    cfg_.moto_speed_threshold;
  const bool odom_has = odom_fresh && std::fabs(chassis_state.odom_speed) >= cfg_.odom_speed_threshold;
  const bool imu_has = imu_fresh && (
    std::fabs(chassis_state.imu_speed_estimate) >= cfg_.imu_speed_threshold ||
    std::fabs(chassis_state.imu_yaw_rate) >= cfg_.imu_yaw_rate_threshold);

  std::ostringstream oss;
  oss << '{'
      << "\"stamp\":" << now_time.seconds() << ','
      << "\"source_module\":\"vehicle_state_judge\","
      << "\"state\":\"" << level << "\","
      << "\"healthy\":" << (healthy ? "true" : "false") << ','
      << "\"summary\":\"" << json_escape(summary) << "\","
      << "\"command_received\":" << (chassis_state.command_received ? "true" : "false") << ','
      << "\"command_fresh\":" << (command_fresh ? "true" : "false") << ','
      << "\"command_has_speed\":" << (command_has ? "true" : "false") << ','
      << "\"command_speed\":" << chassis_state.command_speed << ','
      << "\"motion_detected\":" << ((imu_has || odom_has || moto_has) ? "true" : "false") << ','
      << "\"imu_fresh\":" << (imu_fresh ? "true" : "false") << ','
      << "\"imu_motion\":" << (imu_has ? "true" : "false") << ','
      << "\"imu_speed_estimate\":" << chassis_state.imu_speed_estimate << ','
      << "\"imu_yaw_rate\":" << chassis_state.imu_yaw_rate << ','
      << "\"odom_fresh\":" << (odom_fresh ? "true" : "false") << ','
      << "\"odom_motion\":" << (odom_has ? "true" : "false") << ','
      << "\"odom_speed\":" << chassis_state.odom_speed << ','
      << "\"moto_fresh\":" << (moto_fresh ? "true" : "false") << ','
      << "\"moto_valid\":" << (chassis_state.moto_valid ? "true" : "false") << ','
      << "\"moto_motion\":" << (moto_has ? "true" : "false") << ','
      << "\"faults\":[";

  for (size_t i = 0; i < faults.size(); ++i) {
    if (i > 0) {
      oss << ',';
    }
    const auto & fault = faults[i];
    oss << '{'
        << "\"fault_key\":\"" << json_escape(fault.fault_key) << "\","
        << "\"module_name\":\"" << json_escape(fault.module_name) << "\","
        << "\"level\":\"" << fault_level_to_string(fault.level) << "\","
        << "\"type\":\"" << json_escape(fault.fault_type) << "\","
        << "\"fault_type\":\"" << json_escape(fault.fault_type) << "\","
        << "\"fault_model\":\"" << json_escape(fault.fault_model) << "\","
        << "\"fault_name\":\"" << json_escape(fault.fault_name) << "\","
        << "\"action\":\"" << action_to_string(fault.action) << "\","
        << "\"safety_command\":\"" << safety_command_to_string(fault.safety_command) << "\","
        << "\"safety_slow_down_percentage\":" << fault.safety_slow_down_percentage << ','
        << "\"reason\":\"" << json_escape(fault.reason) << "\""
        << '}';
  }
  oss << "]}";

  std_msgs::msg::String msg;
  msg.data = oss.str();
  state_pub_->publish(msg);
}

void VehicleStateJudgeNode::publish_human_intervention_request(
  const FaultInfo & fault,
  const rclcpp::Time & now_time)
{
  std_msgs::msg::String msg;
  std::ostringstream oss;
  oss << '{'
      << "\"stamp\":" << now_time.seconds() << ','
      << "\"source_module\":\"vehicle_state_judge\","
      << "\"fault_key\":\"" << json_escape(fault.fault_key) << "\","
      << "\"level\":\"" << fault_level_to_string(fault.level) << "\","
      << "\"reason\":\"" << json_escape(fault.reason) << "\","
      << "\"request\":\"human_intervention\""
      << '}';
  msg.data = oss.str();
  human_intervention_pub_->publish(msg);
  RCLCPP_ERROR(
    get_logger(), "Vehicle state requires human intervention: %s", fault.reason.c_str());
}

double VehicleStateJudgeNode::parse_command_speed(const std::string & payload) const
{
  auto key_pos = payload.find("\"speed\"");
  if (key_pos == std::string::npos) {
    return 0.0;
  }
  auto colon_pos = payload.find(':', key_pos);
  if (colon_pos == std::string::npos) {
    return 0.0;
  }
  auto value_begin = payload.find_first_not_of(" \t\r\n", colon_pos + 1);
  if (value_begin == std::string::npos) {
    return 0.0;
  }
  auto value_end = payload.find_first_of(",}", value_begin);
  const std::string token = payload.substr(value_begin, value_end - value_begin);
  try {
    return std::stod(token);
  } catch (...) {
    return 0.0;
  }
}

bool VehicleStateJudgeNode::decode_moto_info(
  const rclcpp::SerializedMessage & msg,
  double & left_speed,
  double & right_speed) const
{
  const auto & serialized = msg.get_rcl_serialized_message();
  const auto * buffer = serialized.buffer;
  const size_t len = serialized.buffer_length;
  if (buffer == nullptr || len < 32) {
    return false;
  }

  const size_t offsets[] = {8, 4, 0};
  for (size_t offset : offsets) {
    if (len < offset + 32) {
      continue;
    }
    double candidate_left = 0.0;
    double candidate_right = 0.0;
    std::memcpy(&candidate_left, buffer + offset + 16, sizeof(double));
    std::memcpy(&candidate_right, buffer + offset + 24, sizeof(double));
    if (std::isfinite(candidate_left) && std::isfinite(candidate_right) &&
      std::fabs(candidate_left) < 1e6 && std::fabs(candidate_right) < 1e6)
    {
      left_speed = candidate_left;
      right_speed = candidate_right;
      return true;
    }
  }
  return false;
}

void VehicleStateJudgeNode::update_imu_motion(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const auto stamp = stamp_or_now(msg->header.stamp);
  const auto receive_time = now();
  if (!imu_time_initialized_) {
    imu_last_stamp_ = stamp;
    imu_last_process_time_ = receive_time;
    imu_time_initialized_ = true;
    data_store_.set_imu_motion(0.0, msg->angular_velocity.z, stamp);
    return;
  }

  if ((receive_time - imu_last_process_time_).seconds() < 0.02) {
    return;
  }
  imu_last_process_time_ = receive_time;

  const double dt = (stamp - imu_last_stamp_).seconds();
  imu_last_stamp_ = stamp;
  if (dt <= 0.0 || dt > 0.5) {
    return;
  }

  const double accel_x = msg->linear_acceleration.x;
  const double yaw_rate = msg->angular_velocity.z;
  const bool static_motion =
    std::fabs(imu_speed_estimate_) < cfg_.imu_static_command_threshold &&
    std::fabs(yaw_rate) < cfg_.imu_yaw_rate_threshold;

  if (!imu_bias_calibrated_ && static_motion && std::isfinite(accel_x)) {
    imu_bias_samples_.push_back(accel_x);
    if (static_cast<int>(imu_bias_samples_.size()) > cfg_.imu_bias_calibration_samples) {
      imu_bias_samples_.erase(imu_bias_samples_.begin());
    }
    if (static_cast<int>(imu_bias_samples_.size()) >= cfg_.imu_bias_calibration_samples) {
      double sum = 0.0;
      for (const auto sample : imu_bias_samples_) {
        sum += sample;
      }
      imu_acc_bias_ = sum / static_cast<double>(imu_bias_samples_.size());
      imu_bias_calibrated_ = true;
    }
  }

  if (imu_bias_calibrated_) {
    const double corrected_accel = accel_x - imu_acc_bias_;
    imu_speed_estimate_ += corrected_accel * dt;
    imu_speed_estimate_ *= cfg_.imu_decay_rate;
    imu_speed_estimate_ = std::clamp(imu_speed_estimate_, -3.0, 3.0);
    if (static_motion) {
      imu_speed_estimate_ *= 0.5;
      if (std::fabs(imu_speed_estimate_) < cfg_.imu_speed_threshold) {
        imu_speed_estimate_ = 0.0;
      }
    }
  }

  data_store_.set_imu_motion(imu_speed_estimate_, yaw_rate, stamp);
}

rclcpp::Time VehicleStateJudgeNode::stamp_or_now(const builtin_interfaces::msg::Time & stamp) const
{
  return (stamp.sec != 0 || stamp.nanosec != 0) ? rclcpp::Time(stamp) : now();
}

std::string VehicleStateJudgeNode::json_escape(const std::string & input)
{
  std::ostringstream oss;
  for (const auto ch : input) {
    switch (ch) {
      case '"': oss << "\\\""; break;
      case '\\': oss << "\\\\"; break;
      case '\n': oss << "\\n"; break;
      case '\r': oss << "\\r"; break;
      case '\t': oss << "\\t"; break;
      default: oss << ch; break;
    }
  }
  return oss.str();
}

std::string VehicleStateJudgeNode::fault_level_to_string(FaultLevel level)
{
  switch (level) {
    case FaultLevel::WARNING:
      return "WARNING";
    case FaultLevel::ERROR:
      return "ERROR";
    case FaultLevel::CRITICAL:
      return "CRITICAL";
    case FaultLevel::NORMAL:
    default:
      return "NORMAL";
  }
}

std::string VehicleStateJudgeNode::action_to_string(ActionType action)
{
  switch (action) {
    case ActionType::SUPERVISOR:
      return "NODEMANAGER";
    case ActionType::SAFETY_SYSTEM:
      return "SAFETY_SYSTEM";
    case ActionType::NONE:
    default:
      return "NONE";
  }
}

std::string VehicleStateJudgeNode::safety_command_to_string(SafetyCommandType command)
{
  switch (command) {
    case SafetyCommandType::SLOW_DOWN:
      return "SLOW_DOWN";
    case SafetyCommandType::SOFT_STOP:
      return "SOFT_STOP";
    case SafetyCommandType::EMERGENCY_STOP:
      return "EMERGENCY_STOP";
    case SafetyCommandType::NONE:
    default:
      return "NONE";
  }
}

}  // namespace nav2_monitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_monitor::VehicleStateJudgeNode>());
  rclcpp::shutdown();
  return 0;
}
