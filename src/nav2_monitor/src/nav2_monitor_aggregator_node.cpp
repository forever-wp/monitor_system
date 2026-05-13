#include "nav2_monitor/nav2_monitor_aggregator_node.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sstream>

#include "nav2_monitor/config_profile_sync.hpp"

namespace nav2_monitor
{

namespace
{
std::string to_lower_copy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

uint8_t fault_level_to_msg(FaultLevel level)
{
  switch (level) {
    case FaultLevel::WARNING:
      return msg::FaultEvent::WARNING;
    case FaultLevel::ERROR:
      return msg::FaultEvent::ERROR;
    case FaultLevel::CRITICAL:
      return msg::FaultEvent::CRITICAL;
    case FaultLevel::NORMAL:
    default:
      return msg::FaultEvent::NORMAL;
  }
}

uint8_t action_to_msg(ActionType action)
{
  switch (action) {
    case ActionType::SUPERVISOR:
      return msg::FaultEvent::NODEMANAGER;
    case ActionType::SAFETY_SYSTEM:
      return msg::FaultEvent::SAFETY_SYSTEM;
    case ActionType::NONE:
    default:
      return msg::FaultEvent::NONE;
  }
}

std::string fault_level_to_string(FaultLevel level)
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

std::string json_escape(const std::string & input)
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

std::optional<std::string> extract_json_string_field(const std::string & json, const std::string & key)
{
  const std::string marker = "\"" + key + "\":";
  const auto marker_pos = json.find(marker);
  if (marker_pos == std::string::npos) {
    return std::nullopt;
  }
  auto pos = json.find('"', marker_pos + marker.size());
  if (pos == std::string::npos) {
    return std::nullopt;
  }
  ++pos;

  std::string value;
  bool escaped = false;
  for (; pos < json.size(); ++pos) {
    const char ch = json[pos];
    if (escaped) {
      switch (ch) {
        case 'n': value.push_back('\n'); break;
        case 'r': value.push_back('\r'); break;
        case 't': value.push_back('\t'); break;
        default: value.push_back(ch); break;
      }
      escaped = false;
      continue;
    }
    if (ch == '\\') {
      escaped = true;
      continue;
    }
    if (ch == '"') {
      return value;
    }
    value.push_back(ch);
  }
  return std::nullopt;
}

std::optional<double> extract_json_number_field(const std::string & json, const std::string & key)
{
  const std::string marker = "\"" + key + "\":";
  const auto marker_pos = json.find(marker);
  if (marker_pos == std::string::npos) {
    return std::nullopt;
  }
  auto pos = marker_pos + marker.size();
  while (pos < json.size() && std::isspace(static_cast<unsigned char>(json[pos]))) {
    ++pos;
  }
  const char * begin = json.c_str() + pos;
  char * end = nullptr;
  const double value = std::strtod(begin, &end);
  if (end == begin) {
    return std::nullopt;
  }
  return value;
}

std::optional<bool> extract_json_bool_field(const std::string & json, const std::string & key)
{
  const std::string marker = "\"" + key + "\":";
  const auto marker_pos = json.find(marker);
  if (marker_pos == std::string::npos) {
    return std::nullopt;
  }
  auto pos = marker_pos + marker.size();
  while (pos < json.size() && std::isspace(static_cast<unsigned char>(json[pos]))) {
    ++pos;
  }
  if (json.compare(pos, 4, "true") == 0) {
    return true;
  }
  if (json.compare(pos, 5, "false") == 0) {
    return false;
  }
  return std::nullopt;
}

bool is_vehicle_state_judge_fault(const FaultInfo & fault)
{
  return fault.module_name == "vehicle_state_judge" ||
         fault.fault_key.find("vehicle_state_judge|") == 0 ||
         fault.fault_key.find("|vehicle_state_") != std::string::npos;
}

ActionType parse_action_type(const std::string & action)
{
  if (action == "SAFETY_SYSTEM" || action == "safety_system" || action == "2") {
    return ActionType::SAFETY_SYSTEM;
  }
  if (
    action == "NODEMANAGER" || action == "SUPERVISOR" ||
    action == "nodemanager" || action == "supervisor" || action == "1")
  {
    return ActionType::SUPERVISOR;
  }
  return ActionType::NONE;
}

FaultLevel parse_fault_level(const std::string & level)
{
  if (level == "WARNING" || level == "WARN" || level == "warning" || level == "warn") {
    return FaultLevel::WARNING;
  }
  if (level == "CRITICAL" || level == "critical") {
    return FaultLevel::CRITICAL;
  }
  if (level == "NORMAL" || level == "OK" || level == "normal" || level == "ok") {
    return FaultLevel::NORMAL;
  }
  return FaultLevel::ERROR;
}

SafetyCommandType parse_safety_command_type(const std::string & command)
{
  if (command == "SLOW_DOWN" || command == "slow_down" || command == "1") {
    return SafetyCommandType::SLOW_DOWN;
  }
  if (command == "SOFT_STOP" || command == "soft_stop" || command == "2") {
    return SafetyCommandType::SOFT_STOP;
  }
  if (command == "EMERGENCY_STOP" || command == "emergency_stop" || command == "3") {
    return SafetyCommandType::EMERGENCY_STOP;
  }
  return SafetyCommandType::NONE;
}

std::vector<FaultInfo> parse_fault_items_json(
  const std::string & json,
  const std::string & default_module,
  const rclcpp::Time & now)
{
  std::vector<FaultInfo> parsed_faults;
  size_t pos = 0;
  while (true) {
    const auto key_pos = json.find("\"fault_key\":", pos);
    if (key_pos == std::string::npos) {
      break;
    }
    const auto item_begin = json.rfind('{', key_pos);
    const auto item_end = json.find('}', key_pos);
    if (item_begin == std::string::npos || item_end == std::string::npos) {
      break;
    }
    pos = item_end + 1;
    const std::string item_json = json.substr(item_begin, item_end - item_begin + 1);

    const auto fault_key = extract_json_string_field(item_json, "fault_key");
    if (!fault_key || fault_key->empty()) {
      continue;
    }

    FaultInfo fault;
    fault.fault_key = *fault_key;
    fault.module_name = extract_json_string_field(item_json, "module_name").value_or(default_module);
    fault.level = parse_fault_level(extract_json_string_field(item_json, "level").value_or("ERROR"));
    fault.reason = extract_json_string_field(item_json, "reason").value_or("External monitor fault");
    fault.fault_type = extract_json_string_field(item_json, "fault_type").value_or("external_state");
    fault.fault_model = extract_json_string_field(item_json, "fault_model").value_or(default_module);
    fault.fault_name = extract_json_string_field(item_json, "fault_name").value_or(*fault_key);
    fault.action = parse_action_type(extract_json_string_field(item_json, "action").value_or("NONE"));
    fault.safety_command = parse_safety_command_type(
      extract_json_string_field(item_json, "safety_command").value_or("NONE"));
    fault.safety_slow_down_percentage = extract_json_number_field(
      item_json, "safety_slow_down_percentage").value_or(0.0);
    fault.timestamp = now;
    parsed_faults.push_back(std::move(fault));
  }
  return parsed_faults;
}

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

std::map<std::string, std::string> default_task_status_code_mappings()
{
  return {
    {"100", "default"},
    {"101", "default"},
    {"102", "default"},
    {"103", "default"},
    {"109", "default"},
    {"200", "elevator"},
    {"201", "elevator"},
    {"202", "elevator"},
    {"203", "elevator"},
    {"209", "elevator"},
    {"300", "todoor"},
    {"301", "todoor"},
    {"302", "todoor"},
    {"303", "todoor"},
    {"304", "todoor"},
    {"400", "default"},
    {"401", "default"},
    {"402", "default"},
    {"403", "default"}
  };
}

rclcpp::ReliabilityPolicy parse_reliability_policy(
  const std::string & raw,
  bool & ok)
{
  const std::string value = to_lower_copy(raw);
  ok = true;
  if (value == "reliable") {
    return rclcpp::ReliabilityPolicy::Reliable;
  }
  if (value == "best_effort" || value == "besteffort") {
    return rclcpp::ReliabilityPolicy::BestEffort;
  }
  if (value == "system_default" || value == "default") {
    return rclcpp::ReliabilityPolicy::SystemDefault;
  }
  ok = false;
  return rclcpp::ReliabilityPolicy::BestEffort;
}

rclcpp::DurabilityPolicy parse_durability_policy(
  const std::string & raw,
  bool & ok)
{
  const std::string value = to_lower_copy(raw);
  ok = true;
  if (value == "transient_local" || value == "transientlocal") {
    return rclcpp::DurabilityPolicy::TransientLocal;
  }
  if (value == "volatile") {
    return rclcpp::DurabilityPolicy::Volatile;
  }
  if (value == "system_default" || value == "default") {
    return rclcpp::DurabilityPolicy::SystemDefault;
  }
  ok = false;
  return rclcpp::DurabilityPolicy::Volatile;
}

std::string reliability_to_string(rclcpp::ReliabilityPolicy reliability)
{
  switch (reliability) {
    case rclcpp::ReliabilityPolicy::Reliable:
      return "reliable";
    case rclcpp::ReliabilityPolicy::BestEffort:
      return "best_effort";
    case rclcpp::ReliabilityPolicy::SystemDefault:
      return "system_default";
    default:
      return "unknown";
  }
}

std::string durability_to_string(rclcpp::DurabilityPolicy durability)
{
  switch (durability) {
    case rclcpp::DurabilityPolicy::TransientLocal:
      return "transient_local";
    case rclcpp::DurabilityPolicy::Volatile:
      return "volatile";
    case rclcpp::DurabilityPolicy::SystemDefault:
      return "system_default";
    default:
      return "unknown";
  }
}
}  // namespace

Nav2MonitorAggregatorNode::Nav2MonitorAggregatorNode()
: Node("nav2_monitor"), timeout_(5.0), safety_cooldown_s_(2.0), nodemanager_cooldown_s_(5.0),
  sys_monitor_(), fault_detector_(this)
{
  timeout_ = this->declare_parameter<double>("timeout", 5.0);
  double scan_rate = this->declare_parameter<double>("scan_rate", 0.5);
  double check_rate = this->declare_parameter<double>("check_rate", 1.0);
  safety_cooldown_s_ = this->declare_parameter<double>("safety_cooldown_s", 2.0);
  const double legacy_supervisor_cooldown_s =
    this->declare_parameter<double>("supervisor_cooldown_s", 5.0);
  nodemanager_cooldown_s_ =
    this->declare_parameter<double>("nodemanager_cooldown_s", legacy_supervisor_cooldown_s);
  topic_states_topic_ = this->declare_parameter<std::string>(
    "topic_states_topic", "/monitor/topic_states");
  vehicle_state_topic_ = this->declare_parameter<std::string>(
    "vehicle_state_topic", "/monitor/vehicle_state");
  vehicle_state_timeout_s_ = std::max(
    0.1, this->declare_parameter<double>("vehicle_state_timeout_s", 1.0));
  node_tf_state_topic_ = this->declare_parameter<std::string>(
    "node_tf_state_topic", "/monitor/node_tf_state");
  node_tf_state_timeout_s_ = std::max(
    0.1, this->declare_parameter<double>("node_tf_state_timeout_s", 2.0));
  monitor_battery_state_topic_ = this->declare_parameter<std::string>(
    "monitor_battery_state_topic", "/monitor/battery_state");
  monitor_battery_state_timeout_s_ = std::max(
    0.1, this->declare_parameter<double>("monitor_battery_state_timeout_s", 3.0));
  feedback_state_topic_ = this->declare_parameter<std::string>(
    "feedback_state_topic", "/monitor/feedback_state");
  feedback_state_timeout_s_ = std::max(
    0.1, this->declare_parameter<double>("feedback_state_timeout_s", 2.0));
  collision_state_topic_ = this->declare_parameter<std::string>(
    "collision_state_topic", "/monitor/collision_state");
  collision_state_timeout_s_ = std::max(
    0.1, this->declare_parameter<double>("collision_state_timeout_s", 1.0));
  config_profile_topic_ = this->declare_parameter<std::string>(
    "config_profile_topic", "/monitor/config_profile");
  const auto fault_event_topic = this->declare_parameter<std::string>(
    "fault_event_topic", "/nav2_monitor/fault_event");
  const auto legacy_supervisor_cmd_topic = this->declare_parameter<std::string>(
    "supervisor_cmd_topic", "/supervisor/cmd");
  const auto nodemanager_cmd_topic = this->declare_parameter<std::string>(
    "nodemanager_cmd_topic", legacy_supervisor_cmd_topic == "/supervisor/cmd" ?
    "/nodemanager/cmd" : legacy_supervisor_cmd_topic);
  const auto safety_cmd_topic = this->declare_parameter<std::string>(
    "safety_cmd_topic", "/safety_system/cmd");
  const auto safety_cmd_republish_period_s = std::max(
    0.05, this->declare_parameter<double>("safety_cmd_republish_period_s", 0.2));
  const auto human_intervention_topic = this->declare_parameter<std::string>(
    "human_intervention_topic", "/nav2_monitor/human_intervention");
  load_topic_qos_overrides();

  fallback_target_nodes_ = this->declare_parameter<std::vector<std::string>>(
    "target_nodes", std::vector<std::string>{});
  fallback_watch_topics_ = this->declare_parameter<std::vector<std::string>>(
    "watch_topics", std::vector<std::string>{});
  target_nodes_ = fallback_target_nodes_;
  watch_topics_ = fallback_watch_topics_;

  auto tf_strs = this->declare_parameter<std::vector<std::string>>(
    "target_transforms", std::vector<std::string>{});
  for (const auto & tf_str : tf_strs) {
    auto pos = tf_str.find("->");
    if (pos != std::string::npos) {
      target_transforms_.push_back({tf_str.substr(0, pos), tf_str.substr(pos + 2)});
    }
  }

  if (scan_rate <= 0.0) {
    scan_rate = 0.5;
  }
  if (check_rate <= 0.0) {
    check_rate = 1.0;
  }

  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  default_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  pub_ = this->create_publisher<msg::MonitorStatus>("/nav2_monitor/status", 10);
  fault_event_pub_ = this->create_publisher<msg::FaultEvent>(fault_event_topic, 10);
  config_profile_pub_ = this->create_publisher<std_msgs::msg::String>(
    config_profile_topic_, rclcpp::QoS(1).reliable().transient_local());
  nodemanager_pub_ = this->create_publisher<std_msgs::msg::String>(nodemanager_cmd_topic, 10);
  human_intervention_pub_ =
    this->create_publisher<std_msgs::msg::String>(human_intervention_topic, 10);
  fault_state_coordinator_.configure(this, safety_cmd_topic, safety_cmd_republish_period_s);
  monitor_reporter_.configure(this);

  base_fault_config_path_ = this->declare_parameter<std::string>("fault_config", "");
  fault_config_reload_enabled_ = this->declare_parameter<bool>("fault_config_reload_enabled", true);
  current_nav_task_ = this->declare_parameter<std::string>("current_nav_task", "default");
  task_status_topic_ = this->declare_parameter<std::string>("task_status_topic", "/task_status_code");
  resolved_base_fault_config_path_ = resolve_config_path(base_fault_config_path_);
  load_task_fault_config_mappings();
  load_task_status_code_mappings();
  task_fault_config_selector_.update_current_task(current_nav_task_);
  fault_detector_.set_feedback_default_max_stale(timeout_);
  update_task_selected_fault_config(true);

  configure_topic_state_subscription();
  configure_vehicle_state_subscription();
  configure_node_tf_state_subscription();
  configure_battery_state_subscription();
  configure_feedback_state_subscription();
  configure_collision_state_subscription();
  configure_task_status_subscription();

  std::string vehicle_status_file = this->declare_parameter<std::string>(
    "vehicle_status_file", "/home/ry/.ros/navigate_status/navigate_todoor_status.json");
  vehicle_monitor_ = std::make_unique<VehicleStatusMonitor>(vehicle_status_file);

  scan_topology();

  scan_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / scan_rate)),
    std::bind(&Nav2MonitorAggregatorNode::scan_topology, this),
    timer_callback_group_);

  check_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / check_rate)),
    std::bind(&Nav2MonitorAggregatorNode::check_health, this),
    timer_callback_group_);

  param_callback_ = this->add_on_set_parameters_callback(
    std::bind(&Nav2MonitorAggregatorNode::on_parameter_change, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "nav2_monitor started");
}

void Nav2MonitorAggregatorNode::on_topic_states(const std_msgs::msg::String::SharedPtr msg)
{
  const auto now = this->now();
  size_t parsed_count = 0;
  size_t applied_count = 0;
  size_t pos = 0;
  while (true) {
    const auto topic_pos = msg->data.find("\"topic\":", pos);
    if (topic_pos == std::string::npos) {
      break;
    }
    const auto item_begin = msg->data.rfind('{', topic_pos);
    if (item_begin == std::string::npos) {
      pos = topic_pos + 8;
      continue;
    }
    const auto item_end = msg->data.find('}', topic_pos);
    if (item_end == std::string::npos) {
      break;
    }
    pos = item_end + 1;
    const std::string item_json = msg->data.substr(item_begin, item_end - item_begin + 1);
    ++parsed_count;

    const auto topic = extract_json_string_field(item_json, "topic");
    if (!topic || topic->empty()) {
      continue;
    }

    const bool has_data = extract_json_bool_field(item_json, "has_data").value_or(false);
    const bool has_publisher =
      extract_json_bool_field(item_json, "has_publisher").value_or(has_data);
    const bool stale = extract_json_bool_field(item_json, "stale").value_or(!has_data);
    const double frequency = std::max(
      0.0, extract_json_number_field(item_json, "frequency_hz").value_or(0.0));
    const double age_s = extract_json_number_field(item_json, "age_s").value_or(-1.0);
    const rclcpp::Time last_received =
      has_data && age_s >= 0.0 ? now - rclcpp::Duration::from_seconds(age_s) : now;
    data_store_.set_watch_topic_publisher(*topic, has_publisher);
    data_store_.set_watch_topic_observation(
      *topic, stale ? 0.0 : frequency, last_received, last_received, has_data, 0);
    ++applied_count;
  }

  if (parsed_count == 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignore topic state message without items from %s", topic_states_topic_.c_str());
  } else if (applied_count == 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Topic state message parsed %zu items but no usable topic field", parsed_count);
  }
}

void Nav2MonitorAggregatorNode::on_vehicle_state(const std_msgs::msg::String::SharedPtr msg)
{
  const auto now_time = this->now();
  std::vector<FaultInfo> parsed_faults;
  size_t pos = 0;
  while (true) {
    const auto key_pos = msg->data.find("\"fault_key\":", pos);
    if (key_pos == std::string::npos) {
      break;
    }
    const auto item_begin = msg->data.rfind('{', key_pos);
    if (item_begin == std::string::npos) {
      pos = key_pos + 12;
      continue;
    }
    const auto item_end = msg->data.find('}', key_pos);
    if (item_end == std::string::npos) {
      break;
    }
    pos = item_end + 1;
    const std::string item_json = msg->data.substr(item_begin, item_end - item_begin + 1);

    const auto fault_key = extract_json_string_field(item_json, "fault_key");
    if (!fault_key || fault_key->empty()) {
      continue;
    }
    const auto type = extract_json_string_field(item_json, "type").value_or("vehicle_state_anomaly");
    const auto action = extract_json_string_field(item_json, "action").value_or("NONE");
    const auto reason = extract_json_string_field(item_json, "reason").value_or("Vehicle state fault");

    FaultInfo fault;
    fault.fault_key = *fault_key;
    fault.module_name = "vehicle_state_judge";
    fault.level = FaultLevel::ERROR;
    const auto level = extract_json_string_field(item_json, "level").value_or("ERROR");
    if (level == "WARNING") {
      fault.level = FaultLevel::WARNING;
    } else if (level == "CRITICAL") {
      fault.level = FaultLevel::CRITICAL;
    } else if (level == "NORMAL" || level == "OK") {
      fault.level = FaultLevel::NORMAL;
    }
    fault.reason = reason;
    fault.fault_type = type;
    fault.fault_model = "vehicle_state";
    fault.fault_name = type;
    fault.action = (action == "NODEMANAGER" || action == "SUPERVISOR") ?
      ActionType::SUPERVISOR : ActionType::NONE;
    fault.safety_command = SafetyCommandType::NONE;
    fault.safety_slow_down_percentage = 0.0;
    fault.timestamp = now_time;
    parsed_faults.push_back(std::move(fault));
  }

  {
    std::lock_guard<std::mutex> lock(mtx_);
    external_vehicle_faults_ = std::move(parsed_faults);
    last_vehicle_state_time_ = now_time;
  }
}

void Nav2MonitorAggregatorNode::on_node_tf_state(const std_msgs::msg::String::SharedPtr msg)
{
  const auto now_time = this->now();
  std::map<std::string, bool> node_active;
  std::map<std::pair<std::string, std::string>, TransformInfo> tf_info;

  size_t pos = 0;
  while (true) {
    const auto name_pos = msg->data.find("\"name\":", pos);
    if (name_pos == std::string::npos) {
      break;
    }
    const auto item_begin = msg->data.rfind('{', name_pos);
    const auto item_end = msg->data.find('}', name_pos);
    if (item_begin == std::string::npos || item_end == std::string::npos) {
      break;
    }
    pos = item_end + 1;
    const std::string item_json = msg->data.substr(item_begin, item_end - item_begin + 1);
    const auto name = extract_json_string_field(item_json, "name");
    if (!name || name->empty()) {
      continue;
    }
    if (const auto active = extract_json_bool_field(item_json, "active")) {
      node_active[*name] = *active;
      continue;
    }
    if (const auto available = extract_json_bool_field(item_json, "available")) {
      const auto arrow = name->find("->");
      if (arrow == std::string::npos) {
        continue;
      }
      TransformInfo info;
      info.last_update = *available ? now_time : rclcpp::Time(0, 0, RCL_ROS_TIME);
      info.latency_ms = extract_json_number_field(item_json, "latency_ms").value_or(-1.0);
      tf_info[{name->substr(0, arrow), name->substr(arrow + 2)}] = info;
    }
  }

  std::lock_guard<std::mutex> lock(mtx_);
  external_node_active_ = std::move(node_active);
  external_tf_info_ = std::move(tf_info);
  last_node_tf_state_time_ = now_time;
}

void Nav2MonitorAggregatorNode::on_monitor_battery_state(const std_msgs::msg::String::SharedPtr msg)
{
  const auto now_time = this->now();
  const bool has_data = extract_json_bool_field(msg->data, "has_data").value_or(false);
  const bool stale = extract_json_bool_field(msg->data, "stale").value_or(!has_data);
  {
    std::lock_guard<std::mutex> lock(mtx_);
    external_battery_state_seen_ = true;
    external_battery_source_has_data_ = has_data;
    external_battery_source_stale_ = stale;
    last_monitor_battery_state_time_ = now_time;
  }
  if (!has_data || stale) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "External battery state unavailable: topic=%s has_data=%s stale=%s",
      monitor_battery_state_topic_.c_str(),
      has_data ? "true" : "false",
      stale ? "true" : "false");
    return;
  }

  const auto temperature = extract_json_number_field(msg->data, "temperature");
  const auto percentage = extract_json_number_field(msg->data, "percentage");
  if (!temperature || !percentage) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "External battery state missing temperature/percentage fields: topic=%s",
      monitor_battery_state_topic_.c_str());
    return;
  }

  const double age_s = extract_json_number_field(msg->data, "age_s").value_or(0.0);
  const auto stamp = age_s >= 0.0 ? now_time - rclcpp::Duration::from_seconds(age_s) : now_time;
  data_store_.set_battery_state(
    static_cast<float>(*temperature),
    static_cast<float>(*percentage),
    stamp);
}

void Nav2MonitorAggregatorNode::on_feedback_state(const std_msgs::msg::String::SharedPtr msg)
{
  const auto now_time = this->now();
  auto faults = parse_fault_items_json(msg->data, "algorithm_feedback_monitor", now_time);
  {
    std::lock_guard<std::mutex> lock(mtx_);
    external_feedback_faults_ = std::move(faults);
    last_feedback_state_time_ = now_time;
  }
}

void Nav2MonitorAggregatorNode::on_collision_state(const std_msgs::msg::String::SharedPtr msg)
{
  const auto now_time = this->now();
  auto faults = parse_fault_items_json(msg->data, "collision_monitor", now_time);
  {
    std::lock_guard<std::mutex> lock(mtx_);
    external_collision_faults_ = std::move(faults);
    last_collision_state_time_ = now_time;
  }
}

rclcpp::SubscriptionOptions Nav2MonitorAggregatorNode::make_subscription_options(
  const rclcpp::CallbackGroup::SharedPtr & callback_group) const
{
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group;
  return options;
}

void Nav2MonitorAggregatorNode::on_task_status(const master_interfaces::msg::TaskStatus::SharedPtr msg)
{
  const std::string raw_code = msg ? TaskStatusMessageAdapter::extract_code(*msg) : std::string();
  const std::string mapped_task = task_status_mapper_.resolve_task_for_code(raw_code);
  if (mapped_task.empty()) {
    RCLCPP_WARN(
      get_logger(), "Ignore task_status code '%s': no task mapping configured", raw_code.c_str());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (mapped_task == current_nav_task_) {
      RCLCPP_DEBUG(
        get_logger(), "Ignore task_status code '%s': current_nav_task already '%s'",
        raw_code.c_str(), current_nav_task_.c_str());
      return;
    }
    pending_task_switch_source_ = std::string("task_status code '") + raw_code + "'";
  }

  const auto result = this->set_parameters_atomically({rclcpp::Parameter("current_nav_task", mapped_task)});

  {
    std::lock_guard<std::mutex> lock(mtx_);
    pending_task_switch_source_.clear();
  }

  if (!result.successful) {
    RCLCPP_WARN(
      get_logger(), "Failed to apply task_status code '%s' -> '%s': %s",
      raw_code.c_str(), mapped_task.c_str(), result.reason.c_str());
  }
}

rclcpp::QoS Nav2MonitorAggregatorNode::build_topic_subscription_qos(
  const std::string & topic, const rclcpp::QoS & fallback, size_t max_depth) const
{
  auto qos = rclcpp::QoS(
    std::max<size_t>(1, std::min<size_t>(fallback.get_rmw_qos_profile().depth, max_depth)));
  qos.history(rclcpp::HistoryPolicy::KeepLast);
  qos.reliability(fallback.reliability());
  qos.durability(fallback.durability());

  try {
    auto infos = this->get_publishers_info_by_topic(topic);
    if (infos.empty()) {
      return apply_topic_qos_override(topic, qos, max_depth);
    }

    size_t depth = std::max<size_t>(1, qos.get_rmw_qos_profile().depth);
    bool saw_reliable = false;
    bool saw_best_effort = false;
    bool saw_transient_local = false;
    bool saw_volatile = false;

    for (const auto & info : infos) {
      const auto & profile = info.qos_profile();
      depth = std::max<size_t>(depth, std::max<size_t>(1, profile.depth()));
      if (profile.reliability() == rclcpp::ReliabilityPolicy::Reliable) {
        saw_reliable = true;
      } else if (profile.reliability() == rclcpp::ReliabilityPolicy::BestEffort) {
        saw_best_effort = true;
      }
      if (profile.durability() == rclcpp::DurabilityPolicy::TransientLocal) {
        saw_transient_local = true;
      } else if (profile.durability() == rclcpp::DurabilityPolicy::Volatile) {
        saw_volatile = true;
      }
    }

    depth = std::max<size_t>(1, std::min<size_t>(depth, max_depth));
    qos = rclcpp::QoS(depth);
    qos.history(rclcpp::HistoryPolicy::KeepLast);
    qos.reliability(
      saw_best_effort ? rclcpp::ReliabilityPolicy::BestEffort :
      (saw_reliable ? rclcpp::ReliabilityPolicy::Reliable : fallback.reliability()));
    qos.durability(
      saw_volatile ? rclcpp::DurabilityPolicy::Volatile :
      (saw_transient_local ? rclcpp::DurabilityPolicy::TransientLocal : fallback.durability()));
  } catch (const std::exception &) {
  }

  return apply_topic_qos_override(topic, qos, max_depth);
}

std::optional<Nav2MonitorAggregatorNode::TopicQosOverride> Nav2MonitorAggregatorNode::find_topic_qos_override(
  const std::string & topic) const
{
  const auto exact_it = topic_qos_overrides_.find(topic);
  if (exact_it != topic_qos_overrides_.end()) {
    return exact_it->second;
  }

  if (!topic.empty() && topic.front() == '/') {
    const auto no_slash_it = topic_qos_overrides_.find(topic.substr(1));
    if (no_slash_it != topic_qos_overrides_.end()) {
      return no_slash_it->second;
    }
  } else {
    const auto slash_it = topic_qos_overrides_.find("/" + topic);
    if (slash_it != topic_qos_overrides_.end()) {
      return slash_it->second;
    }
  }
  return std::nullopt;
}

rclcpp::QoS Nav2MonitorAggregatorNode::apply_topic_qos_override(
  const std::string & topic,
  const rclcpp::QoS & qos,
  size_t max_depth) const
{
  const auto override = find_topic_qos_override(topic);
  if (!override.has_value()) {
    return qos;
  }

  const auto raw_depth = override->has_depth ?
    override->depth : std::max<size_t>(1, qos.get_rmw_qos_profile().depth);
  auto out = rclcpp::QoS(std::max<size_t>(1, std::min<size_t>(raw_depth, max_depth)));
  out.history(rclcpp::HistoryPolicy::KeepLast);
  out.reliability(override->has_reliability ? override->reliability : qos.reliability());
  out.durability(override->has_durability ? override->durability : qos.durability());
  RCLCPP_INFO(
    get_logger(),
    "Apply QoS override: topic=%s reliability=%s durability=%s depth=%zu",
    topic.c_str(),
    reliability_to_string(out.reliability()).c_str(),
    durability_to_string(out.durability()).c_str(),
    static_cast<size_t>(out.get_rmw_qos_profile().depth));
  return out;
}

bool Nav2MonitorAggregatorNode::should_publish_action(
  const std::string & module_name, ActionType action, const rclcpp::Time & now)
{
  double cooldown_s = nodemanager_cooldown_s_;
  if (action == ActionType::SAFETY_SYSTEM) {
    cooldown_s = safety_cooldown_s_;
  } else if (action == ActionType::SUPERVISOR) {
    cooldown_s = nodemanager_cooldown_s_;
  }

  const std::string key = module_name + ":" + std::to_string(static_cast<int>(action));
  auto it = last_action_publish_time_.find(key);
  if (it != last_action_publish_time_.end() && (now - it->second).seconds() < cooldown_s) {
    return false;
  }

  last_action_publish_time_[key] = now;
  return true;
}

void Nav2MonitorAggregatorNode::publish_human_intervention_request(
  const FaultInfo & fault, const rclcpp::Time & now)
{
  if (!human_intervention_pub_) {
    return;
  }

  std_msgs::msg::String msg;
  std::ostringstream oss;
  oss << '{'
      << "\"timestamp\":\"" << json_escape(std::to_string(now.seconds())) << "\","
      << "\"request\":\"human_intervention\","
      << "\"module_name\":\"" << json_escape(fault.module_name) << "\","
      << "\"fault_key\":\"" << json_escape(fault.fault_key) << "\","
      << "\"fault_level\":\"" << json_escape(fault_level_to_string(fault.level)) << "\","
      << "\"reason\":\"" << json_escape(fault.reason) << "\","
      << "\"suggested_action\":\"manual_check\""
      << '}';
  msg.data = oss.str();
  human_intervention_pub_->publish(msg);
}

bool Nav2MonitorAggregatorNode::update_current_nav_task_locked(
  const std::string & task_name,
  const std::string & change_source)
{
  const std::string normalized = task_name.empty() ? "default" : task_name;
  if (normalized == current_nav_task_) {
    RCLCPP_DEBUG(
      get_logger(), "Ignore current_nav_task update from %s: already '%s'",
      change_source.c_str(), current_nav_task_.c_str());
    return false;
  }

  current_nav_task_ = normalized;
  (void)task_fault_config_selector_.update_current_task(current_nav_task_);
  RCLCPP_INFO(
    get_logger(), "Updated current_nav_task via %s: %s",
    change_source.c_str(), current_nav_task_.c_str());
  return true;
}

void Nav2MonitorAggregatorNode::load_task_status_code_mappings()
{
  const std::string prefix = "task_status_code_mappings.";
  const auto defaults = default_task_status_code_mappings();
  std::unordered_set<std::string> param_names;
  for (const auto & entry : defaults) {
    param_names.insert(prefix + entry.first);
  }

  const auto listed = this->list_parameters({prefix}, 2);
  for (const auto & name : listed.names) {
    if (name.rfind(prefix, 0) == 0) {
      param_names.insert(name);
    }
  }

  const auto & overrides = this->get_node_parameters_interface()->get_parameter_overrides();
  for (const auto & [name, _] : overrides) {
    if (name.rfind(prefix, 0) == 0) {
      param_names.insert(name);
    }
  }

  task_status_code_mappings_.clear();
  for (const auto & name : param_names) {
    const std::string code = name.substr(prefix.size());
    const auto default_it = defaults.find(code);
    const std::string default_value =
      (default_it != defaults.end()) ? default_it->second : std::string();
    if (!this->has_parameter(name)) {
      this->declare_parameter<std::string>(name, default_value);
    }

    std::string value;
    try {
      value = this->get_parameter(name).as_string();
    } catch (...) {
      value = default_value;
    }

    if (!value.empty()) {
      task_status_code_mappings_[code] = value;
    }
  }

  task_status_mapper_.configure(task_status_code_mappings_);
  RCLCPP_INFO(get_logger(), "Loaded %zu task_status code mappings", task_status_code_mappings_.size());
}

void Nav2MonitorAggregatorNode::configure_task_status_subscription()
{
  task_status_sub_.reset();
  if (task_status_topic_.empty()) {
    RCLCPP_INFO(get_logger(), "task_status subscription disabled: empty topic");
    return;
  }

  task_status_sub_ = this->create_subscription<master_interfaces::msg::TaskStatus>(
    task_status_topic_, build_topic_subscription_qos(task_status_topic_, rclcpp::QoS(10), 10),
    std::bind(&Nav2MonitorAggregatorNode::on_task_status, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "Subscribed task_status topic: %s", task_status_topic_.c_str());
}

void Nav2MonitorAggregatorNode::load_topic_qos_overrides()
{
  const auto entries = this->declare_parameter<std::vector<std::string>>(
    "topic_qos_overrides", std::vector<std::string>{});
  topic_qos_overrides_.clear();

  for (const auto & entry : entries) {
    std::stringstream ss(entry);
    std::string topic;
    std::string reliability_raw;
    std::string durability_raw;
    std::string depth_raw;
    if (
      !std::getline(ss, topic, ':') ||
      !std::getline(ss, reliability_raw, ':') ||
      !std::getline(ss, durability_raw, ':'))
    {
      RCLCPP_WARN(
        get_logger(),
        "Ignore topic_qos_overrides entry '%s': expected /topic:reliability:durability[:depth]",
        entry.c_str());
      continue;
    }
    (void)std::getline(ss, depth_raw, ':');

    const auto trim = [](std::string value) {
      const auto begin = value.find_first_not_of(" \t\r\n");
      const auto end = value.find_last_not_of(" \t\r\n");
      return begin == std::string::npos ? std::string() : value.substr(begin, end - begin + 1);
    };
    topic = trim(topic);
    reliability_raw = trim(reliability_raw);
    durability_raw = trim(durability_raw);
    depth_raw = trim(depth_raw);
    if (topic.empty()) {
      RCLCPP_WARN(get_logger(), "Ignore topic_qos_overrides entry '%s': empty topic", entry.c_str());
      continue;
    }

    TopicQosOverride override;
    bool ok = false;
    override.reliability = parse_reliability_policy(reliability_raw, ok);
    if (!ok) {
      RCLCPP_WARN(
        get_logger(), "Ignore topic_qos_overrides entry '%s': invalid reliability '%s'",
        entry.c_str(), reliability_raw.c_str());
      continue;
    }
    override.has_reliability = true;
    override.durability = parse_durability_policy(durability_raw, ok);
    if (!ok) {
      RCLCPP_WARN(
        get_logger(), "Ignore topic_qos_overrides entry '%s': invalid durability '%s'",
        entry.c_str(), durability_raw.c_str());
      continue;
    }
    override.has_durability = true;
    if (!depth_raw.empty()) {
      try {
        override.depth = std::max<size_t>(1, static_cast<size_t>(std::stoul(depth_raw)));
        override.has_depth = true;
      } catch (const std::exception &) {
        RCLCPP_WARN(
          get_logger(), "Ignore topic_qos_overrides entry '%s': invalid depth '%s'",
          entry.c_str(), depth_raw.c_str());
        continue;
      }
    }

    topic_qos_overrides_[topic] = override;
    RCLCPP_INFO(
      get_logger(),
      "Loaded QoS override: topic=%s reliability=%s durability=%s depth=%s",
      topic.c_str(),
      reliability_to_string(override.reliability).c_str(),
      durability_to_string(override.durability).c_str(),
      override.has_depth ? std::to_string(override.depth).c_str() : "<auto>");
  }
}

void Nav2MonitorAggregatorNode::load_task_fault_config_mappings()
{
  const std::string prefix = "task_fault_configs.";
  std::unordered_set<std::string> param_names{
    prefix + "default",
    prefix + "todoor",
    prefix + "elevator",
    prefix + "reverse"
  };

  const auto listed = this->list_parameters({prefix}, 2);
  for (const auto & name : listed.names) {
    if (name.rfind(prefix, 0) == 0) {
      param_names.insert(name);
    }
  }

  const auto & overrides = this->get_node_parameters_interface()->get_parameter_overrides();
  for (const auto & [name, _] : overrides) {
    if (name.rfind(prefix, 0) == 0) {
      param_names.insert(name);
    }
  }

  task_fault_config_mappings_.clear();
  for (const auto & name : param_names) {
    const bool is_default = (name == prefix + std::string("default"));
    const std::string default_value = is_default ? base_fault_config_path_ : std::string();
    if (!this->has_parameter(name)) {
      this->declare_parameter<std::string>(name, default_value);
    }

    std::string value;
    try {
      value = this->get_parameter(name).as_string();
    } catch (...) {
      value = default_value;
    }

    task_fault_config_mappings_[name.substr(prefix.size())] = value;
  }

  const auto default_it = task_fault_config_mappings_.find("default");
  const std::string selector_default =
    (default_it != task_fault_config_mappings_.end() && !default_it->second.empty()) ?
    default_it->second : base_fault_config_path_;
  task_fault_config_selector_.configure(selector_default, task_fault_config_mappings_);
}

void Nav2MonitorAggregatorNode::update_task_selected_fault_config(bool force_reload)
{
  const std::string selected_fault_config = task_fault_config_selector_.resolve_fault_config_for_task();
  const std::string resolved_selected_fault_config = resolve_config_path(selected_fault_config);
  const bool path_changed =
    selected_fault_config != fault_config_path_ ||
    resolved_selected_fault_config != resolved_fault_config_path_;

  if (!path_changed && !force_reload) {
    task_fault_config_selector_.clear_task_changed();
    return;
  }

  fault_config_path_ = selected_fault_config;
  resolved_fault_config_path_ = resolved_selected_fault_config;
  fault_config_watcher_.configure(fault_config_path_, resolved_fault_config_path_);

  RCLCPP_INFO(
    get_logger(),
    "Selected fault_config for task '%s': active='%s' resolved='%s'",
    task_fault_config_selector_.current_task().c_str(),
    fault_config_path_.c_str(),
    resolved_fault_config_path_.c_str());

  if (fault_config_path_.empty()) {
    apply_loaded_fault_config();
    fault_config_watcher_.sync_current_state();
  } else {
    (void)reload_fault_config_if_needed(true);
  }
  publish_config_profile();
  task_fault_config_selector_.clear_task_changed();
}

bool Nav2MonitorAggregatorNode::reload_fault_config_if_needed(bool force)
{
  if (fault_config_path_.empty()) {
    return false;
  }

  if (!force) {
    if (!fault_config_reload_enabled_ || !fault_config_watcher_.enabled() ||
      !fault_config_watcher_.poll_changed()) {
      return false;
    }
    RCLCPP_INFO(
      get_logger(), "fault_config changed, reloading: param='%s', resolved='%s'",
      fault_config_path_.c_str(), resolved_fault_config_path_.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(), "fault_config param='%s', resolved='%s'",
      fault_config_path_.c_str(), resolved_fault_config_path_.c_str());
  }

  std::ifstream config_stream(resolved_fault_config_path_);
  if (!config_stream.good()) {
    RCLCPP_ERROR(
      get_logger(),
      "fault_config not readable: %s (resolved: %s), keep current config/fallback targets",
      fault_config_path_.c_str(), resolved_fault_config_path_.c_str());
  } else {
    fault_detector_.load_config(resolved_fault_config_path_);
  }

  apply_loaded_fault_config();
  fault_config_watcher_.sync_current_state();
  publish_config_profile();
  return true;
}

void Nav2MonitorAggregatorNode::publish_config_profile()
{
  if (!config_profile_pub_ || fault_config_path_.empty()) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = config_profile_update_to_json(ConfigProfileUpdate{
      current_nav_task_,
      fault_config_path_,
      resolved_fault_config_path_});
  config_profile_pub_->publish(msg);
  RCLCPP_INFO(
    get_logger(),
    "Published config profile: topic=%s task=%s fault_config=%s",
    config_profile_topic_.c_str(), current_nav_task_.c_str(), fault_config_path_.c_str());
}

void Nav2MonitorAggregatorNode::configure_topic_state_subscription()
{
  topic_states_sub_.reset();
  topic_states_sub_ = this->create_subscription<std_msgs::msg::String>(
    topic_states_topic_,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&Nav2MonitorAggregatorNode::on_topic_states, this, std::placeholders::_1),
    make_subscription_options(default_callback_group_));
  RCLCPP_INFO(
    get_logger(), "External topic state input enabled: topic=%s", topic_states_topic_.c_str());
}

void Nav2MonitorAggregatorNode::configure_vehicle_state_subscription()
{
  vehicle_state_sub_.reset();
  vehicle_state_sub_ = this->create_subscription<std_msgs::msg::String>(
    vehicle_state_topic_,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&Nav2MonitorAggregatorNode::on_vehicle_state, this, std::placeholders::_1),
    make_subscription_options(default_callback_group_));
  RCLCPP_INFO(
    get_logger(), "External vehicle state input enabled: topic=%s", vehicle_state_topic_.c_str());
}

void Nav2MonitorAggregatorNode::configure_node_tf_state_subscription()
{
  node_tf_state_sub_.reset();
  node_tf_state_sub_ = this->create_subscription<std_msgs::msg::String>(
    node_tf_state_topic_,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&Nav2MonitorAggregatorNode::on_node_tf_state, this, std::placeholders::_1),
    make_subscription_options(default_callback_group_));
  RCLCPP_INFO(
    get_logger(), "External node/TF state input enabled: topic=%s", node_tf_state_topic_.c_str());
}

void Nav2MonitorAggregatorNode::configure_battery_state_subscription()
{
  monitor_battery_state_sub_.reset();
  monitor_battery_state_sub_ = this->create_subscription<std_msgs::msg::String>(
    monitor_battery_state_topic_,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&Nav2MonitorAggregatorNode::on_monitor_battery_state, this, std::placeholders::_1),
    make_subscription_options(default_callback_group_));
  RCLCPP_INFO(
    get_logger(), "External battery state input enabled: topic=%s",
    monitor_battery_state_topic_.c_str());
}

void Nav2MonitorAggregatorNode::configure_feedback_state_subscription()
{
  feedback_state_sub_.reset();
  feedback_state_sub_ = this->create_subscription<std_msgs::msg::String>(
    feedback_state_topic_,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&Nav2MonitorAggregatorNode::on_feedback_state, this, std::placeholders::_1),
    make_subscription_options(default_callback_group_));
  RCLCPP_INFO(
    get_logger(), "External feedback state input enabled: topic=%s",
    feedback_state_topic_.c_str());
}

void Nav2MonitorAggregatorNode::configure_collision_state_subscription()
{
  collision_state_sub_.reset();
  collision_state_sub_ = this->create_subscription<std_msgs::msg::String>(
    collision_state_topic_,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&Nav2MonitorAggregatorNode::on_collision_state, this, std::placeholders::_1),
    make_subscription_options(default_callback_group_));
  RCLCPP_INFO(
    get_logger(), "External collision state input enabled: topic=%s",
    collision_state_topic_.c_str());
}

void Nav2MonitorAggregatorNode::apply_loaded_fault_config()
{
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!fault_config_path_.empty() && fault_detector_.has_module_configs()) {
      target_nodes_ = fault_detector_.get_monitored_nodes();
      watch_topics_ = fault_detector_.get_watched_topics();
      monitor_targets_from_fault_config_ = true;
      RCLCPP_INFO(
        get_logger(), "Monitor targets loaded from fault_config modules: %zu nodes, %zu topics",
        target_nodes_.size(), watch_topics_.size());
    } else {
      target_nodes_ = fallback_target_nodes_;
      watch_topics_ = fallback_watch_topics_;
      monitor_targets_from_fault_config_ = false;
      RCLCPP_INFO(
        get_logger(),
        "fault_config missing/empty modules, fallback to target_nodes/watch_topics params");
    }

    target_transforms_.clear();
    const auto & fault_transforms = fault_detector_.get_monitored_transforms();
    std::vector<std::string> fallback_transforms;
    const std::vector<std::string> * active_transforms = &fault_transforms;
    if (fault_config_path_.empty() || fault_transforms.empty()) {
      fallback_transforms = this->get_parameter("target_transforms").as_string_array();
      active_transforms = &fallback_transforms;
    }
    for (const auto & tf_str : *active_transforms) {
      auto pos = tf_str.find("->");
      if (pos != std::string::npos) {
        target_transforms_.push_back({tf_str.substr(0, pos), tf_str.substr(pos + 2)});
      }
    }

  }
}

void Nav2MonitorAggregatorNode::scan_topology()
{
  (void)reload_fault_config_if_needed(false);
}

void Nav2MonitorAggregatorNode::check_health()
{
  const auto now = this->now();
  const auto vehicle_status = vehicle_monitor_->get_status();

  msg::MonitorStatus status_msg;
  std::vector<FaultInfo> pending_faults;
  pending_faults.reserve(8);
  std::vector<FaultInfo> faults;
  std::vector<FaultEdgeEvent> fault_edge_events;
  std::optional<SafetyCommandUpdate> safety_update;
  std::vector<std::string> target_nodes_snapshot;
  std::vector<std::string> watch_topics_snapshot;
  std::vector<std::pair<std::string, std::string>> target_transforms_snapshot;
  std::map<std::pair<std::string, std::string>, TransformInfo> tf_info_snapshot;

  {
    std::lock_guard<std::mutex> lock(mtx_);
    target_nodes_snapshot = target_nodes_;
    watch_topics_snapshot = watch_topics_;
    target_transforms_snapshot = target_transforms_;
    tf_info_snapshot = external_tf_info_;
  }

  status_msg.all_ok = true;
  status_msg.monitored_nodes = target_nodes_snapshot;
  status_msg.monitored_topics = watch_topics_snapshot;

  bool node_tf_state_stale = false;
  std::map<std::string, bool> external_node_active_snapshot;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    node_tf_state_stale =
      last_node_tf_state_time_.nanoseconds() == 0 ||
      (now - last_node_tf_state_time_).seconds() > node_tf_state_timeout_s_;
    external_node_active_snapshot = external_node_active_;
  }

  for (const auto & node : target_nodes_snapshot) {
    const auto external_it = external_node_active_snapshot.find(node);
    const bool node_active =
      !node_tf_state_stale && external_it != external_node_active_snapshot.end() && external_it->second;
    if (node_active) {
      status_msg.active_nodes.push_back(node);
    } else {
      status_msg.timeout_nodes.push_back(node);
      status_msg.all_ok = false;
    }
  }

  for (const auto & topic : watch_topics_snapshot) {
    const auto info = data_store_.get_watch_topic_state(topic);
    const double min_hz = fault_detector_.get_watch_topic_min_hz(topic);
    const bool require_frequency = min_hz > 0.0;
    const double receive_frequency = data_store_.get_watch_topic_frequency(topic, now, min_hz);
    const bool topic_present = info.has_value() && info->has_publisher;
    const bool topic_valid = topic_present && (!require_frequency || receive_frequency > 0.0);
    if (topic_valid) {
      status_msg.active_topics.push_back(topic);
      status_msg.topic_frequencies.push_back(static_cast<float>(receive_frequency));
    } else {
      status_msg.inactive_topics.push_back(topic);
      status_msg.topic_frequencies.push_back(0.0);
      status_msg.all_ok = false;
    }
  }

  for (const auto & [src, tgt] : target_transforms_snapshot) {
    std::string tf_str = src + "->" + tgt;
    status_msg.monitored_transforms.push_back(tf_str);
    const auto tf_it = tf_info_snapshot.find({src, tgt});
    const bool tf_available =
      !node_tf_state_stale && tf_it != tf_info_snapshot.end() &&
      tf_it->second.last_update.nanoseconds() > 0;
    if (tf_available) {
      status_msg.available_transforms.push_back(tf_str);
      status_msg.transform_latencies_ms.push_back(tf_it->second.latency_ms);
    } else {
      status_msg.stale_transforms.push_back(tf_str);
      status_msg.transform_latencies_ms.push_back(-1.0);
      status_msg.all_ok = false;
    }
  }

  status_msg.cpu_usage = sys_monitor_.get_cpu_usage();
  status_msg.mem_usage = sys_monitor_.get_mem_usage();
  status_msg.disk_usage = sys_monitor_.get_disk_usage();
  status_msg.cpu_temp = sys_monitor_.get_cpu_temp();
  status_msg.gpu_usage = sys_monitor_.get_gpu_usage();
  status_msg.gpu_temp = sys_monitor_.get_gpu_temp();
  status_msg.gpu_mem_usage = sys_monitor_.get_gpu_mem();

  status_msg.vehicle_status_valid = vehicle_status.valid;
  status_msg.vehicle_navigation_active = vehicle_status.navigation_active;
  status_msg.vehicle_navigation_succeeded = vehicle_status.navigation_succeeded;
  status_msg.vehicle_progress_percentage = vehicle_status.progress_percentage;
  status_msg.vehicle_simple_status = vehicle_status.simple_status;
  status_msg.vehicle_error_message = vehicle_status.error_message;

  const auto battery_state = data_store_.get_battery_state();
  const bool battery_state_stale =
    (!battery_state.has_data ||
    (now - battery_state.last_seen).seconds() > monitor_battery_state_timeout_s_);
  if (
    battery_state.has_data &&
    (now - battery_state.last_seen).seconds() <= monitor_battery_state_timeout_s_)
  {
    status_msg.battery_temperature = battery_state.temperature;
    status_msg.battery_percentage = battery_state.percentage;
  } else if (battery_state_stale) {
    status_msg.all_ok = false;
  }

  for (const auto & node : target_nodes_snapshot) {
    const auto external_it = external_node_active_snapshot.find(node);
    data_store_.set_node_active(
      node,
      !node_tf_state_stale &&
      external_it != external_node_active_snapshot.end() &&
      external_it->second,
      now);
  }

  faults = fault_detector_.detect_faults(data_store_, now);
  bool monitor_battery_state_stale = false;
  bool source_has_data = false;
  bool source_stale = true;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    monitor_battery_state_stale =
      !external_battery_state_seen_ ||
      last_monitor_battery_state_time_.nanoseconds() == 0 ||
      (now - last_monitor_battery_state_time_).seconds() > monitor_battery_state_timeout_s_;
    source_has_data = external_battery_source_has_data_;
    source_stale = external_battery_source_stale_;
  }
  if (monitor_battery_state_stale || !source_has_data || source_stale) {
    FaultInfo fault;
    fault.fault_key = "battery_monitor|state_stale|action=" +
      std::to_string(static_cast<int>(ActionType::SUPERVISOR));
    fault.module_name = "battery_monitor";
    fault.level = FaultLevel::ERROR;
    fault.reason = monitor_battery_state_stale ?
      "External battery_monitor state missing or stale" :
      "Battery source missing or stale";
    fault.fault_type = "battery_state_source";
    fault.fault_model = "battery";
    fault.fault_name = "battery_state_source";
    fault.action = ActionType::SUPERVISOR;
    fault.safety_command = SafetyCommandType::NONE;
    fault.safety_slow_down_percentage = 0.0;
    fault.timestamp = now;
    faults.push_back(std::move(fault));
  }
  faults.erase(
    std::remove_if(
      faults.begin(), faults.end(),
      [](const FaultInfo & fault) {
        return fault.fault_model == "feedback" || fault.fault_type == "feedback_rule";
      }),
    faults.end());
  std::vector<FaultInfo> feedback_faults_snapshot;
  bool feedback_state_stale = false;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    feedback_state_stale =
      last_feedback_state_time_.nanoseconds() == 0 ||
      (now - last_feedback_state_time_).seconds() > feedback_state_timeout_s_;
    feedback_faults_snapshot = external_feedback_faults_;
  }
  if (feedback_state_stale) {
    FaultInfo fault;
    fault.fault_key = "algorithm_feedback_monitor|state_stale|action=" +
      std::to_string(static_cast<int>(ActionType::SUPERVISOR));
    fault.module_name = "algorithm_feedback_monitor";
    fault.level = FaultLevel::ERROR;
    fault.reason = "External algorithm_feedback_monitor state missing or stale";
    fault.fault_type = "feedback_state_source";
    fault.fault_model = "feedback";
    fault.fault_name = "feedback_state_source";
    fault.action = ActionType::SUPERVISOR;
    fault.safety_command = SafetyCommandType::NONE;
    fault.safety_slow_down_percentage = 0.0;
    fault.timestamp = now;
    faults.push_back(std::move(fault));
  } else {
    faults.insert(
      faults.end(),
      std::make_move_iterator(feedback_faults_snapshot.begin()),
      std::make_move_iterator(feedback_faults_snapshot.end()));
  }
  faults.erase(
    std::remove_if(
      faults.begin(), faults.end(),
      [](const FaultInfo & fault) {
        return fault.fault_type == "collision_detection" ||
               fault.fault_type == "collision_source" ||
               fault.fault_model == "ttc" ||
               fault.fault_model == "zone";
      }),
    faults.end());
  std::vector<FaultInfo> collision_faults_snapshot;
  bool collision_state_stale = false;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    collision_state_stale =
      last_collision_state_time_.nanoseconds() == 0 ||
      (now - last_collision_state_time_).seconds() > collision_state_timeout_s_;
    collision_faults_snapshot = external_collision_faults_;
  }
  if (collision_state_stale) {
    FaultInfo fault;
    fault.fault_key = "collision_monitor|state_stale|action=" +
      std::to_string(static_cast<int>(ActionType::SUPERVISOR));
    fault.module_name = "collision_monitor";
    fault.level = FaultLevel::ERROR;
    fault.reason = "External collision_monitor state missing or stale";
    fault.fault_type = "collision_state_source";
    fault.fault_model = "collision";
    fault.fault_name = "collision_state_source";
    fault.action = ActionType::SUPERVISOR;
    fault.safety_command = SafetyCommandType::NONE;
    fault.safety_slow_down_percentage = 0.0;
    fault.timestamp = now;
    faults.push_back(std::move(fault));
  } else {
    faults.insert(
      faults.end(),
      std::make_move_iterator(collision_faults_snapshot.begin()),
      std::make_move_iterator(collision_faults_snapshot.end()));
  }
  faults.erase(
    std::remove_if(
      faults.begin(), faults.end(),
      [](const FaultInfo & fault) {
        return is_vehicle_state_judge_fault(fault);
      }),
    faults.end());

  std::vector<FaultInfo> vehicle_faults_snapshot;
  bool vehicle_state_stale = false;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    vehicle_state_stale =
      last_vehicle_state_time_.nanoseconds() == 0 ||
      (now - last_vehicle_state_time_).seconds() > vehicle_state_timeout_s_;
    vehicle_faults_snapshot = external_vehicle_faults_;
  }
  if (vehicle_state_stale) {
    FaultInfo fault;
    fault.fault_key = "vehicle_state_judge|state_stale|action=" +
      std::to_string(static_cast<int>(ActionType::SUPERVISOR));
    fault.module_name = "vehicle_state_judge";
    fault.level = FaultLevel::ERROR;
    fault.reason = "External vehicle_state_judge state missing or stale";
    fault.fault_type = "vehicle_state_source";
    fault.fault_model = "vehicle_state";
    fault.fault_name = "vehicle_state_source";
    fault.action = ActionType::SUPERVISOR;
    fault.safety_command = SafetyCommandType::NONE;
    fault.safety_slow_down_percentage = 0.0;
    fault.timestamp = now;
    faults.push_back(std::move(fault));
  } else {
    faults.insert(
      faults.end(),
      std::make_move_iterator(vehicle_faults_snapshot.begin()),
      std::make_move_iterator(vehicle_faults_snapshot.end()));
  }
  std::unordered_map<std::string, size_t> nodemanager_fault_by_module;
  for (const auto & fault : faults) {
    if (fault.action != ActionType::SUPERVISOR) {
      continue;
    }
    const auto it = nodemanager_fault_by_module.find(fault.module_name);
    if (it == nodemanager_fault_by_module.end()) {
      if (should_publish_action(fault.module_name, fault.action, now)) {
        nodemanager_fault_by_module[fault.module_name] = pending_faults.size();
        pending_faults.push_back(fault);
      }
      continue;
    }

    auto & merged_fault = pending_faults[it->second];
    if (merged_fault.fault_key.find(fault.fault_key) == std::string::npos) {
      merged_fault.fault_key += ";" + fault.fault_key;
    }
    if (merged_fault.reason.find(fault.reason) == std::string::npos) {
      merged_fault.reason += " | " + fault.reason;
    }
  }

  auto state_update = fault_state_coordinator_.update(faults);
  fault_edge_events = std::move(state_update.edge_events);
  safety_update = std::move(state_update.safety_update);

  pub_->publish(status_msg);
  monitor_reporter_.publish_heartbeat(status_msg, now);

  if (safety_update.has_value()) {
    nav2_monitor::msg::SafetyCmd reporter_safety_cmd;
    if (safety_update->active) {
      reporter_safety_cmd.action = static_cast<uint8_t>(safety_update->command);
      reporter_safety_cmd.slow_down_percentage = static_cast<float>(safety_update->slow_down_percentage);
      reporter_safety_cmd.reason = safety_update->reason;
    } else {
      reporter_safety_cmd.action = nav2_monitor::msg::SafetyCmd::RESUME;
      reporter_safety_cmd.slow_down_percentage = 0.0F;
      reporter_safety_cmd.reason = safety_update->reason;
    }
    monitor_reporter_.cache_safety_cmd(reporter_safety_cmd, now);
    if (!safety_update->active) {
      RCLCPP_WARN(get_logger(), "Safety state recovered: %s", safety_update->reason.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Safety state updated: command=%d reason=%s",
        static_cast<int>(safety_update->command), safety_update->reason.c_str());
    }
  }

  for (const auto & edge_event : fault_edge_events) {
    msg::FaultEvent event;
    const auto now_ns = now.nanoseconds();
    event.stamp.sec = static_cast<int32_t>(now_ns / 1000000000LL);
    event.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);
    event.module_name = edge_event.fault.module_name;
    event.fault_level = fault_level_to_msg(edge_event.fault.level);
    event.reason = edge_event.fault.reason;
    event.action = action_to_msg(edge_event.fault.action);
    event.edge = edge_event.edge == FaultEdgeType::TRIGGER ?
      msg::FaultEvent::EDGE_TRIGGER : msg::FaultEvent::EDGE_RECOVER;
    fault_event_pub_->publish(event);
    monitor_reporter_.publish_fault_event_json(event, now);
  }

  for (const auto & fault : pending_faults) {
    if (fault.action == ActionType::SUPERVISOR) {
      std_msgs::msg::String cmd;
      std::ostringstream oss;
      oss << '{'
          << "\"module_name\":\"" << json_escape(fault.module_name) << "\","
          << "\"fault_keys\":\"" << json_escape(fault.fault_key) << "\","
          << "\"nodes_to_restart\":[],"
          << "\"reason\":\"" << json_escape(fault.reason) << "\"}";
      cmd.data = oss.str();
      nodemanager_pub_->publish(cmd);
      monitor_reporter_.cache_nodemanager_json(cmd.data, now);
      if (is_vehicle_state_judge_fault(fault)) {
        publish_human_intervention_request(fault, now);
      }
      RCLCPP_WARN(
        get_logger(), "NodeManager restart request: %s - %s",
        fault.module_name.c_str(), fault.reason.c_str());
    }
  }
}

rcl_interfaces::msg::SetParametersResult Nav2MonitorAggregatorNode::on_parameter_change(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool fault_config_changed = false;
  bool fault_config_cleared = false;
  bool task_status_subscription_changed = false;
  bool task_status_code_mappings_changed = false;
  bool topic_state_subscription_changed = false;
  bool vehicle_state_subscription_changed = false;
  bool node_tf_state_subscription_changed = false;
  bool battery_state_subscription_changed = false;
  bool feedback_state_subscription_changed = false;
  bool collision_state_subscription_changed = false;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    for (const auto & param : params) {
      if (param.get_name() == "target_nodes") {
        fallback_target_nodes_ = param.as_string_array();
        if (monitor_targets_from_fault_config_) {
          RCLCPP_WARN(get_logger(), "Ignore target_nodes update: monitor targets are from fault_config modules");
        } else {
          target_nodes_ = fallback_target_nodes_;
          RCLCPP_INFO(get_logger(), "Updated target_nodes: %zu nodes", target_nodes_.size());
        }
      } else if (param.get_name() == "watch_topics") {
        fallback_watch_topics_ = param.as_string_array();
        if (monitor_targets_from_fault_config_) {
          RCLCPP_WARN(get_logger(), "Ignore watch_topics update: monitor targets are from fault_config modules");
        } else {
          watch_topics_ = fallback_watch_topics_;
          RCLCPP_INFO(get_logger(), "Updated watch_topics: %zu topics", watch_topics_.size());
        }
      } else if (param.get_name() == "fault_config") {
        base_fault_config_path_ = param.as_string();
        resolved_base_fault_config_path_ = resolve_config_path(base_fault_config_path_);
        fault_config_changed = true;
        fault_config_cleared = base_fault_config_path_.empty();
        RCLCPP_INFO(
          get_logger(), "Updated base fault_config: param='%s', resolved='%s'",
          base_fault_config_path_.c_str(), resolved_base_fault_config_path_.c_str());
      } else if (param.get_name() == "current_nav_task") {
        const std::string change_source = pending_task_switch_source_.empty() ?
          std::string("param current_nav_task") : pending_task_switch_source_;
        fault_config_changed =
          update_current_nav_task_locked(param.as_string(), change_source) || fault_config_changed;
      } else if (param.get_name() == "task_status_topic") {
        task_status_topic_ = param.as_string();
        task_status_subscription_changed = true;
        RCLCPP_INFO(get_logger(), "Updated task_status_topic: %s", task_status_topic_.c_str());
      } else if (param.get_name() == "topic_states_topic") {
        topic_states_topic_ = param.as_string();
        topic_state_subscription_changed = true;
        RCLCPP_INFO(get_logger(), "Updated topic_states_topic: %s", topic_states_topic_.c_str());
      } else if (param.get_name() == "vehicle_state_topic") {
        vehicle_state_topic_ = param.as_string();
        vehicle_state_subscription_changed = true;
        RCLCPP_INFO(get_logger(), "Updated vehicle_state_topic: %s", vehicle_state_topic_.c_str());
      } else if (param.get_name() == "vehicle_state_timeout_s") {
        vehicle_state_timeout_s_ = std::max(0.1, param.as_double());
        RCLCPP_INFO(get_logger(), "Updated vehicle_state_timeout_s: %.2f", vehicle_state_timeout_s_);
      } else if (param.get_name() == "node_tf_state_topic") {
        node_tf_state_topic_ = param.as_string();
        node_tf_state_subscription_changed = true;
        RCLCPP_INFO(get_logger(), "Updated node_tf_state_topic: %s", node_tf_state_topic_.c_str());
      } else if (param.get_name() == "node_tf_state_timeout_s") {
        node_tf_state_timeout_s_ = std::max(0.1, param.as_double());
        RCLCPP_INFO(get_logger(), "Updated node_tf_state_timeout_s: %.2f", node_tf_state_timeout_s_);
      } else if (param.get_name() == "monitor_battery_state_topic") {
        monitor_battery_state_topic_ = param.as_string();
        battery_state_subscription_changed = true;
        RCLCPP_INFO(
          get_logger(), "Updated monitor_battery_state_topic: %s",
          monitor_battery_state_topic_.c_str());
      } else if (param.get_name() == "monitor_battery_state_timeout_s") {
        monitor_battery_state_timeout_s_ = std::max(0.1, param.as_double());
        RCLCPP_INFO(
          get_logger(), "Updated monitor_battery_state_timeout_s: %.2f",
          monitor_battery_state_timeout_s_);
      } else if (param.get_name() == "feedback_state_topic") {
        feedback_state_topic_ = param.as_string();
        feedback_state_subscription_changed = true;
        RCLCPP_INFO(get_logger(), "Updated feedback_state_topic: %s", feedback_state_topic_.c_str());
      } else if (param.get_name() == "feedback_state_timeout_s") {
        feedback_state_timeout_s_ = std::max(0.1, param.as_double());
        RCLCPP_INFO(get_logger(), "Updated feedback_state_timeout_s: %.2f", feedback_state_timeout_s_);
      } else if (param.get_name() == "collision_state_topic") {
        collision_state_topic_ = param.as_string();
        collision_state_subscription_changed = true;
        RCLCPP_INFO(get_logger(), "Updated collision_state_topic: %s", collision_state_topic_.c_str());
      } else if (param.get_name() == "collision_state_timeout_s") {
        collision_state_timeout_s_ = std::max(0.1, param.as_double());
        RCLCPP_INFO(get_logger(), "Updated collision_state_timeout_s: %.2f", collision_state_timeout_s_);
      } else if (param.get_name().rfind("task_status_code_mappings.", 0) == 0) {
        task_status_code_mappings_changed = true;
        RCLCPP_INFO(get_logger(), "Updated task status code mapping: %s", param.get_name().c_str());
      } else if (param.get_name().rfind("task_fault_configs.", 0) == 0) {
        fault_config_changed = true;
        RCLCPP_INFO(get_logger(), "Updated task fault config mapping: %s", param.get_name().c_str());
      } else if (param.get_name() == "fault_config_reload_enabled") {
        fault_config_reload_enabled_ = param.as_bool();
        RCLCPP_INFO(get_logger(), "Updated fault_config_reload_enabled: %s",
          fault_config_reload_enabled_ ? "true" : "false");
      } else if (param.get_name() == "target_transforms") {
        target_transforms_.clear();
        for (const auto & tf_str : param.as_string_array()) {
          auto pos = tf_str.find("->");
          if (pos != std::string::npos) {
            target_transforms_.push_back({tf_str.substr(0, pos), tf_str.substr(pos + 2)});
          }
        }
        if (!fault_config_path_.empty() && !fault_detector_.get_monitored_transforms().empty()) {
          RCLCPP_WARN(get_logger(), "target_transforms param updated but fault_config target_transforms takes precedence");
        } else {
          RCLCPP_INFO(get_logger(), "Updated target_transforms: %zu transforms", target_transforms_.size());
        }
      } else if (param.get_name() == "timeout") {
        timeout_ = param.as_double();
        fault_detector_.set_feedback_default_max_stale(timeout_);
        RCLCPP_INFO(get_logger(), "Updated timeout: %.1f seconds", timeout_);
      } else if (param.get_name() == "safety_cooldown_s") {
        safety_cooldown_s_ = std::max(0.0, param.as_double());
        RCLCPP_INFO(get_logger(), "Updated safety_cooldown_s: %.2f", safety_cooldown_s_);
      } else if (param.get_name() == "nodemanager_cooldown_s") {
        nodemanager_cooldown_s_ = std::max(0.0, param.as_double());
        RCLCPP_INFO(get_logger(), "Updated nodemanager_cooldown_s: %.2f", nodemanager_cooldown_s_);
      } else if (param.get_name() == "supervisor_cooldown_s") {
        nodemanager_cooldown_s_ = std::max(0.0, param.as_double());
        RCLCPP_INFO(
          get_logger(),
          "Updated legacy supervisor_cooldown_s alias; nodemanager_cooldown_s: %.2f",
          nodemanager_cooldown_s_);
      }
    }
  }

  if (task_status_code_mappings_changed) {
    load_task_status_code_mappings();
  }
  if (task_status_subscription_changed) {
    configure_task_status_subscription();
  }
  if (topic_state_subscription_changed) {
    configure_topic_state_subscription();
  }
  if (vehicle_state_subscription_changed) {
    configure_vehicle_state_subscription();
  }
  if (node_tf_state_subscription_changed) {
    configure_node_tf_state_subscription();
  }
  if (battery_state_subscription_changed) {
    configure_battery_state_subscription();
  }
  if (feedback_state_subscription_changed) {
    configure_feedback_state_subscription();
  }
  if (collision_state_subscription_changed) {
    configure_collision_state_subscription();
  }

  if (fault_config_changed) {
    load_task_fault_config_mappings();
    if (task_fault_config_selector_.current_task() != current_nav_task_) {
      (void)task_fault_config_selector_.update_current_task(current_nav_task_);
    }
    if (fault_config_cleared && task_fault_config_mappings_.find("default") == task_fault_config_mappings_.end()) {
      apply_loaded_fault_config();
      fault_config_watcher_.sync_current_state();
    } else {
      update_task_selected_fault_config(true);
    }
  }

  return result;
}

}  // namespace nav2_monitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_monitor::Nav2MonitorAggregatorNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
