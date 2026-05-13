#include "nav2_monitor/monitor_reporter.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <sstream>

namespace nav2_monitor
{

void MonitorReporter::configure(rclcpp::Node * node)
{
  node_ = node;
  cmd_correlation_window_s_ = std::max(0.0, node_->declare_parameter<double>("reporter.cmd_correlation_window_s", 2.0));
  const auto heartbeat_topic = node_->declare_parameter<std::string>("reporter.heartbeat_json_topic", "/nav2_monitor/reporter/heartbeat_json");
  const auto event_topic = node_->declare_parameter<std::string>("reporter.event_json_topic", "/nav2_monitor/reporter/event_json");
  const auto human_takeover_topic = node_->declare_parameter<std::string>(
    "reporter.human_takeover_json_topic", "/nav2_monitor/reporter/human_takeover_json");
  heartbeat_pub_ = node_->create_publisher<std_msgs::msg::String>(heartbeat_topic, 10);
  event_pub_ = node_->create_publisher<std_msgs::msg::String>(event_topic, 10);
  human_takeover_pub_ = node_->create_publisher<std_msgs::msg::String>(human_takeover_topic, 10);
}

std::string MonitorReporter::json_escape(const std::string & input)
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

std::string MonitorReporter::fault_level_to_string(uint8_t level)
{
  switch (level) {
    case msg::FaultEvent::WARNING: return "WARNING";
    case msg::FaultEvent::ERROR: return "ERROR";
    case msg::FaultEvent::CRITICAL: return "CRITICAL";
    default: return "NORMAL";
  }
}

std::string MonitorReporter::action_to_string(uint8_t action)
{
  switch (action) {
    case msg::FaultEvent::NODEMANAGER: return "NODEMANAGER";
    case msg::FaultEvent::SAFETY_SYSTEM: return "SAFETY_SYSTEM";
    default: return "NONE";
  }
}

std::string MonitorReporter::edge_to_string(uint8_t edge)
{
  switch (edge) {
    case msg::FaultEvent::EDGE_TRIGGER: return "TRIGGER";
    case msg::FaultEvent::EDGE_RECOVER: return "RECOVER";
    default: return "NONE";
  }
}

std::string MonitorReporter::safety_action_to_string(uint8_t action)
{
  switch (action) {
    case msg::SafetyCmd::SLOW_DOWN: return "SLOW_DOWN";
    case msg::SafetyCmd::SOFT_STOP: return "SOFT_STOP";
    case msg::SafetyCmd::EMERGENCY_STOP: return "EMERGENCY_STOP";
    case msg::SafetyCmd::RESUME: return "RESUME";
    default: return "NONE";
  }
}

std::string MonitorReporter::classify_fault_type(const msg::FaultEvent & event)
{
  const auto & reason = event.reason;
  if (reason.rfind("RECOVER fault_key=", 0) == 0) {
    const auto key_begin = std::string("RECOVER fault_key=").size();
    const auto key_end = reason.find(";", key_begin);
    const std::string key = key_end == std::string::npos ? reason.substr(key_begin) : reason.substr(key_begin, key_end - key_begin);
    const auto first_sep = key.find('|');
    const auto action_sep = key.rfind("|action=");
    if (first_sep != std::string::npos && action_sep != std::string::npos && action_sep > first_sep) {
      return key.substr(first_sep + 1, action_sep - first_sep - 1);
    }
    return key;
  }
  if (reason.find("Node inactive") != std::string::npos) return "node_inactive";
  if (reason.find("Feedback") != std::string::npos) return "feedback_rule";
  if (reason.find("Topic frequency low") != std::string::npos) return "watch_topic_frequency";
  if (reason.find("Collision") != std::string::npos || reason.find("ttc=") != std::string::npos) return "collision_detection";
  if (reason.find("Moto active") != std::string::npos || reason.find("stationary") != std::string::npos || reason.find("chassis") != std::string::npos) return "chassis_state";
  return "unknown";
}

std::string MonitorReporter::to_stamp_string(const builtin_interfaces::msg::Time & stamp)
{
  std::ostringstream oss;
  oss << stamp.sec << "." << std::setw(9) << std::setfill('0') << stamp.nanosec;
  return oss.str();
}

std::string MonitorReporter::extract_json_string_field(const std::string & json, const std::string & field)
{
  const std::string key = "\"" + field + "\":";
  const auto pos = json.find(key);
  if (pos == std::string::npos) {
    return "";
  }
  auto value_pos = pos + key.size();
  while (value_pos < json.size() && std::isspace(static_cast<unsigned char>(json[value_pos]))) {
    ++value_pos;
  }
  if (value_pos >= json.size() || json[value_pos] != '"') {
    return "";
  }
  ++value_pos;
  const auto end_pos = json.find('"', value_pos);
  if (end_pos == std::string::npos) {
    return "";
  }
  return json.substr(value_pos, end_pos - value_pos);
}

size_t MonitorReporter::extract_json_array_count(const std::string & json, const std::string & field)
{
  const std::string key = "\"" + field + "\":";
  const auto pos = json.find(key);
  if (pos == std::string::npos) {
    return 0;
  }
  const auto begin = json.find('[', pos + key.size());
  const auto end = json.find(']', begin == std::string::npos ? pos + key.size() : begin + 1);
  if (begin == std::string::npos || end == std::string::npos || end <= begin + 1) {
    return 0;
  }
  size_t count = 1;
  for (size_t idx = begin + 1; idx < end; ++idx) {
    if (json[idx] == ',') {
      ++count;
    }
  }
  return count;
}

void MonitorReporter::cache_nodemanager_json(const std::string & json_payload, const rclcpp::Time & now)
{
  last_nodemanager_cmd_.valid = true;
  last_nodemanager_cmd_.stamp = now;
  last_nodemanager_cmd_.module_name = extract_json_string_field(json_payload, "module_name");
  last_nodemanager_cmd_.nodes_to_restart_count = extract_json_array_count(json_payload, "nodes_to_restart");
  last_nodemanager_cmd_.raw_json = json_payload;
}

void MonitorReporter::cache_supervisor_json(const std::string & json_payload, const rclcpp::Time & now)
{
  cache_nodemanager_json(json_payload, now);
}

void MonitorReporter::cache_safety_cmd(const msg::SafetyCmd & msg, const rclcpp::Time & now)
{
  last_safety_cmd_.valid = true;
  last_safety_cmd_.stamp = now;
  last_safety_cmd_.action = msg.action;
  last_safety_cmd_.slow_down_percentage = msg.slow_down_percentage;
  last_safety_cmd_.reason = msg.reason;
}

void MonitorReporter::publish_heartbeat(const msg::MonitorStatus & status, const rclcpp::Time & now) const
{
  if (!heartbeat_pub_) {
    return;
  }

  std::ostringstream oss;
  oss << '{'
      << "\"timestamp\":\"" << json_escape(std::to_string(now.seconds())) << "\"," 
      << "\"all_ok\":" << (status.all_ok ? "true" : "false") << ','
      << "\"system\":{"
      << "\"cpu_usage\":" << status.cpu_usage << ','
      << "\"mem_usage\":" << status.mem_usage << ','
      << "\"disk_usage\":" << status.disk_usage << ','
      << "\"cpu_temp\":" << status.cpu_temp << ','
      << "\"gpu_usage\":" << status.gpu_usage << ','
      << "\"gpu_temp\":" << status.gpu_temp << ','
      << "\"gpu_mem_usage\":" << status.gpu_mem_usage << "},"
      << "\"battery\":{"
      << "\"temperature\":" << status.battery_temperature << ','
      << "\"percentage\":" << status.battery_percentage << "},"
      << "\"navigation\":{"
      << "\"status_valid\":" << (status.vehicle_status_valid ? "true" : "false") << ','
      << "\"active\":" << (status.vehicle_navigation_active ? "true" : "false") << ','
      << "\"succeeded\":" << (status.vehicle_navigation_succeeded ? "true" : "false") << ','
      << "\"progress_percentage\":" << status.vehicle_progress_percentage << ','
      << "\"simple_status\":\"" << json_escape(status.vehicle_simple_status) << "\"," 
      << "\"error_message\":\"" << json_escape(status.vehicle_error_message) << "\"},"
      << "\"summary\":{"
      << "\"active_nodes\":" << status.active_nodes.size() << ','
      << "\"timeout_nodes\":" << status.timeout_nodes.size() << ','
      << "\"active_topics\":" << status.active_topics.size() << ','
      << "\"inactive_topics\":" << status.inactive_topics.size() << "}}";

  std_msgs::msg::String out;
  out.data = oss.str();
  heartbeat_pub_->publish(out);
}

void MonitorReporter::publish_fault_event_json(const msg::FaultEvent & event, const rclcpp::Time & now) const
{
  if (!event_pub_) {
    return;
  }

  bool nodemanager_match = false;
  bool safety_match = false;
  if (last_nodemanager_cmd_.valid) {
    nodemanager_match =
      event.action == msg::FaultEvent::NODEMANAGER &&
      event.module_name == last_nodemanager_cmd_.module_name &&
      (now - last_nodemanager_cmd_.stamp).seconds() <= cmd_correlation_window_s_;
  }
  if (last_safety_cmd_.valid) {
    safety_match =
      event.action == msg::FaultEvent::SAFETY_SYSTEM &&
      (now - last_safety_cmd_.stamp).seconds() <= cmd_correlation_window_s_;
  }

  const std::string fault_type = classify_fault_type(event);

  std::ostringstream oss;
  oss << '{'
      << "\"timestamp\":\"" << json_escape(to_stamp_string(event.stamp)) << "\"," 
      << "\"edge\":\"" << edge_to_string(event.edge) << "\"," 
      << "\"fault_type\":\"" << json_escape(fault_type) << "\"," 
      << "\"fault_module\":\"" << json_escape(event.module_name) << "\"," 
      << "\"fault_level\":\"" << fault_level_to_string(event.fault_level) << "\"," 
      << "\"fault_message\":\"" << json_escape(event.reason) << "\"," 
      << "\"measure_execution\":{"
      << "\"action_type\":\"" << action_to_string(event.action) << "\"," 
      << "\"nodemanager\":{"
      << "\"matched\":" << (nodemanager_match ? "true" : "false") << ','
      << "\"module_name\":\""
      << json_escape(nodemanager_match ? last_nodemanager_cmd_.module_name : std::string()) << "\","
      << "\"nodes_to_restart_count\":"
      << (nodemanager_match ? last_nodemanager_cmd_.nodes_to_restart_count : 0) << "},"
      << "\"supervisor\":{"
      << "\"matched\":" << (nodemanager_match ? "true" : "false") << ','
      << "\"module_name\":\""
      << json_escape(nodemanager_match ? last_nodemanager_cmd_.module_name : std::string()) << "\","
      << "\"nodes_to_restart_count\":"
      << (nodemanager_match ? last_nodemanager_cmd_.nodes_to_restart_count : 0) << "},"
      << "\"safety\":{"
      << "\"matched\":" << (safety_match ? "true" : "false") << ','
      << "\"action\":\""
      << json_escape(safety_match ? safety_action_to_string(last_safety_cmd_.action) : std::string("NONE")) << "\"," 
      << "\"slow_down_percentage\":"
      << (safety_match ? last_safety_cmd_.slow_down_percentage : 0.0F) << ','
      << "\"reason\":\""
      << json_escape(safety_match ? last_safety_cmd_.reason : std::string()) << "\"},"
      << "\"details\":\""
      << json_escape(nodemanager_match || safety_match ? "matched_by_correlation" : "placeholder_only")
      << "\"}}";

  std_msgs::msg::String out;
  out.data = oss.str();
  event_pub_->publish(out);

  if (node_ != nullptr && fault_type == "collision_detection") {
    const auto fault_level = fault_level_to_string(event.fault_level);
    const auto action = action_to_string(event.action);
    if (event.edge == msg::FaultEvent::EDGE_RECOVER) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Collision fault recovered: level=%s action=%s reason=%s",
        fault_level.c_str(), action.c_str(), event.reason.c_str());
    } else if (event.fault_level == msg::FaultEvent::CRITICAL ||
      event.fault_level == msg::FaultEvent::ERROR)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Collision fault triggered: level=%s action=%s reason=%s",
        fault_level.c_str(), action.c_str(), event.reason.c_str());
    } else {
      RCLCPP_WARN(
        node_->get_logger(),
        "Collision fault triggered: level=%s action=%s reason=%s",
        fault_level.c_str(), action.c_str(), event.reason.c_str());
    }
  }
}

void MonitorReporter::publish_human_takeover(
  const EventHumanTakeoverDecision & decision,
  const std::string & plan_id,
  const rclcpp::Time & now) const
{
  if (!human_takeover_pub_) {
    return;
  }

  std::ostringstream oss;
  oss << '{'
      << "\"timestamp\":\"" << json_escape(std::to_string(now.seconds())) << "\","
      << "\"event_type\":\"human_takeover_required\","
      << "\"request\":\"human_intervention\","
      << "\"plan_id\":\"" << json_escape(plan_id) << "\","
      << "\"rule_id\":\"" << json_escape(decision.rule_id) << "\","
      << "\"module_name\":\"" << json_escape(decision.module_name) << "\","
      << "\"fault_key\":\"" << json_escape(decision.fault_key) << "\","
      << "\"fault_level\":\""
      << fault_level_to_string(static_cast<uint8_t>(decision.level)) << "\","
      << "\"reason\":\"" << json_escape(decision.reason) << "\","
      << "\"suggested_action\":\"manual_check\""
      << '}';

  std_msgs::msg::String out;
  out.data = oss.str();
  human_takeover_pub_->publish(out);
}

}  // namespace nav2_monitor
