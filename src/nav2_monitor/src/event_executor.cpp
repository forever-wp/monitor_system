#include "nav2_monitor/event_executor.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace nav2_monitor
{

namespace
{
std::string safety_cmd_to_string(uint8_t action)
{
  switch (action) {
    case msg::SafetyCmd::SLOW_DOWN:
      return "SLOW_DOWN";
    case msg::SafetyCmd::SOFT_STOP:
      return "SOFT_STOP";
    case msg::SafetyCmd::EMERGENCY_STOP:
      return "EMERGENCY_STOP";
    case msg::SafetyCmd::RESUME:
      return "RESUME";
    default:
      return "NONE";
  }
}
}  // namespace

void EventExecutor::configure(
  rclcpp::Node * node,
  const std::string & safety_cmd_topic,
  const std::string & nodemanager_cmd_topic,
  double safety_cmd_republish_period_s,
  double nodemanager_cooldown_s)
{
  node_ = node;
  update_timing(safety_cmd_republish_period_s, nodemanager_cooldown_s);
  if (node_ == nullptr) {
    return;
  }

  safety_pub_ = node_->create_publisher<msg::SafetyCmd>(
    safety_cmd_topic, rclcpp::QoS(1).reliable().transient_local());
  nodemanager_pub_ = node_->create_publisher<std_msgs::msg::String>(
    nodemanager_cmd_topic, rclcpp::QoS(10).reliable().durability_volatile());
}

void EventExecutor::update_timing(
  double safety_cmd_republish_period_s,
  double nodemanager_cooldown_s)
{
  safety_cmd_republish_period_s_ = std::max(0.05, safety_cmd_republish_period_s);
  nodemanager_cooldown_s_ = std::max(0.0, nodemanager_cooldown_s);
}

msg::SafetyCmd EventExecutor::to_safety_cmd(const SafetyCommandUpdate & update)
{
  msg::SafetyCmd out;
  if (!update.active) {
    out.action = msg::SafetyCmd::RESUME;
    out.slow_down_percentage = 0.0F;
    out.reason = update.reason;
    return out;
  }

  switch (update.command) {
    case SafetyCommandType::SLOW_DOWN:
      out.action = msg::SafetyCmd::SLOW_DOWN;
      break;
    case SafetyCommandType::SOFT_STOP:
      out.action = msg::SafetyCmd::SOFT_STOP;
      break;
    case SafetyCommandType::EMERGENCY_STOP:
      out.action = msg::SafetyCmd::EMERGENCY_STOP;
      break;
    case SafetyCommandType::NONE:
    default:
      out.action = msg::SafetyCmd::RESUME;
      break;
  }
  out.slow_down_percentage = static_cast<float>(update.slow_down_percentage);
  out.reason = update.reason;
  return out;
}

std::string EventExecutor::json_escape(const std::string & input)
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

std::string EventExecutor::build_nodemanager_payload(
  const EventNodeManagerDecision & decision)
{
  std::ostringstream oss;
  oss << '{'
      << "\"module_name\":\"" << json_escape(decision.module_name) << "\","
      << "\"fault_keys\":\"" << json_escape(nodemanager_signature(decision)) << "\","
      << "\"rule_ids\":\"" << json_escape([&decision]() {
        std::ostringstream joined;
        for (size_t i = 0; i < decision.rule_ids.size(); ++i) {
          if (i > 0) {
            joined << ';';
          }
          joined << decision.rule_ids[i];
        }
        return joined.str();
      }()) << "\","
      << "\"modules_to_restart\":[\"" << json_escape(decision.module_name) << "\"],"
      << "\"nodes_to_restart\":[\"" << json_escape(decision.module_name) << "\"],"
      << "\"reason\":\"" << json_escape(decision.reason) << "\"}";
  return oss.str();
}

std::string EventExecutor::nodemanager_signature(
  const EventNodeManagerDecision & decision)
{
  std::vector<std::string> keys = decision.fault_keys;
  std::sort(keys.begin(), keys.end());
  std::ostringstream oss;
  for (size_t i = 0; i < keys.size(); ++i) {
    if (i > 0) {
      oss << ';';
    }
    oss << keys[i];
  }
  return oss.str();
}

bool EventExecutor::should_publish_safety(
  const SafetyCommandUpdate & update,
  const rclcpp::Time & now) const
{
  if (!safety_state_known_) {
    return update.active;
  }

  const bool same_state =
    last_safety_update_.active == update.active &&
    last_safety_update_.command == update.command &&
    std::fabs(last_safety_update_.slow_down_percentage - update.slow_down_percentage) < 1e-6;
  if (!same_state) {
    return true;
  }
  if (update.active &&
    (now - last_safety_publish_time_).seconds() >= safety_cmd_republish_period_s_)
  {
    return true;
  }
  return false;
}

bool EventExecutor::should_publish_nodemanager(
  const EventNodeManagerDecision & decision,
  const std::string & signature,
  const rclcpp::Time & now) const
{
  const auto it = nodemanager_publish_state_.find(decision.module_name);
  if (it == nodemanager_publish_state_.end()) {
    return true;
  }
  if (it->second.last_signature != signature) {
    return true;
  }
  return (now - it->second.last_publish_time).seconds() >= nodemanager_cooldown_s_;
}

EventExecutionResult EventExecutor::execute(
  const EventExecutionPlan & plan,
  const rclcpp::Time & now)
{
  EventExecutionResult result;
  result.plan_id = plan.plan_id;
  result.plan_signature = plan.signature;

  if (plan.safety_update.has_value() && should_publish_safety(*plan.safety_update, now)) {
    const auto msg = to_safety_cmd(*plan.safety_update);
    if (safety_pub_) {
      safety_pub_->publish(msg);
    }
    last_safety_update_ = *plan.safety_update;
    last_safety_publish_time_ = now;
    safety_state_known_ = true;
    result.safety_cmd = msg;
    result.target_results.push_back(EventExecutionResult::TargetResult{
      "safety", true, safety_pub_ != nullptr, plan.safety_update->active,
      safety_cmd_to_string(msg.action), msg.reason});
  } else if (plan.safety_update.has_value()) {
    result.target_results.push_back(EventExecutionResult::TargetResult{
      "safety", plan.safety_update->active, false, plan.safety_update->active,
      plan.safety_update->active ? "LATCHED" : "RESUME", plan.safety_update->reason});
  }

  for (const auto & decision : plan.nodemanager_decisions) {
    const auto signature = nodemanager_signature(decision);
    if (!should_publish_nodemanager(decision, signature, now)) {
      result.target_results.push_back(EventExecutionResult::TargetResult{
        "nodemanager", true, false, false, decision.module_name, decision.reason});
      continue;
    }

    const auto payload = build_nodemanager_payload(decision);
    std_msgs::msg::String msg;
    msg.data = payload;
    if (nodemanager_pub_) {
      nodemanager_pub_->publish(msg);
    }
    nodemanager_publish_state_[decision.module_name] =
      NodeManagerPublishState{now, signature};
    result.nodemanager_json_payloads.push_back(payload);
    result.nodemanager_decisions.push_back(decision);
    result.target_results.push_back(EventExecutionResult::TargetResult{
      "nodemanager", true, nodemanager_pub_ != nullptr, false,
      decision.module_name, decision.reason});
  }

  return result;
}

}  // namespace nav2_monitor
